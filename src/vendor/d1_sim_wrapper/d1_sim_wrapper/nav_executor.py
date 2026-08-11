"""Navigation state machine for /start_navigation -> Nav2 -> /navigation_state.

Holds the vendor NavigationState enum, the one-goal-at-a-time lifecycle, the
pause/continue/stop intent tracking, the goal generation-id guard, and terminal
state retention. All rclpy/action wiring lives in wrapper_node; this module is
pure Python with injectable time so it is unit-testable.
"""
from enum import IntEnum
from typing import Any, Optional, Tuple


class NavState(IntEnum):
    STANDBY = 0
    INITIALIZING = 1
    ACTIVE = 2
    PAUSE = 3
    CANCELLED = 4
    SUCCEEDED = 5
    FAILED = 6


# action_msgs/msg/GoalStatus terminal codes (Humble)
GOAL_STATUS_SUCCEEDED = 4
GOAL_STATUS_CANCELED = 5
GOAL_STATUS_ABORTED = 6

TERMINAL_STATES = (NavState.CANCELLED, NavState.SUCCEEDED, NavState.FAILED)


class Feedback:
    __slots__ = ('distance_remaining', 'eta_sec', 'nav_time_sec')

    def __init__(self) -> None:
        self.distance_remaining = 0.0
        self.eta_sec = 0.0
        self.nav_time_sec = 0.0


class NavExecutor:
    def __init__(self, terminal_hold_sec: float = 5.0) -> None:
        self.terminal_hold_sec = terminal_hold_sec
        self.state = NavState.STANDBY
        self.function_id = 0
        self.goal_pose: Optional[Any] = None  # geometry_msgs/Pose of current POI
        self.generation = 0                   # increments per send; guards stale callbacks
        self.feedback = Feedback()
        self.last_terminal: Optional[NavState] = None
        self._terminal_at: Optional[float] = None
        self._cancel_intent: Optional[NavState] = None  # PAUSE or CANCELLED

    # -------- queries --------

    @property
    def goal_active(self) -> bool:
        return self.state in (NavState.INITIALIZING, NavState.ACTIVE)

    @property
    def busy(self) -> bool:
        return self.state in (NavState.INITIALIZING, NavState.ACTIVE, NavState.PAUSE)

    def is_current(self, generation: int) -> bool:
        return generation == self.generation

    # -------- commands from /start_navigation --------

    def begin_start(self, goal_pose: Any, function_id: int) -> int:
        """Caller has validated preconditions. Returns the new generation id."""
        self.generation += 1
        self.goal_pose = goal_pose
        self.function_id = function_id
        self.feedback = Feedback()
        self.state = NavState.INITIALIZING
        self.last_terminal = None
        self._terminal_at = None
        self._cancel_intent = None
        return self.generation

    def begin_continue(self) -> Optional[int]:
        """Resend the retained goal after a pause. None if not paused."""
        if self.state != NavState.PAUSE or self.goal_pose is None:
            return None
        self.generation += 1
        self.state = NavState.INITIALIZING
        self._cancel_intent = None
        return self.generation

    def request_pause(self) -> bool:
        if not self.goal_active:
            return False
        self._cancel_intent = NavState.PAUSE
        return True

    def request_stop(self, now: float) -> bool:
        if not self.busy:
            return False
        if self.state == NavState.PAUSE:
            # nothing running in Nav2; terminal immediately
            self._enter_terminal(NavState.CANCELLED, now)
            return False  # no Nav2 cancel needed
        self._cancel_intent = NavState.CANCELLED
        return True

    def fail_rejected_start(self, now: float, function_id: int, goal_pose: Any) -> None:
        """Contract §7 precondition violation: observable FAILED, no motion."""
        self.function_id = function_id
        self.goal_pose = goal_pose
        self._enter_terminal(NavState.FAILED, now)

    # -------- events from the Nav2 action client --------

    def on_goal_accepted(self, generation: int) -> None:
        if self.is_current(generation) and self.state == NavState.INITIALIZING:
            self.state = NavState.ACTIVE

    def on_goal_rejected(self, generation: int, now: float) -> None:
        if self.is_current(generation):
            self._enter_terminal(NavState.FAILED, now)

    def on_feedback(self, generation: int, distance_remaining: float,
                    eta_sec: float, nav_time_sec: float) -> None:
        if not self.is_current(generation) or self.state not in (
                NavState.INITIALIZING, NavState.ACTIVE):
            return  # stale goal or late feedback after terminal
        self.state = NavState.ACTIVE
        self.feedback.distance_remaining = distance_remaining
        self.feedback.eta_sec = eta_sec
        self.feedback.nav_time_sec = nav_time_sec

    def on_result(self, generation: int, goal_status: int, now: float) -> None:
        if not self.is_current(generation):
            return
        if self.state in TERMINAL_STATES:
            return  # e.g. localization-loss already FAILED; late cancel result
        if goal_status == GOAL_STATUS_SUCCEEDED:
            self._enter_terminal(NavState.SUCCEEDED, now)
        elif goal_status == GOAL_STATUS_CANCELED:
            if self._cancel_intent == NavState.PAUSE:
                self.state = NavState.PAUSE
                self._cancel_intent = None
            else:
                self._enter_terminal(NavState.CANCELLED, now)
        else:  # ABORTED or anything unexpected
            self._enter_terminal(NavState.FAILED, now)

    def on_localization_lost(self, now: float) -> bool:
        """Contract §4/§8: localization 4/6 while navigating fails the goal.
        Returns True if the caller must cancel the in-flight Nav2 goal."""
        if not self.goal_active:
            return False
        self._enter_terminal(NavState.FAILED, now)
        return True

    # -------- periodic --------

    def tick(self, now: float) -> NavState:
        """Advance terminal retention; returns the state to publish."""
        if (self.state in TERMINAL_STATES and self._terminal_at is not None
                and now - self._terminal_at >= self.terminal_hold_sec):
            self.state = NavState.STANDBY
            self._terminal_at = None
        return self.state

    def _enter_terminal(self, state: NavState, now: Optional[float]) -> None:
        self.state = state
        self.last_terminal = state
        self._terminal_at = now
        self._cancel_intent = None

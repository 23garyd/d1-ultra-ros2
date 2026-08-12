from d1_sim_wrapper.nav_executor import (GOAL_STATUS_ABORTED,
                                         GOAL_STATUS_CANCELED,
                                         GOAL_STATUS_SUCCEEDED, NavExecutor,
                                         NavState)

GOAL = object()  # opaque pose


def make_executor():
    return NavExecutor(terminal_hold_sec=5.0)


def run_to_active(e, t=0.0):
    gen = e.begin_start(GOAL, function_id=4)
    e.on_goal_accepted(gen)
    e.on_feedback(gen, distance_remaining=3.0, eta_sec=10.0, nav_time_sec=1.0)
    assert e.state == NavState.ACTIVE
    return gen


def test_success_flow_and_terminal_hold():
    e = make_executor()
    gen = run_to_active(e)
    e.on_result(gen, GOAL_STATUS_SUCCEEDED, now=100.0)
    assert e.state == NavState.SUCCEEDED
    assert e.tick(102.0) == NavState.SUCCEEDED   # retained during hold
    assert e.tick(105.5) == NavState.STANDBY     # released after hold
    assert e.last_terminal == NavState.SUCCEEDED # latched for the backend


def test_abort_maps_to_failed():
    e = make_executor()
    gen = run_to_active(e)
    e.on_result(gen, GOAL_STATUS_ABORTED, now=10.0)
    assert e.state == NavState.FAILED


def test_pause_continue_flow():
    e = make_executor()
    gen1 = run_to_active(e)
    assert e.request_pause()
    # canceled BECAUSE of pause -> PAUSE, not CANCELLED
    e.on_result(gen1, GOAL_STATUS_CANCELED, now=10.0)
    assert e.state == NavState.PAUSE

    gen2 = e.begin_continue()
    assert gen2 == gen1 + 1
    # stale result from the old goal must be ignored
    e.on_result(gen1, GOAL_STATUS_ABORTED, now=11.0)
    assert e.state == NavState.INITIALIZING
    e.on_goal_accepted(gen2)
    assert e.state == NavState.ACTIVE


def test_stop_maps_to_cancelled():
    e = make_executor()
    gen = run_to_active(e)
    assert e.request_stop(now=10.0)  # needs a Nav2 cancel
    e.on_result(gen, GOAL_STATUS_CANCELED, now=10.5)
    assert e.state == NavState.CANCELLED


def test_stop_while_paused_is_immediate():
    e = make_executor()
    gen = run_to_active(e)
    e.request_pause()
    e.on_result(gen, GOAL_STATUS_CANCELED, now=10.0)
    assert e.state == NavState.PAUSE
    assert not e.request_stop(now=11.0)  # no Nav2 goal to cancel
    assert e.state == NavState.CANCELLED
    assert e.tick(17.0) == NavState.STANDBY


def test_localization_lost_fails_active_goal():
    e = make_executor()
    run_to_active(e)
    assert e.on_localization_lost(now=10.0)
    assert e.state == NavState.FAILED
    # late CANCELED result from our cancel must not overwrite FAILED
    e.on_result(e.generation, GOAL_STATUS_CANCELED, now=10.5)
    assert e.state == NavState.FAILED


def test_localization_lost_ignored_when_idle():
    e = make_executor()
    assert not e.on_localization_lost(now=1.0)
    assert e.state == NavState.STANDBY


def test_rejected_start_is_observable_failed():
    e = make_executor()
    e.fail_rejected_start(now=1.0, function_id=4, goal_pose=GOAL)
    assert e.state == NavState.FAILED
    assert e.tick(2.0) == NavState.FAILED
    assert e.tick(7.0) == NavState.STANDBY


def test_late_feedback_after_terminal_ignored():
    e = make_executor()
    gen = run_to_active(e)
    e.on_result(gen, GOAL_STATUS_SUCCEEDED, now=10.0)
    e.on_feedback(gen, 1.0, 2.0, 3.0)
    assert e.state == NavState.SUCCEEDED
    assert e.feedback.distance_remaining == 3.0  # unchanged


def test_busy_prevents_double_start_by_caller_contract():
    e = make_executor()
    run_to_active(e)
    assert e.busy and e.goal_active
    e.request_pause()
    e.on_result(e.generation, GOAL_STATUS_CANCELED, now=1.0)
    assert e.busy and not e.goal_active  # paused still occupies the slot


def test_nav2_abort_records_error_reason():
    e = make_executor()
    gen = run_to_active(e)
    e.on_result(gen, GOAL_STATUS_ABORTED, now=10.0)
    assert e.state == NavState.FAILED
    assert e.last_error is not None
    assert e.last_error.code != 0
    assert e.last_error.message
    assert e.last_error.scope == "nav2_result"


def test_rejected_start_records_distinct_reason():
    e = make_executor()
    e.fail_rejected_start(now=1.0, function_id=4, goal_pose=GOAL)
    assert e.state == NavState.FAILED
    assert e.last_error.scope == "precondition"


def test_localization_loss_records_distinct_reason():
    e = make_executor()
    run_to_active(e)
    assert e.on_localization_lost(now=5.0) is True
    assert e.state == NavState.FAILED
    assert e.last_error.scope == "localization"


def test_distinct_causes_get_distinct_codes():
    """primary.code must carry information, not be a constant."""
    a = make_executor()
    gen = run_to_active(a)
    a.on_result(gen, GOAL_STATUS_ABORTED, now=10.0)

    b = make_executor()
    b.fail_rejected_start(now=1.0, function_id=4, goal_pose=GOAL)

    assert a.last_error.code != b.last_error.code


def test_success_carries_no_error():
    e = make_executor()
    gen = run_to_active(e)
    e.on_result(gen, GOAL_STATUS_SUCCEEDED, now=10.0)
    assert e.state == NavState.SUCCEEDED
    assert e.last_error is None


def test_later_success_clears_a_stale_error():
    """A FAILED goal must not leave a reason attached to the next success."""
    e = make_executor()
    gen = run_to_active(e)
    e.on_result(gen, GOAL_STATUS_ABORTED, now=10.0)
    assert e.last_error is not None
    e.tick(20.0)  # release the terminal hold
    gen2 = run_to_active(e, t=21.0)
    e.on_result(gen2, GOAL_STATUS_SUCCEEDED, now=25.0)
    assert e.last_error is None

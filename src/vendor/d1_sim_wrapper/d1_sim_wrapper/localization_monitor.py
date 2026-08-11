"""Localization readiness synthesis for /localization_state and /localization_info.

The sim localizes continuously via cartographer (map->odom TF). This monitor
maps "is a saved map loaded" + "how fresh is TF map->base_link" onto the
vendor's LocalizationState.status enum. Pure Python, injectable time — no rclpy.
"""
from enum import IntEnum
from typing import Optional


class LocStatus(IntEnum):
    INIT = 0
    MAP_LOADING = 1
    INIT_LOCALIZATION = 2
    NORMAL = 3
    ERROR = 4
    DYNAMIC_INIT = 5
    LOST = 6


# description strings mirror the captured vendor payloads.
DESCRIPTIONS = {
    LocStatus.INIT: 'init',
    LocStatus.MAP_LOADING: 'map loading',
    LocStatus.INIT_LOCALIZATION: 'initial localization',
    LocStatus.NORMAL: 'continuous loc: CONTINUOUS_NORMAL',
    LocStatus.ERROR: 'error: continuous localization lost',
    LocStatus.LOST: 'loc lost: CONTINUOUS_LOST',
}

# /localization_info status enum (contract §5) — coarser than LocStatus.
INFO_INITIALIZING = 0
INFO_NORMAL = 3
INFO_LOST = 4


class LocalizationMonitor:
    def __init__(self, tf_stale_sec: float = 2.0, loc_error_sec: float = 10.0,
                 load_settle_sec: float = 0.5) -> None:
        self.tf_stale_sec = tf_stale_sec
        self.loc_error_sec = loc_error_sec
        self.load_settle_sec = load_settle_sec
        self.map_loaded = False
        self._load_time: Optional[float] = None
        self._was_normal = False

    def notify_map_loaded(self, now: float) -> None:
        self.map_loaded = True
        self._load_time = now
        self._was_normal = False

    def evaluate(self, now: float, tf_age: Optional[float]) -> LocStatus:
        """tf_age: seconds since the latest map->base_link transform, or None
        if the lookup failed entirely."""
        if not self.map_loaded:
            return LocStatus.INIT
        assert self._load_time is not None
        since_load = now - self._load_time
        if since_load < self.load_settle_sec:
            return LocStatus.MAP_LOADING

        fresh = tf_age is not None and tf_age <= self.tf_stale_sec
        if fresh:
            self._was_normal = True
            return LocStatus.NORMAL

        if not self._was_normal:
            # never localized since load — still acquiring
            return LocStatus.INIT_LOCALIZATION

        stale_by = (tf_age if tf_age is not None else float('inf'))
        if stale_by > self.loc_error_sec:
            return LocStatus.ERROR
        return LocStatus.LOST

    @staticmethod
    def rate_for(status: LocStatus) -> float:
        return 1.0 if status == LocStatus.NORMAL else 0.0

    @staticmethod
    def info_status_for(status: LocStatus) -> int:
        if status == LocStatus.NORMAL:
            return INFO_NORMAL
        if status in (LocStatus.LOST, LocStatus.ERROR):
            return INFO_LOST
        return INFO_INITIALIZING

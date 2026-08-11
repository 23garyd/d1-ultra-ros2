"""Mapping-session state machine for /pub_slam_state and /slam_state_service.

State values follow the contract's observed SlamState.data.data transitions:
READY(2) -> MAPPING(3) -> SAVING(5) -> SAVED(6). Pure Python, no rclpy.
"""
from enum import IntEnum
from typing import Tuple


class SlamStateCode(IntEnum):
    READY = 2
    MAPPING = 3
    SAVING = 5
    SAVED = 6


# error_msg strings mirror the captured vendor payloads.
STATE_DESCRIPTIONS = {
    SlamStateCode.READY: 'Mapping state is normal.',
    SlamStateCode.MAPPING: 'Mapping state is normal.',
    SlamStateCode.SAVING: 'Mapping stopped; saving map.',
    SlamStateCode.SAVED: 'MappingSaveEnd',
}


class SlamSession:
    def __init__(self) -> None:
        self.state = SlamStateCode.READY
        self.map_path_prefix = ''
        self.error_code = 0
        self.error_msg = STATE_DESCRIPTIONS[SlamStateCode.READY]

    @property
    def mapping_active(self) -> bool:
        return self.state in (SlamStateCode.MAPPING, SlamStateCode.SAVING)

    def start(self, navigation_active: bool) -> Tuple[bool, str]:
        if navigation_active:
            return False, 'rejected: navigation goal is active'
        if self.state == SlamStateCode.MAPPING:
            return False, 'rejected: mapping already running'
        if self.state == SlamStateCode.SAVING:
            return False, 'rejected: map save in progress'
        self._set(SlamStateCode.MAPPING)
        self.map_path_prefix = ''
        return True, 'mapping started'

    def begin_save(self) -> Tuple[bool, str]:
        if self.state != SlamStateCode.MAPPING:
            return False, f'rejected: not mapping (state {int(self.state)})'
        self._set(SlamStateCode.SAVING)
        return True, 'saving map'

    def mark_saved(self, map_dir: str) -> None:
        self.map_path_prefix = map_dir
        self._set(SlamStateCode.SAVED)

    def mark_save_failed(self, reason: str) -> None:
        self.state = SlamStateCode.READY
        self.error_code = 1
        self.error_msg = f'map save failed: {reason}'

    def cancel(self) -> Tuple[bool, str]:
        if not self.mapping_active:
            return False, f'rejected: nothing to cancel (state {int(self.state)})'
        self._set(SlamStateCode.READY)
        self.map_path_prefix = ''
        return True, 'mapping cancelled'

    def _set(self, state: SlamStateCode) -> None:
        self.state = state
        self.error_code = 0
        self.error_msg = STATE_DESCRIPTIONS[state]

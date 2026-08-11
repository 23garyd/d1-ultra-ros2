from d1_sim_wrapper.slam_session import SlamSession, SlamStateCode


def test_normal_lifecycle():
    s = SlamSession()
    assert s.state == SlamStateCode.READY

    ok, _ = s.start(navigation_active=False)
    assert ok and s.state == SlamStateCode.MAPPING and s.mapping_active

    ok, _ = s.begin_save()
    assert ok and s.state == SlamStateCode.SAVING

    s.mark_saved('/maps/2026_08_11_00_00_00')
    assert s.state == SlamStateCode.SAVED
    assert s.map_path_prefix == '/maps/2026_08_11_00_00_00'
    assert s.error_code == 0
    assert not s.mapping_active


def test_start_rejected_while_navigating():
    s = SlamSession()
    ok, msg = s.start(navigation_active=True)
    assert not ok and 'navigation' in msg
    assert s.state == SlamStateCode.READY


def test_double_start_rejected():
    s = SlamSession()
    s.start(navigation_active=False)
    ok, _ = s.start(navigation_active=False)
    assert not ok


def test_save_requires_mapping():
    s = SlamSession()
    ok, _ = s.begin_save()
    assert not ok and s.state == SlamStateCode.READY


def test_save_failure_recovers_to_ready():
    s = SlamSession()
    s.start(navigation_active=False)
    s.begin_save()
    s.mark_save_failed('disk full')
    assert s.state == SlamStateCode.READY
    assert s.error_code == 1 and 'disk full' in s.error_msg
    # can start a new session after failure
    ok, _ = s.start(navigation_active=False)
    assert ok


def test_cancel():
    s = SlamSession()
    ok, _ = s.cancel()
    assert not ok
    s.start(navigation_active=False)
    ok, _ = s.cancel()
    assert ok and s.state == SlamStateCode.READY


def test_restart_after_saved_clears_prefix():
    s = SlamSession()
    s.start(navigation_active=False)
    s.begin_save()
    s.mark_saved('/maps/x')
    ok, _ = s.start(navigation_active=False)
    assert ok and s.map_path_prefix == ''

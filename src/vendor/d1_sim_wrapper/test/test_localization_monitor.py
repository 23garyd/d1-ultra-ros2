from d1_sim_wrapper.localization_monitor import LocalizationMonitor, LocStatus


def make_monitor():
    return LocalizationMonitor(tf_stale_sec=2.0, loc_error_sec=10.0,
                               load_settle_sec=0.5)


def test_init_until_map_loaded():
    m = make_monitor()
    assert m.evaluate(0.0, tf_age=0.1) == LocStatus.INIT
    assert m.evaluate(100.0, tf_age=None) == LocStatus.INIT


def test_load_settle_then_acquire_then_normal():
    m = make_monitor()
    m.notify_map_loaded(10.0)
    assert m.evaluate(10.2, tf_age=None) == LocStatus.MAP_LOADING
    assert m.evaluate(10.6, tf_age=None) == LocStatus.INIT_LOCALIZATION
    assert m.evaluate(11.0, tf_age=0.1) == LocStatus.NORMAL
    assert m.rate_for(LocStatus.NORMAL) == 1.0


def test_lost_then_error_when_tf_stales():
    m = make_monitor()
    m.notify_map_loaded(0.0)
    assert m.evaluate(1.0, tf_age=0.1) == LocStatus.NORMAL
    assert m.evaluate(5.0, tf_age=4.0) == LocStatus.LOST      # > tf_stale_sec
    assert m.evaluate(12.0, tf_age=11.0) == LocStatus.ERROR   # > loc_error_sec
    assert m.rate_for(LocStatus.LOST) == 0.0


def test_recovers_to_normal_after_lost():
    m = make_monitor()
    m.notify_map_loaded(0.0)
    m.evaluate(1.0, tf_age=0.1)
    assert m.evaluate(5.0, tf_age=4.0) == LocStatus.LOST
    assert m.evaluate(6.0, tf_age=0.1) == LocStatus.NORMAL


def test_reload_resets_acquisition():
    m = make_monitor()
    m.notify_map_loaded(0.0)
    m.evaluate(1.0, tf_age=0.1)  # NORMAL once
    m.notify_map_loaded(20.0)
    assert m.evaluate(21.0, tf_age=None) == LocStatus.INIT_LOCALIZATION


def test_info_status_mapping():
    m = make_monitor()
    assert m.info_status_for(LocStatus.NORMAL) == 3
    assert m.info_status_for(LocStatus.LOST) == 4
    assert m.info_status_for(LocStatus.ERROR) == 4
    assert m.info_status_for(LocStatus.INIT) == 0
    assert m.info_status_for(LocStatus.MAP_LOADING) == 0

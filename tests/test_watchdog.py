import time

from moveit_arm_node._watchdog import HeartbeatWatchdog


def test_watchdog_fires_on_timeout():
    fired: list[float] = []
    wd = HeartbeatWatchdog(timeout_s=0.05, on_timeout=lambda t: fired.append(t))
    wd.arm()
    time.sleep(0.1)
    wd.tick()
    assert len(fired) == 1


def test_watchdog_does_not_fire_when_heartbeats_arrive():
    fired: list[float] = []
    wd = HeartbeatWatchdog(timeout_s=0.05, on_timeout=lambda t: fired.append(t))
    wd.arm()
    for _ in range(5):
        time.sleep(0.01)
        wd.heartbeat()
        wd.tick()
    assert fired == []


def test_watchdog_fires_only_once():
    fired: list[float] = []
    wd = HeartbeatWatchdog(timeout_s=0.05, on_timeout=lambda t: fired.append(t))
    wd.arm()
    time.sleep(0.1)
    wd.tick()
    time.sleep(0.1)
    wd.tick()
    assert len(fired) == 1


def test_watchdog_disabled_when_timeout_zero():
    fired: list[float] = []
    wd = HeartbeatWatchdog(timeout_s=0.0, on_timeout=lambda t: fired.append(t))
    wd.arm()
    time.sleep(0.05)
    wd.tick()
    assert fired == []

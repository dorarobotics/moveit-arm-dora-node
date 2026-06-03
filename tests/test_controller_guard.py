from moveit_arm_node.controller_guard import ControllerGuard, ControllerBusy


def test_first_caller_gets_slot():
    g = ControllerGuard()
    g.acquire("caller-a")
    assert g.holder == "caller-a"


def test_second_caller_blocked():
    g = ControllerGuard()
    g.acquire("caller-a")
    try:
        g.acquire("caller-b")
    except ControllerBusy as e:
        assert "caller-a" in str(e)
    else:
        raise AssertionError("expected ControllerBusy")


def test_release_frees_slot():
    g = ControllerGuard()
    g.acquire("caller-a")
    g.release("caller-a")
    g.acquire("caller-b")
    assert g.holder == "caller-b"


def test_release_by_non_holder_is_noop():
    g = ControllerGuard()
    g.acquire("caller-a")
    g.release("caller-z")  # not the holder
    assert g.holder == "caller-a"


def test_same_caller_can_reacquire_idempotently():
    g = ControllerGuard()
    g.acquire("caller-a")
    g.acquire("caller-a")  # no error
    assert g.holder == "caller-a"

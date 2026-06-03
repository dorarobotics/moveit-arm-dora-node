from moveit_arm_node.gripper.robotiq_2f85 import Robotiq2F85, RobotiqTransport


class FakeTransport:
    def __init__(self) -> None:
        self.commands: list[tuple[str, float | None]] = []

    def send(self, op: str, value: float | None = None) -> None:
        self.commands.append((op, value))


def test_robotiq_set_sends_position():
    t = FakeTransport()
    g = Robotiq2F85(transport=t)
    g.set(0.040)
    assert t.commands == [("set_position", 0.040)]


def test_robotiq_open_close_send_named_ops():
    t = FakeTransport()
    g = Robotiq2F85(transport=t)
    g.open()
    g.close()
    assert t.commands == [("open", None), ("close", None)]


def test_robotiq_width_returns_last_commanded():
    t = FakeTransport()
    g = Robotiq2F85(transport=t)
    g.set(0.060)
    assert g.width() == 0.060

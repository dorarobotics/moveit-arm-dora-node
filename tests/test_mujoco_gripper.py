from moveit_arm_node.gripper.mujoco import MujocoGripper


def test_set_emits_width():
    sent = []
    g = MujocoGripper(emit=lambda payload: sent.append(payload), open_width=0.085)
    g.set(0.04)
    assert sent == [{"width": 0.04}]
    assert g.width() == 0.04


def test_open_and_close_emit_bounds():
    sent = []
    g = MujocoGripper(emit=lambda payload: sent.append(payload), open_width=0.085)
    g.open()
    g.close()
    assert sent[0] == {"width": 0.085}
    assert sent[1] == {"width": 0.0}
    assert g.width() == 0.0

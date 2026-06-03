import math

from moveit_arm_node._geometry import pose_to_rpy


def test_identity_quaternion_is_zero_rpy():
    out = pose_to_rpy({"position": [0.4, 0.0, 0.3], "orientation": [0.0, 0.0, 0.0, 1.0]})
    assert out[:3] == [0.4, 0.0, 0.3]
    assert all(abs(a) < 1e-9 for a in out[3:])


def test_90deg_yaw_quaternion():
    s = math.sin(math.pi / 4)
    out = pose_to_rpy({"position": [0, 0, 0], "orientation": [0.0, 0.0, s, s]})
    roll, pitch, yaw = out[3], out[4], out[5]
    assert abs(roll) < 1e-6
    assert abs(pitch) < 1e-6
    assert abs(yaw - math.pi / 2) < 1e-6


def test_returns_six_element_list():
    out = pose_to_rpy({"position": [1, 2, 3], "orientation": [0, 0, 0, 1]})
    assert len(out) == 6
    assert out == [1, 2, 3, 0.0, 0.0, 0.0]

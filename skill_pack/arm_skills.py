"""Skill-level robot actions for the octos agent — on-demand MuJoCo IK + bridge HTTP.

The octos LLM agent reasons over these 4 high-level skills; each hides the
unreliable Cartesian move_to_pose path by solving joint configs on demand against
the MuJoCo `pinch` site and driving the verified move_to_joint_state verb, with the
proven reliability tricks (settle-wait, full-orientation grasp hold, staged lift,
grip dwell, radial carry).

  get_ball_position()      -> "x=.., y=.."   (live, after the ball settles)
  get_plate_position()     -> "x=.., y=.."   (the green target site)
  pick_at(x, y)            -> picks the object at (x,y): approach, grasp, lift
  place_at(x, y)           -> carries the held object to (x,y) and releases

Needs MODEL_NAME (ur5e.xml) for IK; talks to the bridge (ARM_BRIDGE_URL) and the
ball_state side-server (BALL_URL).
"""
from __future__ import annotations

import json
import math
import os
import time
import urllib.request

import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import manifest as cfg  # noqa: E402  (skill_pack/manifest.py)

# Deployment paths stay in env (host-specific), NOT the manifest.
BASE = os.environ.get("ARM_BRIDGE_URL", "http://127.0.0.1:8768")
BALL_URL = os.environ.get("BALL_URL", "http://127.0.0.1:8779/ball")

# DOF-generic layout (manifest-driven): a 6-DOF arm (UR5e/reBot) uses the defaults;
# a 5-DOF arm (SO-101) sets num_joints=5. Object-first scene layout means the arm
# joints sit at qpos[arm_qpos_start : +N] and qvel[arm_qvel_start : +N], and the arm
# joints are model joints jnt[jnt_range_start : +N] (the free object is jnt 0).
NUM_JOINTS = int(cfg.f("num_joints", 6))
ARM_QPOS_START = int(cfg.f("arm_qpos_start", 7))
ARM_QVEL_START = int(cfg.f("arm_qvel_start", 6))
JNT_RANGE_START = int(cfg.f("jnt_range_start", 1))
ARM_QPOS = slice(ARM_QPOS_START, ARM_QPOS_START + NUM_JOINTS)
ARM_QVEL = slice(ARM_QVEL_START, ARM_QVEL_START + NUM_JOINTS)
# A <6-DOF arm can't hold a full 6-DOF grasp orientation, so default to down-only
# IK (z-axis down, yaw free = feasible). 6-DOF arms hold the full grasp orientation.
HOLD_FULL_ORIENT = cfg.b("hold_full_orientation", NUM_JOINTS >= 6)

# Robot-specific knobs come from the per-robot MANIFEST (ROBOT_MANIFEST=...json),
# overridable by env. Adding a new arm of this class = write a manifest; this skill
# code never changes.
PLATE_XY = (cfg.f("plate_x", 0.25), cfg.f("plate_y", 0.0))
GRIP_DWELL = cfg.f("grip_dwell", 3.0)
HOME = cfg.vec("arm_home", [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
GRASP_BIAS = cfg.f("grasp_bias", 0.017)  # live arm undershoot; 0 in pure sim
APPROACH_Z = cfg.f("approach_z", 0.22)
GRASP_Z = cfg.f("grasp_z", 0.03) - GRASP_BIAS
PLACE_Z = cfg.f("place_z", 0.04) - GRASP_BIAS
GRIP_OPEN_W = cfg.f("grip_open_w", 0.085)   # width that maps to "open"
GRIP_CLOSE_W = cfg.f("grip_close_w", 0.0)   # width that maps to "closed"
LIFT_ZS = cfg.vec("lift_zs", [0.07, 0.13, APPROACH_Z])  # staged-lift heights

_m = mujoco.MjModel.from_xml_path(os.environ["MODEL_NAME"])
_d = mujoco.MjData(_m)
_site = mujoco.mj_name2id(_m, mujoco.mjtObj.mjOBJ_SITE, "pinch")

# state carried from pick_at -> place_at
_grasp_R = None
_pick_xy = None

MOVE = "vendor.moveit.arm.move_to_joint_state"
NAMED = "vendor.moveit.arm.move_to_named"
GRIP = "vendor.moveit.arm.gripper.set"


# ---------- bridge HTTP ----------
def _call(verb: str, **args):
    body = json.dumps({"args": args}).encode()
    req = urllib.request.Request(f"{BASE}/tools/{verb}", data=body,
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=70) as r:
        return json.loads(r.read().decode())


def _move(q):
    return _call(MOVE, joints=[float(v) for v in q], control_source="octos")


def _ball():
    for _ in range(30):
        try:
            with urllib.request.urlopen(BALL_URL, timeout=2) as r:
                d = json.loads(r.read().decode())
            if d.get("x") is not None:
                return d
        except OSError:
            pass
        time.sleep(0.5)
    return None


def _wait_settled(timeout_s=15.0):
    prev, stable, deadline = None, 0, time.time() + timeout_s
    while time.time() < deadline:
        b = _ball()
        if b and prev is not None:
            if math.hypot(b["x"] - prev["x"], b["y"] - prev["y"]) + abs(b["z"] - prev["z"]) < 0.003 and b["z"] < 0.05:
                stable += 1
                if stable >= 3:
                    return b
            else:
                stable = 0
        prev = b
        time.sleep(0.4)
    return prev


# ---------- on-demand IK (mirrors ik_solve_grasp.solve) ----------
def _solve(xyz, seed, target_R=None, iters=400, pos_w=1.0, rot_w=0.5):
    q = seed.copy()
    target = np.array(xyz)
    jacp = np.zeros((3, _m.nv))
    jacr = np.zeros((3, _m.nv))
    lo = _m.jnt_range[JNT_RANGE_START:JNT_RANGE_START + NUM_JOINTS, 0]
    hi = _m.jnt_range[JNT_RANGE_START:JNT_RANGE_START + NUM_JOINTS, 1]
    for _ in range(iters):
        _d.qpos[ARM_QPOS] = q
        mujoco.mj_forward(_m, _d)
        pos = _d.site_xpos[_site].copy()
        Rc = _d.site_xmat[_site].reshape(3, 3)
        e_pos = (target - pos) * pos_w
        if target_R is not None:
            e_rot = 0.5 * (np.cross(Rc[:, 0], target_R[:, 0]) + np.cross(Rc[:, 1], target_R[:, 1])
                           + np.cross(Rc[:, 2], target_R[:, 2])) * rot_w
        else:
            e_rot = np.cross(Rc[:, 2], np.array([0.0, 0.0, -1.0])) * rot_w
        if np.linalg.norm(e_pos) < 1e-3 and np.linalg.norm(e_rot) < 1e-2:
            break
        mujoco.mj_jacSite(_m, _d, jacp, jacr, _site)
        J = np.vstack([jacp[:, ARM_QVEL], jacr[:, ARM_QVEL]])
        q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), np.concatenate([e_pos, e_rot])), lo, hi)
    return q


# ---------- skills ----------
def _hold():
    """Grasp orientation to hold during carry/grasp: the full down-pointing frame for
    6-DOF arms, or None (down-only, yaw free) for <6-DOF arms that can't hold it."""
    return _grasp_R if HOLD_FULL_ORIENT else None


def get_ball_position() -> str:
    b = _wait_settled()
    if not b:
        return "ERROR: no ball position available"
    return f"x={b['x']:.3f}, y={b['y']:.3f}"


def get_plate_position() -> str:
    return f"x={PLATE_XY[0]:.3f}, y={PLATE_XY[1]:.3f}"


def pick_at(x: float, y: float) -> str:
    global _grasp_R, _pick_xy
    x, y = float(x), float(y)
    # Establish the down-pointing grasp orientation at the APPROACH height (which
    # is reliably reachable from the HOME seed), then hold that orientation down to
    # the grasp. Solving the grasp DIRECTLY from HOME can stick in a local minimum
    # on arms whose grasp pose is far from HOME (reBot does — the DLS solver lands
    # 24cm off, not down-pointing). Seed-chaining approach -> grasp converges.
    above = _solve([x, y, APPROACH_Z], HOME.copy())
    _grasp_R = _d.site_xmat[_site].reshape(3, 3).copy()
    above = _solve([x, y, APPROACH_Z], above, target_R=_hold())
    at = _solve([x, y, GRASP_Z], above, target_R=_hold())
    lifts = [_solve([x, y, z], at, target_R=_hold()) for z in LIFT_ZS]

    _call(NAMED, name="home", control_source="octos")
    _call(GRIP, width=GRIP_OPEN_W)           # open
    _move(above)
    _move(at)
    _call(GRIP, width=GRIP_CLOSE_W)          # close
    time.sleep(GRIP_DWELL)
    for q in lifts:
        _move(q)
    _pick_xy = (x, y)
    b = _ball()
    if b and b["z"] > 0.10:
        return f"OK: grasped and lifted the object from ({x:.3f}, {y:.3f}); now holding it."
    return f"WARNING: lifted but object height is low (z={b['z']:.3f} m); grasp may have missed."


def place_at(x: float, y: float) -> str:
    global _pick_xy
    x, y = float(x), float(y)
    if _grasp_R is None or _pick_xy is None:
        return "ERROR: nothing is being held — call pick_at first."
    bx, by = _pick_xy
    # radial carry arc (dense, orientation-held) from pick xy -> target xy
    seed = _solve([bx, by, APPROACH_Z], HOME.copy(), target_R=_hold())
    n = 3
    for i in range(1, n + 1):
        t = i / (n + 1)
        seed = _solve([bx + t * (x - bx), by + t * (y - by), APPROACH_Z], seed, target_R=_hold())
        _move(seed)
    above = _solve([x, y, APPROACH_Z], seed, target_R=_hold())
    _move(above)
    at = _solve([x, y, PLACE_Z], above, target_R=_hold())
    _move(at)
    _call(GRIP, width=GRIP_OPEN_W)           # release
    _move(above)                             # retract
    _call(NAMED, name="home", control_source="octos")
    _pick_xy = None
    time.sleep(1.5)
    b = _ball()
    if b:
        d = math.hypot(b["x"] - x, b["y"] - y)
        if d < 0.10:
            return f"OK: placed the object at ({x:.3f}, {y:.3f}) — {d*100:.1f} cm from target."
        return f"WARNING: released but object is {d*100:.1f} cm from ({x:.3f}, {y:.3f})."
    return "WARNING: released; could not read final object position."

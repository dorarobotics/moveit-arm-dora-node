# Arm skill pack (octos pick-and-place demo)

The high-level pick-and-place demo for arms driven by this vendor node
(`moveit_arm_node`). It lives **with the vendor node**, not in `octos-dora-bridge`
— the bridge and octos core are generic and never change when you add an arm.

One generic skill codebase drives **any** arm of this class; per-robot differences
live in a small JSON **manifest**. Adding a new arm = a new manifest (+ a
dora-moveit2 config/model + a `skills/<robot>/SKILL.md` in the bridge catalog).

## Layout

```
skill_pack/
  manifest.py            generic manifest loader (env > manifest > default)
  manifests/
    ur5e.json            UR5e grasp geometry
    rebot.json           reBotArm B601-DM grasp geometry
  arm_skills.py          get_ball_position / get_plate_position / pick_at / place_at
                         (on-demand MuJoCo IK + bridge verbs) — manifest-driven
  arm_agent.py           octos LLM agent: sentence -> skills (Ollama qwen3:8b)
  skill_pickplace.py     deterministic skill sequence (no LLM), for tuning
  ik_solve_grasp.py      offline IK precompute (legacy scripted path)
  sim/
    gripper_merge.py     SIM glue: fold gripper width into the MuJoCo control vector
    ball_state.py        SIM glue: serve the free object's pose over HTTP
  dataflows/             reference dataflow wiring (ur5e, rebot)
  scripts/               turnkey launchers (run-rebot-agent.sh, run-rebot-pickplace.sh)
```

## What a manifest holds

Host-agnostic robot description (NOT deployment paths — those stay in env):

```json
{
  "robot_name": "reBotArm B601-DM",
  "object_noun": "red cube",
  "arm_home":   [0.0, -1.0, -1.5, 0.0, 0.0, 0.0],
  "grasp_bias": 0.0, "grasp_z": 0.02, "place_z": 0.03, "approach_z": 0.18,
  "lift_zs":    [0.06, 0.10, 0.18],
  "grip_open_w": 0.085, "grip_close_w": 0.0, "grip_dwell": 3.0,
  "plate_x": 0.25, "plate_y": 0.0
}
```

Select one at runtime: `ROBOT_MANIFEST=manifests/rebot.json`. Any key is still
overridable by the matching upper-case env var (e.g. `GRASP_Z=0.025`).

## Run (epyc)

```bash
bash ~/dorarobotics-test/run-rebot-pickplace.sh      # deterministic (DRIVER=skill)
bash ~/dorarobotics-test/run-rebot-agent.sh "put the red block on the green plate"
```

Deployment paths the launcher injects as env: `MODEL_NAME` (the dora-moveit2
MuJoCo model), `ARM_BRIDGE_URL`, `BALL_URL`, `OCTOS_PY_DIR`.

## Adding another arm

1. dora-moveit2: add `config/<robot>.py` + a MuJoCo model (object freejoint
   declared FIRST so qpos is `object[0:7] / arm[7:13]` — the skill slices are fixed).
2. here: add `manifests/<robot>.json`.
3. octos-dora-bridge: add `skills/<robot>/SKILL.md`.

No edits to `arm_skills.py`, the bridge, the framework, or octos.

#!/usr/bin/env bash
# ============================================================================
#  SO-101 pick-and-place demo — manual, viewer-gated.
#
#  Brings up the MuJoCo viewer + dora dataflow, then WAITS for you to press
#  ENTER before starting the pick (so you never miss it). The arm reaches out
#  to the red cube at its start spot, grasps + lifts it (sim grasp weld), and
#  places it onto the green plate. Re-run the pick as many times as you like;
#  each run first resets the cube to its start position.
#
#  RUN IT (from a terminal INSIDE the epyc remote desktop, so the window shows):
#      bash ~/dorarobotics-test/so101-demo.sh
#
#  Knobs (env):
#      EXEC_INTERP_SPEED=0.5    motion speed (higher = faster; 1.0 ~= 4x)
#      GRIP_DWELL=3.0           pause (s) at the grasp so the close is visible
#      HEADLESS=1               no viewer (for tuning)
#      AUTO=1                   skip the ENTER prompt, pick immediately
#  Tear down: Ctrl-C.
# ============================================================================
set -uo pipefail
export DISPLAY="${DISPLAY:-:0}"
export PATH="$HOME/.cargo/bin:$PATH"

PY=/home/demo/anaconda3/envs/dora-moveit/bin/python
ROOT=/home/demo/dorarobotics-test
SKILL=$ROOT/moveit-arm-dora-node/skill_pack
MANIFEST="${ROBOT_MANIFEST:-$SKILL/manifests/so101.json}"
SRC=$ROOT/so101-mujoco-live.yml
YML=$ROOT/so101-demo.yml
MODEL=/home/demo/Public/github_dora_nav_moveit/dora-moveit2/examples/move_group_demo/models/so101_pickplace.xml
URL=http://127.0.0.1:8768
BALL=http://127.0.0.1:8779/ball
LOG=/tmp/so101-demo.log
HEADLESS="${HEADLESS:-0}"
export EXEC_INTERP_SPEED="${EXEC_INTERP_SPEED:-0.5}"
export GRIP_DWELL="${GRIP_DWELL:-3.0}"
AUTO="${AUTO:-0}"

die() { echo "[so101-demo] ERROR: $*" >&2; exit 1; }

echo "[so101-demo] preflight…"
[ -f "$MODEL" ]    || die "MuJoCo model not found: $MODEL"
[ -f "$MANIFEST" ] || die "manifest not found: $MANIFEST"
[ -f "$SRC" ]      || die "live dataflow not found: $SRC"
command -v dora >/dev/null || die "dora CLI not on PATH ($HOME/.cargo/bin)"

# ---- hard teardown: kill the dora DAEMON too, else it respawns the orphaned
#      bridge on :8768 and the new dataflow's bridge can't bind (Errno 98) ----
hard_teardown() {
  # Kill leftover launcher shells from PRIOR runs — but never ourselves (the
  # pattern 'so101-demo' matches this very script, so exclude our own PID/parent).
  for _pid in $(pgrep -f "run-so101|so101-demo" 2>/dev/null); do
    [ "$_pid" = "$$" ] && continue
    [ "$_pid" = "$PPID" ] && continue
    kill -9 "$_pid" 2>/dev/null
  done
  dora stop --grace 2 >/dev/null 2>&1
  dora destroy   >/dev/null 2>&1
  sleep 2
  # The dora coordinator/daemon processes are literally "dora coordinator"/"dora
  # daemon" (a SPACE, not a hyphen). Matching the hyphenated form kills nothing, so
  # daemons accumulate across runs and multiple daemons each spawn the dataflow ->
  # double-builds, :8768/:8779 port collisions, and a viewer restart-loop. Kill the
  # real names so exactly one daemon ever runs.
  pkill -9 -f "dora daemon" 2>/dev/null
  pkill -9 -f "dora coordinator" 2>/dev/null
  pkill -9 -f "dora_mujoco|moveit_arm_node|ball_state|trajectory_execution|planning_scene|move_group_demo|octos_spec_bridge|skill_pickplace" 2>/dev/null
  for _ in 1 2 3 4 5 6 7 8; do fuser -k 8768/tcp 8779/tcp >/dev/null 2>&1; sleep 1; done
  sleep 1
}
cleanup() { echo; echo "[so101-demo] tearing down…"; hard_teardown; }
trap cleanup EXIT

echo "[so101-demo] clearing any previous run (daemon + ports)…"
hard_teardown
[ -n "$(fuser 8768/tcp 2>/dev/null)" ] && die "port 8768 still held — kill it: fuser -k 8768/tcp"

# ---- build the dataflow: viewer/headless, executor speed + tick, ball_state ----
if [ "$HEADLESS" = "1" ]; then
  cp "$SRC" "$YML"
else
  sed -e 's|MUJOCO_HEADLESS: "1"|MUJOCO_HEADLESS: "0"\n      DISPLAY: ":0"|' "$SRC" > "$YML"
fi
sed -i "/env: {.*ROBOT_CONFIG_MODULE/ s| }|, EXEC_INTERP_SPEED: \"${EXEC_INTERP_SPEED}\" }|" "$YML"
sed -i "/id: trajectory_executor/,/outputs:/ s|tick: dora/timer/millis/[0-9]*|tick: dora/timer/millis/${EXEC_TICK_MS:-20}|" "$YML"
grep -q "id: ball_state" "$YML" || cat >> "$YML" <<YAML

  - id: ball_state
    path: $PY
    args: $SKILL/sim/ball_state.py
    env: { BALL_HTTP_HOST: "127.0.0.1", BALL_HTTP_PORT: "8779" }
    inputs:
      joint_positions: mujoco_sim/joint_positions
YAML

echo "[so101-demo] starting dora daemon…"
dora up >/dev/null 2>&1 || true
# POLL only — re-running `dora up` here spawns a second daemon, and two daemons
# co-spawn the dataflow (two windows). One up, then just wait for the coordinator.
for _ in $(seq 1 20); do dora list >/dev/null 2>&1 && break; sleep 1; done
dora list >/dev/null 2>&1 || die "dora coordinator never came up (try: dora destroy && dora up)"

echo "[so101-demo] launching viewer + dataflow (log: $LOG)…"
dora start "$YML" --attach > "$LOG" 2>&1 &
DORA_PID=$!

echo "[so101-demo] waiting for bridge + object server…"
for _ in $(seq 1 150); do
  curl -fsS -m2 "$URL/healthz" >/dev/null 2>&1 && break
  kill -0 "$DORA_PID" 2>/dev/null || { echo "[so101-demo] dataflow exited early:"; tail -25 "$LOG"; exit 1; }
  sleep 2
done
curl -fsS -m2 "$URL/healthz" >/dev/null 2>&1 || die "bridge /healthz never came up — see $LOG"
grep -qi "address already in use" "$LOG" && die "bridge failed to bind :8768 (orphan still up) — see $LOG"
for _ in $(seq 1 30); do curl -fsS -m2 "$BALL" >/dev/null 2>&1 && break; sleep 1; done
curl -fsS -m2 "$BALL" >/dev/null 2>&1 || die "object-state server never came up — see $LOG"

run_pick() {
  echo
  echo "[so101-demo] >>> running pick-and-place (cube -> green plate)…"
  ARM_BRIDGE_URL="$URL" BALL_URL="$BALL" MODEL_NAME="$MODEL" ROBOT_MANIFEST="$MANIFEST" \
    "$PY" "$SKILL/skill_pickplace.py"
}

echo
echo "[so101-demo] viewer is UP. cube at: $(curl -fsS -m2 "$BALL" 2>/dev/null)"
if [ "$AUTO" != "1" ]; then
  read -r -p "[so101-demo] Put the MuJoCo window in view, then press ENTER to start the pick… " _
fi
run_pick

if [ "$HEADLESS" = "1" ]; then
  echo "[so101-demo] headless run done."
else
  echo "[so101-demo] done — cube is on the plate. Viewer stays up."
  echo "[so101-demo] To run it again (cube resets to its start), just re-run this script."
  echo "[so101-demo] Ctrl-C to tear down."
  wait "$DORA_PID"
fi

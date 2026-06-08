#!/usr/bin/env bash
# ============================================================================
#  octos LLM agent drives the reBotArm B601-DM pick-and-place from a sentence.
#
#  This launcher lives WITH the vendor node (moveit-arm-dora-node/skill_pack) —
#  the octos bridge + dora-moveit framework are generic and untouched. All
#  reBot-specific grasp geometry lives in manifests/rebot.json (ROBOT_MANIFEST);
#  adding another arm of this class = a new manifest + a skills/<robot>/SKILL.md.
#
#  RUN IT (from a terminal inside the epyc remote desktop, so the viewer shows):
#      bash ~/dorarobotics-test/run-rebot-agent.sh
#      bash ~/dorarobotics-test/run-rebot-agent.sh "put the red block on the green plate"
#  DRIVER=skill -> deterministic (no LLM), for geometry tuning.
#  HEADLESS=1   -> no viewer.  Tear down: Ctrl-C.
# ============================================================================
set -uo pipefail
export DISPLAY="${DISPLAY:-:0}"
export PATH="$HOME/.cargo/bin:$PATH"

PY=/home/demo/anaconda3/envs/dora-moveit/bin/python
ROOT=/home/demo/dorarobotics-test
SKILL=$ROOT/moveit-arm-dora-node/skill_pack          # the arm skill pack (vendor repo)
MANIFEST="${ROBOT_MANIFEST:-$SKILL/manifests/rebot.json}"
SRC=$ROOT/rebot-mujoco-live.yml
YML=$ROOT/rebot-agent.yml
URL=http://127.0.0.1:8768
BALL=http://127.0.0.1:8779/ball
MODEL=/home/demo/Public/github_dora_nav_moveit/dora-moveit2/examples/move_group_demo/models/rebot_pickplace.xml
OLLAMA_BASE="${OLLAMA_BASE:-http://127.0.0.1:11434/v1}"
OLLAMA_MODEL="${OLLAMA_MODEL:-qwen3:8b}"
LOG=/tmp/rebot-agent.log
HEADLESS="${HEADLESS:-0}"
SENTENCE="${*:-Pick up the red block and place it on the green plate}"
DRIVER="${DRIVER:-agent}"

die() { echo "[rebot-agent] ERROR: $*" >&2; exit 1; }

echo "[rebot-agent] preflight checks…"
[ -f "$MODEL" ]    || die "MuJoCo model not found: $MODEL"
[ -f "$MANIFEST" ] || die "manifest not found: $MANIFEST"
[ -f "$SRC" ]      || die "live dataflow not found: $SRC (derive from skill_pack/dataflows/rebot-mujoco-bridge.yaml)"
command -v dora >/dev/null || die "dora CLI not on PATH ($HOME/.cargo/bin)"
if [ "$DRIVER" = "agent" ]; then
  curl -fsS -m4 "${OLLAMA_BASE%/v1}/api/tags" >/dev/null 2>&1 \
    || die "Ollama not reachable at $OLLAMA_BASE (start it: 'ollama serve')"
  curl -fsS -m4 "${OLLAMA_BASE%/v1}/api/tags" 2>/dev/null | grep -q "$OLLAMA_MODEL" \
    || die "Ollama model '$OLLAMA_MODEL' not pulled (run: 'ollama pull $OLLAMA_MODEL')"
  OCTOS_PY_DIR="${OCTOS_PY_DIR:-$ROOT}" "$PY" - <<'PYCHK' || die "python deps missing (need openai + octos_py)"
import os, sys
sys.path.insert(0, os.environ.get("OCTOS_PY_DIR"))
import openai            # noqa
import octos_py.agent    # noqa
PYCHK
  echo "[rebot-agent] preflight OK (model, manifest, dora, Ollama+$OLLAMA_MODEL, octos_py)"
else
  echo "[rebot-agent] preflight OK (model, manifest, dora; DRIVER=skill — no LLM needed)"
fi

# ---- build the dataflow: viewer/headless flag, executor speed, ball_state node ----
if [ "$HEADLESS" = "1" ]; then
  cp "$SRC" "$YML"
else
  sed -e 's|MUJOCO_HEADLESS: "1"|MUJOCO_HEADLESS: "0"\n      DISPLAY: ":0"|' "$SRC" > "$YML"
fi
sed -i "/env: {.*ROBOT_CONFIG_MODULE/ s| }|, EXEC_INTERP_SPEED: \"${EXEC_INTERP_SPEED:-0.3}\" }|" "$YML"
cat >> "$YML" <<YAML

  - id: ball_state
    path: $PY
    args: $SKILL/sim/ball_state.py
    env: { BALL_HTTP_HOST: "127.0.0.1", BALL_HTTP_PORT: "8779" }
    inputs:
      joint_positions: mujoco_sim/joint_positions
YAML

cleanup() {
  echo "[rebot-agent] tearing down…"
  dora stop --grace 3 >/dev/null 2>&1 || true
  dora destroy >/dev/null 2>&1 || true
  pkill -f octos_spec_bridge 2>/dev/null || true
  pkill -f ball_state.py 2>/dev/null || true
  pkill -f dora_mujoco 2>/dev/null || true
}
trap cleanup EXIT

echo "[rebot-agent] resetting dora daemon + killing node orphans + freeing ports…"
pkill -9 -f "dora_mujoco|move_group_demo|moveit_arm_node|octos_spec_bridge|ball_state|trajectory_execution|planning_scene" 2>/dev/null || true
fuser -k 8768/tcp 8779/tcp 2>/dev/null || true
dora destroy >/dev/null 2>&1 || true
sleep 3
fuser 8768/tcp >/dev/null 2>&1 && die "port 8768 still held after cleanup (kill it: fuser -k 8768/tcp)"
dora up >/dev/null 2>&1 || true
ready=0
for _ in $(seq 1 15); do
  if dora list >/dev/null 2>&1; then ready=1; break; fi
  dora up >/dev/null 2>&1 || true
  sleep 1
done
[ "$ready" = 1 ] || die "dora coordinator never came up (try: dora destroy && dora up)"

echo "[rebot-agent] starting dataflow (log: $LOG)…"
dora start "$YML" --attach > "$LOG" 2>&1 &
DORA_PID=$!

echo "[rebot-agent] waiting for bridge + object server (up to 5 min)…"
bridge_ok=0
for _ in $(seq 1 300); do
  curl -fsS -m2 "$URL/healthz" >/dev/null 2>&1 && { bridge_ok=1; break; }
  kill -0 "$DORA_PID" 2>/dev/null || { echo "[rebot-agent] dataflow exited early:"; tail -30 "$LOG"; exit 1; }
  sleep 1
done
[ "$bridge_ok" = 1 ] || die "bridge /healthz never came up — see $LOG (tail: $(tail -3 "$LOG"))"
ball_ok=0
for _ in $(seq 1 30); do curl -fsS -m2 "$BALL" >/dev/null 2>&1 && { ball_ok=1; break; }; sleep 1; done
[ "$ball_ok" = 1 ] || die "object-state server never came up — see $LOG"

echo
if [ "$DRIVER" = "agent" ]; then
  echo "[rebot-agent] >>> handing this sentence to the octos agent ($OLLAMA_MODEL):"
  echo "[rebot-agent] >>> \"$SENTENCE\""
  CMD=("$PY" "$SKILL/arm_agent.py" "$SENTENCE")
else
  echo "[rebot-agent] >>> running deterministic skill-level pick-and-place (no LLM)"
  CMD=("$PY" "$SKILL/skill_pickplace.py")
fi
echo
ARM_BRIDGE_URL="$URL" BALL_URL="$BALL" MODEL_NAME="$MODEL" ROBOT_MANIFEST="$MANIFEST" \
  OCTOS_PY_DIR="${OCTOS_PY_DIR:-$ROOT}" \
  OLLAMA_BASE="$OLLAMA_BASE" OLLAMA_MODEL="$OLLAMA_MODEL" \
  "${CMD[@]}"

if [ "$HEADLESS" = "1" ]; then
  echo "[rebot-agent] headless run done."
else
  echo "[rebot-agent] done. Viewer + dataflow stay up — Ctrl-C to tear down."
  wait "$DORA_PID"
fi

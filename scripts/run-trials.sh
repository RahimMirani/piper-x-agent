#!/usr/bin/env bash
# Run N evaluation trials of one agent against one task.
#
# Runs on the Mac, where the agent CLIs are signed in. Each trial gets a fresh
# session in an empty directory, so no CLAUDE.md, AGENTS.md or repo file can
# reach the agent and change its behaviour.
set -euo pipefail

AGENT=claude
TRIALS=20
TASK=cube-in-cup
OUT=""
PI_HOST="${PI_HOST:-pi}"
PI_CHECKOUT="${PI_CHECKOUT:-/home/rahim/piper-x-agent}"
PI_CONFIG="${PI_CONFIG:-config/live.toml}"
MODEL="${MODEL:-}"

usage() {
  cat >&2 <<'USAGE'
Usage: scripts/run-trials.sh [--agent claude|codex] [--trials N] [--task NAME] [--out DIR] [--model ID]

Prerequisite for --agent codex: register the MCP server once, because codex exec
takes no inline MCP config the way claude -p does:

  codex mcp add piper-x -- ssh -T -o BatchMode=yes pi \
    'bash /home/rahim/piper-x-agent/scripts/pi-mcp.sh config/live.toml'

Environment:
  PI_HOST      ssh alias for the Pi           (default: pi)
  PI_CHECKOUT  absolute repo path on the Pi   (default: /home/rahim/piper-x-agent)
  PI_CONFIG    config passed to pi-mcp.sh     (default: config/live.toml)
USAGE
  exit 2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --agent) AGENT="$2"; shift 2 ;;
    --trials) TRIALS="$2"; shift 2 ;;
    --task) TASK="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --model) MODEL="$2"; shift 2 ;;
    *) usage ;;
  esac
done

REPO="$(cd "$(dirname "$0")/.." && pwd)"
PROMPT_FILE="$REPO/eval/tasks/$TASK.txt"
[[ -f "$PROMPT_FILE" ]] || { echo "No such task: $PROMPT_FILE" >&2; exit 1; }
OUT="${OUT:-$REPO/runs-eval/$TASK-$AGENT-$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "$OUT"

MCP_CMD="ssh -T -o BatchMode=yes $PI_HOST bash $PI_CHECKOUT/scripts/pi-mcp.sh $PI_CONFIG"

# The arm must already be armed by an operator at the rig, and the window has to
# outlast the whole batch. Check before burning trials on refusals.
echo "Checking the arming window on $PI_HOST ..."
ARM_STATUS=$(ssh -T -o BatchMode=yes "$PI_HOST" \
  "$PI_CHECKOUT/.venv/bin/piper-agent arm-status" 2>/dev/null || echo '{"armed": false}')
echo "$ARM_STATUS"
if ! grep -q '"armed": true' <<<"$ARM_STATUS"; then
  cat >&2 <<'WARN'

The rig is NOT armed. Motion tools will not appear and every trial will fail.
Run this on the Pi first, at the rig, with the workspace clear:

    .venv/bin/piper-agent arm --minutes 45 --note "cube-in-cup trials"

WARN
  exit 1
fi

cat > "$OUT/batch.json" <<JSON
{
  "task": "$TASK",
  "agent": "$AGENT",
  "model": "${MODEL:-default}",
  "trials": $TRIALS,
  "started_unix_s": $(date +%s),
  "prompt_sha256": "$(shasum -a 256 "$PROMPT_FILE" | cut -d' ' -f1)",
  "repo_revision": "$(git -C "$REPO" rev-parse HEAD)",
  "pi_host": "$PI_HOST",
  "pi_config": "$PI_CONFIG"
}
JSON
cp "$PROMPT_FILE" "$OUT/prompt.txt"

for i in $(seq 1 "$TRIALS"); do
  TRIAL_DIR="$OUT/$(printf 'trial-%03d' "$i")"
  mkdir -p "$TRIAL_DIR"
  echo
  echo "=============================================================="
  echo " Trial $i of $TRIALS  ($TASK / $AGENT)"
  echo "=============================================================="
  echo " Reset the scene: cube on its mark, cup on its mark, arm clear."
  echo " (the arm is homed automatically after you confirm)"
  echo " Start the external video recording."
  read -r -p " Press ENTER when the scene is reset, or type 's' to skip: " REPLY
  if [[ "$REPLY" == "s" ]]; then
    echo '{"outcome": "skipped_by_operator"}' > "$TRIAL_DIR/result.json"
    continue
  fi

  # Return the arm to the configured home pose so every trial starts from the
  # same configuration. Without this, trial N starts wherever trial N-1 stopped
  # and the difference between two models is buried under that drift.
  echo " Homing the arm ..."
  if ! ssh -T -o BatchMode=yes "$PI_HOST" \
      "cd $PI_CHECKOUT && .venv/bin/piper-agent home --config $PI_CONFIG --confirm-motion-test" \
      > "$TRIAL_DIR/home.json" 2> "$TRIAL_DIR/home.log"; then
    echo " HOMING FAILED - see $TRIAL_DIR/home.log; marking trial invalid and stopping." >&2
    echo '{"outcome": "invalid", "reason": "homing failed"}' > "$TRIAL_DIR/result.json"
    exit 1
  fi

  # A fresh empty directory per trial: no repo, no agent instruction files.
  WORKDIR="$(mktemp -d "${TMPDIR:-/tmp}/piper-trial-XXXXXX")"
  START=$(date +%s)
  set +e
  case "$AGENT" in
    claude)
      # --allowedTools alone does NOT restrict the tool surface; it only
      # pre-approves permissions. A probe run with it still had Bash, Edit,
      # Write and the user's other MCP servers, any of which lets the agent
      # bypass every bound in this project. The restriction is:
      #   --restricted        drop Bash and the other code-running tools, and
      #                       ignore user/project/local settings files
      #   --tools ""          drop the remaining built-ins (Read, Write, Edit)
      #   --strict-mcp-config ignore every MCP server except the one below
      # --allowedTools then just avoids permission prompts on the arm tools.
      (cd "$WORKDIR" && claude -p "$(cat "$PROMPT_FILE")" \
          --output-format json \
          --restricted --strict-mcp-config --tools "" \
          --mcp-config "{\"mcpServers\":{\"piper-x\":{\"command\":\"ssh\",\"args\":[\"-T\",\"-o\",\"BatchMode=yes\",\"$PI_HOST\",\"bash $PI_CHECKOUT/scripts/pi-mcp.sh $PI_CONFIG\"]}}}" \
          --allowedTools "mcp__piper-x__robot_status,mcp__piper-x__read_arm_state,mcp__piper-x__observe,mcp__piper-x__observe_cameras,mcp__piper-x__move_joints,mcp__piper-x__move_to_pose,mcp__piper-x__set_gripper,mcp__piper-x__stop,mcp__piper-x__done" \
          ${MODEL:+--model "$MODEL"} \
        ) > "$TRIAL_DIR/transcript.json" 2> "$TRIAL_DIR/stderr.log"
      ;;
    codex)
      # Codex needs the equivalent lockdown before a real batch: confirm from a
      # probe run that it cannot reach a shell, and that only the piper-x server
      # is loaded. Its flags differ from Claude Code's, so do not assume the
      # ones above transfer.
      (cd "$WORKDIR" && codex exec "$(cat "$PROMPT_FILE")" \
          --json ${MODEL:+--model "$MODEL"} \
        ) > "$TRIAL_DIR/transcript.json" 2> "$TRIAL_DIR/stderr.log"
      ;;
    *) echo "Unknown agent: $AGENT" >&2; exit 2 ;;
  esac
  STATUS=$?
  set -e
  END=$(date +%s)
  rm -rf "$WORKDIR"

  # The MCP server writes one run directory per session; record which one.
  PI_RUN=$(ssh -T -o BatchMode=yes "$PI_HOST" \
    "ls -t $PI_CHECKOUT/runs | head -1" 2>/dev/null || echo "")
  cat > "$TRIAL_DIR/result.json" <<JSON
{
  "trial": $i,
  "exit_status": $STATUS,
  "elapsed_s": $((END - START)),
  "pi_run_directory": "$PI_RUN"
}
JSON
  echo " exit=$STATUS elapsed=$((END - START))s pi_run=$PI_RUN"
  echo " Stop the video recording and save it as $TRIAL_DIR/video.mp4"
done

echo
echo "Batch complete: $OUT"
echo "Build the grading sheet:  scripts/grade-trials.py $OUT"

#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-demo}"
case "$MODE" in
  demo|settle|no_zero|no_load|one_arm|verify_early|dream) ;;
  *) echo "Unknown mode: $MODE" >&2; exit 2 ;;
esac

TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUT="${OUT:-$HOME/yam-eval/gripper-force-calibration-$MODE}"
mkdir -p -- "$OUT"
if [[ "$MODE" == dream ]]; then
  export DEMO_MODE=demo  # unused by the Dream policy, valid if demo() is queried
else
  export DEMO_MODE="$MODE"
fi
export TASK_TRACE="$OUT/trace.jsonl"
STEPS="${STEPS:-2500}"
if [[ "$MODE" == settle ]]; then STEPS="${STEPS_SETTLE:-200}"; fi
POLICY=yam_scripted
if [[ "$MODE" == dream ]]; then POLICY=dream; fi

inspect-robots run --embodiment yam_arena --policy "$POLICY" \
  -E "task=$TASK_DIR/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Press white ZERO while the orange reference pad is unloaded. Hold the pad until the status lamp turns cyan, press blue SPAN with the other gripper, then release the pad and press green VERIFY; leave the lamp green." \
  --scorer task_score --max-steps "$STEPS" --store-frames --log-dir "$OUT"

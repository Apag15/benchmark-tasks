#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-demo}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUT="${OUT:-$HOME/yam-eval/tactile-cartridge-diagnostic}"
if [[ "$MODE" == settle ]]; then
  STEPS="${STEPS:-200}"
else
  STEPS="${STEPS:-2400}"
fi
mkdir -p "$OUT/$MODE"
export TASK_TRACE="$OUT/$MODE/trace.jsonl"
export PYTHONDONTWRITEBYTECODE=1

case "$MODE" in
  demo|settle|no_test|no_release|wrong_spare)
    export DEMO_MODE="$MODE"
    POLICY=yam_scripted
    ;;
  dream)
    POLICY=dream
    ;;
  *)
    echo "Unknown mode: $MODE" >&2
    exit 2
    ;;
esac

inspect-robots run --embodiment yam_arena --policy "$POLICY" \
  -E "task=$HERE/task.py" \
  -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Press the white TEST button to identify the needed tactile cartridge, then hold the black RELEASE button while pulling the old cartridge into the red return tray and sliding the indicated spare into the covered socket. Release the button, press TEST again, and leave the green status lamp lit with both grippers clear." \
  --scorer task_score --max-steps "$STEPS" --store-frames \
  --log-dir "$OUT/$MODE"

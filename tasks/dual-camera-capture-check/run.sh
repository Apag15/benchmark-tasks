#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval/dual-camera-capture-check-$MODE}"
STEPS="${STEPS:-2400}"
INSTRUCTION="Press the white ARM button, then hold the red and blue spring targets down together until the green capture lamp lights. Let both targets rise and withdraw both grippers."

case "$MODE" in
  demo|settle|no_arm|one_arm|sequential|short_hold)
    export DEMO_MODE="$MODE"
    POLICY="yam_scripted"
    ;;
  dream)
    POLICY="dream"
    ;;
  *)
    echo "Unknown mode: $MODE" >&2
    exit 2
    ;;
esac

if [[ -f /data/IsaacLab-Arena/yam.env ]]; then
  source /data/IsaacLab-Arena/yam.env
fi

mkdir -p "$OUT"
export TASK_TRACE="$OUT/trace.jsonl"
inspect-robots run --embodiment yam_arena --policy "$POLICY" \
  -E "task=$HERE/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score \
  --max-steps "$STEPS" --store-frames --log-dir "$OUT"

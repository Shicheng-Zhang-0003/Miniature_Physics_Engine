#!/usr/bin/env bash
# V-04: F10 long-run validation (3600 ticks / 60s). GUI interaction required.
set -uo pipefail
SRC="v15R1/src"
[[ -d "$SRC" ]] || { echo "ERROR: $SRC not found." >&2; exit 1; }

echo "=== V-04: F10 Long-Run Validation ==="
if [[ ! -x "$SRC/engine" ]]; then
    echo "engine binary not found; building (normal, non-sanitizer)..."
    (cd "$SRC" && make)
fi

cat <<'EOF'

This test requires the GUI. Steps:

  1. cd v15R1/src && ./engine
  2. When the window opens, press  F10
       - Spawns the long-run scene (stack + pile + spheres)
       - A countdown 'LR:NNs' appears in the top-left overlay
  3. Wait ~60 seconds. Do NOT interact with the scene.
  4. Watch the terminal (stdout). It prints at the end:
       [A3] Long-run validation report v15R1
       [A3] result: PASS        (or FAIL)

PASS criteria (all must hold):
  - objects > 0
  - nan_ticks = 0
  - fallen_ticks = 0
  - final max linear speed  < 0.25
  - final max angular speed < 0.5

Record the result for V-03 (Gate 9) and V-05.
EOF

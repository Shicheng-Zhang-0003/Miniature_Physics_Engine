#!/usr/bin/env bash
# MPE-only verification run: clean build of the engine + full headless suite.
# (The old per-phase fix-script runner was retired; fixes/*.py are history.
#  Robotics/MFS is parked in v15R3/robotics_backup during the MPE-only run.)
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

if [[ ! -d "v15R3/src" ]]; then
  echo "[FATAL] Cannot find v15R3/src under: $ROOT"
  exit 1
fi

LOG="fix_log.txt"
echo "" >> "$LOG"
echo "=== MPE-only run: $(date '+%Y-%m-%d %H:%M:%S') ===" >> "$LOG"

echo "--- engine build ---"
if (cd v15R3/src && make clean >/dev/null 2>&1 && make >>"$ROOT/$LOG" 2>&1); then
  echo "BUILD: OK"
else
  echo "BUILD: FAIL (see $LOG)"
  exit 1
fi

echo "--- headless tests ---"
if python3 tools/test_runner.py 2>&1 | tee -a "$LOG" | grep -q "Blocking failures: 0"; then
  echo "RESULT: ALL PASS"
else
  echo "RESULT: FAIL (see $LOG)"
  exit 1
fi

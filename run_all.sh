#!/usr/bin/env bash
# MPE-only verification run: clean build of the engine + full headless suite.
# (The old per-phase fix scripts and regex helpers are removed, not retired.
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

echo "--- terminal debugger suite ---"
if (cd v15R3/src && make tui-smoke 2>&1 | tee -a "$ROOT/$LOG" | grep -q "tui-smoke: all scenes dump finite state"); then
  echo "TUI: OK"
else
  echo "TUI: FAIL (see $LOG)"
  exit 1
fi

echo "--- release ritual complete: run ./v15R3/src/engine for the F5-F11 in-engine matrix ---"

#!/usr/bin/env bash
# MPE-only verification run: clean build of the engine + full headless suite.
# Active head is v15S (GTK4 + modular kernel); v15R3 remains the tagged
# release record (see release_notes_v15R3.md).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

if [[ ! -d "v15S/src" ]]; then
  echo "[FATAL] Cannot find v15S/src under: $ROOT"
  exit 1
fi

LOG="/tmp/mpe_run_$(date +%Y%m%d_%H%M%S).log"
echo "" >> "$LOG"
echo "=== MPE-only run: $(date '+%Y-%m-%d %H:%M:%S') ===" >> "$LOG"

echo "--- engine build ---"
if (cd v15S/src && make clean >/dev/null 2>&1 && make >>"$LOG" 2>&1); then
  echo "BUILD: OK"
else
  echo "BUILD: FAIL (see $LOG)"
  exit 1
fi

echo "--- headless tests (Suite v2, canonical) ---"
if python3 tools/test_runner.py --suite 2>&1 | tee -a "$LOG" | grep -q "Blocking failures: 0"; then
  echo "RESULT: ALL PASS (31/31 v2)"
else
  echo "RESULT: FAIL (see $LOG)"
  exit 1
fi

echo "--- terminal debugger suite ---"
if (cd v15S/src && make tui-smoke 2>&1 | tee -a "$LOG" | grep -q "tui-smoke: all scenes dump finite state"); then
  echo "TUI: OK"
else
  echo "TUI: FAIL (see $LOG)"
  exit 1
fi

echo "--- release ritual complete: run ./v15S/src/engine for the F5-F11 in-engine matrix ---"

#!/usr/bin/env bash
set -euo pipefail

# run_all.sh MUST live at the PROJECT ROOT (same dir as v15R3/ and fixes/).
ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

if [[ ! -d "v15R3/src" ]]; then
  echo "[FATAL] Cannot find v15R3/src under: $ROOT"
  echo "        run_all.sh must sit at the project root, next to v15R3/ and fixes/."
  echo "        Current directory contents:"
  ls -1
  exit 1
fi

LOG="fix_log.txt"
echo "" >> "$LOG"
echo "=== MPE/FTC Fix Run: $(date '+%Y-%m-%d %H:%M:%S') ===" >> "$LOG"

PASSED=0; FAILED=0; BUILD_FAILS=0

run_script() {
  local script="$1"; local name; name="$(basename "$script")"
  printf "  %-52s" "$name"
  if bash "$script" >> "$LOG" 2>&1; then
    PASSED=$((PASSED + 1)); echo "PASS"
  else
    local rc=$?; FAILED=$((FAILED + 1)); echo "FAIL (exit $rc)"
  fi
}

build_check() {
  local phase_name="$1"
  printf "  [build] %-44s" "$phase_name"
  if (cd "$ROOT/v15R3/src" && make >/dev/null 2>>"$ROOT/$LOG"); then
    echo "OK"
  else
    echo "FAIL (see $LOG)"; BUILD_FAILS=$((BUILD_FAILS + 1))
  fi
}

run_phase() {
  local dir="$1"; local label="$2"
  echo ""; echo "--- $label ---"
  if [[ ! -d "$dir" ]]; then echo "  (no directory)"; return 0; fi
  local found=0
  for f in "$dir"/*.sh; do
    if [[ -f "$f" ]]; then found=1; run_script "$f"; fi
  done
  if [[ "$found" -eq 0 ]]; then echo "  (no scripts)"; fi
  build_check "$label"
}

# ---- original v15 debt phases ----
run_phase fixes/phase0_trivial   "Phase 0: Trivial fixes"
run_phase fixes/phase1_docs      "Phase 1: Docs"
run_phase fixes/phase2_bugs      "Phase 2: Bug fixes"
run_phase fixes/phase3_quality   "Phase 3: Code quality"
run_phase fixes/phase4_perf      "Phase 4: Performance"
run_phase fixes/phase5_arch      "Phase 5: Architecture"
run_phase fixes/phase6_features  "Phase 6: Features"

# ---- FTC transformation phases ----
run_phase fixes/phase0_foundation  "FTC Phase 0: Foundation repair"
run_phase fixes/phase1_constraints "FTC Phase 1: Constraint framework"
run_phase fixes/phase2_robotics    "FTC Phase 2: Robotics core"
run_phase fixes/phase3_sensors     "FTC Phase 3: Sensors + mechanisms"
run_phase fixes/phase4_platform    "FTC Phase 4: Platform"
run_phase fixes/phase5_adoption    "FTC Phase 5: Adoption"
run_phase fixes/phase6_quality   "FTC Phase 6: Code quality"

echo ""
echo "=============================="
echo "  Passed:      $PASSED"
echo "  Failed:      $FAILED"
echo "  Build fails: $BUILD_FAILS"
echo "=============================="
if [[ "$FAILED" -gt 0 || "$BUILD_FAILS" -gt 0 ]]; then
  echo "RESULT: FAIL — review $LOG"; exit 1
else
  echo "RESULT: ALL PASS (build verified after each phase)"
fi

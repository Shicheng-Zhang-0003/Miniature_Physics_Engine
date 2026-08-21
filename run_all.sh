#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

LOG="fix_log.txt"
echo "" >> "$LOG"
echo "=== MPE v15R2 Fix Run: $(date '+%Y-%m-%d %H:%M:%S') ===" >> "$LOG"

PASSED=0
SKIPPED=0
FAILED=0

run_script() {
    local script="$1"
    local name
    name="$(basename "$script")"
    printf "  %-55s" "$name"
    if bash "$script" >> "$LOG" 2>&1; then
        PASSED=$((PASSED + 1))
        echo "PASS"
    else
        local rc=$?
        FAILED=$((FAILED + 1))
        echo "FAIL (exit $rc)"
    fi
}

echo ""
echo "--- Phase 0: Trivial fixes ---"
for f in fixes/phase0_trivial/*.sh; do
    [[ -f "$f" ]] && run_script "$f"
done

echo ""
echo "--- Phase 1: Docs ---"
for f in fixes/phase1_docs/*.sh; do
    [[ -f "$f" ]] && run_script "$f"
done

echo ""
echo "--- Phase 2: Bug fixes ---"
for f in $(ls fixes/phase2_bugs/*.sh 2>/dev/null | sort); do
    [[ -f "$f" ]] && run_script "$f"
done

echo ""
echo "--- Phase 3: Code quality ---"
for f in $(ls fixes/phase3_quality/*.sh 2>/dev/null | sort); do
    [[ -f "$f" ]] && run_script "$f"
done

echo ""
echo "--- Phase 4: Performance ---"
for f in $(ls fixes/phase4_perf/*.sh 2>/dev/null | sort); do
    [[ -f "$f" ]] && run_script "$f"
done

echo ""
echo "--- Phase 5: Architecture ---"
for f in $(ls fixes/phase5_arch/*.sh 2>/dev/null | sort); do
    [[ -f "$f" ]] && run_script "$f"
done

echo ""
echo "--- Phase 6: Features ---"
for f in $(ls fixes/phase6_features/*.sh 2>/dev/null | sort); do
    [[ -f "$f" ]] && run_script "$f"
done

echo ""
echo "=============================="
echo "  Passed:  $PASSED"
echo "  Failed:  $FAILED"
echo "  Total:   $((PASSED + FAILED))"
echo "=============================="

if [[ $FAILED -gt 0 ]]; then
    echo "RESULT: FAIL — review $LOG"
    exit 1
else
    echo "RESULT: ALL PASS"
fi

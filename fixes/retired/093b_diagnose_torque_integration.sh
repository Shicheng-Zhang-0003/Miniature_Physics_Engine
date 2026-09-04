#!/usr/bin/env bash
# ============================================================
# FIX 093b — Diagnose: why torque_accumulator doesn't spin wheels
#   READ-ONLY. Prints the velocity-integration code so we can see
#   exactly where torque is dropped. Root cause is one of:
#     A) rb_integrate_velocity ignores torque_accumulator, or
#     B) physics_world_step clears accumulators before integration.
#
# Phase:   phase3_sensors (cylinder keystone)
# Files:   none modified
# Depends: 093a
# Risk:    none (read-only)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

RB_C="v15R2/src/core/rigidbody.c"
RB_H="v15R2/src/core/rigidbody.h"
PW_C="v15R2/src/core/physics_world.c"

echo "============================================================"
echo "[1] rb_integrate_velocity — full body (rigidbody.c)"
echo "============================================================"
awk '/rb_integrate_velocity[ ]*\(/,/^}/' "$RB_C" || echo "(function not found)"

echo ""
echo "============================================================"
echo "[2] all integrate-related symbols in rigidbody.c"
echo "============================================================"
grep -n 'integrate' "$RB_C" || echo "(none)"

echo ""
echo "============================================================"
echo "[3] accumulator / integrate references in physics_world.c"
echo "============================================================"
grep -n 'accumulator\|integrate\|vector3_zero' "$PW_C" || echo "(none)"

echo ""
echo "============================================================"
echo "[4] physics_world_step — the velocity integration region"
echo "============================================================"
awk '/void physics_world_step/,/^}/' "$PW_C" | grep -n 'integrat\|accumulat\|velocity\|gravity\|for (' || echo "(no matches in step)"

echo ""
echo "============================================================"
echo "[5] VERDICT: does rb_integrate_velocity use torque_accumulator?"
echo "============================================================"
if awk '/rb_integrate_velocity[ ]*\(/,/^}/' "$RB_C" | grep -q 'torque_accumulator'; then
    echo "YES — torque_accumulator IS referenced."
    echo "      If probe A still fails, accumulators are likely cleared"
    echo "      at the START of physics_world_step (cause B)."
else
    echo "NO — torque_accumulator is NOT integrated."
    echo "     >>> Cause A confirmed: this is the root cause of probe A."
fi

echo ""
echo "[DIAG] paste all of the above back."

#!/usr/bin/env bash
# ============================================================
# FIX 061 — PHYS-003: constraint pool + revolute registration
#   Pool, add, remove implemented. constraint_solve_all is the
#   declared solver seam, filled in alongside islanding (Phase 1).
# Phase:   phase1_constraints
# Files:   v15R2/src/physics/constraint.c
# Depends: 060
# Risk:    low (new file only)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET="v15R2/src/physics/constraint.c"
grep -q 'MPE_FTC_061' "$TARGET" 2>/dev/null && { echo "[SKIP] constraint.c already present"; exit 0; }

cat > "$TARGET" <<'EOF'
/* MPE_FTC_061 */
#include "constraint.h"
#include "../config/mpe_constants.h"

static constraint constraint_pool [mpe_max_joints];
static int constraint_count = 0;

void constraint_pool_init (void) {
    for (int i = 0; i < mpe_max_joints; i++) { constraint_pool [i].is_active = false; }
    constraint_count = 0;
}

int constraint_add_revolute (uint32_t id_a, uint32_t id_b, vector3 anchor_a, vector3 anchor_b, vector3 axis_a) {
    if ((id_a == 0) || (id_b == 0) || (id_a == id_b)) { return -1; }
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!constraint_pool [i].is_active) {
            constraint_pool [i].type = CONSTRAINT_REVOLUTE;
            constraint_pool [i].body_id_a = id_a;
            constraint_pool [i].body_id_b = id_b;
            constraint_pool [i].p.revolute.anchor_a = anchor_a;
            constraint_pool [i].p.revolute.anchor_b = anchor_b;
            constraint_pool [i].p.revolute.axis_a = vector3_normalisation (axis_a);
            constraint_pool [i].p.revolute.motor_enabled = false;
            constraint_pool [i].p.revolute.limits_enabled = false;
            constraint_pool [i].is_active = true;
            constraint_count++;
            return i;
        }
    }
    return -1;
}

void constraint_remove (int index) {
    if ((index < 0) || (index >= mpe_max_joints)) { return; }
    if (!constraint_pool [index].is_active) { return; }
    constraint_pool [index].is_active = false;
    constraint_count--;
}

/* Solver seam: populated alongside islanding in Phase 1. */
void constraint_solve_all (int iterations) {
    (void) iterations;
}
EOF

grep -q 'constraint_add_revolute' "$TARGET" || { echo "[FAIL] constraint.c not written"; exit 1; }
echo "[PASS] 061: constraint pool added"

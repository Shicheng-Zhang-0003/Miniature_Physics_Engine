#!/usr/bin/env bash
# ============================================================
# FIX 060 — PHYS-003: constraint framework header (additive)
#   Declares constraint types + base struct. FTC needs hard
#   constraints (revolute/fixed/prismatic), not just spring joints.
# Phase:   phase1_constraints
# Files:   v15R2/src/physics/constraint.h
# Depends: none
# Risk:    low (new file only)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET="v15R2/src/physics/constraint.h"
grep -q 'MPE_FTC_060' "$TARGET" 2>/dev/null && { echo "[SKIP] constraint.h already present"; exit 0; }

cat > "$TARGET" <<'EOF'
/* MPE_FTC_060: generic constraint framework */
#ifndef constraint_h
#define constraint_h

#include "../core/rigidbody.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CONSTRAINT_SPRING,
    CONSTRAINT_REVOLUTE,
    CONSTRAINT_FIXED,
    CONSTRAINT_PRISMATIC,
    CONSTRAINT_DISTANCE
} constraint_type;

typedef struct {
    vector3 anchor_a;
    vector3 anchor_b;
    vector3 axis_a;
    float motor_target_speed;
    float motor_max_torque;
    float limit_min_rad;
    float limit_max_rad;
    bool motor_enabled;
    bool limits_enabled;
} revolute_params;

typedef struct {
    constraint_type type;
    uint32_t body_id_a;
    uint32_t body_id_b;
    bool is_active;
    union {
        revolute_params revolute;
    } p;
} constraint;

void constraint_pool_init (void);
int  constraint_add_revolute (uint32_t id_a, uint32_t id_b, vector3 anchor_a, vector3 anchor_b, vector3 axis_a);
void constraint_remove (int index);
void constraint_solve_all (int iterations);

#endif
EOF

grep -q 'CONSTRAINT_REVOLUTE' "$TARGET" || { echo "[FAIL] constraint.h not written"; exit 1; }
echo "[PASS] 060: constraint.h added"

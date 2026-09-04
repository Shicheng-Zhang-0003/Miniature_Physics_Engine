#!/usr/bin/env bash
# ============================================================
# FIX 063 — PHYS-003: constraint pool dispatch + motor control
#   constraint_solve_all() resolves body IDs and dispatches REVOLUTE
#   constraints to revolute_solve(). constraint_apply_motors() drives
#   enabled motors once per tick. Adds a motor setter for robotics code.
# Phase:   phase1_constraints
# Files:   v15R3/src/physics/constraint.h, constraint.c (rewrite)
# Depends: 060, 061, 062
# Risk:    low (rewrites files this fleet created)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
H="v15R3/src/physics/constraint.h"
C="v15R3/src/physics/constraint.c"
grep -q 'MPE_FTC_063' "$C" 2>/dev/null && { echo "[SKIP] dispatch already present"; exit 0; }
[[ -f "v15R3/src/physics/revolute_joint.h" ]] || { echo "[SKIP] revolute_joint.h missing (run 062)"; exit 0; }
[[ -f "$H" ]] && cp "$H" "${H}.pre_063"
[[ -f "$C" ]] && cp "$C" "${C}.pre_063"

cat > "$H" <<'EOF'
/* MPE_FTC_060 header, updated MPE_FTC_063 */
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
int  constraint_get_count (void);
void constraint_set_revolute_motor (int index, bool enabled, float target_speed, float max_torque);
/* Positional/axis solve — call once per tick. */
void constraint_solve_all (rigidbody *bodies, int body_count, float dt);
/* Motor drive — call once per tick before velocity integration. */
void constraint_apply_motors (rigidbody *bodies, int body_count, float dt);
#endif
EOF

cat > "$C" <<'EOF'
/* MPE_FTC_063 */
#include "constraint.h"
#include "revolute_joint.h"
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
            constraint_pool [i].p.revolute.motor_target_speed = 0.0f;
            constraint_pool [i].p.revolute.motor_max_torque = 0.0f;
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

int constraint_get_count (void) { return constraint_count; }

void constraint_set_revolute_motor (int index, bool enabled, float target_speed, float max_torque) {
    if ((index < 0) || (index >= mpe_max_joints)) { return; }
    if (!constraint_pool [index].is_active) { return; }
    if (constraint_pool [index].type != CONSTRAINT_REVOLUTE) { return; }
    constraint_pool [index].p.revolute.motor_enabled = enabled;
    constraint_pool [index].p.revolute.motor_target_speed = target_speed;
    constraint_pool [index].p.revolute.motor_max_torque = max_torque;
}

static rigidbody *find_body_by_id (rigidbody *bodies, int body_count, uint32_t id) {
    if (!bodies) { return NULL; }
    for (int i = 0; i < body_count; i++) {
        if (bodies [i].object_id == id) { return &bodies [i]; }
    }
    return NULL;
}

static void constraint_dispatch (rigidbody *bodies, int body_count, float dt, bool motors_pass) {
    if ((!bodies) || (body_count <= 0)) { return; }
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!constraint_pool [i].is_active) { continue; }
        constraint *c = &constraint_pool [i];
        rigidbody *body_a = find_body_by_id (bodies, body_count, c->body_id_a);
        rigidbody *body_b = find_body_by_id (bodies, body_count, c->body_id_b);
        if ((!body_a) || (!body_b)) { continue; }
        if (c->type == CONSTRAINT_REVOLUTE) {
            if (motors_pass) { revolute_apply_motor (&c->p.revolute, body_a, body_b, dt); }
            else { revolute_solve (&c->p.revolute, body_a, body_b, dt); }
        }
        /* CONSTRAINT_FIXED / CONSTRAINT_PRISMATIC dispatched in 064 / 065 */
    }
}

void constraint_solve_all (rigidbody *bodies, int body_count, float dt) {
    if (dt <= 0.0f) { return; }
    constraint_dispatch (bodies, body_count, dt, false);
}

void constraint_apply_motors (rigidbody *bodies, int body_count, float dt) {
    if (dt <= 0.0f) { return; }
    constraint_dispatch (bodies, body_count, dt, true);
}
EOF

grep -q 'constraint_solve_all (rigidbody' "$H" || { echo "[FAIL] header signature not updated"; exit 1; }
grep -q 'revolute_solve' "$C" || { echo "[FAIL] dispatch not wired"; exit 1; }
echo "[PASS] 063: constraint pool now dispatches revolute constraints + motors"

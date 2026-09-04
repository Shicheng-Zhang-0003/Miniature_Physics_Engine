#!/usr/bin/env bash
# ============================================================
# FIX 056 — ARCH-001 (real): physics_world implementation
#   Owning struct + working free-body step. Replaces dead scaffold.
# Phase:   phase0_foundation
# Files:   v15R3/src/core/physics_world.c
# Depends: 055
# Risk:    low (whole-file overwrite)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET="v15R3/src/core/physics_world.c"
grep -q 'MPE_FTC_056' "$TARGET" 2>/dev/null && { echo "[SKIP] real impl already present"; exit 0; }
[[ -f "$TARGET" ]] && cp "$TARGET" "${TARGET}.pre_056"

cat > "$TARGET" <<'EOF'
/* MPE_FTC_056: Real physics world — owns bodies, no camera/input coupling. */
#include "physics_world.h"
#include "../config/mpe_constants.h"
#include "../config/mpe_config.h"
#include <stdlib.h>
#include <math.h>

static physics_world g_physics_world = {
    .bodies = NULL, .body_count = 0, .body_capacity = 0, .next_object_id = 1
};

void physics_world_init (physics_world *world) {
    if (!world) {return;}
    if (!world->bodies) {
        world->bodies = (rigidbody *) malloc ((size_t) mpe_max_bodies * sizeof (rigidbody));
        world->body_capacity = mpe_max_bodies;
    }
    world->body_count = 0;
    if (world->next_object_id == 0) {world->next_object_id = 1;}
}

void physics_world_cleanup (physics_world *world) {
    if (!world) {return;}
    if (world->bodies) {free (world->bodies); world->bodies = NULL;}
    world->body_count = 0;
    world->body_capacity = 0;
}

int physics_world_add_sphere (physics_world *world, float radius, float mass, vector3 position) {
    if ((!world) || (!world->bodies) || (world->body_count >= world->body_capacity)) {return -1;}
    rigidbody *rb = &world->bodies [world->body_count];
    rigidbody_initialisation_sphere (rb, radius, mass, position);
    rb->object_id = world->next_object_id++;
    rb->object_generation = 1;
    rigidbody_sanitize (rb);
    return world->body_count++;
}

int physics_world_add_cube (physics_world *world, vector3 position, vector3 half_extensions, float mass) {
    if ((!world) || (!world->bodies) || (world->body_count >= world->body_capacity)) {return -1;}
    rigidbody *rb = &world->bodies [world->body_count];
    rigidbody_initialisation_cube (rb, position, half_extensions, mass);
    rb->object_id = world->next_object_id++;
    rb->object_generation = 1;
    rigidbody_sanitize (rb);
    return world->body_count++;
}

void physics_world_clear (physics_world *world) {
    if (!world) {return;}
    world->body_count = 0;
}

/* Free-body step: gravity -> integrate velocity -> integrate position.
 * Collision + constraints are layered in by later phases. */
void physics_world_step (physics_world *world, float dt) {
    if ((!world) || (!world->bodies) || (dt <= 0.0f)) {return;}
    float linear_damping = powf (g_cfg.world.drag, dt);
    float angular_damping = powf (g_cfg.world.drag * 0.97f, dt);
    vector3 gravity = {0.0f, g_cfg.world.gravity, 0.0f};
    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies [i];
        if ((rb->static_state) || (rb->is_sleeping)) {continue;}
        rb_apply_forces_perfect (rb, vector3_scaling (gravity, rb->mass));
    }
    for (int i = 0; i < world->body_count; i++) {
        rb_integrate_velocity (&world->bodies [i], dt, linear_damping, angular_damping);
    }
    for (int i = 0; i < world->body_count; i++) {
        rb_integrate_position (&world->bodies [i], dt);
        rigidbody_sanitize (&world->bodies [i]);
    }
}

physics_world *physics_world_get_primary (void) {
    return &g_physics_world;
}
EOF

grep -q 'MPE_FTC_056' "$TARGET" || { echo "[FAIL] impl not written"; exit 1; }
grep -q 'physics_world_step' "$TARGET" || { echo "[FAIL] step missing"; exit 1; }
echo "[PASS] 056: real physics_world.c installed"

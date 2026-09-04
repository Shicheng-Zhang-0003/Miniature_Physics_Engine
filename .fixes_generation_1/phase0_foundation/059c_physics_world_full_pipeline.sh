#!/usr/bin/env bash
# ============================================================
# FIX 059c — ARCH-001 (real): full physics_world pipeline
#   Replaces the MPE_FTC_056 free-body-only step with the complete
#   pipeline: sanitize -> broadphase -> narrowphase -> floor ->
#   gravity -> integrate velocity -> iterative solve -> cache save
#   -> integrate position. Operates purely on world->bodies.
#   Deferred (documented in-file): sleep staticize, depenetration,
#   joints (Phase 1), per-world contact cache.
# Phase:   phase0_foundation
# Files:   v15R3/src/core/physics_world.c
# Depends: 055, 056, 059
# Risk:    low (whole-file overwrite of a file this fleet owns)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET="v15R3/src/core/physics_world.c"
[[ -f "$TARGET" ]] || { echo "[SKIP] physics_world.c not found (are 055/056 applied?)"; exit 0; }
grep -q 'MPE_FTC_059C' "$TARGET" && { echo "[SKIP] full pipeline already present"; exit 0; }
cp "$TARGET" "${TARGET}.pre_059c"

cat > "$TARGET" <<'EOF'
/* MPE_FTC_059C: physics world — full pipeline.
 * Supersedes MPE_FTC_056 (free-body step).
 * Pipeline mirrors the legacy loop in simulation.c, minus:
 *   - sleep staticize hack (sleeping bodies keep real mass here),
 *   - positional depenetration pass,
 *   - joints/constraints (Phase 1).
 * NOTE: the contact warm-start cache is still engine-global. Worlds
 * must seed non-overlapping object_id ranges (see tests/two_world_test.c).
 * A per-world cache is tracked as future work.
 */
#include "physics_world.h"
#include "../physics/collision_mechanics.h"
#include "../physics/broadphase.h"
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include <stdlib.h>
#include <math.h>

static physics_world g_physics_world = {
    .bodies = NULL, .body_count = 0, .body_capacity = 0, .next_object_id = 1
};

static broadphase_pair world_pairs [mpe_max_broadphase_pairs];
static collision_data world_manifolds [a3_max_manifolds];

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

void physics_world_step (physics_world *world, float dt) {
    if ((!world) || (!world->bodies) || (dt <= 0.0f) || (world->body_count <= 0)) {return;}

    for (int i = 0; i < world->body_count; i++) {
        rigidbody_sanitize (&world->bodies [i]);
    }

    int pair_count = 0;
    if (world->body_count >= 2) {
        pair_count = broadphase_generate_pairing (world->bodies, world->body_count, world_pairs, mpe_max_broadphase_pairs);
    }

    int manifold_count = 0;
    for (int p = 0; p < pair_count; p++) {
        int index_a = world_pairs [p].object_index_a;
        int index_b = world_pairs [p].object_index_b;
        if ((index_a < 0) || (index_a >= world->body_count)) {continue;}
        if ((index_b < 0) || (index_b >= world->body_count)) {continue;}
        rigidbody *body_a = &world->bodies [index_a];
        rigidbody *body_b = &world->bodies [index_b];
        if ((body_a->is_sleeping) && (body_b->is_sleeping)) {continue;}
        collision_data narrowphase_collision = {0};
        bool collided = false;
        if ((body_a->type == object_sphere) && (body_b->type == object_sphere)) {
            collided = collision_dual_sphere (body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_sphere) && (body_b->type == object_cube)) {
            collided = collision_sphere_cube (body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_cube) && (body_b->type == object_sphere)) {
            collided = collision_sphere_cube (body_b, body_a, &narrowphase_collision);
            if (collided) {
                narrowphase_collision.normal_vector = vector3_scaling (narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            }
        } else if ((body_a->type == object_cube) && (body_b->type == object_cube)) {
            collided = collision_dual_cube (body_a, body_b, &narrowphase_collision);
        }
        if ((collided) && (manifold_count < a3_max_manifolds)) {
            collision_prepare_solver (&narrowphase_collision, &world_manifolds [manifold_count]);
            manifold_count++;
        }
    }

    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies [i];
        if ((rb->static_state) || (rb->is_sleeping)) {continue;}
        collision_data floor_collision = {0};
        if ((collision_static_plane_body (rb, 0.0f, &floor_collision)) && (manifold_count < a3_max_manifolds)) {
            collision_prepare_solver (&floor_collision, &world_manifolds [manifold_count]);
            manifold_count++;
        }
    }

    vector3 gravity = {0.0f, g_cfg.world.gravity, 0.0f};
    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies [i];
        if ((rb->static_state) || (rb->is_sleeping)) {continue;}
        rb_apply_forces_perfect (rb, vector3_scaling (gravity, rb->mass));
    }

    float linear_damping = powf (g_cfg.world.drag, dt);
    float angular_damping = powf (g_cfg.world.drag * 0.97f, dt);
    for (int i = 0; i < world->body_count; i++) {
        rb_integrate_velocity (&world->bodies [i], dt, linear_damping, angular_damping);
    }

    int solver_iterations = g_cfg.timestep.solver_iterations;
    for (int iter = 0; iter < solver_iterations; iter++) {
        for (int m = 0; m < manifold_count; m++) {collision_resolve_iterative (&world_manifolds [m]);}
    }
    contact_cache_save (world_manifolds, manifold_count);

    for (int i = 0; i < world->body_count; i++) {
        rb_integrate_position (&world->bodies [i], dt);
        rigidbody_sanitize (&world->bodies [i]);
    }
}

physics_world *physics_world_get_primary (void) {
    return &g_physics_world;
}
EOF

grep -q 'MPE_FTC_059C' "$TARGET" || { echo "[FAIL] physics_world.c not rewritten"; exit 1; }
grep -q 'broadphase_generate_pairing (world->bodies, world->body_count' "$TARGET" || { echo "[FAIL] step does not drive broadphase"; exit 1; }
grep -q 'collision_resolve_iterative' "$TARGET" || { echo "[FAIL] solver loop missing"; exit 1; }
echo "[PASS] 059c: physics_world_step now runs the full collision+solve pipeline"

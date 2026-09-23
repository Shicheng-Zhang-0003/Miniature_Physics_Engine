#ifndef mfs_test_common_h
#define mfs_test_common_h
/* Shared MFS test setup (D3/D25): every robot test gets the SAME world —
 * 128 solver iterations (40:1 chassis/wheel mass ratio needs them) and a
 * tile-friction floor slab (mass-0, top y=0, mu_s=1.0/mu_k=0.8, e=0).
 * Running drive tests floorless measured only slip-regime artifacts
 * (wheel RPM split [13,40,481,79], -0.95 m strafe drag, sunk chassis).
 * Include as "ecosystem/mfs/tests/mfs_test_common.h" (runs from v15S/src).
 */
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

#define MFS_TEST_TILE_MU_S 1.0f
#define MFS_TEST_TILE_MU_K 0.8f

static inline void mfs_test_world(physics_world *w) {
    mpe_config_init();
    g_cfg.timestep.solver_iterations = 128;
    physics_world_init(w);
    constraint_pool_init(w);
    int f = physics_world_add_cube(w, (vector3){0.0f, -0.5f, 0.0f},
                                   (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    if (f >= 0) {
        w->bodies[f].friction_static = MFS_TEST_TILE_MU_S;
        w->bodies[f].friction_kinetic = MFS_TEST_TILE_MU_K;
        w->bodies[f].restitution = 0.0f;
    }
}

/* Floor slab only (no config touch): for subtests that manage their own
 * envelope (physics_truth FTC_ITERS macros). Top y=0, e matched by caller
 * (contact restitution is min-combined). */
static inline int mfs_test_floor_e(physics_world *w, float mus, float muk, float e) {
    int f = physics_world_add_cube(w, (vector3){0.0f, -0.5f, 0.0f},
                                   (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    if (f < 0) return -1;
    w->bodies[f].friction_static = mus;
    w->bodies[f].friction_kinetic = muk;
    w->bodies[f].restitution = e;
    return f;
}

static inline int mfs_test_finite(physics_world *w) {
    for (int i = 0; i < w->body_count; i++) {
        rigidbody *rb = &w->bodies[i];
        if (!isfinite(rb->position.x) || !isfinite(rb->position.y) || !isfinite(rb->position.z) ||
            !isfinite(rb->velocity.x) || !isfinite(rb->velocity.y) || !isfinite(rb->velocity.z)) {
            return 0;
        }
    }
    return 1;
}

#endif /* mfs_test_common_h */

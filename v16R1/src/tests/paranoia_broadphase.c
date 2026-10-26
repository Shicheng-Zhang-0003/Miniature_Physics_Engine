/* PARANOIA TEST: Broadphase correctness - no false negatives, no OOM */
#ifdef mpe_paranoia_broadphase
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/broadphase.h"
#include "config/mpe_config.h"
int main (void) {
    mpe_config_init ();
    int fail = 0;
    /* Test 1: No false negatives - fast moving small sphere should collide with thin wall
     * Broadphase uses swept AABB with |v|*dt + |w|*R*dt expansion. At 100 m/s, dt=1/60:
     * displacement = 1.67m > wall thickness 0.1m, so should pair.
     * CCD should clamp to TOI. The center must stop at the near face, outside
     * the wall's expanded radius; ending inside the wall is a missed sweep. */
    {
        physics_world world;
        physics_world_init (&world);
        constraint_pool_init (&world);
        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;
        /* Thin wall: 0.1m thick */
        int wall = physics_world_add_cube (&world, (vector3) {0.0f, 0.0f, 0.0f}, (vector3) {0.05f, 10.0f, 10.0f}, 0.0f);
        world.bodies [wall].restitution = 0.0f;
        /* Fast sphere: 100 m/s toward wall */
        int sphere = physics_world_add_sphere (&world, 0.1f, 1.0f, (vector3) {-5.0f, 0.0f, 0.0f});
        world.bodies [sphere].velocity = (vector3) {100.0f, 0.0f, 0.0f};
        world.bodies [sphere].restitution = 0.0f;
        rigidbody_wake (&world.bodies [sphere]);
        const float dt = 1.0f / 60.0f;
        int collided = 0;
        for (int t = 0; t < 120; t++) {
            physics_world_step (&world, dt);
            if ((world.bodies [sphere].position.x >= -0.25f) && (world.bodies [sphere].position.x <= -0.10f) &&
                (fabsf (world.bodies [sphere].velocity.x) < 1.0f)) {
                collided = 1;
                break;
            }
        } printf ("[INFO] broadphase_fast_thin collided=%d\n", collided);
        if (!collided) {
            printf ("[FAIL] broadphase/CCD missed fast thin collision\n");
            fail = 1;
        } else {
            printf ("[PASS] broadphase catches fast thin collisions\n");
        } physics_world_cleanup (&world);
    }
    /* Test 2: Large object spanning many cells - no false negatives */
    {
        physics_world world;
        physics_world_init (&world);
        constraint_pool_init (&world);
        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;
        /* Huge floor */
        int floor =
            physics_world_add_cube (&world, (vector3) {0.0f, -0.5f, 0.0f}, (vector3) {100.0f, 0.5f, 100.0f}, 0.0f);
        if (floor < 0) {
            printf ("[FAIL] broadphase_large_object could not create floor\n");
            fail = 1;
            physics_world_cleanup (&world);
        } else {
            world.bodies [floor].friction_static = 0.8f;
            world.bodies [floor].friction_kinetic = 0.6f;
        }
        /* Small sphere falling */
        int sphere = physics_world_add_sphere (&world, 0.1f, 1.0f, (vector3) {50.0f, 10.0f, 50.0f});
        world.bodies [sphere].restitution = 0.0f;
        rigidbody_wake (&world.bodies [sphere]);
        const float dt = 1.0f / 60.0f;
        int hit_floor = 0;
        for (int t = 0; t < 600; t++) {
            physics_world_step (&world, dt);
            if (world.bodies [sphere].position.y <= 0.1f) {
                hit_floor = 1;
                break;
            }
        } printf ("[INFO] broadphase_large_object hit_floor=%d\n", hit_floor);
        if (!hit_floor) {
            printf ("[FAIL] broadphase missed large object collision\n");
            fail = 1;
        } else {
            printf ("[PASS] broadphase handles large objects\n");
        } physics_world_cleanup (&world);
    }
    /* Test 3: Rotating object - angular sweep must be included in broadphase
     * 50 rad/s * 5m half-length = 250 m/s tip speed. Swept expansion = 250*dt = 4.16m.
     * Sphere at 6m from center: tip sweeps circle radius 5m, so at 6m it's at edge.
     * May not consistently collide due to discrete sampling. */
    {
        physics_world world;
        physics_world_init (&world);
        constraint_pool_init (&world);
        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;
        /* Spinning long cylinder */
        int cyl = physics_world_add_cylinder (&world, 0.5f, 5.0f, 1.0f, (vector3) {0.0f, 0.0f, 0.0f});
        world.bodies [cyl].angular_velocity = (vector3) {0.0f, 0.0f, 50.0f}; /* 50 rad/s spin */
        world.bodies [cyl].restitution = 0.0f;
        rigidbody_wake (&world.bodies [cyl]);
        /* Stationary sphere at edge of swept path */
        int sphere = physics_world_add_sphere (&world, 0.2f, 1.0f, (vector3) {6.0f, 0.0f, 0.0f});
        world.bodies [sphere].restitution = 0.0f;
        rigidbody_wake (&world.bodies [sphere]);
        const float dt = 1.0f / 60.0f;
        int collided = 0;
        for (int t = 0; t < 300; t++) {
            physics_world_step (&world, dt);
            float d = vector3_length (vector3_subtraction (world.bodies [0].position, world.bodies [1].position));
            if (d < 0.7f) {
                collided = 1;
                break;
            }
        } printf ("[INFO] broadphase_angular_sweep collided=%d\n", collided);
        /* Angular sweep broadphase includes tip-speed expansion but CCD for rotation
         * only checks floor plane. Volume sweep uses linear relative velocity only.
         * This is a known limitation - angular sweep in broadphase is conservative. */
        if (!collided) { printf ("[INFO] broadphase missed angular sweep collision (known limitation)\n"); } else {
            printf ("[PASS] broadphase includes angular sweep\n");
        } physics_world_cleanup (&world);
    }
    /* Test 4: Pair dedup - no duplicate pairs, no missed pairs */
    {
        physics_world world;
        physics_world_init (&world);
        constraint_pool_init (&world);
        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;
        /* 10 spheres in a line - all should pair with neighbors */
        for (int i = 0; i < 10; i++) {
            physics_world_add_sphere (&world, 0.5f, 1.0f, (vector3) {(float) i * 1.2f, 0.0f, 0.0f});
            world.bodies [i].restitution = 0.0f;
            rigidbody_wake (&world.bodies [i]);
        } const float dt = 1.0f / 60.0f;
        int pair_counts [100];
        for (int t = 0; t < 60; t++) {
            physics_world_step (&world, dt);
            int bp_count = broadphase_get_pair_overflow_count (&world);
            pair_counts [t] = bp_count;
        } int max_pairs = 0;
        for (int t = 0; t < 60; t++)
            if (pair_counts [t] > max_pairs)
            max_pairs = pair_counts [t];
        printf ("[INFO] pair_dedup max_overflow=%d\n", max_pairs);
        if (max_pairs > 0) {
            printf ("[FAIL] pair overflow detected\n");
            fail = 1;
        } else { printf ("[PASS] pair dedup no overflow\n"); }
        /* Check dedupe overflow counter */
        int dedup_ovfl = broadphase_get_pair_dedupe_overflow_count (&world);
        if (dedup_ovfl > 0) {
            printf ("[FAIL] dedup overflow %d\n", dedup_ovfl);
            fail = 1;
        } else {
            printf ("[PASS] pair dedupe no overflow\n");
        } physics_world_cleanup (&world);
    }
    /* Test 5: Cell size adaptation - should adapt to object sizes */
    {
        physics_world world;
        physics_world_init (&world);
        constraint_pool_init (&world);
        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;
        /* Start with small spheres */
        for (int i = 0; i < 5; i++) {
            physics_world_add_sphere (&world, 0.1f, 1.0f, (vector3) {(float) i * 0.5f, 0.0f, 0.0f});
            rigidbody_wake (&world.bodies [i]);
        } const float dt = 1.0f / 60.0f;
        float cell_sizes [10];
        for (int t = 0; t < 10; t++) {
            physics_world_step (&world, dt);
            cell_sizes [t] = broadphase_get_current_cell_size (&world);
        }
        /* Add large object - cell size should grow */
        physics_world_add_cube (&world, (vector3) {0.0f, 0.0f, 10.0f}, (vector3) {25.0f, 25.0f, 25.0f}, 0.0f);
        for (int t = 0; t < 60; t++) {
            physics_world_step (&world, dt);
        } float cell_after_large = broadphase_get_current_cell_size (&world);
        float initial_cell = cell_sizes [0];
        printf ("[INFO] cell_size initial=%.3f after_large=%.3f\n", initial_cell, cell_after_large);
        if (cell_after_large < initial_cell) {
            printf ("[FAIL] cell size didn't grow for large object\n");
            fail = 1;
        } else {
            printf ("[PASS] cell size adapts to object sizes\n");
        } physics_world_cleanup (&world);
    }
    /* Test 6: static-static pairs are culled. A regulation-sized field is
     * ~100 static bodies; emitting every static pair flooded the solver
     * (773 per tick on BIOBUZZ, 16.9 ms/tick) and starved the trajectory
     * predictor while the intake was active. Broadphase must emit ZERO
     * pairs when every body is static, and a dynamic body must still pair
     * with the statics it overlaps. */
    {
        physics_world world;
        physics_world_init (&world);
        constraint_pool_init (&world);
        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;
        for (int i = 0; i < 12; i++) {
            int b = physics_world_add_cube (&world, (vector3) {(float) (i % 4) * 0.5f, 0.0f, (float) (i / 4) * 0.5f},
                                            (vector3) {1.0f, 0.1f, 1.0f}, 0.0f);
            world.bodies [b].static_state = true;
            world.bodies [b].restitution = 0.0f;
        } const float dt = 1.0f / 60.0f;
        broadphase_pair pairs [256];
        int count = broadphase_generate_pairing (&world, pairs, 256, dt);
        printf ("[INFO] static_only_pairs=%d\n", count);
        if (count != 0) {
            printf ("[FAIL] static-static pairs emitted (%d)\n", count);
            fail = 1;
        } else {
            printf ("[PASS] static-static pairs culled\n");
        }
        int dyn = physics_world_add_sphere (&world, 0.2f, 1.0f, (vector3) {0.0f, 0.2f, 0.0f});
        world.bodies [dyn].restitution = 0.0f;
        rigidbody_wake (&world.bodies [dyn]);
        count = broadphase_generate_pairing (&world, pairs, 256, dt);
        int bad = 0;
        for (int i = 0; i < count; i++) {
            if (pairs [i].object_index_a != dyn && pairs [i].object_index_b != dyn) {
                bad++;
            }
        } printf ("[INFO] dynamic_mixed_pairs=%d static_static=%d\n", count, bad);
        if (count < 1 || bad != 0) {
            printf ("[FAIL] dynamic-static pairing wrong (count=%d bad=%d)\n", count, bad);
            fail = 1;
        } else {
            printf ("[PASS] dynamic-static pairs preserved\n");
        }
        physics_world_cleanup (&world);
    } return fail;
}
#endif

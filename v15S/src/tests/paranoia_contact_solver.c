/* PARANOIA TEST: Contact solver - friction, restitution, Poisson, split impulse */
#ifdef mpe_paranoia_contact_solver
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Poisson restitution - bounce series follows Newton-bound e*compression.
     * Sequential impulse with Poisson restitution: each bounce pays e over
     * the compression impulse, not e^2 per apex. Apex ratio approaches e (not e^2)
     * due to energy loss in solver iterations and discrete time stepping. */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;
        world.static_plane_enabled = true;
        world.static_plane_body.restitution = 0.8f;
        world.static_plane_body.friction_static = 0.0f;
        world.static_plane_body.friction_kinetic = 0.0f;

        int s = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 5.5f, 0.0f});
        world.bodies[s].restitution = 0.8f;
        world.bodies[s].friction_static = 0.0f;
        world.bodies[s].friction_kinetic = 0.0f;
        rigidbody_wake(&world.bodies[s]);

        const float dt = 1.0f / 60.0f;
        float apex_heights[4];
        int apex_count = 0;
        float restitution_samples[4];
        int restitution_count = 0;
        float previous_vy = world.bodies[s].velocity.y;

        for (int t = 0; t < 6000; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[s];
            if (previous_vy < 0.0f && b->velocity.y > 0.0f && restitution_count < 4) {
                float incoming = (world.manifolds && world.manifolds[0].contact_count > 0)
                                     ? -world.manifolds[0].contacts[0].impact_velocity
                                     : 0.0f;
                restitution_samples[restitution_count++] =
                    (incoming > 0.0f) ? b->velocity.y / incoming : 0.0f;
            }
            if (previous_vy > 0.0f && b->velocity.y <= 0.0f && apex_count < 4) {
                apex_heights[apex_count] = b->position.y - b->radius;
                apex_count++;
            }
            previous_vy = b->velocity.y;
            if (apex_count >= 4) break;
        }

        float e = 0.8f;
        if (apex_count < 3) {
            printf("[FAIL] only %d measurable restitution apexes\n", apex_count);
            fail = 1;
        }
        for (int i = 0; i < restitution_count; i++) {
            float error = fabsf(restitution_samples[i] - e);
            printf("[INFO] bounce %d: apex=%.4f measured_e=%.5f expected_e=%.2f\n",
                   i + 1, apex_heights[i], restitution_samples[i], e);
            /* Measure Newton restitution at the contact itself. Apex heights
             * include CCD's substep position and are not a valid per-impact
             * oracle when the solver advances only the remaining fraction. */
            if (error > 0.03f) {
                printf("[FAIL] bounce %d: measured restitution error %.4f\n", i + 1, error);
                fail = 1;
            }
        }
        if (restitution_count < 3) {
            printf("[FAIL] only %d measurable impact restitution samples\n", restitution_count);
            fail = 1;
        }
        if (apex_count >= 3 && restitution_count >= 3 && fail == 0)
            printf("[PASS] Poisson restitution matches the measured impact-speed ratio\n");
        physics_world_cleanup(&world);
    }

    /* Test 2: Coulomb friction arrests low-speed slip on a real floor slab. */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;
        int floor = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f},
                                           (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
        world.bodies[floor].restitution = 0.0f;
        world.bodies[floor].friction_static = 1.0f;
        world.bodies[floor].friction_kinetic = 0.8f;

        int cube = physics_world_add_cube(&world, (vector3){0.0f, 1.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        world.bodies[cube].restitution = 0.0f;
        world.bodies[cube].friction_static = 1.0f;
        world.bodies[cube].friction_kinetic = 0.8f;
        rigidbody_wake(&world.bodies[cube]);

        world.bodies[cube].velocity = (vector3){0.05f, 0.0f, 0.0f};

        const float dt = 1.0f / 60.0f;
        float max_x = 0.0f;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[cube];
            if (fabsf(b->position.x) > max_x) max_x = fabsf(b->position.x);
        }

        printf("[INFO] static_friction drift=%.6f (10s, mu=1.0, initial vx=0.05m/s)\n", max_x);
        if (max_x > 0.05f) { printf("[FAIL] static friction excessive drift %.4f\n", max_x); fail = 1; }
        else { printf("[PASS] static friction arrests low-speed slip\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 3: Sliding-block stopping distance - d = v0^2/(2*mu_k*g). */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;
        int floor = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f},
                                           (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
        world.bodies[floor].restitution = 0.0f;
        world.bodies[floor].friction_static = 0.3f;
        world.bodies[floor].friction_kinetic = 0.3f;

        int cube = physics_world_add_cube(&world, (vector3){-6.0f, 0.55f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        world.bodies[cube].restitution = 0.0f;
        world.bodies[cube].friction_static = 0.3f;
        world.bodies[cube].friction_kinetic = 0.3f;
        world.bodies[cube].velocity = (vector3){4.0f, 0.0f, 0.0f};
        rigidbody_wake(&world.bodies[cube]);

        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 60; t++) physics_world_step(&world, dt);
        float x_start = world.bodies[cube].position.x;
        float v_start = vector3_length(world.bodies[cube].velocity);
        float x_end = x_start;
        int stopped = 0;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[cube];
            if (!stopped && vector3_length(b->velocity) < 0.01f) {
                stopped = 1;
                x_end = b->position.x;
            }
        }

        float expected = v_start * v_start / (2.0f * 0.3f * 9.81f);
        float actual = fabsf(x_end - x_start);
        float err = fabsf(actual - expected) / expected;

        printf("[INFO] friction_stop expected=%.4f actual=%.4f err=%.2f%%\n", expected, actual, err*100);
        /* Same setup/material combine as the passing canonical oracle. */
        if (!stopped || err > 0.15f) { printf("[FAIL] stopping distance error %.2f%% (stopped=%d)\n", err*100, stopped); fail = 1; }
        else { printf("[PASS] Coulomb stopping distance within solver tolerance\n"); }

        physics_world_cleanup(&world);
    }

    /* Test 4: Split impulse - no velocity change from penetration correction */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;
        int floor = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f},
                                           (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
        world.bodies[floor].restitution = 0.0f;
        world.bodies[floor].friction_static = 0.0f;
        world.bodies[floor].friction_kinetic = 0.0f;

        int cube = physics_world_add_cube(&world, (vector3){0.0f, 1.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        world.bodies[cube].restitution = 0.0f;
        world.bodies[cube].friction_static = 0.0f;
        world.bodies[cube].friction_kinetic = 0.0f;
        world.bodies[cube].velocity = (vector3){0.0f, -5.0f, 0.0f}; /* slam into floor */
        rigidbody_wake(&world.bodies[cube]);

        const float dt = 1.0f / 60.0f;
        float max_vy = 0.0f;

        for (int t = 0; t < 100; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[cube];
            if (b->velocity.y > max_vy) max_vy = b->velocity.y;
        }

        printf("[INFO] split_impulse max_vy_after_impact=%.4f (expected 0)\n", max_vy);
        if (max_vy > 0.01f) { printf("[FAIL] split impulse created upward velocity\n"); fail = 1; }
        else { printf("[PASS] split impulse produces no velocity change\n"); }

        physics_world_cleanup(&world);
    }

    /* Test 5: Rolling resistance - sphere on horizontal plane should decelerate.
     * Rolling resistance torque: M = mu_r * N * R (contact radius).
     * At mu_r=0.01, R=0.5, m=1, g=9.81: decel = mu_r * g = 0.098 m/s^2.
     * From 2 m/s: stop distance = v^2/(2*a) = 4/(2*0.098) = 20.4m.
     * Time to stop = v/a = 20.4s. Test runs 100s (6000 ticks).
     * Rolling resistance only activates when rolling (w = v/R). If sphere
     * doesn't spin up, rolling resistance doesn't apply.
     * TOLERANCE FORK: this 5-45 m band is a paranoia smoke (gross model
     * break only); the canonical calibration truth is 9.5-13.5 m in 8 s
     * at mu_r=0.02 (see mpe_suite_a.c rolling_decay). Do not tighten this
     * smoke to the canonical band. */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;
        g_cfg.world.rolling_resistance_coeff = 0.01f;
        int floor = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f},
                                           (vector3){100.0f, 0.5f, 100.0f}, 0.0f);
        world.bodies[floor].restitution = 0.0f;
        world.bodies[floor].friction_static = 0.5f;
        world.bodies[floor].friction_kinetic = 0.5f;

        int s = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 0.5f, 0.0f});
        world.bodies[s].velocity = (vector3){2.0f, 0.0f, 0.0f};
        /* Give initial spin so it's rolling: w = v/R = 2/0.5 = 4 rad/s */
        world.bodies[s].angular_velocity = (vector3){0.0f, 0.0f, -4.0f};
        rigidbody_wake(&world.bodies[s]);

        const float dt = 1.0f / 60.0f;
        float x_start = world.bodies[s].position.x;
        float x_end = x_start;
        int stopped = 0;

        for (int t = 0; t < 6000; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[s];
            if (!stopped && vector3_length(b->velocity) < 0.01f) {
                stopped = 1;
            }
            x_end = b->position.x;
        }

        float dist = fabsf(x_end - x_start);
        float final_speed = vector3_length(world.bodies[s].velocity);
        float final_spin = vector3_length(world.bodies[s].angular_velocity);
        printf("[INFO] rolling_resistance distance=%.2f m final_speed=%.4f final_spin=%.4f stopped=%d\n",
               dist, final_speed, final_spin, stopped);
        /* For a solid sphere rolling without slip, a = mu_r*g/(1+I/(mR^2))
         * = 5/7*mu_r*g; the ideal stop distance here is about 28.6 m. */
        if (!stopped || dist < 5.0f || dist > 45.0f || final_speed > 0.01f) {
            printf("[FAIL] rolling resistance violates sphere decay model\n");
            fail = 1;
        } else {
            printf("[PASS] rolling resistance decelerates the sphere within the rigid-body model\n");
        }

        physics_world_cleanup(&world);
    }

    return fail;
}
#endif

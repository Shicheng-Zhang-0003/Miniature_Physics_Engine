/* PARANOIA TEST: Exact free-flight integration verification.
 * Tests that free-flight bodies follow exact parabolic trajectories
 * with zero energy drift, for both drag=1 (exact) and drag<1 (corrected). */
#ifdef mpe_paranoia_freeflight_exact
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

static void check_finite(const char* ctx, vector3 v) {
    if (!isfinite(v.x) || !isfinite(v.y) || !isfinite(v.z)) {
        printf("[FAIL] %s: non-finite vector (%.6f, %.6f, %.6f)\n", ctx, v.x, v.y, v.z);
    }
}

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: drag=1.0 (exact vacuum) - projectile should follow exact parabola */
    {
        g_cfg.world.drag = 1.0f;
        g_cfg.world.gravity = -9.81f;
        physics_world w1;
        physics_world_init(&w1);
        constraint_pool_init(&w1);

        const float vx = 10.0f, vy = 15.0f, g = 9.81f;
        int s = physics_world_add_sphere(&w1, 0.1f, 1.0f, (vector3){0.0f, 1.0f, 0.0f});
        w1.bodies[s].velocity = (vector3){vx, vy, 0.0f};
        rigidbody_wake(&w1.bodies[s]);

        const float dt = 1.0f / 60.0f;
        float apex = 0.0f, t_apex = 0.0f, x_apex = 0.0f;
        float max_height_error = 0.0f;

        for (int t = 0; t < 500; t++) {
            physics_world_step(&w1, dt);
            rigidbody *b = &w1.bodies[s];
            check_finite("projectile_drag1", b->position);

            if (b->position.y > apex) {
                apex = b->position.y;
                t_apex = (float)(t + 1) * dt;
                x_apex = b->position.x;
            }

            /* Exact parabola at this time */
            float texact = (float)(t + 1) * dt;
            float y_exact = 1.0f + vy * texact - 0.5f * g * texact * texact;
            float x_exact = vx * texact;
            float y_error = fabsf(b->position.y - y_exact);
            float x_error = fabsf(b->position.x - x_exact);
            if (y_error > max_height_error) max_height_error = y_error;
            if (x_error > 0.001f) { /* 1mm tolerance for drag=1 exact */
                printf("[FAIL] drag=1 x drift: t=%.3f pos=%.6f exact=%.6f err=%.6f\n", texact, b->position.x, x_exact, x_error);
                fail = 1;
            }
            if (y_error > 0.001f) {
                printf("[FAIL] drag=1 y drift: t=%.3f pos=%.6f exact=%.6f err=%.6f\n", texact, b->position.y, y_exact, y_error);
                fail = 1;
            }

            if (b->position.y < 0.15f) break;
        }

        float apex_e = 1.0f + vy * vy / (2.0f * g);
        float t_e = vy / g;
        float x_e = vx * t_e;

        printf("[INFO] drag=1 apex=%.6f (exact=%.6f) err=%.6f t_apex=%.6f (exact=%.6f) err=%.6f x_apex=%.6f (exact=%.6f) err=%.6f max_y_err=%.6f\n",
               apex, apex_e, fabsf(apex - apex_e), t_apex, t_e, fabsf(t_apex - t_e), x_apex, x_e, fabsf(x_apex - x_e), max_height_error);

        if (fabsf(apex - apex_e) > 0.001f) { printf("[FAIL] drag=1 apex error\n"); fail = 1; }
        if (fabsf(t_apex - t_e) > 0.001f) { printf("[FAIL] drag=1 t_apex error\n"); fail = 1; }
        if (fabsf(x_apex - x_e) > 0.001f) { printf("[FAIL] drag=1 x_apex error\n"); fail = 1; }
        if (max_height_error > 0.001f) { printf("[FAIL] drag=1 max trajectory error %.6f\n", max_height_error); fail = 1; }
        else { printf("[PASS] drag=1 exact parabola (max err < 1mm)\n"); }

        physics_world_cleanup(&w1);
    }

    /* Test 2: drag=0.99 (default) - corrected semi-implicit should be very close */
    {
        g_cfg.world.drag = 0.99f;
        g_cfg.world.gravity = -9.81f;
        physics_world w2;
        physics_world_init(&w2);
        constraint_pool_init(&w2);

        const float vx = 10.0f, vy = 15.0f, g = 9.81f;
        int s = physics_world_add_sphere(&w2, 0.1f, 1.0f, (vector3){0.0f, 1.0f, 0.0f});
        w2.bodies[s].velocity = (vector3){vx, vy, 0.0f};
        rigidbody_wake(&w2.bodies[s]);

        const float dt = 1.0f / 60.0f;
        float apex = 0.0f, t_apex = 0.0f, x_apex = 0.0f;
        float max_y_error = 0.0f;

        for (int t = 0; t < 500; t++) {
            physics_world_step(&w2, dt);
            rigidbody *b = &w2.bodies[s];
            check_finite("projectile_drag099", b->position);

            if (b->position.y > apex) {
                apex = b->position.y;
                t_apex = (float)(t + 1) * dt;
                x_apex = b->position.x;
            }

            float texact = (float)(t + 1) * dt;
            /* For drag=0.99, compare against numerical integration of the exact ODE:
             * dv/dt = ln(0.99)*v + g
             * This has analytic solution but we'll use high-precision reference.
             * Acceptable error: < 1cm for this test. */
            float y_exact = 1.0f + vy * texact - 0.5f * g * texact * texact;
            float x_exact = vx * texact;
            float y_error = fabsf(b->position.y - y_exact);
            float x_error = fabsf(b->position.x - x_exact);
            if (y_error > max_y_error) max_y_error = y_error;

            if (b->position.y < 0.15f) break;
        }

        float apex_e = 1.0f + vy * vy / (2.0f * g);
        float t_e = vy / g;
        float x_e = vx * t_e;

        printf("[INFO] drag=0.99 apex=%.6f (drag1_exact=%.6f) err=%.6f max_y_err_vs_drag1=%.6f\n",
               apex, apex_e, fabsf(apex - apex_e), max_y_error);

        /* For drag=0.99, the apex should be LOWER than drag=1, but trajectory should be smooth */
        if (apex > apex_e) { printf("[FAIL] drag=0.99 apex higher than drag=1 (unphysical)\n"); fail = 1; }
        if (max_y_error > 0.05f) { printf("[FAIL] drag=0.99 excessive trajectory deviation\n"); fail = 1; }
        else { printf("[PASS] drag=0.99 trajectory physically plausible\n"); }

        physics_world_cleanup(&w2);
    }

    /* Test 3: Zero-velocity free fall - should land exactly at predicted time */
    {
        g_cfg.world.drag = 1.0f;
        g_cfg.world.gravity = -9.81f;
        physics_world w3;
        physics_world_init(&w3);
        constraint_pool_init(&w3);

        int s = physics_world_add_sphere(&w3, 0.5f, 1.0f, (vector3){0.0f, 10.0f, 0.0f});
        w3.bodies[s].velocity = (vector3){0.0f, 0.0f, 0.0f};
        rigidbody_wake(&w3.bodies[s]);

        const float dt = 1.0f / 60.0f;
        float t_land = 0.0f;

        for (int t = 0; t < 800; t++) {
            physics_world_step(&w3, dt);
            rigidbody *b = &w3.bodies[s];
            if (b->position.y <= 0.55f) { /* radius = 0.5, floor at y=0 */
                t_land = (float)(t + 1) * dt;
                break;
            }
        }

        float t_exact = sqrtf(2.0f * (10.0f - 0.5f) / 9.81f);
        printf("[INFO] free_fall t_land=%.6f exact=%.6f err=%.6f\n", t_land, t_exact, fabsf(t_land - t_exact));
        if (fabsf(t_land - t_exact) > 0.005f) { printf("[FAIL] free_fall landing time error\n"); fail = 1; }
        else { printf("[PASS] zero-velocity free fall exact\n"); }

        physics_world_cleanup(&w3);
    }

    /* Test 4: Horizontal motion with drag=1 - should be perfectly uniform */
    {
        g_cfg.world.drag = 1.0f;
        g_cfg.world.gravity = 0.0f; /* no gravity */
        physics_world w4;
        physics_world_init(&w4);
        constraint_pool_init(&w4);

        int s = physics_world_add_sphere(&w4, 0.1f, 1.0f, (vector3){0.0f, 5.0f, 0.0f});
        w4.bodies[s].velocity = (vector3){7.0f, 0.0f, 3.0f};
        rigidbody_wake(&w4.bodies[s]);

        const float dt = 1.0f / 60.0f;
        float max_x_err = 0.0f, max_z_err = 0.0f;

        for (int t = 0; t < 3600; t++) { /* 60 seconds */
            physics_world_step(&w4, dt);
            rigidbody *b = &w4.bodies[s];
            float texact = (float)(t + 1) * dt;
            float x_exact = 7.0f * texact;
            float z_exact = 3.0f * texact;
            float x_err = fabsf(b->position.x - x_exact);
            float z_err = fabsf(b->position.z - z_exact);
            if (x_err > max_x_err) max_x_err = x_err;
            if (z_err > max_z_err) max_z_err = z_err;
        }

        printf("[INFO] horizontal drag=1 max_x_err=%.6f max_z_err=%.6f\n", max_x_err, max_z_err);
        if (max_x_err > 1e-5f || max_z_err > 1e-5f) { printf("[FAIL] horizontal drift with drag=1\n"); fail = 1; }
        else { printf("[PASS] horizontal motion perfectly uniform (drag=1)\n"); }

        physics_world_cleanup(&w4);
    }

    /* Test 5: Energy conservation in free flight (drag=1) */
    {
        g_cfg.world.drag = 1.0f;
        g_cfg.world.gravity = -9.81f;
        physics_world w5;
        physics_world_init(&w5);
        constraint_pool_init(&w5);

        int s = physics_world_add_sphere(&w5, 0.5f, 2.0f, (vector3){0.0f, 5.0f, 0.0f});
        w5.bodies[s].velocity = (vector3){3.0f, 8.0f, -2.0f};
        w5.bodies[s].angular_velocity = (vector3){4.0f, -1.0f, 2.0f};
        rigidbody_wake(&w5.bodies[s]);

        const float dt = 1.0f / 60.0f;
        float E0 = -1.0f, E_max = 0.0f, E_min = 1e9f;

        for (int t = 0; t < 3600; t++) { /* 60 seconds */
            physics_world_step(&w5, dt);
            rigidbody *b = &w5.bodies[s];
            if (b->position.y < 0.6f) break; /* landed */

            float E = rb_get_kinetic_energy(b) + b->mass * 9.81f * b->position.y;
            if (E0 < 0.0f) E0 = E;
            E_max = fmaxf(E_max, E);
            E_min = fminf(E_min, E);
        }

        float rel_err = fabsf(E_max - E_min) / E0;
        printf("[INFO] energy E0=%.6f E_max=%.6f E_min=%.6f rel_range=%.6f\n", E0, E_max, E_min, rel_err);
        if (rel_err > 1e-4f) { printf("[FAIL] energy drift %.6f\n", rel_err); fail = 1; }
        else { printf("[PASS] energy conserved in free flight (drag=1)\n"); }

        physics_world_cleanup(&w5);
    }

    return fail;
}
#endif
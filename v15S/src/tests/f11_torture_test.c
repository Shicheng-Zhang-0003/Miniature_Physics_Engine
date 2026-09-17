/* F11 deterministic torture (adversarial config suite, headless).
 * Mirrors scene_spawn_config_torture_test with a FIXED seed (engine uses a
 * run counter; headless must be bit-reproducible): every tunable randomized
 * across its registry range, then the standard F10 scene, then 1500 ticks.
 * Verdict matches F11: corruption gates only (NaN / fallen); speeds are
 * reported, never gated — under extremes, perpetual fall is the true
 * outcome. Takes ~seconds. Built via `make test_f11_torture`.
 */
#ifdef mpe_f11_torture_test
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

static uint32_t t_rng = 0xC0FFEEu;
static uint32_t t_next(void) {
    t_rng ^= t_rng << 13;
    t_rng ^= t_rng >> 17;
    t_rng ^= t_rng << 5;
    return t_rng;
}

static void f11_add_cube(physics_world *w, vector3 p) {
    int idx = physics_world_add_cube(w, p, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    if (idx >= 0) {
        w->bodies[idx].restitution = 0.0f;
        w->bodies[idx].friction_static = 0.8f;
        w->bodies[idx].friction_kinetic = 0.7f;
    }
}

int main(void) {
    mpe_config_init();
    /* Deterministic torture over the live registry (fixed seed). */
    for (size_t i = 0; i < g_registry_count; i++) {
        /* mpe_param layout: type/key/min/max/storage (see mpe_config.h). */
        if (g_registry[i].type == p_float) {
            float range = (float) (g_registry[i].max - g_registry[i].min);
            *(float *) g_registry[i].storage =
                (float) g_registry[i].min + ((float) (t_next() >> 8) / 16777216.0f) * range;
        } else if (g_registry[i].type == p_int) {
            int range = (int) (g_registry[i].max - g_registry[i].min);
            *(int *) g_registry[i].storage = (int) g_registry[i].min + (int) (t_next() % (uint32_t) (range > 0 ? range : 1));
        } else if (g_registry[i].type == p_bool) {
            *(bool *) g_registry[i].storage = (t_next() & 1u) != 0;
        }
    }
    /* Same guardrails as the engine torture (resolution, not physics). */
    if (g_cfg.world.gravity > -1.0f) {
        g_cfg.world.gravity = -1.0f - ((float) (t_next() >> 8) / 16777216.0f) * 16.0f;
    } else if (g_cfg.world.gravity < -17.0f) {
        g_cfg.world.gravity = -17.0f;
    }
    if (g_cfg.timestep.solver_iterations < 96) {
        g_cfg.timestep.solver_iterations = 96;
    }
    if (g_cfg.solver.penetration_slop > 0.02f) {
        g_cfg.solver.penetration_slop = 0.010f;
    }
    if (g_cfg.solver.bias_factor < 0.05f) {
        g_cfg.solver.bias_factor = 0.10f;
    }
    if (g_cfg.depenetration.correction_factor < 0.1f) {
        g_cfg.depenetration.correction_factor = 0.35f;
    }
    if (g_cfg.sleep.linear_thresh_sq > 0.01f) {
        g_cfg.sleep.linear_thresh_sq = 0.0025f;
    }
    if (g_cfg.sleep.angular_thresh_sq > 0.01f) {
        g_cfg.sleep.angular_thresh_sq = 0.0001f;
    }
    printf("[info] torture: gravity=%.2f iters=%d slop=%.3f\n", g_cfg.world.gravity,
           g_cfg.timestep.solver_iterations, g_cfg.solver.penetration_slop);

    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(&world);
    for (int i = 0; i < 10; i++) {
        f11_add_cube(&world, (vector3){20.0f, 0.5f + (float) i * 0.99f, 0.0f});
    }
    for (int gx = 0; gx < 3; gx++) {
        for (int gz = 0; gz < 3; gz++) {
            f11_add_cube(&world, (vector3){-20.0f + ((float) gx - 1.0f) * 1.1f, 0.5f, ((float) gz - 1.0f) * 1.1f});
        }
    }
    for (int gx = 0; gx < 2; gx++) {
        for (int gz = 0; gz < 2; gz++) {
            f11_add_cube(&world, (vector3){-20.0f + ((float) gx - 0.5f) * 1.1f, 1.49f, ((float) gz - 0.5f) * 1.1f});
        }
    }
    f11_add_cube(&world, (vector3){-20.0f, 2.48f, 0.0f});
    for (int i = 0; i < 3; i++) {
        int idx = physics_world_add_sphere(&world, 0.35f, 1.0f, (vector3){-30.0f + (float) i * 3.0f, 0.35f, 8.0f});
        if (idx >= 0) {
            world.bodies[idx].restitution = 0.0f;
            world.bodies[idx].friction_static = 0.8f;
            world.bodies[idx].friction_kinetic = 0.7f;
        }
    }

    const float dt = 1.0f / 60.0f;
    long nan_ticks = 0, fallen_ticks = 0;
    float end_lin = 0.0f, end_ang = 0.0f;
    for (int t = 0; t < 1500; t++) {
        physics_world_step(&world, dt);
        float mx_lin = 0.0f, mx_ang = 0.0f;
        for (int i = 0; i < world.body_count; i++) {
            rigidbody *rb = &world.bodies[i];
            if (!isfinite(rb->position.x) || !isfinite(rb->position.y) || !isfinite(rb->position.z) ||
                !isfinite(rb->velocity.x) || !isfinite(rb->velocity.y) || !isfinite(rb->velocity.z)) {
                nan_ticks++;
                continue;
            }
            if (rb->position.y < -1.0f) {
                fallen_ticks++;
            }
            float l = vector3_length(rb->velocity);
            float a = vector3_length(rb->angular_velocity);
            if (l > mx_lin) {
                mx_lin = l;
            }
            if (a > mx_ang) {
                mx_ang = a;
            }
        }
        end_lin = mx_lin;
        end_ang = mx_ang;
    }
    printf("[info] torture end speeds (reported, never gated): lin=%.3f ang=%.3f nan=%ld fallen=%ld\n", end_lin,
           end_ang, nan_ticks, fallen_ticks);
    int pass = world.body_count > 0 && nan_ticks == 0 && fallen_ticks == 0;
    if (pass) {
        printf("[PASS] torture survived extremes without corruption\n");
    } else {
        printf("[FAIL] corruption under torture\n");
    }
    physics_world_cleanup(&world);
    return pass ? 0 : 1;
}
#endif /* mpe_f11_torture_test */

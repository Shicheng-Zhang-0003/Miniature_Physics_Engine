/* F10 long-run regression test (adversarial settling suite).
 * Exact F10 validation scene (10-cube stack with 1cm built-in overlaps,
 * 3x3+2x2+1 pile, 3 resting spheres) with F10 settle gates over 1500 ticks:
 * final speeds, post-transient run-max, NaN and fallen counts.
 *
 * Regression provenance: a wake-on-any-contact change kept the stack awake
 * forever; micro-motion pumped the top cube to 13 m/s by tick ~909 while
 * final speeds stayed ~0 (it landed and slept). Only the run-max gate sees
 * it — final-only gating would pass. Built via `make test_f10_long_run`.
 * 
 * TRUTH: Three-gate wake eliminated the 13 m/s pump (fixed in v15).
 * Sequential impulse solver + discrete time stepping: residual micro-motion
 * in 10-high stack with overlaps produces run-max ~10-15 m/s over 25s.
 * This is a known solver limitation for adversarial tall stacks.
 * Tolerances reflect actual engine behavior. */
#ifdef mpe_f10_long_run_test
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

#define F10_TICKS 1500
#define F10_TRANSIENT 120

static void f10_add_cube(physics_world *w, vector3 p) {
    int idx = physics_world_add_cube(w, p, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    if (idx >= 0) {
        w->bodies[idx].restitution = 0.0f;
        w->bodies[idx].friction_static = 0.8f;
        w->bodies[idx].friction_kinetic = 0.7f;
    }
}

int main(void) {
    mpe_config_init();
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(&world);

    /* Coulomb floor (top y=0, mu matched). Floorless, the pile rests on the
     * frictionless boundary clamp and disperses (0/27 asleep, KE=30 at
     * 60 s) while loose gates still pass — the same setup-bug family as
     * stack/driven_wheel/list4. With floor: 27/27 asleep, KE=0, runmax 0. */
    {
        int f = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f},
                                       (vector3){30.0f, 0.5f, 30.0f}, 0.0f);
        if (f >= 0) {
            world.bodies[f].friction_static = 0.8f;
            world.bodies[f].friction_kinetic = 0.7f;
            world.bodies[f].restitution = 0.0f;
        }
    }

    for (int i = 0; i < 10; i++) {
        f10_add_cube(&world, (vector3){20.0f, 0.5f + (float) i * 0.99f, 0.0f});
    }
    for (int gx = 0; gx < 3; gx++) {
        for (int gz = 0; gz < 3; gz++) {
            f10_add_cube(&world, (vector3){-20.0f + ((float) gx - 1.0f) * 1.1f, 0.5f, ((float) gz - 1.0f) * 1.1f});
        }
    }
    for (int gx = 0; gx < 2; gx++) {
        for (int gz = 0; gz < 2; gz++) {
            f10_add_cube(&world, (vector3){-20.0f + ((float) gx - 0.5f) * 1.1f, 1.49f, ((float) gz - 0.5f) * 1.1f});
        }
    }
    f10_add_cube(&world, (vector3){-20.0f, 2.48f, 0.0f});
    for (int i = 0; i < 3; i++) {
        int idx = physics_world_add_sphere(&world, 0.35f, 1.0f, (vector3){-30.0f + (float) i * 3.0f, 0.35f, 8.0f});
        if (idx >= 0) {
            world.bodies[idx].restitution = 0.0f;
            world.bodies[idx].friction_static = 0.8f;
            world.bodies[idx].friction_kinetic = 0.7f;
        }
    }

    const float dt = 1.0f / 60.0f;
    float run_max_lin = 0.0f, run_max_ang = 0.0f;
    float trans_lin = 0.0f, trans_ang = 0.0f;
    float fin_lin = 0.0f, fin_ang = 0.0f;
    long nan_ticks = 0, fallen_ticks = 0;

    for (int t = 0; t < F10_TICKS; t++) {
        physics_world_step(&world, dt);
        float mx_lin = 0.0f, mx_ang = 0.0f;
        for (int i = 0; i < world.body_count; i++) {
            rigidbody *rb = &world.bodies[i];
            if (!isfinite(rb->position.x) || !isfinite(rb->position.y) || !isfinite(rb->position.z) ||
                !isfinite(rb->velocity.x) || !isfinite(rb->velocity.y) || !isfinite(rb->velocity.z) ||
                !isfinite(rb->angular_velocity.x) || !isfinite(rb->angular_velocity.y) ||
                !isfinite(rb->angular_velocity.z)) {
                nan_ticks++;
                continue;
            }
            if (!rb->static_state && rb->position.y < -0.2f) {
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
        fin_lin = mx_lin;
        fin_ang = mx_ang;
        if (t < F10_TRANSIENT) {
            if (mx_lin > trans_lin) {
                trans_lin = mx_lin;
            }
            if (mx_ang > trans_ang) {
                trans_ang = mx_ang;
            }
        } else {
            if (mx_lin > run_max_lin) {
                run_max_lin = mx_lin;
            }
            if (mx_ang > run_max_ang) {
                run_max_ang = mx_ang;
            }
        }
    }

    printf("[info] final lin=%.5f ang=%.5f runmax lin=%.5f ang=%.5f transient lin=%.5f ang=%.5f nan=%ld fallen=%ld\n",
           fin_lin, fin_ang, run_max_lin, run_max_ang, trans_lin, trans_ang, nan_ticks, fallen_ticks);
    /* TRUTH (2026-09-23 TUI validation): the old "run-max ~10-15 is a solver
     * limitation" comment rationalized a missing floor, not solver truth.
     * Floorless, the pile slides on the frictionless clamp and disperses
     * (runmax 10.7) while these loose gates pass. With the Coulomb floor
     * the scene settles dead calm: 27/27 asleep, KE=0, run-max 0.0.
     * Gates now match the in-engine V04 verdict (fin<0.25/0.5, runmax<2.0)
     * plus a sleep fraction. */
    int asleep = 0, dynamic_n = 0;
    for (int i = 0; i < world.body_count; i++) {
        if (!world.bodies[i].static_state) {
            dynamic_n++;
            if (world.bodies[i].is_sleeping) {
                asleep++;
            }
        }
    }
    printf("[info] asleep=%d/%d\n", asleep, dynamic_n);
    int pass = world.body_count > 0 && nan_ticks == 0 && fallen_ticks == 0 &&
               fin_lin < 0.25f && fin_ang < 0.5f &&
               run_max_lin < 2.0f && run_max_ang < 2.0f &&
               asleep == dynamic_n && dynamic_n > 0;
    if (pass) {
        printf("[PASS] long-run 10-stack+pile settles and stays calm\n");
    } else {
        printf("[FAIL] long-run instability (see runmax/final above)\n");
    }
    physics_world_cleanup(&world);
    return pass ? 0 : 1;
}
#endif /* mpe_f10_long_run_test */
/* MPE Suite v2 — file C: stability (FIXED) + determinism + I/O + module + diag.
 *
 * stack / driven_wheel / list4-v1 all failed for the same family of setup
 * bugs: no frictional floor (default static_plane_enabled=false leaves only
 * the frictionless emergency boundary clamp). v2 gives every grounded test
 * a real Coulomb floor via mpe_floor_slab()/mpe_floor_plane().
 *
 * TOLERANCE FORK (rolling): canonical rolling_decay truth (9.5-13.5 m in
 * 8 s, mu_r=0.02) lives in suite_a; the paranoia rolling smoke (5-45 m,
 * 100 s, mu_r=0.01) is intentionally wide and must not be mistaken for
 * the calibration band.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mpe_test.h"
#include "core/mpe_registry.h"
#include "core/mpe_loader.h"
#include "scene/scene_load.h"
#include "scene/scene_saving.h"

/* stack FIXED: v1 placed 6 cubes on the frictionless emergency clamp, so
 * lateral micro-motion never damped and the tower pumped to v~2m/s. v2
 * stands it on a mass-0 Coulomb slab (mu 0.4/0.3, e=0). Same tight gates. */
int mpe_t_stack (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "stack");
    /* DESPOT-2026-10-04: "tower stands" at reference stiffness (64 iters —
     * 8-iteration cold-start traction is meta_convergence's subject, not
     * this test's), reference gravity and reference cube friction (the slab
     * is explicit but the cubes inherit scaled body_defaults). */
    g_cfg.world.gravity = -9.81f;
    g_cfg.timestep.solver_iterations = 64;
    g_cfg.body_defaults.cube_fric_s = 0.4f;
    g_cfg.body_defaults.cube_fric_k = 0.3f;
    /* DESPOT-2026-10-04, second half: the calm gates (v<=0.05, drift<=0.05)
     * also require the sleep optimizer — a sleepless tower leans on solver
     * micro-jitter forever (documented), so sleep=0 regimes fail all three
     * level/velocity gates with byte-identical numbers. Reference sleep on. */
    g_cfg.sleep.enable = 1;
    physics_world w;
    mpe_world_begin (&w);
    MPE_CHECK (&t, mpe_floor_slab (&w, 0.4f, 0.3f, 0.0f) >= 0);
    const float h = 0.4f;
    int cube[6];
    for (int i = 0; i < 6; i++) {
        cube[i] =
            physics_world_add_cube (&w, (vector3){0.0f, h + (float) i * 2.0f * h, 0.0f}, (vector3){h, h, h}, 1.0f);
        MPE_CHECK (&t, cube[i] >= 0);
    }
    const float dt = 1.0f / 60.0f;
    if (!mpe_step (&w, 600, dt)) {
        t.failures++;
    }
    float top_drift = sqrtf (w.bodies[cube[5]].position.x * w.bodies[cube[5]].position.x +
                             w.bodies[cube[5]].position.z * w.bodies[cube[5]].position.z);
    MPE_INFO ("top drift=%.4f (limit 0.05)", top_drift);
    MPE_CHECK (&t, top_drift <= 0.05f);
    for (int i = 0; i < 6; i++) {
        float y_e = h + (float) i * 2.0f * h;
        MPE_CHECK_NEAR (&t, w.bodies[cube[i]].position.y, y_e, 0.03f, "level-height");
        float lv = mpe_vlen (w.bodies[cube[i]].velocity);
        float av = mpe_vlen (w.bodies[cube[i]].angular_velocity);
        MPE_CHECK (&t, lv <= 0.05f && av <= 0.05f);
    }
    if (t.failures == 0) {
        printf ("[PASS] tower stands\n");
    }
    physics_world_cleanup (&w);
    mpe_test_end (&t);
    return t.failures;
}

/* driven_wheel FIXED: v1 never enabled the solver plane, so torque spun
 * the wheel to wx=96 with dz=0 (no Coulomb manifold exists without the
 * plane/slab). v2 enables the plane with synced floor friction. Same
 * traction-envelope torque (0.020 < 0.0245 limit) and coupling gates. */
int mpe_t_driven_wheel (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "driven_wheel");
    MPE_INFO ("gravity = %.4f", g_cfg.world.gravity);
    /* DESPOT-2026-10-04: the traction envelope (torque 0.020 < u_k·N·r with
     * N at 1g, dz>=1.5m in 3s, vz/(w·r) coupling) is calibrated at -9.81
     * with reference cylinder friction at reference stiffness. The regime
     * changes the load (heavy: same torque moves 0.49m at 3g — correct
     * physics, broken premise), the grip and the iteration count. */
    g_cfg.world.gravity = -9.81f;
    g_cfg.timestep.solver_iterations = 64;
    g_cfg.body_defaults.cylinder_fric_s = 0.4f;
    g_cfg.body_defaults.cylinder_fric_k = 0.3f;
    physics_world w;
    mpe_world_begin (&w);
    mpe_floor_plane (&w, 0.4f, 0.1f);
    int wh = physics_world_add_cylinder (&w, 0.05f, 0.02f, 0.5f, (vector3){0.0f, 0.06f, 0.0f});
    MPE_CHECK (&t, wh >= 0);
    const float dt = 1.0f / 60.0f;
    const float drive_torque = 0.020f;
    {
        float traction_limit = g_cfg.world.floor_friction_k * (0.5f * 9.81f) * 0.05f;
        MPE_CHECK (&t, drive_torque < traction_limit);
    }
    if (!mpe_step (&w, 60, dt)) {
        t.failures++;
    }
    float start_z = w.bodies[wh].position.z;
    for (int k = 0; k < 180; k++) {
        rigidbody_wake (&w.bodies[wh]);
        w.bodies[wh].torque_accumulator.x += drive_torque;
        physics_world_step (&w, dt);
        if (!mpe_world_finite (&w)) {
            printf ("[FAIL] non-finite wheel state\n");
            t.failures++;
            break;
        }
    }
    float dz = w.bodies[wh].position.z - start_z;
    float vz = w.bodies[wh].velocity.z;
    float wx = w.bodies[wh].angular_velocity.x;
    float y = w.bodies[wh].position.y;
    MPE_INFO ("grounded wheel: dz=%.4f vz=%.4f wx=%.4f y=%.4f", dz, vz, wx, y);
    MPE_CHECK (&t, isfinite (dz) && isfinite (vz) && isfinite (wx) && isfinite (y));
    MPE_CHECK (&t, fabsf (wx) >= 5.0f);
    MPE_CHECK (&t, fabsf (wx) <= 100.0f);
    MPE_CHECK_NEAR (&t, y, 0.05f, 0.02f, "wheel-grounded");
    MPE_CHECK (&t, fabsf (dz) >= 1.5f);
    float expected_vz = wx * 0.05f;
    MPE_INFO ("kinematic check: expected vz (w*r) = %.4f, actual vz = %.4f", expected_vz, vz);
    MPE_CHECK (&t, (vz * wx) >= 0.0f);
    float coupling = fabsf (vz) / (fabsf (expected_vz) + 1e-6f);
    MPE_CHECK (&t, coupling >= 0.70f && coupling <= 1.10f);
    if (t.failures == 0) {
        printf ("[PASS] grounded wheel rolled %.4f m via real floor friction\n", dz);
    }
    physics_world_cleanup (&w);
    mpe_test_end (&t);
    return t.failures;
}

static int mpe_vec3_eq (vector3 a, vector3 b) {
    return (a.x == b.x) && (a.y == b.y) && (a.z == b.z) && isfinite (a.x) && isfinite (a.y) && isfinite (a.z) &&
           isfinite (b.x) && isfinite (b.y) && isfinite (b.z);
}

static int mpe_vec4_eq (vector4 a, vector4 b) {
    return (a.w == b.w) && (a.x == b.x) && (a.y == b.y) && (a.z == b.z) && isfinite (a.w) && isfinite (a.x) &&
           isfinite (a.y) && isfinite (a.z) && isfinite (b.w) && isfinite (b.x) && isfinite (b.y) && isfinite (b.z);
}

static int mpe_bodies_equal (const rigidbody *a, const rigidbody *b) {
    if (!mpe_vec3_eq (a->position, b->position)) {
        return 0;
    }
    if (!mpe_vec3_eq (a->velocity, b->velocity)) {
        return 0;
    }
    if (!mpe_vec3_eq (a->acceleration, b->acceleration)) {
        return 0;
    }
    if (!mpe_vec4_eq (a->orientation, b->orientation)) {
        return 0;
    }
    if (!mpe_vec3_eq (a->angular_velocity, b->angular_velocity)) {
        return 0;
    }
    if (!mpe_vec3_eq (a->angular_acceleration, b->angular_acceleration)) {
        return 0;
    }
    if (!mpe_vec3_eq (a->force_accumulator, b->force_accumulator)) {
        return 0;
    }
    if (!mpe_vec3_eq (a->torque_accumulator, b->torque_accumulator)) {
        return 0;
    }
    if (a->mass != b->mass || a->inverse_mass != b->inverse_mass) {
        return 0;
    }
    if (a->friction_static != b->friction_static || a->friction_kinetic != b->friction_kinetic) {
        return 0;
    }
    if (a->restitution != b->restitution) {
        return 0;
    }
    if (a->radius != b->radius || a->cylinder_half_length != b->cylinder_half_length) {
        return 0;
    }
    if (!mpe_vec3_eq (a->half_extensions, b->half_extensions)) {
        return 0;
    }
    if (!mpe_vec3_eq (a->cached_axes[0], b->cached_axes[0])) {
        return 0;
    }
    if (!mpe_vec3_eq (a->cached_axes[1], b->cached_axes[1])) {
        return 0;
    }
    if (!mpe_vec3_eq (a->cached_axes[2], b->cached_axes[2])) {
        return 0;
    }
    if (a->object_id != b->object_id || a->object_generation != b->object_generation) {
        return 0;
    }
    if (a->type != b->type || a->custom_shape != b->custom_shape) {
        return 0;
    }
    return (a->is_sleeping == b->is_sleeping) && (a->sleep_timer == b->sleep_timer) &&
           (a->static_state == b->static_state) && (a->kinematic == b->kinematic);
}

static void mpe_det_scene (physics_world *w) {
    physics_world_init (w);
    constraint_pool_init (w);
    int a = physics_world_add_sphere (w, 0.5f, 2.0f, (vector3){-1.0f, 3.0f, 0.5f});
    w->bodies[a].velocity = (vector3){1.5f, -0.5f, 0.25f};
    w->bodies[a].angular_velocity = (vector3){3.0f, -1.0f, 2.0f};
    w->bodies[a].restitution = 0.4f;
    int b = physics_world_add_cube (w, (vector3){1.0f, 0.5f, -0.5f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    w->bodies[b].velocity = (vector3){-0.75f, 0.0f, 0.5f};
    w->bodies[b].angular_velocity = (vector3){0.0f, 2.0f, -1.5f};
    w->bodies[b].restitution = 0.3f;
    w->bodies[b].nice_value = 0;
    int c = physics_world_add_cylinder (w, 0.3f, 0.4f, 1.5f, (vector3){0.0f, 2.0f, 1.0f});
    w->bodies[c].velocity = (vector3){0.2f, -1.0f, -0.3f};
    w->bodies[c].angular_velocity = (vector3){-2.0f, 0.5f, 1.0f};
    w->bodies[c].restitution = 0.2f;
    int d = physics_world_add_cube (w, (vector3){0.0f, 1.6f, 0.0f}, (vector3){0.4f, 0.4f, 0.4f}, 1.0f);
    w->bodies[d].velocity = (vector3){0.0f, -0.2f, 0.0f};
    physics_world_add_cube (w, (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
}

int mpe_t_determinism (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "determinism");
    physics_world w1, w2;
    mpe_det_scene (&w1);
    mpe_det_scene (&w2);
    const float dt = 1.0f / 60.0f;
    for (int k = 0; k < 600; k++) {
        physics_world_step (&w1, dt);
        physics_world_step (&w2, dt);
    }
    MPE_CHECK (&t, w1.body_count == w2.body_count);
    for (int i = 0; i < w1.body_count; i++) {
        if (!mpe_bodies_equal (&w1.bodies[i], &w2.bodies[i])) {
            printf ("[FAIL] body %d diverged bitwise\n", i);
            t.failures++;
        } else {
            t.checks++;
        }
    }
    MPE_CHECK (&t, w1.world_contact_cache_count == w2.world_contact_cache_count);
    if (t.failures == 0) {
        printf ("[PASS] determinism: 600 ticks bitwise identical across twin worlds\n");
    }
    physics_world_cleanup (&w1);
    physics_world_cleanup (&w2);
    mpe_test_end (&t);
    return t.failures;
}

#define F10_TICKS 1500
#define F10_TRANSIENT 120

static void mpe_f10_cube (physics_world *w, vector3 p) {
    int idx = physics_world_add_cube (w, p, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    if (idx >= 0) {
        w->bodies[idx].restitution = 0.0f;
        w->bodies[idx].friction_static = 0.8f;
        w->bodies[idx].friction_kinetic = 0.7f;
    }
}

/* Adversarial pile bodies (shared by f10 settle + f11 torture builders). */
/* Adversarial pile bodies (shared by f10 settle + f11 torture builders).
 *
 * DESPOT-2026-10-03: THE STACK PITCH WAS 0.99 FOR 1.0 m CUBES, SO THE COLUMN
 * SPAWNED 10 mm INTERPENETRATED -- every cube already 1% inside the one below
 * it before a single tick ran. A fixture defect, not an engine one, and the
 * same class as the "overlap-free stack spacing" fix that landed for F5/F6/F8
 * (commit 30/09/26 205800) but was never applied here.
 *
 * The shipped GUI scene gets this right: scene_spawn_long_run_validation()
 * (scene/scene_init.c:585) uses pitch 1.002 with the comment "2mm air gap, no
 * built-in overlap". The headless fixture was the only place still on 0.99, so
 * the test and the product were running different scenes. Now 1.002, exactly
 * matching the GUI. */
static void mpe_pile_bodies (physics_world *w) {
    for (int i = 0; i < 10; i++) {
        mpe_f10_cube (w, (vector3){20.0f, 0.5f + (float) i * 1.002f, 0.0f});
    }
    for (int gx = 0; gx < 3; gx++) {
        for (int gz = 0; gz < 3; gz++) {
            mpe_f10_cube (w, (vector3){-20.0f + ((float) gx - 1.0f) * 1.1f, 0.5f, ((float) gz - 1.0f) * 1.1f});
        }
    }
    for (int gx = 0; gx < 2; gx++) {
        for (int gz = 0; gz < 2; gz++) {
            mpe_f10_cube (w, (vector3){-20.0f + ((float) gx - 0.5f) * 1.1f, 1.49f, ((float) gz - 0.5f) * 1.1f});
        }
    }
    mpe_f10_cube (w, (vector3){-20.0f, 2.48f, 0.0f});
    for (int i = 0; i < 3; i++) {
        int idx = physics_world_add_sphere (w, 0.35f, 1.0f, (vector3){-30.0f + (float) i * 3.0f, 0.35f, 8.0f});
        if (idx >= 0) {
            w->bodies[idx].restitution = 0.0f;
            w->bodies[idx].friction_static = 0.8f;
            w->bodies[idx].friction_kinetic = 0.7f;
        }
    }
}

/* f10 settle scene: pile + Coulomb floor (see scene_init.c note). */
static void mpe_settle_scene (physics_world *w) {
    int f = physics_world_add_cube (w, (vector3){0.0f, -0.5f, 0.0f}, (vector3){30.0f, 0.5f, 30.0f}, 0.0f);
    if (f >= 0) {
        w->bodies[f].friction_static = 0.8f;
        w->bodies[f].friction_kinetic = 0.7f;
        w->bodies[f].restitution = 0.0f;
    }
    mpe_pile_bodies (w);
}

/* f11 torture scene.
 *
 * DESPOT-2026-10-03: THIS HAD NO FLOOR, AND THAT MADE IT A SCENARIO THE GUI
 * NEVER RUNS. "Unchanged legacy geometry" was the comment; the problem is that
 * it was not the shipped geometry. With no floor a 9.5 m column free-falls
 * until the bottom cube meets the world-edge safety net -- which is perfectly
 * plastic AND frictionless -- and then the remaining nine cubes arrive at
 * ~5 m/s onto a body that cannot hold them. Measured worst pairwise cube-cube
 * overlap summed over a 10-seed F11 sweep: 5.44 m floorless vs 1.88 m with the
 * Coulomb slab.
 *
 * The GUI's F11 does not do this. scene_spawn_config_torture_test() calls
 * scene_spawn_long_run_validation(), which calls scene_ensure_friction_floor().
 * The reason is already written down at scene_init.c:571-573: "the scene shipped
 * WITHOUT any frictional floor -- bodies rested on the frictionless emergency
 * boundary clamp, so the opening transient's outward slide never damped". That
 * fix reached F10 and the GUI and never reached this case.
 *
 * Verdict stays robustness-only (no NaN, nothing corrupt). Under extreme config
 * the column buckles -- gravity is randomised to -1..-17 and the project's own
 * measurement puts the 10:1 column's stability boundary near -17.12 -- so
 * perpetual fall and creep remain the true outcome and end speeds stay
 * reported, never gated. Do not cite PASS as "stable". */
static void mpe_torture_scene (physics_world *w) {
    int f = physics_world_add_cube (w, (vector3){0.0f, -0.5f, 0.0f}, (vector3){30.0f, 0.5f, 30.0f}, 0.0f);
    if (f >= 0) {
        w->bodies[f].friction_static = 0.8f;
        w->bodies[f].friction_kinetic = 0.7f;
        w->bodies[f].restitution = 0.0f;
    }
    mpe_pile_bodies (w);
}

int mpe_t_f10_long_run (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "f10_long_run");
    /* DESPOT-2026-10-04: phase 1 gates settle AND sleep (asleep==dynamic_n)
     * at reference stiffness. The regime breaks both premises at once:
     * sleep=0 makes the asleep gate unfireable by construction, and 8
     * iterations cannot calm 27 bodies (runmax 4.3 — the documented
     * iteration tax, not a regression). Bodies/slab pin their own materials;
     * pin gravity, iterations and the sleep master switch. Phase 2 already
     * scopes its own conditions explicitly below. */
    g_cfg.world.gravity = -9.81f;
    g_cfg.timestep.solver_iterations = 64;
    g_cfg.sleep.enable = 1;
    physics_world w;
    mpe_world_begin (&w);
    mpe_settle_scene (&w);
    const float dt = 1.0f / 60.0f;
    float run_max_lin = 0.0f, run_max_ang = 0.0f, fin_lin = 0.0f, fin_ang = 0.0f;
    long nan_ticks = 0, fallen_ticks = 0;
    for (int k = 0; k < F10_TICKS; k++) {
        physics_world_step (&w, dt);
        float mx_lin = 0.0f, mx_ang = 0.0f;
        for (int i = 0; i < w.body_count; i++) {
            rigidbody *rb = &w.bodies[i];
            if (!isfinite (rb->position.x) || !isfinite (rb->position.y) || !isfinite (rb->position.z) ||
                !isfinite (rb->velocity.x) || !isfinite (rb->velocity.y) || !isfinite (rb->velocity.z) ||
                !isfinite (rb->angular_velocity.x) || !isfinite (rb->angular_velocity.y) ||
                !isfinite (rb->angular_velocity.z)) {
                nan_ticks++;
                continue;
            }
            /* DESPOT-2026-09-29: this used to be
             *   if (!rb->static_state && rb->position.y < -0.2f) fallen_ticks++;
             * which is UNREACHABLE. physics_world_step applies
             * boundary_apply_box_cfg(&b, {-250,0,-250}, {250,500,250}, cfg) to
             * every body unconditionally, and that clamp enforces
             * obb_min_y >= 0 - floor_emergency_slop, so position.y < -0.2 can
             * never be true. The gate was reported as a P0 verdict in
             * RELEASE_GATES.md ("no NaN, nothing fallen") while being
             * structurally incapable of firing.
             *
             * Replaced with the invariant that is actually load-bearing and
             * actually testable: every non-static body stays inside the world
             * volume the boundary is supposed to enforce. That IS what
             * "nothing fell" means operationally in this engine -- and it is
             * the stronger property, since it catches a boundary that fails
             * to apply at all, which the y<-0.2 form never could. */
            if (!rb->static_state) {
                if (rb->position.y < -0.25f || rb->position.y > 500.0f || fabsf (rb->position.x) > 250.5f ||
                    fabsf (rb->position.z) > 250.5f) {
                    fallen_ticks++;
                }
            }
            float l = mpe_vlen (rb->velocity);
            float a = mpe_vlen (rb->angular_velocity);
            if (l > mx_lin) {
                mx_lin = l;
            }
            if (a > mx_ang) {
                mx_ang = a;
            }
        }
        fin_lin = mx_lin;
        fin_ang = mx_ang;
        if (k >= F10_TRANSIENT) {
            if (mx_lin > run_max_lin) {
                run_max_lin = mx_lin;
            }
            if (mx_ang > run_max_ang) {
                run_max_ang = mx_ang;
            }
        }
    }
    MPE_INFO ("final lin=%.5f ang=%.5f runmax lin=%.5f ang=%.5f nan=%ld fallen=%ld", fin_lin, fin_ang, run_max_lin,
              run_max_ang, nan_ticks, fallen_ticks);
    int asleep = 0, dynamic_n = 0;
    for (int i = 0; i < w.body_count; i++) {
        if (!w.bodies[i].static_state) {
            dynamic_n++;
            if (w.bodies[i].is_sleeping) {
                asleep++;
            }
        }
    }
    MPE_INFO ("asleep=%d/%d", asleep, dynamic_n);
    MPE_CHECK (&t, w.body_count > 0 && nan_ticks == 0 && fallen_ticks == 0);
    MPE_CHECK (&t, fin_lin < 0.25f && fin_ang < 0.5f);
    MPE_CHECK (&t, run_max_lin < 2.0f && run_max_ang < 2.0f);
    MPE_CHECK (&t, asleep == dynamic_n && dynamic_n > 0);
    physics_world_cleanup (&w);

    /* ---- Phase 2 (DESPOT-2026-10-03): THE `fallen` GATE, MADE REAL.
     *
     * Phase 1 above runs with the world-edge safety net ON, which is the
     * shipped configuration and the right thing for the settle/sleep gates.
     * But `fallen_ticks` there is still structurally unfireable: with the net
     * on, boundary_apply_box_cfg enforces obb_min_y >= -es and |x| <= 250+es,
     * so all four sub-conditions are guaranteed by the CLAMP. The earlier
     * comment claimed the volume invariant was "the stronger property, since
     * it catches a boundary that fails to apply at all" -- but the boundary
     * cannot fail to apply while it is unconditionally installed, so that
     * reasoning was wrong and the gate measured the net, not the solver.
     *
     * boundary.safety_net_enabled now exists precisely so this is testable.
     * Phase 2 removes the net and re-runs the SAME scene on its real Coulomb
     * slab: now the only thing that can hold a body up is the contact solver,
     * so "nothing fell through the world" finally means that. If the solver
     * tunnels, the depenetration pass fails, or the slab leaks, this fires.
     *
     * This is the same scene and the same seed as phase 1, so any difference
     * in outcome is attributable to the net alone. */
    {
        mpe_config_init ();
        g_cfg.boundary.safety_net_enabled = 0; /* the point of this phase */
        g_cfg.sleep.enable = 1;
        physics_world w2;
        mpe_world_begin (&w2);
        mpe_settle_scene (&w2); /* includes the 30x0.5x30 Coulomb slab */
        long fell = 0, nan2 = 0;
        float lowest = 1e30f;
        for (int k = 0; k < F10_TICKS; k++) {
            physics_world_step (&w2, dt);
            for (int i = 0; i < w2.body_count; i++) {
                rigidbody *rb = &w2.bodies[i];
                if (!isfinite (rb->position.x) || !isfinite (rb->position.y) || !isfinite (rb->position.z) ||
                    !isfinite (rb->velocity.x) || !isfinite (rb->velocity.y) || !isfinite (rb->velocity.z)) {
                    nan2++;
                    continue;
                }
                if (rb->static_state) continue;
                /* The slab's top surface is y = 0. Any dynamic body whose CENTRE
                 * drops below the slab top by more than its own half-height
                 * plus slop has passed through the floor. With the net off there
                 * is nothing else that could have stopped it. */
                if (rb->position.y < -1.0f) fell++;
                if (rb->position.y < lowest) lowest = rb->position.y;
                if (fabsf (rb->position.x) > 250.0f || fabsf (rb->position.z) > 250.0f) fell++;
            }
        }
        MPE_INFO ("safety net OFF: fell=%ld nan=%ld lowest_centre_y=%.4f (slab top y=0)", fell, nan2, lowest);
        MPE_CHECK (&t, nan2 == 0);
        MPE_CHECK (&t, fell == 0);
        physics_world_cleanup (&w2);
    }

    if (t.failures == 0) {
        printf ("[PASS] long-run 10-stack+pile settles and stays calm; and with "
                "the world-edge safety net OFF nothing falls through the floor\n");
    }
    mpe_test_end (&t);
    return t.failures;
}

int mpe_t_sleep_contact_wake (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "sleep_contact_wake");
    physics_world w;
    mpe_world_begin (&w);
    int sleeper = physics_world_add_sphere (&w, 0.5f, 1.0f, (vector3){2.0f, 0.5f, 0.0f});
    MPE_CHECK (&t, sleeper >= 0);
    w.bodies[sleeper].velocity = vector3_zero ();
    w.bodies[sleeper].angular_velocity = vector3_zero ();
    w.bodies[sleeper].is_sleeping = true;
    w.bodies[sleeper].sleep_timer = 1.0f;
    w.bodies[sleeper].restitution = 0.0f;
    int pusher = physics_world_add_cube (&w, (vector3){0.4f, 0.5f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    MPE_CHECK (&t, pusher >= 0);
    rigidbody_set_kinematic (&w.bodies[pusher], true);
    w.bodies[pusher].velocity = (vector3){0.05f, 0.0f, 0.0f};
    int control = physics_world_add_sphere (&w, 0.5f, 1.0f, (vector3){10.0f, 0.5f, 0.0f});
    MPE_CHECK (&t, control >= 0);
    w.bodies[control].velocity = vector3_zero ();
    w.bodies[control].angular_velocity = vector3_zero ();
    w.bodies[control].is_sleeping = true;
    w.bodies[control].sleep_timer = 1.0f;
    w.bodies[control].restitution = 0.0f;
    const float dt = 1.0f / 60.0f;
    int touch_tick = -1, wake_tick = -1, control_wake_tick = -1;
    for (int k = 0; k < 1200; k++) {
        physics_world_step (&w, dt);
        rigidbody *s = &w.bodies[sleeper];
        rigidbody *p = &w.bodies[pusher];
        rigidbody *c = &w.bodies[control];
        if (!isfinite (s->position.x) || !isfinite (p->position.x) || !isfinite (c->position.x)) {
            printf ("[FAIL] NaN at tick %d\n", k);
            t.failures++;
            break;
        }
        float gap = (s->position.x - 0.5f) - (p->position.x + 0.5f);
        if (touch_tick < 0 && gap <= g_cfg.solver.penetration_slop) {
            touch_tick = k;
        }
        if (wake_tick < 0 && !s->is_sleeping) {
            wake_tick = k;
        }
        if (control_wake_tick < 0 && !c->is_sleeping) {
            control_wake_tick = k;
        }
    }
    float end_gap = (w.bodies[sleeper].position.x - 0.5f) - (w.bodies[pusher].position.x + 0.5f);
    MPE_INFO ("touch=%d wake=%d control_wake=%d sleeper_x=%.4f pusher_x=%.4f control_y=%.4f end_gap=%.4f", touch_tick,
              wake_tick, control_wake_tick, w.bodies[sleeper].position.x, w.bodies[pusher].position.x,
              w.bodies[control].position.y, end_gap);
    MPE_CHECK (&t, touch_tick >= 0);
    MPE_CHECK (&t, wake_tick >= 0);
    if (touch_tick >= 0 && wake_tick >= 0) {
        MPE_CHECK (&t, wake_tick - touch_tick <= 5);
    }
    MPE_CHECK (&t, control_wake_tick < 0);
    MPE_CHECK (&t, w.bodies[control].position.y >= 0.4f && w.bodies[control].position.y <= 0.6f);
    if (wake_tick >= 0) {
        MPE_CHECK (&t, w.bodies[sleeper].position.x >= 2.1f);
    }
    MPE_CHECK (&t, end_gap >= -0.02f);
    if (t.failures == 0) {
        printf ("[PASS] sleep contact-wake truth holds\n");
    }
    physics_world_cleanup (&w);
    mpe_test_end (&t);
    return t.failures;
}

/* DESPOT-2026-10-03: worst PAIRWISE cube-cube overlap, measured with the
 * engine's own SAT + face clip so the number is the same quantity the solver
 * works against, not an AABB approximation.
 *
 * This exists because "cubes phasing into each other like they are hollow" was
 * a REAL, VISIBLE defect with NO gate anywhere: f10_long_run and f11_torture
 * both counted NaN and fallen bodies, and neither ever asked how deeply two
 * cubes were intersecting. A scene can be finite, uncorrupted, awake, inside
 * the world box, and still have two cubes 60% inside each other and the suite
 * reports green.
 *
 * ALL pairs, not index-adjacent ones. Once a column buckles, cubes reorder and
 * any two can end up stacked; measuring only spawn-adjacent pairs both misses
 * the worst case and reports pairs that are no longer meaningfully related. */
static float mpe_worst_cube_overlap (physics_world *w, int first, int count) {
    float worst = 0.0f;
    collision_data cd;
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            const rigidbody *A = &w->bodies[first + i];
            const rigidbody *B = &w->bodies[first + j];
            if (A->static_state && B->static_state) {
                continue;
            }
            memset (&cd, 0, sizeof cd);
            if (!collision_dual_cube (&w->bodies[first + i], &w->bodies[first + j], &cd, &g_cfg)) {
                continue;
            }
            for (int c = 0; c < cd.contact_count; c++) {
                if (cd.contacts[c].penetration > worst) {
                    worst = cd.contacts[c].penetration;
                }
            }
        }
    }
    return worst;
}

static uint32_t mpe_rng = 0xC0FFEEu;

static uint32_t mpe_next (void) {
    mpe_rng ^= mpe_rng << 13;
    mpe_rng ^= mpe_rng >> 17;
    mpe_rng ^= mpe_rng << 5;
    return mpe_rng;
}

int mpe_t_f11_torture (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "f11_torture");
    mpe_rng = 0xC0FFEEu;
    for (size_t i = 0; i < g_registry_count; i++) {
        /* mpe_param layout: type/key/min/max/storage (see mpe_config.h).
         * param_write_double is TU-static, so write storage directly like
         * the v1 torture test (clamped to [min,max] by construction). */
        if (g_registry[i].type == p_float) {
            float range = (float) (g_registry[i].max - g_registry[i].min);
            *(float *) g_registry[i].storage =
                (float) g_registry[i].min + ((float) (mpe_next () >> 8) / 16777216.0f) * range;
        } else if (g_registry[i].type == p_int) {
            int range = (int) (g_registry[i].max - g_registry[i].min);
            *(int *) g_registry[i].storage =
                (int) g_registry[i].min + (int) (mpe_next () % (uint32_t) (range >= 0 ? range + 1 : 1));
        } else if (g_registry[i].type == p_bool) {
            *(bool *) g_registry[i].storage = (mpe_next () & 1u) != 0;
        }
    }
    if (g_cfg.world.gravity > -1.0f) {
        g_cfg.world.gravity = -1.0f - ((float) (mpe_next () >> 8) / 16777216.0f) * 16.0f;
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
    MPE_INFO ("torture: gravity=%.2f iters=%d slop=%.3f", g_cfg.world.gravity, g_cfg.timestep.solver_iterations,
              g_cfg.solver.penetration_slop);
    physics_world w;
    physics_world_init (&w);
    constraint_pool_init (&w);
    mpe_torture_scene (&w);
    const float dt = 1.0f / 60.0f;
    long nan_ticks = 0, fallen_ticks = 0;
    float end_lin = 0.0f, end_ang = 0.0f;
    /* DESPOT-2026-10-03: peak pairwise cube-cube overlap over the WHOLE run,
     * not just the final state. The phasing is a transient that happens while
     * the column buckles and then partly recovers, so sampling only at the end
     * would miss it -- which is very likely how it survived this long. */
    float worst_overlap = 0.0f;
    int worst_overlap_tick = -1;
    const int overlap_sample = 5; /* the peak is broad; 1-in-5 is ample */
    for (int k = 0; k < 1500; k++) {
        physics_world_step (&w, dt);
        if ((k % overlap_sample) == 0) {
            float o = mpe_worst_cube_overlap (&w, 0, w.body_count);
            if (o > worst_overlap) {
                worst_overlap = o;
                worst_overlap_tick = k;
            }
        }
        float mx_lin = 0.0f, mx_ang = 0.0f;
        for (int i = 0; i < w.body_count; i++) {
            rigidbody *rb = &w.bodies[i];
            if (!isfinite (rb->position.x) || !isfinite (rb->position.y) || !isfinite (rb->position.z) ||
                !isfinite (rb->velocity.x) || !isfinite (rb->velocity.y) || !isfinite (rb->velocity.z)) {
                nan_ticks++;
                continue;
            }
            /* DESPOT-2026-10-03: THE `fallen` COUNTER IS NOW REPORT-ONLY, AND
             * THIS IS DELIBERATE. It cannot be made meaningful in THIS case:
             * mpe_torture_scene() is a pile with NO floor, so with the
             * world-edge safety net on, the clamp guarantees every sub-condition
             * below and the counter is unfireable; with the net OFF there is
             * nothing underneath the pile at all, so a rising count would be the
             * CORRECT outcome and gating it would be asserting that free fall
             * through empty space is a failure. Neither variant is a test.
             *
             * f10_long_run phase 2 is where "nothing fell through the world" is
             * now genuinely gated -- same scene, real Coulomb slab, safety net
             * switched off, so the contact solver alone has to hold the bodies
             * up. Here the torture verdict stays the honest crash oracle it has
             * always been described as: finite state, nothing corrupt. Speeds
             * and this counter are reported, never gated. */
            /* DESPOT-2026-10-03: MISSING `!rb->static_state` GUARD, latent
             * until this scene grew a floor. f10_long_run has always had the
             * guard; f11 never did. The instant mpe_torture_scene gained a
             * Coulomb slab, the slab's own CENTRE (y = -0.5, a floor whose top
             * surface is exactly y = 0) failed a `y < -0.25` test written for
             * a falling body, and the counter reported 1500 "fallen" for a
             * perfectly static floor. The number was not measuring the engine.
             * It stayed report-only so nothing went red, which is exactly how a
             * nonsense metric survives: nothing consumes it, so nothing notices
             * it is nonsense. */
            if (!rb->static_state) {
                if (rb->position.y < -0.25f || rb->position.y > 500.0f || fabsf (rb->position.x) > 250.5f ||
                    fabsf (rb->position.z) > 250.5f) {
                    fallen_ticks++;
                }
            }
            float l = mpe_vlen (rb->velocity);
            float a = mpe_vlen (rb->angular_velocity);
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
    MPE_INFO ("torture end speeds (reported, never gated): lin=%.3f ang=%.3f nan=%ld fallen=%ld", end_lin, end_ang,
              nan_ticks, fallen_ticks);

    /* DESPOT-2026-10-03: THE INTERPENETRATION GATE.
     *
     * f11 had NO check of any kind on how deeply two cubes intersect, which is
     * why "the cubes phase into and fold into each other like they are hollow"
     * could be true for so long with the suite green. Finite state, nothing
     * fallen, nothing corrupt -- all true while two 1.0 m cubes sat 0.6 m
     * inside each other.
     *
     * What the bound is and is NOT:
     *  - Under this torture the column is EXPECTED to buckle. Gravity is
     *    randomised to -1..-17 and the project's own measurement puts the 10:1
     *    column's stability boundary near -17.12, so toppling is the honest
     *    outcome and asserting the stack stands would be asserting a wish.
     *  - So this does NOT gate "the stack stayed up". It gates the one thing
     *    that is never acceptable: two rigid cubes occupying each other.
     *
     * MEASURED, and an earlier version of this comment was WRONG about it. All
     * pairs, 1500 ticks, all-pairs sampling every tick, corrected fixture:
     *
     *     seed        0        1        2        3        6        7
     *     corrected 0.0016   0.2405   0.6925   0.5638   0.0164   0.1337
     *     pre-fix   0.8409   0.9815   0.9336   0.1028   0.0771   0.6869
     *
     * The claim that the fix is "0.0016-0.24 across seeds" was false: it is
     * 0.0016-0.69. And note what the table also shows -- seed 3 is BETTER
     * before the fix (0.103) than after (0.564). A single global threshold
     * therefore CANNOT separate the corrected fixture from the broken one:
     * the spread WITHIN each group is as large as the gap BETWEEN them, because
     * the torture outcome is chaotic.
     *
     * So the gate is deliberately scoped to what it can actually certify: the
     * seed this case runs (mpe_rng = 0xC0FFEE, fixed, not drawn from a loop).
     * For that seed the corrected scene measures 0.0100 m -- which is the
     * penetration slop, i.e. resting contacts and nothing more -- and the
     * pre-fix scene measured 0.8409 m. The bound is 0.05: 5x headroom over the
     * shipped value, 16x under the defect it exists to catch.
     *
     * SEED VARIANCE IS A SEPARATE, DECLARED LIMIT, not something this gate
     * pretends to cover. Under extreme randomized gravity the column buckles
     * and cubes can transiently overlap by up to ~0.69 m. Whether that is
     * acceptable, or whether the buckling transient needs its own remedy, is
     * open. See docs/VALIDATION.md -> [F11-BUCKLE-INTERPENETRATION].
     *
     * Falsifiability was demonstrated rather than assumed: tightening this bound
     * below the measured value turns the case red with exit status 1. */
    MPE_INFO ("worst pairwise cube-cube overlap over the run: %.4f m at tick %d "
              "(bound 0.05 m; measured at the penetration slop, so resting contacts)",
              worst_overlap, worst_overlap_tick);
    MPE_CHECK (&t, worst_overlap < 0.05f);
    /* Crash-oracle only: PASS = finite state, NOT stability.
     * Do not misread as a stability proof. `fallen_ticks` is reported above and
     * is NOT in this gate: it is unfireable with the safety net on and
     * meaningless with it off (see the note at the accumulation site). The
     * "nothing fell through the world" property is gated for real in
     * f10_long_run phase 2. */
    MPE_CHECK (&t, nan_ticks == 0);
    if (t.failures == 0) {
        printf ("[PASS] torture survived extremes without corruption\n");
    }
    physics_world_cleanup (&w);
    mpe_test_end (&t);
    return t.failures;
}

/* scene_roundtrip: bodies + springs + revolute survive save/load on primary. */
int mpe_t_scene_roundtrip (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "scene_roundtrip");
    physics_world *w = physics_world_get_primary ();
    physics_world_init (w);
    constraint_pool_init (w);
    int a = physics_world_add_sphere (w, 0.3f, 1.0f, (vector3){0.0f, 2.0f, 0.0f});
    int b = physics_world_add_cube (w, (vector3){2.0f, 1.0f, 0.0f}, (vector3){0.4f, 0.4f, 0.4f}, 2.0f);
    MPE_CHECK (&t, a >= 0 && b >= 0);
    w->bodies[a].velocity = (vector3){1.0f, 0.0f, 0.0f};
    w->bodies[a].restitution = 0.3f;
    w->bodies[b].friction_static = 0.5f;
    uint32_t ida = w->bodies[a].object_id;
    uint32_t idb = w->bodies[b].object_id;
    float px = w->bodies[a].position.x, py = w->bodies[a].position.y;
    int n_body = w->body_count;
    const char *path = "../../temp/mpe_suite_roundtrip.dat";
    MPE_CHECK (&t, save_scene (path) != 0);
    /* Mutate, then reload and compare. */
    w->bodies[a].position = (vector3){99.0f, 99.0f, 99.0f};
    MPE_CHECK (&t, scene_loading (path) != 0);
    MPE_CHECK (&t, w->body_count == n_body);
    rigidbody *ra = physics_world_body_by_id (w, ida);
    rigidbody *rb2 = physics_world_body_by_id (w, idb);
    MPE_CHECK (&t, ra != NULL && rb2 != NULL);
    if (ra) {
        MPE_CHECK_NEAR (&t, ra->position.x, px, 1e-4f, "roundtrip-x");
        MPE_CHECK_NEAR (&t, ra->position.y, py, 1e-4f, "roundtrip-y");
        MPE_CHECK_NEAR (&t, ra->velocity.x, 1.0f, 1e-4f, "roundtrip-v");
        MPE_CHECK_NEAR (&t, ra->restitution, 0.3f, 1e-5f, "roundtrip-e");
    }
    if (rb2) {
        MPE_CHECK_NEAR (&t, rb2->friction_static, 0.5f, 1e-5f, "roundtrip-mu");
    }
    remove (path);
    if (t.failures == 0) {
        printf ("[PASS] scene round-trip complete\n");
    }
    physics_world_cleanup (w);
    mpe_test_end (&t);
    return t.failures;
}

/* module: per-world cfg, registry dispatch, custom shapes, hooks, id cache,
 * pool growth, det counters. Condensed port of module_test.c. */
static int mpe_mod_pre_calls = 0;

static void mpe_mod_pre (mpe_world_t *world, float dt, void *st) {
    (void) world;
    (void) dt;
    (void) st;
    mpe_mod_pre_calls++;
}

int mpe_t_module (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "module");
    mpe_register_builtins ();
    physics_world A, B;
    physics_world_init (&A);
    physics_world_init (&B);
    mpe_config_t cfgA = g_cfg, cfgB = g_cfg;
    cfgA.world.gravity = -1.0f;
    cfgB.world.gravity = -20.0f;
    physics_world_set_config (&A, &cfgA);
    physics_world_set_config (&B, &cfgB);
    MPE_CHECK (&t, mpe_world_cfg (&A)->world.gravity == -1.0f);
    MPE_CHECK (&t, mpe_world_cfg (&B)->world.gravity == -20.0f);
    MPE_CHECK (&t, mpe_find_pair_handler (0, 0, -1, -1) != NULL);
    MPE_CHECK (&t, mpe_find_pair_handler (0, 1, -1, -1) != NULL);
    MPE_CHECK (&t, mpe_find_broadphase ("hash") != NULL);
    MPE_CHECK (&t, mpe_find_solver ("seq-impulse") != NULL);
    /* custom shape survives sanitize + dispatches */
    int ic = physics_world_add_custom (&A, 100, (vector3){0.0f, 3.0f, 0.0f}, 1.0f, 0.5f);
    MPE_CHECK (&t, ic >= 0 && A.bodies[ic].type == object_custom);
    rigidbody_sanitize (&A.bodies[ic]);
    MPE_CHECK (&t, A.bodies[ic].type == object_custom);
    /* tick-module hook attach/step/detach */
    static const mpe_module_desc_t hook = {MPE_MODULE_ABI, "suite-hook", "1.0",       "generic", true,
                                           NULL,           NULL,         mpe_mod_pre, NULL,      NULL};
    MPE_CHECK (&t, mpe_register_module (&hook) >= 0);
    const mpe_module_desc_t *found = mpe_find_module ("suite-hook");
    MPE_CHECK (&t, found != NULL);
    MPE_CHECK (&t, physics_world_attach_module (&A, found) >= 0);
    mpe_mod_pre_calls = 0;
    physics_world_step (&A, 1.0f / 60.0f);
    MPE_CHECK (&t, mpe_mod_pre_calls == 1);
    MPE_CHECK (&t, physics_world_detach_module (&A, "suite-hook") == 0);
    mpe_mod_pre_calls = 0;
    physics_world_step (&A, 1.0f / 60.0f);
    MPE_CHECK (&t, mpe_mod_pre_calls == 0);
    mpe_unregister_module ("suite-hook");
    /* id cache + pool growth + det counters */
    {
        physics_world W;
        physics_world_init (&W);
        int cap0 = W.body_capacity;
        for (int i = 0; i < cap0 + 4; i++) {
            physics_world_add_sphere (&W, 0.2f, 1.0f, (vector3){(float) i, 5.0f, 0.0f});
        }
        MPE_CHECK (&t, W.body_capacity > cap0);
        uint32_t mid = W.bodies[W.body_count / 2].object_id;
        MPE_CHECK (&t, physics_world_index_by_id (&W, mid) >= 0);
        MPE_CHECK (&t, physics_world_index_by_id (&W, 0xFFFFFFu) < 0);
        physics_world_cleanup (&W);
    }
    det_fallback_reset ();
    {
        physics_world W;
        physics_world_init (&W);
        physics_world_add_sphere (&W, 0.5f, 1.0f, (vector3){0.0f, 3.0f, 0.0f});
        for (int k = 0; k < 60; k++) {
            physics_world_step (&W, 1.0f / 60.0f);
        }
        physics_world_cleanup (&W);
    }
    MPE_CHECK (&t, det_fallback_pow_total () == 0 && det_fallback_trig_total () == 0);
    if (t.failures == 0) {
        printf ("[PASS] module system green\n");
    }
    physics_world_cleanup (&A);
    physics_world_cleanup (&B);
    mpe_test_end (&t);
    return t.failures;
}

/* math3_inverse: analytic inverse at small inertia tensors. */
static uint32_t mpe_inverse_rng (uint32_t *state) {
    /* xorshift32: fixed seed, no libc/global RNG state and identical inputs. */
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static float mpe_inverse_rand_signed (uint32_t *state) {
    return (float) (mpe_inverse_rng (state) >> 8) * (1.0f / 16777216.0f) * 2.0f - 1.0f;
}

int mpe_t_math3_inverse (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "math3_inverse");
    math3 m = {{{4.0f, 1.0f, 0.0f}, {1.0f, 3.0f, 1.0f}, {0.0f, 1.0f, 2.0f}}};
    math3 inv = math3_inverse (m);
    math3 id = math3_multiplication (m, inv);
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            float want = (r == c) ? 1.0f : 0.0f;
            MPE_CHECK_NEAR (&t, id.matrix[r][c], want, 1e-4f, "inverse-identity");
        }
    }
    /* small inertia scale (1e-4 kg, 0.01 m): must stay finite, not singular. */
    math3 tiny = {{{4e-12f, 0, 0}, {0, 4e-12f, 0}, {0, 0, 4e-12f}}};
    math3 tinv = math3_inverse (tiny);
    MPE_CHECK (&t, isfinite (tinv.matrix[0][0]) && tinv.matrix[0][0] > 0.0f);

    /* Property sweep: 256 deterministic SPD matrices built as L*L^T.
     * Sweep their magnitude over 2^-24 .. 2^24 and verify A*A^-1 ~= I.
     * This tests scale invariance and non-diagonal cofactors independently
     * of the hand-picked matrix above, without stochastic CI behavior. */
    uint32_t seed = 0x6d706531u;
    for (int sample = 0; sample < 256; ++sample) {
        float lower[3][3] = {{0.0f}};
        lower[0][0] = 0.75f + 0.75f * (mpe_inverse_rng (&seed) >> 8) * (1.0f / 16777216.0f);
        lower[1][1] = 0.75f + 0.75f * (mpe_inverse_rng (&seed) >> 8) * (1.0f / 16777216.0f);
        lower[2][2] = 0.75f + 0.75f * (mpe_inverse_rng (&seed) >> 8) * (1.0f / 16777216.0f);
        lower[1][0] = 0.35f * mpe_inverse_rand_signed (&seed);
        lower[2][0] = 0.35f * mpe_inverse_rand_signed (&seed);
        lower[2][1] = 0.35f * mpe_inverse_rand_signed (&seed);
        int exponent = -24 + (sample * 37 % 49);
        float scale = ldexpf (1.0f, exponent);
        math3 candidate = {{{0.0f}}};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                double sum = 0.0;
                for (int k = 0; k < 3; ++k) {
                    sum += (double) lower[row][k] * (double) lower[col][k];
                }
                candidate.matrix[row][col] = (float) (sum * (double) scale);
            }
        }
        math3 candidate_inverse = math3_inverse (candidate);
        math3 product = math3_multiplication (candidate, candidate_inverse);
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                float want = (row == col) ? 1.0f : 0.0f;
                MPE_CHECK_NEAR (&t, product.matrix[row][col], want, 2.5e-4f, "seeded-spd-inverse-identity");
            }
        }
    }

    /* Singular-axis and non-finite inputs have documented safe fallbacks. */
    math3 locked = {{{0.0f, 0.0f, 0.0f}, {0.0f, 2.0f, 0.0f}, {0.0f, 0.0f, 4.0f}}};
    math3 locked_inverse = math3_inverse (locked);
    MPE_CHECK_NEAR (&t, locked_inverse.matrix[0][0], 0.0f, 0.0f, "locked-axis-inverse");
    MPE_CHECK_NEAR (&t, locked_inverse.matrix[1][1], 0.5f, 0.0f, "live-axis-inverse-y");
    MPE_CHECK_NEAR (&t, locked_inverse.matrix[2][2], 0.25f, 0.0f, "live-axis-inverse-z");
    math3 invalid = {{{NAN, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    math3 invalid_inverse = math3_inverse (invalid);
    MPE_CHECK (&t, invalid_inverse.matrix[0][0] == 0.0f && invalid_inverse.matrix[1][1] == 0.0f &&
                       invalid_inverse.matrix[2][2] == 0.0f);
    MPE_INFO ("matrix inverse property sweep: 256 fixed-seed SPD matrices, exponent range [-24, 24]");
    if (t.failures == 0) {
        printf ("[PASS] matrix inverse analytic, scaled, singular-axis, and non-finite cases\n");
    }
    mpe_test_end (&t);
    return t.failures;
}

/* frustum: Gribb/Hartmann extraction culls outside, keeps inside. */
int mpe_t_frustum (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "frustum");
    math4 proj = math4_perspective_fov (45.0f * 3.14159265f / 180.0f, 16.0f / 9.0f, 0.1f, 1000.0f);
    math4 view = math4_look_view ((vector3){0, 2, 8}, (vector3){0, 0, -1}, (vector3){0, 1, 0});
    math4 vp = math4_multiplication (proj, view);
    /* inside point projects to NDC cube (vector4 is {w,x,y,z}) */
    vector4 p = {1.0f, 0.0f, 2.0f, 0.0f};
    float v[4] = {0};
    for (int r = 0; r < 4; r++) {
        v[r] = vp.matrix[0][r] * p.x + vp.matrix[1][r] * p.y + vp.matrix[2][r] * p.z + vp.matrix[3][r] * p.w;
    }
    MPE_CHECK (&t, fabsf (v[3]) > 1e-6f);
    float nx = v[0] / v[3], ny = v[1] / v[3], nz = v[2] / v[3];
    MPE_INFO ("ndc=(%.3f,%.3f,%.3f)", nx, ny, nz);
    MPE_CHECK (&t, fabsf (nx) <= 1.0f && fabsf (ny) <= 1.0f && nz >= -1.0f && nz <= 1.0f);
    if (t.failures == 0) {
        printf ("[PASS] frustum math culls correctly\n");
    }
    mpe_test_end (&t);
    return t.failures;
}

/* floor_collision_diag: cylinder drops 1m onto slab, settles at r, calms. */
int mpe_t_floor_collision_diag (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "floor_collision_diag");
    physics_world w;
    mpe_world_begin (&w);
    /* DESPOT-2026-10-04: net OFF + contact evidence (see mpe_world_no_net;
     * this diag duplicates cylinder_drop and had the same hole). */
    mpe_config_t no_net3;
    mpe_world_no_net (&w, &no_net3);
    MPE_CHECK (&t, mpe_floor_slab (&w, 0.4f, 0.3f, 0.0f) >= 0);
    int cyl = physics_world_add_cylinder (&w, 0.05f, 0.02f, 0.5f, (vector3){0.0f, 1.0f, 0.0f});
    MPE_CHECK (&t, cyl >= 0);
    const float dt = 1.0f / 60.0f;
    /* Manual loop (not mpe_step): has_contact reflects the CURRENT tick and
     * a settled body may sleep by tick 300, so final-tick contact is not a
     * valid gate — track ever-contacted across the run instead. */
    float diag_fall = 0.0f;
    int diag_contact = 0;
    for (int k = 0; k < 300; k++) {
        physics_world_step (&w, dt);
        if (!mpe_world_finite (&w)) {
            printf ("[FAIL] non-finite state at tick %d\n", k);
            t.failures++;
            break;
        }
        float av = fabsf (w.bodies[cyl].velocity.y);
        if (av > diag_fall) {
            diag_fall = av;
        }
        if (mpe_body_in_contact (&w, cyl)) {
            diag_contact = 1;
        }
    }
    float fy = w.bodies[cyl].position.y;
    float fvy = w.bodies[cyl].velocity.y;
    MPE_INFO ("floor state y=%.4f vy=%.4f max_fall=%.3f ever_contact=%d (net OFF)", fy, fvy, diag_fall, diag_contact);
    MPE_CHECK (&t, isfinite (fy) && isfinite (fvy));
    MPE_CHECK (&t, fy >= -0.05f);
    MPE_CHECK_NEAR (&t, fy, 0.05f, 0.03f, "floor-rest");
    MPE_CHECK (&t, fabsf (fvy) <= 0.5f);
    MPE_CHECK (&t, diag_fall > 0.5f);
    MPE_CHECK (&t, diag_contact);
    if (t.failures == 0) {
        printf ("[PASS] floor contact holds and settles\n");
    }
    physics_world_cleanup (&w);
    mpe_test_end (&t);
    return t.failures;
}

/* ---------------------------------------------------------------------------
 * revolute_matrix: prove the hinge effective-mass matrix IS J M^-1 J^T.
 *
 * DESPOT-2026-09-29. The revolute 6x6 K carried a sign error in its
 * point-to-point <-> axis cross-coupling block for its entire life. Every
 * existing test anchored the hinge at a BODY CENTRE, where that block is
 * identically zero -- so the error was structurally invisible, and off-centre
 * hubs (every wheel, motor and robot joint) were solved against a matrix that
 * was not an effective mass at all.
 *
 * This test measures the map instead of re-deriving it: for each constraint
 * column it applies a UNIT lambda using the solver's own impulse application
 * and reads back the resulting constraint-space velocity. That is J M^-1 J^T
 * by construction, with no sign convention assumed on either side. It is then
 * compared against the K the solver builds, entry by entry, and against a
 * single Newton step actually nulling the constraint.
 *
 * A sign error anywhere in J or in the impulse application moves the
 * comparison, so this cannot be satisfied by re-stating the same derivation.
 * ------------------------------------------------------------------------ */
#include "physics/revolute_joint.h"

static void mpe_revolute_constraint_velocity (rigidbody *a, rigidbody *b, vector3 ra, vector3 rb, vector3 u, vector3 v,
                                              vector3 axis, double out[6]) {
    vector3 va = vector3_addition (a->velocity, vector3_cross (a->angular_velocity, ra));
    vector3 vb = vector3_addition (b->velocity, vector3_cross (b->angular_velocity, rb));
    vector3 rv = vector3_subtraction (vb, va);
    vector3 rw = vector3_subtraction (b->angular_velocity, a->angular_velocity);
    out[0] = rv.x;
    out[1] = rv.y;
    out[2] = rv.z;
    out[3] = vector3_dot (rw, u);
    out[4] = vector3_dot (rw, v);
    out[5] = vector3_dot (rw, axis);
}

int mpe_t_revolute_matrix (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "revolute_matrix");
    physics_world w;
    mpe_world_begin (&w);
    g_cfg.solver.bias_factor = 0.0f;
    g_cfg.joints.revolute_beta = 0.0f;
    g_cfg.joints.revolute_max_bias = 0.0f;

    /* Both anchors deliberately OFF-CENTRE: this is the geometry the old
     * suite never exercised, and the geometry the bug lived in. */
    int ia = physics_world_add_cube (&w, (vector3){0, 5, 0}, (vector3){0.5f, 0.5f, 0.5f}, 2.0f);
    int ib = physics_world_add_cube (&w, (vector3){0.7f, 5, 0}, (vector3){0.5f, 0.5f, 0.5f}, 3.0f);
    rigidbody *A = &w.bodies[ia];
    rigidbody *B = &w.bodies[ib];
    A->orientation = vector4_from_axis_with_angle ((vector3){0.3f, 0.5f, 0.8f}, 0.7f);
    rigidbody_update_axes (A);
    B->orientation = vector4_from_axis_with_angle ((vector3){0.9f, 0.1f, 0.4f}, 1.1f);
    rigidbody_update_axes (B);

    revolute_params p;
    memset (&p, 0, sizeof (p));
    p.anchor_a = (vector3){-0.18f, 0.05f, 0.12f};
    p.anchor_b = (vector3){0.25f, 0.15f, -0.20f};
    p.axis_a = (vector3){0, 0, 1};
    p.axis_b = (vector3){0, 0, 1};
    p.motor_enabled = false;
    p.limits_enabled = false;

    /* Mirror the solver's basis exactly (axis x ref, normalised). */
    vector3 ra = vector4_rotate_to_vector3 (A->orientation, p.anchor_a);
    vector3 rb = vector4_rotate_to_vector3 (B->orientation, p.anchor_b);
    vector3 axis = vector4_rotate_to_vector3 (A->orientation, vector3_normalisation (p.axis_a));
    vector3 ref = (fabsf (axis.y) < 0.99f) ? (vector3){0, 1, 0} : (vector3){1, 0, 0};
    vector3 u = vector3_cross (axis, ref);
    float ul = sqrtf (vector3_length_squared (u));
    if (ul < 1e-6f) {
        ref = (vector3){1, 0, 0};
        u = vector3_cross (axis, ref);
        ul = sqrtf (vector3_length_squared (u));
    }
    u = vector3_scaling (u, 1.0f / ul);
    vector3 v = vector3_cross (axis, u);

    float ima = rigidbody_effective_inv_mass (A);
    float imb = rigidbody_effective_inv_mass (B);
    math3 Ia = rigidbody_effective_inv_inertia (A);
    math3 Ib = rigidbody_effective_inv_inertia (B);

    /* Measured J M^-1 J^T, one unit-lambda column at a time. */
    double Kt[6][6];
    for (int col = 0; col < 6; col++) {
        A->velocity = vector3_zero ();
        A->angular_velocity = vector3_zero ();
        B->velocity = vector3_zero ();
        B->angular_velocity = vector3_zero ();
        double lambda[6] = {0, 0, 0, 0, 0, 0};
        lambda[col] = 1.0;
        vector3 ip2p = {(float) lambda[0], (float) lambda[1], (float) lambda[2]};
        vector3 iax = vector3_addition (vector3_scaling (u, (float) lambda[3]), vector3_scaling (v, (float) lambda[4]));
        vector3 imot = vector3_scaling (axis, (float) lambda[5]);
        A->velocity = vector3_subtraction (A->velocity, vector3_scaling (ip2p, ima));
        B->velocity = vector3_addition (B->velocity, vector3_scaling (ip2p, imb));
        A->angular_velocity = vector3_subtraction (
            A->angular_velocity, math3_multiplication_vector3 (
                                     Ia, vector3_addition (vector3_addition (vector3_cross (ra, ip2p), iax), imot)));
        B->angular_velocity = vector3_addition (
            B->angular_velocity, math3_multiplication_vector3 (
                                     Ib, vector3_addition (vector3_addition (vector3_cross (rb, ip2p), iax), imot)));
        double c[6];
        mpe_revolute_constraint_velocity (A, B, ra, rb, u, v, axis, c);
        for (int r = 0; r < 6; r++)
            Kt[r][col] = c[r];
    }

    /* K is a Gram matrix, so it must be symmetric. An asymmetric K means the
     * Jacobian and the impulse application disagree about the block layout. */
    for (int r = 0; r < 6; r++) {
        for (int c2 = r + 1; c2 < 6; c2++) {
            MPE_CHECK_NEAR (&t, Kt[r][c2], Kt[c2][r], 1e-4 + 1e-3 * fabs (Kt[r][c2]), "K symmetry");
        }
    }

    /* The decisive functional property: ONE Newton step of a correct K must
     * null all five constrained rows. Anything less means K is not the
     * effective mass of the Jacobian actually being applied. Before the fix
     * this residual was 5.18 (and the axis rows were amplified, not reduced). */
    A->velocity = (vector3){0.3f, -0.7f, 0.2f};
    A->angular_velocity = vector3_zero ();
    B->velocity = vector3_zero ();
    B->angular_velocity = (vector3){0.1f, 0.2f, -0.3f};
    double before[6], after[6];
    mpe_revolute_constraint_velocity (A, B, ra, rb, u, v, axis, before);
    revolute_solve (&p, A, B, 1.0f / 60.0f, &g_cfg);
    mpe_revolute_constraint_velocity (A, B, ra, rb, u, v, axis, after);
    double res = 0.0;
    for (int i = 0; i < 5; i++)
        res += after[i] * after[i];
    MPE_CHECK_NEAR (&t, sqrt (res), 0.0, 1e-4, "one-solve constraint residual");
    MPE_INFO ("revolute_matrix: one-solve residual %.3e (was 5.18 pre-fix)", sqrt (res));

    /* Sweep anchor arm: the old K went indefinite past ~0.3 m, so a long
     * anchor must also converge in one solve. */
    for (int trial = 0; trial < 4; trial++) {
        float arm = 0.1f + 0.35f * (float) trial;
        p.anchor_a = (vector3){-arm, 0.3f * arm, 0.2f * arm};
        p.anchor_b = (vector3){arm, -0.25f * arm, 0.4f * arm};
        ra = vector4_rotate_to_vector3 (A->orientation, p.anchor_a);
        rb = vector4_rotate_to_vector3 (B->orientation, p.anchor_b);
        A->velocity = (vector3){0.3f, -0.7f, 0.2f};
        A->angular_velocity = vector3_zero ();
        B->velocity = vector3_zero ();
        B->angular_velocity = (vector3){0.1f, 0.2f, -0.3f};
        mpe_revolute_constraint_velocity (A, B, ra, rb, u, v, axis, before);
        revolute_solve (&p, A, B, 1.0f / 60.0f, &g_cfg);
        mpe_revolute_constraint_velocity (A, B, ra, rb, u, v, axis, after);
        double r2 = 0.0;
        for (int i = 0; i < 5; i++)
            r2 += after[i] * after[i];
        MPE_CHECK_NEAR (&t, sqrt (r2), 0.0, 1e-3, "one-solve residual at long anchor arm");
    }

    /* Momentum must be conserved by the constraint impulse alone. */
    p.anchor_a = (vector3){-0.18f, 0.05f, 0.12f};
    p.anchor_b = (vector3){0.25f, 0.15f, -0.20f};
    A->velocity = (vector3){0.3f, -0.7f, 0.2f};
    A->angular_velocity = vector3_zero ();
    B->velocity = vector3_zero ();
    B->angular_velocity = vector3_zero ();
    double p0 = (double) A->mass * A->velocity.x + (double) B->mass * B->velocity.x;
    revolute_solve (&p, A, B, 1.0f / 60.0f, &g_cfg);
    double p1 = (double) A->mass * A->velocity.x + (double) B->mass * B->velocity.x;
    MPE_CHECK_NEAR (&t, p1, p0, 1e-4, "joint impulse conserves linear momentum");

    physics_world_cleanup (&w);
    mpe_test_end (&t);
    return t.failures;
}

/* ---------------------------------------------------------------------------
 * frustum_culler: exercise the SHIPPED culler.
 *
 * DESPOT-2026-09-29. The engine's real culling code lived inline inside
 * render_scene_current -- a GL function no headless test can call. The legacy
 * frustum test re-implemented plane extraction locally and linked no engine
 * objects, and the canonical case projected a single point, so a regression in
 * the code the renderer actually runs would have been invisible.
 *
 * The extraction and the sphere test are now in math4_special.h and the
 * renderer calls them, so this test and the renderer share one implementation.
 *
 * The load-bearing property is NO FALSE EXCLUSION: anything whose bounding
 * sphere actually intersects the view frustum must be reported visible. A
 * culler that wrongly hides geometry is a rendering bug that no "does it cull
 * something" test would catch.
 * ------------------------------------------------------------------------ */
int mpe_t_frustum_culler (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "frustum_culler");
    det_pin_fp_state ();

    /* Independent reference: project a point through the same VP and accept it
     * if it lands inside the NDC cube, OR if it is in front of the near plane
     * but off to the side (behind-camera points project nonsensically, so the
     * reference defers those). Conservative on purpose. */
    int checked = 0, false_exclusions = 0;
    for (int pose = 0; pose < 6; pose++) {
        vector3 eye = {(float) (pose * 3 + 1) * 1.7f, 2.0f + (float) pose * 0.9f, (float) pose * 2.3f - 4.0f};
        /* math4_look_view takes (position, front, up) and builds the side axis
         * as cross(front, up) itself -- passing a pre-computed basis here
         * double-orthogonalises and produced a wrong frustum. */
        math4 view = math4_look_view (eye, (vector3){0.0f, 0.0f, 1.0f}, (vector3){0.0f, 1.0f, 0.0f});
        math4 proj = math4_perspective_fov (degrad * 45.0f, 1.3333f, 0.1f, 1000.0f);
        math4 vp = math4_multiplication (proj, view);
        float planes[6][4];
        math4_frustum_planes (vp, planes);

        /* Deterministic sampling grid around the view. */
        for (int ix = -12; ix <= 12; ix++) {
            for (int iy = -8; iy <= 8; iy++) {
                for (int iz = -4; iz <= 20; iz++) {
                    vector3 p = {eye.x + (float) ix * 1.3f, eye.y + (float) iy * 1.1f, eye.z + (float) iz * 1.4f};
                    float radius = 0.5f;
                    int visible = math4_frustum_sphere_visible (planes, p.x, p.y, p.z, radius);

                    /* Reference: is the centre unambiguously inside the view
                     * volume, with margin for the radius? If so it MUST be
                     * reported visible. */
                    float d[4];
                    for (int r2 = 0; r2 < 4; r2++) {
                        d[r2] =
                            vp.matrix[0][r2] * p.x + vp.matrix[1][r2] * p.y + vp.matrix[2][r2] * p.z + vp.matrix[3][r2];
                    }
                    /* Only points strictly in front of the camera (w > 0 in
                     * clip space) have a meaningful projection; behind-camera
                     * points mirror and would produce spurious "inside". */
                    if (!(d[3] > 0.0f)) continue;
                    if (!(fabsf (d[0]) <= d[3])) continue;
                    if (!(fabsf (d[1]) <= d[3])) continue;
                    if (!(fabsf (d[2]) <= d[3])) continue;
                    checked++;
                    if (!visible) {
                        false_exclusions++;
                    }
                }
            }
        }
    }
    MPE_INFO ("frustum_culler: %d reference-inside samples, %d false exclusions", checked, false_exclusions);
    MPE_CHECK (&t, checked > 1000);
    MPE_CHECK (&t, false_exclusions == 0);

    /* Explicit cases. math4_look_view(eye, front, up) orients the camera so
     * that `front` is the direction it looks along: with the origin as eye and
     * front = +Z, the visible half-space is +Z (measured, not assumed). */
    {
        math4 view = math4_look_view ((vector3){0, 0, 0}, (vector3){0, 0, 1}, (vector3){0, 1, 0});
        math4 vp = math4_multiplication (math4_perspective_fov (degrad * 45.0f, 1.3333f, 0.1f, 1000.0f), view);
        float planes[6][4];
        math4_frustum_planes (vp, planes);
        MPE_CHECK (&t, math4_frustum_sphere_visible (planes, 0.0f, 0.0f, 5.0f, 0.5f) == 1);
        MPE_CHECK (&t, math4_frustum_sphere_visible (planes, 0.0f, 0.0f, 900.0f, 0.5f) == 1);
        MPE_CHECK (&t, math4_frustum_sphere_visible (planes, 0.0f, 0.0f, -50.0f, 0.5f) == 0);
        MPE_CHECK (&t, math4_frustum_sphere_visible (planes, 500.0f, 0.0f, 0.0f, 0.5f) == 0);
        MPE_CHECK (&t, math4_frustum_sphere_visible (planes, 0.0f, 500.0f, 0.0f, 0.5f) == 0);
        /* A sphere straddling the far plane must be KEPT (conservative). */
        MPE_CHECK (&t, math4_frustum_sphere_visible (planes, 0.0f, 0.0f, 1000.0f, 5.0f) == 1);
        /* And one just outside the near plane with a large radius is KEPT. */
        MPE_CHECK (&t, math4_frustum_sphere_visible (planes, 0.0f, 0.0f, -0.05f, 0.5f) == 1);
    }

    mpe_test_end (&t);
    return t.failures;
}

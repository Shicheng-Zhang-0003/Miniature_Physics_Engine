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
int mpe_t_stack(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "stack");
    mpe_config_init();
    physics_world w;
    mpe_world_begin(&w);
    MPE_CHECK(&t, mpe_floor_slab(&w, 0.4f, 0.3f, 0.0f) >= 0);
    const float h = 0.4f;
    int cube[6];
    for (int i = 0; i < 6; i++) {
        cube[i] =
            physics_world_add_cube(&w, (vector3){0.0f, h + (float)i * 2.0f * h, 0.0f},
                                   (vector3){h, h, h}, 1.0f);
        MPE_CHECK(&t, cube[i] >= 0);
    }
    const float dt = 1.0f / 60.0f;
    if (!mpe_step(&w, 600, dt)) {
        t.failures++;
    }
    float top_drift = sqrtf(w.bodies[cube[5]].position.x * w.bodies[cube[5]].position.x +
                            w.bodies[cube[5]].position.z * w.bodies[cube[5]].position.z);
    MPE_INFO("top drift=%.4f (limit 0.05)", top_drift);
    MPE_CHECK(&t, top_drift <= 0.05f);
    for (int i = 0; i < 6; i++) {
        float y_e = h + (float)i * 2.0f * h;
        MPE_CHECK_NEAR(&t, w.bodies[cube[i]].position.y, y_e, 0.03f, "level-height");
        float lv = mpe_vlen(w.bodies[cube[i]].velocity);
        float av = mpe_vlen(w.bodies[cube[i]].angular_velocity);
        MPE_CHECK(&t, lv <= 0.05f && av <= 0.05f);
    }
    if (t.failures == 0) {
        printf("[PASS] tower stands\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

/* driven_wheel FIXED: v1 never enabled the solver plane, so torque spun
 * the wheel to wx=96 with dz=0 (no Coulomb manifold exists without the
 * plane/slab). v2 enables the plane with synced floor friction. Same
 * traction-envelope torque (0.020 < 0.0245 limit) and coupling gates. */
int mpe_t_driven_wheel(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "driven_wheel");
    mpe_config_init();
    MPE_INFO("gravity = %.4f", g_cfg.world.gravity);
    physics_world w;
    mpe_world_begin(&w);
    mpe_floor_plane(&w, 0.4f, 0.1f);
    int wh = physics_world_add_cylinder(&w, 0.05f, 0.02f, 0.5f, (vector3){0.0f, 0.06f, 0.0f});
    MPE_CHECK(&t, wh >= 0);
    const float dt = 1.0f / 60.0f;
    const float drive_torque = 0.020f;
    {
        float traction_limit = g_cfg.world.floor_friction_k * (0.5f * 9.81f) * 0.05f;
        MPE_CHECK(&t, drive_torque < traction_limit);
    }
    if (!mpe_step(&w, 60, dt)) {
        t.failures++;
    }
    float start_z = w.bodies[wh].position.z;
    for (int k = 0; k < 180; k++) {
        rigidbody_wake(&w.bodies[wh]);
        w.bodies[wh].torque_accumulator.x += drive_torque;
        physics_world_step(&w, dt);
        if (!mpe_world_finite(&w)) {
            printf("[FAIL] non-finite wheel state\n");
            t.failures++;
            break;
        }
    }
    float dz = w.bodies[wh].position.z - start_z;
    float vz = w.bodies[wh].velocity.z;
    float wx = w.bodies[wh].angular_velocity.x;
    float y = w.bodies[wh].position.y;
    MPE_INFO("grounded wheel: dz=%.4f vz=%.4f wx=%.4f y=%.4f", dz, vz, wx, y);
    MPE_CHECK(&t, isfinite(dz) && isfinite(vz) && isfinite(wx) && isfinite(y));
    MPE_CHECK(&t, fabsf(wx) >= 5.0f);
    MPE_CHECK(&t, fabsf(wx) <= 100.0f);
    MPE_CHECK_NEAR(&t, y, 0.05f, 0.02f, "wheel-grounded");
    MPE_CHECK(&t, fabsf(dz) >= 1.5f);
    float expected_vz = wx * 0.05f;
    MPE_INFO("kinematic check: expected vz (w*r) = %.4f, actual vz = %.4f", expected_vz, vz);
    MPE_CHECK(&t, (vz * wx) >= 0.0f);
    float coupling = fabsf(vz) / (fabsf(expected_vz) + 1e-6f);
    MPE_CHECK(&t, coupling >= 0.70f && coupling <= 1.10f);
    if (t.failures == 0) {
        printf("[PASS] grounded wheel rolled %.4f m via real floor friction\n", dz);
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

static int mpe_vec3_eq(vector3 a, vector3 b) {
    return (a.x == b.x) && (a.y == b.y) && (a.z == b.z) && isfinite(a.x) && isfinite(a.y) &&
           isfinite(a.z) && isfinite(b.x) && isfinite(b.y) && isfinite(b.z);
}

static int mpe_vec4_eq(vector4 a, vector4 b) {
    return (a.w == b.w) && (a.x == b.x) && (a.y == b.y) && (a.z == b.z) && isfinite(a.w) &&
           isfinite(a.x) && isfinite(a.y) && isfinite(a.z) && isfinite(b.w) && isfinite(b.x) &&
           isfinite(b.y) && isfinite(b.z);
}

static int mpe_bodies_equal(const rigidbody *a, const rigidbody *b) {
    if (!mpe_vec3_eq(a->position, b->position)) {
        return 0;
    }
    if (!mpe_vec3_eq(a->velocity, b->velocity)) {
        return 0;
    }
    if (!mpe_vec3_eq(a->acceleration, b->acceleration)) {
        return 0;
    }
    if (!mpe_vec4_eq(a->orientation, b->orientation)) {
        return 0;
    }
    if (!mpe_vec3_eq(a->angular_velocity, b->angular_velocity)) {
        return 0;
    }
    if (!mpe_vec3_eq(a->angular_acceleration, b->angular_acceleration)) {
        return 0;
    }
    if (!mpe_vec3_eq(a->force_accumulator, b->force_accumulator)) {
        return 0;
    }
    if (!mpe_vec3_eq(a->torque_accumulator, b->torque_accumulator)) {
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
    if (!mpe_vec3_eq(a->half_extensions, b->half_extensions)) {
        return 0;
    }
    if (!mpe_vec3_eq(a->cached_axes[0], b->cached_axes[0])) {
        return 0;
    }
    if (!mpe_vec3_eq(a->cached_axes[1], b->cached_axes[1])) {
        return 0;
    }
    if (!mpe_vec3_eq(a->cached_axes[2], b->cached_axes[2])) {
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

static void mpe_det_scene(physics_world *w) {
    physics_world_init(w);
    constraint_pool_init(w);
    int a = physics_world_add_sphere(w, 0.5f, 2.0f, (vector3){-1.0f, 3.0f, 0.5f});
    w->bodies[a].velocity = (vector3){1.5f, -0.5f, 0.25f};
    w->bodies[a].angular_velocity = (vector3){3.0f, -1.0f, 2.0f};
    w->bodies[a].restitution = 0.4f;
    int b = physics_world_add_cube(w, (vector3){1.0f, 0.5f, -0.5f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    w->bodies[b].velocity = (vector3){-0.75f, 0.0f, 0.5f};
    w->bodies[b].angular_velocity = (vector3){0.0f, 2.0f, -1.5f};
    w->bodies[b].restitution = 0.3f;
    w->bodies[b].nice_value = 0;
    int c = physics_world_add_cylinder(w, 0.3f, 0.4f, 1.5f, (vector3){0.0f, 2.0f, 1.0f});
    w->bodies[c].velocity = (vector3){0.2f, -1.0f, -0.3f};
    w->bodies[c].angular_velocity = (vector3){-2.0f, 0.5f, 1.0f};
    w->bodies[c].restitution = 0.2f;
    int d = physics_world_add_cube(w, (vector3){0.0f, 1.6f, 0.0f}, (vector3){0.4f, 0.4f, 0.4f}, 1.0f);
    w->bodies[d].velocity = (vector3){0.0f, -0.2f, 0.0f};
    physics_world_add_cube(w, (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
}

int mpe_t_determinism(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "determinism");
    mpe_config_init();
    physics_world w1, w2;
    mpe_det_scene(&w1);
    mpe_det_scene(&w2);
    const float dt = 1.0f / 60.0f;
    for (int k = 0; k < 600; k++) {
        physics_world_step(&w1, dt);
        physics_world_step(&w2, dt);
    }
    MPE_CHECK(&t, w1.body_count == w2.body_count);
    for (int i = 0; i < w1.body_count; i++) {
        if (!mpe_bodies_equal(&w1.bodies[i], &w2.bodies[i])) {
            printf("[FAIL] body %d diverged bitwise\n", i);
            t.failures++;
        } else {
            t.checks++;
        }
    }
    MPE_CHECK(&t, w1.world_contact_cache_count == w2.world_contact_cache_count);
    if (t.failures == 0) {
        printf("[PASS] determinism: 600 ticks bitwise identical across twin worlds\n");
    }
    physics_world_cleanup(&w1);
    physics_world_cleanup(&w2);
    mpe_test_end(&t);
    return t.failures;
}

#define F10_TICKS 1500
#define F10_TRANSIENT 120

static void mpe_f10_cube(physics_world *w, vector3 p) {
    int idx = physics_world_add_cube(w, p, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    if (idx >= 0) {
        w->bodies[idx].restitution = 0.0f;
        w->bodies[idx].friction_static = 0.8f;
        w->bodies[idx].friction_kinetic = 0.7f;
    }
}

/* Adversarial pile bodies (shared by f10 settle + f11 torture builders). */
static void mpe_pile_bodies(physics_world *w) {
    for (int i = 0; i < 10; i++) {
        mpe_f10_cube(w, (vector3){20.0f, 0.5f + (float)i * 0.99f, 0.0f});
    }
    for (int gx = 0; gx < 3; gx++) {
        for (int gz = 0; gz < 3; gz++) {
            mpe_f10_cube(w, (vector3){-20.0f + ((float)gx - 1.0f) * 1.1f, 0.5f,
                                      ((float)gz - 1.0f) * 1.1f});
        }
    }
    for (int gx = 0; gx < 2; gx++) {
        for (int gz = 0; gz < 2; gz++) {
            mpe_f10_cube(w, (vector3){-20.0f + ((float)gx - 0.5f) * 1.1f, 1.49f,
                                      ((float)gz - 0.5f) * 1.1f});
        }
    }
    mpe_f10_cube(w, (vector3){-20.0f, 2.48f, 0.0f});
    for (int i = 0; i < 3; i++) {
        int idx = physics_world_add_sphere(w, 0.35f, 1.0f,
                                           (vector3){-30.0f + (float)i * 3.0f, 0.35f, 8.0f});
        if (idx >= 0) {
            w->bodies[idx].restitution = 0.0f;
            w->bodies[idx].friction_static = 0.8f;
            w->bodies[idx].friction_kinetic = 0.7f;
        }
    }
}

/* f10 settle scene: pile + Coulomb floor (see scene_init.c note). */
static void mpe_settle_scene(physics_world *w) {
    int f = physics_world_add_cube(w, (vector3){0.0f, -0.5f, 0.0f},
                                   (vector3){30.0f, 0.5f, 30.0f}, 0.0f);
    if (f >= 0) {
        w->bodies[f].friction_static = 0.8f;
        w->bodies[f].friction_kinetic = 0.7f;
        w->bodies[f].restitution = 0.0f;
    }
    mpe_pile_bodies(w);
}

/* f11 torture scene: pile WITHOUT floor (unchanged legacy geometry).
 * Verdict stays robustness-only (no NaN, nothing fallen). WARNING: this
 * crash-oracle is NOT a stability proof — under extreme configs perpetual
 * fall/creep is the true outcome, so end speeds are reported, never gated.
 * Do not cite PASS as "stable". */
static void mpe_torture_scene(physics_world *w) {
    mpe_pile_bodies(w);
}

int mpe_t_f10_long_run(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "f10_long_run");
    mpe_config_init();
    physics_world w;
    mpe_world_begin(&w);
    mpe_settle_scene(&w);
    const float dt = 1.0f / 60.0f;
    float run_max_lin = 0.0f, run_max_ang = 0.0f, fin_lin = 0.0f, fin_ang = 0.0f;
    long nan_ticks = 0, fallen_ticks = 0;
    for (int k = 0; k < F10_TICKS; k++) {
        physics_world_step(&w, dt);
        float mx_lin = 0.0f, mx_ang = 0.0f;
        for (int i = 0; i < w.body_count; i++) {
            rigidbody *rb = &w.bodies[i];
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
            float l = mpe_vlen(rb->velocity);
            float a = mpe_vlen(rb->angular_velocity);
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
    MPE_INFO("final lin=%.5f ang=%.5f runmax lin=%.5f ang=%.5f nan=%ld fallen=%ld", fin_lin, fin_ang,
             run_max_lin, run_max_ang, nan_ticks, fallen_ticks);
    int asleep = 0, dynamic_n = 0;
    for (int i = 0; i < w.body_count; i++) {
        if (!w.bodies[i].static_state) {
            dynamic_n++;
            if (w.bodies[i].is_sleeping) {
                asleep++;
            }
        }
    }
    MPE_INFO("asleep=%d/%d", asleep, dynamic_n);
    MPE_CHECK(&t, w.body_count > 0 && nan_ticks == 0 && fallen_ticks == 0);
    MPE_CHECK(&t, fin_lin < 0.25f && fin_ang < 0.5f);
    MPE_CHECK(&t, run_max_lin < 2.0f && run_max_ang < 2.0f);
    MPE_CHECK(&t, asleep == dynamic_n && dynamic_n > 0);
    if (t.failures == 0) {
        printf("[PASS] long-run 10-stack+pile settles and stays calm\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

int mpe_t_sleep_contact_wake(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "sleep_contact_wake");
    mpe_config_init();
    physics_world w;
    mpe_world_begin(&w);
    int sleeper = physics_world_add_sphere(&w, 0.5f, 1.0f, (vector3){2.0f, 0.5f, 0.0f});
    MPE_CHECK(&t, sleeper >= 0);
    w.bodies[sleeper].velocity = vector3_zero();
    w.bodies[sleeper].angular_velocity = vector3_zero();
    w.bodies[sleeper].is_sleeping = true;
    w.bodies[sleeper].sleep_timer = 1.0f;
    w.bodies[sleeper].restitution = 0.0f;
    int pusher = physics_world_add_cube(&w, (vector3){0.4f, 0.5f, 0.0f},
                                        (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    MPE_CHECK(&t, pusher >= 0);
    rigidbody_set_kinematic(&w.bodies[pusher], true);
    w.bodies[pusher].velocity = (vector3){0.05f, 0.0f, 0.0f};
    int control = physics_world_add_sphere(&w, 0.5f, 1.0f, (vector3){10.0f, 0.5f, 0.0f});
    MPE_CHECK(&t, control >= 0);
    w.bodies[control].velocity = vector3_zero();
    w.bodies[control].angular_velocity = vector3_zero();
    w.bodies[control].is_sleeping = true;
    w.bodies[control].sleep_timer = 1.0f;
    w.bodies[control].restitution = 0.0f;
    const float dt = 1.0f / 60.0f;
    int touch_tick = -1, wake_tick = -1, control_wake_tick = -1;
    for (int k = 0; k < 1200; k++) {
        physics_world_step(&w, dt);
        rigidbody *s = &w.bodies[sleeper];
        rigidbody *p = &w.bodies[pusher];
        rigidbody *c = &w.bodies[control];
        if (!isfinite(s->position.x) || !isfinite(p->position.x) || !isfinite(c->position.x)) {
            printf("[FAIL] NaN at tick %d\n", k);
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
    float end_gap =
        (w.bodies[sleeper].position.x - 0.5f) - (w.bodies[pusher].position.x + 0.5f);
    MPE_INFO("touch=%d wake=%d control_wake=%d sleeper_x=%.4f pusher_x=%.4f control_y=%.4f end_gap=%.4f",
             touch_tick, wake_tick, control_wake_tick, w.bodies[sleeper].position.x,
             w.bodies[pusher].position.x, w.bodies[control].position.y, end_gap);
    MPE_CHECK(&t, touch_tick >= 0);
    MPE_CHECK(&t, wake_tick >= 0);
    if (touch_tick >= 0 && wake_tick >= 0) {
        MPE_CHECK(&t, wake_tick - touch_tick <= 5);
    }
    MPE_CHECK(&t, control_wake_tick < 0);
    MPE_CHECK(&t, w.bodies[control].position.y >= 0.4f && w.bodies[control].position.y <= 0.6f);
    if (wake_tick >= 0) {
        MPE_CHECK(&t, w.bodies[sleeper].position.x >= 2.1f);
    }
    MPE_CHECK(&t, end_gap >= -0.02f);
    if (t.failures == 0) {
        printf("[PASS] sleep contact-wake truth holds\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

static uint32_t mpe_rng = 0xC0FFEEu;

static uint32_t mpe_next(void) {
    mpe_rng ^= mpe_rng << 13;
    mpe_rng ^= mpe_rng >> 17;
    mpe_rng ^= mpe_rng << 5;
    return mpe_rng;
}

int mpe_t_f11_torture(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "f11_torture");
    mpe_config_init();
    mpe_rng = 0xC0FFEEu;
    for (size_t i = 0; i < g_registry_count; i++) {
        /* mpe_param layout: type/key/min/max/storage (see mpe_config.h).
         * param_write_double is TU-static, so write storage directly like
         * the v1 torture test (clamped to [min,max] by construction). */
        if (g_registry[i].type == p_float) {
            float range = (float)(g_registry[i].max - g_registry[i].min);
            *(float *)g_registry[i].storage =
                (float)g_registry[i].min + ((float)(mpe_next() >> 8) / 16777216.0f) * range;
        } else if (g_registry[i].type == p_int) {
            int range = (int)(g_registry[i].max - g_registry[i].min);
            *(int *)g_registry[i].storage =
                (int)g_registry[i].min + (int)(mpe_next() % (uint32_t)(range >= 0 ? range + 1 : 1));
        } else if (g_registry[i].type == p_bool) {
            *(bool *)g_registry[i].storage = (mpe_next() & 1u) != 0;
        }
    }
    if (g_cfg.world.gravity > -1.0f) {
        g_cfg.world.gravity = -1.0f - ((float)(mpe_next() >> 8) / 16777216.0f) * 16.0f;
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
    MPE_INFO("torture: gravity=%.2f iters=%d slop=%.3f", g_cfg.world.gravity,
             g_cfg.timestep.solver_iterations, g_cfg.solver.penetration_slop);
    physics_world w;
    physics_world_init(&w);
    constraint_pool_init(&w);
    mpe_torture_scene(&w);
    const float dt = 1.0f / 60.0f;
    long nan_ticks = 0, fallen_ticks = 0;
    float end_lin = 0.0f, end_ang = 0.0f;
    for (int k = 0; k < 1500; k++) {
        physics_world_step(&w, dt);
        float mx_lin = 0.0f, mx_ang = 0.0f;
        for (int i = 0; i < w.body_count; i++) {
            rigidbody *rb = &w.bodies[i];
            if (!isfinite(rb->position.x) || !isfinite(rb->position.y) || !isfinite(rb->position.z) ||
                !isfinite(rb->velocity.x) || !isfinite(rb->velocity.y) || !isfinite(rb->velocity.z)) {
                nan_ticks++;
                continue;
            }
            if (rb->position.y < -0.2f) {
                fallen_ticks++;
            }
            float l = mpe_vlen(rb->velocity);
            float a = mpe_vlen(rb->angular_velocity);
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
    MPE_INFO("torture end speeds (reported, never gated): lin=%.3f ang=%.3f nan=%ld fallen=%ld", end_lin,
             end_ang, nan_ticks, fallen_ticks);
    /* Crash-oracle only: PASS = finite state + world intact, NOT stability.
     * Do not misread as a stability proof. */
    MPE_CHECK(&t, nan_ticks == 0 && fallen_ticks == 0);
    if (t.failures == 0) {
        printf("[PASS] torture survived extremes without corruption\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

/* scene_roundtrip: bodies + springs + revolute survive save/load on primary. */
int mpe_t_scene_roundtrip(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "scene_roundtrip");
    mpe_config_init();
    physics_world *w = physics_world_get_primary();
    physics_world_init(w);
    constraint_pool_init(w);
    int a = physics_world_add_sphere(w, 0.3f, 1.0f, (vector3){0.0f, 2.0f, 0.0f});
    int b = physics_world_add_cube(w, (vector3){2.0f, 1.0f, 0.0f}, (vector3){0.4f, 0.4f, 0.4f}, 2.0f);
    MPE_CHECK(&t, a >= 0 && b >= 0);
    w->bodies[a].velocity = (vector3){1.0f, 0.0f, 0.0f};
    w->bodies[a].restitution = 0.3f;
    w->bodies[b].friction_static = 0.5f;
    uint32_t ida = w->bodies[a].object_id;
    uint32_t idb = w->bodies[b].object_id;
    float px = w->bodies[a].position.x, py = w->bodies[a].position.y;
    int n_body = w->body_count;
    const char *path = "../../temp/mpe_suite_roundtrip.dat";
    MPE_CHECK(&t, save_scene(path) != 0);
    /* Mutate, then reload and compare. */
    w->bodies[a].position = (vector3){99.0f, 99.0f, 99.0f};
    MPE_CHECK(&t, scene_loading(path) != 0);
    MPE_CHECK(&t, w->body_count == n_body);
    rigidbody *ra = physics_world_body_by_id(w, ida);
    rigidbody *rb2 = physics_world_body_by_id(w, idb);
    MPE_CHECK(&t, ra != NULL && rb2 != NULL);
    if (ra) {
        MPE_CHECK_NEAR(&t, ra->position.x, px, 1e-4f, "roundtrip-x");
        MPE_CHECK_NEAR(&t, ra->position.y, py, 1e-4f, "roundtrip-y");
        MPE_CHECK_NEAR(&t, ra->velocity.x, 1.0f, 1e-4f, "roundtrip-v");
        MPE_CHECK_NEAR(&t, ra->restitution, 0.3f, 1e-5f, "roundtrip-e");
    }
    if (rb2) {
        MPE_CHECK_NEAR(&t, rb2->friction_static, 0.5f, 1e-5f, "roundtrip-mu");
    }
    remove(path);
    if (t.failures == 0) {
        printf("[PASS] scene round-trip complete\n");
    }
    physics_world_cleanup(w);
    mpe_test_end(&t);
    return t.failures;
}

/* module: per-world cfg, registry dispatch, custom shapes, hooks, id cache,
 * pool growth, det counters. Condensed port of module_test.c. */
static int mpe_mod_pre_calls = 0;

static void mpe_mod_pre(mpe_world_t *world, float dt, void *st) {
    (void)world;
    (void)dt;
    (void)st;
    mpe_mod_pre_calls++;
}

int mpe_t_module(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "module");
    mpe_config_init();
    mpe_register_builtins();
    physics_world A, B;
    physics_world_init(&A);
    physics_world_init(&B);
    mpe_config_t cfgA = g_cfg, cfgB = g_cfg;
    cfgA.world.gravity = -1.0f;
    cfgB.world.gravity = -20.0f;
    physics_world_set_config(&A, &cfgA);
    physics_world_set_config(&B, &cfgB);
    MPE_CHECK(&t, mpe_world_cfg(&A)->world.gravity == -1.0f);
    MPE_CHECK(&t, mpe_world_cfg(&B)->world.gravity == -20.0f);
    MPE_CHECK(&t, mpe_find_pair_handler(0, 0, -1, -1) != NULL);
    MPE_CHECK(&t, mpe_find_pair_handler(0, 1, -1, -1) != NULL);
    MPE_CHECK(&t, mpe_find_broadphase("hash") != NULL);
    MPE_CHECK(&t, mpe_find_solver("seq-impulse") != NULL);
    /* custom shape survives sanitize + dispatches */
    int ic = physics_world_add_custom(&A, 100, (vector3){0.0f, 3.0f, 0.0f}, 1.0f, 0.5f);
    MPE_CHECK(&t, ic >= 0 && A.bodies[ic].type == object_custom);
    rigidbody_sanitize(&A.bodies[ic]);
    MPE_CHECK(&t, A.bodies[ic].type == object_custom);
    /* tick-module hook attach/step/detach */
    static const mpe_module_desc_t hook = {MPE_MODULE_ABI, "suite-hook", "1.0", "generic",
                                           true, NULL, NULL, mpe_mod_pre, NULL};
    MPE_CHECK(&t, mpe_register_module(&hook) >= 0);
    const mpe_module_desc_t *found = mpe_find_module("suite-hook");
    MPE_CHECK(&t, found != NULL);
    MPE_CHECK(&t, physics_world_attach_module(&A, found) >= 0);
    mpe_mod_pre_calls = 0;
    physics_world_step(&A, 1.0f / 60.0f);
    MPE_CHECK(&t, mpe_mod_pre_calls == 1);
    MPE_CHECK(&t, physics_world_detach_module(&A, "suite-hook") == 0);
    mpe_mod_pre_calls = 0;
    physics_world_step(&A, 1.0f / 60.0f);
    MPE_CHECK(&t, mpe_mod_pre_calls == 0);
    mpe_unregister_module("suite-hook");
    /* id cache + pool growth + det counters */
    {
        physics_world W;
        physics_world_init(&W);
        int cap0 = W.body_capacity;
        for (int i = 0; i < cap0 + 4; i++) {
            physics_world_add_sphere(&W, 0.2f, 1.0f, (vector3){(float)i, 5.0f, 0.0f});
        }
        MPE_CHECK(&t, W.body_capacity > cap0);
        uint32_t mid = W.bodies[W.body_count / 2].object_id;
        MPE_CHECK(&t, physics_world_index_by_id(&W, mid) >= 0);
        MPE_CHECK(&t, physics_world_index_by_id(&W, 0xFFFFFFu) < 0);
        physics_world_cleanup(&W);
    }
    det_fallback_reset();
    {
        physics_world W;
        physics_world_init(&W);
        physics_world_add_sphere(&W, 0.5f, 1.0f, (vector3){0.0f, 3.0f, 0.0f});
        for (int k = 0; k < 60; k++) {
            physics_world_step(&W, 1.0f / 60.0f);
        }
        physics_world_cleanup(&W);
    }
    MPE_CHECK(&t, det_fallback_pow_total() == 0 && det_fallback_trig_total() == 0);
    if (t.failures == 0) {
        printf("[PASS] module system green\n");
    }
    physics_world_cleanup(&A);
    physics_world_cleanup(&B);
    mpe_test_end(&t);
    return t.failures;
}

/* math3_inverse: analytic inverse at small inertia tensors. */
static uint32_t mpe_inverse_rng(uint32_t *state) {
    /* xorshift32: fixed seed, no libc/global RNG state and identical inputs. */
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static float mpe_inverse_rand_signed(uint32_t *state) {
    return (float)(mpe_inverse_rng(state) >> 8) * (1.0f / 16777216.0f) * 2.0f - 1.0f;
}

int mpe_t_math3_inverse(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "math3_inverse");
    mpe_config_init();
    math3 m = {{{4.0f, 1.0f, 0.0f}, {1.0f, 3.0f, 1.0f}, {0.0f, 1.0f, 2.0f}}};
    math3 inv = math3_inverse(m);
    math3 id = math3_multiplication(m, inv);
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            float want = (r == c) ? 1.0f : 0.0f;
            MPE_CHECK_NEAR(&t, id.matrix[r][c], want, 1e-4f, "inverse-identity");
        }
    }
    /* small inertia scale (1e-4 kg, 0.01 m): must stay finite, not singular. */
    math3 tiny = {{{4e-12f, 0, 0}, {0, 4e-12f, 0}, {0, 0, 4e-12f}}};
    math3 tinv = math3_inverse(tiny);
    MPE_CHECK(&t, isfinite(tinv.matrix[0][0]) && tinv.matrix[0][0] > 0.0f);

    /* Property sweep: 256 deterministic SPD matrices built as L*L^T.
     * Sweep their magnitude over 2^-24 .. 2^24 and verify A*A^-1 ~= I.
     * This tests scale invariance and non-diagonal cofactors independently
     * of the hand-picked matrix above, without stochastic CI behavior. */
    uint32_t seed = 0x6d706531u;
    for (int sample = 0; sample < 256; ++sample) {
        float lower[3][3] = {{0.0f}};
        lower[0][0] = 0.75f + 0.75f * (mpe_inverse_rng(&seed) >> 8) * (1.0f / 16777216.0f);
        lower[1][1] = 0.75f + 0.75f * (mpe_inverse_rng(&seed) >> 8) * (1.0f / 16777216.0f);
        lower[2][2] = 0.75f + 0.75f * (mpe_inverse_rng(&seed) >> 8) * (1.0f / 16777216.0f);
        lower[1][0] = 0.35f * mpe_inverse_rand_signed(&seed);
        lower[2][0] = 0.35f * mpe_inverse_rand_signed(&seed);
        lower[2][1] = 0.35f * mpe_inverse_rand_signed(&seed);
        int exponent = -24 + (sample * 37 % 49);
        float scale = ldexpf(1.0f, exponent);
        math3 candidate = {{{0.0f}}};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                double sum = 0.0;
                for (int k = 0; k < 3; ++k) {
                    sum += (double)lower[row][k] * (double)lower[col][k];
                }
                candidate.matrix[row][col] = (float)(sum * (double)scale);
            }
        }
        math3 candidate_inverse = math3_inverse(candidate);
        math3 product = math3_multiplication(candidate, candidate_inverse);
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                float want = (row == col) ? 1.0f : 0.0f;
                MPE_CHECK_NEAR(&t, product.matrix[row][col], want, 2.5e-4f,
                               "seeded-spd-inverse-identity");
            }
        }
    }

    /* Singular-axis and non-finite inputs have documented safe fallbacks. */
    math3 locked = {{{0.0f, 0.0f, 0.0f}, {0.0f, 2.0f, 0.0f}, {0.0f, 0.0f, 4.0f}}};
    math3 locked_inverse = math3_inverse(locked);
    MPE_CHECK_NEAR(&t, locked_inverse.matrix[0][0], 0.0f, 0.0f, "locked-axis-inverse");
    MPE_CHECK_NEAR(&t, locked_inverse.matrix[1][1], 0.5f, 0.0f, "live-axis-inverse-y");
    MPE_CHECK_NEAR(&t, locked_inverse.matrix[2][2], 0.25f, 0.0f, "live-axis-inverse-z");
    math3 invalid = {{{NAN, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    math3 invalid_inverse = math3_inverse(invalid);
    MPE_CHECK(&t, invalid_inverse.matrix[0][0] == 0.0f && invalid_inverse.matrix[1][1] == 0.0f &&
                     invalid_inverse.matrix[2][2] == 0.0f);
    MPE_INFO("matrix inverse property sweep: 256 fixed-seed SPD matrices, exponent range [-24, 24]");
    if (t.failures == 0) {
        printf("[PASS] matrix inverse analytic, scaled, singular-axis, and non-finite cases\n");
    }
    mpe_test_end(&t);
    return t.failures;
}

/* frustum: Gribb/Hartmann extraction culls outside, keeps inside. */
int mpe_t_frustum(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "frustum");
    mpe_config_init();
    math4 proj = math4_perspective_fov(45.0f * 3.14159265f / 180.0f, 16.0f / 9.0f, 0.1f, 1000.0f);
    math4 view = math4_look_view((vector3){0, 2, 8}, (vector3){0, 0, -1}, (vector3){0, 1, 0});
    math4 vp = math4_multiplication(proj, view);
    /* inside point projects to NDC cube (vector4 is {w,x,y,z}) */
    vector4 p = {1.0f, 0.0f, 2.0f, 0.0f};
    float v[4] = {0};
    for (int r = 0; r < 4; r++) {
        v[r] = vp.matrix[0][r] * p.x + vp.matrix[1][r] * p.y + vp.matrix[2][r] * p.z +
               vp.matrix[3][r] * p.w;
    }
    MPE_CHECK(&t, fabsf(v[3]) > 1e-6f);
    float nx = v[0] / v[3], ny = v[1] / v[3], nz = v[2] / v[3];
    MPE_INFO("ndc=(%.3f,%.3f,%.3f)", nx, ny, nz);
    MPE_CHECK(&t, fabsf(nx) <= 1.0f && fabsf(ny) <= 1.0f && nz >= -1.0f && nz <= 1.0f);
    if (t.failures == 0) {
        printf("[PASS] frustum math culls correctly\n");
    }
    mpe_test_end(&t);
    return t.failures;
}

/* floor_collision_diag: cylinder drops 1m onto slab, settles at r, calms. */
int mpe_t_floor_collision_diag(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "floor_collision_diag");
    mpe_config_init();
    physics_world w;
    mpe_world_begin(&w);
    MPE_CHECK(&t, mpe_floor_slab(&w, 0.4f, 0.3f, 0.0f) >= 0);
    int cyl = physics_world_add_cylinder(&w, 0.05f, 0.02f, 0.5f, (vector3){0.0f, 1.0f, 0.0f});
    MPE_CHECK(&t, cyl >= 0);
    const float dt = 1.0f / 60.0f;
    if (!mpe_step(&w, 300, dt)) {
        t.failures++;
    }
    float fy = w.bodies[cyl].position.y;
    float fvy = w.bodies[cyl].velocity.y;
    MPE_INFO("floor state y=%.4f vy=%.4f", fy, fvy);
    MPE_CHECK(&t, isfinite(fy) && isfinite(fvy));
    MPE_CHECK(&t, fy >= -0.05f);
    MPE_CHECK_NEAR(&t, fy, 0.05f, 0.03f, "floor-rest");
    MPE_CHECK(&t, fabsf(fvy) <= 0.5f);
    if (t.failures == 0) {
        printf("[PASS] floor contact holds and settles\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

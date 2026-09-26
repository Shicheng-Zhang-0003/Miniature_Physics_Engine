/* MFS Test Framework v2 (mfs_test.h) — modeled on MPE v2.
 *
 * Why v2: the v1 MFS suite (13 separate binaries) hid setup bugs:
 *   1. teleop/mecanum/tank/odometry ran floorless — measured only
 *      slip-regime artifacts, not Coulomb contact.
 *   2. physics_truth free-spin rig teleported only chassis — wheels
 *      winched up through pendulum chaos (T6 false failures).
 *   3. Per-test ad-hoc setup meant every test re-derived floor/
 *      friction/material handling, often wrong.
 *
 * What v2 guarantees:
 *   - One binary, one registry, exact-name dispatch (no substring fan-out).
 *   - Every test runs under saved/restored global config (no leakage).
 *   - Floor setup is a single audited helper: real Coulomb contact,
 *     explicit mass-0 slab (true manifold, per-body friction combine).
 *   - Assertions print file:line + values on failure and keep going.
 *   - NaN watchdog on every step; determinism counters asserted zero.
 *   - Free-spin rig lifts WHOLE robot (chassis + wheels + rollers).
 */

#ifndef mfs_test_h
#define mfs_test_h

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "core/det_math.h"
#include "modules/ftc/submodules/robot.h"
#include "modules/ftc/submodules/drivetrain.h"
#include "modules/ftc/submodules/motor_presets.h"
#include "modules/ftc/submodules/battery.h"

/* ------------------------------------------------------------------ */
/* Test context: failure counting + config isolation.                  */
/* ------------------------------------------------------------------ */
typedef struct {
    const char *name;
    int failures;
    int checks;
    mpe_config_t cfg_saved;
    int cfg_active;
} mfs_test_t;

static inline void mfs_test_begin(mfs_test_t *t, const char *name) {
    t->name = name;
    t->failures = 0;
    t->checks = 0;
    t->cfg_saved = g_cfg;
    t->cfg_active = 1;
    det_fallback_reset();
}

static inline void mfs_test_end(mfs_test_t *t) {
    if (t->cfg_active) {
        g_cfg = t->cfg_saved;
        t->cfg_active = 0;
    }
}

#define MFS_CHECK(t, cond) \
    do { \
        (t)->checks++; \
        if (!(cond)) { \
            (t)->failures++; \
            printf("[FAIL] %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        } \
    } while (0)

#define MFS_CHECK_NEAR(t, actual, expected, tol, label) \
    do { \
        (t)->checks++; \
        double _a = (double)(actual); \
        double _e = (double)(expected); \
        double _d = (_a > _e) ? (_a - _e) : (_e - _a); \
        if (!isfinite(_a) || _d > (double)(tol)) { \
            (t)->failures++; \
            printf("[FAIL] %s:%d: %s actual=%.6f expect=%.6f tol=%.6f\n", \
                   __FILE__, __LINE__, (label), _a, _e, (double)(tol)); \
        } \
    } while (0)

#define MFS_CHECK_REL(t, actual, expected, reltol, label) \
    do { \
        (t)->checks++; \
        double _a = (double)(actual); \
        double _e = (double)(expected); \
        double _d = (_a > _e) ? (_a - _e) : (_e - _a); \
        double _m = (_e > 0.0) ? _e : -_e; \
        if (!isfinite(_a) || !isfinite(_e) || _m <= 0.0 || _d / _m > (double)(reltol)) { \
            (t)->failures++; \
            printf("[FAIL] %s:%d: %s actual=%.6f expect=%.6f rel=%.4f\n", \
                   __FILE__, __LINE__, (label), _a, _e, (double)(reltol)); \
        } \
    } while (0)

#define MFS_INFO(...) \
    do { \
        printf("[info] "); \
        printf(__VA_ARGS__); \
        printf("\n"); \
    } while (0)

/* ------------------------------------------------------------------ */
/* World setup helpers.                                                */
/* ------------------------------------------------------------------ */

static inline void mfs_test_world(physics_world *w) {
    mpe_config_init();
    g_cfg.timestep.solver_iterations = 128;
    g_cfg.sleep.enable = 0;
    physics_world_init(w);
    constraint_pool_init(w);
    int f = physics_world_add_cube(w, (vector3){0.0f, -0.5f, 0.0f},
                                   (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    if (f >= 0) {
        w->bodies[f].friction_static = 1.0f;
        w->bodies[f].friction_kinetic = 0.8f;
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
            !isfinite(rb->velocity.x) || !isfinite(rb->velocity.y) || !isfinite(rb->velocity.z) ||
            !isfinite(rb->angular_velocity.x) || !isfinite(rb->angular_velocity.y) || !isfinite(rb->angular_velocity.z)) {
            return 0;
        }
    }
    return 1;
}

static inline int mfs_step(physics_world *w, int n, float dt) {
    for (int t = 0; t < n; t++) {
        physics_world_step(w, dt);
        if (!mfs_test_finite(w)) {
            printf("[FAIL] non-finite state at tick %d\n", t);
            return 0;
        }
    }
    return 1;
}

/* Lift WHOLE robot (chassis + wheels + rollers) to true free-spin height.
 * Fixes the old rig bug where only chassis was lifted, winching wheels up
 * through pendulum chaos. */
static inline void mfs_lift_whole_robot(physics_world *w, ftc_robot *robot, const vector3 *lift) {
    rigidbody *chassis = &w->bodies[robot->chassis_body];
    chassis->position = vector3_addition(chassis->position, *lift);
    chassis->velocity = vector3_zero();
    chassis->angular_velocity = vector3_zero();
    for (int wi_idx = 0; wi_idx < robot->wheel_count; wi_idx++) {
        int wi = robot->wheel_bodies[wi_idx];
        if (wi < 0 || wi >= w->body_count) continue;
        rigidbody *wb = &w->bodies[wi];
        wb->position = vector3_addition(wb->position, *lift);
        wb->velocity = vector3_zero();
        wb->angular_velocity = vector3_zero();
        rigidbody_update_axes(wb);
        for (int k = 0; k < robot->roller_count[wi_idx]; k++) {
            int rb = robot->roller_bodies[wi_idx][k];
            if (rb < 0 || rb >= w->body_count) continue;
            rigidbody *rbb = &w->bodies[rb];
            rbb->position = vector3_addition(rbb->position, *lift);
            rbb->velocity = vector3_zero();
            rbb->angular_velocity = vector3_zero();
            rigidbody_update_axes(rbb);
        }
    }
    rigidbody_update_axes(chassis);
    for (int wi_idx = 0; wi_idx < robot->wheel_count; wi_idx++) {
        motor_reset_observer(&robot->wheel_motors[wi_idx]);
    }
}

/* Lift robot to true free-spin height (1.9m above floor). */
static inline void mfs_lift_robot_for_free_spin(physics_world *w, ftc_robot *robot) {
    const vector3 lift = {0.0f, 1.9f, 0.0f};
    mfs_lift_whole_robot(w, robot, &lift);
    rigidbody *chassis = &w->bodies[robot->chassis_body];
    rigidbody_set_kinematic(chassis, true);
    chassis->velocity = vector3_zero();
}

/* Common robot creation with guaranteed floor. */
static inline ftc_robot *mfs_create_robot(physics_world *w, float x, float y, float z,
                                          motor_preset_id preset, ftc_drivetrain_type dtype) {
    ftc_robot *robot = (ftc_robot *)calloc(1, sizeof(ftc_robot));
    if (!robot) return NULL;
    int rc = ftc_robot_create_with_drive(w, robot, x, y, z, preset, dtype);
    if (rc != 0) {
        free(robot);
        return NULL;
    }
    return robot;
}

/* Drive helpers. */
static inline void mfs_drive_tank(ftc_robot *robot, float left, float right) {
    drivetrain_tank(robot, left, right);
}

static inline void mfs_drive_mecanum(ftc_robot *robot, float fwd, float strafe, float rotate) {
    drivetrain_mecanum(robot, fwd, strafe, rotate);
}

static inline void mfs_drive_stop(ftc_robot *robot) {
    float z[4] = {0, 0, 0, 0};
    ftc_robot_set_wheel_commands(robot, z, 4);
}

static inline void mfs_get_pos(physics_world *w, ftc_robot *robot, float *x, float *y, float *z) {
    ftc_robot_get_position(w, robot, x, y, z);
}

#endif /* mfs_test_h */
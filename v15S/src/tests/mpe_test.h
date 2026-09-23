/* MPE Test Framework v2 (mpe_test.h) — written from scratch.
 *
 * Why a new framework: the v1 suite (30 separate binaries + python runner)
 * hid three systematic setup bugs for the whole release cycle:
 *   1. stack / driven_wheel / list4 never enabled the frictional floor
 *      (static_plane_enabled=false default), so they measured the
 *      frictionless emergency boundary clamp instead of Coulomb contact.
 *   2. list4 rotated the cylinder about the wrong axis (Y keeps the axle
 *      horizontal; Z stands it up) and asserted the wrong rest height.
 *   3. Per-test ad-hoc setup meant every test re-derived (or mis-derived)
 *      floor/friction/material handling on its own.
 *
 * What v2 guarantees:
 *   - One binary, one registry, exact-name dispatch (no substring fan-out).
 *   - Every test runs under a saved/restored global config (no leakage).
 *   - Floor setup is a single audited helper: real Coulomb contact, either
 *     as an explicit mass-0 slab (mpe_floor_slab, preferred: true manifold,
 *     per-body friction) or as the infinite solver plane with synced
 *     friction (mpe_floor_plane).
 *   - Assertions print file:line + values on failure and keep going, so one
 *     run reports every broken oracle instead of aborting at the first.
 *   - NaN watchdog on every step; determinism counters asserted zero.
 */
#ifndef mpe_test_h
#define mpe_test_h

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "core/det_math.h"

/* ------------------------------------------------------------------ */
/* Test context: failure counting + config isolation.                  */
/* ------------------------------------------------------------------ */
typedef struct {
    const char *name;
    int failures;
    int checks;
    mpe_config_t cfg_saved;
    int cfg_active;
} mpe_test_t;

static inline void mpe_test_begin(mpe_test_t *t, const char *name) {
    t->name = name;
    t->failures = 0;
    t->checks = 0;
    t->cfg_saved = g_cfg;
    t->cfg_active = 1;
    det_fallback_reset();
}

static inline void mpe_test_end(mpe_test_t *t) {
    if (t->cfg_active) {
        g_cfg = t->cfg_saved;
        t->cfg_active = 0;
    }
}

#define MPE_CHECK(t, cond) \
    do { \
        (t)->checks++; \
        if (!(cond)) { \
            (t)->failures++; \
            printf("[FAIL] %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        } \
    } while (0)

#define MPE_CHECK_NEAR(t, actual, expected, tol, label) \
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

#define MPE_CHECK_REL(t, actual, expected, reltol, label) \
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

#define MPE_INFO(...) \
    do { \
        printf("[info] "); \
        printf(__VA_ARGS__); \
        printf("\n"); \
    } while (0)

/* ------------------------------------------------------------------ */
/* World setup helpers.                                                */
/* ------------------------------------------------------------------ */
static inline void mpe_world_begin(physics_world *w) {
    physics_world_init(w);
    constraint_pool_init(w);
}

static inline int mpe_world_finite(physics_world *w) {
    for (int i = 0; i < w->body_count; i++) {
        rigidbody *b = &w->bodies[i];
        if (!isfinite(b->position.x) || !isfinite(b->position.y) || !isfinite(b->position.z) ||
            !isfinite(b->velocity.x) || !isfinite(b->velocity.y) || !isfinite(b->velocity.z) ||
            !isfinite(b->angular_velocity.x) || !isfinite(b->angular_velocity.y) ||
            !isfinite(b->angular_velocity.z)) {
            return 0;
        }
    }
    return 1;
}

/* Step n ticks; returns 0 if any NaN/Inf appears (prints tick). */
static inline int mpe_step(physics_world *w, int n, float dt) {
    for (int t = 0; t < n; t++) {
        physics_world_step(w, dt);
        if (!mpe_world_finite(w)) {
            printf("[FAIL] non-finite state at tick %d\n", t);
            return 0;
        }
    }
    return 1;
}

/* Explicit mass-0 floor slab, top surface exactly y=0, with matched
 * Coulomb friction and restitution. Preferred floor: a true manifold
 * with per-body material combine (this is what friction_stop,
 * static_hold and determinism already used). */
static inline int mpe_floor_slab(physics_world *w, float mus, float muk, float e) {
    int f = physics_world_add_cube(w, (vector3){0.0f, -0.5f, 0.0f},
                                   (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    if (f < 0) {
        return -1;
    }
    w->bodies[f].friction_static = mus;
    w->bodies[f].friction_kinetic = muk;
    w->bodies[f].restitution = e;
    return f;
}

/* Infinite solver plane at y=0 with synced friction. The plane body reads
 * step_cfg->world.floor_friction_* every tick, so sync the globals AND the
 * cached body copy (covers worlds initialised before the sync). */
static inline void mpe_floor_plane(physics_world *w, float mus, float muk) {
    g_cfg.world.floor_friction_s = mus;
    g_cfg.world.floor_friction_k = muk;
    w->static_plane_enabled = true;
    w->static_plane_body.friction_static = mus;
    w->static_plane_body.friction_kinetic = muk;
}

static inline float mpe_vlen(vector3 v) {
    return vector3_length(v);
}

#endif /* mpe_test_h */

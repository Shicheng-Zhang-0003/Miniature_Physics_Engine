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
#include <stdlib.h>
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
    const char *regime;
} mpe_test_t;
/* DESPOT-2026-09-29 -- THE HARNESS CONTRACT.
 *
 * The single worst bug of this audit was not a physics bug: for the entire
 * life of the suite, `mpe_suite_main.c` never called `mpe_config_init()`, so
 * every test ran against a zero-initialised `g_cfg` -- 0 solver iterations, 0
 * Baumgarte bias, 0 penetration slop, 0 restitution, 0 friction. The solver was
 * not iterating and 34/34 were green. A gate could not have caught it, because
 * the suite was green in both regimes.
 *
 * So the harness now has two jobs it previously skipped:
 *
 *   1. ESTABLISH state, not merely save it. `mpe_test_begin` used to snapshot
 *      `g_cfg` and restore it afterwards, faithfully preserving whatever
 *      garbage it inherited. Save-and-restore without initialise is a harness
 *      that launders the bug.
 *   2. ASSERT its own preconditions. If the config is degenerate, every
 *      measurement below it is fiction, and the suite must say so loudly
 *      instead of reporting a confident green.
 */
/* Non-degenerate config = a config with a working solver in it. These are the
 * fields whose zero value silently disables physics rather than failing. */
static inline int mpe_cfg_is_degenerate (const mpe_config_t *c) {
    return (c == NULL) || (c->timestep.solver_iterations < 1) || !(c->solver.penetration_slop > 0.0f) ||
           !(c->solver.bias_factor > 0.0f) || !(c->timestep.max_substeps >= 1) ||
           !(c->body_defaults.sphere_restitution > 0.0f) || !(c->body_defaults.sphere_fric_s > 0.0f);
}
/* Apply a named regime. Regimes exist so the suite can be run across a spread
 * of configurations: a physics invariant that holds at one setting and not
 * another is a bug that a single golden number cannot see. The defaults are
 * chosen to stay inside the engine's documented parameter ranges so we are
 * exercising the shipped model, not an unreachable corner of the schema. */
typedef struct {
    const char *name;
    int iterations; /* <0 = leave alone */
    float gravity_mult; /* 1.0 = as configured */
    float friction_mult; /* 1.0 = as configured */
    float restitution; /* <0 = leave alone */
    int sleep; /* -1 = leave alone */
} mpe_regime_t;
static inline const mpe_regime_t *mpe_regime_lookup (const char *name) {
    static const mpe_regime_t regimes [] = {
        {"default", -1, 1.00f, 1.00f, -1.0f, -1}, {"light", 8, 0.25f, 0.50f, 0.10f, 0},
        {"heavy", 128, 3.00f, 2.00f, 0.80f, 1},   {"brittle", 64, 1.00f, 0.25f, 0.00f, 0},
        {"sticky", 64, 1.00f, 4.00f, 0.95f, 1},
    };
    const int n = (int) (sizeof (regimes) / sizeof (regimes [0]));
    if (!name || !*name)
        return &regimes [0];
    for (int i = 0; i < n; i++) {
        if (strcmp (regimes [i].name, name) == 0)
            return &regimes [i];
    }
    return NULL; /* unknown regime: caller must fail loudly */
}
static inline int mpe_regime_apply (const mpe_regime_t *r) {
    if (!r)
        return 0;
    if (r->iterations >= 0)
        g_cfg.timestep.solver_iterations = (float) r->iterations;
    if (g_cfg.world.gravity != 0.0f)
        g_cfg.world.gravity *= r->gravity_mult;
    if (g_cfg.body_defaults.sphere_fric_s > 0.0f) {
        g_cfg.body_defaults.sphere_fric_s *= r->friction_mult;
        g_cfg.body_defaults.cube_fric_s *= r->friction_mult;
        g_cfg.body_defaults.cylinder_fric_s *= r->friction_mult;
        g_cfg.world.floor_friction_s *= r->friction_mult;
    }
    if (r->restitution >= 0.0f) {
        g_cfg.body_defaults.sphere_restitution = r->restitution;
        g_cfg.body_defaults.cube_restitution = r->restitution;
        g_cfg.body_defaults.cylinder_restitution = r->restitution;
    }
    if (r->sleep >= 0)
        g_cfg.sleep.enable = r->sleep;
    return 1;
}
static inline void mpe_test_begin (mpe_test_t *t, const char *name) {
    t->name = name;
    t->failures = 0;
    t->checks = 0;
    /* Establish, then apply the requested regime, THEN save. Saving before
     * establishing is what let a zeroed g_cfg survive every test. */
    mpe_config_init ();
    const char *regime = getenv ("MPE_TEST_REGIME");
    if (regime && *regime && strcmp (regime, "default") != 0) {
        const mpe_regime_t *r = mpe_regime_lookup (regime);
        if (r)
            mpe_regime_apply (r);
        t->regime = r ? r->name : "INVALID";
    } else {
        t->regime = "default";
    }
    t->cfg_saved = g_cfg;
    t->cfg_active = 1;
    det_fallback_reset ();
}
/* DESPOT-2026-09-29: the header claimed "determinism counters asserted zero"
 * and mpe_test_end did not assert them -- only 1 of 32 tests did, by hand. A
 * libm fallback inside a test is a silent cross-platform determinism escape,
 * and nothing was watching. Assert here, where every test already passes. */
static inline int mpe_det_fallbacks_used (void) {
    return (int) (det_fallback_pow_total () + det_fallback_trig_total ());
}
static inline void mpe_test_end (mpe_test_t *t) {
    if (t->cfg_active) {
        if (mpe_det_fallbacks_used () != 0) {
            t->failures++;
            printf ("[FAIL] %s: %d libm determinism fallback(s) during test "
                    "(pow=%lu trig=%lu); results are no longer bit-deterministic\n",
                    t->name ? t->name : "?", mpe_det_fallbacks_used (), (unsigned long) det_fallback_pow_total (),
                    (unsigned long) det_fallback_trig_total ());
        }
        g_cfg = t->cfg_saved;
        t->cfg_active = 0;
    }
}
/* Return value for a case that could not run. Distinct from any possible
 * failure count (which is >= 0), so a test with failing checks can never be
 * mistaken for a skip.
 * DESPOT-2026-10-04: this used to be 2 — and mpe_run_one compared the raw
 * failure count against it, so ANY test with exactly 2 failed checks
 * reported "[SKIP] (coverage did not run)" instead of "[FAIL]". The C
 * summary then undercounted blocking failures (light-regime driven_wheel,
 * heavy incline_accel/list4 all vanished into SKIP). -1 is unreachable by
 * counting failures, closing the collision. */
#define MPE_SKIPPED (-1)
#define MPE_CHECK(t, cond)                                                                                             \
    do {                                                                                                               \
        (t)->checks++;                                                                                                 \
        if (!(cond)) {                                                                                                 \
            (t)->failures++;                                                                                           \
            printf ("[FAIL] %s:%d: %s\n", __FILE__, __LINE__, #cond);                                                  \
        }                                                                                                              \
    } while (0)
#define MPE_CHECK_NEAR(t, actual, expected, tol, label)                                                                \
    do {                                                                                                               \
        (t)->checks++;                                                                                                 \
        double _a = (double) (actual);                                                                                 \
        double _e = (double) (expected);                                                                               \
        double _d = (_a > _e) ? (_a - _e) : (_e - _a);                                                                 \
        if (!isfinite (_a) || _d > (double) (tol)) {                                                                   \
            (t)->failures++;                                                                                           \
            printf ("[FAIL] %s:%d: %s actual=%.6f expect=%.6f tol=%.6f\n", __FILE__, __LINE__, (label), _a, _e,        \
                    (double) (tol));                                                                                   \
        }                                                                                                              \
    } while (0)
#define MPE_CHECK_REL(t, actual, expected, reltol, label)                                                              \
    do {                                                                                                               \
        (t)->checks++;                                                                                                 \
        double _a = (double) (actual);                                                                                 \
        double _e = (double) (expected);                                                                               \
        double _d = (_a > _e) ? (_a - _e) : (_e - _a);                                                                 \
        double _m = (_e > 0.0) ? _e : -_e;                                                                             \
        if (!isfinite (_a) || !isfinite (_e) || _m <= 0.0 || _d / _m > (double) (reltol)) {                            \
            (t)->failures++;                                                                                           \
            printf ("[FAIL] %s:%d: %s actual=%.6f expect=%.6f rel=%.4f\n", __FILE__, __LINE__, (label), _a, _e,        \
                    (double) (reltol));                                                                                \
        }                                                                                                              \
    } while (0)
#define MPE_INFO(...)                                                                                                  \
    do {                                                                                                               \
        printf ("[info] ");                                                                                            \
        printf (__VA_ARGS__);                                                                                          \
        printf ("\n");                                                                                                 \
    } while (0)
/* ------------------------------------------------------------------ */
/* World setup helpers.                                                */
/* ------------------------------------------------------------------ */
static inline void mpe_world_begin (physics_world *w) {
    physics_world_init (w);
    constraint_pool_init (w);
}
static inline int mpe_world_finite (physics_world *w) {
    for (int i = 0; i < w->body_count; i++) {
        rigidbody *b = &w->bodies [i];
        /* DESPOT-2026-09-29: the ORIENTATION was not checked. A NaN quaternion
         * poisons every contact lever arm in the next tick while position and
         * velocity stay perfectly finite, so this whole function reported a
         * corrupted world as clean. The MFS harness (mfs_test.h) already had
         * the check; the MPE harness was the un-fixed copy. */
        if (!isfinite (b->position.x) || !isfinite (b->position.y) || !isfinite (b->position.z) ||
            !isfinite (b->velocity.x) || !isfinite (b->velocity.y) || !isfinite (b->velocity.z) ||
            !isfinite (b->angular_velocity.x) || !isfinite (b->angular_velocity.y) ||
            !isfinite (b->angular_velocity.z) || !isfinite (b->orientation.w) || !isfinite (b->orientation.x) ||
            !isfinite (b->orientation.y) || !isfinite (b->orientation.z)) {
            return 0;
        }
    }
    return 1;
}
/* Step n ticks; returns 0 if any NaN/Inf appears (prints tick). */
static inline int mpe_step (physics_world *w, int n, float dt) {
    for (int t = 0; t < n; t++) {
        physics_world_step (w, dt);
        if (!mpe_world_finite (w)) {
            printf ("[FAIL] non-finite state at tick %d\n", t);
            return 0;
        }
    }
    return 1;
}
/* Explicit mass-0 floor slab, top surface exactly y=0, with matched
 * Coulomb friction and restitution. Preferred floor: a true manifold
 * with per-body material combine (this is what friction_stop,
 * static_hold and determinism already used). */
static inline int mpe_floor_slab (physics_world *w, float mus, float muk, float e) {
    int f = physics_world_add_cube (w, (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    if (f < 0) {
        return -1;
    }
    w->bodies [f].friction_static = mus;
    w->bodies [f].friction_kinetic = muk;
    w->bodies [f].restitution = e;
    return f;
}
/* Infinite solver plane at y=0 with synced friction. The plane body reads
 * step_cfg->world.floor_friction_* every tick, so sync the globals AND the
 * cached body copy (covers worlds initialised before the sync). */
static inline void mpe_floor_plane (physics_world *w, float mus, float muk) {
    g_cfg.world.floor_friction_s = mus;
    g_cfg.world.floor_friction_k = muk;
    w->static_plane_enabled = true;
    w->static_plane_body.friction_static = mus;
    w->static_plane_body.friction_kinetic = muk;
}
/* DESPOT-2026-10-04 [CLAMP-TAUTOLOGY closure]: bind a per-world config copy
 * with the world-edge safety net OFF. With the net on, a rest-height gate is
 * satisfiable by the emergency clamp alone (a no-op solver still reports
 * y=support and passes). With it off, only genuine contact manifolds can
 * hold a body up. The copy is taken from the (regime-applied) g_cfg, so all
 * other tunables — including the active test regime — carry over; the
 * caller's mpe_config_t slot must outlive the stepping loop. */
static inline void mpe_world_no_net (physics_world *w, mpe_config_t *slot) {
    *slot = g_cfg;
    slot->boundary.safety_net_enabled = 0;
    physics_world_set_config (w, slot);
}
/* Contact evidence: world.has_contact[i] flags bodies that generated a
 * manifold on the CURRENT tick. A clamp-held body never flags. */
static inline int mpe_body_in_contact (const physics_world *w, int idx) {
    return (w && w->has_contact && idx >= 0 && idx < w->body_count) ? (w->has_contact [idx] != 0) : 0;
}
static inline float mpe_vlen (vector3 v) {
    return vector3_length (v);
}
#endif /* mpe_test_h */

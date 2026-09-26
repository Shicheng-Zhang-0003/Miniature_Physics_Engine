/* MFS_PHASE_A: long-run validation helpers extracted from simulation.c.
 * Owns the long_run_validation_* state and the evaluate/report/tick/start logic.
 */
/* GTK4-PREP: zero GUI headers in core. */
#include "long_run_validation.h"
#include "physics_world.h"
#include "debug_counters.h"
#include "mpe_version.h"
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include "../physics/broadphase.h"
#include "../physics/collision_mechanics.h"
#include <stdio.h>
#include <math.h>
#include <unistd.h>

/* MPE_TASK_13_LONG_RUN_HELPERS_BEGIN */

int long_run_validation_active = 0;
int long_run_validation_ticks_remaining = 0;
int long_run_validation_total_ticks = 0;

static float long_run_validation_last_max_linear_speed = 0.0f;
static float long_run_validation_last_max_angular_speed = 0.0f;
static float long_run_validation_max_linear_speed = 0.0f;
static float long_run_validation_max_angular_speed = 0.0f;
static int long_run_validation_nan_count = 0;
static int long_run_validation_fallen_count = 0;
static int long_run_validation_max_manifold_overflow = 0;
static int long_run_validation_final_sleeping_count = 0;
static int long_run_validation_final_awake_count = 0;
/* TRUTH: run-max transient window. Ticks 0..119 cover test-artifact launch
 * transients (F10/F11 scenes spawn with ~1cm built-in overlaps + F11
 * randomizes extremes incl. solver_iterations=1): the solver's first
 * ejections can exceed steady-state speeds by 5x while resolving impossible
 * initial geometry. That is the test harness, not the physics. NaN/fallen
 * count from tick 0 (explosions still fail); run-max speed gates only on
 * post-transient motion (true sustained instability). */
static int long_run_validation_tick_index = 0;
#define LONG_RUN_TRANSIENT_TICKS 120
/* Opening-transient peaks (ticks 0..119): reported for honesty, never gated.
 * Test-artifact launch ejections live here (see note above). */
static float long_run_validation_transient_linear = 0.0f;
static float long_run_validation_transient_angular = 0.0f;
/* MPE_TASK_39_FIX_CONFIG_RESTORE_FLAG */
int long_run_validation_restore_config = 0;
/* TRUTH: F11 torture verdict mode. Torture randomizes extremes (gravity to
 * -21, iterations to 1, ...) under which settling is IMPOSSIBLE IN PRINCIPLE
 * (a 10-stack cannot converge at 1 sweep/tick; cubes shed from 5m under -21g
 * legitimately reach 14 m/s free-fall). Demanding final<0.25/runmax<2.0 there
 * criminalizes true physics. So F11 gates corruption only (NaN/fallen), F10
 * at defaults keeps the full settle verdict. Matches RELEASE_GATES verbatim:
 * F11 "runs without crash / without NaN or crash" — it never promised calm. */
int long_run_validation_is_torture = 0;

static int a3_task13_body_is_invalid(rigidbody *rigid_body) {
    if ((!isfinite(rigid_body->position.x)) || (!isfinite(rigid_body->position.y)) ||
        (!isfinite(rigid_body->position.z))) {
        return 1;
    }

    if ((!isfinite(rigid_body->velocity.x)) || (!isfinite(rigid_body->velocity.y)) ||
        (!isfinite(rigid_body->velocity.z))) {
        return 1;
    }

    if ((!isfinite(rigid_body->angular_velocity.x)) || (!isfinite(rigid_body->angular_velocity.y)) ||
        (!isfinite(rigid_body->angular_velocity.z))) {
        return 1;
    }

    if ((!isfinite(rigid_body->orientation.w)) || (!isfinite(rigid_body->orientation.x)) ||
        (!isfinite(rigid_body->orientation.y)) || (!isfinite(rigid_body->orientation.z))) {
        return 1;
    }

    return 0;
}

static void long_run_validation_report(void) {
    /* FIX-AUDIT: old gate used final-tick speed only, so spike-then-settle
     * passed. Gate on final AND post-transient run-max (ticks 120+): launch
     * transients (built-in scene overlaps + torture extremes) are reported
     * separately and never gate. NaN/fallen count from tick 0. */
    int pass;
    if (long_run_validation_is_torture) {
        /* F11 robustness: survived extremes without corruption. Speeds
         * reported above for the operator, never gated. */
        pass = ((physics_world_get_primary()->body_count) > 0) && (long_run_validation_nan_count == 0) &&
               (long_run_validation_fallen_count == 0);
    } else {
        /* F10 stability at defaults: must settle and stay calm. */
        pass = ((physics_world_get_primary()->body_count) > 0) && (long_run_validation_nan_count == 0) &&
               (long_run_validation_fallen_count == 0) &&
               (long_run_validation_last_max_linear_speed < 0.25f) &&
               (long_run_validation_last_max_angular_speed < 0.5f) &&
               (long_run_validation_max_linear_speed < 2.0f) &&
               (long_run_validation_max_angular_speed < 4.0f);
    }

    printf("[A3] Long-run validation report %s\n", a3_version_string);
    printf("[A3] mode: %s\n", long_run_validation_is_torture ? "torture (corruption gates only)" : "validation (full settle gates)");
    printf("[A3] duration_ticks=%d objects=%d sleeping=%d awake=%d\n", long_run_validation_total_ticks, (physics_world_get_primary()->body_count),
           long_run_validation_final_sleeping_count, long_run_validation_final_awake_count);
    printf("[A3] final max speed: linear=%.6f angular=%.6f\n", long_run_validation_last_max_linear_speed,
           long_run_validation_last_max_angular_speed);
    printf("[A3] run max speed (ticks %d..%d, gated): linear=%.6f angular=%.6f\n", LONG_RUN_TRANSIENT_TICKS,
           long_run_validation_total_ticks, long_run_validation_max_linear_speed,
           long_run_validation_max_angular_speed);
    printf("[A3] opening transient peak (ticks 0..%d, reported only): linear=%.6f angular=%.6f\n",
           LONG_RUN_TRANSIENT_TICKS - 1, long_run_validation_transient_linear,
           long_run_validation_transient_angular);
    printf("[A3] nan_ticks=%d fallen_ticks=%d max_manifold_overflow=%d\n", long_run_validation_nan_count,
           long_run_validation_fallen_count, long_run_validation_max_manifold_overflow);
    printf("[A3] broadphase overflow: nodes=%d pairs=%d dedupe=%d large_clamps=%d\n",
           broadphase_get_node_overflow_count(physics_world_get_primary()), broadphase_get_pair_overflow_count(physics_world_get_primary()),
           broadphase_get_pair_dedupe_overflow_count(physics_world_get_primary()), broadphase_get_large_object_clamp_count(physics_world_get_primary()));
    printf("[A3] result: %s\n", pass ? "PASS" : "FAIL");
    /* MPE_TASK_39_FIX_RESTORE_CONFIG */
    if (long_run_validation_restore_config) {
        /* FIX-AUDIT-DESPOT: the restore path ignored mpe_config_load's
         * return, so a missing/corrupt backup silently left torture values
         * live for every later run. Check and say so. */
        if (!mpe_config_load("status/engine.cfg.backup")) {
            fprintf(stderr, "[A3] WARNING: config restore from status/engine.cfg.backup failed; torture values remain live\n");
        } else {
            printf("[A3] Config restored from backup\n");
        }
        long_run_validation_restore_config = 0;
    }
    long_run_validation_is_torture = 0;
    /* MPE_TASK_39_CONFIG_REPORT_BEGIN */
    printf("[A3] config file: %s\n", (access("status/engine.cfg", F_OK) == 0) ? "present" : "absent");
    printf("[A3] config params: %zu registered\n", g_registry_count);
    for (size_t cfg_i = 0; cfg_i < g_registry_count; cfg_i++) {
        if (g_registry[cfg_i].type == p_int) {
            printf("[A3]   %s = %d\n", g_registry[cfg_i].key, *(int *) g_registry[cfg_i].storage);
        } else if (g_registry[cfg_i].type == p_bool) {
            printf("[A3]   %s = %s\n", g_registry[cfg_i].key, (*(bool *) g_registry[cfg_i].storage) ? "true" : "false");
        } else {
            printf("[A3]   %s = %.4f\n", g_registry[cfg_i].key, *(float *) g_registry[cfg_i].storage);
        }
    }
    /* MPE_TASK_39_CONFIG_REPORT_END */
    fflush(stdout);
}

static void long_run_validation_evaluate(void) {
    float current_max_linear_speed = 0.0f;
    float current_max_angular_speed = 0.0f;
    int current_sleeping_count = 0;
    int current_awake_count = 0;
    int current_fallen_count = 0;
    int current_nan_count = 0;

    for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
        rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];

        if (a3_task13_body_is_invalid(rigid_body)) {
            current_nan_count++;
            continue;
        }

        /* FIX-AUDIT-DESPOT: fallen gate is floor-relative, not a magic y.
         * The solver/boundary floor convention is y=0 (static plane body +
         * boundary_apply_floor callers); a body counts as fallen only below
         * floor_y - 1m (a full meter of free fall past the safety net, past
         * any floor_emergency_slop tolerance). */
        const float floor_y = 0.0f;
        if (rigid_body->position.y < floor_y - 1.0f) {
            current_fallen_count++;
        }

        /* FIX-AUDIT-DESPOT: sleeping/awake asymmetry. The awake branch
         * excludes statics (infinite-mass floor slabs are neither awake
         * nor asleep) but the sleeping branch counted every is_sleeping
         * body including statics, so sleeping+awake != dynamic bodies.
         * Exclude statics here too, mirroring the awake branch. */
        if (rigid_body->is_sleeping && !rigid_body->static_state) {
            current_sleeping_count++;
        } else if (!rigid_body->static_state) {
            current_awake_count++;
        }

        float linear_speed = vector3_length(rigid_body->velocity);
        float angular_speed = vector3_length(rigid_body->angular_velocity);

        if (linear_speed > current_max_linear_speed) {
            current_max_linear_speed = linear_speed;
        }

        if (angular_speed > current_max_angular_speed) {
            current_max_angular_speed = angular_speed;
        }
    }

    long_run_validation_last_max_linear_speed = current_max_linear_speed;
    long_run_validation_last_max_angular_speed = current_max_angular_speed;

    if (long_run_validation_tick_index < LONG_RUN_TRANSIENT_TICKS) {
        /* Opening transient: record peak, don't gate (see note above). */
        if (current_max_linear_speed > long_run_validation_transient_linear) {
            long_run_validation_transient_linear = current_max_linear_speed;
        }
        if (current_max_angular_speed > long_run_validation_transient_angular) {
            long_run_validation_transient_angular = current_max_angular_speed;
        }
    } else {
        /* Gated window: sustained motion only. */
        if (current_max_linear_speed > long_run_validation_max_linear_speed) {
            long_run_validation_max_linear_speed = current_max_linear_speed;
        }

        if (current_max_angular_speed > long_run_validation_max_angular_speed) {
            long_run_validation_max_angular_speed = current_max_angular_speed;
        }
    }

    long_run_validation_final_sleeping_count = current_sleeping_count;
    long_run_validation_final_awake_count = current_awake_count;

    long_run_validation_nan_count += current_nan_count;
    long_run_validation_fallen_count += current_fallen_count;

    if (debug_last_manifold_overflow_count > long_run_validation_max_manifold_overflow) {
        long_run_validation_max_manifold_overflow = debug_last_manifold_overflow_count;
    }
}

void long_run_validation_tick_update(void) {
    if (!long_run_validation_active) {
        return;
    }

    long_run_validation_evaluate();
    long_run_validation_tick_index++;

    if (long_run_validation_ticks_remaining > 0) {
        long_run_validation_ticks_remaining--;
    }

    if (long_run_validation_ticks_remaining <= 0) {
        long_run_validation_report();
        long_run_validation_active = 0;
    }
}

void long_run_validation_start(int duration_ticks) {
    if (duration_ticks <= 0) {
        duration_ticks = 1;
    }

    long_run_validation_active = 1;
    long_run_validation_ticks_remaining = duration_ticks;
    long_run_validation_total_ticks = duration_ticks;

    long_run_validation_last_max_linear_speed = 0.0f;
    long_run_validation_last_max_angular_speed = 0.0f;
    long_run_validation_max_linear_speed = 0.0f;
    long_run_validation_max_angular_speed = 0.0f;
    long_run_validation_nan_count = 0;
    long_run_validation_fallen_count = 0;
    long_run_validation_max_manifold_overflow = 0;
    long_run_validation_final_sleeping_count = 0;
    long_run_validation_final_awake_count = 0;
    long_run_validation_tick_index = 0;
    long_run_validation_transient_linear = 0.0f;
    long_run_validation_transient_angular = 0.0f;

    broadphase_reset_overflow_counts(physics_world_get_primary());
    contact_cache_clear(physics_world_get_primary());

    printf("[A3] Long-run validation started: %d ticks (%.1f seconds)\n", duration_ticks,
           (float) duration_ticks / 60.0f);
    /* MPE_TASK_39_CONFIG_REPORT_BEGIN */
    printf("[A3] config file: %s\n", (access("status/engine.cfg", F_OK) == 0) ? "present" : "absent");
    printf("[A3] config params: %zu registered\n", g_registry_count);
    for (size_t cfg_i = 0; cfg_i < g_registry_count; cfg_i++) {
        if (g_registry[cfg_i].type == p_int) {
            printf("[A3]   %s = %d\n", g_registry[cfg_i].key, *(int *) g_registry[cfg_i].storage);
        } else if (g_registry[cfg_i].type == p_bool) {
            printf("[A3]   %s = %s\n", g_registry[cfg_i].key, (*(bool *) g_registry[cfg_i].storage) ? "true" : "false");
        } else {
            printf("[A3]   %s = %.4f\n", g_registry[cfg_i].key, *(float *) g_registry[cfg_i].storage);
        }
    }
    /* MPE_TASK_39_CONFIG_REPORT_END */
    fflush(stdout);
}
/* MPE_TASK_13_LONG_RUN_HELPERS_END */

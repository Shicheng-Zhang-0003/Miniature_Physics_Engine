/* MFS_PHASE_A: long-run validation helpers extracted from simulation.c.
 * Owns the long_run_validation_* state and the evaluate/report/tick/start logic.
 */
#include "long_run_validation.h"
#include "../mpe_engine.h"
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
/* MPE_TASK_39_FIX_CONFIG_RESTORE_FLAG */
int long_run_validation_restore_config = 0;

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
    int pass = (object_count > 0) && (long_run_validation_nan_count == 0) && (long_run_validation_fallen_count == 0) &&
               (long_run_validation_last_max_linear_speed < 0.25f) &&
               (long_run_validation_last_max_angular_speed < 0.5f);

    printf("[A3] Long-run validation report %s\n", a3_version_string);
    printf("[A3] duration_ticks=%d objects=%d sleeping=%d awake=%d\n", long_run_validation_total_ticks, object_count,
           long_run_validation_final_sleeping_count, long_run_validation_final_awake_count);
    printf("[A3] final max speed: linear=%.6f angular=%.6f\n", long_run_validation_last_max_linear_speed,
           long_run_validation_last_max_angular_speed);
    printf("[A3] run max speed: linear=%.6f angular=%.6f\n", long_run_validation_max_linear_speed,
           long_run_validation_max_angular_speed);
    printf("[A3] nan_ticks=%d fallen_ticks=%d max_manifold_overflow=%d\n", long_run_validation_nan_count,
           long_run_validation_fallen_count, long_run_validation_max_manifold_overflow);
    printf("[A3] broadphase overflow: nodes=%d pairs=%d dedupe=%d large_clamps=%d\n",
           broadphase_get_node_overflow_count(), broadphase_get_pair_overflow_count(),
           broadphase_get_pair_dedupe_overflow_count(), broadphase_get_large_object_clamp_count());
    printf("[A3] result: %s\n", pass ? "PASS" : "FAIL");
    /* MPE_TASK_39_FIX_RESTORE_CONFIG */
    if (long_run_validation_restore_config) {
        mpe_config_load("status/engine.cfg.backup");
        long_run_validation_restore_config = 0;
        printf("[A3] Config restored from backup\n");
    }
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

    for (int object_index = 0; object_index < object_count; object_index++) {
        rigidbody *rigid_body = &obj_per_scene[object_index];

        if (a3_task13_body_is_invalid(rigid_body)) {
            current_nan_count++;
            continue;
        }

        if (rigid_body->position.y < -1.0f) {
            current_fallen_count++;
        }

        if (rigid_body->is_sleeping) {
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

    if (current_max_linear_speed > long_run_validation_max_linear_speed) {
        long_run_validation_max_linear_speed = current_max_linear_speed;
    }

    if (current_max_angular_speed > long_run_validation_max_angular_speed) {
        long_run_validation_max_angular_speed = current_max_angular_speed;
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

    broadphase_reset_overflow_counts();
    contact_cache_clear(NULL);

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

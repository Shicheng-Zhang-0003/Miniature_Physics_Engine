/* MFS_PHASE_A: engine validation/status report, extracted from simulation.c.
* Read-only reporter: dumps engine counters, broadphase/contact-cache state,
* the config registry, and menu state to stdout (triggered by F9).
*/
#include "../mpe_engine.h"
#include <stdio.h>
#include <unistd.h>

void validation_report_print(void) {
    printf("[A3] Validation report %s\n", a3_version_string);
    printf("[A3] objects=%d capacity=%d joints=%d selected=%d\n", object_count, object_capacity, current_joint_count,
           selected_object);
    /* MPE_TASK_12_VALIDATION_PRINT_BEGIN */
    printf("[A3] sleeping objects: last_frame=%d\n", debug_last_sleeping_object_count);
    /* MPE_TASK_12_VALIDATION_PRINT_END */
    printf("[A3] debug last: obj=%d pairs=%d manifolds=%d frame_time=%f\n", debug_last_object_count,
           debug_last_broadphase_pair_count, debug_last_manifold_count, debug_last_frame_time);
    printf("[A3] broadphase overflow: nodes=%d pairs=%d\n", broadphase_get_node_overflow_count(),
           broadphase_get_pair_overflow_count());
    /* MPE_TASK_17_VALIDATION_PRINT_BEGIN */
    printf("[A3] broadphase cell size: %.2f\n", broadphase_get_current_cell_size());
    /* MPE_TASK_17_VALIDATION_PRINT_END */
    /* MPE_TASK_11_VALIDATION_PRINT_BEGIN */
    printf("[A3] broadphase large object clamps: last_run=%d\n", broadphase_get_large_object_clamp_count());
    /* MPE_TASK_11_VALIDATION_PRINT_END */
    /* MPE_TASK_10_VALIDATION_PRINT_BEGIN */
    printf("[A3] pair dedupe overflow: last_run=%d\n", broadphase_get_pair_dedupe_overflow_count());
    /* MPE_TASK_10_VALIDATION_PRINT_END */
    /* MPE_TASK_09_VALIDATION_PRINT_BEGIN */
    printf("[A3] manifold overflow: last_frame=%d\n", debug_last_manifold_overflow_count);
    /* MPE_TASK_09_VALIDATION_PRINT_END */
    printf("[A3] contact cache: hits=%d misses=%d\n", contact_cache_get_hits(), contact_cache_get_misses());
    printf("[A3] menus: open=%d spawner=%d velocity=%d object=%d marked_joint=%d\n", main_inputs.is_menu_open,
           main_inputs.spawner_menu_level, main_inputs.velocity_menu_level, main_inputs.object_menu_level,
           main_inputs.marked_joint_object_index);
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

/* MPE_FTC_054 */
#include "scene_id_remap.h"
#include "../config/mpe_constants.h"
typedef struct {
    uint32_t old_id;
    uint32_t new_id;
} id_remap_entry;
static id_remap_entry remap_table[mpe_max_bodies];
static int remap_count = 0;
void scene_id_remap_reset(void) {
    remap_count = 0;
}
void scene_id_remap_add(uint32_t old_id, uint32_t new_id) {
    if (remap_count >= mpe_max_bodies) {
        return;
    }
    remap_table[remap_count].old_id = old_id;
    remap_table[remap_count].new_id = new_id;
    remap_count++;
}
uint32_t scene_id_remap_resolve(uint32_t old_id) {
    for (int i = 0; i < remap_count; i++) {
        if (remap_table[i].old_id == old_id) {
            return remap_table[i].new_id;
        }
    }
    return old_id;
}

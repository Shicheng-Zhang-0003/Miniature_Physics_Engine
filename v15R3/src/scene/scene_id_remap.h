/* MPE_FTC_054 */
#ifndef scene_id_remap_h
#define scene_id_remap_h
#include <stdint.h>
void scene_id_remap_reset(void);
void scene_id_remap_add(uint32_t old_id, uint32_t new_id);
uint32_t scene_id_remap_resolve(uint32_t old_id);
#endif

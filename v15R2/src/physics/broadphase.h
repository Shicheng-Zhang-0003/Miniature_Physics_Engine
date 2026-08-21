#ifndef broadphase_h
#define broadphase_h
#include "../core/math3D.h"
#include "../core/rigidbody.h"

typedef struct {
    int object_index_a, object_index_b;
} broadphase_pair;

int broadphase_generate_pairing (broadphase_pair *collision_pairs_output_array, int maximum_pairs_allowed);

int broadphase_get_node_overflow_count (void);
int broadphase_get_pair_overflow_count (void);
/* MPE_TASK_11_LARGE_OBJECT_CLAMP_HEADER_BEGIN */
int broadphase_get_large_object_clamp_count (void);
/* MPE_TASK_11_LARGE_OBJECT_CLAMP_HEADER_END */
/* MPE_TASK_10_PAIR_DEDUPE_HEADER_BEGIN */
int broadphase_get_pair_dedupe_overflow_count (void);
/* MPE_TASK_10_PAIR_DEDUPE_HEADER_END */
void broadphase_reset_overflow_counts (void);
/* MPE_TASK_17_CELL_SIZE_HEADER_BEGIN */
float broadphase_get_current_cell_size (void);
/* MPE_TASK_17_CELL_SIZE_HEADER_END */
#endif

#ifndef broadphase_h
#define broadphase_h
#include "../core/math3d.h"
#include "../core/rigidbody.h"
#include "../config/mpe_constants.h"
#include <stdint.h>

typedef struct {
    int object_index_a, object_index_b;
} broadphase_pair;

struct physics_world;

typedef struct {
    int object_index;
    int next_entry;
} hash_node;

/* Per-world broadphase workspace (heap). Migrated from file-scope statics
 * so worlds never share mutable broadphase state. All fields are solver
 * scratch rebuilt per pairing call, except the overflow counters (read by
 * overlay/validation) and the node pool allocation itself. */
typedef struct {
    hash_node *node_pool;
    int node_pool_capacity;
    int node_count;
    int hash_table[hash_table_size];
    uint64_t pair_hash_keys[a3_pair_hash_table_size];
    uint32_t pair_hash_generations[a3_pair_hash_table_size];
    uint32_t pair_hash_generation;
    float current_cell_size;
    int node_overflow_count;
    int pair_overflow_count;
    int pair_dedupe_overflow_count;
    int large_object_clamp_count;
    /* Cell-size cache: recomputing avg radius every tick is O(n) per tick
     * (PERF-005). Cache and only recompute when the population changes
     * significantly or every 60 ticks. */
    int cached_body_count;
    int ticks_since_cell_recompute;
} broadphase_workspace;

int broadphase_generate_pairing(struct physics_world *world, broadphase_pair *collision_pairs_output_array,
                                int maximum_pairs_allowed, float dt); /* MPE_FTC_059 */
/* Conservative bounding-sphere radius (rotation-invariant). Shared with the
 * renderer's frustum culling so both use one definition. */
float broadphase_bounding_radius(rigidbody *rb);

int broadphase_get_node_overflow_count(const struct physics_world *world);
int broadphase_get_pair_overflow_count(const struct physics_world *world);
/* MPE_TASK_11_LARGE_OBJECT_CLAMP_HEADER_BEGIN */
int broadphase_get_large_object_clamp_count(const struct physics_world *world);
/* MPE_TASK_11_LARGE_OBJECT_CLAMP_HEADER_END */
/* MPE_TASK_10_PAIR_DEDUPE_HEADER_BEGIN */
int broadphase_get_pair_dedupe_overflow_count(const struct physics_world *world);
/* MPE_TASK_10_PAIR_DEDUPE_HEADER_END */
void broadphase_reset_overflow_counts(struct physics_world *world);
void broadphase_cleanup(struct physics_world *world);
/* MPE_TASK_17_CELL_SIZE_HEADER_BEGIN */
float broadphase_get_current_cell_size(const struct physics_world *world);
/* MPE_TASK_17_CELL_SIZE_HEADER_END */
#endif

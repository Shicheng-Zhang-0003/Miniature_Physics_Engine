#include "../mpe_engine.h"
#include "broadphase.h"
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#define GRID_CELL_SIZE 5.0f
#define HASH_TABLE_SIZE 8192
#define MAX_OBJECTS 16384
typedef struct { int object_index; int next_entry; } hash_node;
static hash_node *node_pool = NULL; /* A3_PATCH_15_DYNAMIC_NODE_POOL */
static int node_pool_capacity = 0;
static int node_count = 0;
static int hash_table [HASH_TABLE_SIZE];

/* A3_PATCH_14_OVERFLOW_VISIBILITY */
static int broadphase_node_overflow_count = 0;
static int broadphase_pair_overflow_count = 0;

int broadphase_get_node_overflow_count (void) {
    return broadphase_node_overflow_count;
}

int broadphase_get_pair_overflow_count (void) {
    return broadphase_pair_overflow_count;
}

void broadphase_reset_overflow_counts (void) {
    broadphase_node_overflow_count = 0;
    broadphase_pair_overflow_count = 0;
}
static int hash_coordinate (int x, int y, int z) {
    /* A3_PATCH_13_HASH_SAFETY */
    unsigned int h = ((unsigned int) x) * 73856093u;
    h ^= ((unsigned int) y) * 19349663u;
    h ^= ((unsigned int) z) * 83492791u;
    return (int) (h % HASH_TABLE_SIZE);
}

static bool broadphase_ensure_node_capacity (void) {
    /* A3_PATCH_15_DYNAMIC_NODE_POOL */
    if (node_pool == NULL) {
        node_pool_capacity = MAX_OBJECTS * 8;
        node_pool = (hash_node *) malloc ((size_t) node_pool_capacity * sizeof (hash_node));

        if (node_pool == NULL) {
            node_pool_capacity = 0;
            broadphase_node_overflow_count++;
            return false;
        }

        return true;
    }

    if (node_count < node_pool_capacity) {return true;}

    int new_capacity = (node_pool_capacity > 0) ? (node_pool_capacity * 2) : (MAX_OBJECTS * 8);

    if (new_capacity < node_pool_capacity) {
        broadphase_node_overflow_count++;
        return false;
    }

    hash_node *new_pool = (hash_node *) realloc (node_pool, (size_t) new_capacity * sizeof (hash_node));

    if (new_pool == NULL) {
        broadphase_node_overflow_count++;
        return false;
    }

    node_pool = new_pool;
    node_pool_capacity = new_capacity;
    return true;
}

static void insert_into_hash (int object_index, int x, int y, int z) {
    if (!broadphase_ensure_node_capacity ()) {return;}
    int hash = hash_coordinate (x, y, z);
    node_pool [node_count].object_index = object_index;
    node_pool [node_count].next_entry = hash_table [hash];
    hash_table [hash] = node_count;
    node_count++;

}

static inline float broadphase_bounding_radius (rigidbody *rb) {
    if (rb -> type == object_sphere) {
        return rb -> radius;
    }

    return sqrtf (
        rb -> half_extensions.x * rb -> half_extensions.x +
        rb -> half_extensions.y * rb -> half_extensions.y +
        rb -> half_extensions.z * rb -> half_extensions.z
    );
}

#define A3_PAIR_HASH_TABLE_SIZE (1 << 18)
#define A3_PAIR_HASH_MASK (A3_PAIR_HASH_TABLE_SIZE - 1)

static uint64_t a3_pair_hash_keys [A3_PAIR_HASH_TABLE_SIZE];
static uint32_t a3_pair_hash_generations [A3_PAIR_HASH_TABLE_SIZE];
static uint32_t a3_pair_hash_generation = 0;

static inline uint64_t a3_broadphase_pair_key (int object_a, int object_b) {
    return ((uint64_t) (uint32_t) object_a << 32) | (uint64_t) (uint32_t) object_b;
}

static inline uint32_t a3_broadphase_pair_hash (uint64_t key) {
    uint64_t mixed_key = key;
    mixed_key ^= mixed_key >> 33;
    mixed_key *= 0xff51afd7ed558ccdULL;
    mixed_key ^= mixed_key >> 33;
    mixed_key *= 0xc4ceb9fe1a85ec53ULL;
    mixed_key ^= mixed_key >> 33;
    return (uint32_t) (mixed_key & A3_PAIR_HASH_MASK);
}

static void broadphase_pair_dedupe_begin (void) {
    /* A3_PATCH_12_PAIR_HASH */
    a3_pair_hash_generation++;

    if (a3_pair_hash_generation == 0) {
        for (int i = 0; i < A3_PAIR_HASH_TABLE_SIZE; i++) {
            a3_pair_hash_generations [i] = 0;
        }

        a3_pair_hash_generation = 1;
    }
}

static bool pair_already_checked (int min_obj, int max_obj) {
    uint64_t key = a3_broadphase_pair_key (min_obj, max_obj);
    uint32_t index = a3_broadphase_pair_hash (key);

    for (uint32_t probe = 0; probe < A3_PAIR_HASH_TABLE_SIZE; probe++) {
        uint32_t slot = (index + probe) & A3_PAIR_HASH_MASK;

        if (a3_pair_hash_generations [slot] != a3_pair_hash_generation) {
            a3_pair_hash_keys [slot] = key;
            a3_pair_hash_generations [slot] = a3_pair_hash_generation;
            return false;
        }

        if (a3_pair_hash_keys [slot] == key) {return true;}
    }

    return false;
}

int broadphase_generate_pairing (broadphase_pair *collision_pairs_output_array, int maximum_pairs_allowed) {
    if (object_count < 2) {return 0;}
    for (int i = 0; i < HASH_TABLE_SIZE; i++) {hash_table [i] = -1;}
    node_count = 0;
        /* A3_PATCH_12_PAIR_HASH */
    broadphase_pair_dedupe_begin ();
    int collision_pair_counter = 0;
    for (int i = 0; i < object_count; i++) {
        rigidbody *rb = &obj_per_scene [i];
        /* A3_PATCH_11_SLEEPING_BROADPHASE */
        float extent_x, extent_y, extent_z;
        if (rb -> type == object_sphere) {extent_x = extent_y = extent_z = rb -> radius;}
        else {
            vector3 *axes = rb -> cached_axes;
            extent_x = fabsf (axes [0].x) * rb -> half_extensions.x + fabsf (axes [1].x) * rb -> half_extensions.y + fabsf (axes [2].x) * rb -> half_extensions.z;
            extent_y = fabsf (axes [0].y) * rb -> half_extensions.x + fabsf (axes [1].y) * rb -> half_extensions.y + fabsf (axes [2].y) * rb -> half_extensions.z;
            extent_z = fabsf (axes [0].z) * rb -> half_extensions.x + fabsf (axes [1].z) * rb -> half_extensions.y + fabsf (axes [2].z) * rb -> half_extensions.z;
        } int min_x = (int) floorf ((rb -> position.x - extent_x) / GRID_CELL_SIZE);
        int max_x = (int) floorf ((rb -> position.x + extent_x) / GRID_CELL_SIZE);
        int min_y = (int) floorf ((rb -> position.y - extent_y) / GRID_CELL_SIZE);
        int max_y = (int) floorf ((rb -> position.y + extent_y) / GRID_CELL_SIZE);
        int min_z = (int) floorf ((rb -> position.z - extent_z) / GRID_CELL_SIZE);
        int max_z = (int) floorf ((rb -> position.z + extent_z) / GRID_CELL_SIZE);
        for (int x = min_x; x <= max_x; x++) {
            for (int y = min_y; y <= max_y; y++) {
                for (int z = min_z; z <= max_z; z++) { insert_into_hash (i, x, y, z); }
            }
        }
    } for (int i = 0; i < HASH_TABLE_SIZE; i++) {
        int node_idx = hash_table [i];
        while (node_idx != -1) {
            int obj_a = node_pool [node_idx].object_index;
            int next_node_idx = node_pool [node_idx].next_entry;
            while (next_node_idx != -1) {
                int obj_b = node_pool [next_node_idx].object_index;
                if (obj_a != obj_b) {
                    int min_obj = obj_a < obj_b ? obj_a : obj_b;
                    int max_obj = obj_a > obj_b ? obj_a : obj_b;
                    if (!pair_already_checked (min_obj, max_obj)) {
                        rigidbody *rb_a = &obj_per_scene [min_obj];
                        rigidbody *rb_b = &obj_per_scene [max_obj];
                        float dist_sq = vector3_length_squared (vector3_subtraction (rb_a -> position, rb_b -> position));
                        float rad_sum = broadphase_bounding_radius (rb_a) + broadphase_bounding_radius (rb_b);
                        if (dist_sq <= rad_sum * rad_sum) {
                            if (collision_pair_counter < maximum_pairs_allowed) {
                                collision_pairs_output_array [collision_pair_counter].object_index_a = min_obj;
                                collision_pairs_output_array [collision_pair_counter].object_index_b = max_obj;
                                collision_pair_counter++;
                            } else {broadphase_pair_overflow_count++;}
                        }
                    }
                } next_node_idx = node_pool [next_node_idx].next_entry;
            } node_idx = node_pool [node_idx].next_entry;
        }
    } return collision_pair_counter;
}

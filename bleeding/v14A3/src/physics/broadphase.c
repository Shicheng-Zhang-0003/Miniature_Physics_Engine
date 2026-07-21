#include "broadphase.h"
#include <stdlib.h>
#include <math.h>
#define GRID_CELL_SIZE 5.0f
#define HASH_TABLE_SIZE 8192
#define MAX_OBJECTS 16384
typedef struct { int object_index; int next_entry; } hash_node;
static hash_node node_pool [MAX_OBJECTS * 8];
static int node_count = 0;
static int hash_table [HASH_TABLE_SIZE];
static int hash_coordinate (int x, int y, int z) {
    int h = (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);
    return abs (h) % HASH_TABLE_SIZE;
} static void insert_into_hash (int object_index, int x, int y, int z) {
    if (node_count >= MAX_OBJECTS * 8) {return;}
    int hash = hash_coordinate (x, y, z);
    node_pool [node_count].object_index = object_index;
    node_pool [node_count].next_entry = hash_table [hash];
    hash_table [hash] = node_count;
    node_count++;
} int broadphase_generate_pairing (broadphase_pair *collision_pairs_output_array, int maximum_pairs_allowed) {
    if (object_count < 2) {return 0;}
    for (int i = 0; i < HASH_TABLE_SIZE; i++) {hash_table [i] = -1;}
    node_count = 0;
    static int *last_checked_gen = NULL;
    static int checked_capacity = 0;
    static int current_gen = 0;
    if (object_count * object_count > checked_capacity) {
        checked_capacity = object_count * object_count + 1024;
        last_checked_gen = realloc (last_checked_gen, checked_capacity * sizeof (int));
        for (int i = 0; i < checked_capacity; i++) {last_checked_gen [i] = 0;}
    } current_gen += 1;
    int collision_pair_counter = 0;
    for (int i = 0; i < object_count; i++) {
        rigidbody *rb = &obj_per_scene [i];
        if (rb -> is_sleeping) {continue;}
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
                    int pair_index = min_obj * object_count + max_obj;
                    if (last_checked_gen [pair_index] != current_gen) {
                        last_checked_gen [pair_index] = current_gen;
                        rigidbody *rb_a = &obj_per_scene [min_obj];
                        rigidbody *rb_b = &obj_per_scene [max_obj];
                        static inline float get_bounding_radius (rigidbody *rb) {
                            if (rb -> type == object_sphere) {return rb -> radius;}
                            return sqrtf (rb -> half_extensions.x * rb -> half_extensions.x + rb -> half_extensions.y * rb -> half_extensions.y + rb -> half_extensions.z * rb -> half_extensions.z);
                        }
                        float dist_sq = vector3_length_squared (vector3_subtraction (rb_a -> position, rb_b -> position));
                        float rad_sum = get_bounding_radius (rb_a) + get_bounding_radius (rb_b);
                        if (dist_sq <= rad_sum * rad_sum) {
                            if (collision_pair_counter < maximum_pairs_allowed) {
                                collision_pairs_output_array [collision_pair_counter].object_index_a = min_obj;
                                collision_pairs_output_array [collision_pair_counter].object_index_b = max_obj;
                                collision_pair_counter++;
                            }
                        }
                    }
                } next_node_idx = node_pool [next_node_idx].next_entry;
            } node_idx = node_pool [node_idx].next_entry;
        }
    } return collision_pair_counter;
}

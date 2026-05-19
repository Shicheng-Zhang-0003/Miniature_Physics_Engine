#include "broadphase.h"
#include <stdlib.h>
#include <math.h>
typedef struct {
    float x_minimum;
    int object_index;
} broadphase_sweep_entry;
typedef struct {
    float x_max, y_min, y_max, z_min, z_max;
} broadphase_aabb_cache;
int broadphase_generate_pairing (broadphase_pair *collision_pairs_output_array, int maximum_pairs_allowed) {
    if (object_count < 2) {return 0;}
    static broadphase_sweep_entry *persistent_sweep_array = NULL;
    static broadphase_aabb_cache *aabb_cache = NULL;
    static int persistent_sweep_capacity = 0;
    static int last_object_count = 0;
    //Grow buffers if needed
    if (object_count > persistent_sweep_capacity) {
        persistent_sweep_capacity = object_count + 64;
        persistent_sweep_array = realloc (persistent_sweep_array, persistent_sweep_capacity * sizeof (broadphase_sweep_entry));
        aabb_cache = realloc (aabb_cache, persistent_sweep_capacity * sizeof (broadphase_aabb_cache));
        last_object_count = 0; //Force re-fill
    } //Initialise sweep array with object indices if count changed
    if (object_count != last_object_count) {
        for (int i = 0; i < object_count; i++) {persistent_sweep_array [i].object_index = i;}
        last_object_count = object_count;
    } //Update AABBs and x_minimum values
    for (int object_iterator = 0; object_iterator < object_count; object_iterator++) {
        int object_index = persistent_sweep_array [object_iterator].object_index;
        rigidbody *rigid_body = &obj_per_scene [object_index];
        float extent_x, extent_y, extent_z;
        if (rigid_body -> type == object_sphere) {extent_x = extent_y = extent_z = rigid_body -> radius;}
        else {
            vector3 *axes = rigid_body -> cached_axes;
            extent_x = fabsf (axes [0].x) * rigid_body -> half_extensions.x + fabsf (axes [1].x) * rigid_body -> half_extensions.y + fabsf (axes [2].x) * rigid_body -> half_extensions.z;
            extent_y = fabsf (axes [0].y) * rigid_body -> half_extensions.x + fabsf (axes [1].y) * rigid_body -> half_extensions.y + fabsf (axes [2].y) * rigid_body -> half_extensions.z;
            extent_z = fabsf (axes [0].z) * rigid_body -> half_extensions.x + fabsf (axes [1].z) * rigid_body -> half_extensions.y + fabsf (axes [2].z) * rigid_body -> half_extensions.z;
        } persistent_sweep_array [object_iterator].x_minimum = rigid_body -> position.x - extent_x;
        aabb_cache [object_index].x_max = rigid_body -> position.x + extent_x;
        aabb_cache [object_index].y_min = rigid_body -> position.y - extent_y;
        aabb_cache [object_index].y_max = rigid_body -> position.y + extent_y;
        aabb_cache [object_index].z_min = rigid_body -> position.z - extent_z;
        aabb_cache [object_index].z_max = rigid_body -> position.z + extent_z;
    }
 //Sorting (O (N) loop on average since invocation is determined by number of instances)
    for (int i = 1; i < object_count; i++) {
        broadphase_sweep_entry key = persistent_sweep_array [i];
        int j = i - 1;
        while (j >= 0 && persistent_sweep_array [j].x_minimum > key.x_minimum) {
            persistent_sweep_array [j + 1] = persistent_sweep_array [j];
            j -= 1;
        } persistent_sweep_array [j + 1] = key;
    } //Sweep and Prune with AABB rejection
    int collision_pair_counter = 0;
    for (int outer_iterator = 0; outer_iterator < object_count; outer_iterator++) {
        int object_index_a = persistent_sweep_array [outer_iterator].object_index;
        float x_max_a = aabb_cache [object_index_a].x_max;
        for (int inner_iterator = outer_iterator + 1; inner_iterator < object_count; inner_iterator++) {
            //If next object starts after current object ends on X
            if (persistent_sweep_array [inner_iterator].x_minimum > x_max_a) {break;}
            if (collision_pair_counter >= maximum_pairs_allowed) {break;}
            int object_index_b = persistent_sweep_array [inner_iterator].object_index;
            //Check Y and Z separation of objects
            if ((aabb_cache [object_index_b].y_min > aabb_cache [object_index_a].y_max) || (aabb_cache [object_index_b].y_max < aabb_cache [object_index_a].y_min)) {continue;}
            if ((aabb_cache [object_index_b].z_min > aabb_cache [object_index_a].z_max) || (aabb_cache [object_index_b].z_max < aabb_cache [object_index_a].z_min)) {continue;}
            collision_pairs_output_array [collision_pair_counter].object_index_a = object_index_a;
            collision_pairs_output_array [collision_pair_counter].object_index_b = object_index_b;
            collision_pair_counter++;
        }
    }
 return collision_pair_counter;
}

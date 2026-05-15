#include "broadphase.h"
#include <stdlib.h>
typedef struct {
    float x_minimum;
    int object_index;
} broadphase_sweep_entry;
int broadphase_generate_pairing (broadphase_pair *collision_pairs_output_array, int maximum_pairs_allowed) {
    if (object_count < 2) {return 0;}
    static broadphase_sweep_entry *persistent_sweep_array = NULL;
    static int persistent_sweep_capacity = 0;
    
    if (object_count > persistent_sweep_capacity) {
        persistent_sweep_capacity = object_count + 64; // Small buffer
        persistent_sweep_array = realloc (persistent_sweep_array, persistent_sweep_capacity * sizeof (broadphase_sweep_entry));
    }
    
    if (!persistent_sweep_array) {return 0;}
    for (int object_index = 0; object_index < object_count; object_index++) {
        persistent_sweep_array [object_index].x_minimum = obj_per_scene [object_index].position.x - obj_per_scene [object_index].radius;
        persistent_sweep_array [object_index].object_index = object_index;
    } for (int sweep_index_y = 1; sweep_index_y < object_count; sweep_index_y++) {
        broadphase_sweep_entry sweep_entry_key = persistent_sweep_array [sweep_index_y];
        int sweep_index_x = sweep_index_y - 1;
        while ((sweep_index_x >= 0) && (persistent_sweep_array [sweep_index_x].x_minimum > sweep_entry_key.x_minimum)) {
            persistent_sweep_array [sweep_index_x + 1] = persistent_sweep_array [sweep_index_x];
            sweep_index_x = sweep_index_x - 1;
        } persistent_sweep_array [sweep_index_x + 1] = sweep_entry_key;
    } int collision_pair_counter = 0;
    for (int sweep_index_a = 0; sweep_index_a < object_count; sweep_index_a++) {
        int object_index_a = persistent_sweep_array [sweep_index_a].object_index;
        rigidbody *rigid_body_a = &obj_per_scene [object_index_a];
        float x_maximum_a = rigid_body_a -> position.x + rigid_body_a -> radius * 1.1f;
        for (int sweep_index_b = sweep_index_a + 1; sweep_index_b < object_count; sweep_index_b++) {
            if (persistent_sweep_array [sweep_index_b].x_minimum > x_maximum_a) {break;}
            if (collision_pair_counter >= maximum_pairs_allowed) {break;}
            int object_index_b = persistent_sweep_array [sweep_index_b].object_index;
            rigidbody *rigid_body_b = &obj_per_scene [object_index_b];
            
            float combined_radius = (rigid_body_a -> radius + rigid_body_b -> radius) * 1.1f;
            
            // AABB Check on Y and Z (X already handled by sweep)
            if (fabsf (rigid_body_b -> position.y - rigid_body_a -> position.y) <= combined_radius) {
                if (fabsf (rigid_body_b -> position.z - rigid_body_a -> position.z) <= combined_radius) {
                    collision_pairs_output_array [collision_pair_counter].object_index_a = object_index_a;
                    collision_pairs_output_array [collision_pair_counter].object_index_b = object_index_b;
                    collision_pair_counter += 1;
                }
            }
        }
    } return collision_pair_counter;
}

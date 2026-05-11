#include "broadphase.h"
#include <stdlib.h>
typedef struct {
    float x_minimum;
    int object_index;
} broadphase_sweep_entry;
int broadphase_generate_pairing (broadphase_pair *collision_pairs_output_array, int maximum_pairs_allowed) {
    if (object_count < 2) {return 0;}
    broadphase_sweep_entry *sweep_entry_array = malloc (object_count * sizeof (broadphase_sweep_entry));
    if (!sweep_entry_array) {return 0;}
    for (int object_index = 0; object_index < object_count; object_index++) {
        sweep_entry_array [object_index].x_minimum = obj_per_scene [object_index].position.x - obj_per_scene [object_index].radius;
        sweep_entry_array [object_index].object_index = object_index;
    } for (int sweep_index_y = 1; sweep_index_y < object_count; sweep_index_y++) {
        broadphase_sweep_entry sweep_entry_key = sweep_entry_array [sweep_index_y];
        int sweep_index_x = sweep_index_y - 1;
        while ((sweep_index_x >= 0) && (sweep_entry_array [sweep_index_x].x_minimum > sweep_entry_key.x_minimum)) {
            sweep_entry_array [sweep_index_x + 1] = sweep_entry_array [sweep_index_x];
            sweep_index_x = sweep_index_x - 1;
        } sweep_entry_array [sweep_index_x + 1] = sweep_entry_key;
    } int collision_pair_counter = 0;
    for (int sweep_index_a = 0; sweep_index_a < object_count; sweep_index_a++) {
        int object_index_a = sweep_entry_array [sweep_index_a].object_index;
        rigidbody *rigid_body_a = &obj_per_scene [object_index_a];
        float x_maximum_a = rigid_body_a -> position.x + rigid_body_a -> radius * 1.1f;
        for (int sweep_index_b = sweep_index_a + 1; sweep_index_b < object_count; sweep_index_b++) {
            if (sweep_entry_array [sweep_index_b].x_minimum > x_maximum_a) {break;}
            if (collision_pair_counter >= maximum_pairs_allowed) {break;}
            int object_index_b = sweep_entry_array [sweep_index_b].object_index;
            rigidbody *rigid_body_b = &obj_per_scene [object_index_b];
            float combined_collision_radius_with_margin = (rigid_body_a -> radius + rigid_body_b -> radius) * 1.1f;
            vector3 relative_position_vector = vector3_subtraction (rigid_body_b -> position, rigid_body_a -> position);
            float distance_between_centers_squared = vector3_length_squared (relative_position_vector);
            if (distance_between_centers_squared <= (combined_collision_radius_with_margin * combined_collision_radius_with_margin)) {
                collision_pairs_output_array [collision_pair_counter].object_index_a = object_index_a;
                collision_pairs_output_array [collision_pair_counter].object_index_b = object_index_b;
                collision_pair_counter += 1;
            }
        }
    } free (sweep_entry_array);
    return collision_pair_counter;
}

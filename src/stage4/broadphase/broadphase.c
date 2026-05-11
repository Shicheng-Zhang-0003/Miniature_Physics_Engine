#include "broadphase.h"
int broadphase_generate_pairing (broadphase_pair *collision_pairs_output_array, int maximum_pairs_allowed) {
    int collision_pair_counter = 0;
    for (int object_index_a = 0; object_index_a < object_count; object_index_a++) {
        for (int object_index_b = object_index_a + 1; object_index_b < object_count; object_index_b++) {
            if (collision_pair_counter >= maximum_pairs_allowed) {break;}
            rigidbody *rigid_body_a = &obj_per_scene [object_index_a];
            rigidbody *rigid_body_b = &obj_per_scene [object_index_b];
            //Distance Check (Margin + Tolerance)
            float combined_collision_radius_with_margin = (rigid_body_a -> radius + rigid_body_b -> radius) * 1.1f;
            vector3 relative_position_vector = vector3_subtraction (rigid_body_b -> position, rigid_body_a -> position);
            float distance_between_centers_squared = vector3_length_squared (relative_position_vector);
            if (distance_between_centers_squared <= (combined_collision_radius_with_margin * combined_collision_radius_with_margin)) {
                collision_pairs_output_array [collision_pair_counter].object_index_a = object_index_a;
                collision_pairs_output_array [collision_pair_counter].object_index_b = object_index_b;
                collision_pair_counter += 1;
            }
        }
    } return collision_pair_counter;
}

#include "broadphase.h"
int broadphase_generate_pairing (broadphase_pair *output_collision_pairs_array, int maximum_collision_pairs_allowed) {
    int collision_pair_counter = 0;
    for (int object_index_a = 0; object_index_a < object_count; object_index_a++) {
        for (int object_index_b = object_index_a + 1; object_index_b < object_count; object_index_b++) {
            if (collision_pair_counter >= maximum_collision_pairs_allowed) {break;}
            rigidbody *rigidbody_object_a_pointer = &obj_per_scene [object_index_a];
            rigidbody *rigidbody_object_b_pointer = &obj_per_scene [object_index_b];
            //Distance Check (Margin + Tolerance)
            float collision_proximity_margin_with_tolerance = (rigidbody_object_a_pointer -> radius + rigidbody_object_b_pointer -> radius) * 1.1;
            vector3 position_delta_vector = vector3_subtraction (rigidbody_object_b_pointer -> position, rigidbody_object_a_pointer -> position);
            float distance_between_centres_squared = vector3_length_squared (position_delta_vector);
            if (distance_between_centres_squared <= (collision_proximity_margin_with_tolerance * collision_proximity_margin_with_tolerance)) {
                output_collision_pairs_array [collision_pair_counter].a = object_index_a;
                output_collision_pairs_array [collision_pair_counter].b = object_index_b;
                collision_pair_counter += 1;
            }
        }
    } return collision_pair_counter;
}

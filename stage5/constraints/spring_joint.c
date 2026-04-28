#include "spring_joint.h"
#include <stdio.h>
spring_joint_mechanism joint_pool [max_joint_count];
int joint_count = 0;
int add_joint (int rigidbody_object_index_a, int rigidbody_object_index_b, float spring_resting_length_value, float spring_stiffness_k_constant, float spring_damping_coefficient) {
    //Add a new object in a empty row of buffer
    for (int joint_pool_iterator_index = 0; joint_pool_iterator_index < max_joint_count; joint_pool_iterator_index++) {
        if (!joint_pool [joint_pool_iterator_index].activation) {
            joint_pool [joint_pool_iterator_index].index_av = rigidbody_object_index_a;
            joint_pool [joint_pool_iterator_index].index_bv = rigidbody_object_index_b;
            joint_pool [joint_pool_iterator_index].spring_length_rest = spring_resting_length_value;
            joint_pool [joint_pool_iterator_index].k_constant = spring_stiffness_k_constant;
            joint_pool [joint_pool_iterator_index].dampening = spring_damping_coefficient;
            joint_pool [joint_pool_iterator_index].activation = true;
            joint_count += 1;
            return joint_pool_iterator_index;
        }
    } fprintf (stderr, "Error SJA001: No Remaining Space in Buffer\n");
    return -1;
} void remove_joint (int target_joint_pool_index) {
    if ((target_joint_pool_index < 0) || (target_joint_pool_index >= max_joint_count)) {return;}
    if (!joint_pool [target_joint_pool_index].activation) {return;}
    joint_pool [target_joint_pool_index].activation = false;
    joint_count -= 1;
} void apply_force_all_joints (void) {
    for (int joint_pool_iterator_index = 0; joint_pool_iterator_index < max_joint_count; joint_pool_iterator_index++) {
        if (!joint_pool [joint_pool_iterator_index].activation) {continue;}
        spring_joint_mechanism *current_spring_joint_pointer = &joint_pool [joint_pool_iterator_index];
        //Validate Indices
        if ((current_spring_joint_pointer -> index_av >= object_count) || (current_spring_joint_pointer -> index_bv >= object_count)) {
            remove_joint (joint_pool_iterator_index);
            continue;
        } rigidbody *rigidbody_object_a_pointer = &obj_per_scene [current_spring_joint_pointer -> index_av];
        rigidbody *rigidbody_object_b_pointer = &obj_per_scene [current_spring_joint_pointer -> index_bv];
        //Vector: A to B
        vector3 position_difference_vector = vector3_subtraction (rigidbody_object_b_pointer -> position, rigidbody_object_a_pointer -> position);
        float current_distance_between_objects = vector3_length (position_difference_vector);
        if (current_distance_between_objects < math_epsilon) {continue;}
        float spring_extension_displacement = current_distance_between_objects - current_spring_joint_pointer -> spring_length_rest;
        vector3 spring_force_direction_normalised_vector = vector3_normalisation (position_difference_vector);
        //Spring Force (Fs = -kx)
        vector3 calculated_spring_force_vector = vector3_scaling (spring_force_direction_normalised_vector, current_spring_joint_pointer -> k_constant * spring_extension_displacement);
        vector3 relative_velocity_vector = vector3_subtraction (rigidbody_object_b_pointer -> velocity, rigidbody_object_a_pointer -> velocity);
        float relative_velocity_along_spring_axis = vector3_dot (relative_velocity_vector, spring_force_direction_normalised_vector);
        //Spring Damping (F = -cv)
        vector3 calculated_spring_damping_force_vector = vector3_scaling (spring_force_direction_normalised_vector, current_spring_joint_pointer -> dampening * relative_velocity_along_spring_axis);
        //Net Resultant Force
        vector3 net_resultant_spring_force_vector = vector3_addition (calculated_spring_force_vector, calculated_spring_damping_force_vector);
        //Apply Equal and Opposite forces
        rb_apply_forces_perfect (rigidbody_object_a_pointer, net_resultant_spring_force_vector);
        rb_apply_forces_perfect (rigidbody_object_b_pointer, vector3_scaling (net_resultant_spring_force_vector, -1.0));
    }
} void remove_joints_from_object (int object_index) {
    for (int step = 0; step < max_joint_count; step++) {
        if (!joint_pool [step].activation) {continue;}
        if ((joint_pool [step].index_av == object_index) || (joint_pool [step].index_bv == object_index)) {remove_joint (step);}
    }
} void joint_init_pool (void) {
    for (int step = 0; step < max_joint_count; step++) {joint_pool [step].activation = false;}
    joint_count = 0;
}

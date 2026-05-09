#include "spring_joint.h"
#include <stdio.h>
spring_joint joint_pool [maximum_joint_count];
int current_joint_count = 0;
int add_joint (int object_index_a, int object_index_b, float equilibrium_length, float spring_constant, float damping_coefficient) {
    //Add a new object in a empty row of buffer
    for (int joint_index = 0; joint_index < maximum_joint_count; joint_index++) {
        if (!joint_pool [joint_index].is_active) {
            joint_pool [joint_index].object_index_a = object_index_a;
            joint_pool [joint_index].object_index_b = object_index_b;
            joint_pool [joint_index].equilibrium_length = equilibrium_length;
            joint_pool [joint_index].spring_constant = spring_constant;
            joint_pool [joint_index].damping_coefficient = damping_coefficient;
            joint_pool [joint_index].is_active = true;
            current_joint_count += 1;
            return joint_index;
        }
    } fprintf (stderr, "Error SJA001: No Remaining Space in Buffer\n");
    return -1;
} void remove_joint (int joint_pool_index) {
    if ((joint_pool_index < 0) || (joint_pool_index >= maximum_joint_count)) {return;}
    if (!joint_pool [joint_pool_index].is_active) {return;}
    joint_pool [joint_pool_index].is_active = false;
    current_joint_count -= 1;
} void apply_force_all_joints (void) {
    for (int joint_index = 0; joint_index < maximum_joint_count; joint_index++) {
        if (!joint_pool [joint_index].is_active) {continue;}
        spring_joint *current_spring_joint = &joint_pool [joint_index];
        //Validate Indices
        if ((current_spring_joint -> object_index_a >= object_count) || (current_spring_joint -> object_index_b >= object_count)) {
            remove_joint (joint_index);
            continue;
        } rigidbody *rigid_body_a = &obj_per_scene [current_spring_joint -> object_index_a];
        rigidbody *rigid_body_b = &obj_per_scene [current_spring_joint -> object_index_b];
        //Vector: A to B
        vector3 displacement_vector = vector3_subtraction (rigid_body_b -> position, rigid_body_a -> position);
        float current_separation_distance = vector3_length (displacement_vector);
        if (current_separation_distance < math_epsilon) {continue;}
        float spring_extension = current_separation_distance - current_spring_joint -> equilibrium_length;
        vector3 spring_axis_direction = vector3_normalisation (displacement_vector);
        //Spring Force (Fs = -kx)
        vector3 restoration_force = vector3_scaling (spring_axis_direction, current_spring_joint -> spring_constant * spring_extension);
        vector3 relative_velocity = vector3_subtraction (rigid_body_b -> velocity, rigid_body_a -> velocity);
        float velocity_along_spring_axis = vector3_dot (relative_velocity, spring_axis_direction);
        //Spring Damping (F = -cv)
        vector3 damping_force = vector3_scaling (spring_axis_direction, current_spring_joint -> damping_coefficient * velocity_along_spring_axis);
        //Net Resultant Force
        vector3 net_joint_force = vector3_addition (restoration_force, damping_force);
        //Apply Equal and Opposite forces
        rb_apply_forces_perfect (rigid_body_a, net_joint_force);
        rb_apply_forces_perfect (rigid_body_b, vector3_scaling (net_joint_force, -1.0));
    }
} void remove_joints_from_object (int object_index) {
    for (int joint_index = 0; joint_index < maximum_joint_count; joint_index++) {
        if (!joint_pool [joint_index].is_active) {continue;}
        if ((joint_pool [joint_index].object_index_a == object_index) || (joint_pool [joint_index].object_index_b == object_index)) {remove_joint (joint_index);}
    }
} void joint_init_pool (void) {
    for (int joint_index = 0; joint_index < maximum_joint_count; joint_index++) {joint_pool [joint_index].is_active = false;}
    current_joint_count = 0;
}

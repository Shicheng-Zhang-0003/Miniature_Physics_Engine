#include "spring_joint.h"
#include <stdio.h>
spring_joint_mechanism joint_pool [max_joint_count];
int joint_count = 0;
int add_joint (int index_av, int index_bv, float spring_length_rest, float k_constant, float dampening) {
    //Add a new object in a empty row of buffer
    for (int step = 0; step < max_joint_count; step++) {
        if (!joint_pool [step].activation) {
            joint_pool [step].index_av = index_av;
            joint_pool [step].index_bv = index_bv;
            joint_pool [step].spring_length_rest = spring_length_rest;
            joint_pool [step].k_constant = k_constant;
            joint_pool [step].dampening = dampening;
            joint_pool [step].activation = true;
            joint_count += 1;
            return step;
        }
    } fprintf (stderr, "Error SJA001: No Remaining Space in Buffer\n");
    return -1;
} void remove_joint (int joint_index) {
    if ((joint_index < 0) || (joint_index >= max_joint_count)) {return;}
    if (!joint_pool [joint_index].activation) {return;}
    joint_pool [joint_index].activation = false;
    joint_count -= 1;
} void apply_force_all_joints (void) {
    for (int step = 0; step < max_joint_count; step++) {
        if (!joint_pool [step].activation) {continue;}
        spring_joint_mechanism *joint = &joint_pool [step];
        //Validate no out of range pull of attributes
        if ((joint->index_av >= object_count) || (joint->index_bv >= object_count)) {
            remove_joint (step);
            continue;
        } rigidbody *a = &obj_per_scene [joint->index_av];
        rigidbody *b = &obj_per_scene [joint->index_bv];
        //Vector from object a to object b
        vector3 delta_constant = vector3_subtraction (b->position, a->position);
        float current_length = vector3_length (delta_constant);
        if (current_length < math_epsilon) {continue;}
        float displacement = current_length - joint->spring_length_rest;
        vector3 direction_travel = vector3_normalisation (delta_constant);
        //Spring force: Fs = -kx (Fs = -k_constant * displacement)
        vector3 spring_force = vector3_scaling (direction_travel, joint->k_constant * displacement);
        vector3 relative_velocity = vector3_subtraction (b->velocity, a->velocity);
        float velocity_along_spring = vector3_dot (relative_velocity, direction_travel);
        //Dampen the relative velocity of the spring in momentum
        vector3 dampening_force = vector3_scaling (direction_travel, joint->dampening * velocity_along_spring);
        //Apply a net force resultant to all objects
        vector3 net_force_applied = vector3_addition (spring_force, dampening_force);
        //Equal and opposite force to each object accordingly
        rb_apply_forces_perfect (a, net_force_applied);
        rb_apply_forces_perfect (b, vector3_scaling (net_force_applied, -1.0));
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

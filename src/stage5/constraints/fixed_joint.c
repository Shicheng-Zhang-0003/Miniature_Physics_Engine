#include "fixed_joint.h"
#include <stdio.h>
fixed_joint fixed_joint_pool [maximum_fixed_joint_count];
int current_fixed_joint_count = 0;
int add_fixed_joint (int object_index_a, int object_index_b, vector3 anchor) {
    for (int joint_index = 0; joint_index < maximum_fixed_joint_count; joint_index++) {
        if (!fixed_joint_pool [joint_index].is_active) {
            fixed_joint_pool [joint_index].object_index_a = object_index_a;
            fixed_joint_pool [joint_index].object_index_b = object_index_b;
            fixed_joint_pool [joint_index].anchor_point = anchor;
            fixed_joint_pool [joint_index].is_active = true;
            current_fixed_joint_count += 1;
            return joint_index;
        }
    } fprintf (stderr, "Error FJA001: No Remaining Space in Fixed Joint Buffer\n");
    return -1;
}
void remove_fixed_joint (int joint_pool_index) {
    if ((joint_pool_index < 0) || (joint_pool_index >= maximum_fixed_joint_count)) {return;}
    if (!fixed_joint_pool [joint_pool_index].is_active) {return;}
    fixed_joint_pool [joint_pool_index].is_active = false;
    current_fixed_joint_count -= 1;
}
void solve_all_fixed_joints (float delta_time) {
    for (int joint_index = 0; joint_index < maximum_fixed_joint_count; joint_index++) {
        if (!fixed_joint_pool [joint_index].is_active) {continue;}
        fixed_joint *current_joint = &fixed_joint_pool [joint_index];
        if ((current_joint -> object_index_a >= object_count) || (current_joint -> object_index_b >= object_count)) {
            remove_fixed_joint (joint_index);
            continue;
        }
        rigidbody *rigid_body_a = &obj_per_scene [current_joint -> object_index_a];
        rigidbody *rigid_body_b = &obj_per_scene [current_joint -> object_index_b];
        if ((rigid_body_a -> static_state) && (rigid_body_b -> static_state)) {continue;}
        vector3 ra = vector3_subtraction (current_joint -> anchor_point, rigid_body_a -> position);
        vector3 rb = vector3_subtraction (current_joint -> anchor_point, rigid_body_b -> position);
        vector3 va = vector3_addition (rigid_body_a -> velocity, vector3_cross (rigid_body_a -> angular_velocity, ra));
        vector3 vb = vector3_addition (rigid_body_b -> velocity, vector3_cross (rigid_body_b -> angular_velocity, rb));
        vector3 v_rel = vector3_subtraction (va, vb);
        float inv_mass_sum = rigid_body_a -> inverse_mass + rigid_body_b -> inverse_mass;
        if (inv_mass_sum > 0.0f) {
            vector3 j_imp = vector3_scaling (v_rel, -1.0f / inv_mass_sum);
            rigid_body_a -> velocity = vector3_addition (rigid_body_a -> velocity, vector3_scaling (j_imp, rigid_body_a -> inverse_mass));
            rigid_body_b -> velocity = vector3_addition (rigid_body_b -> velocity, vector3_scaling (j_imp, -rigid_body_b -> inverse_mass));
        }
        vector3 rel_w = vector3_subtraction (rigid_body_a -> angular_velocity, rigid_body_b -> angular_velocity);
        float inv_inertia_sum = rigid_body_a -> inverse_mass + rigid_body_b -> inverse_mass;
        if (inv_inertia_sum > 0.0f) {
            vector3 ang_imp = vector3_scaling (rel_w, -1.0f / inv_inertia_sum);
            rigid_body_a -> angular_velocity = vector3_addition (rigid_body_a -> angular_velocity, vector3_scaling (ang_imp, rigid_body_a -> inverse_mass));
            rigid_body_b -> angular_velocity = vector3_addition (rigid_body_b -> angular_velocity, vector3_scaling (ang_imp, -rigid_body_b -> inverse_mass));
        }
    }
}
void remove_fixed_joints_from_object (int object_index) {
    for (int joint_index = 0; joint_index < maximum_fixed_joint_count; joint_index++) {
        if (!fixed_joint_pool [joint_index].is_active) {continue;}
        if ((fixed_joint_pool [joint_index].object_index_a == object_index) || 
            (fixed_joint_pool [joint_index].object_index_b == object_index)) {
            remove_fixed_joint (joint_index);
        }
    }
}
void fixed_joint_init_pool (void) {
    for (int joint_index = 0; joint_index < maximum_fixed_joint_count; joint_index++) {
        fixed_joint_pool [joint_index].is_active = false;
    }
    current_fixed_joint_count = 0;
}

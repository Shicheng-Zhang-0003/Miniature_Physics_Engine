#include "revolute_joint.h"
#include <stdio.h>
#include <math.h>
revolute_joint revolute_joint_pool [maximum_revolute_joint_count];
int current_revolute_joint_count = 0;
static float compute_angle_around_axis (vector3 r1, vector3 r2, vector3 axis) {
    vector3 p = vector3_normalisation (r1);
    vector3 q = vector3_normalisation (r2);
    vector3 cross_prod = vector3_cross (p, q);
    float dot_val = vector3_dot (p, q);
    float proj = vector3_dot (cross_prod, axis);
    return atan2f (proj, dot_val);
}
int add_revolute_joint (int object_index_a, int object_index_b, vector3 anchor, vector3 axis) {
    for (int joint_index = 0; joint_index < maximum_revolute_joint_count; joint_index++) {
        if (!revolute_joint_pool [joint_index].is_active) {
            revolute_joint_pool [joint_index].object_index_a = object_index_a;
            revolute_joint_pool [joint_index].object_index_b = object_index_b;
            revolute_joint_pool [joint_index].anchor_point = anchor;
            revolute_joint_pool [joint_index].axis = vector3_normalisation (axis);
            revolute_joint_pool [joint_index].lower_limit = -1e30f;
            revolute_joint_pool [joint_index].upper_limit = 1e30f;
            revolute_joint_pool [joint_index].motor_speed = 0.0f;
            revolute_joint_pool [joint_index].max_motor_torque = 0.0f;
            revolute_joint_pool [joint_index].use_motor = false;
            revolute_joint_pool [joint_index].use_limits = false;
            revolute_joint_pool [joint_index].current_angle = 0.0f;
            revolute_joint_pool [joint_index].is_active = true;
            current_revolute_joint_count += 1;
            return joint_index;
        }
    } fprintf (stderr, "Error RJA001: No Remaining Space in Revolute Joint Buffer\n");
    return -1;
}
void remove_revolute_joint (int joint_pool_index) {
    if ((joint_pool_index < 0) || (joint_pool_index >= maximum_revolute_joint_count)) {return;}
    if (!revolute_joint_pool [joint_pool_index].is_active) {return;}
    revolute_joint_pool [joint_pool_index].is_active = false;
    current_revolute_joint_count -= 1;
}
void solve_all_revolute_joints (float delta_time) {
    for (int joint_index = 0; joint_index < maximum_revolute_joint_count; joint_index++) {
        if (!revolute_joint_pool [joint_index].is_active) {continue;}
        revolute_joint *current_joint = &revolute_joint_pool [joint_index];
        if ((current_joint -> object_index_a >= object_count) || (current_joint -> object_index_b >= object_count)) {
            remove_revolute_joint (joint_index);
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
        float w_a = vector3_dot (rigid_body_a -> angular_velocity, current_joint -> axis);
        float w_b = vector3_dot (rigid_body_b -> angular_velocity, current_joint -> axis);
        float rel_w = w_a - w_b;
        if (current_joint -> use_motor) {
            float speed_err = current_joint -> motor_speed - rel_w;
            float torque = speed_err * 10.0f;
            if (torque > current_joint -> max_motor_torque) {torque = current_joint -> max_motor_torque;}
            if (torque < -current_joint -> max_motor_torque) {torque = -current_joint -> max_motor_torque;}
            vector3 motor_imp = vector3_scaling (current_joint -> axis, torque * delta_time);
            rigid_body_a -> angular_velocity = vector3_addition (rigid_body_a -> angular_velocity, vector3_scaling (motor_imp, rigid_body_a -> inverse_mass));
            rigid_body_b -> angular_velocity = vector3_subtraction (rigid_body_b -> angular_velocity, vector3_scaling (motor_imp, rigid_body_b -> inverse_mass));
        }
        if (current_joint -> use_limits) {
            bool hit_limit = false;
            if ((rel_w > 0.0f) && (current_joint -> current_angle >= current_joint -> upper_limit)) {hit_limit = true;}
            else if ((rel_w < 0.0f) && (current_joint -> current_angle <= current_joint -> lower_limit)) {hit_limit = true;}
            if (hit_limit) {
                float inv_inertia_sum = rigid_body_a -> inverse_mass + rigid_body_b -> inverse_mass;
                if (inv_inertia_sum > 0.0f) {
                    float lambda = -rel_w / inv_inertia_sum;
                    vector3 limit_imp = vector3_scaling (current_joint -> axis, lambda);
                    rigid_body_a -> angular_velocity = vector3_addition (rigid_body_a -> angular_velocity, vector3_scaling (limit_imp, rigid_body_a -> inverse_mass));
                    rigid_body_b -> angular_velocity = vector3_subtraction (rigid_body_b -> angular_velocity, vector3_scaling (limit_imp, rigid_body_b -> inverse_mass));
                }
            }
        }
        vector3 new_ra = vector3_subtraction (current_joint -> anchor_point, rigid_body_a -> position);
        vector3 new_rb = vector3_subtraction (current_joint -> anchor_point, rigid_body_b -> position);
        current_joint -> current_angle = compute_angle_around_axis (ra, new_ra, current_joint -> axis);
    }
}
void remove_revolute_joints_from_object (int object_index) {
    for (int joint_index = 0; joint_index < maximum_revolute_joint_count; joint_index++) {
        if (!revolute_joint_pool [joint_index].is_active) {continue;}
        if ((revolute_joint_pool [joint_index].object_index_a == object_index) || 
            (revolute_joint_pool [joint_index].object_index_b == object_index)) {
            remove_revolute_joint (joint_index);
        }
    }
}
void revolute_joint_init_pool (void) {
    for (int joint_index = 0; joint_index < maximum_revolute_joint_count; joint_index++) {
        revolute_joint_pool [joint_index].is_active = false;
    }
    current_revolute_joint_count = 0;
}

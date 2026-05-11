#include "boundary.h"
void boundary_apply_floor (rigidbody *rigid_body, float floor_y_coordinate) {
    if (rigid_body -> static_state) {return;}
    float contact_y_position = floor_y_coordinate + rigid_body -> radius;
    if (rigid_body -> position.y < contact_y_position) {
        // Actual Collision Position
        rigid_body -> position.y = contact_y_position;
        // Bouncing vertical velocity upwards with restitution velocity loss
        if (rigid_body -> velocity.y < 0) {
            rigid_body -> velocity.y = -rigid_body -> velocity.y * rigid_body -> restitution;
            // Dampen horizontal velocity --> torque generated friction
            rigid_body -> velocity.x *= 0.98f;
            rigid_body -> velocity.z *= 0.98f;
        } // Dampen angular velocity on collision --> torques
        rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
    }
} void boundary_apply_box (rigidbody *rigid_body, vector3 minimum_bounds, vector3 maximum_bounds) {
    if (rigid_body -> static_state) {return;}
    // X axis
    // Minimum Position Bound/Left Wall
    if (rigid_body -> position.x - rigid_body -> radius < minimum_bounds.x) {
        //Actual Position Calibration (APC)
        rigid_body -> position.x = minimum_bounds.x + rigid_body -> radius;
        //Restitution Velocity (RSV)
        if (rigid_body -> velocity.x < 0) {
            rigid_body -> velocity.x = -rigid_body -> velocity.x * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    } //Maximum Position Bound/Right Wall
    if (rigid_body -> position.x + rigid_body -> radius > maximum_bounds.x) {
        //APC
        rigid_body -> position.x = maximum_bounds.x - rigid_body -> radius;
        //RSV
        if (rigid_body -> velocity.x > 0) {
            rigid_body -> velocity.x = -rigid_body -> velocity.x * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    }
    // Y axis
    // Minimum Position Bound/Bottom Wall
    if (rigid_body -> position.y - rigid_body -> radius < minimum_bounds.y) {
        //APC
        rigid_body -> position.y = minimum_bounds.y + rigid_body -> radius;
        //RSV
        if (rigid_body -> velocity.y < 0) {
            rigid_body -> velocity.y = -rigid_body -> velocity.y * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    } //Maximum Position Bound/Top Wall
    if (rigid_body -> position.y + rigid_body -> radius > maximum_bounds.y) {
        //APC
        rigid_body -> position.y = maximum_bounds.y - rigid_body -> radius;
        //RSV
        if (rigid_body -> velocity.y > 0) {
            rigid_body -> velocity.y = -rigid_body -> velocity.y * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    }
    // Z axis
    // Minimum Position Bound/Wall Behind
    if (rigid_body -> position.z - rigid_body -> radius < minimum_bounds.z) {
        //APC
        rigid_body -> position.z = minimum_bounds.z + rigid_body -> radius;
        //RSV
        if (rigid_body -> velocity.z < 0) {
            rigid_body -> velocity.z = -rigid_body -> velocity.z * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    } //Maximum Position Bound/Wall In Front
    if (rigid_body -> position.z + rigid_body -> radius > maximum_bounds.z) {
        //APC
        rigid_body -> position.z = maximum_bounds.z - rigid_body -> radius;
        //RSV
        if (rigid_body -> velocity.z > 0) {
            rigid_body -> velocity.z = -rigid_body -> velocity.z * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    }
}

#include "boundary.h"
void boundary_apply_floor (rigidbody *rb, float floor_y) {
    if (rb -> static_state) {return;}
    float contact_y = floor_y + rb -> radius;
    if (rb -> position.y < contact_y) {
        // Actual Collision Position
        rb -> position.y = contact_y;
        // Bouncing vertical velocity upwards with restitution velocity loss
        if (rb -> velocity.y < 0) {
            rb -> velocity.y = -rb -> velocity.y * rb -> restitution;
            // Dampen horizontal velocity --> torque generated friction
            rb -> velocity.x *= 0.98;
            rb -> velocity.z *= 0.98;
        } // Dampen angular velocity on collision --> torques
        rb -> angular_velocity = vector3_scaling (rb -> angular_velocity, 0.98f);
    }
} void boundary_apply_box (rigidbody *rb, vector3 minimum_bounds, vector3 maximum_bounds) {
    if (rb -> static_state) {return;}
    // X axis
    // Minimum Position Bound/Left Wall
    if (rb -> position.x - rb -> radius < minimum_bounds.x) {
        //Actual Position Calibration (APC)
        rb -> position.x = minimum_bounds.x + rb -> radius;
        //Restitution Velocity (RSV)
        if (rb -> velocity.x < 0) {
            rb -> velocity.x = -rb -> velocity.x * rb -> restitution;
            rb -> angular_velocity = vector3_scaling (rb -> angular_velocity, 0.98f);
        }
    } //Maximum Position Bound/Right Wall
    if (rb -> position.x + rb -> radius > maximum_bounds.x) {
        //APC
        rb -> position.x = maximum_bounds.x - rb -> radius;
        //RSV
        if (rb -> velocity.x > 0) {
            rb -> velocity.x = -rb -> velocity.x * rb -> restitution;
            rb -> angular_velocity = vector3_scaling (rb -> angular_velocity, 0.98f);
        }
    }
    // Y axis
    // Minimum Position Bound/Bottom Wall
    if (rb -> position.y - rb -> radius < minimum_bounds.y) {
        //APC
        rb -> position.y = minimum_bounds.y + rb -> radius;
        //RSV
        if (rb -> velocity.y < 0) {
            rb -> velocity.y = -rb -> velocity.y * rb -> restitution;
            rb -> angular_velocity = vector3_scaling (rb -> angular_velocity, 0.98f);
        }
    } //Maximum Position Bound/Top Wall
    if (rb -> position.y + rb -> radius > maximum_bounds.y) {
        //APC
        rb -> position.y = maximum_bounds.y - rb -> radius;
        //RSV
        if (rb -> velocity.y > 0) {
            rb -> velocity.y = -rb -> velocity.y * rb -> restitution;
            rb -> angular_velocity = vector3_scaling (rb -> angular_velocity, 0.98f);
        }
    }
    // Z axis
    // Minimum Position Bound/Wall Behind
    if (rb -> position.z - rb -> radius < minimum_bounds.z) {
        //APC
        rb -> position.z = minimum_bounds.z + rb -> radius;
        //RSV
        if (rb -> velocity.z < 0) {
            rb -> velocity.z = -rb -> velocity.z * rb -> restitution;
            rb -> angular_velocity = vector3_scaling (rb -> angular_velocity, 0.98f);
        }
    } //Maximum Position Bound/Wall In Front
    if (rb -> position.z + rb -> radius > maximum_bounds.z) {
        //APC
        rb -> position.z = maximum_bounds.z - rb -> radius;
        //RSV
        if (rb -> velocity.z > 0) {
            rb -> velocity.z = -rb -> velocity.z * rb -> restitution;
            rb -> angular_velocity = vector3_scaling (rb -> angular_velocity, 0.98f);
        }
    }
}

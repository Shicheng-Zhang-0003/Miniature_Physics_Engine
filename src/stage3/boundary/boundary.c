#include "boundary.h"
#include <math.h>
// Helper: Get lowest point of OBB along an axis
static float get_obb_min_along_axis (rigidbody *rb, vector3 axis) {
    if (rb -> type == object_sphere) return vector3_dot (rb -> position, axis) - rb -> radius;
    vector3 axes [3];
    math3 rot = vector4_to_math3 (rb -> orientation);
    axes [0] = (vector3) {rot.matrix [0][0], rot.matrix [1][0], rot.matrix [2][0]};
    axes [1] = (vector3) {rot.matrix [0][1], rot.matrix [1][1], rot.matrix [2][1]};
    axes [2] = (vector3) {rot.matrix [0][2], rot.matrix [1][2], rot.matrix [2][2]};
    float projection = rb -> half_extensions.x * fabsf (vector3_dot (axes [0], axis)) +
                       rb -> half_extensions.y * fabsf (vector3_dot (axes [1], axis)) +
                       rb -> half_extensions.z * fabsf (vector3_dot (axes [2], axis));
    return vector3_dot (rb -> position, axis) - projection;
} // Helper: Get highest point of OBB along an axis
static float get_obb_max_along_axis (rigidbody *rb, vector3 axis) {
    if (rb -> type == object_sphere) return vector3_dot (rb -> position, axis) + rb -> radius;
    vector3 axes [3];
    math3 rot = vector4_to_math3 (rb -> orientation);
    axes [0] = (vector3) {rot.matrix [0][0], rot.matrix [1][0], rot.matrix [2][0]};
    axes [1] = (vector3) {rot.matrix [0][1], rot.matrix [1][1], rot.matrix [2][1]};
    axes [2] = (vector3) {rot.matrix [0][2], rot.matrix [1][2], rot.matrix [2][2]};
    float projection = rb -> half_extensions.x * fabsf (vector3_dot (axes [0], axis)) +
                       rb -> half_extensions.y * fabsf (vector3_dot (axes [1], axis)) +
                       rb -> half_extensions.z * fabsf (vector3_dot (axes [2], axis));
    return vector3_dot (rb -> position, axis) + projection;
} void boundary_apply_floor (rigidbody *rigid_body, float floor_y_level) {
    if (rigid_body -> static_state) {return;}
    float min_y = get_obb_min_along_axis (rigid_body, (vector3) {0, 1, 0});
    if (min_y < floor_y_level) {
        rigid_body -> position.y += (floor_y_level - min_y);
        if (rigid_body -> velocity.y < 0) {
            rigid_body -> velocity.y = -rigid_body -> velocity.y * rigid_body -> restitution;
            rigid_body -> velocity.x *= 0.98f;
            rigid_body -> velocity.z *= 0.98f;
        } rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
    }
} void boundary_apply_box (rigidbody *rigid_body, vector3 min_bounds, vector3 max_bounds) {
    if (rigid_body -> static_state) {return;}
    // X axis
    float min_x = get_obb_min_along_axis (rigid_body, (vector3) {1, 0, 0});
    if (min_x < min_bounds.x) {
        rigid_body -> position.x += (min_bounds.x - min_x);
        if (rigid_body -> velocity.x < 0) {
            rigid_body -> velocity.x = -rigid_body -> velocity.x * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    } float max_x = get_obb_max_along_axis (rigid_body, (vector3) {1, 0, 0});
    if (max_x > max_bounds.x) {
        rigid_body -> position.x -= (max_x - max_bounds.x);
        if (rigid_body -> velocity.x > 0) {
            rigid_body -> velocity.x = -rigid_body -> velocity.x * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    } // Y axis
    float min_y = get_obb_min_along_axis (rigid_body, (vector3) {0, 1, 0});
    if (min_y < min_bounds.y) {
        rigid_body -> position.y += (min_bounds.y - min_y);
        if (rigid_body -> velocity.y < 0) {
            rigid_body -> velocity.y = -rigid_body -> velocity.y * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    } float max_y = get_obb_max_along_axis (rigid_body, (vector3) {0, 1, 0});
    if (max_y > max_bounds.y) {
        rigid_body -> position.y -= (max_y - max_bounds.y);
        if (rigid_body -> velocity.y > 0) {
            rigid_body -> velocity.y = -rigid_body -> velocity.y * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    } // Z axis
    float min_z = get_obb_min_along_axis (rigid_body, (vector3) {0, 0, 1});
    if (min_z < min_bounds.z) {
        rigid_body -> position.z += (min_bounds.z - min_z);
        if (rigid_body -> velocity.z < 0) {
            rigid_body -> velocity.z = -rigid_body -> velocity.z * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    } float max_z = get_obb_max_along_axis (rigid_body, (vector3) {0, 0, 1});
    if (max_z > max_bounds.z) {
        rigid_body -> position.z -= (max_z - max_bounds.z);
        if (rigid_body -> velocity.z > 0) {
            rigid_body -> velocity.z = -rigid_body -> velocity.z * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    }
}

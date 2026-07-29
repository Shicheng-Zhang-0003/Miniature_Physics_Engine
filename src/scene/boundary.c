#include "../mpe_engine.h"
#include "boundary.h"
#include <math.h>

/* A3_PATCH_18_BOUNDARY_FLOOR_EMERGENCY */
/* MPE_TASK_08_FLOOR_EMERGENCY_TUNING_BEGIN */
#define A3_BOUNDARY_FLOOR_EMERGENCY_SLOP 0.05f
#define A3_BOUNDARY_FLOOR_VELOCITY_SLOP 0.10f
/* MPE_TASK_08_FLOOR_EMERGENCY_TUNING_END */
// Helper: Get lowest point of OBB along an axis
static float get_obb_min_along_axis (rigidbody *rigid_body, vector3 axis) {
if (rigid_body -> type == object_sphere) return vector3_dot (rigid_body -> position, axis) - rigid_body -> radius;
/* MPE_TASK_16_BOUNDARY_CACHED_AXES_MIN_BEGIN */
vector3 *axes = rigid_body -> cached_axes;
float projection =
rigid_body -> half_extensions.x * fabsf (vector3_dot (axes [0], axis)) +
rigid_body -> half_extensions.y * fabsf (vector3_dot (axes [1], axis)) +
rigid_body -> half_extensions.z * fabsf (vector3_dot (axes [2], axis));
/* MPE_TASK_16_BOUNDARY_CACHED_AXES_MIN_END */
return vector3_dot (rigid_body -> position, axis) - projection;
} // Helper: Get highest point of OBB along an axis
static float get_obb_max_along_axis (rigidbody *rigid_body, vector3 axis) {
if (rigid_body -> type == object_sphere) return vector3_dot (rigid_body -> position, axis) + rigid_body -> radius;
/* MPE_TASK_16_BOUNDARY_CACHED_AXES_MAX_BEGIN */
vector3 *axes = rigid_body -> cached_axes;
float projection =
rigid_body -> half_extensions.x * fabsf (vector3_dot (axes [0], axis)) +
rigid_body -> half_extensions.y * fabsf (vector3_dot (axes [1], axis)) +
rigid_body -> half_extensions.z * fabsf (vector3_dot (axes [2], axis));
/* MPE_TASK_16_BOUNDARY_CACHED_AXES_MAX_END */
return vector3_dot (rigid_body -> position, axis) + projection;
} void boundary_apply_floor (rigidbody *rigid_body, float floor_y_level) {
    if (rigid_body -> static_state) {return;}
    float min_y = get_obb_min_along_axis (rigid_body, (vector3) {0, 1, 0});
    if (min_y < (floor_y_level - A3_BOUNDARY_FLOOR_EMERGENCY_SLOP)) {
/* MPE_TASK_08_FLOOR_APPLY_BEGIN */
float a3_floor_penetration = floor_y_level - min_y;

rigid_body -> position.y += a3_floor_penetration;

if (rigid_body -> velocity.y < 0.0f) {
if (a3_floor_penetration > A3_BOUNDARY_FLOOR_VELOCITY_SLOP) {
rigidbody_wake (rigid_body);
rigid_body -> velocity.y = -rigid_body -> velocity.y * rigid_body -> restitution;
rigid_body -> velocity.x *= 0.98f;
rigid_body -> velocity.z *= 0.98f;
rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
} else {
rigid_body -> velocity.y = 0.0f;
}
} else if (a3_floor_penetration > A3_BOUNDARY_FLOOR_VELOCITY_SLOP) {
rigidbody_wake (rigid_body);
rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
}
/* MPE_TASK_08_FLOOR_APPLY_END */
}
} void boundary_apply_box (rigidbody *rigid_body, vector3 min_bounds, vector3 max_bounds) {
    if (rigid_body -> static_state) {return;}
    // X axis
    float min_x = get_obb_min_along_axis (rigid_body, (vector3) {1, 0, 0});
    if (min_x < min_bounds.x) {
        rigid_body -> position.x += (min_bounds.x - min_x);
rigidbody_wake (rigid_body); /* A3_PATCH_46_BOUNDARY_WAKE */
        if (rigid_body -> velocity.x < 0) {
            rigid_body -> velocity.x = -rigid_body -> velocity.x * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    } float max_x = get_obb_max_along_axis (rigid_body, (vector3) {1, 0, 0});
    if (max_x > max_bounds.x) {
        rigid_body -> position.x -= (max_x - max_bounds.x);
rigidbody_wake (rigid_body); /* A3_PATCH_46_BOUNDARY_WAKE */
        if (rigid_body -> velocity.x > 0) {
            rigid_body -> velocity.x = -rigid_body -> velocity.x * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    } // Y axis
    float min_y = get_obb_min_along_axis (rigid_body, (vector3) {0, 1, 0});
    if (min_y < (min_bounds.y - A3_BOUNDARY_FLOOR_EMERGENCY_SLOP)) {
/* MPE_TASK_08_BOX_FLOOR_APPLY_BEGIN */
float a3_floor_penetration = min_bounds.y - min_y;

rigid_body -> position.y += a3_floor_penetration;

if (rigid_body -> velocity.y < 0.0f) {
if (a3_floor_penetration > A3_BOUNDARY_FLOOR_VELOCITY_SLOP) {
rigidbody_wake (rigid_body);
rigid_body -> velocity.y = -rigid_body -> velocity.y * rigid_body -> restitution;
rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
} else {
rigid_body -> velocity.y = 0.0f;
}
} else if (a3_floor_penetration > A3_BOUNDARY_FLOOR_VELOCITY_SLOP) {
rigidbody_wake (rigid_body);
}
/* MPE_TASK_08_BOX_FLOOR_APPLY_END */
} float max_y = get_obb_max_along_axis (rigid_body, (vector3) {0, 1, 0});
    if (max_y > max_bounds.y) {
        rigid_body -> position.y -= (max_y - max_bounds.y);
rigidbody_wake (rigid_body); /* A3_PATCH_46_BOUNDARY_WAKE */
        if (rigid_body -> velocity.y > 0) {
            rigid_body -> velocity.y = -rigid_body -> velocity.y * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    } // Z axis
    float min_z = get_obb_min_along_axis (rigid_body, (vector3) {0, 0, 1});
    if (min_z < min_bounds.z) {
        rigid_body -> position.z += (min_bounds.z - min_z);
rigidbody_wake (rigid_body); /* A3_PATCH_46_BOUNDARY_WAKE */
        if (rigid_body -> velocity.z < 0) {
            rigid_body -> velocity.z = -rigid_body -> velocity.z * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    } float max_z = get_obb_max_along_axis (rigid_body, (vector3) {0, 0, 1});
    if (max_z > max_bounds.z) {
        rigid_body -> position.z -= (max_z - max_bounds.z);
rigidbody_wake (rigid_body); /* A3_PATCH_46_BOUNDARY_WAKE */
        if (rigid_body -> velocity.z > 0) {
            rigid_body -> velocity.z = -rigid_body -> velocity.z * rigid_body -> restitution;
            rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, 0.98f);
        }
    }
}

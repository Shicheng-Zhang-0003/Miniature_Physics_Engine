#include "../mpe_engine.h"
#include "boundary.h"
#include <math.h>

/* MPE_TASK_08_FLOOR_EMERGENCY_TUNING_BEGIN */
/* MPE_TASK_08_FLOOR_EMERGENCY_TUNING_END */
// Helper: Get lowest point of OBB along an axis
static float get_obb_min_along_axis(rigidbody *rigid_body, vector3 axis) {
    if (rigid_body->type == object_sphere)
        return vector3_dot(rigid_body->position, axis) - rigid_body->radius;
    /* MPE_TASK_16_BOUNDARY_CACHED_AXES_MIN_BEGIN */
    vector3 *axes = rigid_body->cached_axes;
    float projection = rigid_body->half_extensions.x * fabsf(vector3_dot(axes[0], axis)) +
                       rigid_body->half_extensions.y * fabsf(vector3_dot(axes[1], axis)) +
                       rigid_body->half_extensions.z * fabsf(vector3_dot(axes[2], axis));
    /* MPE_TASK_16_BOUNDARY_CACHED_AXES_MIN_END */
    return vector3_dot(rigid_body->position, axis) - projection;
} // Helper: Get highest point of OBB along an axis
static float get_obb_max_along_axis(rigidbody *rigid_body, vector3 axis) {
    if (rigid_body->type == object_sphere)
        return vector3_dot(rigid_body->position, axis) + rigid_body->radius;
    /* MPE_TASK_16_BOUNDARY_CACHED_AXES_MAX_BEGIN */
    vector3 *axes = rigid_body->cached_axes;
    float projection = rigid_body->half_extensions.x * fabsf(vector3_dot(axes[0], axis)) +
                       rigid_body->half_extensions.y * fabsf(vector3_dot(axes[1], axis)) +
                       rigid_body->half_extensions.z * fabsf(vector3_dot(axes[2], axis));
    /* MPE_TASK_16_BOUNDARY_CACHED_AXES_MAX_END */
    return vector3_dot(rigid_body->position, axis) + projection;
}
/* World-edge safety net: PERFECTLY PLASTIC positional clamp.
 *
 * Bounce (restitution) belongs to material contacts in the solver, which
 * combines both bodies' properties. The boundary is not a material, so it
 * never reflects velocity and never applies magic damping: it repositions
 * bodies inside the playable volume and kills only the inward (escaping)
 * velocity component. Deep escape still wakes the body so it rejoins. */
void boundary_apply_floor(rigidbody *rigid_body, float floor_y_level) {
    if (rigid_body->static_state) {
        return;
    }
    float min_y = get_obb_min_along_axis(rigid_body, (vector3){0, 1, 0});
    if (min_y < (floor_y_level - g_cfg.boundary.floor_emergency_slop)) {
        rigid_body->position.y += (floor_y_level - min_y);
        if (rigid_body->velocity.y < 0.0f) {
            rigid_body->velocity.y = 0.0f;
        }
        rigidbody_wake(rigid_body);
    }
}
void boundary_apply_box(rigidbody *rigid_body, vector3 min_bounds, vector3 max_bounds) {
    if (rigid_body->static_state) {
        return;
    }
    // X axis
    float min_x = get_obb_min_along_axis(rigid_body, (vector3){1, 0, 0});
    if (min_x < min_bounds.x) {
        rigid_body->position.x += (min_bounds.x - min_x);
        rigidbody_wake(rigid_body);
        if (rigid_body->velocity.x < 0) {
            rigid_body->velocity.x = 0.0f;
        }
    }
    float max_x = get_obb_max_along_axis(rigid_body, (vector3){1, 0, 0});
    if (max_x > max_bounds.x) {
        rigid_body->position.x -= (max_x - max_bounds.x);
        rigidbody_wake(rigid_body);
        if (rigid_body->velocity.x > 0) {
            rigid_body->velocity.x = 0.0f;
        }
    } // Y axis
    float min_y = get_obb_min_along_axis(rigid_body, (vector3){0, 1, 0});
    if (min_y < (min_bounds.y - g_cfg.boundary.floor_emergency_slop)) {
        rigid_body->position.y += (min_bounds.y - min_y);
        rigidbody_wake(rigid_body);
        if (rigid_body->velocity.y < 0.0f) {
            rigid_body->velocity.y = 0.0f;
        }
    }
    float max_y = get_obb_max_along_axis(rigid_body, (vector3){0, 1, 0});
    if (max_y > max_bounds.y) {
        rigid_body->position.y -= (max_y - max_bounds.y);
        rigidbody_wake(rigid_body);
        if (rigid_body->velocity.y > 0) {
            rigid_body->velocity.y = 0.0f;
        }
    } // Z axis
    float min_z = get_obb_min_along_axis(rigid_body, (vector3){0, 0, 1});
    if (min_z < min_bounds.z) {
        rigid_body->position.z += (min_bounds.z - min_z);
        rigidbody_wake(rigid_body);
        if (rigid_body->velocity.z < 0) {
            rigid_body->velocity.z = 0.0f;
        }
    }
    float max_z = get_obb_max_along_axis(rigid_body, (vector3){0, 0, 1});
    if (max_z > max_bounds.z) {
        rigid_body->position.z -= (max_z - max_bounds.z);
        rigidbody_wake(rigid_body);
        if (rigid_body->velocity.z > 0) {
            rigid_body->velocity.z = 0.0f;
        }
    }
}

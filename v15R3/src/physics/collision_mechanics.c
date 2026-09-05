#include "../mpe_engine.h"
#include "collision_mechanics.h"
#include "../core/physics_world.h" /* MFS_131 */
#include <stdint.h>
#include <stdlib.h>


static cached_contact contact_impulse_cache[max_cached_contacts];
static int contact_impulse_cache_count = 0;

bool collision_dual_sphere(rigidbody *rigidbody_object_a, rigidbody *rigidbody_object_b,
                           collision_data *collision_output_data) {
    vector3 relative_position_vector = vector3_subtraction(rigidbody_object_b->position, rigidbody_object_a->position);
    float distance_between_centres_squared = vector3_length_squared(relative_position_vector);
    float total_combined_radius = rigidbody_object_a->radius + rigidbody_object_b->radius;
    if (distance_between_centres_squared >= total_combined_radius * total_combined_radius) {
        return false;
    }
    float distance_between_centres = sqrtf(distance_between_centres_squared);
    collision_output_data->object_a = rigidbody_object_a;
    collision_output_data->object_b = rigidbody_object_b;
    const float minimum_distance_threshold_epsilon = 0.0001f;
    if (distance_between_centres > minimum_distance_threshold_epsilon) {
        collision_output_data->normal_vector =
            vector3_scaling(relative_position_vector, 1.0f / distance_between_centres);
    } else {
        collision_output_data->normal_vector = (vector3){0.0f, 1.0f, 0.0f};
    }

    contact_point_data *cp = &collision_output_data->contacts[0];
    cp->penetration = total_combined_radius - distance_between_centres;
    cp->position = vector3_addition(rigidbody_object_a->position,
                                    vector3_scaling(collision_output_data->normal_vector, rigidbody_object_a->radius));
    collision_output_data->contact_count = 1;
    return true;
}

float project_obb(rigidbody *rigid_body, vector3 axis, vector3 axes[3]) {
    return rigid_body->half_extensions.x * fabsf(vector3_dot(axes[0], axis)) +
           rigid_body->half_extensions.y * fabsf(vector3_dot(axes[1], axis)) +
           rigid_body->half_extensions.z * fabsf(vector3_dot(axes[2], axis));
}

bool collision_sphere_cube(rigidbody *sphere, rigidbody *cube, collision_data *collision_output_data) {
    vector3 *axes_cube = cube->cached_axes;
    vector3 relative_position = vector3_subtraction(sphere->position, cube->position);
    vector3 closest_point = cube->position;
    bool inside = true;
    float minimum_distance = 1000000.0f;
    int nearest_face_axis = 0;
    float nearest_face_sign = 1.0f;
    for (int axis_index = 0; axis_index < 3; axis_index++) {
        float distance = vector3_dot(relative_position, axes_cube[axis_index]);
        float extent = (axis_index == 0)   ? cube->half_extensions.x
                       : (axis_index == 1) ? cube->half_extensions.y
                                           : cube->half_extensions.z;
        if (distance > extent) {
            distance = extent;
            inside = false;
        } else if (distance < -extent) {
            distance = -extent;
            inside = false;
        } else {
            float d_pos = extent - distance;
            float d_neg = extent + distance;
            if (d_pos < minimum_distance) {
                minimum_distance = d_pos;
                nearest_face_axis = axis_index;
                nearest_face_sign = 1.0f;
            }
            if (d_neg < minimum_distance) {
                minimum_distance = d_neg;
                nearest_face_axis = axis_index;
                nearest_face_sign = -1.0f;
            }
        }
        closest_point = vector3_addition(closest_point, vector3_scaling(axes_cube[axis_index], distance));
    }
    vector3 difference = vector3_subtraction(sphere->position, closest_point);
    float distance_sq = vector3_length_squared(difference);
    if (!inside && distance_sq > sphere->radius * sphere->radius)
        return false;
    collision_output_data->object_a = sphere;
    collision_output_data->object_b = cube;

    contact_point_data *cp = &collision_output_data->contacts[0];
    if (inside) {
        collision_output_data->normal_vector = vector3_scaling(axes_cube[nearest_face_axis], nearest_face_sign);
        cp->penetration = sphere->radius + minimum_distance;
        cp->position = closest_point;
    } else {
        float distance = sqrtf(distance_sq);
        if (distance > 0.0001f) {
            collision_output_data->normal_vector = vector3_scaling(difference, -1.0f / distance);
        } else {
            collision_output_data->normal_vector = (vector3){0.0f, -1.0f, 0.0f};
        }
        cp->penetration = sphere->radius - distance;
        cp->position = closest_point;
    }
    collision_output_data->contact_count = 1;
    return true;
}

static void clip_obb_faces(rigidbody *ref_body, rigidbody *inc_body, vector3 normal, float overlap,
                           collision_data *collision_output_data) {
    vector3 *ref_axes = ref_body->cached_axes;
    vector3 ref_extents = ref_body->half_extensions;
    int ref_axis_idx = 0;
    float max_dot = -1.0f;
    for (int i = 0; i < 3; i++) {
        float dot_val = vector3_dot(ref_axes[i], normal);
        if (fabsf(dot_val) > max_dot) {
            max_dot = fabsf(dot_val);
            ref_axis_idx = i;
        }
    }
    vector3 ref_normal = ref_axes[ref_axis_idx];
    if (vector3_dot(ref_normal, normal) < 0.0f) {
        ref_normal = vector3_scaling(ref_normal, -1.0f);
    }
    int side_axis_idx_1 = (ref_axis_idx + 1) % 3;
    int side_axis_idx_2 = (ref_axis_idx + 2) % 3;
    vector3 side_axis_1 = ref_axes[side_axis_idx_1];
    vector3 side_axis_2 = ref_axes[side_axis_idx_2];
    float ref_extent_n = (ref_axis_idx == 0) ? ref_extents.x : (ref_axis_idx == 1) ? ref_extents.y : ref_extents.z;
    float ref_extent_1 = (side_axis_idx_1 == 0)   ? ref_extents.x
                         : (side_axis_idx_1 == 1) ? ref_extents.y
                                                  : ref_extents.z;
    float ref_extent_2 = (side_axis_idx_2 == 0)   ? ref_extents.x
                         : (side_axis_idx_2 == 1) ? ref_extents.y
                                                  : ref_extents.z;
    vector3 ref_center = vector3_addition(ref_body->position, vector3_scaling(ref_normal, ref_extent_n));

    vector3 *inc_axes = inc_body->cached_axes;
    vector3 inc_extents = inc_body->half_extensions;
    /* MPE_F5_FACE_CLIP_INCIDENT_FIX_BEGIN */
    int inc_axis_idx = 0;
    float max_abs_dot = -1.0f;

    for (int i = 0; i < 3; i++) {
        float dot_val = fabsf(vector3_dot(inc_axes[i], ref_normal));
        if (dot_val > max_abs_dot) {
            max_abs_dot = dot_val;
            inc_axis_idx = i;
        }
    }

    if (max_abs_dot < 0.000001f) {
        inc_axis_idx = 0;
    }

    vector3 inc_normal = inc_axes[inc_axis_idx];

    if (vector3_dot(inc_normal, ref_normal) > 0.0f) {
        inc_normal = vector3_scaling(inc_normal, -1.0f);
    }
    /* MPE_F5_FACE_CLIP_INCIDENT_FIX_END */
    int inc_u_idx = (inc_axis_idx + 1) % 3;
    int inc_v_idx = (inc_axis_idx + 2) % 3;
    vector3 inc_u_axis = inc_axes[inc_u_idx];
    vector3 inc_v_axis = inc_axes[inc_v_idx];
    float inc_extent_n = (inc_axis_idx == 0) ? inc_extents.x : (inc_axis_idx == 1) ? inc_extents.y : inc_extents.z;
    float inc_extent_u = (inc_u_idx == 0) ? inc_extents.x : (inc_u_idx == 1) ? inc_extents.y : inc_extents.z;
    float inc_extent_v = (inc_v_idx == 0) ? inc_extents.x : (inc_v_idx == 1) ? inc_extents.y : inc_extents.z;
    vector3 inc_center = vector3_addition(inc_body->position, vector3_scaling(inc_normal, inc_extent_n));

    vector3 input_polygon[16];
    input_polygon[0] = vector3_addition(inc_center, vector3_addition(vector3_scaling(inc_u_axis, inc_extent_u),
                                                                     vector3_scaling(inc_v_axis, inc_extent_v)));
    input_polygon[1] = vector3_addition(inc_center, vector3_subtraction(vector3_scaling(inc_u_axis, inc_extent_u),
                                                                        vector3_scaling(inc_v_axis, inc_extent_v)));
    input_polygon[2] = vector3_subtraction(inc_center, vector3_addition(vector3_scaling(inc_u_axis, inc_extent_u),
                                                                        vector3_scaling(inc_v_axis, inc_extent_v)));
    input_polygon[3] = vector3_subtraction(inc_center, vector3_subtraction(vector3_scaling(inc_u_axis, inc_extent_u),
                                                                           vector3_scaling(inc_v_axis, inc_extent_v)));
    int input_count = 4;

    vector3 clip_normals[4];
    float clip_offsets[4];
    clip_normals[0] = side_axis_1;
    clip_offsets[0] = vector3_dot(ref_center, side_axis_1) + ref_extent_1;
    clip_normals[1] = vector3_scaling(side_axis_1, -1.0f);
    clip_offsets[1] = -vector3_dot(ref_center, side_axis_1) + ref_extent_1;
    clip_normals[2] = side_axis_2;
    clip_offsets[2] = vector3_dot(ref_center, side_axis_2) + ref_extent_2;
    clip_normals[3] = vector3_scaling(side_axis_2, -1.0f);
    clip_offsets[3] = -vector3_dot(ref_center, side_axis_2) + ref_extent_2;

    vector3 output_polygon[16];
    for (int p = 0; p < 4; p++) {
        int output_count = 0;
        if (input_count < 2) {
            input_count = 0;
            break;
        }
        vector3 v1 = input_polygon[input_count - 1];
        float d1 = vector3_dot(v1, clip_normals[p]) - clip_offsets[p];
        for (int i = 0; i < input_count; i++) {
            vector3 v2 = input_polygon[i];
            float d2 = vector3_dot(v2, clip_normals[p]) - clip_offsets[p];
            if (d1 <= 0.0f && d2 <= 0.0f) {
                if (output_count < 16) {
                    output_polygon[output_count++] = v2;
                }
            } else if (d1 <= 0.0f && d2 > 0.0f) {
                float t = d1 / (d1 - d2);
                vector3 v_int = vector3_addition(v1, vector3_scaling(vector3_subtraction(v2, v1), t));
                if (output_count < 16) {
                    output_polygon[output_count++] = v_int;
                }
            } else if (d1 > 0.0f && d2 <= 0.0f) {
                float t = d1 / (d1 - d2);
                vector3 v_int = vector3_addition(v1, vector3_scaling(vector3_subtraction(v2, v1), t));
                if (output_count < 16) {
                    output_polygon[output_count++] = v_int;
                }
                if (output_count < 16) {
                    output_polygon[output_count++] = v2;
                }
            }
            v1 = v2;
            d1 = d2;
        }
        input_count = output_count;
        for (int i = 0; i < input_count; i++) {
            input_polygon[i] = output_polygon[i];
        }
    }
    float ref_height = vector3_dot(ref_center, ref_normal);
    int manifold_idx = 0;
    for (int i = 0; i < input_count; i++) {
        vector3 v = input_polygon[i];
        float penetration = ref_height - vector3_dot(v, ref_normal);
        if (penetration >= -0.01f) {
            if (manifold_idx < 4) {
                contact_point_data *cp = &collision_output_data->contacts[manifold_idx++];
                cp->position = v;
                cp->penetration = penetration > 0.0f ? penetration : 0.0f;
            }
        }
    }
    if (manifold_idx == 0) {
        contact_point_data *cp = &collision_output_data->contacts[0];
        cp->position = inc_center;
        float penetration = ref_height - vector3_dot(inc_center, ref_normal);
        cp->penetration = penetration > 0.0f ? penetration : overlap;
        manifold_idx = 1;
    }
    collision_output_data->contact_count = manifold_idx;
}

static inline float a3_cube_extent_axis(rigidbody *cube, int axis_index) {
    if (axis_index == 0) {
        return cube->half_extensions.x;
    }
    if (axis_index == 1) {
        return cube->half_extensions.y;
    }
    return cube->half_extensions.z;
}

/* MPE_TASK_04_CUBE_NORMAL_CONSISTENCY_BEGIN */
static void a3_task04_enforce_cube_normal_consistency(collision_data *collision_output_data, rigidbody *cube_a,
                                                      rigidbody *cube_b) {
    if ((!collision_output_data) || (!cube_a) || (!cube_b)) {
        return;
    }

    collision_output_data->object_a = cube_a;
    collision_output_data->object_b = cube_b;

    if (collision_output_data->contact_count < 0) {
        collision_output_data->contact_count = 0;
    }

    if (collision_output_data->contact_count > 4) {
        collision_output_data->contact_count = 4;
    }

    vector3 normal = collision_output_data->normal_vector;
    float normal_length_squared = vector3_length_squared(normal);

    if ((!isfinite(normal_length_squared)) || (normal_length_squared < 0.000001f)) {
        normal = (vector3){0.0f, 1.0f, 0.0f};
    } else {
        normal = vector3_scaling(normal, 1.0f / sqrtf(normal_length_squared));
    }

    vector3 a_to_b = vector3_subtraction(cube_b->position, cube_a->position);
    float a_to_b_length_squared = vector3_length_squared(a_to_b);

    if (a_to_b_length_squared > 0.000001f) {
        /*
         * Convention:
         * collision normal points from object_a toward object_b.
         */
        if (vector3_dot(a_to_b, normal) < 0.0f) {
            normal = vector3_scaling(normal, -1.0f);
        }
    } else {
        /*
         * Near-coincident centres:
         * choose a deterministic orientation by forcing the first
         * significant component to be positive.
         */
        if (fabsf(normal.x) > 0.000001f) {
            if (normal.x < 0.0f) {
                normal = vector3_scaling(normal, -1.0f);
            }
        } else if (fabsf(normal.y) > 0.000001f) {
            if (normal.y < 0.0f) {
                normal = vector3_scaling(normal, -1.0f);
            }
        } else {
            if (normal.z < 0.0f) {
                normal = vector3_scaling(normal, -1.0f);
            }
        }
    }

    collision_output_data->normal_vector = normal;
}
/* MPE_TASK_04_CUBE_NORMAL_CONSISTENCY_END */

bool collision_dual_cube(rigidbody *cube_a, rigidbody *cube_b, collision_data *collision_output_data) {
    vector3 *axes_a = cube_a->cached_axes;
    vector3 *axes_b = cube_b->cached_axes;
    vector3 relative_position = vector3_subtraction(cube_b->position, cube_a->position);
    float minimum_overlap = 1000000.0f;
    vector3 best_axis = {0, 0, 0};
    int best_axis_index = -1;

    for (int axis_index = 0; axis_index < 6; axis_index++) {
        vector3 axis = (axis_index < 3) ? axes_a[axis_index] : axes_b[axis_index - 3];
        float projection_a = project_obb(cube_a, axis, axes_a);
        float projection_b = project_obb(cube_b, axis, axes_b);
        float distance = fabsf(vector3_dot(relative_position, axis));
        float overlap = projection_a + projection_b - distance;
        if (overlap < 0.0f) {
            return false;
        }
        if (overlap < minimum_overlap) {
            minimum_overlap = overlap;
            best_axis = axis;
            best_axis_index = axis_index;
        }
    }
    for (int axis_index_a = 0; axis_index_a < 3; axis_index_a++) {
        for (int axis_index_b = 0; axis_index_b < 3; axis_index_b++) {
            vector3 axis = vector3_cross(axes_a[axis_index_a], axes_b[axis_index_b]);
            float length_squared = vector3_length_squared(axis);
            if (length_squared < 0.0001f)
                continue;
            axis = vector3_scaling(axis, 1.0f / sqrtf(length_squared));
            float projection_a = project_obb(cube_a, axis, axes_a);
            float projection_b = project_obb(cube_b, axis, axes_b);
            float distance = fabsf(vector3_dot(relative_position, axis));
            float overlap = projection_a + projection_b - distance;
            if (overlap < 0.0f) {
                return false;
            }
            if (overlap < minimum_overlap) {
                minimum_overlap = overlap;
                best_axis = axis;
                best_axis_index = 6 + axis_index_a * 3 + axis_index_b;
            }
        }
    }
    if (vector3_dot(relative_position, best_axis) < 0) {
        best_axis = vector3_scaling(best_axis, -1.0f);
    }
    collision_output_data->object_a = cube_a;
    collision_output_data->object_b = cube_b;
    collision_output_data->normal_vector = best_axis;

    if (best_axis_index >= 6) {
        int edge_axis_a = (best_axis_index - 6) / 3;
        int edge_axis_b = (best_axis_index - 6) % 3;

        vector3 edge_dir_a = axes_a[edge_axis_a];
        vector3 edge_dir_b = axes_b[edge_axis_b];

        float edge_extent_a = a3_cube_extent_axis(cube_a, edge_axis_a);
        float edge_extent_b = a3_cube_extent_axis(cube_b, edge_axis_b);

        vector3 anchor_a = cube_a->position;

        for (int axis_index = 0; axis_index < 3; axis_index++) {
            if (axis_index == edge_axis_a) {
                continue;
            }

            float extent = a3_cube_extent_axis(cube_a, axis_index);
            vector3 axis = axes_a[axis_index];

            if (vector3_dot(axis, best_axis) > 0.0f) {
                anchor_a = vector3_addition(anchor_a, vector3_scaling(axis, extent));
            } else {
                anchor_a = vector3_subtraction(anchor_a, vector3_scaling(axis, extent));
            }
        }

        vector3 anchor_b = cube_b->position;

        for (int axis_index = 0; axis_index < 3; axis_index++) {
            if (axis_index == edge_axis_b) {
                continue;
            }

            float extent = a3_cube_extent_axis(cube_b, axis_index);
            vector3 axis = axes_b[axis_index];

            if (vector3_dot(axis, best_axis) > 0.0f) {
                anchor_b = vector3_subtraction(anchor_b, vector3_scaling(axis, extent));
            } else {
                anchor_b = vector3_addition(anchor_b, vector3_scaling(axis, extent));
            }
        }

        vector3 anchor_delta = vector3_subtraction(anchor_a, anchor_b);

        float aa = vector3_dot(edge_dir_a, edge_dir_a);
        float bb = vector3_dot(edge_dir_a, edge_dir_b);
        float cc = vector3_dot(edge_dir_b, edge_dir_b);
        float d = vector3_dot(edge_dir_a, anchor_delta);
        float e = vector3_dot(edge_dir_b, anchor_delta);

        float denominator = aa * cc - bb * bb;

        float t_a = 0.0f;
        float t_b = 0.0f;

        if (fabsf(denominator) > 0.000001f) {
            t_a = (bb * e - cc * d) / denominator;
            t_b = (aa * e - bb * d) / denominator;
        }

        if (t_a > edge_extent_a) {
            t_a = edge_extent_a;
        }
        if (t_a < -edge_extent_a) {
            t_a = -edge_extent_a;
        }
        if (t_b > edge_extent_b) {
            t_b = edge_extent_b;
        }
        if (t_b < -edge_extent_b) {
            t_b = -edge_extent_b;
        }

        if (cc > 0.000001f) {
            t_b = (e + bb * t_a) / cc;
            if (t_b > edge_extent_b) {
                t_b = edge_extent_b;
            }
            if (t_b < -edge_extent_b) {
                t_b = -edge_extent_b;
            }
        }

        if (aa > 0.000001f) {
            t_a = (bb * t_b - d) / aa;
            if (t_a > edge_extent_a) {
                t_a = edge_extent_a;
            }
            if (t_a < -edge_extent_a) {
                t_a = -edge_extent_a;
            }
        }

        vector3 closest_a = vector3_addition(anchor_a, vector3_scaling(edge_dir_a, t_a));
        vector3 closest_b = vector3_addition(anchor_b, vector3_scaling(edge_dir_b, t_b));
        vector3 contact_point = vector3_scaling(vector3_addition(closest_a, closest_b), 0.5f);

        collision_output_data->contact_count = 0;

        contact_point_data *cp = &collision_output_data->contacts[0];
        cp->position = contact_point;
        cp->penetration = minimum_overlap;
        collision_output_data->contact_count = 1;

        float parallel_alignment = fabsf(bb);
        float contact_spread = fminf(edge_extent_a, edge_extent_b) * 0.5f;

        if ((parallel_alignment > 0.95f) && (contact_spread > 0.05f)) {
            float t_offsets[2];
            t_offsets[0] = t_a - contact_spread;
            t_offsets[1] = t_a + contact_spread;

            for (int offset_index = 0; offset_index < 2; offset_index++) {
                if (collision_output_data->contact_count >= 4) {
                    break;
                }

                float sample_t_a = t_offsets[offset_index];

                if (sample_t_a > edge_extent_a) {
                    sample_t_a = edge_extent_a;
                }
                if (sample_t_a < -edge_extent_a) {
                    sample_t_a = -edge_extent_a;
                }

                float sample_t_b = t_b;

                if (cc > 0.000001f) {
                    sample_t_b = (e + bb * sample_t_a) / cc;
                    if (sample_t_b > edge_extent_b) {
                        sample_t_b = edge_extent_b;
                    }
                    if (sample_t_b < -edge_extent_b) {
                        sample_t_b = -edge_extent_b;
                    }
                }

                vector3 sample_closest_a = vector3_addition(anchor_a, vector3_scaling(edge_dir_a, sample_t_a));
                vector3 sample_closest_b = vector3_addition(anchor_b, vector3_scaling(edge_dir_b, sample_t_b));
                vector3 sample_contact_point =
                    vector3_scaling(vector3_addition(sample_closest_a, sample_closest_b), 0.5f);

                if (vector3_length_squared(vector3_subtraction(sample_contact_point, contact_point)) > 0.0001f) {
                    contact_point_data *extra_cp =
                        &collision_output_data->contacts[collision_output_data->contact_count];
                    extra_cp->position = sample_contact_point;
                    extra_cp->penetration = minimum_overlap;
                    collision_output_data->contact_count++;
                }
            }
        }
    } else {
        if (best_axis_index < 3) {
            clip_obb_faces(cube_a, cube_b, best_axis, minimum_overlap, collision_output_data);
        } else {
            clip_obb_faces(cube_b, cube_a, vector3_scaling(best_axis, -1.0f), minimum_overlap, collision_output_data);
            collision_output_data->object_a = cube_a;
            collision_output_data->object_b = cube_b;
        }
    }
    /* MPE_TASK_04_CUBE_NORMAL_CALL_BEGIN */
    a3_task04_enforce_cube_normal_consistency(collision_output_data, cube_a, cube_b);
    /* MPE_TASK_04_CUBE_NORMAL_CALL_END */
    return true;
}
static rigidbody *collision_static_plane_body_proxy(float plane_y) {
    static rigidbody static_plane_body;
    static int static_plane_initialized = 0;

    if (!static_plane_initialized) {
        rigidbody_initialisation_sphere(&static_plane_body, 1.0f, 0.0f, (vector3){0.0f, plane_y, 0.0f});
        static_plane_body.static_state = true;
        static_plane_body.inverse_mass = 0.0f;
        static_plane_body.inverse_inertia_tensor_local = (math3){{{0}}};
        static_plane_body.inverse_inertia_system = (math3){{{0}}};
        static_plane_body.restitution = 1.0f; /* A3_HOTFIX_FLOOR_BOUNCE */
        static_plane_body.object_id = 0xFFFFFFFFu; /* A3_PATCH_16_FLOOR_MANIFOLD */
        static_plane_body.object_generation = 1;
        static_plane_initialized = 1;
    }

    static_plane_body.position.y = plane_y;
    static_plane_body.friction_static = g_cfg.world.floor_friction_s;
    static_plane_body.friction_kinetic = g_cfg.world.floor_friction_k;

    return &static_plane_body;
}

bool collision_static_plane_sphere(rigidbody *sphere, float plane_y, collision_data *collision_output_data) {
    if (sphere->type != object_sphere) {
        return false;
    }

    float lowest_y = sphere->position.y - sphere->radius;
    float penetration = plane_y - lowest_y;

    if (penetration <= 0.0f) {
        return false;
    }

    rigidbody *plane_body = collision_static_plane_body_proxy(plane_y);

    collision_output_data->object_a = sphere;
    collision_output_data->object_b = plane_body;
    collision_output_data->normal_vector = (vector3){0.0f, -1.0f, 0.0f};
    collision_output_data->contact_count = 1;

    contact_point_data *cp = &collision_output_data->contacts[0];
    cp->position = (vector3){sphere->position.x, lowest_y, sphere->position.z};
    cp->penetration = penetration;

    return true;
}

bool collision_static_plane_cube(rigidbody *cube, float plane_y, collision_data *collision_output_data) {
    if (cube->type != object_cube) {
        return false;
    }

    vector3 *axes = cube->cached_axes;
    vector3 extents = cube->half_extensions;

    vector3 candidate_positions[8];
    float candidate_penetrations[8];
    int candidate_count = 0;

    for (int sx = 0; sx < 2; sx++) {
        float sign_x = sx ? 1.0f : -1.0f;

        for (int sy = 0; sy < 2; sy++) {
            float sign_y = sy ? 1.0f : -1.0f;

            for (int sz = 0; sz < 2; sz++) {
                float sign_z = sz ? 1.0f : -1.0f;

                vector3 corner = cube->position;
                corner = vector3_addition(corner, vector3_scaling(axes[0], sign_x * extents.x));
                corner = vector3_addition(corner, vector3_scaling(axes[1], sign_y * extents.y));
                corner = vector3_addition(corner, vector3_scaling(axes[2], sign_z * extents.z));

                float penetration = plane_y - corner.y;

                if (penetration > -0.005f) {
                    candidate_positions[candidate_count] = corner;
                    candidate_penetrations[candidate_count] = (penetration > 0.0f) ? penetration : 0.0f;
                    candidate_count++;
                }
            }
        }
    }

    if (candidate_count == 0) {
        return false;
    }

    rigidbody *plane_body = collision_static_plane_body_proxy(plane_y);

    collision_output_data->object_a = cube;
    collision_output_data->object_b = plane_body;
    collision_output_data->normal_vector = (vector3){0.0f, -1.0f, 0.0f};
    collision_output_data->contact_count = 0;

    int max_contacts = (candidate_count < 4) ? candidate_count : 4;

    for (int i = 0; i < max_contacts; i++) {
        int best = i;

        for (int j = i + 1; j < candidate_count; j++) {
            if (candidate_penetrations[j] > candidate_penetrations[best]) {
                best = j;
            }
        }

        if (best != i) {
            vector3 temp_position = candidate_positions[i];
            candidate_positions[i] = candidate_positions[best];
            candidate_positions[best] = temp_position;

            float temp_penetration = candidate_penetrations[i];
            candidate_penetrations[i] = candidate_penetrations[best];
            candidate_penetrations[best] = temp_penetration;
        }

        contact_point_data *cp = &collision_output_data->contacts[i];
        cp->position = candidate_positions[i];
        cp->penetration = candidate_penetrations[i];
        collision_output_data->contact_count++;
    }

    return true;
}

/* MPE_FTC_093: cylinder floor contact */
bool collision_static_plane_cylinder(rigidbody *cyl, float plane_y, collision_data *collision_output_data);
bool collision_static_plane_body(rigidbody *body, float plane_y, collision_data *collision_output_data) {
    if (body->type == object_cylinder) {return collision_static_plane_cylinder(body, plane_y, collision_output_data);}
    if (body->type == object_sphere) {
        return collision_static_plane_sphere(body, plane_y, collision_output_data);
    }
    if (body->type == object_cube) {
        return collision_static_plane_cube(body, plane_y, collision_output_data);
    }
    return false;
}

static inline vector4 collision_inverse_orientation(vector4 orientation) {
    return (vector4){orientation.w, -orientation.x, -orientation.y, -orientation.z};
}

static inline vector3 collision_world_offset_to_body_local(rigidbody *body, vector3 world_offset) {
    return vector4_rotate_to_vector3(collision_inverse_orientation(body->orientation), world_offset);
}

static inline vector3 collision_body_local_to_world_offset(rigidbody *body, vector3 local_offset) {
    return vector4_rotate_to_vector3(body->orientation, local_offset);
}

static int contact_cache_hit_count = 0;
static int contact_cache_miss_count = 0;

void contact_cache_stats_reset(void) {
    contact_cache_hit_count = 0;
    contact_cache_miss_count = 0;
}

int contact_cache_get_hits(void) {
    return contact_cache_hit_count;
}

int contact_cache_get_misses(void) {
    return contact_cache_miss_count;
}

/* MPE_TASK_05_CACHE_VALIDATE_BEGIN */
static uint32_t a3_task05_mix_u32(uint32_t hash_value, uint32_t input_value) {
    hash_value ^= input_value + 0x9e3779b9u + (hash_value << 6) + (hash_value >> 2);
    return hash_value;
}

static uint32_t a3_task05_float_bits(float value) {
    union {
        float float_value;
        uint32_t integer_value;
    } converter;

    converter.float_value = value;
    return converter.integer_value;
}

static uint32_t a3_task05_body_property_stamp(const rigidbody *rigid_body) {
    if (!rigid_body) {
        return 0;
    }

    uint32_t stamp = 2166136261u;

    stamp = a3_task05_mix_u32(stamp, (uint32_t) rigid_body->type);
    stamp = a3_task05_mix_u32(stamp, rigid_body->static_state ? 1u : 0u);

    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->mass));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->inverse_mass));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->radius));

    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->half_extensions.x));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->half_extensions.y));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->half_extensions.z));

    return stamp;
}

static bool a3_task05_cached_impulses_are_usable(float normal_impulse, float tangent_impulse) {
    if ((!isfinite(normal_impulse)) || (!isfinite(tangent_impulse))) {
        return false;
    }

    if (normal_impulse < 0.0f) {
        return false;
    }

    if (fabsf(normal_impulse) > 1000000.0f) {
        return false;
    }
    if (fabsf(tangent_impulse) > 1000000.0f) {
        return false;
    }

    return true;
}
/* MPE_TASK_05_CACHE_VALIDATE_END */

void collision_prepare_solver(collision_data *source, collision_data *m) {
    *m = *source;
    static int a3_patch_19_cache_reset_done = 0; /* A3_PATCH_19_CACHE_RESET */
    if (!a3_patch_19_cache_reset_done) {
        contact_impulse_cache_count = 0;
        a3_patch_19_cache_reset_done = 1;
    }

    for (int i = 0; i < m->contact_count; i++) {
        contact_point_data *cp = &m->contacts[i];
        cp->ra = vector3_subtraction(cp->position, m->object_a->position);
        cp->rb = vector3_subtraction(cp->position, m->object_b->position);
        cp->local_position_a =
            collision_world_offset_to_body_local(m->object_a, cp->ra); /* A3_PATCH_19_BODY_LOCAL_WARM_START */
        cp->local_position_b = collision_world_offset_to_body_local(m->object_b, cp->rb);

        cp->accumulated_normal_impulse = 0.0f;
        cp->accumulated_tangent_impulse = 0.0f;
        const float penetration_slop = g_cfg.solver.penetration_slop; /* MPE_TASK_30 */
        const float bias_factor = g_cfg.solver.bias_factor; /* MPE_TASK_30 */
        cp->separation_bias = bias_factor * fmaxf(cp->penetration - penetration_slop, 0.0f) * 60.0f;
        if (cp->separation_bias > g_cfg.solver.max_separation_bias) {
            cp->separation_bias = g_cfg.solver.max_separation_bias;
        }

        /* MPE_TASK_05_CACHE_MATCH_BEGIN */
        uint32_t cache_id_a = (m->object_a) ? m->object_a->object_id : 0;
        uint32_t cache_id_b = (m->object_b) ? m->object_b->object_id : 0;

        uint32_t cache_stamp_a = a3_task05_body_property_stamp(m->object_a);
        uint32_t cache_stamp_b = a3_task05_body_property_stamp(m->object_b);

        int cache_match_found = 0;

        for (int c = 0; c < contact_impulse_cache_count; c++) {
            cached_contact *cc = &contact_impulse_cache[c];

            if ((cache_id_a != 0) && (cache_id_b != 0) && (cc->object_id_a == cache_id_a) &&
                (cc->object_id_b == cache_id_b) && (cc->property_stamp_a == cache_stamp_a) &&
                (cc->property_stamp_b == cache_stamp_b)) {
                float dist_sq = vector3_length_squared(vector3_subtraction(cc->local_position_a, cp->local_position_a));

                if ((dist_sq < g_cfg.solver.warm_start_match_dist_sq) &&
                    (a3_task05_cached_impulses_are_usable(cc->accumulated_normal_impulse,
                                                          cc->accumulated_tangent_impulse))) {
                    cp->accumulated_normal_impulse = fmaxf(cc->accumulated_normal_impulse, 0.0f);
                    cp->accumulated_tangent_impulse = cc->accumulated_tangent_impulse;
                    cache_match_found = 1;
                    break;
                }
            } else if ((cache_id_a != 0) && (cache_id_b != 0) && (cc->object_id_a == cache_id_b) &&
                       (cc->object_id_b == cache_id_a) && (cc->property_stamp_a == cache_stamp_b) &&
                       (cc->property_stamp_b == cache_stamp_a)) {
                float dist_sq_ab =
                    vector3_length_squared(vector3_subtraction(cc->local_position_a, cp->local_position_b));

                float dist_sq_ba =
                    vector3_length_squared(vector3_subtraction(cc->local_position_b, cp->local_position_a));

                float dist_sq = fminf(dist_sq_ab, dist_sq_ba);

                if ((dist_sq < g_cfg.solver.warm_start_match_dist_sq) &&
                    (a3_task05_cached_impulses_are_usable(cc->accumulated_normal_impulse,
                                                          cc->accumulated_tangent_impulse))) {
                    /*
             * Swapped body order:
             *
             * Normal impulse magnitude remains positive because the current
             * manifold normal should already point from current object_a
             * toward current object_b.
             *
             * Tangent impulse direction is reversed because the relative
             * velocity order is reversed.
             */
                    cp->accumulated_normal_impulse = fmaxf(cc->accumulated_normal_impulse, 0.0f);
                    cp->accumulated_tangent_impulse = -cc->accumulated_tangent_impulse;
                    cache_match_found = 1;
                    break;
                }
            }
        }
        /* MPE_TASK_05_CACHE_MATCH_END */
        if (cache_match_found) {
            contact_cache_hit_count++;
        } else {
            contact_cache_miss_count++;
        }

        vector3 va = vector3_addition(m->object_a->velocity, vector3_cross(m->object_a->angular_velocity, cp->ra));
        vector3 vb = vector3_addition(m->object_b->velocity, vector3_cross(m->object_b->angular_velocity, cp->rb));
        vector3 rel_vel = vector3_subtraction(vb, va);
        float vn_initial = vector3_dot(rel_vel, m->normal_vector);

        float restitution = fminf(m->object_a->restitution, m->object_b->restitution);
        if (vn_initial < g_cfg.solver.restitution_velocity_thresh) { /* A3_PATCH_20_RESTITUTION_TUNING */
            cp->restitution_bias = -restitution * vn_initial;
            if (cp->restitution_bias > g_cfg.solver.max_restitution_bias) {
                cp->restitution_bias = g_cfg.solver.max_restitution_bias;
            }
        } else {
            cp->restitution_bias = 0.0f;
        }

        vector3 ra_cross_n = vector3_cross(cp->ra, m->normal_vector);
        vector3 rb_cross_n = vector3_cross(cp->rb, m->normal_vector);
        vector3 ang_a =
            vector3_cross(math3_multiplication_vector3(m->object_a->inverse_inertia_system, ra_cross_n), cp->ra);
        vector3 ang_b =
            vector3_cross(math3_multiplication_vector3(m->object_b->inverse_inertia_system, rb_cross_n), cp->rb);
        float k_normal = m->object_a->inverse_mass + m->object_b->inverse_mass +
                         vector3_dot(vector3_addition(ang_a, ang_b), m->normal_vector);
        cp->effective_mass_normal = (k_normal > 0.0f) ? (1.0f / k_normal) : 0.0f;

        vector3 rel_vel_tangent = vector3_subtraction(rel_vel, vector3_scaling(m->normal_vector, vn_initial));
        float tangent_speed = vector3_length(rel_vel_tangent);
        
        /* MFS_MECANUM_FRICTION: for mecanum wheels, use roller-based tangent direction */
        bool mecanum_tangent_set = false;
        rigidbody *mecanum_wheel = NULL;
        
        if (m->object_a && m->object_a->is_mecanum) {
            mecanum_wheel = m->object_a;
        } else if (m->object_b && m->object_b->is_mecanum) {
            mecanum_wheel = m->object_b;
        }
        
        if (mecanum_wheel && mecanum_wheel->type == object_cylinder) {
            /* Compute roller's free-slide direction in world space.
             * Roller angle is measured from the axle (local X axis).
             * For a wheel resting on the floor (y=0 plane), the roller direction
             * projected onto the floor plane determines the free-slide direction.
             * Friction grips perpendicular to that direction. */
            
            /* Get wheel's local axes in world space */
            vector3 axle_world = mecanum_wheel->cached_axes[0]; /* local X = axle */
            
            /* Compute roller direction: rotate axle by roller_angle around the wheel's local Y axis
             * (which points along the wheel's radius at the contact point).
             * For simplicity, assume the wheel is upright (axle horizontal).
             * The roller direction in the contact plane is perpendicular to the grip direction. */
            
            /* Floor normal is (0, 1, 0) or (0, -1, 0) depending on convention */
            vector3 floor_normal = m->normal_vector;
            if (vector3_length_squared(floor_normal) < 0.0001f) {
                floor_normal = (vector3){0.0f, 1.0f, 0.0f};
            }
            
            /* Project axle onto floor plane */
            vector3 axle_proj = vector3_subtraction(
                axle_world,
                vector3_scaling(floor_normal, vector3_dot(axle_world, floor_normal))
            );
            float axle_proj_len = vector3_length(axle_proj);
            if (axle_proj_len > 0.0001f) {
                axle_proj = vector3_scaling(axle_proj, 1.0f / axle_proj_len);
                
                /* Roller direction is at roller_angle from axle, in the wheel's tangent plane.
                 * For a mecanum wheel on the floor, the roller's free direction is:
                 * cos(angle) * axle_proj + sin(angle) * (floor_normal × axle_proj)
                 * The grip direction (friction tangent) is perpendicular to this. */
                float cos_a = cosf(mecanum_wheel->roller_angle_rad);
                float sin_a = sinf(mecanum_wheel->roller_angle_rad);
                vector3 perp = vector3_cross(floor_normal, axle_proj);
                vector3 roller_free = vector3_addition(
                    vector3_scaling(axle_proj, cos_a),
                    vector3_scaling(perp, sin_a)
                );
                
                /* Grip direction is perpendicular to roller_free, still in the floor plane */
                vector3 grip_dir = vector3_cross(floor_normal, roller_free);

/* MFS_DEBUG_STRAFE: diagnostic for mecanum strafe debugging */
#ifdef MFS_DEBUG_STRAFE
{
    static int strafe_diag_counter = 0;
    if ((strafe_diag_counter++ % 60) == 0) {
        float grip_len = vector3_length(grip_dir);
        printf("[STRAFE_DIAG] roller_angle=%.2f rad grip_len=%.4f mecanum=%d\n",
               mecanum_wheel->roller_angle_rad, grip_len, mecanum_wheel->is_mecanum ? 1 : 0);
    }
}
#endif
                float grip_len = vector3_length(grip_dir);
                if (grip_len > 0.0001f) {
                    cp->tangent_vector = vector3_scaling(grip_dir, 1.0f / grip_len);
                    mecanum_tangent_set = true;
                }
            }
/* MFS_127_STRAFE_DIAG: Conditional diagnostics for mecanum strafe debugging.
 * Compile with -DMFS_DEBUG_STRAFE to enable. */


        }
        
        if (mecanum_tangent_set || tangent_speed > 0.0001f) {
            if (!mecanum_tangent_set) {
                cp->tangent_vector = vector3_scaling(rel_vel_tangent, -1.0f / tangent_speed);
            }
            vector3 ra_cross_t = vector3_cross(cp->ra, cp->tangent_vector);
            vector3 rb_cross_t = vector3_cross(cp->rb, cp->tangent_vector);
            vector3 ang_a_t =
                vector3_cross(math3_multiplication_vector3(m->object_a->inverse_inertia_system, ra_cross_t), cp->ra);
            vector3 ang_b_t =
                vector3_cross(math3_multiplication_vector3(m->object_b->inverse_inertia_system, rb_cross_t), cp->rb);
            float k_tangent = m->object_a->inverse_mass + m->object_b->inverse_mass +
                              vector3_dot(vector3_addition(ang_a_t, ang_b_t), cp->tangent_vector);
            cp->effective_mass_tangent = (k_tangent > 0.0f) ? (1.0f / k_tangent) : 0.0f;
        } else {
            cp->tangent_vector = vector3_zero();
            cp->effective_mass_tangent = 0.0f;
        }

        if (cp->accumulated_normal_impulse != 0.0f || cp->accumulated_tangent_impulse != 0.0f) {
            vector3 impulse = vector3_addition(vector3_scaling(m->normal_vector, cp->accumulated_normal_impulse),
                                               vector3_scaling(cp->tangent_vector, cp->accumulated_tangent_impulse));
            if (!m->object_a->static_state) {
                m->object_a->velocity =
                    vector3_subtraction(m->object_a->velocity, vector3_scaling(impulse, m->object_a->inverse_mass));
                m->object_a->angular_velocity = vector3_subtraction(
                    m->object_a->angular_velocity,
                    math3_multiplication_vector3(m->object_a->inverse_inertia_system, vector3_cross(cp->ra, impulse)));
            }
            if (!m->object_b->static_state) {
                m->object_b->velocity =
                    vector3_addition(m->object_b->velocity, vector3_scaling(impulse, m->object_b->inverse_mass));
                m->object_b->angular_velocity = vector3_addition(
                    m->object_b->angular_velocity,
                    math3_multiplication_vector3(m->object_b->inverse_inertia_system, vector3_cross(cp->rb, impulse)));
            }
        }
    }
}

void collision_resolve_iterative(collision_data *m) {
    for (int i = 0; i < m->contact_count; i++) {
        contact_point_data *cp = &m->contacts[i];

        vector3 va = vector3_addition(m->object_a->velocity, vector3_cross(m->object_a->angular_velocity, cp->ra));
        vector3 vb = vector3_addition(m->object_b->velocity, vector3_cross(m->object_b->angular_velocity, cp->rb));
        vector3 rel_vel = vector3_subtraction(vb, va);
        float vn = vector3_dot(rel_vel, m->normal_vector);

        float lambda_n = (-vn + cp->restitution_bias + cp->separation_bias) * cp->effective_mass_normal;
        float old_impulse = cp->accumulated_normal_impulse;
        cp->accumulated_normal_impulse = fmaxf(old_impulse + lambda_n, 0.0f);
        lambda_n = cp->accumulated_normal_impulse - old_impulse;
        if (lambda_n != 0.0f) {
            vector3 impulse = vector3_scaling(m->normal_vector, lambda_n);
            if (!m->object_a->static_state) {
                m->object_a->velocity =
                    vector3_subtraction(m->object_a->velocity, vector3_scaling(impulse, m->object_a->inverse_mass));
                m->object_a->angular_velocity = vector3_subtraction(
                    m->object_a->angular_velocity,
                    math3_multiplication_vector3(m->object_a->inverse_inertia_system, vector3_cross(cp->ra, impulse)));
            }
            if (!m->object_b->static_state) {
                m->object_b->velocity =
                    vector3_addition(m->object_b->velocity, vector3_scaling(impulse, m->object_b->inverse_mass));
                m->object_b->angular_velocity = vector3_addition(
                    m->object_b->angular_velocity,
                    math3_multiplication_vector3(m->object_b->inverse_inertia_system, vector3_cross(cp->rb, impulse)));
            }
        }

        va = vector3_addition(m->object_a->velocity, vector3_cross(m->object_a->angular_velocity, cp->ra));
        vb = vector3_addition(m->object_b->velocity, vector3_cross(m->object_b->angular_velocity, cp->rb));
        rel_vel = vector3_subtraction(vb, va);
        vector3 tangent = cp->tangent_vector;
        if (vector3_length_squared(tangent) < 0.0001f) {
            vector3 rel_vel_tangent =
                vector3_subtraction(rel_vel, vector3_scaling(m->normal_vector, vector3_dot(rel_vel, m->normal_vector)));
            float tangent_length = vector3_length(rel_vel_tangent);
            if (tangent_length > 0.0001f) {
                tangent = vector3_scaling(rel_vel_tangent, -1.0f / tangent_length);
                cp->tangent_vector = tangent;
            }
        }
        if (vector3_length_squared(tangent) > 0.0001f) {
            float vt = vector3_dot(rel_vel, tangent);

            vector3 ra_cross_t = vector3_cross(cp->ra, tangent);
            vector3 rb_cross_t = vector3_cross(cp->rb, tangent);
            vector3 ang_a_t =
                vector3_cross(math3_multiplication_vector3(m->object_a->inverse_inertia_system, ra_cross_t), cp->ra);
            vector3 ang_b_t =
                vector3_cross(math3_multiplication_vector3(m->object_b->inverse_inertia_system, rb_cross_t), cp->rb);
            float k_tangent = m->object_a->inverse_mass + m->object_b->inverse_mass +
                              vector3_dot(vector3_addition(ang_a_t, ang_b_t), tangent);
            float eff_mass_t = (k_tangent > 0.0f) ? (1.0f / k_tangent) : 0.0f;

            float lambda_t = -vt * eff_mass_t;
            float tangential_speed = fabsf(vt);
            const float static_friction_threshold = g_cfg.solver.static_friction_thresh; /* MPE_TASK_30 */
            float static_friction_coeff = fminf(m->object_a->friction_static, m->object_b->friction_static);
            float kinetic_friction_coeff = fminf(m->object_a->friction_kinetic, m->object_b->friction_kinetic);
            if (static_friction_coeff < kinetic_friction_coeff) {
                static_friction_coeff = kinetic_friction_coeff;
            }
            float friction_coeff =
                (tangential_speed < static_friction_threshold) ? static_friction_coeff : kinetic_friction_coeff;
            float max_friction = cp->accumulated_normal_impulse * friction_coeff;
            float old_tangent_impulse = cp->accumulated_tangent_impulse;
            cp->accumulated_tangent_impulse = fmaxf(-max_friction, fminf(old_tangent_impulse + lambda_t, max_friction));
            lambda_t = cp->accumulated_tangent_impulse - old_tangent_impulse;
            if (lambda_t != 0.0f) {
                vector3 friction_impulse = vector3_scaling(tangent, lambda_t);
                if (!m->object_a->static_state) {
                    m->object_a->velocity = vector3_subtraction(
                        m->object_a->velocity, vector3_scaling(friction_impulse, m->object_a->inverse_mass));
                    m->object_a->angular_velocity =
                        vector3_subtraction(m->object_a->angular_velocity,
                                            math3_multiplication_vector3(m->object_a->inverse_inertia_system,
                                                                         vector3_cross(cp->ra, friction_impulse)));
                }
                if (!m->object_b->static_state) {
                    m->object_b->velocity = vector3_addition(
                        m->object_b->velocity, vector3_scaling(friction_impulse, m->object_b->inverse_mass));
                    m->object_b->angular_velocity =
                        vector3_addition(m->object_b->angular_velocity,
                                         math3_multiplication_vector3(m->object_b->inverse_inertia_system,
                                                                      vector3_cross(cp->rb, friction_impulse)));
                }
            }
        }
    }
}

void contact_cache_save(struct physics_world *world, collision_data *manifolds, int count) {
    /* MFS_131A: per-world warm-start cache.
     * world == NULL (legacy GUI path) falls back to the global cache. */
    int *cache_count;
    cached_contact *cache_array;
    if ((world) && (world->world_contact_cache)) {
        cache_count = &world->world_contact_cache_count;
        cache_array = world->world_contact_cache;
    } else {
        cache_count = &contact_impulse_cache_count;
        cache_array = contact_impulse_cache;
    }
    *cache_count = 0;
    for (int m = 0; m < count; m++) {
        collision_data *manifold = &manifolds[m];
        for (int i = 0; i < manifold->contact_count; i++) {
            if (*cache_count >= max_cached_contacts) {
                return;
            }
            contact_point_data *cp = &manifold->contacts[i];
            cached_contact *cc = &cache_array[(*cache_count)++];
            cc->object_id_a = (manifold->object_a) ? manifold->object_a->object_id : 0;
            cc->object_id_b = (manifold->object_b) ? manifold->object_b->object_id : 0;
            /* MPE_TASK_05_CACHE_SAVE_STAMP_BEGIN */
            cc->property_stamp_a = a3_task05_body_property_stamp(manifold->object_a);
            cc->property_stamp_b = a3_task05_body_property_stamp(manifold->object_b);
            /* MPE_TASK_05_CACHE_SAVE_STAMP_END */
            cc->local_position_a = cp->local_position_a;
            cc->local_position_b = cp->local_position_b;
            cc->accumulated_normal_impulse = cp->accumulated_normal_impulse;
            cc->accumulated_tangent_impulse = cp->accumulated_tangent_impulse;
        }
    }
}

void contact_cache_clear(struct physics_world *world) {
    /* MFS_131A: NULL world = legacy global cache. */
    if ((world) && (world->world_contact_cache)) {
        world->world_contact_cache_count = 0;
    } else {
        contact_impulse_cache_count = 0;
    }
}

/* MPE_FTC_093: Cylinder vs static floor plane.
 * Models the cylinder as axle segment + radius. Each axle endpoint acts
 * like a sphere of radius r; an endpoint below the plane yields a contact.
 * Two contacts (one per axle end) give a stable resting wheel.
 * Normal matches the sphere-floor convention: (0,-1,0). */
bool collision_static_plane_cylinder(rigidbody *cyl, float plane_y, collision_data *collision_output_data) {
    if (cyl->type != object_cylinder) {return false;}
    vector3 axis = cyl->cached_axes[0]; /* axle = local X in world space */
    float r = cyl->radius;
    float h = cyl->cylinder_half_length;
    vector3 axle_offset = vector3_scaling(axis, h);
    vector3 e1 = vector3_subtraction(cyl->position, axle_offset);
    vector3 e2 = vector3_addition(cyl->position, axle_offset);
    rigidbody *plane_body = collision_static_plane_body_proxy(plane_y);
    collision_output_data->object_a = cyl;
    collision_output_data->object_b = plane_body;
    collision_output_data->normal_vector = (vector3){0.0f, -1.0f, 0.0f};
    collision_output_data->contact_count = 0;
    float pen1 = plane_y - (e1.y - r);
    if ((pen1 > 0.0f) && (collision_output_data->contact_count < 2)) {
        contact_point_data *cp = &collision_output_data->contacts[collision_output_data->contact_count];
        cp->position = (vector3){e1.x, e1.y - r, e1.z};
        cp->penetration = pen1;
        collision_output_data->contact_count++;
    }
    float pen2 = plane_y - (e2.y - r);
    if ((pen2 > 0.0f) && (collision_output_data->contact_count < 2)) {
        contact_point_data *cp = &collision_output_data->contacts[collision_output_data->contact_count];
        cp->position = (vector3){e2.x, e2.y - r, e2.z};
        cp->penetration = pen2;
        collision_output_data->contact_count++;
    }
    return collision_output_data->contact_count > 0;
}

/* ================================================================
 * MFS_172: Cylinder-vs-object narrowphase
 * ================================================================ */

/* Cylinder vs Sphere.
 * The cylinder is modelled as its axle segment [E1,E2] with radius r_c.
 * Find the closest point on the segment to the sphere centre, then
 * do a sphere-sphere test at that point. */
bool collision_cylinder_sphere(rigidbody *cyl, rigidbody *sph,
                               collision_data *out) {
    if ((cyl->type != object_cylinder) || (sph->type != object_sphere)) {
        return false;
    }
    vector3 axis = cyl->cached_axes[0];
    float r_c = cyl->radius;
    float h   = cyl->cylinder_half_length;
    float r_s = sph->radius;

    vector3 e1 = vector3_subtraction(cyl->position, vector3_scaling(axis, h));
    vector3 e2 = vector3_addition(cyl->position, vector3_scaling(axis, h));

    /* closest point on segment [e1,e2] to sphere centre */
    vector3 seg = vector3_subtraction(e2, e1);
    float seg_len_sq = vector3_length_squared(seg);
    float t = 0.0f;
    if (seg_len_sq > 0.000001f) {
        t = vector3_dot(vector3_subtraction(sph->position, e1), seg) / seg_len_sq;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
    }
    vector3 closest = vector3_addition(e1, vector3_scaling(seg, t));

    float dist = vector3_length(vector3_subtraction(sph->position, closest));
    float min_dist = r_c + r_s;
    if (dist >= min_dist) return false;

    out->object_a = cyl;
    out->object_b = sph;
    out->contact_count = 1;

    if (dist > 0.0001f) {
        out->normal_vector = vector3_scaling(
            vector3_subtraction(sph->position, closest), 1.0f / dist);
    } else {
        out->normal_vector = (vector3){0.0f, 1.0f, 0.0f};
    }
    contact_point_data *cp = &out->contacts[0];
    cp->penetration = min_dist - dist;
    cp->position = vector3_addition(closest,
        vector3_scaling(out->normal_vector, r_c));
    return true;
}

/* Cylinder vs Cube (OBB).
 * Sample N points along the axle, find the one closest to the OBB
 * surface, then do a sphere-OBB test at that point with the
 * cylinder radius. 5 samples is enough for short axles (wheels). */
bool collision_cylinder_cube(rigidbody *cyl, rigidbody *cube,
                             collision_data *out) {
    if ((cyl->type != object_cylinder) || (cube->type != object_cube)) {
        return false;
    }
    vector3 axis = cyl->cached_axes[0];
    float r = cyl->radius;
    float h = cyl->cylinder_half_length;

    vector3 e1 = vector3_subtraction(cyl->position, vector3_scaling(axis, h));
    vector3 e2 = vector3_addition(cyl->position, vector3_scaling(axis, h));

    const int SAMPLES = 5;
    float best_dist = 1e30f;
    vector3 best_on_obb = cube->position;

    for (int s = 0; s <= SAMPLES; s++) {
        float t = (float)s / (float)SAMPLES;
        vector3 pt = vector3_addition(e1,
            vector3_scaling(vector3_subtraction(e2, e1), t));

        /* project into OBB local space */
        vector3 rel = vector3_subtraction(pt, cube->position);
        vector3 *axes = cube->cached_axes;
        vector3 local = {
            vector3_dot(rel, axes[0]),
            vector3_dot(rel, axes[1]),
            vector3_dot(rel, axes[2])
        };
        vector3 clamped = {
            fmaxf(-cube->half_extensions.x, fminf(cube->half_extensions.x, local.x)),
            fmaxf(-cube->half_extensions.y, fminf(cube->half_extensions.y, local.y)),
            fmaxf(-cube->half_extensions.z, fminf(cube->half_extensions.z, local.z))
        };
        vector3 on_obb = cube->position;
        on_obb = vector3_addition(on_obb, vector3_scaling(axes[0], clamped.x));
        on_obb = vector3_addition(on_obb, vector3_scaling(axes[1], clamped.y));
        on_obb = vector3_addition(on_obb, vector3_scaling(axes[2], clamped.z));

        float d = vector3_length(vector3_subtraction(pt, on_obb));
        if (d < best_dist) {
            best_dist = d;
            best_on_obb = on_obb;
        }
    }

    if (best_dist >= r) return false;

    out->object_a = cyl;
    out->object_b = cube;
    out->contact_count = 1;

    if (best_dist > 0.0001f) {
        out->normal_vector = vector3_scaling(
            vector3_subtraction(cyl->position, best_on_obb),
            1.0f / vector3_length(vector3_subtraction(cyl->position, best_on_obb)));
    } else {
        out->normal_vector = (vector3){0.0f, 1.0f, 0.0f};
    }
    contact_point_data *cp = &out->contacts[0];
    cp->penetration = r - best_dist;
    cp->position = best_on_obb;
    return true;
}

/* Cylinder vs Cylinder.
 * Segment-segment closest points, then sphere-sphere at those
 * points with respective radii. */
bool collision_cylinder_cylinder(rigidbody *cyl_a, rigidbody *cyl_b,
                                 collision_data *out) {
    if ((cyl_a->type != object_cylinder) || (cyl_b->type != object_cylinder)) {
        return false;
    }
    vector3 ax = cyl_a->cached_axes[0];
    vector3 bx = cyl_b->cached_axes[0];
    float ha = cyl_a->cylinder_half_length;
    float hb = cyl_b->cylinder_half_length;

    vector3 a1 = vector3_subtraction(cyl_a->position, vector3_scaling(ax, ha));
    vector3 a2 = vector3_addition(cyl_a->position, vector3_scaling(ax, ha));
    vector3 b1 = vector3_subtraction(cyl_b->position, vector3_scaling(bx, hb));
    vector3 b2 = vector3_addition(cyl_b->position, vector3_scaling(bx, hb));

    /* segment-segment closest points (Ericson, Real-Time Collision Detection) */
    vector3 d1 = vector3_subtraction(a2, a1);
    vector3 d2 = vector3_subtraction(b2, b1);
    vector3 r  = vector3_subtraction(a1, b1);
    float a = vector3_dot(d1, d1);
    float e = vector3_dot(d2, d2);
    float f = vector3_dot(d2, r);
    float s, t;

    if ((a <= 0.000001f) && (e <= 0.000001f)) {
        s = t = 0.0f;
    } else if (a <= 0.000001f) {
        s = 0.0f;
        t = f / e;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
    } else {
        float c = vector3_dot(d1, r);
        if (e <= 0.000001f) {
            t = 0.0f;
            s = -c / a;
            if (s < 0.0f) s = 0.0f;
            if (s > 1.0f) s = 1.0f;
        } else {
            float b = vector3_dot(d1, d2);
            float denom = a * e - b * b;
            s = (denom > 0.000001f) ? (b * f - c * e) / denom : 0.0f;
            if (s < 0.0f) s = 0.0f;
            if (s > 1.0f) s = 1.0f;
            t = (b * s + f) / e;
            if (t < 0.0f) { t = 0.0f; s = -c / a; }
            if (t > 1.0f) { t = 1.0f; s = (b - c) / a; }
            if (s < 0.0f) s = 0.0f;
            if (s > 1.0f) s = 1.0f;
        }
    }

    vector3 pa = vector3_addition(a1, vector3_scaling(d1, s));
    vector3 pb = vector3_addition(b1, vector3_scaling(d2, t));
    float dist = vector3_length(vector3_subtraction(pa, pb));
    float min_dist = cyl_a->radius + cyl_b->radius;
    if (dist >= min_dist) return false;

    out->object_a = cyl_a;
    out->object_b = cyl_b;
    out->contact_count = 1;
    if (dist > 0.0001f) {
        out->normal_vector = vector3_scaling(
            vector3_subtraction(pb, pa), 1.0f / dist);
    } else {
        out->normal_vector = (vector3){0.0f, 1.0f, 0.0f};
    }
    contact_point_data *cp = &out->contacts[0];
    cp->penetration = min_dist - dist;
    cp->position = vector3_scaling(vector3_addition(pa, pb), 0.5f);
    return true;
}

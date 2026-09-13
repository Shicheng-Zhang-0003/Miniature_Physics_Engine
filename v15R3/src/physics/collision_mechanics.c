#include "../mpe_engine.h"
#include "collision_mechanics.h"
#include "../core/physics_world.h" /* MFS_131 */
#include <stdint.h>
#include <stdlib.h>


/* Warm-start hash: 4096 buckets over canonical (min_id, max_id) pairs.
 * Chains live in cached_contact.hash_next and are rebuilt on every save
 * in array order (reverse-prepend), so a lookup walk visits candidates in
 * exactly the array order the old legacy linear scan used: identical
 * first-hit, O(chain) instead of O(cache). Heads live in
 * physics_world.contact_hash_head (heap, per world). The old file-scope
 * global cache is retired; a NULL cache degrades to all-miss. */
#define CONTACT_HASH_BITS 12
#define CONTACT_HASH_SIZE (1 << CONTACT_HASH_BITS)
#define CONTACT_HASH_MASK (CONTACT_HASH_SIZE - 1)

static inline uint32_t contact_pair_key(uint32_t id_a, uint32_t id_b) {
    uint32_t lo = (id_a < id_b) ? id_a : id_b;
    uint32_t hi = (id_a < id_b) ? id_b : id_a;
    uint64_t key = ((uint64_t) lo << 32) | (uint64_t) hi;
    /* splitmix64 finalizer over the combined key. */
    key ^= key >> 30;
    key *= 0xbf58476d1ce4e5b9ULL;
    key ^= key >> 27;
    key *= 0x94d049bb133111ebULL;
    key ^= key >> 31;
    return (uint32_t) (key & CONTACT_HASH_MASK);
}

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
        /* FIX-AUDIT: normal convention is A->B (sphere->cube). Sphere near
         * +face: cube.pos-sphere.pos points -axis (toward cube center).
         * Old code returned +axis (outward), pushing the sphere deeper via
         * vA -= lam*n. Negate to satisfy dot(cube.pos-sphere.pos,n)>0. */
        vector3 outward = vector3_scaling(axes_cube[nearest_face_axis], nearest_face_sign);
        collision_output_data->normal_vector = vector3_scaling(outward, -1.0f);
        cp->penetration = sphere->radius + minimum_distance;
        /* Contact on the cube FACE (not the sphere center): the lever arms
         * ra/rb must span contact-to-center for correct torque. */
        float face_local[3] = {vector3_dot(relative_position, axes_cube[0]),
                               vector3_dot(relative_position, axes_cube[1]),
                               vector3_dot(relative_position, axes_cube[2])};
        float face_extent = (nearest_face_axis == 0)   ? cube->half_extensions.x
                            : (nearest_face_axis == 1) ? cube->half_extensions.y
                                                       : cube->half_extensions.z;
        face_local[nearest_face_axis] = nearest_face_sign * face_extent;
        cp->position = vector3_addition(
            cube->position,
            vector3_addition(vector3_scaling(axes_cube[0], face_local[0]),
                             vector3_addition(vector3_scaling(axes_cube[1], face_local[1]),
                                              vector3_scaling(axes_cube[2], face_local[2]))));
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
    (void) overlap; /* slop-gated admission below; no phantom fallback uses this */
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
        if (input_count < 1) {
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
    /* Slop-gated persistent contacts (Box2D linearSlop practice): points
     * within penetration slop are admitted as zero-depth contacts. They
     * generate friction (persistent support) but no separation impulse
     * (bias subtracts slop) and no restitution (velocity-gated), so they
     * cannot bounce at a distance. Strict >0 flickers rolling/wheel
     * support and starves friction. */
    float clip_slop = g_cfg.solver.penetration_slop;
    for (int i = 0; i < input_count; i++) {
        vector3 v = input_polygon[i];
        float penetration = ref_height - vector3_dot(v, ref_normal);
        if (penetration >= -clip_slop) {
            if (manifold_idx < 4) {
                contact_point_data *cp = &collision_output_data->contacts[manifold_idx++];
                cp->position = v;
                cp->penetration = (penetration > 0.0f) ? penetration : 0.0f;
            }
        }
    }
    /* FIX-AUDIT: old code fabricated a center contact with full overlap when
     * clipping was empty (SAT/clip disagreement). That injects phantom
     * impulse at wrong lever arms. Report no contact instead. */
    /* AUDIT NOTE (reverted experiment): reducing face manifolds 4->3
     * (deepest + max-area triangle) made tall stacks strictly WORSE at low
     * iteration counts (8-high regressed HOLDS->SHEARS). The 4th corner
     * carries load the solver needs; thin-quad degeneracy also risked
     * near-collinear support. Keep all clipped points. */
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
    /* AUDIT: face and edge minima tracked separately (see below). */
    float face_minimum_overlap = 1000000.0f;
    vector3 face_best_axis = {0, 0, 0};
    int face_best_axis_index = -1;
    float edge_minimum_overlap = 1000000.0f;
    vector3 edge_best_axis = {0, 0, 0};
    int edge_best_axis_index = -1;

    for (int axis_index = 0; axis_index < 6; axis_index++) {
        vector3 axis = (axis_index < 3) ? axes_a[axis_index] : axes_b[axis_index - 3];
        float projection_a = project_obb(cube_a, axis, axes_a);
        float projection_b = project_obb(cube_b, axis, axes_b);
        float distance = fabsf(vector3_dot(relative_position, axis));
        float overlap = projection_a + projection_b - distance;
        if (overlap < 0.0f) {
            return false;
        }
        if (overlap < face_minimum_overlap) {
            face_minimum_overlap = overlap;
            face_best_axis = axis;
            face_best_axis_index = axis_index;
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
            if (overlap < edge_minimum_overlap) {
                edge_minimum_overlap = overlap;
                edge_best_axis = axis;
                edge_best_axis_index = 6 + axis_index_a * 3 + axis_index_b;
            }
        }
    }
    /* Face-axis hysteresis: the raw 15-axis minimum is knife-edge
     * discontinuous — infinitesimal tilt flips face↔edge, collapsing a
     * 4-point face manifold into a single corner contact whose torque kicks
     * the tilt further (measured: flip at t=60, full topple by t=300 at 16
     * iterations). The face axis is always a valid separating axis, so take
     * the edge axis only when it wins decisively (>=10% shallower).
     * Genuine edge contacts (corner landings, crossed boxes) clear this bar
     * by a wide margin; resting-face jitter never does. */
    float minimum_overlap = 0.0f;
    vector3 best_axis = {0, 0, 0};
    int best_axis_index = -1;
    if ((edge_best_axis_index >= 0) && (edge_minimum_overlap < face_minimum_overlap * 0.9f)) {
        minimum_overlap = edge_minimum_overlap;
        best_axis = edge_best_axis;
        best_axis_index = edge_best_axis_index;
    } else {
        minimum_overlap = face_minimum_overlap;
        best_axis = face_best_axis;
        best_axis_index = face_best_axis_index;
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

    /* Slop-gated like cube/cylinder paths: resting contact persists as
     * zero-depth (friction without bounce) instead of flickering. */
    if (penetration <= -g_cfg.solver.penetration_slop) {
        return false;
    }

    rigidbody *plane_body = collision_static_plane_body_proxy(plane_y);

    collision_output_data->object_a = sphere;
    collision_output_data->object_b = plane_body;
    collision_output_data->normal_vector = (vector3){0.0f, -1.0f, 0.0f};
    collision_output_data->contact_count = 1;

    contact_point_data *cp = &collision_output_data->contacts[0];
    cp->position = (vector3){sphere->position.x, lowest_y, sphere->position.z};
    cp->penetration = (penetration > 0.0f) ? penetration : 0.0f;

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

                /* Slop-gated (see clip_obb_faces): admit within slop as
                 * zero-depth for persistent friction, no bounce. */
                if (penetration > -g_cfg.solver.penetration_slop) {
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

void contact_cache_stats_reset(struct physics_world *world) {
    if (!world) {
        return;
    }
    world->contact_cache_hits = 0;
    world->contact_cache_misses = 0;
}

int contact_cache_get_hits(const struct physics_world *world) {
    if (!world) {
        return 0;
    }
    return world->contact_cache_hits;
}

int contact_cache_get_misses(const struct physics_world *world) {
    if (!world) {
        return 0;
    }
    return world->contact_cache_misses;
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

/* Warm-start match role of one cache entry. Bit-exact transcription of the
 * legacy linear-scan predicates (same-order side-A matching with the
 * documented loose side-B tradeoff; strict both-side swapped matching):
 * 0 = no match, 1 = same-order hit, 2 = swapped-order hit. Shared by the
 * hash-chain walk and the legacy linear fallback so both agree. */
static int contact_cache_match_role(const cached_contact *cc, uint32_t id_a, uint32_t id_b, uint32_t stamp_a,
                                    uint32_t stamp_b, vector3 local_a, vector3 local_b) {
    if ((!cc) || (id_a == 0) || (id_b == 0)) {
        return 0;
    }
    float match_dist_sq = g_cfg.solver.warm_start_match_dist_sq;
    if ((cc->object_id_a == id_a) && (cc->object_id_b == id_b) && (cc->property_stamp_a == stamp_a) &&
        (cc->property_stamp_b == stamp_b)) {
        float dist_a_sq = vector3_length_squared(vector3_subtraction(cc->local_position_a, local_a));
        if ((dist_a_sq < match_dist_sq) && (a3_task05_cached_impulses_are_usable(cc->accumulated_normal_impulse,
                                                                                cc->accumulated_tangent_impulse))) {
            return 1;
        }
        return 0;
    }
    if ((cc->object_id_a == id_b) && (cc->object_id_b == id_a) && (cc->property_stamp_a == stamp_b) &&
        (cc->property_stamp_b == stamp_a)) {
        float dist_sq_ab = vector3_length_squared(vector3_subtraction(cc->local_position_a, local_b));
        float dist_sq_ba = vector3_length_squared(vector3_subtraction(cc->local_position_b, local_a));
        if ((dist_sq_ab < match_dist_sq) && (dist_sq_ba < match_dist_sq) &&
            (a3_task05_cached_impulses_are_usable(cc->accumulated_normal_impulse, cc->accumulated_tangent_impulse))) {
            return 2;
        }
    }
    return 0;
}

/* Tangent-adopt predicate (legacy transcription): same-order entry carrying
 * a usable stick frame. */
static bool contact_cache_adoptable(const cached_contact *cc, uint32_t id_a, uint32_t id_b, uint32_t stamp_a,
                                    uint32_t stamp_b, vector3 local_a) {
    if ((!cc) || (id_a == 0) || (id_b == 0)) {
        return false;
    }
    if (!((cc->object_id_a == id_a) && (cc->object_id_b == id_b) && (cc->property_stamp_a == stamp_a) &&
          (cc->property_stamp_b == stamp_b))) {
        return false;
    }
    float dist_a_sq = vector3_length_squared(vector3_subtraction(cc->local_position_a, local_a));
    if (dist_a_sq >= g_cfg.solver.warm_start_match_dist_sq) {
        return false;
    }
    return vector3_length_squared(cc->tangent_dir) > 0.0001f;
}

void collision_prepare_solver(struct physics_world *world, collision_data *source, collision_data *m, float dt) {
    *m = *source;
    if (dt <= 0.0f) {
        dt = 1.0f / 60.0f;
    }
    /* Per-world cache; a missing cache degrades to all-miss (cold solve).
     * No global fallback remains. */
    cached_contact *cache_array = (world) ? world->world_contact_cache : NULL;
    int cache_count = (world) ? world->world_contact_cache_count : 0;
    int32_t *hash_head = (world) ? world->contact_hash_head : NULL;
    if (!cache_array) {
        cache_count = 0;
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
        /* AUDIT: no velocity-level Baumgarte bias is computed here on
         * purpose (see header). g_cfg.solver.bias_factor drives the
         * positional split-impulse correction instead, where bias velocity
         * cannot leak into impulses. An earlier revision computed a
         * per-contact separation_bias that nothing read. */

        /* MPE_TASK_05_CACHE_MATCH_BEGIN */
        uint32_t cache_id_a = (m->object_a) ? m->object_a->object_id : 0;
        uint32_t cache_id_b = (m->object_b) ? m->object_b->object_id : 0;

        uint32_t cache_stamp_a = a3_task05_body_property_stamp(m->object_a);
        uint32_t cache_stamp_b = a3_task05_body_property_stamp(m->object_b);

        int cache_match_found = 0;

        /* Warm-start matching (baseline side-A-only): side-B and property
         * stamps are intentionally loose here. Bisect showed strict
         * both-side matching + stamps destabilize resting wheel contacts
         * (fewer hits = colder solver = idle spin-up and bounce). Aliasing
         * on shared A-anchors is a known accepted tradeoff, documented for
         * future 2-tangent + consistent-rotation warm-start work. */
        if ((hash_head) && (cache_id_a != 0) && (cache_id_b != 0)) {
            /* Hash walk: visits the pair's entries in save order, i.e. the
             * same first-hit the legacy linear scan below would find. */
            uint32_t slot0 = contact_pair_key(cache_id_a, cache_id_b);
            for (int32_t slot = hash_head[slot0], guard = 0;
                 (slot >= 0) && (slot < cache_count) && (guard <= cache_count);
                 slot = cache_array[slot].hash_next, guard++) {
                cached_contact *cc = &cache_array[slot];
                int role = contact_cache_match_role(cc, cache_id_a, cache_id_b, cache_stamp_a, cache_stamp_b,
                                                    cp->local_position_a, cp->local_position_b);
                if (role == 1) {
                    cp->accumulated_normal_impulse = fmaxf(cc->accumulated_normal_impulse, 0.0f);
                    cp->accumulated_tangent_impulse = cc->accumulated_tangent_impulse;
                    cp->warmed = true;
                    cache_match_found = 1;
                    break;
                } else if (role == 2) {
                    /* Swapped body order: normal stays positive (manifold
                     * normal already points A->B); tangent reverses with
                     * the relative-velocity order. */
                    cp->accumulated_normal_impulse = fmaxf(cc->accumulated_normal_impulse, 0.0f);
                    cp->accumulated_tangent_impulse = -cc->accumulated_tangent_impulse;
                    cp->warmed = true;
                    cache_match_found = 1;
                    break;
                }
            }
        } else {
            /* Legacy linear fallback (no hash heads, e.g. malloc failure). */
            for (int c = 0; c < cache_count; c++) {
                cached_contact *cc = &cache_array[c];
                int role = contact_cache_match_role(cc, cache_id_a, cache_id_b, cache_stamp_a, cache_stamp_b,
                                                    cp->local_position_a, cp->local_position_b);
                if (role == 1) {
                    cp->accumulated_normal_impulse = fmaxf(cc->accumulated_normal_impulse, 0.0f);
                    cp->accumulated_tangent_impulse = cc->accumulated_tangent_impulse;
                    cp->warmed = true;
                    cache_match_found = 1;
                    break;
                } else if (role == 2) {
                    cp->accumulated_normal_impulse = fmaxf(cc->accumulated_normal_impulse, 0.0f);
                    cp->accumulated_tangent_impulse = -cc->accumulated_tangent_impulse;
                    cp->warmed = true;
                    cache_match_found = 1;
                    break;
                }
            }
        }
        /* MPE_TASK_05_CACHE_MATCH_END */
        if (world) {
            if (cache_match_found) {
                world->contact_cache_hits++;
            } else {
                world->contact_cache_misses++;
            }
        }

        vector3 va = vector3_addition(m->object_a->velocity, vector3_cross(m->object_a->angular_velocity, cp->ra));
        vector3 vb = vector3_addition(m->object_b->velocity, vector3_cross(m->object_b->angular_velocity, cp->rb));
        vector3 rel_vel = vector3_subtraction(vb, va);
        float vn_initial = vector3_dot(rel_vel, m->normal_vector);

        /* Poisson gate input: pre-solve approach speed of this tick. */
        cp->impact_velocity = vn_initial;

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

        /* Coulomb tangent frame. At rest there is no slip direction, so a
         * resting contact adopts the remembered tangent from warm start:
         * without direction memory static friction cannot stick (every tick
         * would start with a zero tangent and zero hold). */
        vector3 adopted_tangent = vector3_zero();
        if (tangent_speed <= 0.0001f) {
            /* Same first-hit as the legacy full-array scan (see hash note
             * above): the bucket holds exactly the matchable entries in
             * save order. */
            if ((hash_head) && (cache_id_a != 0) && (cache_id_b != 0)) {
                uint32_t slot0 = contact_pair_key(cache_id_a, cache_id_b);
                for (int32_t slot = hash_head[slot0], guard = 0;
                     (slot >= 0) && (slot < cache_count) && (guard <= cache_count);
                     slot = cache_array[slot].hash_next, guard++) {
                    cached_contact *cc = &cache_array[slot];
                    if (contact_cache_adoptable(cc, cache_id_a, cache_id_b, cache_stamp_a, cache_stamp_b,
                                                cp->local_position_a)) {
                        adopted_tangent = cc->tangent_dir;
                        break;
                    }
                }
            } else {
                for (int c = 0; c < cache_count; c++) {
                    cached_contact *cc = &cache_array[c];
                    if (contact_cache_adoptable(cc, cache_id_a, cache_id_b, cache_stamp_a, cache_stamp_b,
                                                cp->local_position_a)) {
                        adopted_tangent = cc->tangent_dir;
                        break;
                    }
                }
            }
            if (vector3_length_squared(adopted_tangent) > 0.0001f) {
                /* Re-orthogonalize against the current normal. */
                adopted_tangent = vector3_subtraction(
                    adopted_tangent,
                    vector3_scaling(m->normal_vector, vector3_dot(adopted_tangent, m->normal_vector)));
                if (vector3_length_squared(adopted_tangent) > 0.0001f) {
                    adopted_tangent = vector3_normalisation(adopted_tangent);
                } else {
                    adopted_tangent = vector3_zero();
                }
            }
        }

        if ((tangent_speed > 0.0001f) || (vector3_length_squared(adopted_tangent) > 0.0001f)) {
            /* Standard Coulomb friction tangent from relative slip velocity. */
            if (tangent_speed > 0.0001f) {
                cp->tangent_vector = vector3_scaling(rel_vel_tangent, -1.0f / tangent_speed);
            } else {
                cp->tangent_vector = adopted_tangent;
            }
            /* Second tangent completes the Coulomb disc: t2 = n x t1. */
            cp->tangent2 = vector3_cross(m->normal_vector, cp->tangent_vector);
            if (vector3_length_squared(cp->tangent2) > 0.0001f) {
                cp->tangent2 = vector3_normalisation(cp->tangent2);
            } else {
                cp->tangent2 = vector3_zero();
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
            if (vector3_length_squared(cp->tangent2) > 0.0001f) {
                vector3 ra_cross_t2 = vector3_cross(cp->ra, cp->tangent2);
                vector3 rb_cross_t2 = vector3_cross(cp->rb, cp->tangent2);
                vector3 ang_a_t2 = vector3_cross(
                    math3_multiplication_vector3(m->object_a->inverse_inertia_system, ra_cross_t2), cp->ra);
                vector3 ang_b_t2 = vector3_cross(
                    math3_multiplication_vector3(m->object_b->inverse_inertia_system, rb_cross_t2), cp->rb);
                float k_tangent2 = m->object_a->inverse_mass + m->object_b->inverse_mass +
                                   vector3_dot(vector3_addition(ang_a_t2, ang_b_t2), cp->tangent2);
                cp->effective_mass_tangent2 = (k_tangent2 > 0.0f) ? (1.0f / k_tangent2) : 0.0f;
            } else {
                cp->effective_mass_tangent2 = 0.0f;
            }
        } else {
            /* No slip and no remembered direction: nothing to hold against. */
            cp->tangent_vector = vector3_zero();
            cp->effective_mass_tangent = 0.0f;
            cp->tangent2 = vector3_zero();
            cp->effective_mass_tangent2 = 0.0f;
            cp->accumulated_tangent2_impulse = 0.0f;
        }

        if (cp->accumulated_normal_impulse != 0.0f || cp->accumulated_tangent_impulse != 0.0f ||
            cp->accumulated_tangent2_impulse != 0.0f) {
            vector3 impulse = vector3_addition(
                vector3_scaling(m->normal_vector, cp->accumulated_normal_impulse),
                vector3_addition(vector3_scaling(cp->tangent_vector, cp->accumulated_tangent_impulse),
                                 vector3_scaling(cp->tangent2, cp->accumulated_tangent2_impulse)));
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
        /* Poisson base: compression impulse entering the iterations (warm
         * start included). The restitution pass pays e over the delta. */
        cp->base_normal_impulse = cp->accumulated_normal_impulse;
    }
}

/* Sort context for the comparator below: single-flight transient (set
 * immediately before qsort, never carried across calls). */
static const float *manifold_sort_keys_active = NULL;

static int manifold_sort_compare(const void *pa, const void *pb) {
    int ia = *(const int *) pa;
    int ib = *(const int *) pb;
    float ka = manifold_sort_keys_active[ia];
    float kb = manifold_sort_keys_active[ib];
    if (ka < kb) {
        return -1;
    }
    if (ka > kb) {
        return 1;
    }
    /* Total order by original index: deterministic regardless of qsort
     * internals, identical for twin runs. */
    if (ia < ib) {
        return -1;
    }
    if (ia > ib) {
        return 1;
    }
    return 0;
}

void collision_manifold_solve_order(struct physics_world *world, collision_data *manifolds, int manifold_count,
                                      int *order_out) {
    if ((!world) || (!world->manifold_sort_keys) || (!manifolds) || (!order_out) || (manifold_count <= 0)) {
        return;
    }
    if (manifold_count > a3_max_manifolds) {
        manifold_count = a3_max_manifolds;
    }
    for (int m = 0; m < manifold_count; m++) {
        float lowest = 1000000.0f;
        for (int i = 0; i < manifolds[m].contact_count; i++) {
            float y = manifolds[m].contacts[i].position.y;
            if (y < lowest) {
                lowest = y;
            }
        }
        world->manifold_sort_keys[m] = lowest;
        order_out[m] = m;
    }
    manifold_sort_keys_active = world->manifold_sort_keys;
    qsort(order_out, (size_t) manifold_count, sizeof(int), manifold_sort_compare);
    manifold_sort_keys_active = NULL;
}

float collision_resolve_iterative(collision_data *m, float dt, bool friction_only, int start_index) {
    if (dt <= 0.0f) {
        dt = 1.0f / 60.0f;
    }
    /* Returns the largest impulse magnitude applied this visit (normal +
     * friction deltas). Callers use it for local-convergence polishing. */
    float max_applied = 0.0f;
    /* AUDIT NOTE (reverted experiment): rotating the contact start index
     * per sweep was tried to symmetrize first-solver bias, but it broke
     * sliding kinetic friction badly (3x stopping distance: order cycling
     * interacts with the acc>=0 clamp, rectifying oscillation into net
     * drift). Fixed order + local double-visit (see callers) converges
     * without that interaction. start_index is accepted and ignored. */
    (void) start_index;
    for (int k = 0; k < m->contact_count; k++) {
        int i = k;
        contact_point_data *cp = &m->contacts[i];

        vector3 va = vector3_addition(m->object_a->velocity, vector3_cross(m->object_a->angular_velocity, cp->ra));
        vector3 vb = vector3_addition(m->object_b->velocity, vector3_cross(m->object_b->angular_velocity, cp->rb));
        vector3 rel_vel = vector3_subtraction(vb, va);
        float vn = vector3_dot(rel_vel, m->normal_vector);

        /* Pure compression: no restitution bias here (Poisson pass later). */
        if (!friction_only) {
            float lambda_n = -vn * cp->effective_mass_normal;
            float old_impulse = cp->accumulated_normal_impulse;
            cp->accumulated_normal_impulse = fmaxf(old_impulse + lambda_n, 0.0f);
            lambda_n = cp->accumulated_normal_impulse - old_impulse;
            /* Provenance-gated SOR: warm (persistent, steady-state)
             * contacts take damped steps (coupled ping-pong control);
             * cold contacts (fresh impacts, migrating slide, transients)
             * take full steps (fast kill). Fixed points unchanged (zero
             * step stays zero); twin runs agree bit-for-bit. */
            if (cp->warmed) {
                lambda_n *= 0.5f;
            }
            cp->accumulated_normal_impulse = old_impulse + lambda_n;
            if (lambda_n != 0.0f) {
                float applied_n = fabsf(lambda_n);
                if (applied_n > max_applied) {
                    max_applied = applied_n;
                }
                vector3 impulse = vector3_scaling(m->normal_vector, lambda_n);
                if (!m->object_a->static_state) {
                    m->object_a->velocity = vector3_subtraction(
                        m->object_a->velocity, vector3_scaling(impulse, m->object_a->inverse_mass));
                    m->object_a->angular_velocity = vector3_subtraction(
                        m->object_a->angular_velocity,
                        math3_multiplication_vector3(m->object_a->inverse_inertia_system,
                                                     vector3_cross(cp->ra, impulse)));
                }
                if (!m->object_b->static_state) {
                    m->object_b->velocity = vector3_addition(
                        m->object_b->velocity, vector3_scaling(impulse, m->object_b->inverse_mass));
                    m->object_b->angular_velocity = vector3_addition(
                        m->object_b->angular_velocity,
                        math3_multiplication_vector3(m->object_b->inverse_inertia_system,
                                                     vector3_cross(cp->rb, impulse)));
                }
            }
        } /* !friction_only: normal solve skipped in relaxation so the
             Poisson bounce is never subtracted back out. */

        va = vector3_addition(m->object_a->velocity, vector3_cross(m->object_a->angular_velocity, cp->ra));
        vb = vector3_addition(m->object_b->velocity, vector3_cross(m->object_b->angular_velocity, cp->rb));
        rel_vel = vector3_subtraction(vb, va);
        /* Ensure a complete orthonormal tangent frame: rebuild from current
         * slip when the stored frame is missing, refresh t2 otherwise. */
        vector3 tangent = cp->tangent_vector;
        if (vector3_length_squared(tangent) < 0.0001f) {
            vector3 rel_vel_tangent =
                vector3_subtraction(rel_vel, vector3_scaling(m->normal_vector, vector3_dot(rel_vel, m->normal_vector)));
            float tangent_length = vector3_length(rel_vel_tangent);
            if (tangent_length > 0.0001f) {
                tangent = vector3_scaling(rel_vel_tangent, -1.0f / tangent_length);
                cp->tangent_vector = tangent;
                cp->tangent2 = vector3_normalisation(vector3_cross(m->normal_vector, tangent));
            }
        } else {
            vector3 t2_check = vector3_cross(m->normal_vector, tangent);
            if (vector3_length_squared(t2_check) > 0.0001f) {
                cp->tangent2 = vector3_normalisation(t2_check);
            }
        }
        if (vector3_length_squared(tangent) > 0.0001f) {
            vector3 tangent2 = cp->tangent2;
            bool has_t2 = (vector3_length_squared(tangent2) > 0.0001f);

            float vt1 = vector3_dot(rel_vel, tangent);
            float vt2 = has_t2 ? vector3_dot(rel_vel, tangent2) : 0.0f;
            float slip_speed = sqrtf(vt1 * vt1 + vt2 * vt2);

            float eff1 = cp->effective_mass_tangent;
            float eff2 = has_t2 ? cp->effective_mass_tangent2 : 0.0f;
            if (eff1 <= 0.0f) {
                vector3 ra_c = vector3_cross(cp->ra, tangent);
                vector3 rb_c = vector3_cross(cp->rb, tangent);
                float k = m->object_a->inverse_mass + m->object_b->inverse_mass +
                          vector3_dot(vector3_addition(
                                          vector3_cross(math3_multiplication_vector3(
                                                            m->object_a->inverse_inertia_system, ra_c),
                                                        cp->ra),
                                          vector3_cross(math3_multiplication_vector3(
                                                            m->object_b->inverse_inertia_system, rb_c),
                                                        cp->rb)),
                                      tangent);
                eff1 = (k > 0.0f) ? (1.0f / k) : 0.0f;
                cp->effective_mass_tangent = eff1;
            }
            if (has_t2 && (eff2 <= 0.0f)) {
                vector3 ra_c2 = vector3_cross(cp->ra, tangent2);
                vector3 rb_c2 = vector3_cross(cp->rb, tangent2);
                float k2 = m->object_a->inverse_mass + m->object_b->inverse_mass +
                           vector3_dot(vector3_addition(
                                           vector3_cross(math3_multiplication_vector3(
                                                             m->object_a->inverse_inertia_system, ra_c2),
                                                         cp->ra),
                                           vector3_cross(math3_multiplication_vector3(
                                                             m->object_b->inverse_inertia_system, rb_c2),
                                                         cp->rb)),
                                       tangent2);
                eff2 = (k2 > 0.0f) ? (1.0f / k2) : 0.0f;
                cp->effective_mass_tangent2 = eff2;
            }

            /* Stick/slip select on combined slip speed. With a persistent
             * frame and an honest normal impulse, stick (full slip kill
             * inside the cone) genuinely holds; sliding clamps to mu_k. */
            const float static_friction_threshold = g_cfg.solver.static_friction_thresh; /* MPE_TASK_30 */
            float static_friction_coeff = fminf(m->object_a->friction_static, m->object_b->friction_static);
            float kinetic_friction_coeff = fminf(m->object_a->friction_kinetic, m->object_b->friction_kinetic);
            if (static_friction_coeff < kinetic_friction_coeff) {
                static_friction_coeff = kinetic_friction_coeff;
            }
            float friction_coeff =
                (slip_speed < static_friction_threshold) ? static_friction_coeff : kinetic_friction_coeff;
            float max_friction = cp->accumulated_normal_impulse * friction_coeff;

            /* Coulomb disc: solve both tangents, clamp the COMBINED vector. */
            float lambda_t1 = -vt1 * eff1;
            float lambda_t2 = has_t2 ? (-vt2 * eff2) : 0.0f;
            float new_acc1 = cp->accumulated_tangent_impulse + lambda_t1;
            float new_acc2 = cp->accumulated_tangent2_impulse + lambda_t2;
            float combo_sq = new_acc1 * new_acc1 + new_acc2 * new_acc2;
            if ((max_friction > 0.0f) && (combo_sq > max_friction * max_friction)) {
                float scale = max_friction / sqrtf(combo_sq);
                new_acc1 *= scale;
                new_acc2 *= scale;
            } else if (max_friction <= 0.0f) {
                new_acc1 = 0.0f;
                new_acc2 = 0.0f;
            }
            /* Provenance-gated SOR (see normal solve above). */
            float step1 = new_acc1 - cp->accumulated_tangent_impulse;
            float step2 = new_acc2 - cp->accumulated_tangent2_impulse;
            if (cp->warmed) {
                step1 *= 0.5f;
                step2 *= 0.5f;
            }
            cp->accumulated_tangent_impulse += step1;
            cp->accumulated_tangent2_impulse += step2;
            vector3 friction_delta = vector3_addition(vector3_scaling(tangent, step1),
                                                      has_t2 ? vector3_scaling(tangent2, step2) : vector3_zero());
            {
                float applied_t = sqrtf(vector3_length_squared(friction_delta));
                if (applied_t > max_applied) {
                    max_applied = applied_t;
                }
            }
            if (vector3_length_squared(friction_delta) > 0.0f) {
                if (!m->object_a->static_state) {
                    m->object_a->velocity = vector3_subtraction(
                        m->object_a->velocity, vector3_scaling(friction_delta, m->object_a->inverse_mass));
                    m->object_a->angular_velocity =
                        vector3_subtraction(m->object_a->angular_velocity,
                                            math3_multiplication_vector3(m->object_a->inverse_inertia_system,
                                                                         vector3_cross(cp->ra, friction_delta)));
                }
                if (!m->object_b->static_state) {
                    m->object_b->velocity = vector3_addition(
                        m->object_b->velocity, vector3_scaling(friction_delta, m->object_b->inverse_mass));
                    m->object_b->angular_velocity =
                        vector3_addition(m->object_b->angular_velocity,
                                         math3_multiplication_vector3(m->object_b->inverse_inertia_system,
                                                                      vector3_cross(cp->rb, friction_delta)));
                }
            }

        }
    }
    return max_applied;
}

/* Poisson restitution (Mirtich): restitution is a ratio of impulses, not a
 * velocity bias. The compression loop above accumulates the unbiased impact
 * impulse; this pass pays e times this tick's compression delta exactly
 * once per fresh impact, then the caller runs relaxation iterations so
 * friction sees the post-bounce velocities. Gated to fresh impacts by the
 * recorded pre-solve approach speed: resting contacts (and sustained
 * pushes below threshold) get exactly zero. */
void collision_apply_poisson_restitution(collision_data *manifolds, int manifold_count) {
    if ((!manifolds) || (manifold_count <= 0)) {
        return;
    }
    for (int m = 0; m < manifold_count; m++) {
        collision_data *man = &manifolds[m];
        for (int i = 0; i < man->contact_count; i++) {
            contact_point_data *cp = &man->contacts[i];
            float e = fminf(man->object_a->restitution, man->object_b->restitution);
            if (e <= 0.0f) {
                continue;
            }
            if (cp->impact_velocity >= g_cfg.solver.restitution_velocity_thresh) {
                continue;
            }
            float compression = cp->accumulated_normal_impulse - cp->base_normal_impulse;
            if (compression <= 0.0f) {
                continue;
            }
            float lambda_r = e * compression;
            /* Newton bound: restitution reverses the RECORDED approach, not
             * the accumulated sum. Interleaved joint bias can re-inject
             * approach every iteration (joint pulls, contact re-stops), so
             * the accumulator exceeds true compression (measured 7x). The
             * Newtonian payment e*(-vn_impact)*m_eff is immune to that. */
            float approach = -cp->impact_velocity;
            if (approach < 0.0f) {
                approach = 0.0f;
            }
            float newton_bound = e * approach * cp->effective_mass_normal;
            if (lambda_r > newton_bound) {
                lambda_r = newton_bound;
            }
            float cap = g_cfg.solver.max_restitution_bias * cp->effective_mass_normal;
            if (lambda_r > cap) {
                lambda_r = cap;
            }
            if (lambda_r <= 0.0f) {
                continue;
            }
            cp->accumulated_normal_impulse += lambda_r;
            vector3 impulse = vector3_scaling(man->normal_vector, lambda_r);
            if (!man->object_a->static_state) {
                man->object_a->velocity = vector3_subtraction(
                    man->object_a->velocity, vector3_scaling(impulse, man->object_a->inverse_mass));
                man->object_a->angular_velocity = vector3_subtraction(
                    man->object_a->angular_velocity,
                    math3_multiplication_vector3(man->object_a->inverse_inertia_system,
                                                 vector3_cross(cp->ra, impulse)));
            }
            if (!man->object_b->static_state) {
                man->object_b->velocity = vector3_addition(
                    man->object_b->velocity, vector3_scaling(impulse, man->object_b->inverse_mass));
                man->object_b->angular_velocity = vector3_addition(
                    man->object_b->angular_velocity,
                    math3_multiplication_vector3(man->object_b->inverse_inertia_system,
                                                 vector3_cross(cp->rb, impulse)));
            }
        }
    }
}

/* Rolling + spin resistance (Coulomb-style contact-patch model).
 * Moment opposes angular velocity, scaled by the true normal force (the
 * solved, bias-free accumulated impulse / dt) and a patch lever. Consumes
 * world.rolling_resistance_coeff. Clamped per-axis so resistance can never
 * reverse spin (no energy creation). Runs ONCE per tick after the velocity
 * iterations (per-iteration application would multiply the torque by the
 * iteration count). Independent of the tangent frame: pure rolling has no
 * slip direction, but resistance torque is still physical.
 * Note: applied per body, so a body-body contact dissipates on both sides
 * (shared patch, 2x rate); floor contacts are exact. */
void collision_apply_rolling_resistance(collision_data *manifolds, int manifold_count, float dt) {
    if ((!manifolds) || (manifold_count <= 0) || (dt <= 0.0f)) {
        return;
    }
    float rolling_mu = g_cfg.world.rolling_resistance_coeff;
    if (rolling_mu <= 0.0f) {
        return;
    }
    for (int m = 0; m < manifold_count; m++) {
        collision_data *man = &manifolds[m];
        for (int i = 0; i < man->contact_count; i++) {
            contact_point_data *cp = &man->contacts[i];
            if (cp->accumulated_normal_impulse <= 0.0f) {
                continue;
            }
            {
                float normal_force = cp->accumulated_normal_impulse / dt;
                rigidbody *bodies[2] = {man->object_a, man->object_b};
                vector3 rlev[2] = {cp->ra, cp->rb};
                for (int bi = 0; bi < 2; bi++) {
                    rigidbody *bd = bodies[bi];
                    if ((bd->static_state) || (bd->is_sleeping)) {
                        continue;
                    }
                    /* Rolling part: oppose tangential-plane spin. */
                    vector3 spin_n = vector3_scaling(man->normal_vector,
                                                     vector3_dot(bd->angular_velocity, man->normal_vector));
                    vector3 roll_w = vector3_subtraction(bd->angular_velocity, spin_n);
                    float roll_speed = vector3_length(roll_w);
                    float lever = sqrtf(vector3_length_squared(rlev[bi]));
                    if ((roll_speed > 0.0001f) && (lever > 0.0001f)) {
                        vector3 roll_axis = vector3_scaling(roll_w, 1.0f / roll_speed);
                        float inertia_axis = 1.0f / fmaxf(vector3_dot(
                            roll_axis, math3_multiplication_vector3(bd->inverse_inertia_system, roll_axis)),
                            1e-9f);
                        float dw = rolling_mu * normal_force * lever * dt / inertia_axis;
                        if (dw > roll_speed) {
                            dw = roll_speed;
                        }
                        bd->angular_velocity = vector3_subtraction(
                            bd->angular_velocity, vector3_scaling(roll_axis, dw));
                    }
                    /* Spin part: oppose twist about the contact normal with a
                     * patch-scale lever (contact patch << body size). */
                    float spin_speed = vector3_length(spin_n);
                    float patch = 0.15f * sqrtf(vector3_length_squared(rlev[bi]));
                    if ((spin_speed > 0.0001f) && (patch > 0.00001f)) {
                        vector3 spin_axis = vector3_scaling(spin_n, 1.0f / spin_speed);
                        float inertia_spin = 1.0f / fmaxf(vector3_dot(
                            spin_axis, math3_multiplication_vector3(bd->inverse_inertia_system, spin_axis)),
                            1e-9f);
                        float dw_spin = rolling_mu * normal_force * patch * dt / inertia_spin;
                        if (dw_spin > spin_speed) {
                            dw_spin = spin_speed;
                        }
                        bd->angular_velocity = vector3_subtraction(
                            bd->angular_velocity, vector3_scaling(spin_axis, dw_spin));
                    }
                }
            }
        }
    }
}

void contact_cache_save(struct physics_world *world, collision_data *manifolds, int count) {
    /* Per-world warm-start cache (no global fallback remains). A missing
     * cache degrades to no warm start for the next tick. */
    if ((!world) || (!world->world_contact_cache)) {
        return;
    }
    int *cache_count = &world->world_contact_cache_count;
    cached_contact *cache_array = world->world_contact_cache;
    int32_t *hash_head = world->contact_hash_head;
    *cache_count = 0;
    for (int m = 0; m < count; m++) {
        collision_data *manifold = &manifolds[m];
        for (int i = 0; i < manifold->contact_count; i++) {
            if (*cache_count >= max_cached_contacts) {
                break;
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
            /* Remember the stick frame for resting contacts next tick. */
            cc->tangent_dir = cp->tangent_vector;
        }
        if (*cache_count >= max_cached_contacts) {
            break;
        }
    }
    /* Rebuild the lookup chains in array order (reverse-prepend), so a
     * lookup walk visits candidates in exactly the order the legacy linear
     * scan used: identical first-hit. Zero-id entries never match any
     * predicate and stay unchained. */
    if (hash_head) {
        for (int h = 0; h < CONTACT_HASH_SIZE; h++) {
            hash_head[h] = -1;
        }
        for (int c = *cache_count - 1; c >= 0; c--) {
            cached_contact *cc = &cache_array[c];
            if ((cc->object_id_a == 0) || (cc->object_id_b == 0)) {
                cc->hash_next = -1;
                continue;
            }
            uint32_t slot = contact_pair_key(cc->object_id_a, cc->object_id_b);
            cc->hash_next = hash_head[slot];
            hash_head[slot] = c;
        }
    }
}

/* Split impulse (Erin Catto, GDC 2009): correct residual penetration by
 * moving positions directly, mass-weighted, with zero velocity change.
 * Runs after the velocity iterations. Unlike Baumgarte bias velocity, this
 * adds no energy to the momentum solve and cannot inflate friction. */
void collision_apply_split_impulse(collision_data *manifolds, int manifold_count, float dt) {
    if ((!manifolds) || (manifold_count <= 0) || (dt <= 0.0f)) {
        return;
    }
    const float slop = g_cfg.solver.penetration_slop;
    const float beta = g_cfg.solver.bias_factor;
    const float max_corr = g_cfg.solver.max_separation_bias * dt;
    for (int m = 0; m < manifold_count; m++) {
        collision_data *man = &manifolds[m];
        rigidbody *body_a = man->object_a;
        rigidbody *body_b = man->object_b;
        if ((!body_a) || (!body_b)) {
            continue;
        }
        float inv_a = body_a->static_state ? 0.0f : body_a->inverse_mass;
        float inv_b = body_b->static_state ? 0.0f : body_b->inverse_mass;
        /* Sleeping bodies hold infinite mass (undisturbed rest) unless the
         * correction is significant, in which case wake first. */
        if ((!body_a->static_state) && (body_a->is_sleeping)) {
            inv_a = 0.0f;
        }
        if ((!body_b->static_state) && (body_b->is_sleeping)) {
            inv_b = 0.0f;
        }
        float inv_sum = inv_a + inv_b;
        if (inv_sum <= 0.0f) {
            /* Both sides locked: check whether the overlap is significant
             * enough to wake the dynamic sleepers. */
            float deepest_check = 0.0f;
            for (int i = 0; i < man->contact_count; i++) {
                if (man->contacts[i].penetration > deepest_check) {
                    deepest_check = man->contacts[i].penetration;
                }
            }
            if (deepest_check > g_cfg.depenetration.wake_depth_thresh) {
                if (!body_a->static_state) {
                    rigidbody_wake(body_a);
                }
                if (!body_b->static_state) {
                    rigidbody_wake(body_b);
                }
            }
            continue;
        }
        float deepest = 0.0f;
        for (int i = 0; i < man->contact_count; i++) {
            if (man->contacts[i].penetration > deepest) {
                deepest = man->contacts[i].penetration;
            }
        }
        float corr = beta * fmaxf(deepest - slop, 0.0f);
        if (corr > max_corr) {
            corr = max_corr;
        }
        if (corr <= 0.0f) {
            continue;
        }
        vector3 shift = vector3_scaling(man->normal_vector, corr / inv_sum);
        if (!body_a->static_state) {
            body_a->position = vector3_subtraction(body_a->position, vector3_scaling(shift, inv_a));
            if (corr > 0.01f) {
                rigidbody_wake(body_a);
            }
        }
        if (!body_b->static_state) {
            body_b->position = vector3_addition(body_b->position, vector3_scaling(shift, inv_b));
            if (corr > 0.01f) {
                rigidbody_wake(body_b);
            }
        }
    }
}

void contact_cache_clear(struct physics_world *world) {
    if (!world) {
        return;
    }
    world->world_contact_cache_count = 0;
}

/* CCD swept clamp (see header). Position pre-clamp only: narrowphase +
 * the velocity solver do the actual response at the TOI pose. */
static float ccd_support_depth(const rigidbody *body) {
    /* Lowest-point offset below the center along world -Y. */
    if (body->type == object_sphere) {
        return body->radius;
    }
    if (body->type == object_cylinder) {
        float ay = body->cached_axes[0].y;
        if (ay > 1.0f) {
            ay = 1.0f;
        }
        if (ay < -1.0f) {
            ay = -1.0f;
        }
        return body->radius * sqrtf(fmaxf(0.0f, 1.0f - ay * ay)) +
               body->cylinder_half_length * fabsf(ay);
    }
    vector3 down = {0.0f, -1.0f, 0.0f};
    return body->half_extensions.x * fabsf(vector3_dot(body->cached_axes[0], down)) +
           body->half_extensions.y * fabsf(vector3_dot(body->cached_axes[1], down)) +
           body->half_extensions.z * fabsf(vector3_dot(body->cached_axes[2], down));
}

static float ccd_min_thickness(const rigidbody *body) {
    if (body->type == object_sphere) {
        return body->radius;
    }
    if (body->type == object_cylinder) {
        return fminf(body->radius, body->cylinder_half_length);
    }
    return fminf(body->half_extensions.x, fminf(body->half_extensions.y, body->half_extensions.z));
}

int collision_ccd_sweep_clamp(rigidbody *bodies, int body_count, float dt) {
    if ((!bodies) || (body_count <= 0) || (dt <= 0.0f)) {
        return 0;
    }
    int clamped = 0;
    for (int i = 0; i < body_count; i++) {
        rigidbody *mover = &bodies[i];
        if ((mover->static_state) || (mover->is_sleeping)) {
            continue;
        }
        float speed = vector3_length(mover->velocity);
        if (speed <= 0.0001f) {
            continue;
        }
        float thickness = ccd_min_thickness(mover);
        if ((thickness <= 0.0f) || (!isfinite(thickness))) {
            continue;
        }
        float displacement = speed * dt;
        /* Plane sweep is O(1) per body: run it whenever the tick motion
         * exceeds slop, so impacts never fall in the gap between slop-band
         * sampling and the safety nets (which would delete the bounce).
         * The O(n^2) sphere/box sweep stays gated on body thickness. */
        bool do_volumes = displacement > thickness;
        if ((displacement <= g_cfg.solver.penetration_slop) && (!do_volumes)) {
            continue; /* discrete sampling suffices */
        }
        float best_toi = dt;
        bool hit = false;

        /* 1. Floor plane y = 0. */
        if (mover->velocity.y < -0.0001f) {
            float lowest = mover->position.y - ccd_support_depth(mover);
            if (lowest > 0.0f) {
                float toi = lowest / -mover->velocity.y;
                if ((toi > 0.0f) && (toi < best_toi)) {
                    best_toi = toi;
                    hit = true;
                }
            }
        }

        /* 2. Spheres (dynamic or static) and static boxes: only when the
         * tick motion exceeds body thickness (cost control). */
        if (do_volumes) {
        for (int j = 0; j < body_count; j++) {
            if (j == i) {
                continue;
            }
            rigidbody *other = &bodies[j];
            if (other->type == object_sphere) {
                vector3 dp = vector3_subtraction(other->position, mover->position);
                vector3 other_v = other->is_sleeping ? vector3_zero() : other->velocity;
                vector3 dv = vector3_subtraction(other_v, mover->velocity);
                float rr = mover->radius + other->radius;
                if (mover->type != object_sphere) {
                    rr = ccd_min_thickness(mover) + other->radius;
                }
                float a = vector3_dot(dv, dv);
                float c = vector3_dot(dp, dp) - rr * rr;
                if ((c <= 0.0f) || (a <= 1e-12f)) {
                    continue; /* already overlapping or no relative motion */
                }
                float b = 2.0f * vector3_dot(dp, dv);
                float disc = b * b - 4.0f * a * c;
                if (disc < 0.0f) {
                    continue;
                }
                float toi = (-b - sqrtf(disc)) / (2.0f * a);
                if ((toi > 0.0f) && (toi < best_toi) && (toi <= dt)) {
                    best_toi = toi;
                    hit = true;
                }
            } else if ((other->type == object_cube) && (other->static_state)) {
                /* Swept sphere-vs-static-OBB via slab test in box space. */
                float sr = (mover->type == object_sphere) ? mover->radius : ccd_min_thickness(mover);
                vector3 rel = vector3_subtraction(mover->position, other->position);
                vector3 ax0 = other->cached_axes[0];
                vector3 ax1 = other->cached_axes[1];
                vector3 ax2 = other->cached_axes[2];
                float pl[3] = {vector3_dot(rel, ax0), vector3_dot(rel, ax1), vector3_dot(rel, ax2)};
                float vl[3] = {vector3_dot(mover->velocity, ax0), vector3_dot(mover->velocity, ax1),
                               vector3_dot(mover->velocity, ax2)};
                float ex[3] = {other->half_extensions.x + sr, other->half_extensions.y + sr,
                               other->half_extensions.z + sr};
                float tmin = 0.0f, tmax = dt;
                bool miss = false;
                for (int a3 = 0; a3 < 3; a3++) {
                    float p = pl[a3], v = vl[a3], e = ex[a3];
                    if (fabsf(v) < 1e-9f) {
                        if ((p < -e) || (p > e)) {
                            miss = true;
                            break;
                        }
                    } else {
                        float t1 = (-e - p) / v;
                        float t2 = (e - p) / v;
                        if (t1 > t2) {
                            float tmp = t1;
                            t1 = t2;
                            t2 = tmp;
                        }
                        if (t1 > tmin) {
                            tmin = t1;
                        }
                        if (t2 < tmax) {
                            tmax = t2;
                        }
                        if (tmin > tmax) {
                            miss = true;
                            break;
                        }
                    }
                }
                /* tmin > 0 with a hit means true entry (starting outside);
                 * starting inside gives tmin <= 0: discrete path owns it. */
                if ((!miss) && (tmin > 0.0f) && (tmin < best_toi)) {
                    best_toi = tmin;
                    hit = true;
                }
            }
        }
        } /* do_volumes */

        if (hit) {
            mover->position = vector3_addition(mover->position, vector3_scaling(mover->velocity, best_toi));
            rigidbody_wake(mover);
            rigidbody_update_axes(mover);
            clamped++;
        }
    }
    return clamped;
}
/* Cylinder vs static floor plane.
 * Models the cylinder as axle segment + radius. Each axle endpoint acts
 * like a sphere of radius r; an endpoint below the plane yields a contact.
 * Two contacts (one per axle end) give a stable resting wheel.
 * Normal matches the sphere-floor convention: (0,-1,0). */
/* LIST4 NEW-01: robust cylinder/static-plane contact.
 *
 * The old version only tested the two axle endpoints as if they were
 * sphere centres. That misses the true lowest point when the cylinder
 * is tipped, and can allow a cylinder to fall through the floor.
 *
 * This version computes the actual lowest support point of the cylinder
 * against a horizontal plane. For near-horizontal axles it generates two
 * contacts at the axle ends for stacking/resting stability. For tilted or
 * vertical axles it generates the correct single support contact.
 */
bool collision_static_plane_cylinder(rigidbody *cyl, float plane_y, collision_data *collision_output_data) {
    if (cyl->type != object_cylinder) {
        return false;
    }

    float r = cyl->radius;
    float h = cyl->cylinder_half_length;

    if ((r <= 0.0f) || (h <= 0.0f) || (!isfinite(r)) || (!isfinite(h))) {
        return false;
    }

    vector3 axis = cyl->cached_axes[0];
    float axis_len_sq = vector3_length_squared(axis);

    if (axis_len_sq < 1e-8f) {
        axis = (vector3){1.0f, 0.0f, 0.0f};
        axis_len_sq = 1.0f;
    }

    axis = vector3_scaling(axis, 1.0f / sqrtf(axis_len_sq));

    float ay = axis.y;
    if (ay > 1.0f) {
        ay = 1.0f;
    }
    if (ay < -1.0f) {
        ay = -1.0f;
    }

    /*
     * For a cylinder against a horizontal plane:
     *
     * vertical half-extent =
     *     r * sqrt(1 - axis.y^2)
     *   + h * fabs(axis.y)
     *
     * The first term is the barrel contribution.
     * The second term is the end-cap/axle contribution.
     */
    float horizontal = sqrtf(fmaxf(0.0f, 1.0f - ay * ay));

    /*
     * Radial offset to the lowest barrel point.
     * The plane normal is up, so the downward direction is (0,-1,0).
     * Remove the component parallel to the axle.
     */
    vector3 down = {0.0f, -1.0f, 0.0f};
    float down_along_axis = vector3_dot(down, axis);
    vector3 radial = vector3_subtraction(down, vector3_scaling(axis, down_along_axis));
    float radial_len = vector3_length(radial);

    if (radial_len > 1e-6f) {
        radial = vector3_scaling(radial, r / radial_len);
    } else {
        radial = vector3_zero();
    }

    rigidbody *plane_body = collision_static_plane_body_proxy(plane_y);

    collision_output_data->object_a = cyl;
    collision_output_data->object_b = plane_body;
    collision_output_data->normal_vector = (vector3){0.0f, -1.0f, 0.0f};
    collision_output_data->contact_count = 0;

    /*
     * Near-horizontal axle:
     * generate two contacts at the axle ends for stability.
     * This is the normal FTC wheel case.
     * Slop-gated (see clip_obb_faces): wheels rolling within slop keep
     * persistent friction contacts instead of flickering support.
     */
    if (fabsf(ay) < 0.35f) {
        float axle_offsets[2] = {-h, h};
        float wheel_slop = g_cfg.solver.penetration_slop;

        for (int i = 0; i < 2; i++) {
            vector3 end_center =
                vector3_addition(cyl->position, vector3_scaling(axis, axle_offsets[i]));

            vector3 contact_point = vector3_addition(end_center, radial);
            float local_penetration = plane_y - contact_point.y;

            if ((local_penetration > -wheel_slop) && (collision_output_data->contact_count < 2)) {
                contact_point_data *cp =
                    &collision_output_data->contacts[collision_output_data->contact_count];
                cp->position = contact_point;
                cp->penetration = (local_penetration > 0.0f) ? local_penetration : 0.0f;
                collision_output_data->contact_count++;
            }
        }
    }

    /*
     * Tilted or vertical axle:
     * generate the single true support contact.
     */
    if (collision_output_data->contact_count == 0) {
        float axle_offset = (ay >= 0.0f) ? -h : h;

        vector3 contact_point = vector3_addition(
            vector3_addition(cyl->position, vector3_scaling(axis, axle_offset)),
            radial);

        float local_penetration = plane_y - contact_point.y;

        /*
         * Fallback safety:
         * if numerical error makes the direct contact miss, use the
         * analytical lowest height. Slop-gated like the main path.
         */
        if (local_penetration <= -g_cfg.solver.penetration_slop) {
            float lowest_offset = (r * horizontal) + (h * fabsf(ay));
            local_penetration = plane_y - (cyl->position.y - lowest_offset);
        }

        if (local_penetration > -g_cfg.solver.penetration_slop) {
            contact_point_data *cp = &collision_output_data->contacts[0];
            cp->position = contact_point;
            cp->penetration = (local_penetration > 0.0f) ? local_penetration : 0.0f;
            collision_output_data->contact_count = 1;
        }
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

/* LIST4 NEW-02 / NEW-03:
 * Correct cylinder-vs-cube contact normal and sample resolution.
 *
 * Convention:
 *   object_a = cylinder
 *   object_b = cube
 *   normal must point from cylinder toward cube.
 *
 * The old version used:
 *   cyl->position - best_on_obb
 * which pointed from the cube surface toward the cylinder center.
 * That inverted the solver response for floor/wall contacts.
 *
 * The corrected version uses:
 *   best_on_obb - best_pt
 * where best_pt is the closest sampled point on the cylinder axle.
 */
bool collision_cylinder_cube(rigidbody *cyl, rigidbody *cube,
                             collision_data *out) {
    if ((cyl->type != object_cylinder) || (cube->type != object_cube)) {
        return false;
    }

    float r = cyl->radius;
    float h = cyl->cylinder_half_length;

    if ((r <= 0.0f) || (h <= 0.0f) || (!isfinite(r)) || (!isfinite(h))) {
        return false;
    }

    vector3 axis = cyl->cached_axes[0];
    float axis_len_sq = vector3_length_squared(axis);

    if (axis_len_sq < 1e-8f) {
        axis = (vector3){1.0f, 0.0f, 0.0f};
        axis_len_sq = 1.0f;
    }

    axis = vector3_scaling(axis, 1.0f / sqrtf(axis_len_sq));

    vector3 e1 = vector3_subtraction(cyl->position, vector3_scaling(axis, h));
    vector3 e2 = vector3_addition(cyl->position, vector3_scaling(axis, h));
    vector3 seg = vector3_subtraction(e2, e1);

    /*
     * Nine samples gives better coverage than the original five without
     * becoming expensive. For FTC wheel-sized cylinders this is enough.
     */
    const int SAMPLES = 9;

    /* Collect every penetrating sample: a cylinder lying on a face needs
     * several contacts along the axle for stable support (single-point
     * contact rocks). Keep the 4 deepest, slop-gated like other paths. */
    vector3 samp_pt[10];
    vector3 samp_obb[10];
    float samp_d[10];
    int samp_count = 0;

    for (int s = 0; s <= SAMPLES; s++) {
        float t = (float) s / (float) SAMPLES;
        vector3 pt = vector3_addition(e1, vector3_scaling(seg, t));

        vector3 rel = vector3_subtraction(pt, cube->position);
        vector3 *axes = cube->cached_axes;

        vector3 local = (vector3){
            vector3_dot(rel, axes[0]),
            vector3_dot(rel, axes[1]),
            vector3_dot(rel, axes[2])
        };

        vector3 clamped = (vector3){
            fmaxf(-cube->half_extensions.x, fminf(cube->half_extensions.x, local.x)),
            fmaxf(-cube->half_extensions.y, fminf(cube->half_extensions.y, local.y)),
            fmaxf(-cube->half_extensions.z, fminf(cube->half_extensions.z, local.z))
        };

        vector3 on_obb = cube->position;
        on_obb = vector3_addition(on_obb, vector3_scaling(axes[0], clamped.x));
        on_obb = vector3_addition(on_obb, vector3_scaling(axes[1], clamped.y));
        on_obb = vector3_addition(on_obb, vector3_scaling(axes[2], clamped.z));

        float d = vector3_length(vector3_subtraction(pt, on_obb));

        if ((d < r + g_cfg.solver.penetration_slop) && (samp_count < 10)) {
            samp_pt[samp_count] = pt;
            samp_obb[samp_count] = on_obb;
            samp_d[samp_count] = d;
            samp_count++;
        }
    }

    if (samp_count == 0) {
        return false;
    }

    out->object_a = cyl;
    out->object_b = cube;
    out->contact_count = 0;

    /* Deepest-first selection into the 4-slot manifold. */
    for (int k = 0; (k < samp_count) && (out->contact_count < 4); k++) {
        int deepest_idx = -1;
        float deepest_d = 1e30f;
        for (int s = 0; s < samp_count; s++) {
            if (samp_d[s] < deepest_d) {
                bool used = false;
                for (int u = 0; u < out->contact_count; u++) {
                    if (out->contacts[u].position.x == samp_obb[s].x &&
                        out->contacts[u].position.y == samp_obb[s].y &&
                        out->contacts[u].position.z == samp_obb[s].z) {
                        used = true;
                        break;
                    }
                }
                if (!used) {
                    deepest_d = samp_d[s];
                    deepest_idx = s;
                }
            }
        }
        if (deepest_idx < 0) {
            break;
        }
        /* Normal: axle sample toward the cube surface (A->B). Degenerate
         * (sample inside the cube): escape toward the cube center, which
         * pushes A outward under the A->B sign convention. */
        vector3 toward_obb = vector3_subtraction(samp_obb[deepest_idx], samp_pt[deepest_idx]);
        float toward_len = vector3_length(toward_obb);
        vector3 nrm;
        if (toward_len > 0.0001f) {
            nrm = vector3_scaling(toward_obb, 1.0f / toward_len);
        } else {
            vector3 fallback = vector3_subtraction(cube->position, samp_pt[deepest_idx]);
            float fallback_len = vector3_length(fallback);
            if (fallback_len > 0.0001f) {
                nrm = vector3_scaling(fallback, 1.0f / fallback_len);
            } else {
                nrm = (vector3){0.0f, -1.0f, 0.0f};
            }
        }
        if (out->contact_count == 0) {
            out->normal_vector = nrm;
        }
        contact_point_data *cp = &out->contacts[out->contact_count];
        float pen = r - samp_d[deepest_idx];
        cp->penetration = (pen > 0.0f) ? pen : 0.0f;
        cp->position = samp_obb[deepest_idx];
        out->contact_count++;
    }

    if (out->contact_count == 0) {
        return false;
    }

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

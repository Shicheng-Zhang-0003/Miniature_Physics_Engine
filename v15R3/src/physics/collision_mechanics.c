#include "../mpe_engine.h"
#include "collision_mechanics.h"
#include "../core/physics_world.h" /* MFS_131 */
#include "../core/det_math.h"
#include <stdint.h>
#include <stdlib.h>


/* Warm-start hash: 4096 buckets over canonical (min_id, max_id) pairs.
 * Chains live in cached_contact.hash_next and are rebuilt on every save
 * in array order (reverse-prepend), so a lookup walk visits candidates in
 * exactly the array order the old legacy linear scan used: identical
 * first-hit, O(chain) instead of O(cache). Heads live in
 * physics_world.contact_hash_head (heap, per world). The old file-scope
 * global cache is retired; a NULL cache degrades to all-miss. */
#define contact_hash_bits 12
#define contact_hash_size (1 << contact_hash_bits)
#define contact_hash_mask (contact_hash_size - 1)

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
    return (uint32_t) (key & contact_hash_mask);
}

bool collision_dual_sphere(rigidbody *rigidbody_object_a, rigidbody *rigidbody_object_b,
                           collision_data *collision_output_data) {
    vector3 relative_position_vector = vector3_subtraction(rigidbody_object_b->position, rigidbody_object_a->position);
    float distance_between_centres_squared = vector3_length_squared(relative_position_vector);
    float total_combined_radius = rigidbody_object_a->radius + rigidbody_object_b->radius;
    /* TRUTH P0-4: slop-band persistence parity with cube/cylinder/floor.
     * Resting spheres at exact contact (dist == r1+r2) must report a
     * zero-depth contact for friction; strict < flickers support at 30Hz.
     * Admit pen >= -slop, clamp negatives to 0 (no bounce, friction only). */
    float slop = g_cfg.solver.penetration_slop;
    float outer = total_combined_radius + slop;
    if (distance_between_centres_squared >= outer * outer) {
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
    float raw_pen = total_combined_radius - distance_between_centres;
    cp->penetration = (raw_pen > 0.0f) ? raw_pen : 0.0f;
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
    /* TRUTH: slop parity. Outside branch must admit [-slop,0) as zero-depth
     * like every other path (dual_sphere, clip, floor, cyl). Old strict
     * radius test flickered resting contact. */
    float slop_sc = g_cfg.solver.penetration_slop;
    float outer_sc = sphere->radius + slop_sc;
    if (!inside && distance_sq > outer_sc * outer_sc) {
        return false;
    }
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
        float raw_pen_sc = sphere->radius - distance;
        /* TRUTH: clamp slop-band negatives to zero (friction-only). */
        cp->penetration = (raw_pen_sc > 0.0f) ? raw_pen_sc : 0.0f;
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
    /* TRUTH: raw 15-axis minimum, no hysteresis. An earlier 0.9x face-bias
     * suppressed true edge contacts within a 10% band (wrong points/normal/
     * torque for tilted boxes) to hide solver jitter. Real boxes DO change
     * manifold discontinuously on tilt; the solver (slop persistence +
     * warm-start + 64 iterations) must handle it, not geometry lies.
     * Hysteresis deleted; genuine face contacts still win raw whenever
     * shallower (the common resting case). */
    float minimum_overlap = 0.0f;
    vector3 best_axis = {0, 0, 0};
    int best_axis_index = -1;
    if ((edge_best_axis_index >= 0) && (edge_minimum_overlap < face_minimum_overlap)) {
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
    /* TRUTH: clip can return 0 (SAT/clip disagreement at grazing angles).
     * Old code returned true with 0 contacts -> phantom manifold consumed a
     * slot, set has_contact=1 (killing exact gravity), solver no-op. */
    if (collision_output_data->contact_count <= 0) {
        return false;
    }
    return true;
}
rigidbody *collision_static_plane_body_proxy(float plane_y) {
    /* Thread-local so concurrent worlds/threads never share mutable state.
     * restitution=1.0 is intentionally neutral: effective bounce is
     * min(body_restitution, 1.0) == body_restitution. */
    static _Thread_local rigidbody static_plane_body;
    static _Thread_local int static_plane_initialized = 0;

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

/* TRUTH: pair-novelty probe for wake-on-first-touch. A sleeping body must
 * wake when a NEW contact edge forms at ANY relative speed (slow kinematic
 * pushers defeat velocity gates; per-body "had contact" flags are blinded
 * by floor contacts every rester holds). The warm-start cache IS the
 * contact memory: every solved manifold is saved each tick, so a pair with
 * no entry has never touched (or its entries were evicted — wake is then
 * the safe direction). Pair-level on purpose: rotation drift changes local
 * points but not pair novelty. Linear scan, early-out; only called for
 * pairs with a sleeping side (rare). NULL cache => true (fail-open awake).
 * NOTE: probe strictly prior ticks — contact_cache_save runs at step end,
 * after all process_pair calls. */
bool contact_cache_has_pair(struct physics_world *world, uint32_t id_a, uint32_t id_b) {
    if (!world || id_a == 0 || id_b == 0) {
        return true;
    }
    cached_contact *cache = world->world_contact_cache;
    int count = world->world_contact_cache_count;
    if (!cache || count <= 0) {
        return false;
    }
    if (count > max_cached_contacts) {
        count = max_cached_contacts;
    }
    for (int i = 0; i < count; i++) {
        uint32_t ca = cache[i].object_id_a;
        uint32_t cb = cache[i].object_id_b;
        if (((ca == id_a) && (cb == id_b)) || ((ca == id_b) && (cb == id_a))) {
            return true;
        }
    }
    return false;
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

    /* TRUTH: friction/restitution/kinematic affect the solved impulse.
     * Old stamp omitted them: editing friction or toggling kinematic hit a
     * stale acc_n*new_mu (wrong friction cone for a tick). Include. */
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->friction_static));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->friction_kinetic));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->restitution));
    stamp = a3_task05_mix_u32(stamp, rigid_body->kinematic ? 2u : 0u);
    stamp = a3_task05_mix_u32(stamp, rigid_body->is_sleeping ? 4u : 0u);

    /* Orientation quantized to 1e-3: rotation invalidates local-space cache
     * matching. Without this, a body that rotates significantly between
     * frames can false-positive match a stale contact (PHYS-007). Quantizing
     * keeps resting contacts stable while forcing a miss on real rotation. */
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(roundf(rigid_body->orientation.w * 1000.0f) / 1000.0f));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(roundf(rigid_body->orientation.x * 1000.0f) / 1000.0f));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(roundf(rigid_body->orientation.y * 1000.0f) / 1000.0f));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(roundf(rigid_body->orientation.z * 1000.0f) / 1000.0f));

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
        /* TRUTH: strict both-side matching. Old side-A-only aliased two B
         * bodies sharing an A anchor within 5cm (wrong impulse injection).
         * Both material points must coincide; resting contacts satisfy this
         * exactly (body-local storage survives rigid translation). */
        float dist_a_sq = vector3_length_squared(vector3_subtraction(cc->local_position_a, local_a));
        float dist_b_sq = vector3_length_squared(vector3_subtraction(cc->local_position_b, local_b));
        if ((dist_a_sq < match_dist_sq) && (dist_b_sq < match_dist_sq) &&
            (a3_task05_cached_impulses_are_usable(cc->accumulated_normal_impulse, cc->accumulated_tangent_impulse))) {
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
        cp->accumulated_tangent2_impulse = 0.0f;
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

        /* Warm-start matching: strict both-side material-point coincidence
         * (see contact_cache_match_role). A hit adopts cached impulses at
         * full step — no damped SOR, no provenance flags. NOTE (truth):
         * only the normal + PRIMARY tangent are restored. The second disc
         * tangent is deliberately never cached: t2_new = n×t1_new can point
         * anywhere relative to a cached t2_old when frames rotate, and
         * restoring it injected sideways energy; cold t2 re-converges in
         * the relaxation sweeps. Normal is frame-independent: always warm. */
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
                    cache_match_found = 1;
                    break;
                } else if (role == 2) {
                    /* Swapped body order: normal stays positive (manifold
                     * normal already points A->B); tangent reverses with
                     * the relative-velocity order. */
                    cp->accumulated_normal_impulse = fmaxf(cc->accumulated_normal_impulse, 0.0f);
                    cp->accumulated_tangent_impulse = -cc->accumulated_tangent_impulse;
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
                    cache_match_found = 1;
                    break;
                } else if (role == 2) {
                    cp->accumulated_normal_impulse = fmaxf(cc->accumulated_normal_impulse, 0.0f);
                    cp->accumulated_tangent_impulse = -cc->accumulated_tangent_impulse;
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
        vector3 ang_a = vector3_cross(
            math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_a), ra_cross_n), cp->ra);
        vector3 ang_b = vector3_cross(
            math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_b), rb_cross_n), cp->rb);
        float k_normal = rigidbody_effective_inv_mass(m->object_a) + rigidbody_effective_inv_mass(m->object_b) +
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
                vector3_cross(math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_a), ra_cross_t), cp->ra);
            vector3 ang_b_t =
                vector3_cross(math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_b), rb_cross_t), cp->rb);
            float k_tangent = rigidbody_effective_inv_mass(m->object_a) + rigidbody_effective_inv_mass(m->object_b) +
                              vector3_dot(vector3_addition(ang_a_t, ang_b_t), cp->tangent_vector);
            cp->effective_mass_tangent = (k_tangent > 0.0f) ? (1.0f / k_tangent) : 0.0f;
            if (vector3_length_squared(cp->tangent2) > 0.0001f) {
                vector3 ra_cross_t2 = vector3_cross(cp->ra, cp->tangent2);
                vector3 rb_cross_t2 = vector3_cross(cp->rb, cp->tangent2);
                vector3 ang_a_t2 = vector3_cross(
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_a), ra_cross_t2), cp->ra);
                vector3 ang_b_t2 = vector3_cross(
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_b), rb_cross_t2), cp->rb);
                float k_tangent2 = rigidbody_effective_inv_mass(m->object_a) + rigidbody_effective_inv_mass(m->object_b) +
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
        /* NOTE (truth, measured): the primary tangent keeps its cached
         * magnitude even on fresh slip (original static-hold behavior). An
         * experiment zeroing it here fixed a 127 rad/s wheel singularity but
         * regressed the 6-cube stack (drift 0.27m vs 0.003m), so it was
         * reverted: extreme-spin contacts are handled by keeping the wheel
         * test in the resolvable regime (see driven_wheel_test), not by
         * weakening everyday friction. The second disc tangent is never
         * cached (cold every tick): t2_new = n×t1_new can point anywhere
         * relative to a cached t2_old when frames rotate. */

        if (cp->accumulated_normal_impulse != 0.0f || cp->accumulated_tangent_impulse != 0.0f ||
            cp->accumulated_tangent2_impulse != 0.0f) {
            vector3 impulse = vector3_addition(
                vector3_scaling(m->normal_vector, cp->accumulated_normal_impulse),
                vector3_addition(vector3_scaling(cp->tangent_vector, cp->accumulated_tangent_impulse),
                                 vector3_scaling(cp->tangent2, cp->accumulated_tangent2_impulse)));
            if (!m->object_a->static_state) {
                m->object_a->velocity =
                    vector3_subtraction(m->object_a->velocity, vector3_scaling(impulse, rigidbody_effective_inv_mass(m->object_a)));
                m->object_a->angular_velocity = vector3_subtraction(
                    m->object_a->angular_velocity,
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_a), vector3_cross(cp->ra, impulse)));
            }
            if (!m->object_b->static_state) {
                m->object_b->velocity =
                    vector3_addition(m->object_b->velocity, vector3_scaling(impulse, rigidbody_effective_inv_mass(m->object_b)));
                m->object_b->angular_velocity = vector3_addition(
                    m->object_b->angular_velocity,
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_b), vector3_cross(cp->rb, impulse)));
            }
        }
        /* Poisson base: compression impulse entering the iterations (warm
         * start included). The restitution pass pays e over the delta. */
        cp->base_normal_impulse = cp->accumulated_normal_impulse;
    }
}

/* Sort context: TRUTH deterministic stable mergesort, no qsort globals.
 * Old static manifold_sort_keys_active was racy/reentrant across threads.
 * Bottom-up mergesort on (key,index) is total-order deterministic,
 * thread-safe, and stable. Manifolds <=8192: O(n log n) with scratch. */
static void collision_manifold_merge_sort(const float *keys, int *order, int *scratch, int n) {
    for (int i = 0; i < n; i++) {
        order[i] = i;
    }
    int *src = order;
    int *dst = scratch;
    for (int width = 1; width < n; width *= 2) {
        for (int lo = 0; lo < n; lo += 2 * width) {
            int mid = lo + width < n ? lo + width : n;
            int hi = lo + 2 * width < n ? lo + 2 * width : n;
            int a = lo, b = mid, o = lo;
            while (a < mid && b < hi) {
                float ka = keys[src[a]];
                float kb = keys[src[b]];
                bool take_a;
                if (ka < kb) {
                    take_a = true;
                } else if (ka > kb) {
                    take_a = false;
                } else {
                    take_a = src[a] < src[b];
                }
                dst[o++] = take_a ? src[a++] : src[b++];
            }
            while (a < mid) {
                dst[o++] = src[a++];
            }
            while (b < hi) {
                dst[o++] = src[b++];
            }
        }
        int *tmp = src;
        src = dst;
        dst = tmp;
    }
    if (src != order) {
        for (int i = 0; i < n; i++) {
            order[i] = src[i];
        }
    }
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
    /* TRUTH: mergesort with thread-local scratch (no malloc, no globals).
     * Deterministic total order, race-free. */
    {
        static _Thread_local int merge_scratch[8192];
        if (manifold_count <= 8192) {
            collision_manifold_merge_sort(world->manifold_sort_keys, order_out, merge_scratch, manifold_count);
            return;
        }
    }
    /* Tiny fallback: insertion sort (deterministic, no globals). */
    for (int i = 1; i < manifold_count; i++) {
        int key_idx = order_out[i];
        float key_val = world->manifold_sort_keys[key_idx];
        int j = i - 1;
        while (j >= 0) {
            int cur_idx = order_out[j];
            float cur_val = world->manifold_sort_keys[cur_idx];
            bool shift = (cur_val > key_val) || (cur_val == key_val && cur_idx > key_idx);
            if (!shift) {
                break;
            }
            order_out[j + 1] = order_out[j];
            j--;
        }
        order_out[j + 1] = key_idx;
    }
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
            /* TRUTH: full step always. Old provenance-gated SOR (warm *=0.5)
             * halved steady-state corrections to mask cache aliasing
             * ping-pong; with strict both-side matching (above) the alias
             * source is gone, so dampening true warm starts only slows
             * convergence 2x. Fixed points unchanged either way. */
            cp->accumulated_normal_impulse = old_impulse + lambda_n;
            if (lambda_n != 0.0f) {
                float applied_n = fabsf(lambda_n);
                if (applied_n > max_applied) {
                    max_applied = applied_n;
                }
                vector3 impulse = vector3_scaling(m->normal_vector, lambda_n);
                if (!m->object_a->static_state) {
                    m->object_a->velocity = vector3_subtraction(
                        m->object_a->velocity, vector3_scaling(impulse, rigidbody_effective_inv_mass(m->object_a)));
                    m->object_a->angular_velocity = vector3_subtraction(
                        m->object_a->angular_velocity,
                        math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_a),
                                                     vector3_cross(cp->ra, impulse)));
                }
                if (!m->object_b->static_state) {
                    m->object_b->velocity = vector3_addition(
                        m->object_b->velocity, vector3_scaling(impulse, rigidbody_effective_inv_mass(m->object_b)));
                    m->object_b->angular_velocity = vector3_addition(
                        m->object_b->angular_velocity,
                        math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_b),
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
                float k = rigidbody_effective_inv_mass(m->object_a) + rigidbody_effective_inv_mass(m->object_b) +
                          vector3_dot(vector3_addition(
                                          vector3_cross(math3_multiplication_vector3(
                                                            rigidbody_effective_inv_inertia(m->object_a), ra_c),
                                                        cp->ra),
                                          vector3_cross(math3_multiplication_vector3(
                                                            rigidbody_effective_inv_inertia(m->object_b), rb_c),
                                                        cp->rb)),
                                      tangent);
                eff1 = (k > 0.0f) ? (1.0f / k) : 0.0f;
                cp->effective_mass_tangent = eff1;
            }
            if (has_t2 && (eff2 <= 0.0f)) {
                vector3 ra_c2 = vector3_cross(cp->ra, tangent2);
                vector3 rb_c2 = vector3_cross(cp->rb, tangent2);
                float k2 = rigidbody_effective_inv_mass(m->object_a) + rigidbody_effective_inv_mass(m->object_b) +
                           vector3_dot(vector3_addition(
                                           vector3_cross(math3_multiplication_vector3(
                                                             rigidbody_effective_inv_inertia(m->object_a), ra_c2),
                                                         cp->ra),
                                           vector3_cross(math3_multiplication_vector3(
                                                             rigidbody_effective_inv_inertia(m->object_b), rb_c2),
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
            /* TRUTH: full friction step (see normal solve: SOR deleted). */
            float step1 = new_acc1 - cp->accumulated_tangent_impulse;
            float step2 = new_acc2 - cp->accumulated_tangent2_impulse;
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
                        m->object_a->velocity, vector3_scaling(friction_delta, rigidbody_effective_inv_mass(m->object_a)));
                    m->object_a->angular_velocity =
                        vector3_subtraction(m->object_a->angular_velocity,
                                            math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_a),
                                                                         vector3_cross(cp->ra, friction_delta)));
                }
                if (!m->object_b->static_state) {
                    m->object_b->velocity = vector3_addition(
                        m->object_b->velocity, vector3_scaling(friction_delta, rigidbody_effective_inv_mass(m->object_b)));
                    m->object_b->angular_velocity =
                        vector3_addition(m->object_b->angular_velocity,
                                         math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_b),
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
void collision_refresh_impact_velocities(collision_data *manifolds, int manifold_count) {
    if ((!manifolds) || (manifold_count <= 0)) {
        return;
    }
    for (int m = 0; m < manifold_count; m++) {
        collision_data *man = &manifolds[m];
        if ((!man->object_a) || (!man->object_b)) {
            continue;
        }
        for (int i = 0; i < man->contact_count; i++) {
            contact_point_data *cp = &man->contacts[i];
            vector3 va = vector3_addition(man->object_a->velocity,
                                          vector3_cross(man->object_a->angular_velocity, cp->ra));
            vector3 vb = vector3_addition(man->object_b->velocity,
                                          vector3_cross(man->object_b->angular_velocity, cp->rb));
            cp->impact_velocity = vector3_dot(vector3_subtraction(vb, va), man->normal_vector);
        }
    }
}
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
            /* TRUTH P0-9: NO artificial restitution cap. The Newton bound
             * above IS the physical bound (e reverses recorded approach).
             * Capping at max_restitution_bias*m_eff (default 20 m/s) deadens
             * fast bounce 7x (144 m/s e=1 pays 20). Param retained for
             * emergency NaN guard at 1e6 scale (never binds physically). */
            {
                float emergency_cap = 1.0e6f * cp->effective_mass_normal;
                if (lambda_r > emergency_cap) {
                    lambda_r = emergency_cap;
                }
            }
            if (lambda_r <= 0.0f) {
                continue;
            }
            cp->accumulated_normal_impulse += lambda_r;
            vector3 impulse = vector3_scaling(man->normal_vector, lambda_r);
            if (!man->object_a->static_state) {
                man->object_a->velocity = vector3_subtraction(
                    man->object_a->velocity, vector3_scaling(impulse, rigidbody_effective_inv_mass(man->object_a)));
                man->object_a->angular_velocity = vector3_subtraction(
                    man->object_a->angular_velocity,
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(man->object_a),
                                                 vector3_cross(cp->ra, impulse)));
            }
            if (!man->object_b->static_state) {
                man->object_b->velocity = vector3_addition(
                    man->object_b->velocity, vector3_scaling(impulse, rigidbody_effective_inv_mass(man->object_b)));
                man->object_b->angular_velocity = vector3_addition(
                    man->object_b->angular_velocity,
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(man->object_b),
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
 * TRUTH: shared patch split 1/2 per side for body-body (old code dissipated
 * 2x: floor exact, body-body double). Spin lever from Hertz a=sqrt(R*delta)
 * using contact penetration (no 0.15 magic). */
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
        /* Shared patch: halve per side when BOTH bodies are dynamic (each
         * side dissipates half the patch loss; floor/static bodies take the
         * full single-sided rate). */
        bool b_dynamic =
            (man->object_b) && (!man->object_b->static_state) && (!man->object_b->is_sleeping);
        bool a_dynamic =
            (man->object_a) && (!man->object_a->static_state) && (!man->object_a->is_sleeping);
        float share = (a_dynamic && b_dynamic) ? 0.5f : 1.0f;
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
                    if ((!bd) || (bd->static_state) || (bd->is_sleeping) || (bd->kinematic)) {
                        continue;
                    }
                    /* TRUTH: roll lever = |r| (contact radius, =R for spheres:
                     * M=mu*N*R, standard Coulomb rolling resistance). Spin
                     * lever = Hertz patch a=sqrt(R*pen_eff), capped 0.3R.
                     * An earlier revision used patch for roll too, which
                     * under-damped 28x (R=0.5,pen=0.5mm: R/patch~32) and
                     * failed rolling_decay (15.8m vs 4-14m). Roll and spin
                     * are different physics: roll resists translation via
                     * R, spin resists yaw via patch. Slop zero-depth gets
                     * pen_eff=0.5mm floor so resting spin still decays. */
                    float pen_raw = (cp->penetration > 0.0f) ? cp->penetration : 0.0f;
                    float pen_eff = (pen_raw > 0.0005f) ? pen_raw : 0.0005f;
                    float r_eff = sqrtf(vector3_length_squared(rlev[bi]));
                    if ((!isfinite(r_eff)) || (r_eff < 1e-6f)) {
                        continue;
                    }
                    float patch = sqrtf(fmaxf(r_eff * pen_eff, 0.0f));
                    float patch_cap = 0.3f * r_eff;
                    if (patch > patch_cap) {
                        patch = patch_cap;
                    }
                    /* Rolling part: oppose tangential-plane spin, lever=|r|
                     * (contact radius: M=mu*N*R, standard Coulomb rolling
                     * resistance). Applies to all shapes; boxes in face
                     * contact get tipping damping that settles stacks
                     * (verified: F10 10-stack calm, 6-cube holds). Spin
                     * below uses the Hertz patch. */
                    vector3 spin_n = vector3_scaling(man->normal_vector,
                                                     vector3_dot(bd->angular_velocity, man->normal_vector));
                    vector3 roll_w = vector3_subtraction(bd->angular_velocity, spin_n);
                    float roll_speed = vector3_length(roll_w);
                    if (roll_speed > 0.0001f) {
                        vector3 roll_axis = vector3_scaling(roll_w, 1.0f / roll_speed);
                        float inertia_axis = 1.0f / fmaxf(vector3_dot(
                            roll_axis, math3_multiplication_vector3(rigidbody_effective_inv_inertia(bd), roll_axis)),
                            1e-9f);
                        if (!isfinite(inertia_axis) || inertia_axis <= 0.0f) {
                            continue;
                        }
                        float dw = share * rolling_mu * normal_force * r_eff * dt / inertia_axis;
                        if (!isfinite(dw) || dw < 0.0f) {
                            continue;
                        }
                        if (dw > roll_speed) {
                            dw = roll_speed;
                        }
                        bd->angular_velocity = vector3_subtraction(
                            bd->angular_velocity, vector3_scaling(roll_axis, dw));
                    }
                    /* Spin part: same patch. */
                    float spin_speed = vector3_length(spin_n);
                    if (spin_speed > 0.0001f) {
                        vector3 spin_axis = vector3_scaling(spin_n, 1.0f / spin_speed);
                        float inertia_spin = 1.0f / fmaxf(vector3_dot(
                            spin_axis, math3_multiplication_vector3(rigidbody_effective_inv_inertia(bd), spin_axis)),
                            1e-9f);
                        if (!isfinite(inertia_spin) || inertia_spin <= 0.0f) {
                            continue;
                        }
                        float dw_spin = share * rolling_mu * normal_force * patch * dt / inertia_spin;
                        if (!isfinite(dw_spin) || dw_spin < 0.0f) {
                            continue;
                        }
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
        for (int h = 0; h < contact_hash_size; h++) {
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
    if ((!manifolds) || (manifold_count <= 0) || (!(dt > 0.0f))) {
        return;
    }
    float slop = g_cfg.solver.penetration_slop;
    float beta = g_cfg.solver.bias_factor;
    float max_bias_vel = g_cfg.solver.max_separation_bias;
    /* TRUTH: runtime clamps survive old config files with huge caps.
     * slop 0..5cm, beta 0..1, bias vel <=10 m/s. */
    if (!isfinite(slop) || slop < 0.0f) {
        slop = 0.01f;
    }
    if (slop > 0.05f) {
        slop = 0.05f;
    }
    if (!isfinite(beta) || beta < 0.0f) {
        beta = 0.0f;
    }
    if (beta > 1.0f) {
        beta = 1.0f;
    }
    if (!isfinite(max_bias_vel) || max_bias_vel < 0.0f) {
        max_bias_vel = 5.0f;
    }
    if (max_bias_vel > 10.0f) {
        max_bias_vel = 10.0f;
    }
    const float max_corr = max_bias_vel * dt;
    for (int m = 0; m < manifold_count; m++) {
        collision_data *man = &manifolds[m];
        rigidbody *body_a = man->object_a;
        rigidbody *body_b = man->object_b;
        if ((!body_a) || (!body_b)) {
            continue;
        }
        float inv_a = rigidbody_effective_inv_mass(body_a);
        float inv_b = rigidbody_effective_inv_mass(body_b);
        /* Sleeping bodies hold infinite mass (undisturbed rest) unless the
         * correction is significant, in which case wake first. */
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
        /* TRUTH: kinematic has stored inv!=0 but effective 0. Old
         * !static_state moved kinematics, corrupting prescribed motion.
         * Gate on effective inv (zero for kinematic/sleeping/static). */
        if (inv_a > 0.0f) {
            body_a->position = vector3_subtraction(body_a->position, vector3_scaling(shift, inv_a));
            if (corr > 0.01f) {
                rigidbody_wake(body_a);
            }
        }
        if (inv_b > 0.0f) {
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

int collision_ccd_sweep_clamp_full(rigidbody *bodies, int body_count, float dt, float *time_remaining_out) {
    if ((!bodies) || (body_count <= 0) || (!(dt > 0.0f)) || !isfinite(dt)) {
        return 0;
    }
    /* TRUTH: low-memory NULL path must NOT pre-move. Old code did
     * pos+=v*toi with no remainder recorded, then caller integrated full dt
     * => toi+dt double-count overshoot. Degraded mode: discrete only. */
    bool record_remainder = (time_remaining_out != NULL);
    if (time_remaining_out) {
        for (int k = 0; k < body_count; k++) {
            time_remaining_out[k] = dt;
        }
    }
    /* TRUTH: symmetric two-phase clamp. Old sequential per-body move vs
     * already-moved positions let B tunnel through A (A clamps to contact
     * vs B_old, moves; B sees c<=0 vs A_new, skips, then integrates full dt
     * through A). Phase 1 computes all TOIs vs OLD positions; phase 2 moves
     * all simultaneously. Order-independent. */
    float *best_tois = NULL;
    unsigned char *hit_flags = NULL;
    bool use_heap = body_count > 64;
    float stack_tois[64];
    unsigned char stack_hits[64];
    if (use_heap) {
        best_tois = (float *) malloc((size_t) body_count * sizeof(float));
        hit_flags = (unsigned char *) malloc((size_t) body_count * sizeof(unsigned char));
        if (!best_tois || !hit_flags) {
            free(best_tois);
            free(hit_flags);
            return 0;
        }
    } else {
        best_tois = stack_tois;
        hit_flags = stack_hits;
    }
    for (int i = 0; i < body_count; i++) {
        best_tois[i] = dt;
        hit_flags[i] = 0;
    }
    for (int i = 0; i < body_count; i++) {
        rigidbody *mover = &bodies[i];
        if ((mover->static_state) || (mover->is_sleeping)) {
            continue;
        }
        /* TRUTH P1-19: angular sweep bound. Tip speed |w x r| <= |w|*R sweeps
         * a disc; linear-only bound tunnels for fast spinners. */
        float lin_speed = vector3_length(mover->velocity);
        float ang_speed = vector3_length(mover->angular_velocity);
        float bound_r = broadphase_bounding_radius(mover);
        if ((!isfinite(bound_r)) || (bound_r < 0.0f)) {
            bound_r = 0.0f;
        }
        float sweep_speed = lin_speed + ang_speed * bound_r;
        if (sweep_speed <= 0.0001f) {
            continue;
        }
        float thickness = ccd_min_thickness(mover);
        if ((thickness <= 0.0f) || (!isfinite(thickness))) {
            continue;
        }
        float displacement = sweep_speed * dt;
        /* Plane sweep is O(1) per body: run it whenever the tick motion
         * exceeds slop, so impacts never fall in the gap between slop-band
         * sampling and the safety nets (which would delete the bounce).
         * TRUTH: volume gate must consider obstacle thinness, not mover
         * alone. Large mover (1m) moving 0.5m would skip volumes and tunnel
         * a thin 0.1m wall. Gate on min(mover, 0.2m) so fast large bodies
         * still sweep. */
        bool do_volumes = displacement > fminf(thickness, 0.2f);
        if ((displacement <= g_cfg.solver.penetration_slop) && (!do_volumes)) {
            continue; /* discrete sampling suffices */
        }
        float best_toi = dt;
        bool hit = false;

        /* 1. Floor plane y = 0. Center-velocity TOI only — deliberately.
         * TRUTH correction of an overreach: an earlier revision used the
         * lowest-point velocity (vy - |w|R) so a "diving corner" would clamp.
         * That fired every tick for pure spinners in stable contact (a wheel
         * at 127 rad/s reports vy_low = -6.8 m/s while its center is
         * stationary), teleporting/rotating rolling contact into bounce
         * growth (driven_wheel levitated to y=2.9). Rotation alone cannot
         * translate the center through the plane — corners dipping below it
         * are bounded oscillation the discrete solver re-seats each tick.
         * CCD is for TRANSLATION tunneling; spin is the solver's job. */
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

        /* 2. Volumes: spheres, boxes (static AND dynamic via relative
         * velocity in obstacle frame), cylinders (conservative bounding
         * spheres). TRUTH: dynamic boxes/cylinders must sweep too; ignoring
         * them tunnels box-box at 5-30 m/s (below old thickness gate). */
        if (do_volumes) {
        for (int j = 0; j < body_count; j++) {
            if (j == i) {
                continue;
            }
            rigidbody *other = &bodies[j];
            vector3 other_v =
                ((other->static_state) || (other->is_sleeping)) ? vector3_zero() : other->velocity;
            if (other->type == object_sphere) {
                vector3 dp = vector3_subtraction(other->position, mover->position);
                vector3 dv = vector3_subtraction(other_v, mover->velocity);
                /* TRUTH: non-sphere movers use bounding radius (conservative,
                 * never misses). Old min_thickness underestimated boxes. */
                float rr = (mover->type == object_sphere)
                    ? (mover->radius + other->radius)
                    : (broadphase_bounding_radius(mover) + other->radius);
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
            } else if (other->type == object_cylinder) {
                /* Exact segment-vs-sphere sweep for cylinder obstacles.
                 * Cylinder = segment (axle) + radius. Sweep the mover's bounding
                 * sphere against the cylinder's swept capsule.
                 *
                 * For a moving sphere (or sphere-bounded body) vs static cylinder:
                 *   - The cylinder's axle endpoints sweep spheres of radius r
                 *   - The barrel sweeps a capsule along the relative velocity
                 *   - We solve for the earliest TOI by checking segment-sphere
                 *     and capsule-sphere sweep.
                 *
                 * For a moving cylinder vs static cylinder: both segments sweep.
                 * Full segment-segment sweep is complex; fall back to bounding
                 * sphere for cylinder-vs-cylinder (conservative, never misses).
                 */
                if (mover->type == object_sphere) {
                    /* Sphere vs cylinder: exact segment-sphere sweep. */
                    vector3 ax = other->cached_axes[0];
                    float ax_len = vector3_length(ax);
                    if (ax_len < 1e-6f) {
                        ax = (vector3){1.0f, 0.0f, 0.0f};
                    } else {
                        ax = vector3_scaling(ax, 1.0f / ax_len);
                    }
                    float h = other->cylinder_half_length;
                    float r_cyl = other->radius;
                    float r_sph = mover->radius;
                    float rr = r_cyl + r_sph;

                    /* Cylinder endpoints in world space. */
                    vector3 ep1 = vector3_addition(other->position, vector3_scaling(ax, -h));
                    vector3 ep2 = vector3_addition(other->position, vector3_scaling(ax, h));

                    /* Relative motion. */
                    vector3 dp1 = vector3_subtraction(ep1, mover->position);
                    vector3 dp2 = vector3_subtraction(ep2, mover->position);
                    vector3 dv = vector3_subtraction(other_v, mover->velocity);

                    /* Sweep against both endpoint spheres. */
                    float best_cyl_toi = dt;
                    for (int ep = 0; ep < 2; ep++) {
                        vector3 dp = (ep == 0) ? dp1 : dp2;
                        float a = vector3_dot(dv, dv);
                        float c = vector3_dot(dp, dp) - rr * rr;
                        if ((c <= 0.0f) || (a <= 1e-12f)) continue;
                        float b = 2.0f * vector3_dot(dp, dv);
                        float disc = b * b - 4.0f * a * c;
                        if (disc < 0.0f) continue;
                        float toi = (-b - sqrtf(disc)) / (2.0f * a);
                        if ((toi > 0.0f) && (toi < best_cyl_toi) && (toi <= dt)) {
                            best_cyl_toi = toi;
                        }
                    }

                    /* Sweep against barrel (capsule segment).
                     * Project relative velocity onto plane perpendicular to axle. */
                    float dv_ax = vector3_dot(dv, ax);
                    vector3 dv_perp = vector3_subtraction(dv, vector3_scaling(ax, dv_ax));
                    float dv_perp_len_sq = vector3_length_squared(dv_perp);
                    if (dv_perp_len_sq > 1e-12f) {
                        /* Relative motion has perpendicular component - capsule sweep.
                         * The capsule is the segment extruded along dv_perp.
                         * Find closest approach of sphere to swept capsule. */
                        vector3 dp_mid = vector3_subtraction(other->position, mover->position);
                        float dp_ax = vector3_dot(dp_mid, ax);
                        vector3 dp_perp = vector3_subtraction(dp_mid, vector3_scaling(ax, dp_ax));
                        float dp_perp_len_sq = vector3_length_squared(dp_perp);
                        (void)sqrtf(dv_perp_len_sq); /* dv_perp_len used in debug builds */

                        /* Quadratic for perpendicular distance == rr.
                         * |dp_perp + t*dv_perp|^2 = rr^2 */
                        float a = dv_perp_len_sq;
                        float b = 2.0f * vector3_dot(dp_perp, dv_perp);
                        float c = dp_perp_len_sq - rr * rr;
                        if (a > 1e-12f) {
                            float disc = b * b - 4.0f * a * c;
                            if (disc >= 0.0f) {
                                float toi = (-b - sqrtf(disc)) / (2.0f * a);
                                if ((toi > 0.0f) && (toi < best_cyl_toi) && (toi <= dt)) {
                                    /* Check if contact point is within segment bounds at TOI. */
                                    vector3 rel_pos = vector3_addition(dp_mid, vector3_scaling(dv, toi));
                                    float rel_ax = vector3_dot(rel_pos, ax);
                                    if (fabsf(rel_ax) <= h + rr) {
                                        best_cyl_toi = toi;
                                    }
                                }
                            }
                        }
                        (void)dv_perp_len_sq; /* silence unused in some configs */
                    }

                    if ((best_cyl_toi > 0.0f) && (best_cyl_toi < best_toi)) {
                        best_toi = best_cyl_toi;
                        hit = true;
                    }
                } else {
                    /* Non-sphere mover vs cylinder: conservative bounding sphere sweep.
                     * (Exact segment-segment sweep for cylinder-vs-cylinder is complex;
                     * bounding sphere is conservative and never misses.) */
                    vector3 dp = vector3_subtraction(other->position, mover->position);
                    vector3 dv = vector3_subtraction(other_v, mover->velocity);
                    float rr = broadphase_bounding_radius(mover) + broadphase_bounding_radius(other);
                    float a = vector3_dot(dv, dv);
                    float c = vector3_dot(dp, dp) - rr * rr;
                    if ((c <= 0.0f) || (a <= 1e-12f)) {
                        continue;
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
                }
            } else if (other->type == object_cube) {
                /* Swept sphere-vs-OBB via slab test in box space, with
                 * RELATIVE velocity so dynamic boxes sweep correctly. */
                float sr = (mover->type == object_sphere) ? mover->radius
                                                          : broadphase_bounding_radius(mover);
                vector3 rel = vector3_subtraction(mover->position, other->position);
                vector3 rel_v = vector3_subtraction(mover->velocity, other_v);
                vector3 ax0 = other->cached_axes[0];
                vector3 ax1 = other->cached_axes[1];
                vector3 ax2 = other->cached_axes[2];
                float pl[3] = {vector3_dot(rel, ax0), vector3_dot(rel, ax1), vector3_dot(rel, ax2)};
                float vl[3] = {vector3_dot(rel_v, ax0), vector3_dot(rel_v, ax1),
                               vector3_dot(rel_v, ax2)};
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

        /* Phase 1: record only (no move yet — symmetric two-phase). */
        if (hit && best_toi < dt && best_toi > 0.0f) {
            best_tois[i] = best_toi;
            hit_flags[i] = 1;
        }
    }
    /* Phase 2: apply all clamps simultaneously vs OLD positions.
     * TRUTH: rotate orientation by w*toi too (old code lost toi rotation:
     * total became w*rem not w*dt). Linear pre-move + angular pre-rotate,
     * remainder integration completes both. */
    int clamped = 0;
    if (record_remainder) {
        for (int i = 0; i < body_count; i++) {
            if (!hit_flags[i]) {
                continue;
            }
            rigidbody *mover = &bodies[i];
            if ((mover->static_state) || (mover->is_sleeping)) {
                continue;
            }
            float toi = best_tois[i];
            if (!(toi > 0.0f) || !(toi < dt)) {
                continue;
            }
            mover->position = vector3_addition(mover->position, vector3_scaling(mover->velocity, toi));
            float spin = vector3_length(mover->angular_velocity);
            if (spin > 1e-6f && isfinite(spin)) {
                double half = 0.5 * (double) spin * (double) toi;
                vector4 rotor;
                if (half > -0.5 && half < 0.5) {
                    double s = det_sin_small(half);
                    double c = det_cos_small(half);
                    double inv = 1.0 / (double) spin;
                    rotor = (vector4){(float) c, (float) (mover->angular_velocity.x * inv * s),
                                     (float) (mover->angular_velocity.y * inv * s),
                                     (float) (mover->angular_velocity.z * inv * s)};
                } else {
                    rotor = vector4_from_axis_with_angle(
                        vector3_scaling(mover->angular_velocity, 1.0f / spin), spin * toi);
                }
                mover->orientation = vector4_normalisation(vector4_multiplication(rotor, mover->orientation));
            }
            rigidbody_wake(mover);
            rigidbody_update_axes(mover);
            clamped++;
            float rem = dt - toi;
            time_remaining_out[i] = (rem > 0.0f) ? rem : 0.0f;
        }
    } else {
        /* Degraded NULL mode: no pre-move (would double-count). Count only. */
        for (int i = 0; i < body_count; i++) {
            if (hit_flags[i]) {
                clamped++;
            }
        }
    }
    if (use_heap) {
        free(best_tois);
        free(hit_flags);
    }
    return clamped;
}

int collision_ccd_sweep_clamp(rigidbody *bodies, int body_count, float dt) {
    return collision_ccd_sweep_clamp_full(bodies, body_count, dt, NULL);
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
/* Cylinder narrowphase lives in collision_cylinder.c (extracted to shrink this file).
 * Declarations in physics/collision_cylinder.h, shared proxy below. */
#include "collision_cylinder.h"

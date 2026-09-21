/* GTK4-PREP: zero GUI headers in physics. */
#include "collision_mechanics.h"
#include "collision_cylinder.h"
#include "../core/physics_world.h"
#include "../core/rigidbody.h"
#include "../core/det_math.h"
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

bool collision_dual_sphere(rigidbody *rigidbody_object_a, rigidbody *rigidbody_object_b,
                           collision_data *collision_output_data, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    /* TRUTH: degenerate spheres (NaN/radius<=0) must return false, never a
     * phantom zero-depth contact with an arbitrary +Y normal. */
    if (!isfinite(rigidbody_object_a->radius) || !isfinite(rigidbody_object_b->radius)) {
        return false;
    }
    if (rigidbody_object_a->radius <= 0.0f || rigidbody_object_b->radius <= 0.0f) {
        return false;
    }
    vector3 relative_position_vector = vector3_subtraction(rigidbody_object_b->position, rigidbody_object_a->position);
    float distance_between_centres_squared = vector3_length_squared(relative_position_vector);
    if (!isfinite(distance_between_centres_squared)) {
        return false;
    }
    float total_combined_radius = rigidbody_object_a->radius + rigidbody_object_b->radius;
    /* TRUTH P0-4: slop-band persistence parity with cube/cylinder/floor.
     * Resting spheres at exact contact (dist == r1+r2) must report a
     * zero-depth contact for friction; strict < flickers support at 30Hz.
     * Admit pen >= -slop, clamp negatives to 0 (no bounce, friction only). */
    float slop = C->solver.penetration_slop;
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

bool collision_sphere_cube(rigidbody *sphere, rigidbody *cube, collision_data *collision_output_data,
                           const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    /* TRUTH: degenerate inputs must return false, never phantom contacts.
     * Zero-radius spheres and NaN cube geometry previously emitted
     * slop-band contacts with NaN closest points. */
    if (!isfinite(sphere->radius) || sphere->radius <= 0.0f) {
        return false;
    }
    if (!isfinite(cube->half_extensions.x) || !isfinite(cube->half_extensions.y) ||
        !isfinite(cube->half_extensions.z)) {
        return false;
    }
    vector3 *axes_cube = cube->cached_axes;
    for (int ai = 0; ai < 3; ai++) {
        if (!isfinite(axes_cube[ai].x) || !isfinite(axes_cube[ai].y) || !isfinite(axes_cube[ai].z)) {
            return false;
        }
    }
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
    float slop_sc = C->solver.penetration_slop;
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
                           collision_data *collision_output_data, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
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
    float clip_slop = C->solver.penetration_slop;
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

bool collision_dual_cube(rigidbody *cube_a, rigidbody *cube_b, collision_data *collision_output_data,
                         const mpe_config_t *cfg) {
    /* TRUTH: slop-band parity with sphere/cylinder/floor paths. Strict
     * overlap<0 rejection drops resting pairs sitting at gap<=slop
     * (friction flicker, sleep churn). Admit pen>=-slop everywhere. */
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    float slop_sat = C->solver.penetration_slop;
    if (!isfinite(slop_sat) || slop_sat < 0.0f) {
        slop_sat = 0.0f;
    }
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
        if (overlap < -slop_sat) {
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
            if (overlap < -slop_sat) {
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
        /* TRUTH: slop-band admission (SAT above) can yield minimum_overlap
         * in [-slop,0): clamp to zero-depth (friction persistence only),
         * never hand the solver a negative penetration. */
        cp->penetration = (minimum_overlap > 0.0f) ? minimum_overlap : 0.0f;
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
                    extra_cp->penetration = (minimum_overlap > 0.0f) ? minimum_overlap : 0.0f;
                    collision_output_data->contact_count++;
                }
            }
        }
    } else {
        if (best_axis_index < 3) {
            clip_obb_faces(cube_a, cube_b, best_axis, minimum_overlap, collision_output_data, cfg);
        } else {
            clip_obb_faces(cube_b, cube_a, vector3_scaling(best_axis, -1.0f), minimum_overlap, collision_output_data,
                           cfg);
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

/* Fill a caller-provided static plane proxy body.
     * No thread-local state - caller owns the storage.
     * restitution=1.0 is intentionally neutral: effective bounce is
     * min(body_restitution, 1.0) == body_restitution. */
    void collision_static_plane_body_proxy_fill(rigidbody *out, float plane_y, const mpe_config_t *cfg) {
    rigidbody_initialisation_sphere(out, 1.0f, 0.0f, (vector3){0.0f, plane_y, 0.0f});
    out->static_state = true;
    out->inverse_mass = 0.0f;
    out->inverse_inertia_tensor_local = (math3){{{0}}};
    out->inverse_inertia_system = (math3){{{0}}};
    out->restitution = 1.0f; /* A3_HOTFIX_FLOOR_BOUNCE */
    out->object_id = 0xFFFFFFFFu; /* A3_PATCH_16_FLOOR_MANIFOLD */
    out->object_generation = 1;
    out->body_index = -1; /* Not in world's body array */
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    out->friction_static = C->world.floor_friction_s;
    out->friction_kinetic = C->world.floor_friction_k;
}

bool collision_static_plane_sphere(rigidbody *plane_body, rigidbody *sphere, float plane_y,
                                    collision_data *collision_output_data, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if (sphere->type != object_sphere) {
        return false;
    }

    float lowest_y = sphere->position.y - sphere->radius;
    float penetration = plane_y - lowest_y;

    /* Slop-gated like cube/cylinder paths: resting contact persists as
     * zero-depth (friction without bounce) instead of flickering. */
    if (penetration <= -C->solver.penetration_slop) {
        return false;
    }

    collision_output_data->object_a = sphere;
    collision_output_data->object_b = plane_body;
    collision_output_data->normal_vector = (vector3){0.0f, -1.0f, 0.0f};
    collision_output_data->contact_count = 1;

    contact_point_data *cp = &collision_output_data->contacts[0];
    cp->position = (vector3){sphere->position.x, lowest_y, sphere->position.z};
    cp->penetration = (penetration > 0.0f) ? penetration : 0.0f;

    return true;
}

bool collision_static_plane_cube(rigidbody *plane_body, rigidbody *cube, float plane_y, collision_data *collision_output_data,
                                 const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
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
                if (penetration > -C->solver.penetration_slop) {
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

    // collision_static_plane_body_proxy_fill removed

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

bool collision_static_plane_body(rigidbody *plane_body, rigidbody *body, float plane_y,
                                 collision_data *collision_output_data, const mpe_config_t *cfg) {
    collision_output_data->object_a = body;
    collision_output_data->object_b = plane_body;
    collision_output_data->normal_vector = (vector3){0.0f, -1.0f, 0.0f};
    collision_output_data->contact_count = 0;
    if (body->type == object_cylinder) {return collision_static_plane_cylinder(plane_body, body, plane_y, collision_output_data, cfg);}
    if (body->type == object_sphere) {
        return collision_static_plane_sphere(plane_body, body, plane_y, collision_output_data, cfg);
    }
    if (body->type == object_cube) {
        return collision_static_plane_cube(plane_body, body, plane_y, collision_output_data, cfg);
    }
    return false;
}

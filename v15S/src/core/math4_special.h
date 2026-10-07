#ifndef math4_h
#define math4_h
/* STORAGE CONVENTION (read before indexing): math4 is COLUMN-major,
 * m[col][row]; translation lives in m[3][0..2]. math3 (math3d.h) is
 * ROW-major. Do not mix indexing across the two types. */
#include "math3d.h"
typedef struct {float matrix [4][4];} math4;
// Zero Init
static inline math4 math4_init () {
    math4 result_matrix = {{{0}}};
    return result_matrix;
} // Identity Matrices
static inline math4 math4_identity () {
    math4 result_matrix = {{{0}}};
    result_matrix.matrix [0][0] = 1.0f;
    result_matrix.matrix [1][1] = 1.0f;
    result_matrix.matrix [2][2] = 1.0f;
    result_matrix.matrix [3][3] = 1.0f;
    return result_matrix;
} // Viewing Perspective
static inline math4 math4_look_view (vector3 camera_position, vector3 camera_front, vector3 camera_up) {
    math4 result_matrix = math4_identity ();
    vector3 forward_vector = vector3_normalisation (camera_front);
    /* Degenerate-guard: forward invalid or parallel to up would produce NaN
     * side vector. Fall back to -Z forward and Y up. */
    if (vector3_length_squared (forward_vector) < 1e-12f) {
        forward_vector = (vector3) {0.0f, 0.0f, -1.0f};
    }
    vector3 side_raw = vector3_cross (forward_vector, camera_up);
    if (vector3_length_squared (side_raw) < 1e-12f) {
        /* camera_up parallel to forward (e.g. looking straight up): pick
         * an orthogonal reference so the basis stays valid. */
        vector3 alt_up = (fabsf (forward_vector.y) < 0.99f) ? (vector3) {0.0f, 1.0f, 0.0f} : (vector3) {1.0f, 0.0f, 0.0f};
        side_raw = vector3_cross (forward_vector, alt_up);
    }
    vector3 side_vector = vector3_normalisation (side_raw);
    /* TRUTH: second degeneracy must be re-checked. If still degenerate
     * (NaN input), side is zero -> zero rows silently. Fall back to identity
     * basis instead. */
    if (vector3_length_squared (side_vector) < 1e-12f) {
        side_vector = (vector3) {1.0f, 0.0f, 0.0f};
        forward_vector = (vector3) {0.0f, 0.0f, -1.0f};
    }
    /* Re-orthogonalize: up = normalize(cross(side, forward)). If that
     * degenerates (NaN input), fall back to a vector orthogonal to side. */
    vector3 up_vector = vector3_cross (side_vector, forward_vector);
    float up_len_sq = vector3_length_squared (up_vector);
    if ((!isfinite (up_len_sq)) || (up_len_sq < 1e-12f)) {
        vector3 ref = (fabsf (side_vector.y) < 0.99f) ? (vector3) {0.0f, 1.0f, 0.0f} : (vector3) {1.0f, 0.0f, 0.0f};
        up_vector = vector3_cross (ref, side_vector);
        float rl2 = vector3_length_squared (up_vector);
        if ((!isfinite (rl2)) || (rl2 < 1e-12f)) {
            up_vector = (vector3) {0.0f, 1.0f, 0.0f};
        } else {
            up_vector = vector3_scaling (up_vector, 1.0f / sqrtf (rl2));
        }
    } else {
        up_vector = vector3_scaling (up_vector, 1.0f / sqrtf (up_len_sq));
    }
    result_matrix.matrix [0][0] = side_vector.x;
    result_matrix.matrix [1][0] = side_vector.y;
    result_matrix.matrix [2][0] = side_vector.z;
    result_matrix.matrix [0][1] = up_vector.x;
    result_matrix.matrix [1][1] = up_vector.y;
    result_matrix.matrix [2][1] = up_vector.z;
    result_matrix.matrix [0][2] = -forward_vector.x;
    result_matrix.matrix [1][2] = -forward_vector.y;
    result_matrix.matrix [2][2] = -forward_vector.z;
    result_matrix.matrix [3][0] = -vector3_dot (side_vector, camera_position);
    result_matrix.matrix [3][1] = -vector3_dot (up_vector, camera_position);
    result_matrix.matrix [3][2] = vector3_dot (forward_vector, camera_position);
    return result_matrix;
} // Perspective Projection Matrices
static inline math4 math4_perspective_fov (float field_of_view, float aspect_ratio, float near_plane, float far_plane) {
    math4 result_matrix = {{{0}}};
    /* Clamp degenerate inputs so a bad config can never produce NaN/Inf. */
    if (!(field_of_view > 0.01f && field_of_view < 3.14159265f)) {
        field_of_view = 45.0f * 3.14159265358979323846f / 180.0f;
    }
    if (!(aspect_ratio > 1e-6f && aspect_ratio < 1e6f)) {
        aspect_ratio = 16.0f / 9.0f;
    }
    if (!(near_plane > 1e-6f)) {
        near_plane = 0.1f;
    }
    if (!(far_plane > near_plane + 1e-4f)) {
        far_plane = near_plane + 1000.0f;
    }
    float focal_length = 1.0f / tanf (field_of_view / 2.0f);
    result_matrix.matrix [0][0] = focal_length / aspect_ratio;
    result_matrix.matrix [1][1] = focal_length;
    result_matrix.matrix [2][2] = (far_plane + near_plane) / (near_plane - far_plane);
    result_matrix.matrix [3][2] = (2.0f * far_plane * near_plane) / (near_plane - far_plane);
    result_matrix.matrix [2][3] = -1.0f;
    return result_matrix;
} // Translational Motion Matrix
static inline math4 math4_translation (vector3 translation_vector) {
    math4 result_matrix = math4_identity ();
    result_matrix.matrix [3][0] = translation_vector.x;
    result_matrix.matrix [3][1] = translation_vector.y;
    result_matrix.matrix [3][2] = translation_vector.z;
    return result_matrix;
} // Scaling Matrix
static inline math4 math4_scaling (vector3 scale_vector) {
    math4 result_matrix = math4_identity ();
    result_matrix.matrix [0][0] = scale_vector.x;
    result_matrix.matrix [1][1] = scale_vector.y;
    result_matrix.matrix [2][2] = scale_vector.z;
    return result_matrix;
} // Multiplication
static inline math4 math4_multiplication (math4 matrix_a, math4 matrix_b) {
    math4 result_matrix = {{{0}}};
    for (int column_index = 0; column_index < 4; column_index++) {
        for (int row_index = 0; row_index < 4; row_index++) {
            result_matrix.matrix [column_index][row_index] =
                (matrix_a.matrix [0][row_index] * matrix_b.matrix [column_index][0]) +
                (matrix_a.matrix [1][row_index] * matrix_b.matrix [column_index][1]) +
                (matrix_a.matrix [2][row_index] * matrix_b.matrix [column_index][2]) +
                (matrix_a.matrix [3][row_index] * matrix_b.matrix [column_index][3]);
        }
    }
    return result_matrix;
} // Quaternion to Matrix Interface
/* TRUTH: column-major storage (math3 is row-major). Normalize input like
 * vector4_to_math3 so non-unit quats never scale the render matrix. */
static inline math4 vector4_to_math4 (vector4 quaternion) {
    /* Same degenerate rule as vector4_to_math3 (|q|>1e-6 normalize, else
     * identity) so physics and render never disagree on quats.
     * Double accumulation matches vector4_to_math3 after its fix. */
    double n2_d = (double) quaternion.w * (double) quaternion.w + (double) quaternion.x * (double) quaternion.x +
                  (double) quaternion.y * (double) quaternion.y + (double) quaternion.z * (double) quaternion.z;
    if (isfinite (n2_d) && (n2_d > 1e-12)) {
        double inv_d = 1.0 / sqrt (n2_d);
        float inv = (float) inv_d;
        quaternion.w *= inv;
        quaternion.x *= inv;
        quaternion.y *= inv;
        quaternion.z *= inv;
    } else {
        quaternion = vector4_identity ();
    }
    math4 result_matrix = math4_identity ();
    float x_double = quaternion.x + quaternion.x, y_double = quaternion.y + quaternion.y,
          z_double = quaternion.z + quaternion.z;
    float x_x = quaternion.x * x_double, x_y = quaternion.x * y_double, x_z = quaternion.x * z_double;
    float y_y = quaternion.y * y_double, y_z = quaternion.y * z_double, z_z = quaternion.z * z_double;
    float w_x = quaternion.w * x_double, w_y = quaternion.w * y_double, w_z = quaternion.w * z_double;
    result_matrix.matrix [0][0] = 1.0f - (y_y + z_z);
    result_matrix.matrix [1][0] = x_y - w_z;
    result_matrix.matrix [2][0] = x_z + w_y;
    result_matrix.matrix [0][1] = x_y + w_z;
    result_matrix.matrix [1][1] = 1.0f - (x_x + z_z);
    result_matrix.matrix [2][1] = y_z - w_x;
    result_matrix.matrix [0][2] = x_z - w_y;
    result_matrix.matrix [1][2] = y_z + w_x;
    result_matrix.matrix [2][2] = 1.0f - (x_x + y_y);
    return result_matrix;
} // GPU flat array interface
/* TRUTH: NULL/short buffer was heap overflow. Require non-NULL; caller owns 16 floats. */
static inline void math4_to_flat_array (math4 matrix, float *output_array) {
    if (!output_array) {
        return;
    }
    for (int column_index = 0; column_index < 4; column_index++) {
        for (int row_index = 0; row_index < 4; row_index++) {
            output_array [column_index * 4 + row_index] = matrix.matrix [column_index][row_index];
        }
    }
} /* ---------------------------------------------------------------------------
 * Frustum culling (Gribb/Hartmann), extracted so the shipped renderer and a
 * test exercise the SAME code.
 *
 * DESPOT-2026-09-29: this logic lived inline inside render_scene_current, a GL
 * function no headless test can call, so the culler the engine actually runs
 * was untested -- the legacy frustum test re-implemented plane extraction
 * locally and linked no engine objects, and the canonical case projected a
 * single point. A regression in the real culler would have been invisible.
 * The renderer now calls these two functions.
 *
 * `planes` is filled with six INWARD-facing planes in {a,b,c,d} order, so
 * the signed distance of p is a*x + b*y + c*z + d.
 *
 * DESPOT-2026-09-29: the first version of this extraction kept the original
 * inline code's packing, which stored the vector4 as (d,a,b,c) and read it
 * back as .x*x + .y*y + .z*z + .w. Copying that packing while writing a
 * (a,b,c,d) consumer transposed the planes: the far plane came out with a
 * zero z coefficient and a huge x coefficient, so EVERY point was culled.
 * mpe_t_frustum_culler caught it immediately. The packing is now the natural
 * (a,b,c,d) and no consumer has to remember a reordering.
 *
 * The extraction transposes the column-major view-projection into rows first:
 * m[col][row] means row r is {m[0][r], m[1][r], m[2][r], m[3][r]}.
 * ------------------------------------------------------------------------ */
static inline void math4_frustum_planes (math4 view_projection, float planes [6][4]) {
    float row0 [4] = {view_projection.matrix [0][0], view_projection.matrix [1][0], view_projection.matrix [2][0],
                     view_projection.matrix [3][0]};
    float row1 [4] = {view_projection.matrix [0][1], view_projection.matrix [1][1], view_projection.matrix [2][1],
                     view_projection.matrix [3][1]};
    float row2 [4] = {view_projection.matrix [0][2], view_projection.matrix [1][2], view_projection.matrix [2][2],
                     view_projection.matrix [3][2]};
    float row3 [4] = {view_projection.matrix [0][3], view_projection.matrix [1][3], view_projection.matrix [2][3],
                     view_projection.matrix [3][3]};
    const float combos [6][4] = {
        {row3 [0] + row0 [0], row3 [1] + row0 [1], row3 [2] + row0 [2], row3 [3] + row0 [3]},
        {row3 [0] - row0 [0], row3 [1] - row0 [1], row3 [2] - row0 [2], row3 [3] - row0 [3]},
        {row3 [0] + row1 [0], row3 [1] + row1 [1], row3 [2] + row1 [2], row3 [3] + row1 [3]},
        {row3 [0] - row1 [0], row3 [1] - row1 [1], row3 [2] - row1 [2], row3 [3] - row1 [3]},
        {row3 [0] + row2 [0], row3 [1] + row2 [1], row3 [2] + row2 [2], row3 [3] + row2 [3]},
        {row3 [0] - row2 [0], row3 [1] - row2 [1], row3 [2] - row2 [2], row3 [3] - row2 [3]},
    };
    for (int p = 0; p < 6; p++) {
        float len = sqrtf (combos [p][0] * combos [p][0] + combos [p][1] * combos [p][1] + combos [p][2] * combos [p][2]);
        if (len < 0.000001f) {
            len = 1.0f;
        }
        planes [p][0] = combos [p][0] / len; /* a */
        planes [p][1] = combos [p][1] / len; /* b */
        planes [p][2] = combos [p][2] / len; /* c */
        planes [p][3] = combos [p][3] / len; /* d */
    }
} /* Conservative sphere-vs-frustum: a sphere is culled only if it is fully
 * outside one plane by more than its own radius, so a sphere straddling a
 * plane is always kept. Returns 1 = visible, 0 = culled. */
static inline int math4_frustum_sphere_visible (const float planes [6][4], float cx, float cy, float cz, float radius) {
    for (int p = 0; p < 6; p++) {
        float dist = planes [p][0] * cx + planes [p][1] * cy + planes [p][2] * cz + planes [p][3];
        if (dist < -radius) {
            return 0;
        }
    }
    return 1;
}
#endif

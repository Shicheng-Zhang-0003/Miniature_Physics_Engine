#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <float.h>
//File library file definition
#ifndef math3d_h
#define math3d_h
//Pi definition
#ifndef math_pi
#define math_pi 3.14159265358979323846f
#endif
//Define Radians and Degree Calculation converter
#define degrad (math_pi / 180.0f)
#define raddeg (180.0f / math_pi)
#define math_epsilon 0.000001f
//Structures for use as typedefs
//Vector in 3D for objects in motion
typedef struct {
    float x, y, z;
} vector3;
//3 ^ 3 matrix for computing Inertia tensoring
typedef struct {
    float matrix[3][3];
} math3;
//4D axial rotational matrix motion (w + xi + yj + zk)
typedef struct {
    float w, x, y, z;
} vector4;
/* Canonical aliases: quaternions are vector4 storage, but deserve a distinct
 * name for readability. mat3 is the descriptive alias for math3. */
typedef vector4 quaternion;
typedef math3 mat3;
//Functions for computing different vector3
static inline vector3 vector3_new(float x_coordinate, float y_coordinate, float z_coordinate) {
    return (vector3){x_coordinate, y_coordinate, z_coordinate};
}
static inline vector3 vector3_zero(void) {
    return (vector3){0.0f, 0.0f, 0.0f};
}
static inline vector3 vector3_addition(vector3 vector_a, vector3 vector_b) {
    return (vector3){vector_a.x + vector_b.x, vector_a.y + vector_b.y, vector_a.z + vector_b.z};
}
static inline vector3 vector3_subtraction(vector3 vector_a, vector3 vector_b) {
    return (vector3){vector_a.x - vector_b.x, vector_a.y - vector_b.y, vector_a.z - vector_b.z};
}
static inline vector3 vector3_scaling(vector3 vector, float scale_factor) {
    return (vector3){vector.x * scale_factor, vector.y * scale_factor, vector.z * scale_factor};
}
//Dot: work and projection of vectors
static inline float vector3_dot(vector3 vector_a, vector3 vector_b) {
    return (float) (vector_a.x * vector_b.x + vector_a.y * vector_b.y + vector_a.z * vector_b.z);
}
//Cross: Torque conversion and computation
static inline vector3 vector3_cross(vector3 vector_a, vector3 vector_b) {
    return (vector3){(vector_a.y * vector_b.z - vector_a.z * vector_b.y),
                     (vector_a.z * vector_b.x - vector_a.x * vector_b.z),
                     (vector_a.x * vector_b.y - vector_a.y * vector_b.x)};
}
static inline float vector3_length_squared(vector3 vector) {
    if (!isfinite(vector.x) || !isfinite(vector.y) || !isfinite(vector.z)) {
        return INFINITY;
    }
    double dx = (double) vector.x, dy = (double) vector.y, dz = (double) vector.z;
    double s = dx * dx + dy * dy + dz * dz;
    if (s > (double) FLT_MAX) {
        return INFINITY;
    }
    return (float) s;
}
static inline float vector3_length(vector3 vector) {
    float s2 = vector3_length_squared(vector);
    if (!isfinite(s2)) {
        return INFINITY;
    }
    return sqrtf(s2);
}
static inline vector3 vector3_normalisation(vector3 vector) {
    if (!isfinite(vector.x) || !isfinite(vector.y) || !isfinite(vector.z)) {
        return vector3_zero();
    }
    float length = vector3_length(vector);
    if ((!isfinite(length)) || (length < math_epsilon)) {
        return vector3_zero();
    }
    return vector3_scaling(vector, 1.0f / length);
} //Quarternion (4D) Functions
//orientation in rotational w-axis w/o gimbal in any axis
static inline vector4 vector4_identity() {
    return (vector4){1.0f, 0.0f, 0.0f, 0.0f};
}
static inline vector4 vector4_normalisation(vector4 quaternion) {
    if (!isfinite(quaternion.w) || !isfinite(quaternion.x) || !isfinite(quaternion.y) ||
        !isfinite(quaternion.z)) {
        return vector4_identity();
    }
    /* TRUTH: accumulate in double. float accumulation overflows to Inf for
     * large components (1e20^2), returning identity instead of direction. */
    double w = (double) quaternion.w, x = (double) quaternion.x, y = (double) quaternion.y,
           z = (double) quaternion.z;
    double n2 = w * w + x * x + y * y + z * z;
    if (!isfinite(n2) || n2 < (double) math_epsilon * (double) math_epsilon) {
        return vector4_identity();
    }
    float inverse_length = (float) (1.0 / sqrt(n2));
    return (vector4){quaternion.w * inverse_length, quaternion.x * inverse_length, quaternion.y * inverse_length,
                     quaternion.z * inverse_length};
} //Multiplication of two 4D matrices at once (combinatoric rotational motion)
static inline vector4 vector4_multiplication(vector4 quaternion_a, vector4 quaternion_b) {
    return (vector4){quaternion_a.w * quaternion_b.w - quaternion_a.x * quaternion_b.x -
                         quaternion_a.y * quaternion_b.y - quaternion_a.z * quaternion_b.z,
                     quaternion_a.w * quaternion_b.x + quaternion_a.x * quaternion_b.w +
                         quaternion_a.y * quaternion_b.z - quaternion_a.z * quaternion_b.y,
                     quaternion_a.w * quaternion_b.y - quaternion_a.x * quaternion_b.z +
                         quaternion_a.y * quaternion_b.w + quaternion_a.z * quaternion_b.x,
                     quaternion_a.w * quaternion_b.z + quaternion_a.x * quaternion_b.y -
                         quaternion_a.y * quaternion_b.x + quaternion_a.z * quaternion_b.w};
} //Rotate a 3D vector by a 4D rotational matrix
//v_a = q * v * q_conjugation
static inline vector3 vector4_rotate_to_vector3(vector4 quaternion, vector3 vector) {
    /* TRUTH: rotation formula requires a unit quaternion; non-unit scales
     * the result by |q|^2 (energy/momentum error). Normalize (cheap). */
    quaternion = vector4_normalisation(quaternion);
    //Nominal formula for crossing 3D to 4D axial
    vector3 quaternion_vector = {quaternion.x, quaternion.y, quaternion.z};
    vector3 temp_cross = vector3_cross(quaternion_vector, vector);
    //Scale by a factor of 2
    temp_cross = vector3_scaling(temp_cross, 2.0f);
    vector3 cross_result = vector3_cross(quaternion_vector, temp_cross);
    //Extrapolate and enhance to w axis
    vector3 w_axis_scaled = vector3_scaling(temp_cross, quaternion.w);
    return vector3_addition(vector, vector3_addition(w_axis_scaled, cross_result));
} //Rotation from the Axis with angular orientation
/* TRUTH: degenerate axis can never define a rotation. Old code returned
 * {cos(h),0,0,0} with |q|!=1 for angle!=0, scaling R by |q|^2. */
static inline vector4 vector4_from_axis_with_angle(vector3 rotation_axis, float angle_radians) {
    /* COLD PATH ONLY: uses libm sinf/cosf, which is not bit-deterministic
     * across platforms. Per-tick rotors must use det_sin/det_cos
     * (see rigidbody.c rotor); this helper is for setup/tests only. */
    if (!isfinite(angle_radians)) {
        return vector4_identity();
    }
    /* TRUTH: accumulate axis length in double; float overflows for huge
     * axis components (1e20 -> Inf -> identity instead of the axis). */
    double axis_len_sq_d = (double) rotation_axis.x * (double) rotation_axis.x +
                           (double) rotation_axis.y * (double) rotation_axis.y +
                           (double) rotation_axis.z * (double) rotation_axis.z;
    if ((!isfinite(axis_len_sq_d)) || (axis_len_sq_d < (double) math_epsilon * (double) math_epsilon)) {
        return vector4_identity();
    }
    float half_angle = angle_radians * 0.5f;
    if (!isfinite(half_angle)) {
        return vector4_identity();
    }
    float sine_half_angle = sinf(half_angle);
    vector3 normalized_axis = vector3_normalisation(rotation_axis);
    float nlen_sq = normalized_axis.x * normalized_axis.x + normalized_axis.y * normalized_axis.y +
                    normalized_axis.z * normalized_axis.z;
    if ((!isfinite(nlen_sq)) || (nlen_sq < 0.5f)) {
        return vector4_identity();
    }
    return (vector4){cosf(half_angle), normalized_axis.x * sine_half_angle, normalized_axis.y * sine_half_angle,
                     normalized_axis.z * sine_half_angle};
} //3 ^ 3 matrix Functions
static inline math3 math3_identity() {
    math3 result_matrix = {{{0}}};
    result_matrix.matrix[0][0] = 1.0f;
    result_matrix.matrix[1][1] = 1.0f;
    result_matrix.matrix[2][2] = 1.0f;
    return result_matrix;
} //Multiply specific matrix by a existing vector
static inline vector3 math3_multiplication_vector3(math3 matrix, vector3 vector) {
    return (vector3){matrix.matrix[0][0] * vector.x + matrix.matrix[0][1] * vector.y + matrix.matrix[0][2] * vector.z,
                     matrix.matrix[1][0] * vector.x + matrix.matrix[1][1] * vector.y + matrix.matrix[1][2] * vector.z,
                     matrix.matrix[2][0] * vector.x + matrix.matrix[2][1] * vector.y + matrix.matrix[2][2] * vector.z};
} //Convert 4D to rotational matrix (Inertia Tensor rotations)
//I_total = R * I_local * * R_transposed
/* TRUTH: callers must pass unit quats. Sanitize normalizes drift >1e-6, but
 * tick-internal drift still scales R by |q|^2. Normalize here (cheap, exact)
 * so a non-unit quat can never inject energy. Row-major. */
static inline math3 vector4_to_math3(vector4 quaternion) {
    /* TRUTH: normalize unconditionally. Threshold matches vector4_to_math4
     * (|q|>1e-6); below that the direction is noise -> identity. Physics and
     * render must agree bit-for-bit on degenerate quats.
     * Accumulate n2 in double (matches vector4_normalisation): float
     * accumulation overflows to Inf for ~1e20 components, diverging from
     * the double path at extremes. */
    double n2_d = (double) quaternion.w * (double) quaternion.w + (double) quaternion.x * (double) quaternion.x +
                  (double) quaternion.y * (double) quaternion.y + (double) quaternion.z * (double) quaternion.z;
    float n2 = (n2_d > (double) FLT_MAX) ? INFINITY : (float) n2_d;
    if (isfinite(n2) && (n2 > 1e-12f)) {
        float inv = 1.0f / sqrtf(n2);
        quaternion.w *= inv;
        quaternion.x *= inv;
        quaternion.y *= inv;
        quaternion.z *= inv;
    } else {
        quaternion = vector4_identity();
    }
    math3 result_matrix;
    //Defining actual plug in values
    float x_double = quaternion.x + quaternion.x, y_double = quaternion.y + quaternion.y,
          z_double = quaternion.z + quaternion.z;
    float x_x = quaternion.x * x_double, x_y = quaternion.x * y_double, x_z = quaternion.x * z_double;
    float y_y = quaternion.y * y_double, y_z = quaternion.y * z_double, z_z = quaternion.z * z_double;
    float w_x = quaternion.w * x_double, w_y = quaternion.w * y_double, w_z = quaternion.w * z_double;
    //Affix to math3 format
    result_matrix.matrix[0][0] = 1.0f - (y_y + z_z), result_matrix.matrix[0][1] = x_y - w_z,
    result_matrix.matrix[0][2] = x_z + w_y;
    result_matrix.matrix[1][0] = x_y + w_z, result_matrix.matrix[1][1] = 1.0f - (x_x + z_z),
    result_matrix.matrix[1][2] = y_z - w_x;
    result_matrix.matrix[2][0] = x_z - w_y, result_matrix.matrix[2][1] = y_z + w_x,
    result_matrix.matrix[2][2] = 1.0f - (x_x + y_y);
    return result_matrix;
} //Matrix Multiplication
static inline math3 math3_multiplication(math3 matrix_a, math3 matrix_b) {
    math3 result_matrix = {{{0}}};
    for (int row_index = 0; row_index < 3; row_index++) {
        for (int column_index = 0; column_index < 3; column_index++) {
            result_matrix.matrix[row_index][column_index] =
                (matrix_a.matrix[row_index][0] * matrix_b.matrix[0][column_index]) +
                (matrix_a.matrix[row_index][1] * matrix_b.matrix[1][column_index]) +
                (matrix_a.matrix[row_index][2] * matrix_b.matrix[2][column_index]);
        }
    }
    return result_matrix;
}
static inline math3 math3_transposition(math3 matrix) {
    math3 result_matrix;
    for (int row_index = 0; row_index < 3; row_index++) {
        for (int column_index = 0; column_index < 3; column_index++) {
            result_matrix.matrix[row_index][column_index] = matrix.matrix[column_index][row_index];
        }
    }
    return result_matrix;
} //Matrix inversion (3 ^ 3 specific)
// Angular Constraint Calculation (change_p = J * M ^ -1 * J_transposed)
static inline math3 math3_inverse(math3 matrix) {
    /* TRUTH: accumulate in double to avoid frob overflow (1e20^2 -> Inf). */
    double frob_sq_d = 0.0;
    for (int _r = 0; _r < 3; _r++)
        for (int _c = 0; _c < 3; _c++) {
            double v = (double) matrix.matrix[_r][_c];
            if (!isfinite(v)) {
                math3 nan_out = {{{0.0f}}};
                return nan_out;
            }
            frob_sq_d += v * v;
        }
    float frob_sq = (frob_sq_d > (double) FLT_MAX) ? INFINITY : (float) frob_sq_d;
    double det_d = (double) matrix.matrix[0][0] *
                       ((double) matrix.matrix[1][1] * matrix.matrix[2][2] -
                        (double) matrix.matrix[2][1] * matrix.matrix[1][2]) -
                   (double) matrix.matrix[0][1] *
                       ((double) matrix.matrix[1][0] * matrix.matrix[2][2] -
                        (double) matrix.matrix[1][2] * matrix.matrix[2][0]) +
                   (double) matrix.matrix[0][2] *
                       ((double) matrix.matrix[1][0] * matrix.matrix[2][1] -
                        (double) matrix.matrix[1][1] * matrix.matrix[2][0]);
    float determinant =
        (det_d > (double) FLT_MAX || det_d < -(double) FLT_MAX) ? ((det_d > 0) ? INFINITY : -INFINITY) : (float) det_d;
    /* Scale-invariant singularity test: |det| / ||M||_F^3 < 1e-12.
     * For M = s*M0: det ~ s^3, ||M||_F^3 ~ s^3, ratio is constant. */
    float frob_norm_cubed = frob_sq * sqrtf(fmaxf(frob_sq, 1e-24f));
    float eps = 1e-12f * frob_norm_cubed;
    if (frob_sq <= 0.0f) {
        eps = 1e-24f;
    }
    if ((!isfinite(determinant)) || (fabsf(determinant) < eps)) {
        /* TRUTH: adjugate of a singular matrix is zero everywhere, which
         * locks ALL rotation even when only one axis is degenerate
         * (needle cylinder Ixx=0, Iyy=Izz=a). If the matrix is (near-)
         * diagonal, invert per-axis: 0 -> 0 (that axis locked), others
         * exact. Otherwise true singularity -> zero.
         * LIMIT: diagonal fallback is valid only for axis-aligned local
         * inertia. A rotated singular world matrix (large off-diagonals)
         * returns zero (all locked). World inverse must therefore use
         * R*inv_local*R^T, never math3_inverse(world). Local inertia is
         * always diagonal, so the fallback covers every live call. */
        float off = fabsf(matrix.matrix[0][1]) + fabsf(matrix.matrix[0][2]) + fabsf(matrix.matrix[1][0]) +
                    fabsf(matrix.matrix[1][2]) + fabsf(matrix.matrix[2][0]) + fabsf(matrix.matrix[2][1]);
        float diag_scale = fabsf(matrix.matrix[0][0]) + fabsf(matrix.matrix[1][1]) + fabsf(matrix.matrix[2][2]);
        if ((diag_scale > 0.0f) && isfinite(diag_scale) && (off <= 1e-6f * diag_scale)) {
            math3 d = {{{0.0f}}};
            /* Minimum diagonal for inversion: prevents Inf from 1/0 for needle cylinders.
             * For truly zero inertia (locked axis), inverse is 0 (infinite mass along that axis).
             * For near-zero, clamp to prevent numerical explosion while allowing rotation. */
            const float min_diag = 1e-12f * fmaxf(fmaxf(fabsf(matrix.matrix[0][0]), fabsf(matrix.matrix[1][1])),
                                                 fabsf(matrix.matrix[2][2]));
            float d0 = fabsf(matrix.matrix[0][0]) > min_diag ? matrix.matrix[0][0] : 0.0f;
            float d1 = fabsf(matrix.matrix[1][1]) > min_diag ? matrix.matrix[1][1] : 0.0f;
            float d2 = fabsf(matrix.matrix[2][2]) > min_diag ? matrix.matrix[2][2] : 0.0f;
            d.matrix[0][0] = (d0 != 0.0f) ? (1.0f / d0) : 0.0f;
            d.matrix[1][1] = (d1 != 0.0f) ? (1.0f / d1) : 0.0f;
            d.matrix[2][2] = (d2 != 0.0f) ? (1.0f / d2) : 0.0f;
            if (isfinite(d.matrix[0][0]) && isfinite(d.matrix[1][1]) && isfinite(d.matrix[2][2])) {
                return d;
            }
        }
        math3 singular_matrix = {{{0.0f}}};
        return singular_matrix;
    }
    /* TRUTH: cofactors in double. float cofactors overflow/lose precision
     * for large M (physics range I~6.6e9 gives cofactors ~4e19, safe only
     * because mass/radius caps hold; double removes the fragility). */
    double inverse_determinant = 1.0 / det_d;
    math3 result_matrix;
    //n ~= {0, 2}
    //[0][n]
    result_matrix.matrix[0][0] = (float) (((double) matrix.matrix[1][1] * (double) matrix.matrix[2][2] -
                                            (double) matrix.matrix[2][1] * (double) matrix.matrix[1][2]) *
                                           inverse_determinant);
    result_matrix.matrix[0][1] = (float) (((double) matrix.matrix[0][2] * (double) matrix.matrix[2][1] -
                                            (double) matrix.matrix[0][1] * (double) matrix.matrix[2][2]) *
                                           inverse_determinant);
    result_matrix.matrix[0][2] = (float) (((double) matrix.matrix[0][1] * (double) matrix.matrix[1][2] -
                                            (double) matrix.matrix[0][2] * (double) matrix.matrix[1][1]) *
                                           inverse_determinant);
    //[1][n]
    result_matrix.matrix[1][0] = (float) (((double) matrix.matrix[1][2] * (double) matrix.matrix[2][0] -
                                            (double) matrix.matrix[1][0] * (double) matrix.matrix[2][2]) *
                                           inverse_determinant);
    result_matrix.matrix[1][1] = (float) (((double) matrix.matrix[0][0] * (double) matrix.matrix[2][2] -
                                            (double) matrix.matrix[0][2] * (double) matrix.matrix[2][0]) *
                                           inverse_determinant);
    result_matrix.matrix[1][2] = (float) (((double) matrix.matrix[1][0] * (double) matrix.matrix[0][2] -
                                            (double) matrix.matrix[0][0] * (double) matrix.matrix[1][2]) *
                                           inverse_determinant);
    //[2][n]
    result_matrix.matrix[2][0] = (float) (((double) matrix.matrix[1][0] * (double) matrix.matrix[2][1] -
                                            (double) matrix.matrix[2][0] * (double) matrix.matrix[1][1]) *
                                           inverse_determinant);
    result_matrix.matrix[2][1] = (float) (((double) matrix.matrix[2][0] * (double) matrix.matrix[0][1] -
                                            (double) matrix.matrix[0][0] * (double) matrix.matrix[2][1]) *
                                           inverse_determinant);
    result_matrix.matrix[2][2] = (float) (((double) matrix.matrix[0][0] * (double) matrix.matrix[1][1] -
                                            (double) matrix.matrix[1][0] * (double) matrix.matrix[0][1]) *
                                           inverse_determinant);
    return result_matrix;
}
#endif //math3d_h

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
//File library file definition
#ifndef math3D_h
#define math3D_h
//Pi definition
#ifndef math_pi
#define math_pi 3.14159265358979323846
#endif
//Define Radians and Degree Calculation converter
#define degrad (math_pi / 180.0)
#define raddeg (180.0 / math_pi)
#define math_epsilon 0.000001
//Structures for use as typedefs
//Vector in 3D for objects in motion
typedef struct { float x, y, z; } vector3;
//3 ^ 3 matrix for computing Inertia tensoring
typedef struct { float matrix [3][3]; } math3;
//4D axial rotational matrix motion (w + xi + yj + zk)
typedef struct { float w, x, y, z; } vector4;
//Functions for computing different vector3
static inline vector3 vector3_new (float x_coordinate, float y_coordinate, float z_coordinate) {return (vector3) {x_coordinate, y_coordinate, z_coordinate};}
static inline vector3 vector3_zero (void) {return (vector3) {0.0, 0.0, 0.0};}
static inline vector3 vector3_addition (vector3 vector_a, vector3 vector_b) {return (vector3) {vector_a.x + vector_b.x, vector_a.y + vector_b.y, vector_a.z + vector_b.z};}
static inline vector3 vector3_subtraction (vector3 vector_a, vector3 vector_b) {return (vector3) {vector_a.x - vector_b.x, vector_a.y - vector_b.y, vector_a.z - vector_b.z};}
static inline vector3 vector3_scaling (vector3 vector, float scale_factor) {return (vector3) {vector.x * scale_factor, vector.y * scale_factor, vector.z * scale_factor};}
//Dot: work and projection of vectors
static inline float vector3_dot (vector3 vector_a, vector3 vector_b) {return (float) (vector_a.x * vector_b.x + vector_a.y * vector_b.y + vector_a.z * vector_b.z);}
//Cross: Torque conversion and computation
static inline vector3 vector3_cross (vector3 vector_a, vector3 vector_b) {return (vector3) {(vector_a.y * vector_b.z - vector_a.z * vector_b.y), (vector_a.z * vector_b.x - vector_a.x * vector_b.z), (vector_a.x * vector_b.y - vector_a.y * vector_b.x)};}
static inline float vector3_length_squared (vector3 vector) {return vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;}
static inline float vector3_length (vector3 vector) {return sqrtf (vector3_length_squared (vector));}
static inline vector3 vector3_normalisation (vector3 vector) {
    float length = vector3_length (vector);
    if (length < math_epsilon) {return vector3_zero ();}
    return vector3_scaling (vector, 1.0 / length);
} //Quarternion (4D) Functions
//orientation in rotational w-axis w/o gimbal in any axis
static inline vector4 vector4_identity () {return (vector4) {1.0, 0.0, 0.0, 0.0};}
static inline vector4 vector4_normalisation (vector4 quaternion) {
    float length = sqrtf (quaternion.w * quaternion.w + quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    if (length < math_epsilon) {return vector4_identity ();}
    float inverse_length = 1.0 / length;
    return (vector4) {quaternion.w * inverse_length, quaternion.x * inverse_length, quaternion.y * inverse_length, quaternion.z * inverse_length};
} //Multiplication of two 4D matrices at once (combinatoric rotational motion)
static inline vector4 vector4_multiplication (vector4 quaternion_a, vector4 quaternion_b) {
    return (vector4) {
        quaternion_a.w * quaternion_b.w - quaternion_a.x * quaternion_b.x - quaternion_a.y * quaternion_b.y - quaternion_a.z * quaternion_b.z,
        quaternion_a.w * quaternion_b.x + quaternion_a.x * quaternion_b.w + quaternion_a.y * quaternion_b.z - quaternion_a.z * quaternion_b.y,
        quaternion_a.w * quaternion_b.y - quaternion_a.x * quaternion_b.z + quaternion_a.y * quaternion_b.w + quaternion_a.z * quaternion_b.x,
        quaternion_a.w * quaternion_b.z + quaternion_a.x * quaternion_b.y - quaternion_a.y * quaternion_b.x + quaternion_a.z * quaternion_b.w
    };
} //Rotate a 3D vector by a 4D rotational matrix
//v_a = q * v * q_conjugation
static inline vector3 vector4_rotate_to_vector3 (vector4 quaternion, vector3 vector) {
    //Nominal formula for crossing 3D to 4D axial
    vector3 quaternion_vector = {quaternion.x, quaternion.y, quaternion.z};
    vector3 temp_cross = vector3_cross (quaternion_vector, vector);
    //Scale by a factor of 2
    temp_cross = vector3_scaling (temp_cross, 2.0);
    vector3 cross_result = vector3_cross (quaternion_vector, temp_cross);
    //Extrapolate and enhance to w axis
    vector3 w_axis_scaled = vector3_scaling (temp_cross, quaternion.w);
    return vector3_addition (vector, vector3_addition (w_axis_scaled, cross_result));
} //Rotation from the Axis with angular orientation
static inline vector4 vector4_from_axis_with_angle (vector3 rotation_axis, float angle_radians) {
    float half_angle = angle_radians * 0.5;
    float sine_half_angle = sinf (half_angle);
    vector3 normalized_axis = vector3_normalisation (rotation_axis);
    return (vector4) {cosf (half_angle), normalized_axis.x * sine_half_angle, normalized_axis.y * sine_half_angle, normalized_axis.z * sine_half_angle};
} //3 ^ 3 matrix Functions
static inline math3 math3_identity () {
    math3 result_matrix = {{{0}}};
    result_matrix.matrix [0][0] = 1.0; result_matrix.matrix [1][1] = 1.0; result_matrix.matrix [2][2] = 1.0;
    return result_matrix;
} //Multiply specific matrix by a existing vector
static inline vector3 math3_multiplication_vector3 (math3 matrix, vector3 vector) {
    return (vector3) {matrix.matrix [0][0] * vector.x + matrix.matrix [0][1] * vector.y + matrix.matrix [0][2] * vector.z, matrix.matrix [1][0] * vector.x + matrix.matrix [1][1] * vector.y + matrix.matrix [1][2] * vector.z, matrix.matrix [2][0] * vector.x + matrix.matrix [2][1] * vector.y + matrix.matrix [2][2] * vector.z};
} //Convert 4D to rotational matrix (Inertia Tensor rotations)
//I_total = R * I_local * * R_transposed
static inline math3 vector4_to_math3 (vector4 quaternion) {
    math3 result_matrix;
    //Defining actual plug in values
    float x_double = quaternion.x + quaternion.x, y_double = quaternion.y + quaternion.y, z_double = quaternion.z + quaternion.z;
    float x_x = quaternion.x * x_double, x_y = quaternion.x * y_double, x_z = quaternion.x * z_double;
    float y_y = quaternion.y * y_double, y_z = quaternion.y * z_double, z_z = quaternion.z * z_double;
    float w_x = quaternion.w * x_double, w_y = quaternion.w * y_double, w_z = quaternion.w * z_double;
    //Affix to math3 format
    result_matrix.matrix [0][0] = 1.0 - (y_y + z_z), result_matrix.matrix [0][1] = x_y - w_z, result_matrix.matrix [0][2] = x_z + w_y;
    result_matrix.matrix [1][0] = x_y + w_z, result_matrix.matrix [1][1] = 1.0 - (x_x + z_z), result_matrix.matrix [1][2] = y_z - w_x;
    result_matrix.matrix [2][0] = x_z - w_y, result_matrix.matrix [2][1] = y_z + w_x, result_matrix.matrix [2][2] = 1.0 - (x_x + y_y);
    return result_matrix;
} //Matrix Multiplication
static inline math3 math3_multiplication (math3 matrix_a, math3 matrix_b) {
    math3 result_matrix = {{{0}}};
    for (int row_index = 0; row_index < 3; row_index++) {
        for (int column_index = 0; column_index < 3; column_index++) {
            result_matrix.matrix [row_index][column_index] = (matrix_a.matrix [row_index][0] * matrix_b.matrix [0][column_index]) + (matrix_a.matrix [row_index][1] * matrix_b.matrix [1][column_index]) + (matrix_a.matrix [row_index][2] * matrix_b.matrix [2][column_index]);
        }
    } return result_matrix;
} static inline math3 math3_transposition (math3 matrix) {
    math3 result_matrix;
    for (int row_index = 0; row_index < 3; row_index++) {
        for (int column_index = 0; column_index < 3; column_index++) {result_matrix.matrix [row_index][column_index] = matrix.matrix [column_index][row_index];}
    } return result_matrix;
} //Matrix inversion (3 ^ 3 specific)
// Angular Constraint Calculation (change_p = J * M ^ -1 * J_transposed)
static inline math3 math3_inverse (math3 matrix) {
    float determinant = (matrix.matrix [0][0] * (matrix.matrix [1][1] * matrix.matrix [2][2] - matrix.matrix [2][1] * matrix.matrix [1][2])) - (matrix.matrix [0][1] * (matrix.matrix [1][0] * matrix.matrix [2][2] - matrix.matrix [1][2] * matrix.matrix [2][0])) + (matrix.matrix [0][2] * (matrix.matrix [1][0] * matrix.matrix [2][1] - matrix.matrix [1][1] * matrix.matrix [2][0]));
    if (fabsf (determinant) < math_epsilon) {return math3_identity ();} //Buffer Check
    float inverse_determinant = 1.0 / determinant;
    math3 result_matrix;
    //n ~= {0, 2}
    //[0][n]
    result_matrix.matrix [0][0] = (matrix.matrix [1][1] * matrix.matrix [2][2] - matrix.matrix [2][1] * matrix.matrix [1][2]) * inverse_determinant;
    result_matrix.matrix [0][1] = (matrix.matrix [0][2] * matrix.matrix [2][1] - matrix.matrix [0][1] * matrix.matrix [2][2]) * inverse_determinant;
    result_matrix.matrix [0][2] = (matrix.matrix [0][1] * matrix.matrix [1][2] - matrix.matrix [0][2] * matrix.matrix [1][1]) * inverse_determinant;
    //[1][n]
    result_matrix.matrix [1][0] = (matrix.matrix [1][2] * matrix.matrix [2][0] - matrix.matrix [1][0] * matrix.matrix [2][2]) * inverse_determinant;
    result_matrix.matrix [1][1] = (matrix.matrix [0][0] * matrix.matrix [2][2] - matrix.matrix [0][2] * matrix.matrix [2][0]) * inverse_determinant;
    result_matrix.matrix [1][2] = (matrix.matrix [1][0] * matrix.matrix [0][2] - matrix.matrix [0][0] * matrix.matrix [1][2]) * inverse_determinant;
    //[2][n]
    result_matrix.matrix [2][0] = (matrix.matrix [1][0] * matrix.matrix [2][1] - matrix.matrix [2][0] * matrix.matrix [1][1]) * inverse_determinant;
    result_matrix.matrix [2][1] = (matrix.matrix [2][0] * matrix.matrix [0][1] - matrix.matrix [0][0] * matrix.matrix [2][1]) * inverse_determinant;
    result_matrix.matrix [2][2] = (matrix.matrix [0][0] * matrix.matrix [1][1] - matrix.matrix [1][0] * matrix.matrix [0][1]) * inverse_determinant;
    return result_matrix;
} static inline math3 fov_aspr_perspective (float field_of_view, float aspect_ratio, float near_plane, float far_plane) {
    math3 perspective_matrix = {{{0}}};
    float focal_length = 1.0 / tanf (field_of_view / 2.0);
    perspective_matrix.matrix [0][0] = focal_length / aspect_ratio;
    perspective_matrix.matrix [1][1] = focal_length;
    perspective_matrix.matrix [2][2] = (far_plane + near_plane) / (near_plane - far_plane);
    //3 ^ 3 handling of Z - Translations is suboptimal
    //Counter by simply scaling size
    return perspective_matrix;
}
#endif //math3D_h

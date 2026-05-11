#ifndef math4_h
#define math4_h
#include "math3D.h"
typedef struct { float matrix [4][4]; } math4;
//Zero Init
static inline math4 math4_init () {
    math4 result_matrix = {{{0}}};
    return result_matrix;
} //Identity Matrices
static inline math4 math4_identity () {
    math4 result_matrix = {{{0}}};
    result_matrix.matrix [0][0] = 1.0f;
    result_matrix.matrix [1][1] = 1.0f;
    result_matrix.matrix [2][2] = 1.0f;
    result_matrix.matrix [3][3] = 1.0f;
    return result_matrix;
} //Viewing Perspective
static inline math4 math4_look_view (vector3 camera_position, vector3 camera_front, vector3 camera_up) {
    math4 result_matrix = math4_identity ();
    vector3 forward_vector = vector3_normalisation (camera_front);
    vector3 side_vector = vector3_normalisation (vector3_cross (forward_vector, camera_up));
    vector3 up_vector = vector3_cross (side_vector, forward_vector);
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
} //Perspective Projection Matrices
static inline math4 math4_perspective_fov (float field_of_view, float aspect_ratio, float near_plane, float far_plane) {
    math4 result_matrix = {{{0}}};
    float focal_length = 1.0f / tanf (field_of_view / 2.0f);
    result_matrix.matrix [0][0] = focal_length / aspect_ratio;
    result_matrix.matrix [1][1] = focal_length;
    result_matrix.matrix [2][2] = (far_plane + near_plane) / (near_plane - far_plane);
    result_matrix.matrix [3][2] = (2.0f * far_plane * near_plane) / (near_plane - far_plane);
    result_matrix.matrix [2][3] = -1.0f;
    return result_matrix;
} //Translational Motion Matrix
static inline math4 math4_translation (vector3 translation_vector) {
    math4 result_matrix = math4_identity ();
    result_matrix.matrix [3][0] = translation_vector.x;
    result_matrix.matrix [3][1] = translation_vector.y;
    result_matrix.matrix [3][2] = translation_vector.z;
    return result_matrix;
} //Scaling Matrix
static inline math4 math4_scaling (vector3 scale_vector) {
    math4 result_matrix = math4_identity ();
    result_matrix.matrix [0][0] = scale_vector.x;
    result_matrix.matrix [1][1] = scale_vector.y;
    result_matrix.matrix [2][2] = scale_vector.z;
    return result_matrix;
} //Multiplication
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
    } return result_matrix;
} //Quaternion to Matrix Interface
static inline math4 vector4_to_math4 (vector4 quaternion) {
    math4 result_matrix = math4_identity ();
    float x_double = quaternion.x + quaternion.x, y_double = quaternion.y + quaternion.y, z_double = quaternion.z + quaternion.z;
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
} //GPU flat array interface
static inline void math4_to_flat_array (math4 matrix, float *output_array) {
    for (int column_index = 0; column_index < 4; column_index++) {
        for (int row_index = 0; row_index < 4; row_index++) {output_array [column_index * 4 + row_index] = matrix.matrix [column_index][row_index];}
    }
}
#endif

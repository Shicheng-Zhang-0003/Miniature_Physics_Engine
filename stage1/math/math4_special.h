#ifndef math4_h
#define math4_h
#include "math3D.h"
typedef struct { float matrix [4][4]; } math4;
//Zero Init
static inline math4 math4_init () {
    math4 m = {{{0}}};
    return m;
} //Identity Matrices
static inline math4 math4_identity () {
    math4 m = {{{0}}};
    m.matrix [0][0] = 1.0;
    m.matrix [1][1] = 1.0;
    m.matrix [2][2] = 1.0;
    m.matrix [3][3] = 1.0;
    return m;
} //Perspective Projection Matrices (full quaternion)
static inline math4 math4_perspective_fov (float fov, float aspect_rto, float nearfield, float farfield) {
    math4 m = {{{0}}};
    float f = 1.0 / tanf (fov / 2.0);
    m.matrix [0][0] = f / aspect_rto;
    m.matrix [1][1] = f;
    m.matrix [2][2] = (farfield + nearfield) / (nearfield - farfield);
    m.matrix [2][3] = -1.0;
    m.matrix [3][2] = (2.0 * farfield * nearfield) / (nearfield - farfield);
    return m;
} //Viewing Perspective
//Matrix from camera angle vectors (scrap camera_pov.h camera_view_matrix)
static inline math4 math4_look_view (vector3 position, vector3 frontal, vector3 up) {
    math4 m = {{{0}}};
    vector3 front = vector3_normalisation (frontal);
    vector3 right = vector3_normalisation (vector3_cross (front, up));
    vector3 upper = vector3_cross (right, front);
    m.matrix [0][0] = right.x;
    m.matrix [1][0] = right.y;
    m.matrix [2][0] = right.z;
    m.matrix [0][1] = upper.x;
    m.matrix [1][1] = upper.y;
    m.matrix [2][1] = upper.z;
    m.matrix [0][2] = -front.x;
    m.matrix [1][2] = -front.y;
    m.matrix [2][2] = -front.z;
    m.matrix [3][0] = -vector3_dot (right, position);
    m.matrix [3][1] = -vector3_dot (upper, position);
    m.matrix [3][2] = vector3_dot (front, position);
    m.matrix [3][3] = 1.0;
    return m;
} //Multiplication of quaternion Matrices
static inline math4 math4_multiplication (math4 a, math4 b) {
    math4 res = {{{0}}};
    for (int step1 = 0; step1 < 4; step1++) {
        for (int step2 = 0; step2 < 4; step2++) {
            res.matrix [step1][step2] =
                a.matrix [step1][0] * b.matrix [0][step2] +
                a.matrix [step1][1] * b.matrix [1][step2] +
                a.matrix [step1][2] * b.matrix [2][step2] +
                a.matrix [step1][3] * b.matrix [3][step2];
        }
    } return res;
} //Translational Motion Matrix
static inline math4 math4_translation (vector3 translate) {
    math4 m = math4_identity ();
    m.matrix [3][0] = translate.x;
    m.matrix [3][1] = translate.y;
    m.matrix [3][2] = translate.z;
    return m;
} //Scaling Matrix
static inline math4 math4_scaling (vector3 scalor) {
    math4 m = math4_identity ();
    m.matrix [0][0] = scalor.x;
    m.matrix [1][1] = scalor.y;
    m.matrix [2][2] = scalor.z;
    return m;
} static inline math4 vector4_to_math4 (vector4 quart) {
    math4 m = math4_identity ();
    float x2 = quart.x + quart.x, y2 = quart.y + quart.y, z2 = quart.z + quart.z;
    float xx = quart.x * x2, xy = quart.x * y2, xz = quart.x * z2;
    float yy = quart.y * y2, yz = quart.y * z2, zz = quart.z * z2;
    float wx = quart.w * x2, wy = quart.w * y2, wz = quart.w * z2;
    m.matrix [0][0] = 1.0 - (yy + zz);
    m.matrix [1][0] = xy - wz;
    m.matrix [2][0] = xz + wy;
    m.matrix [0][1] = xy + wz;
    m.matrix [1][1] = 1.0 - (xx + zz);
    m.matrix [2][1] = yz - wx;
    m.matrix [0][2] = xz - wy;
    m.matrix [1][2] = yz + wx;
    m.matrix [2][2] = 1.0 - (xx + yy);
    return m;
} //When required, compress to a ordinary floating point array of size 16
//Required for OpenGL interface
static inline void math4_to_flat_array (math4 m, float *output) {
    for (int step1 = 0; step1 < 4; step1++) {
        for (int step2 = 0; step2 < 4; step2++) {output [step1 * 4 + step2] = m.matrix [step2][step1];}
    }
}
#endif

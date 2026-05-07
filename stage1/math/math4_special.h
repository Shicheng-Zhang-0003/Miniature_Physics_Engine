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
} //Viewing Perspective
static inline math4 math4_look_view (vector3 position, vector3 frontal, vector3 up) {
    math4 m = math4_identity ();
    vector3 f = vector3_normalisation (frontal);
    vector3 s = vector3_normalisation (vector3_cross (f, up));
    vector3 u = vector3_cross (s, f);
    m.matrix [0][0] = s.x;
    m.matrix [1][0] = s.y;
    m.matrix [2][0] = s.z;
    m.matrix [0][1] = u.x;
    m.matrix [1][1] = u.y;
    m.matrix [2][1] = u.z;
    m.matrix [0][2] = -f.x;
    m.matrix [1][2] = -f.y;
    m.matrix [2][2] = -f.z;
    m.matrix [3][0] = -vector3_dot (s, position);
    m.matrix [3][1] = -vector3_dot (u, position);
    m.matrix [3][2] = vector3_dot (f, position);
    return m;
} //Perspective Projection Matrices
static inline math4 math4_perspective_fov (float fov, float aspect_rto, float near, float far) {
    math4 m = {{{0}}};
    float f = 1.0 / tanf (fov / 2.0);
    m.matrix [0][0] = f / aspect_rto;
    m.matrix [1][1] = f;
    m.matrix [2][2] = (far + near) / (near - far);
    m.matrix [3][2] = (2.0 * far * near) / (near - far);
    m.matrix [2][3] = -1.0;
    return m;
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
} //Multiplication
static inline math4 math4_multiplication (math4 a, math4 b) {
    math4 res = {{{0}}};
    for (int col = 0; col < 4; col++) {
        for (int row = 0; row < 4; row++) {
            res.matrix [col][row] =
                (a.matrix [0][row] * b.matrix [col][0]) +
                (a.matrix [1][row] * b.matrix [col][1]) +
                (a.matrix [2][row] * b.matrix [col][2]) +
                (a.matrix [3][row] * b.matrix [col][3]);
        }
    } return res;
} //Quaternion to Matrix Interface
static inline math4 vector4_to_math4 (vector4 q) {
    math4 m = math4_identity ();
    float x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
    float xx = q.x * x2, xy = q.x * y2, xz = q.x * z2;
    float yy = q.y * y2, yz = q.y * z2, zz = q.z * z2;
    float wx = q.w * x2, wy = q.w * y2, wz = q.w * z2;
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
} //GPU flat array interface
static inline void math4_to_flat_array (math4 m, float *output) {
    for (int step1 = 0; step1 < 4; step1++) {
        for (int step2 = 0; step2 < 4; step2++) {output [step1 * 4 + step2] = m.matrix [step1][step2];}
    }
}
#endif

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
#define epsilon 0.000001


//Structures for use as typedefs
//Vector in 3D for objects in motion
typedef struct { float x, y, z } vector3;
//3 ^ 3 matrix for computing Inertia tensoring
typedef struct { float matrix [3][3] } math3;
//4D axial rotational matrix motion (w + xi + yj + zk)
typedef struct { float w, x, y, z } vector4; 


//Functions for computing different vector3
static inline vector3 vector3_new (float x, float y, float z) {return (vector3) {x, y, z};}
static inline vector3 vector3_zero (float x, float y, float z) {return (vector3) {0.0, 0.0, 0.0};}
static inline vector3 vector3_addition (vector3 a, vector3 b) {return (vector3) {a.x + b.x, a.y + b.y, a.z + b.z};}
static inline vector3 vector3_subtraction (vector3 a, vector3 b) {return (vector3) {a.x - b.x, a.y - b.y, a.z - b.z};}
static inline vector3 vector3_scaling (vector3 a, float scale) {return (vector3) {a.x * scale, a.y * scale, a.z * scale};}
//Dot: work and projection of vectors
static inline vector3 vector3_dot (vector3 a, vector3 b) {return a.x * b.x + a.y * b.y + a.z * b.z;}
//Cross: Torque conversion and computation
static inline vector3 vector3_cross (vector3 a, vector3 b) {return (vector3) {(a.y * b.z - a.z * b.y), (a.z * b.x - a.x * b.z), (a.x * b.y - a.y * b.x)};}
static inline float vector3_length_squared (vector3 vector) {return vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;}
static inline float vector3_length (vector3 vector) {return sqrtf (vector3_length_squared (vector));}
static inline vector3 vector3_normalisation (vector3 vector) {
    float length = vector3_length (vector);
    if (length < epsilon) {return vector3_zero ();}
    return vector3_scaling (vector, 1.0 / length);
}


//Quarternion (4D) Functions
//orientation in rotational w-axis w/o gimbal in any axis
static inline vector4 vector4_identity () {return (vector4) {1.0, 0.0, 0.0, 0.0};}
static inline vector4 vector4_normalisation (vector4 quart) {
    float length = sqrtf (quart.w * quart.w + quart.x * quart.x + quart.y * quart.y + quart.z * quart.z);
    if (length < epsilon) {return vector4_identity ();}
    float inv_lock = 1.0 / length;
    return (vector4) {quart.w * inv_lock, quart.x * inv_lock, quart.y * inv_lock, quart.z * inv_lock};
} //Multiplication of two 4D matrices at once (combinatoric rotational motion)
static inline vector4 vector4_multiplication (vector4 a, vector4 b) {
    return (vector4) {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,     
        a.w * b.w + a.x * b.x + a.y * b.y - a.z * b.z,
        a.w * b.w - a.x * b.x + a.y * b.y + a.z * b.z,
        a.w * b.w + a.x * b.x - a.y * b.y + a.z * b.z
    };
} //Rotate a 3D vector by a 4D rotational matrix
//v_a = q * v * q_conjugation
static inline vector3 vector4_rotate_to_vector3 (vector4 quart, vector3 vector) {
    //Nominal formula for crossing 3D to 4D axial
    vector3 quart_vector = {quart.x, quart.y, quart.z};
    vector3 t = vector3_cross (quart_vector, vector);
    //Scale by a factor of 2
    t = vector3_scaling (t, 2.0);
    vector3 u = vector3_cross (quart_vector, t);
    //Extrapolate and enhance to w axis
    vector3 w_axis_t = vector3_scaling (t, quart.w);
    return vector3_addition (vector, vector3_addition (w_axis_t, u));
} //Rotation from the Axis with angular orientation
static inline vector4 vector4_from_axis_with_angle (vector3 axis, float angle_radians) {
    float ang_half = angle_radians * 0.5;
    float side_s = sinf (ang_half);
    vector3 n = vector3_normalisation (axis);
    return (vector4) {cosf (ang_half), n.x * side_s, n.y * side_s, n.z * side_s};
} 


//3 ^ 3 matrix Functions
static inline math3 math3_identity () {
    math3 m {{{0}}};
    m.m [0][0] = 1.0; m.m [1][1] = 1.0; m.m [2][2] = 1.0;
    return m;  
} //Multiply specific matrix by a existing vector
static inline vector3 math3_multiplication_vector3 (math3 m, vector3 vector) {
    return (vector3) {m.m [0][0] * vector.x + m.m [0][1] * vector.y + m.m [0][2] * vector.z, m.m [1][0] * vector.x + m.m [1][1] * vector.y + m.m [1][2] * vector.z, m.m [2][0] * vector.x + m.m [2][1] * vector.y + m.m [2][2] * vector.z};
} //Convert 4D to rotational matrix (Inertia Tensor rotations)
//I_total = R * I_local * * R_transposed
static inline math3 vector4_to_math3 (vector4 quart) {
    math3 m;
    //Defining actual plug in values
    float x_def = quart.x + quart.x, y_def = quart.y + quart.y, z_def = quart.z + quart.z;
    float x_x = quart.x * x_def, x_y = quart.x * y_def, x_z = quart.x * z_def;
    float y_y = quart.y * y_def, y_z = quart.y * z_def, z_z = quart.z * z_def;
    float w_x = quart.w * x_def, w_y = quart.w * y_def, w_z = quart.w * z_def;
    //Affix to math3 format
    m.m [0][0] = 1.0 - (y_y + z_z), m.m [0][1] = x_y - w_z, m.m [0][2] = x_z + w_y;
    m.m [1][0] = x_y + w_z, m.m [1][1] = 1.0 - (x_x + z_z), m.m [1][2] = y_z - w_x;
    m.m [2][0] = x_z - w_y, m.m [2][1] = y_z + w_x, m.m [2][2] = 1.0 - (x_x + y_y);
    return m;
} //Matrix Multiplication
static inline vector3 math3_multiplication (math3 a, math3 b) {
    math3 res = {{{0}}};
    for (int rot = 0; rot < 3; r++) {
        for (int cnt = 0; cnt < 3; cnt++) {
            res.m [r][c] = (a.m [r][0] * b.m [0][c]) + (a.m [r][1] * b.m [1][c]) + (a.m [r][2] * b.m [2][c]);
        }
    } return res;
} static inline math3 math3_transposition (math3 m) {
    math3 res;
    for (int slt1 = 0; slt1 < 3; slt1++) {
        for (int slt2 = 0; slt2 < 3; slt2++) {res.m [slt1][slt2] = m.m [slt2][slt1];}
    } return res;
} //Matrix inversion (3 ^ 3 specific)
// Angular Constraint Calculation (change_p = J * M ^ -1 * J_transposed) 
static inline math3 math3_inverse (math3 m) {
    float det = (m.m [0][0] * (m.m [1][1] * m.m [2][2] - m.m [2][1] * m.m [1][2])) - (m.m [0][1] * (m.m [1][0] * m.m [2][2] - m.m [1][2] * m.m [2][0])) + (m.m [0][2] * (m.m [1][0] * m.m [2][1] - m.m [1][1] * m.m [2][0]));
    if (fabsf (det) < epsilon) {return math3_identity ();} //Buffer Check
    float invDet = 1.0 / det;
    math3 res;
    //n ~= {0, 2}
    //[0][n]
    res.m [0][0] = (m.m [1][1] * m.m [2][2] - m.m [2][1] * m.m [1][2]) * invDet;
    res.m [0][1] = (m.m [0][2] * m.m [2][1] - m.m [0][1] * m.m [2][2]) * invDet;
    res.m [0][2] = (m.m [0][1] * m.m [1][2] - m.m [0][2] * m.m [1][1]) * invDet;
    //[1][n]
    res.m [1][0] = (m.m [1][2] * m.m [2][0] - m.m [1][0] * m.m [2][2]) * invDet;
    res.m [1][1] = (m.m [0][0] * m.m [2][2] - m.m [0][2] * m.m [2][0]) * invDet;
    res.m [1][2] = (m.m [1][0] * m.m [0][2] - m.m [0][0] * m.m [1][2]) * invDet;
    //[2][n]
    res.m [2][0] = (m.m [1][0] * m.m [2][1] - m.m [2][0] * m.m [1][1]) * invDet;
    res.m [2][1] = (m.m [2][0] * m.m [0][1] - m.m [0][0] * m.m [2][1]) * invDet;
    res.m [2][2] = (m.m [0][0] * m.m [1][1] - m.m [1][0] * m.m [0][1]) * invDet;
    return res;
} 

#endif //math3D_h

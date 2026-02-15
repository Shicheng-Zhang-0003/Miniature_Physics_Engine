//NOTE: W/O MATH4 W-AXIS 3D SUPPORT (FORCE SQUASH TO 2D)
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <gtk/gtk.h>
#include "../../stage1/math/math3D.h"
static inline math3 pov_projection (float fov, float aspect, float nearfield, float farfield) {
    math3 m = {{{0}}};
    float tangent_half_fov = tanf (fov / 2.0);
    m.matrix [0][0] = 1.0 / (aspect * tangent_half_fov);
    m.matrix [1][1] = 1.0 / tangent_half_fov;
    m.matrix [2][2] = -(farfield + nearfield) / (farfield - nearfield);
    //Standard 3x3 --> no translated W-axis
    //math4 (4 ^ 4 matrix) would be required in the future for 3D 
    return m;
}

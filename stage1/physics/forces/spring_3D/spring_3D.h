#include <stdio.h>
#include "../../../math/math3D.h"
#include "../../../math_phys_buffer/buffer.h"
void apply_spring_force_3D (rigidbody *rb, vector3 anchor, float rest_length, float k_constant) {
    vector3 delta_variable = vector3_subtraction (rb->position, anchor);
    float current_length = vector3_length (delta_variable);
    if (current_length > epsilon) {
        vector3 direction = vector3_normalisation (delta_variable);
        float displacement = current_length - rest_length;
        //Fs = -kx
        vector3 spring_force = vector3_scaling (direction, -k_constant * displacement);
        rb_apply_forces_perfect (rb, spring_force);
    }
}

#include <stdio.h>
#include "../../../math/math3D.h"
#include "../../../math_phys_buffer/buffer.h"
static void apply_spring_force_3D (rigidbody *rigid_body, vector3 anchor_position, float equilibrium_length, float spring_constant) {
    vector3 displacement_vector = vector3_subtraction (rigid_body -> position, anchor_position);
    float current_length = vector3_length (displacement_vector);
    if (current_length > math_epsilon) {
        vector3 normalized_direction = vector3_normalisation (displacement_vector);
        float displacement_magnitude = current_length - equilibrium_length;
        //Fs = -kx
        vector3 spring_force = vector3_scaling (normalized_direction, -spring_constant * displacement_magnitude);
        rb_apply_forces_perfect (rigid_body, spring_force);
    }
}

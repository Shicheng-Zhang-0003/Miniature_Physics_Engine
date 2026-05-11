#include <stdio.h>
#include "../../../math/math3D.h"
#include "../../../math_phys_buffer/buffer.h"
static void applicant_drag_3D (rigidbody *rigid_body, float drag_coefficient, bool use_quadratic_drag) {
    float linear_speed = vector3_length (rigid_body -> velocity);
    if (linear_speed < math_epsilon) {return;}
    vector3 opposition_direction = vector3_scaling (rigid_body -> velocity, -1.0f); //Reverse directional vectoring
    vector3 normalized_opposition = vector3_normalisation (opposition_direction);
    float drag_force_magnitude;
    if (use_quadratic_drag) {drag_force_magnitude = drag_coefficient * linear_speed * linear_speed;} //High Speed or 3D application, Fd = bv ^ 2
    else {drag_force_magnitude = drag_coefficient * linear_speed;} //Low speed or Linear Application, Fd = bv
    rb_apply_forces_perfect (rigid_body, vector3_scaling (normalized_opposition, drag_force_magnitude)); //Apply Actual drag force to the object with opposition vector and magnitude
}

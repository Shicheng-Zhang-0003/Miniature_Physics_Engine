#include <stdio.h>
#include "../../../math/math3D.h"
#include "../../../math_phys_buffer/buffer.h"
void applicant_drag_3D (rigidbody *rb, float b, bool quadratic_incorp) { //B = coefficient of drag
    float speed = vector3_length (rb->velocity);
    if (speed < epsilon) {return;}
    vector3 opposite_direction = vector3_scaling (rb->velocity, -1.0); //Reverse directional vectoring
    vector3 normalised_opposition = vector3_normalisation (opposite_direction);
    float drag_magnitude;
    if (quadratic_incorp) {drag_magnitude = b * speed * speed;} //High Speed or 3D application, Fd = bv ^ 2
    else {drag_magnitude = b * speed} //Low speed or Linear Application, Fd = bv
    rb_apply_forces_perfect(rb, vector3_scaling (normalised_opposition, drag_magnitude)); //Apply Actual drag force to the object with opposition vector and magnitude
}

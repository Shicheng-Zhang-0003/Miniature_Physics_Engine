#include <stdio.h>
#include "../../../math/math3D.h"
#include "../../../math_phys_buffer/buffer.h"
void applicant_centripetal_vector3 (rigidbody *rb, vector3 centrepoint, float strength) {
    //Radius Vector (3D) (r = centre_position - position_current)
    vector3 lock_vector = vector3_subtraction(centrepoint, rb->position);
    float radius = vector3_length (lock_vector);
    if (radius > epsilon) {
        //Normalise the vector towards the inwards direction to centrepoint
        vector3 direction = vector3_normalisation (lock_vector);
        //Fc = mv ^ 2 * r ^ -1
        float velocity_centripetal = vector3_length (rb->velocity);
        float magnitude = strength * rb->mass * velocity_centripetal * velocity_centripetal / radius;
        vector3 force_centripetal = vector3_scaling (direction, magnitude);
        rb_apply_forces_perfect (rb, force_centripetal);
    }
}

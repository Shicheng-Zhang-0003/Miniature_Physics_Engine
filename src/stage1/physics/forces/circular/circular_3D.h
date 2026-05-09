#include <stdio.h>
#include "../../../math/math3D.h"
#include "../../../math_phys_buffer/buffer.h"
static void applicant_centripetal_vector3 (rigidbody *rigid_body, vector3 center_point, float centripetal_strength_modifier) {
    //Radius Vector (3D) (r = center_position - position_current)
    vector3 radial_vector = vector3_subtraction (center_point, rigid_body -> position);
    float radial_distance = vector3_length (radial_vector);
    if (radial_distance > math_epsilon) {
        //Normalise the vector towards the inwards direction to center_point
        vector3 inward_direction = vector3_normalisation (radial_vector);
        //Fc = mv ^ 2 * r ^ -1
        float linear_speed = vector3_length (rigid_body -> velocity);
        float centripetal_force_magnitude = centripetal_strength_modifier * rigid_body -> mass * linear_speed * linear_speed / radial_distance;
        vector3 centripetal_force = vector3_scaling (inward_direction, centripetal_force_magnitude);
        rb_apply_forces_perfect (rigid_body, centripetal_force);
    }
}

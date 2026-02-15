#include <stdio.h>
#include "../../../math/math3D.h"
#include "../../../math_phys_buffer/buffer.h"
void applicant_friction_3D (rigidbody *rb, vector3 surface_normal, float mu_kinetic) {
    //Calculate How much the object moves along the surface at the moment of calculation
    float velocity_dot_normal = vector3_dot (rb->velocity, surface_normal);
    vector3 velocity_normal_compute = vector3_scaling (surface_normal, velocity_dot_normal);
    vector3 velocity_tangent = vector3_subtraction (rb->velocity, velocity_normal_compute);
    float speed_tangent = vector3_length (velocity_tangent);
    if (speed_tangent > epsilon) {
        //Friction Magnitude --> Ff = uFn
        //Assume Fn to Fg   
        float weight = rb->mass * 9.81;
        float friction_magnitude = mu_kinetic * weight;
        vector3 friction_vector = vector3_scaling(vector3_normalisation(velocity_tangent), -1.0);
        rb_apply_forces_perfect (rb, vector3_scaling (friction_vector, friction_magnitude));
    }
}

#ifndef forces_h
#define forces_h
#include <stdio.h>
#include <math.h>
#include "../../math/math3D.h"
#include "../../math_phys_buffer/buffer.h"
//Gravity (Regular and Sloped Parallel Gravity)
//Local Gravity (Fg = mg)
void force_applicant_gravity_normal (rigidbody *rb, vector3 gravitational_acceleration, vector3 normal_from_surface) {
    if (rb->static_state) {return;}
    //Fg = mg
    vector3 force_gravity = vector3_scaling (gravitational_acceleration, rb->mass);
    rb_apply_forces_perfect (rb, force_gravity);
    //Fn = -mg (only if object is on a particular surface and supported)
    float weight_slope = vector3_dot (force_gravity, normal_from_surface);
    if (weight_slope < 0) { //Gravity pushes into surface volume
        vector3 force_normal = vector3_scaling (normal_from_surface, -weight_slope); //Apply Fg in inverse vector
        rb_apply_forces_perfect (rb, force_normal);
    }
} 

//Universal Law of Gravitation
//Fg = Gm1m2r ^ -2
#define big_g 6.67430e-11
void force_applicant_universal_gravity (rigidbody *a, rigidbody *b) {
    vector3 relation_vector = vector3_subtraction (b->position, a->position); //Relative Vector and distance
    float distance_squared = vector3_length_squared (relation_vector); //r ^ 2
    if (distance_squared < epsilon) {return;}
    float distance = sqrtf (distance_squared); //r
    float force_magnitude = (big_g * a->mass * b->mass) / distance_squared; //Fg = Gm1m2r ^ -2
    vector3 force_out = vector3_scaling (vector3_normalisation (relation_vector), force_magnitude); //Check magnitude and vectors for applications
    rb_apply_forces_perfect (a, force_out); //Apply to positive vector object (a)
    rb_apply_forces_perfect (n, vector3_scaling (force_out, -1.0)); //Apply to negative vector object (b), equal and opposite direction 
}

//Friction Definition (3D tangent plane fields)
void force_applicant_friction (rigidbody *rb, vector3 surface_normal, float mu_static, float mu_kinetic) {
    //Calculate the magnitude of normal forces
    //Ff = uFn
    vector3 force_gravity = vector3_scaling ((vector3) {0, -9.81, 0}, rb->mass); //Scale Mass by -9.81 to get mg, Fg
    float force_normal_magnitude = fabsf (vector3_dot (force_gravity, surface_normal)); //Magnitude of Force Normal
    //Find the tangent velocity of the objetc along the surface of support
    //velocity_tangent = velocity - (v * n) * n
    float velocity_d_normal = vector3_dot (rb->velocity, surface_normal);
    vector3 velocity_tangent = vector3_subtraction (rb->velocity, vector3_scaling (surface_normal, velocity_d_normal));
    float speed_tangent = vector3_length (velocity_tangent);
    if (speed_tangent > epsilon) {
        //Kinetic Friction: Resists curent sliding motion
        vector3 kinetic_friction = vector3_scaling (vector3_normalisation (velocity_tangent),
                -mu_kinetic * force_normal_magnitude);
        rb_apply_forces_perfect (rb, kinetic_friction);
    } else { 
        //Static Friction: opposition to any form of newly added motion from externalised force
        //Calculated already applied forces
        vector3 force_tangent_accumulated = vector3_subtraction (rb->force_accumulator, 
                vector3_scaling (surface_normal, vector3_dot (rb->force_accumulator, surface_normal))); //Accumulated force applied to existing net input for net force output applied on the object
        float accumulated_force_magnitude = vector3_length (force_tangent_accumulated);
        if (accumulated_force_magnitude < mu_static * force_normal_magnitude) {
            //Force applied < Force Static Friction --> No movement
            //Neutralise Sliding Force
            rb_apply_forces_perfect (rb, vector3_scale (force_tangent_accumulated, -1.0));
            rb->velocity = vector3_zero ();
        }
    }
}

//Spring Force, Tension, Hooke Law
void force_applicant_string (rigidbody *rb, vector3 anchor, float resistance_length, float k_constant, float damping) {
    vector3 direction_vector = vector3_subtraction (rb->position, anchor);
    float current_length = vector3_length (direction_vector);
    if (current_length < epsilon) {return;}
    float x_value = current_length - resistance_length;
    vector3 direction = vector3_normalisation (direction_vector);
    //Fs = -kx
    vector3 force_spring = vector3_scale (direction, -k_constant * x_value);
    //Damping = -c * v (prevents infinite oscillation from release spring compression)
    vector3 force_damping = vector3_scaling (rb->velocity, -damping);
    rb_apply_forces_perfect (rb, vector3_addition (force_spring, force_damping)); //Net of spring and dampening forces
}

//Vertical Circular Motion
//Vertical circular motion requires Ft to counteract gravity in upper segments
//This also gives Fc (centripetal) and consequently centrifugal
void force_applicant_vertical_anchor (rigidbody *rb, vector3 pivot_point, float radius) {
    vector3 rotational_vector = vector3_subtraction (rb->position, pivot_point);
    vector3 direction = vector3_normalisation (rotational_vector);
    //Centripetal Force Required: Fc = m(vc) ^ 2(r ^ -1)
    float speed_squared = vector3_length_squared (rb->velocity);
    float force_centripetal_magnitude = (rb->mass * speed_squared) / radius;
    //Gravitational component along the anchoring motion (mg * cos (theta_relative_to_anchor_gravity))
    vector3 gravity = {0, -9.81 * rb->mass, 0}; //Gravitational Magnitude
    float force_gravity_along_anchor = vector3_dot (gravity, direction);
    //Tension (Ft): Simultaneously satisfy Fc and counteract Fg
    float force_tension_magnitude - force_gravity_along_anchor;
    //Apply Tension towards the vector of the pivot
    rb_apply_forces_perfect (rb, vector3_scale (direction, -force_tension_magnitude));
}


//Monitor the Energy component of the objects related
typedef struct {float ek, epg, eps, em;} state_energy;
state_energy force_to_system_energy_amount (rigidbody *rb, vector3 gravitational_acceleration) {
    state_energy es;
    //Ek --> kinetic
    es.ek = rb_get_Ek (rb);
    //Epg (Y value in vectoring is height)
    es.epg = rb->mass * fabsf (gravitational_acceleration.y) * rb->position.y;
    //Eps --> calculated on a per spring basis, not included
    //Em --> total MEC
    es.em = es.ek + es.epg;
    return es;
}
#endif


#ifndef forces_h
#define forces_h
#include <stdio.h>
#include <math.h>
#include "../../../math/math3D.h"
#include "../../../math_phys_buffer/buffer.h"
//Gravity (Regular and Sloped Parallel Gravity)
//Local Gravity (Fg = mg)
static void force_applicant_gravity_normal (rigidbody *rigid_body, vector3 gravitational_acceleration, vector3 surface_normal) {
    if (rigid_body -> static_state) {return;}
    //Fg = mg
    vector3 gravity_force = vector3_scaling (gravitational_acceleration, rigid_body -> mass);
    rb_apply_forces_perfect (rigid_body, gravity_force);
    //Fn = -mg (only if object is on a particular surface and supported)
    float gravitational_weight_along_normal = vector3_dot (gravity_force, surface_normal);
    if (gravitational_weight_along_normal < 0) { //Gravity pushes into surface volume
        vector3 normal_force = vector3_scaling (surface_normal, -gravitational_weight_along_normal); //Apply Fg in inverse vector
        rb_apply_forces_perfect (rigid_body, normal_force);
    }
} //Universal Law of Gravitation
//Fg = Gm1m2r ^ -2
#define big_g 6.67430e-11f
static void force_applicant_universal_gravity (rigidbody *rigid_body_a, rigidbody *rigid_body_b) {
    vector3 relative_position_vector = vector3_subtraction (rigid_body_b -> position, rigid_body_a -> position); //Relative Vector and distance
    float distance_squared = vector3_length_squared (relative_position_vector); //r ^ 2
    if (distance_squared < math_epsilon) {return;}
    float force_magnitude = (big_g * rigid_body_a -> mass * rigid_body_b -> mass) / distance_squared; //Fg = Gm1m2r ^ -2
    vector3 gravitational_force = vector3_scaling (vector3_normalisation (relative_position_vector), force_magnitude); //Check magnitude and vectors for applications
    rb_apply_forces_perfect (rigid_body_a, gravitational_force); //Apply to positive vector object (a)
    rb_apply_forces_perfect (rigid_body_b, vector3_scaling (gravitational_force, -1.0f)); //Apply to negative vector object (b), equal and opposite direction
} //Friction Definition (3D tangent plane fields)
static void force_applicant_friction_rolling (rigidbody *rigid_body, vector3 surface_normal, float static_friction_coefficient, float kinetic_friction_coefficient, float gravity_y) {
    //Calculate the magnitude of normal forces
    //Ff = uFn
    vector3 gravity_force = vector3_scaling ((vector3) {0, gravity_y, 0}, rigid_body -> mass); //Scale Mass by -9.81f to get mg, Fg
    float normal_force_magnitude = fabsf (vector3_dot (gravity_force, surface_normal)); //Magnitude of Force Normal
    //Contact Vector (r = -radius * normal)
    vector3 relative_contact_vector = vector3_scaling (surface_normal, -rigid_body -> radius);
    vector3 velocity_at_contact_point = vector3_addition (rigid_body -> velocity, vector3_cross (rigid_body -> angular_velocity, relative_contact_vector));
    //Tangent velocity at contact point
    float velocity_component_along_normal = vector3_dot (velocity_at_contact_point, surface_normal);
    vector3 tangential_velocity = vector3_subtraction (velocity_at_contact_point, vector3_scaling (surface_normal, velocity_component_along_normal));
    float tangential_speed = vector3_length (tangential_velocity);
    if (tangential_speed > math_epsilon) {
        //Kinetic Friction --> Resists contact sliding
        vector3 kinetic_friction_force = vector3_scaling (vector3_normalisation (tangential_velocity), -kinetic_friction_coefficient * normal_force_magnitude);
        //Force at Contact Point (Torque Generation)
        rb_apply_forces_localised (rigid_body, kinetic_friction_force, vector3_addition (rigid_body -> position, relative_contact_vector));
    } else {
        //Static Friction --> Neutralise tangent forces
        vector3 accumulated_tangential_force = vector3_subtraction (rigid_body -> force_accumilator, vector3_scaling (surface_normal, vector3_dot (rigid_body -> force_accumilator, surface_normal)));
        float accumulated_force_magnitude = vector3_length (accumulated_tangential_force);
        if (accumulated_force_magnitude < static_friction_coefficient * normal_force_magnitude) {
            rb_apply_forces_perfect (rigid_body, vector3_scaling (accumulated_tangential_force, -1.0f));
            //Stop micro-sliding
            rigid_body -> velocity = vector3_subtraction (rigid_body -> velocity, tangential_velocity);
        }
    }
} static void force_applicant_friction (rigidbody *rigid_body, vector3 surface_normal, float static_friction_coefficient, float kinetic_friction_coefficient, float gravity_y) {
    //Calculate the magnitude of normal forces
    //Ff = uFn
    vector3 gravity_force = vector3_scaling ((vector3) {0, gravity_y, 0}, rigid_body -> mass); //Scale Mass by gravity_y to get mg, Fg
    float normal_force_magnitude = fabsf (vector3_dot (gravity_force, surface_normal)); //Magnitude of Force Normal
    //Find the tangent velocity of the objetc along the surface of support
    //velocity_tangent = velocity - (v * n) * n
    float velocity_component_along_normal = vector3_dot (rigid_body -> velocity, surface_normal);
    vector3 tangential_velocity = vector3_subtraction (rigid_body -> velocity, vector3_scaling (surface_normal, velocity_component_along_normal));
    float tangential_speed = vector3_length (tangential_velocity);
    if (tangential_speed > math_epsilon) {
        //Kinetic Friction: Resists curent sliding motion
        vector3 kinetic_friction_force = vector3_scaling (vector3_normalisation (tangential_velocity), -kinetic_friction_coefficient * normal_force_magnitude);
        rb_apply_forces_perfect (rigid_body, kinetic_friction_force);
    } else {
        //Static Friction: opposition to any form of newly added motion from externalised force
        //Calculated already applied forces
        vector3 accumulated_tangential_force = vector3_subtraction (rigid_body -> force_accumilator, vector3_scaling (surface_normal, vector3_dot (rigid_body -> force_accumilator, surface_normal))); //Accumulated force applied to existing net input for net force output applied on the object
        float accumulated_force_magnitude = vector3_length (accumulated_tangential_force);
        if (accumulated_force_magnitude < static_friction_coefficient * normal_force_magnitude) {
            //Force applied < Force Static Friction --> No movement
            //Neutralise Sliding Force
            rb_apply_forces_perfect (rigid_body, vector3_scaling (accumulated_tangential_force, -1.0f));
            rigid_body -> velocity = vector3_zero ();
        }
    }
} //Spring Force, Tension, Hooke Law
static void force_applicant_string (rigidbody *rigid_body, vector3 anchor_position, float equilibrium_length, float spring_constant, float damping_coefficient) {
    vector3 displacement_vector = vector3_subtraction (rigid_body -> position, anchor_position);
    float current_length = vector3_length (displacement_vector);
    if (current_length < math_epsilon) {return;}
    float displacement_magnitude = current_length - equilibrium_length;
    vector3 direction_vector = vector3_normalisation (displacement_vector);
    //Fs = -kx
    vector3 spring_force = vector3_scaling (direction_vector, -spring_constant * displacement_magnitude);
    //Damping = -c * v (prevents infinite oscillation from release spring compression)
    vector3 damping_force = vector3_scaling (rigid_body -> velocity, -damping_coefficient);
    rb_apply_forces_perfect (rigid_body, vector3_addition (spring_force, damping_force)); //Net of spring and dampening forces
} //Vertical Circular Motion
//Vertical circular motion requires Ft to counteract gravity in upper segments
//This also gives Fc (centripetal) and consequently centrifugal
static void force_applicant_vertical_anchor (rigidbody *rigid_body, vector3 pivot_point, float radius, float gravity_y) {
    vector3 relative_position_vector = vector3_subtraction (rigid_body -> position, pivot_point);
    vector3 radial_direction = vector3_normalisation (relative_position_vector);
    //Centripetal Force Required: Fc = m(vc) ^ 2(r ^ -1)
    float speed_squared = vector3_length_squared (rigid_body -> velocity);
    float centripetal_force_magnitude = (rigid_body -> mass * speed_squared) / radius;
    //Gravitational component along the anchoring motion (mg * cos (theta_relative_to_anchor_gravity))
    vector3 gravity_force = {0, gravity_y * rigid_body -> mass, 0}; //Gravitational Magnitude
    float gravitational_force_along_radial_axis = vector3_dot (gravity_force, radial_direction);
    //Tension (Ft): Simultaneously satisfy Fc and counteract Fg
    float tension_force_magnitude = centripetal_force_magnitude + gravitational_force_along_radial_axis;
    //Apply Tension towards the vector of the pivot
    rb_apply_forces_perfect (rigid_body, vector3_scaling (radial_direction, -tension_force_magnitude));
} //Monitor the Energy component of the objects related
typedef struct {float linear_kinetic_energy, rotational_kinetic_energy, kinetic_energy, gravitational_potential_energy, spring_potential_energy, mechanical_energy;} state_energy;
static state_energy force_to_system_energy_amount (rigidbody *rigid_body, vector3 gravitational_acceleration) {
    state_energy energy_state;
    //Ek linear = 0.5fmv^2
    energy_state.linear_kinetic_energy = 0.5f * rigid_body -> mass * vector3_length_squared (rigid_body -> velocity);
    //Ek rotational = 0.5fwIw
    vector3 angular_momentum = math3_multiplication_vector3 (math3_inverse (rigid_body -> inverse_inertia_system), rigid_body -> angular_velocity);
    energy_state.rotational_kinetic_energy = 0.5f * vector3_dot (rigid_body -> angular_velocity, angular_momentum);
    //Ek --> total kinetic
    energy_state.kinetic_energy = energy_state.linear_kinetic_energy + energy_state.rotational_kinetic_energy;
    //Epg (Y value in vectoring is height)
    energy_state.gravitational_potential_energy = rigid_body -> mass * fabsf (gravitational_acceleration.y) * rigid_body -> position.y;
    //Eps --> calculated on a per spring basis, not included
    energy_state.spring_potential_energy = 0.0f;
    //Em --> total MEC
    energy_state.mechanical_energy = energy_state.kinetic_energy + energy_state.gravitational_potential_energy + energy_state.spring_potential_energy;
    return energy_state;
}
#endif

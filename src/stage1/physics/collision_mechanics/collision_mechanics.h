#include <stdio.h>
#include <math.h>
#include "../../math/math3D.h"
#include "../../math_phys_buffer/buffer.h"
#include "../forces/defines_majority_forces/define_forces.h"
#ifndef collisions_h
#define collisions_h
typedef struct {
    rigidbody *object_a;
    rigidbody *object_b;
    vector3 normal_vector; //Normal direction from obj A to obj B
    vector3 contact_point; //Position on both objects of contact
    float penetration_contact; //Overlap amount
} collision_data;
//Detection of actual collision (Sphere to Sphere contact)
static bool collision_dual_sphere (rigidbody *rigidbody_object_a, rigidbody *rigidbody_object_b, collision_data *collision_output_data) {
    vector3 relative_position_vector = vector3_subtraction (rigidbody_object_b -> position, rigidbody_object_a -> position);
    float distance_between_centres_squared = vector3_length_squared (relative_position_vector);
    float total_combined_radius = rigidbody_object_a -> radius + rigidbody_object_b -> radius;
    //Check Radial discrepancy
    if (distance_between_centres_squared >= total_combined_radius * total_combined_radius) {return false;}
    float distance_between_centres = sqrt (distance_between_centres_squared);
    collision_output_data -> object_a = rigidbody_object_a;
    collision_output_data -> object_b = rigidbody_object_b;
    //Normal = Vector from A to B
    //Safety Check (Div zero)
    const float minimum_distance_threshold_epsilon = 0.0001f;
    if (distance_between_centres > minimum_distance_threshold_epsilon) {
        collision_output_data -> normal_vector = vector3_scaling (relative_position_vector, 1.0f / distance_between_centres);
    } else {
        //Fallback Normal (Overlapping)
        collision_output_data -> normal_vector = (vector3) {0.0f, 1.0f, 0.0f};
    } collision_output_data -> penetration_contact = total_combined_radius - distance_between_centres;
    //Contact point between objects
    collision_output_data -> contact_point = vector3_addition (rigidbody_object_a -> position, vector3_scaling (collision_output_data -> normal_vector, rigidbody_object_a -> radius));
    return true;
} //Resolution (Impulse-Momemtum) (x, y, z directional vectoring for impulse related calculations)
static void collision_resolve (collision_data *collision) {
    rigidbody *object_a = collision -> object_a, *object_b = collision -> object_b;
    //Calculate the vectors from the centre of mass to the contact point between objects (rad_a, and rad_b)]
    vector3 relative_contact_vector_a = vector3_subtraction (collision -> contact_point, object_a -> position);
    vector3 relative_contact_vector_b = vector3_subtraction (collision -> contact_point, object_b -> position);
    //Relative Velocity of the system at the point of contact
    vector3 velocity_a_at_contact = vector3_addition (object_a -> velocity, vector3_cross (object_a -> angular_velocity, relative_contact_vector_a)); //Compute Same change of velocity stemming from angualr velocity change for object A
    vector3 velocity_b_at_contact = vector3_addition (object_b -> velocity, vector3_cross (object_b -> angular_velocity, relative_contact_vector_b)); //Compute change of angular velocity change of the object sphere for object B and combine with already existing velocity
    vector3 relative_velocity = vector3_subtraction (velocity_b_at_contact, velocity_a_at_contact); //Compute Net Velocity of system (both objects)
    //Velocity along the normal line of vector connecting objects to contact point
    float relative_velocity_dot_normal = vector3_dot (relative_velocity, collision -> normal_vector);
    //If objects are moving apart/away, no need for implement
    if (relative_velocity_dot_normal > 0) {return;}
    //Calculate Impulse on Scalar (delta P)
    //Delta P = (-1(1 + e) * vrdn) / ( ((mass_a) ^ -1) + ((mass_b) ^ -1) + (n * (((impulse_a) ^ -1 * ra * normal_vector * ra) + ((impulse_b) ^ -1 * rb * normal_vector * rb))))
    float restitution_coefficient = fminf (object_a -> restitution, object_b -> restitution);
    //Linear Components
    float inverse_mass_sum = object_a -> inverse_mass + object_b -> inverse_mass;
    //Rotational Components (Inertial resistance to rotational momemtum experiences by getting hit by other object)
    vector3 ra_cross_normal = vector3_cross (relative_contact_vector_a, collision -> normal_vector);
    vector3 rb_cross_normal = vector3_cross (relative_contact_vector_b, collision -> normal_vector);
    vector3 angular_motion_component_a = vector3_cross (math3_multiplication_vector3 (object_a -> inverse_inertia_system, ra_cross_normal), relative_contact_vector_a);
    vector3 angular_motion_component_b = vector3_cross (math3_multiplication_vector3 (object_b -> inverse_inertia_system, rb_cross_normal), relative_contact_vector_b);
    float rotational_resistance_sum = vector3_dot (vector3_addition (angular_motion_component_a, angular_motion_component_b), collision -> normal_vector);
    float impulse_denominator = inverse_mass_sum + rotational_resistance_sum;
    if (impulse_denominator <= 0.0f) {return;}
    float impulse_scalar = (-(1.0f + restitution_coefficient) * relative_velocity_dot_normal) / impulse_denominator;
    //Apply Impulse to objects
    vector3 impulse_vector = vector3_scaling (collision -> normal_vector, impulse_scalar);
    //Linear Velocity Changes: (delta v = impulse * mass ^ -1)
    object_a -> velocity = vector3_subtraction (object_a -> velocity, vector3_scaling (impulse_vector, object_a -> inverse_mass)); //Add current Velocity to delta v (impulse * mass ^ -1) (Object A)
    object_b -> velocity = vector3_addition (object_b -> velocity, vector3_scaling (impulse_vector, object_b -> inverse_mass)); //Add current Velocity to delta v (impulse * mass ^ -1) (Object B)
    //Changes to angular velocity: (delta angular_v = I ^ -1 * (r * impulse_scalar))
    //I ^ -1: inverse of the moment of inertia (tensor)
    vector3 a_angular_impulse = vector3_cross (relative_contact_vector_a, vector3_scaling (impulse_vector, -1.0f)); //Impulse scalar for object a
    vector3 b_angular_impulse = vector3_cross (relative_contact_vector_b, impulse_vector); //impulse_scalar for object b
    object_a -> angular_velocity = vector3_addition (object_a -> angular_velocity, math3_multiplication_vector3 (object_a -> inverse_inertia_system, a_angular_impulse)); //Delta angular_v + current angular_v = final angular_v (Object A)
    object_b -> angular_velocity = vector3_addition (object_b -> angular_velocity, math3_multiplication_vector3 (object_b -> inverse_inertia_system, b_angular_impulse)); //Delta angular_v + current_angualr_v = final angular_v (Object B)
    //Correct Position Change Values (FPU error, results in sinking of objects into each other)
    const float error_correction_percent = 0.2f; //20% correction per frame and motion calculated
    const float penetration_allowance_slop = 0.01f; //Allowance for object overlap (penetration, sinking)
    //Correction: Push back proportions by 20% for dropping beneath mesh
    if (inverse_mass_sum <= 0.0f) {return;}
    vector3 position_correction = vector3_scaling (collision -> normal_vector, (fmaxf (collision -> penetration_contact - penetration_allowance_slop, 0.0f)) / inverse_mass_sum * error_correction_percent);
    object_a -> position = vector3_subtraction (object_a -> position, vector3_scaling (position_correction, object_a -> inverse_mass));
    object_b -> position = vector3_addition (object_b -> position, vector3_scaling (position_correction, object_b -> inverse_mass));
}
#endif

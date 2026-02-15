#include <stdio.h>
#include <math.h>
#include "../../math/math3D.h"
#include "../../math_phys_buffer/buffer.h"
#include "../forces/define_forces.h"

#ifndef collisions_h
#define collisions_h

typedef struct {
    rigidbody *a;
    rigidbody *b;
    vector3 normal_vector; //Normal direction from obj A to obj B
    vector3 contact_point; //Position on both objects of contact
    float penetration_contact; //Overlap amount
} collision_data;

//Detection of actual collision (Sphere to Sphere contact)
bool collision_dual_sphere (rigidbody *a, rigidbody *b, collision_data *out) {
    vector3 relative_position = vector3_subtraction (b->position, a->position);
    float distance_squared = vector3_length_squared (relative_position);
    float radius_total = a->radius + b->radius;
    //Check Radial discrepancy
    if (distance_squared >= radius_total * radius_total) {return false;}
    float distance = sqrtf (distance_squared);
    out->a = a;
    out->b = b; 
    //Normal = Vector from A to B
    if (distance > 0) {out->normal = vector3_scaling (relative_position, 1.0 / distance);}
    else {(vector3) {0, 1, 0};}
    out->penetration = radius_total - distance;
    //Contact point between the objects is in between the two centre points
    out->contact_point = vector3_addition (a->position, vector3_scaling (out->normal_vector, a->radius));
    return true;
}

//Resolution (Impulse-Momemtum) (x, y, z directional vectoring for impulse related calculations)
void collision_resolve (collision_data *c) {
    rigidbody *a = c->a, *b = c->b;
    //Calculate the vectors from the centre of mass to the contact point between objects (rad_a, and rad_b)]
    vector3 ra = vector3_subtraction (c->contact_point, a->position);
    vector3 rb = vector3_subtraction (c->contact_point, b->position);
    //Relative Velocity of the system at the point of contact
    vector3 velocity_a_at_contact = vector3_addition (a->velocity, vector3_cross (a->angular_velocity, ra)); //Compute Same change of velocity stemming from angualr velocity change for object A
    vector3 velocity_b_at_contact = vector3_addition (b->velocity, vector3_cross (b->angular_velocity, rb)); //Compute change of angular velocity change of the object sphere for object B and combine with already existing velocity
    vector3 relative_velocity = vector3_subtraction (velocity_b_at_contact, velocity_a_at_contact); //Compute Net Velocity of system (both objects)
    //Velocity along the normal line of vector connecting objects to contact point
    float velocity_relative_dot_normal = vector3_dot (relative_velocity, c->normal);
    //If objects are moving apart/away, no need for implement
    if (velocity_relative_dot_normal > 0) {return;} //variable abbrev vrdn
    //Calculate Impulse on Scalar (delta P)
    //Delta P = (-1(1 + e) * vrdn) / ( ((mass_a) ^ -1) + ((mass_b) ^ -1) + (n * (((impulse_a) ^ -1 * ra * normal_vector * ra) + ((impulse_b) ^ -1 * rb * normal_vector * rb))))
    float e = fminf (a->restitution, b-> restitution);
    //Linear Components
    float inverse_mass_sum = a->inverse_mass + b->inverse_mass;
    //Rotational Components (Inertial resistance to rotational momemtum experiences by getting hit by other object)
    vector3 ra_cross_normal = vector3_cross (ra, c->normal);
    vector3 rb_cross_normal = vector3_cross (rb, c->normal);
    vector3 angular_mot_a = vector3_cross (math3_multiplication_vector3 (a->inverse_inertia_system, ra_cross_normal));
    vector3 angular_mot_b = vector3_cross (math3_multiplication_vector3 (b->inverse_inertia_system, rb_cross_normal));
    float rotational_termination = vector3_dot (vector3_addition (angular_mot_a, angular_mot_b), c->normal);
    float j = ((-1.0 + e) * velocity_relative_dot_normal) / (inverse_mass_sum + rotational_termination);
    //Apply Impulse to objects
    vector3 impulse_vector = vector3_scaling (c->normal, j);
    //Linear Velocity Changes: (delta v = impulse * mass ^ -1)
    a->velocity = vector3_subtraction (a->velocity, vector3_scale (impulse_vector, a->inverse_mass)); //Add current Velocity to delta v (impulse * mass ^ -1) (Object A)                                                                                                 
    b->velocity = vector3_addition (b->velocity, vector3_scale (impulse_vector, b->inverse_mass)); //Add current Velocity to delta v (impulse * mass ^ -1) (Object B)
    //Changes to angular velocity: (delta angular_v = I ^ -1 * (r * impulse_scalar))
    //I ^ -1: inverse of the moment of inertia (tensor)
    vector3 a_angular_impulse = vector3_cross (ra, vector3_scaling (impulse_vector, -1.0)); //Impulse scalar for object a
    vector3 b_angular_impulse = vector3_cross (rb, impulse_vector); //impulse_scalar for object b
    a->angular_velocity = vector3_addition (a->angular_velocity, math3_multiplication_vector3 (a->inverse_inertia_system, a_angular_impulse)); //Delta angular_v + current angular_v = final angular_v (Object A)
    b->angular_velocity = vector3_addition (a->angular_velocity, math3_multiplication_vector3 (b->inverse_inertia_system, b_angular_impulse)); //Delta angular_v + current_angualr_v = final angular_v (Object B)
    //Correct Position Change Values (FPU error, results in sinking of objects into each other)
    const float error_percent = 0.2; //20% correction per frame and motion calculated
    const float slop = 0.01; //Allowance for object overlap (penetration, sinking)
    //Correction: Push back each proportion by a certain amount (20%) for each "sink"
    vector3 correction = vector3_scaling (c->normal, (fmaxf (c->penetration_contact - slop, 0.0)) * (a->inverse_mass + b->inverse_mass) * error_percent);
    a->position = vector3_subtraction (a->position, vector3_scaling (correction, a->inverse_mass));    
    b->position = vector3_subtraction (b->position, vector3_scaling (correction, b->inverse_mass));
}
#endif

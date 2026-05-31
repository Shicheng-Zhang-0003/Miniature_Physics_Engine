#ifndef forces_h
#define forces_h
#include <stdio.h>
#include <math.h>
#include "../../../math/math3D.h"
#include "../../../math_phys_buffer/buffer.h"
//Universal Law of Gravitation
#define big_g 6.67430e-11f
typedef struct {float linear_kinetic_energy, rotational_kinetic_energy, kinetic_energy, gravitational_potential_energy, spring_potential_energy, mechanical_energy;} state_energy;
//Gravity (Regular and Sloped Parallel Gravity)
void force_applicant_gravity_normal (rigidbody *rigid_body, vector3 gravitational_acceleration, vector3 surface_normal);
//Universal Law of Gravitation
void force_applicant_universal_gravity (rigidbody *rigid_body_a, rigidbody *rigid_body_b);
//Friction Definition (3D tangent plane fields)
void force_applicant_friction_rolling (rigidbody *rigid_body, vector3 surface_normal, float static_friction_coefficient, float kinetic_friction_coefficient, float gravity_y);
void force_applicant_friction (rigidbody *rigid_body, vector3 surface_normal, float static_friction_coefficient, float kinetic_friction_coefficient, float gravity_y);
//Spring Force, Tension, Hooke Law
void force_applicant_string (rigidbody *rigid_body, vector3 anchor_position, float equilibrium_length, float spring_constant, float damping_coefficient);
//Vertical Circular Motion
void force_applicant_vertical_anchor (rigidbody *rigid_body, vector3 pivot_point, float radius, float gravity_y);
//Monitor the Energy component of the objects related
state_energy force_to_system_energy_amount (rigidbody *rigid_body, vector3 gravitational_acceleration);
#endif

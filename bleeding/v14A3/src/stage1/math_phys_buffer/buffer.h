#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "../math/math3D.h"
#ifndef buffer_h
#define buffer_h
typedef enum {object_sphere, object_cube} object_type;
typedef struct {
    //Linear Kinematics
    vector3 position, velocity, acceleration;
    //Rotational Motion
    vector4 orientation;
    vector3 angular_velocity, angular_acceleration;
    //Dynamics (Properties)
    float mass, inverse_mass, restitution;
    //Inertial Tensor
    math3 inertia_tensor_local, inverse_inertia_tensor_local, inverse_inertia_system;
    //Force and Torque accumulation
    vector3 force_accumulator, torque_accumulator;
    //Dimensions
    float radius;
    bool static_state;
    float friction_static, friction_kinetic;
    vector3 colour;
    object_type type;
    //Cube Specific Variables
    vector3 half_extensions;
    vector3 cached_axes [3];
    //v1.2 Sleeping Bodies
    bool is_sleeping;
    float sleep_timer;
} rigidbody;
void rigidbody_update_axes (rigidbody *rigid_body);
void rigidbody_initialisation_sphere (rigidbody *rigid_body, float radius, float mass, vector3 position_input);
void rigidbody_update_inertia_sphere (rigidbody *rigid_body);
void rigidbody_update_inertia_cube (rigidbody *rigid_body);
void rb_apply_forces_perfect (rigidbody *rigid_body, vector3 force_applied);
void rb_apply_forces_localised (rigidbody *rigid_body, vector3 force_applied, vector3 locale_impact);
float rb_get_kinetic_energy (rigidbody *rigid_body);
void rb_integrate (rigidbody *rigid_body, float delta_time, float linear_damping, float angular_damping);
vector3 make_half_extents (float width, float height, float depth);
void rigidbody_initialisation_cube (rigidbody *rigid_body, vector3 position_input, vector3 half_extensions, float mass);
void rigidbody_wake (rigidbody *rigid_body);
#endif

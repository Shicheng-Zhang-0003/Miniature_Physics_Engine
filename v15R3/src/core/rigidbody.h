#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "math3D.h"
#include "math4_special.h"
#ifndef rigidbody_h
#define rigidbody_h
typedef enum { object_sphere, object_cube, object_cylinder } object_type; /* MPE_FTC_090 */
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
    float cylinder_half_length; /* MPE_FTC_090: half-length along axle (X) */
    bool static_state;
    float friction_static, friction_kinetic;
    vector3 colour;
    object_type type;
    //Cube Specific Variables
    vector3 half_extensions;
    vector3 cached_axes[3];
    /* MPE_TASK_15_AXIS_CACHE_FIELD_BEGIN */
    vector4 cached_axes_orientation;
    /* MPE_TASK_15_AXIS_CACHE_FIELD_END */
    //v1.2 Sleeping Bodies
    bool is_sleeping;
    float sleep_timer;
    /* MPE_TASK_V15R2_NICE_VALUE_BEGIN */
    int nice_value; /* -20 to +19, default 0. Positive = extra damping. */
    /* MPE_TASK_V15R2_NICE_VALUE_END */
    /* MFS_MECANUM_FRICTION: anisotropic roller friction support */
    bool is_mecanum;          /* true = mecanum wheel with angled rollers */
    float roller_angle_rad;
bool driven_this_tick;  /* MFS_169: set when motor torque applied this tick */   /* roller angle from axle (X axis), typically ±45° */
    uint32_t object_id;
    uint32_t object_generation;
} rigidbody;
void rigidbody_update_axes(rigidbody *rigid_body);
void rigidbody_initialisation_sphere(rigidbody *rigid_body, float radius, float mass, vector3 position_input);
void rigidbody_update_inertia_sphere(rigidbody *rigid_body);
void rigidbody_update_inertia_cube(rigidbody *rigid_body);
void rigidbody_initialisation_cylinder(rigidbody *rigid_body, float radius, float half_length, float mass, vector3 position_input); /* MPE_FTC_090 */
void rigidbody_update_inertia_cylinder(rigidbody *rigid_body); /* MPE_FTC_090 */
void rb_apply_forces_perfect(rigidbody *rigid_body, vector3 force_applied);
void rb_apply_forces_localised(rigidbody *rigid_body, vector3 force_applied, vector3 locale_impact);
float rb_get_kinetic_energy(rigidbody *rigid_body);
vector3 make_half_extents(float width, float height, float depth);
void rigidbody_initialisation_cube(rigidbody *rigid_body, vector3 position_input, vector3 half_extensions, float mass);
void rigidbody_wake(rigidbody *rigid_body);

void rigidbody_sanitize(rigidbody *rigid_body);
void rigidbody_set_static(rigidbody *rigid_body, bool make_static);

void rb_integrate_velocity(rigidbody *rigid_body, float delta_time, float linear_damping, float angular_damping);
void rb_integrate_position(rigidbody *rigid_body, float delta_time);
void rigidbody_set_mecanum(rigidbody *rb, bool enable, float roller_angle_rad);
#endif
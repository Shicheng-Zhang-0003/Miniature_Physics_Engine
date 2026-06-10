#ifndef buffer_h
#define buffer_h

#include <stdio.h>
#include <math.h>

// Structures required
typedef enum { object_sphere, object_cube } object_type;

typedef struct {
    // Linear Kinematics
    vector3 position, velocity, acceleration;
    // Rotational Motion
    vector4 orientation;
    vector3 angular_velocity, angular_acceleration;
    // Dynamics (Properties)
    float mass, inverse_mass, restitution;
    // 2 = mass ^ -1 for efficient calculation, 3 = bounce
    // Inertial Tensor (Rotational Mass Calculations)
    math3 inertia_tensor_local, inverse_inertia_tensor_local, inverse_inertia_system;
    // Force and Torque accumulation
    vector3 force_accumulator, torque_accumulator;
    // Dimensions of individual object definition (for collisions and rendering)
    float radius; // Spherical calculation
    bool static_state; // If set to true object is naturally immobile
    float friction_static, friction_kinetic;
    vector3 colour;
    // Type of object
    object_type type;
    // Cube Specific Variables
    vector3 half_extensions;
    vector3 cached_axes [3];
} rigidbody;

// Helper to update axes from orientation
void rigidbody_update_axes (rigidbody *rigid_body);

// Init Spherical Rigid Body
void rigidbody_initialisation_sphere (rigidbody *rigid_body, float radius, float mass, vector3 position_input);

// Update Inertia Tensor after mass/radius change
void rigidbody_update_inertia_sphere (rigidbody *rigid_body);

// Update Inertia Tensor after cube mass change
void rigidbody_update_inertia_cube (rigidbody *rigid_body);

// Apply a force at a centre of mass (perfect collision movement, linear movement only defined)
void rb_apply_forces_perfect (rigidbody *rigid_body, vector3 force_applied);

// Apply force at a point not the centre of mass (which generates rotational motion and torque)
void rb_apply_forces_localised (rigidbody *rigid_body, vector3 force_applied, vector3 locale_impact);

// Energy Computation
float rb_get_kinetic_energy (rigidbody *rigid_body);

// Integration Segmentation (Movement Compute)
void rb_integrate (rigidbody *rigid_body, float delta_time, float linear_damping, float angular_damping);

// Half Extent Helper
vector3 make_half_extents (float width, float height, float depth);

// Initialize a cube: Box, OBB
void rigidbody_initialisation_cube (rigidbody *rigid_body, vector3 position_input, vector3 half_extensions, float mass);

#endif // buffer_h

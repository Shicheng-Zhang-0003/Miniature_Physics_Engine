#include <stdio.h>
#include <math.h>
#include "../math/math3D.h"
#ifndef buffer_h
#define buffer_h
//Structures required
typedef struct {
    //Linear Kinematics
    vector3 position, velocity, acceleration;
    //Rotational Motion
    vector4 orientation;
    vector3 angular_velocity, angular_acceleration;
    //Dynamics (Properties)
    float mass, inverse_mass, restitution;
    //2 = mass ^ -1 for efficient calculation, 3 = bounce
    //Inertial Tensor (Rotational Mass Calculations)
    math3 inertia_tensor_local, inverse_inertia_system;
    //Force and Torque accumilation
    vector3 force_accumilator, torque_accumilator;
    //Dimensions of individual object definition (for collisions and rendering)
    float radius; //Spherical calculation
    bool static_state; //If set to true object is naturally immobile
    vector3 colour;
} rigidbody;
//Init
static void rigidbody_initialisation_sphere (rigidbody *rb, float radius, float mass, vector3 position_input) {
    //Kinematic
    rb->position = position_input;
    rb->velocity = vector3_zero ();
    rb->acceleration = vector3_zero ();
    rb->orientation = vector4_identity ();
    rb->angular_velocity = vector3_zero ();
    rb->angular_acceleration = vector3_zero ();
    rb->colour = (vector3) {0.2, 0.6, 1.0};
    //Dynamic
    rb->mass = mass;
    if (mass > 0) {rb->inverse_mass = 1.0 / mass;}
    else {rb->inverse_mass = 0.0;}
    rb->radius = radius;
    rb->restitution = 0.5; //Default Bounce Energy Return
    rb->static_state = (mass == 0); //Static Objects
    //Inertial Tensors
    //I = 0.4mr ^ 2
    float inertia_coefficient_sphere = (0.4) * mass * radius * radius;
    rb->inertia_tensor_local = (math3) {{{0}}};
    rb->inertia_tensor_local.matrix [0][0] = inertia_coefficient_sphere;
    rb->inertia_tensor_local.matrix [1][1] = inertia_coefficient_sphere;
    rb->inertia_tensor_local.matrix [2][2] = inertia_coefficient_sphere;
    //Initialize Inverse Inertia System
    if (mass > 0) {
        rb->inverse_inertia_system = math3_inverse (rb->inertia_tensor_local);
    } else {
        rb->inverse_inertia_system = (math3) {{{0}}};
    }
    //Total Force and Torque accumilation
    rb->force_accumilator = vector3_zero ();
    rb->torque_accumilator = vector3_zero ();
} //Force application and Torque Dynamics
//Apply a force at a centre of mass (perfect collision movement, linear movement only defined)
static void rb_apply_forces_perfect (rigidbody *rb, vector3 force_applied) {
    if (rb->static_state) {return;}
    rb->force_accumilator = vector3_addition (rb->force_accumilator, force_applied); //Force applied to torque and circular momentum
} //Apply force at a point not the centre of mass (which generates rotational motion and torque)
//locale_impact = impact point on object identified
static void rb_apply_forces_localised (rigidbody *rb, vector3 force_applied, vector3 locale_impact) {
    if (rb->static_state) {return;}
    rb_apply_forces_perfect (rb, force_applied);
    //Torque = r * F (r = vector from Centre of Mass to the point of actual contact between objects)
    vector3 r = vector3_subtraction (locale_impact, rb->position);
    vector3 torque_actual = vector3_cross (r, force_applied);
    rb->torque_accumilator = vector3_addition (rb->torque_accumilator, torque_actual);
} //Energy Computation
static float rb_get_Ek (rigidbody *rb) {
    //EK normal = 0.5mv ^ 2
    float linear_ek = 0.5 * rb->mass * vector3_length_squared (rb->velocity);
    //EK rotational = 0.5wIw
    vector3 angular_momemtum = math3_multiplication_vector3 (math3_inverse (rb->inverse_inertia_system), rb->angular_velocity);
    float rotational_ek = 0.5 * vector3_dot (rb->angular_velocity, angular_momemtum);
    return linear_ek + rotational_ek;
} //Integration Segmentation (Movement Compute)
static void rb_integrate (rigidbody *rb, float dt) {
    if ((rb->static_state) || (dt <= 0.0)) {return;}
    //Update inverse inertia tensor based on current orientation before using it
    //inverse_inertia_system = rotational * inverse_inertia_local * transposed value in 4D rotational axis
    math3 rotation_matrix_current = vector4_to_math3 (rb->orientation); //W axis orientation of rotation
    math3 rotation_matrix_transposed = math3_transposition (rotation_matrix_current);
    math3 inertia_tensor_inverse_local = math3_inverse (rb->inertia_tensor_local); //Inverse Local Inertia
    rb->inverse_inertia_system = math3_multiplication (rotation_matrix_current, math3_multiplication (inertia_tensor_inverse_local, rotation_matrix_transposed));
    //Calculate Linear Acceleration (F = ma, a = Fm ^ -1)
    rb->acceleration = vector3_scaling (rb->force_accumilator, rb->inverse_mass); //Multiply Force by inverse of mass
    //Calculate Instantaneous Velocity
    rb->velocity = vector3_addition (rb->velocity, vector3_scaling (rb->acceleration, dt)); //Add currenty velocity to delta v
    //Damping (Air Resistance)
    float linear_velocity_damping_factor = pow (0.99, dt);
    rb->velocity = vector3_scaling (rb->velocity, linear_velocity_damping_factor);
    //Calculate Position Standard
    rb->position = vector3_addition (rb->position, vector3_scaling (rb->velocity, dt)); //Add current position to delta d
    //Update Angular Acceleration Standard
    rb->angular_acceleration = math3_multiplication_vector3 (rb->inverse_inertia_system, rb->torque_accumilator);
    //Update Standard Angular Velocity
    rb->angular_velocity = vector3_addition (rb->angular_velocity, vector3_scaling (rb->angular_acceleration, dt)); //Sum of current angular velocity by delta angular velocity
    //Angular Damping
    float angular_velocity_damping_factor = pow (0.99, dt);
    rb->angular_velocity = vector3_scaling (rb->angular_velocity, angular_velocity_damping_factor);
    //Update General Orientation (4D)
    //delta_q = [0, w-axis_values] * q * 0.5 * dt
    vector4 angular_velocity_quaternion = {0, rb->angular_velocity.x, rb->angular_velocity.y, rb->angular_velocity.z}; //Start with no w axis definition
    vector4 orientation_change_delta = vector4_multiplication (angular_velocity_quaternion, rb->orientation); //Orientation = W-Axis value
    //Set Orientation individually
    rb->orientation.w += orientation_change_delta.w * 0.5 * dt;
    rb->orientation.x += orientation_change_delta.x * 0.5 * dt;
    rb->orientation.y += orientation_change_delta.y * 0.5 * dt;
    rb->orientation.z += orientation_change_delta.z * 0.5 * dt;
    rb->orientation = vector4_normalisation (rb->orientation);
    //Clear accumilators of force and torque for next implementation
    rb->force_accumilator = vector3_zero ();
    rb->torque_accumilator = vector3_zero ();
}
#endif


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
} rigidbody;


//Init
void rigidbody_initialisation_sphere (rigidbody *rb, float radius, float mass, vector3 position_input) {
    //Kinematic
    rb->position = position_input;
    rb->velocity = vector3_zero ();
    rb->acceleration = vector3_zero ();
    rb->orientation = vector4_identity ();
    rb->angular_velocity = vector3_zero ();
    rb->angular_acceleration = vector3_zero ();
    //Dynamic
    rb->mass = mass;
    if (mass > 0) {rb->inverse_mass = 1.0 / mass;}
    else {rb->inverse_mass = 0.0;}
    rb->radius = radius;
    rb->restitution = 0.5; //Default Bounce Energy Return
    rb->static_state = (mass == 0); //Static Objects
    //Inertial Tensors
    //I = 0.4mr ^ 2
    float iner = (0.4f) * mass * radius * radius; //0.4f, not 0.4, for decimal float point accuracy
    rb->inertia_tensor_local = math3_identity (); 
    rb->inertia_tensor_local.m [0][0] = iner;
    rb->inertia_tensor_local.m [1][1] = iner;
    rb->inertia_tensor_local.m [2][2] = iner;
    //Total Force and Torque accumilation
    rb->force_accumilator = vector3_zero ();
    rb->torque_accumilator = vector3_zero ();
}

//Force application and Torque Dynamics
//Apply a force at a centre of mass (perfect collision movement, linear movement only defined)
void rb_apply_forces_perfect (rigidbody *rb, vector3 force_applied) {
    if (rb->static_state) {return;}
    rb->force_accumilator = vector3_addition (rb->force_accumilator, force); //Force applied to torque and circular momentum
} 

//Apply force at a point not the centre of mass (which generates rotational motion and torque)
//locale_impact = impact point on object identified
void rb_apply_forces_localised (rigidbody *rb, vector3 force_applied, vector3 locale_impact) {
    if (rb->static_state) {return;}
    rb_apply_forces_perfect (rb, force_applied);
    //Torque = r * F (r = vector from Centre of Mass to the point of actual contact between objects)
    vector3 r = vector3_subtraction (locale_impact, rb->position);
    vector3 torque_actual = vector3_cross (r, force_applied);
    rb->torque_accumilator = vector3_addition (rb->torque_accumilator, torque_actual);
}

//Energy Computation
float rb_get_Ek (rigidbody *rb) {
    //EK normal = 0.5mv ^ 2
    float linear_ek = 0.5 * rb->mass * vector3_length_squared (rb->velocity);
    //EK rotational = 0.5wIw
    vector3 angular_momemtum = math3_multiplication_vector3 (math3_inverse (rb->inverse_inertia_system), rb->angular_velocity);
    float rotational_ek = 0.5 * vector3_dot (rb->angular_velocity, angular_momemtum);
    return linear_ek + rotational_ek;
}

//Integration Segmentation (Movement Compute)
void rb_integrate (rigidbody *rb, float dt) {
    if ((rb->static_state) || (dt <= 0.0)) {return;}
    //Calculate Linear Acceleration (F = ma, a = Fm ^ -1)
    rb->acceleration = vector3_scaling (rb->force_accumilator, rb->inverse_mass); //Multiply Force by inverse of mass
    //Calculate Instantaneous Velocity
    rb->velocity = vector3_addition (rb->velocity, vector3_scaling (rb->acceleration, dt)); //Add currenty velocity to delta v
    //Calculate Position Standard
    rb->position = vector3_addition (rb->position, vector3_scaling (rb->velocity, dt)); //Add current position to delta d
    //Update Angular Acceleration Standard
    //P1: Update inverse inertia tensor: inverse_inertia_system = rotational * inverse_inertia_local * transposed value in 4D rotational axis
    math3 rotation = vector4_to_math3 (rb->orientation); //W axis orientation of rotation
    math3 rotation_t = math3_transposition (rotation);
    math3 inverse_local = math3_invserse (rb->inertia_tensor_local); //Inverse Local Inertia
    rb->inverse_inertia_system = math3_multiplication (rotation, math3_multiplication (inverse_local, rotationan_t));
    rb->angular_acceleration = math3_multiplication_vector3 (rb->inverse_inertia_system, rb->torque_accumilator);
    //Update Standard Angular Velocity
    rb->angular_velocity = vector3_addition (rb->angular_velocity, vector3_scaling (rb->angular_acceleration, dt)); //Sum of current angular velocity by delta angular velocity
    //Update General Orientation (4D)
    //delta_q = [0, w-axis_values] * q * 0.5 * dt
    vector4 w_quarternion = {0, rb->angular_velocity.x, rb->angular_velocity.y, rb->angular_velocity.z}; //Start with no w axis definition
    vector4 delta_q = vector4_multiplication (w_quarternion, rb->orientation); //Orientation = W-Axis value
    //Set Orientation individually
    rb->orientation.w += delta_q.w * 0.5 * dt; 
    rb->orientation.x += delta_q.x * 0.5 * dt;
    rb->orientation.y += delta_q.y * 0.5 * dt;
    rb->orientation.z += delta_q.z * 0.5 * dt;
    //Clear accumilators of force and torque for next implementation 
    rb->force_accumilator = vector3_zero ();
    rb->torque_accumilator = vector3_zero ();
}
#endif


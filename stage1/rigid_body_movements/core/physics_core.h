#include <stdio.h>
#include <math.h>
#include "~/Desktop/work/code/major/code/physics_engine/stage1/3D_integration/math3D.h"

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
    float radious; //Spherical calculation
    bool static_state; //If set to true object is naturally immobile
} rigidbody;


//Init
void rigidbody_initialisation_sphere (rigidbody * rb, float radius, float mass, vector3 position_input) {
    rb->position = position_input;
    rb->velocity = vector3_zero ();
    rb->acceleration = vector3_zero ();
    rb->orientation = vector4_identity (); 
}

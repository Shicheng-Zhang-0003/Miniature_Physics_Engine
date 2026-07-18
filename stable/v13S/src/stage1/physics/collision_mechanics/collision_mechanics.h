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
bool collision_dual_sphere (rigidbody *rigidbody_object_a, rigidbody *rigidbody_object_b, collision_data *collision_output_data);
//Project OBB onto axis renders
float project_obb (rigidbody *rigid_body, vector3 axis, vector3 axes [3]);
//Detection of Sphere to Cube collisions, OBB
bool collision_sphere_cube (rigidbody *sphere, rigidbody *cube, collision_data *collision_output_data);
//Cube to Cube OBB-OBB collision detection using SAT
bool collision_dual_cube (rigidbody *cube_a, rigidbody *cube_b, collision_data *collision_output_data);
//Resolution (Impulse-Momemtum) (x, y, z directional vectoring for impulse related calculations)
void collision_resolve (collision_data *collision);
#endif

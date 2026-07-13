#include <stdio.h>
#include <math.h>
#include "../../math/math3D.h"
#include "../../math_phys_buffer/buffer.h"
#include "../forces/defines_majority_forces/define_forces.h"
#ifndef collisions_h
#define collisions_h
typedef struct {
    vector3 position;
    float penetration;
    vector3 local_position_a;
    vector3 local_position_b;
    float accumulated_normal_impulse;
    float accumulated_tangent_impulse;
    float effective_mass_normal;
    float effective_mass_tangent;
    vector3 tangent_vector;
    float restitution_bias;
    vector3 ra;
    vector3 rb;
} contact_point_data;

typedef struct {
    rigidbody *object_a;
    rigidbody *object_b;
    vector3 normal_vector;
    contact_point_data contacts [4];
    int contact_count;
} collision_data;
bool collision_dual_sphere (rigidbody *rigidbody_object_a, rigidbody *rigidbody_object_b, collision_data *collision_output_data);
float project_obb (rigidbody *rigid_body, vector3 axis, vector3 axes [3]);
bool collision_sphere_cube (rigidbody *sphere, rigidbody *cube, collision_data *collision_output_data);
bool collision_dual_cube (rigidbody *cube_a, rigidbody *cube_b, collision_data *collision_output_data);
void collision_resolve (collision_data *collision);
void collision_prepare_solver (collision_data *source, collision_data *manifold_entry);
void collision_resolve_iterative (collision_data *manifold_entry);
void contact_cache_save (collision_data *manifolds, int count);
#endif

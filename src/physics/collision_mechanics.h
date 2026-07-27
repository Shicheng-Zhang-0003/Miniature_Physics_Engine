#include <stdio.h>
#include <math.h>
#include "../core/math3D.h"
#include "../core/rigidbody.h"
#include "define_forces.h"
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
    vector3 cached_tangent;
    float restitution_bias;
    float separation_bias;
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
void collision_prepare_solver (collision_data *source, collision_data *manifold_entry);
void collision_resolve_iterative (collision_data *manifold_entry);
void contact_cache_save (collision_data *manifolds, int count);
void contact_cache_clear (void);


bool collision_static_plane_sphere (rigidbody *sphere, float plane_y, collision_data *collision_output_data);
bool collision_static_plane_cube (rigidbody *cube, float plane_y, collision_data *collision_output_data);
bool collision_static_plane_body (rigidbody *body, float plane_y, collision_data *collision_output_data);

/* A3_PATCH_36_DEBUG_COUNTERS */
void contact_cache_stats_reset (void);
int contact_cache_get_hits (void);
int contact_cache_get_misses (void);
#endif

#ifndef object_spawner_h
#define object_spawner_h
#include "../core/math3D.h"
#include "../core/rigidbody.h"


void spawner_launch_sphere (float spherical_radius, float physical_mass, float launch_speed);
void spawner_static_sphere (float spherical_radius, float physical_mass, vector3 static_position);
void spawner_launch_cube (vector3 position, vector3 half_extensions, float physical_mass);
void spawner_static_cube (vector3 position, vector3 half_extensions, float physical_mass);
#endif

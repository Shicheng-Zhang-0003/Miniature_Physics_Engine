#ifndef object_spawner_h
#define object_spawner_h
#include "../../../stage1/master_header.h"
#include "../../../stage2/master_header_2.h"
#include "../../../stage3/master_header_3.h"
extern float spawn_mass;
extern float spawn_radius;
extern float spawn_cube_mass;
extern float spawn_cube_extent;
extern float spawn_speed;
extern float friction_static;
extern float friction_kinetic;
extern float spawn_cube_side_length;
//Spawn a sphere at the position of the camera, and fire in a vector exactly equal to the vector of camera view
void spawner_launch_sphere (float spherical_radius, float physical_mass, float launch_speed);
//Spawn a static sphere (floating sphere), fixed position
void spawner_static_sphere (float spherical_radius, float physical_mass, vector3 static_position);
//Spawn a cube at the position of the camera, same firing mechanism as sphere launch_speed
void spawner_launch_cube (vector3 position, vector3 half_extensions, float physical_mass);
//Spawn a static cube at the position of the camera
void spawner_static_cube (vector3 position, vector3 half_extensions, float physical_mass);
#endif

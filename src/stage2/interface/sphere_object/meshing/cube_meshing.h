#ifndef cube_meshing_h
#define cube_meshing_h
#include "sphere_meshing.h"
extern mesh cube_mesh;
void cube_meshing_init (void);
void render_cube_object (mesh *mesh_object, rigidbody *rigid_body);
#endif

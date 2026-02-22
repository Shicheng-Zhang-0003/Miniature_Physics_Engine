#ifndef sphere_mesh_h
#define sphere_mesh_h
#include <epoxy/gl.h>
#include "../../master_header_2.h"
#include "../../../stage1/master_header.h"
//GLuint = GPU buffer ID
typedef struct {
    GLuint vao, vbo, ebo;
    //vao = vertex array object, organisation
    //vbo = vertex buffer object, coords
    //ebo = element buffer object, index store and category
    int index; //Number of points to draw
} mesh;
//Function Pre-declaration
void init_sm_system (mesh *mesh_s, int sections, int stack);
void render_sphere_object (mesh * mesh_s, rigidbody *rb, math3 projections, math3 viewpoint);

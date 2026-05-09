#ifndef sphere_mesh_h
#define sphere_mesh_h
#include <epoxy/gl.h>
#include "stage1/master_header.h"
#include "stage2/master_header_2.h"
//GLuint = GPU buffer ID
typedef struct {
    GLuint vao, vbo, ebo;
    //vao = vertex array object, organisation
    //vbo = vertex buffer object, coords
    //ebo = element buffer object, index store and category
    int index; //Number of points to draw
    GLuint wireframe_ebo;
    int wireframe_index_count;
} mesh;
extern mesh sphere_mesh;
//Function Pre-declaration
void init_sm_system (mesh *mesh_s, int sections, int stack);
void render_sphere_object (mesh *mesh_s, rigidbody *rb);
#endif

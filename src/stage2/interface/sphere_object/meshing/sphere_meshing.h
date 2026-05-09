#ifndef sphere_mesh_h
#define sphere_mesh_h
#include <epoxy/gl.h>
#include "stage1/master_header.h"
#include "stage2/master_header_2.h"
//GLuint = GPU buffer ID
typedef struct {
    GLuint vertex_array_object;
    GLuint vertex_buffer_object;
    GLuint element_buffer_object;
    //vao = vertex array object, organisation
    //vbo = vertex buffer object, coords
    //ebo = element buffer object, index store and category
    int index_count; //Number of points to draw
    GLuint wireframe_element_buffer_object;
    int wireframe_index_count;
} mesh;
extern mesh sphere_mesh;
//Function Pre-declaration
void init_sm_system (mesh *mesh_object, int horizontal_sections, int vertical_stacks);
void render_sphere_object (mesh *mesh_object, rigidbody *rigid_body);
#endif

#ifndef sphere_mesh_h
#define sphere_mesh_h
#include <epoxy/gl.h>
#include "stage1/master_header.h"

typedef struct {
    GLuint vertex_array_object;
    GLuint vertex_buffer_object;
    GLuint element_buffer_object;
    int index_count;
    GLuint wireframe_element_buffer_object;
    int wireframe_index_count;
    GLuint instance_vbo;
    int instance_capacity;
} mesh;

extern mesh sphere_mesh;
void init_sm_system (mesh *mesh_object, int horizontal_sections, int vertical_stacks);
void render_sphere_object (mesh *mesh_object, rigidbody *rigid_body);
#endif

#ifndef grid_h
#define grid_h
#include <epoxy/gl.h>
#include "../../stage1/master_header.h"
#include "../../stage2/master_header_2.h"
typedef struct {
    GLuint vertex_array_object, vertex_buffer_object;
    int line_vertex_count;
} grid_mesh;
//Call after OpenGL calls are initialised
void grid_init (grid_mesh *grid_mesh_object, int half_extent, int cell_spacing);
//Call per render frame
void grid_render (grid_mesh *grid_mesh_object, GLuint shader_program, math4 view_matrix, math4 projection_matrix);
#endif

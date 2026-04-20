#ifndef grid_h
#define grid_h
#include <epoxy/gl.h>
#include "../../stage1/master_header.h"
#include "../../stage2/master_header_2.h"
typedef struct {
    GLuint vao, vbo;
    int line_count;
} grid_mesh;
//Call after OpenGL calls are initialised
void grid_init (grid_mesh *gm, int half_buffer, int spacing);
//Call per render frame
void grid_render (grid_mesh *gm, GLuint shader_program_call, math4 view, math4 projection);
#endif

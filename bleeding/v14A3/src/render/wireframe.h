#ifndef wireframe_h
#define wireframe_h
#include <epoxy/gl.h>
#include "../core/math3D.h"
#include "../core/math4_special.h"
#include "../core/buffer.h"

void wireframe_render_selected_object (GLuint shader_program, math4 view_matrix, math4 projection_matrix);
void wireframe_render_object (GLuint shader_program, math4 view_matrix, math4 projection_matrix, rigidbody *rigid_body, vector3 wireframe_colour);
#endif

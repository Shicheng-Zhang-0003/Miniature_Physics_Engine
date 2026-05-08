#ifndef wireframe_h
#define wireframe_h
#include <epoxy/gl.h>
#include "stage1/master_header.h"
#include "stage2/master_header_2.h"
#include "stage4/interaction/selector/object_selector.h"
// Renders objects as a wireframe outline with the solid sphere
void wireframe_render_selected_object (GLuint shader_program, math4 view, math4 projection);
void wireframe_render_object (GLuint shader_program, math4 view, math4 projection, rigidbody *rb, vector3 wire_colour);
#endif

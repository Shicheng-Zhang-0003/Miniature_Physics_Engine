#ifndef wireframe_h
#define wireframe_h
#include <epoxy/gl.h>
#include "../core/math3d.h"
#include "../core/math4_special.h"
#include "../core/rigidbody.h"

void wireframe_render_selected_object(GLuint shader_program, math4 view_matrix, math4 projection_matrix);
void wireframe_render_object(GLuint shader_program, math4 view_matrix, math4 projection_matrix, rigidbody *rigid_body,
                             vector3 wireframe_colour);
/* Drop cached program/uniform locations (render_cleanup calls this after
 * deleting GL programs: GL ids are reused, so a stale cache would hand
 * the next program's draws to the previous program's locations). */
void wireframe_invalidate_cache(void);
#endif

#ifndef traj_line_h
#define traj_line_h
#include <epoxy/gl.h>
#include "../core/math3d.h"
#include "../core/math4_special.h"
/* Dynamic purple polyline (GL_LINES) showing the predicted ball trajectory.
 * Mirrors the grid's draw pattern but re-uploads a small buffer each frame.
 * Uses the shared "utility" shader (aPos + constant normal + object_colour)
 * exactly like grid_render, so no new shader is needed. */
typedef struct {
    GLuint vertex_array_object, vertex_buffer_object;
    int vertex_count; /* floats in the buffer (multiple of 3) */
    float *cpu;       /* staging buffer, vertex_count floats */
    int cpu_cap;      /* floats allocated in cpu */
} traj_line_mesh;
void traj_line_init (traj_line_mesh *m);
/* Replace the polyline with `n` world-space points (drawn as n-1 segments).
 * n < 2 clears the line. */
void traj_line_update (traj_line_mesh *m, const vector3 *pts, int n);
void traj_line_render (traj_line_mesh *m, GLuint shader_program, math4 view_matrix, math4 projection_matrix);
void traj_line_cleanup (traj_line_mesh *m);
#endif /* traj_line_h */
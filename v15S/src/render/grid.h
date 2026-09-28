#ifndef grid_h
#define grid_h
#include <epoxy/gl.h>
#include "../core/math3d.h"
#include "../core/math4_special.h"

typedef struct {
    GLuint vertex_array_object, vertex_buffer_object;
    int line_vertex_count;
} grid_mesh;

void grid_init(grid_mesh *grid_mesh_object, int half_extent, int cell_spacing);
void grid_render(grid_mesh *grid_mesh_object, GLuint shader_program, math4 view_matrix, math4 projection_matrix);
/* Drop cached program/uniform locations after GL programs are deleted
 * (see render_cleanup): GL ids are reused, so a stale cache would bind
 * the next program's draws to the previous program's locations. */
void grid_invalidate_cache(void);
#endif

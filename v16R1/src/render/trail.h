#ifndef trail_h
#define trail_h
#include <epoxy/gl.h>
#include "../core/math3d.h"
#include "../core/math4_special.h"
/* SANDBOX S3: motion-trajectory trail for the SELECTED object only.
 * A cyan polyline of the selected body's recent positions, sampled once
 * per simulated physics tick and cleared whenever the selection changes
 * or empties. Drawn next to the selected-object wireframe in
 * render_scene_current — nothing is drawn (and nothing is kept) when no
 * body is selected, which is the whole point: trajectory indicators are
 * a selection tool, not world decoration. Uses the shared "utility"
 * shader via traj_line (no new shader), exactly like the grid and the
 * FTC predicted-shot line. */
void trail_init (void);
/* Sample the selected body's position once per simulated tick (skip
 * while frozen — a frozen world must not grow its trail; the O
 * frame-step DOES sample, because it simulates a tick). */
void trail_note_tick (void);
/* Upload + draw the trail (no-op with fewer than 2 points or no
 * selection). GL context must be current (call from the render tick). */
void trail_render (GLuint shader_program, math4 view_matrix, math4 projection_matrix);
void trail_cleanup (void);
#endif /* trail_h */

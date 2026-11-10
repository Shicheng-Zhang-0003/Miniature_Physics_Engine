/* SANDBOX S3: selected-object trajectory trail. See trail.h. */
#include "trail.h"
#include "traj_line.h"
#include "../mpe_engine.h"
#include "../ui_input/object_selector.h"
#include <string.h>
#define TRAIL_POINTS_MAX 256 /* ≈ 4.3 s at the fixed 1/60 s step */
static traj_line_mesh trail_mesh;
static int trail_initialised = 0;
static vector3 trail_points [TRAIL_POINTS_MAX];
static int trail_count = 0;
/* The trail belongs to exactly one body. 0 is the no-selection sentinel
 * (body ids start at 1 — physics_world_add_* force next_object_id != 0),
 * so a cleared selection drops the trail naturally. */
static uint32_t trail_owner_id = 0;
static void trail_reset (uint32_t new_owner) {
    trail_owner_id = new_owner;
    trail_count = 0;
}
void trail_init (void) {
    traj_line_init (&trail_mesh);
    trail_initialised = 1;
    trail_reset (0);
}
void trail_cleanup (void) {
    if (trail_initialised) { traj_line_cleanup (&trail_mesh); }
    trail_initialised = 0;
    trail_reset (0);
}
void trail_note_tick (void) {
    /* Deselect empties the trail: nothing selected → nothing kept. */
    if ((selected_object < 0) || (selected_object_id == 0)) {
        trail_reset (0);
        return;
    }
    physics_world *w = physics_world_get_primary ();
    if ((!w) || (selected_object >= w -> body_count)) {
        trail_reset (0);
        return;
    }
    /* Selection changed → the old path is another body's history. */
    if (trail_owner_id != selected_object_id) { trail_reset (selected_object_id); }
    const rigidbody *rb = &w -> bodies [selected_object];
    if (trail_count == TRAIL_POINTS_MAX) {
        /* Ring via a one-slot drop: the oldest point falls off. */
        memmove (trail_points, trail_points + 1, (TRAIL_POINTS_MAX - 1) * sizeof (vector3));
        trail_count = TRAIL_POINTS_MAX - 1;
    }
    trail_points [trail_count++] = rb -> position;
}
void trail_render (GLuint shader_program, math4 view_matrix, math4 projection_matrix) {
    if ((!trail_initialised) || (trail_count < 2) || (selected_object < 0)) { return; }
    traj_line_update (&trail_mesh, trail_points, trail_count);
    /* Cyan — distinct from the yellow selected wireframe and the purple
     * FTC predicted-shot line. */
    traj_line_render_colour (&trail_mesh, shader_program, view_matrix, projection_matrix,
                             (vector3) {0.2f, 0.9f, 1.0f});
}

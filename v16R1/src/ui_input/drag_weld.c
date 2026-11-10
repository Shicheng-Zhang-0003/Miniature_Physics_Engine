/* SANDBOX S4b: the drag implementation. See drag_weld.h.
 * Grab distance = the body's depth along the camera forward at grab
 * time (dot(pos − cam_pos, fwd), clamped to >= 1 m) so the body keeps
 * its reach instead of snapping to the crosshair plane. The tick runs
 * from input dispatch every frame — including while physics is frozen
 * (P) — so a frozen world is still an arrangement world. */
#include "drag_weld.h"
#include "../mpe_engine.h"
#include "object_selector.h"
#include <math.h>
#include <stdio.h>

static bool drag_active = false;
static float grab_distance = 1.0f;
static uint32_t dragged_id = 0;

void drag_weld_press (void) {
    if (drag_active) { return; } /* second button-down while held: ignore */
    /* LMB is also a selector: pick under the crosshair first (miss clears
     * the selection — same contract as right-click), then grab the pick. */
    selector_ray_tracing ();
    if (selected_object < 0) { return; }
    physics_world *w = physics_world_get_primary ();
    if ((!w) || (selected_object >= w -> body_count)) { return; }
    rigidbody *rb = &w -> bodies [selected_object];
    vector3 to_body = vector3_subtraction (rb -> position, main_camera_fov.position);
    grab_distance = vector3_dot (to_body, main_camera_fov.forward_vector);
    if (!isfinite (grab_distance) || grab_distance < 1.0f) { grab_distance = 1.0f; }
    dragged_id = rb -> object_id;
    drag_active = true;
    printf ("[drag] grabbed [%d] id %u at %.1f m\n",
            selected_object, (unsigned) dragged_id, grab_distance);
}

void drag_weld_tick (void) {
    if (!drag_active) { return; }
    physics_world *w = physics_world_get_primary ();
    if (!w) { drag_active = false; dragged_id = 0; return; }
    int idx = scene_find_object_index_by_id (dragged_id);
    if ((idx < 0) || (idx >= w -> body_count)) {
        /* Deleted mid-drag (middle mouse, terminal kill). */
        drag_active = false;
        dragged_id = 0;
        return;
    }
    rigidbody *rb = &w -> bodies [idx];
    rb -> position = vector3_addition (main_camera_fov.position,
                                       vector3_scaling (main_camera_fov.forward_vector, grab_distance));
    /* Carried, not thrown: kill momentum every frame so solver contacts
     * can't accumulate velocity under our hands. */
    rb -> velocity = vector3_zero ();
    rb -> angular_velocity = vector3_zero ();
    rigidbody_wake (rb);
}

void drag_weld_release (void) {
    if (!drag_active) { return; }
    drag_active = false;
    dragged_id = 0;
    printf ("[drag] dropped\n");
    /* SANDBOX S4c: weld-on-drop check lands here. */
}

bool drag_weld_is_active (void) {
    return drag_active;
}

/* SANDBOX S4b: the drag implementation. See drag_weld.h.
 * Grab distance = the body's depth along the camera forward at grab
 * time (dot(pos − cam_pos, fwd), clamped to >= 1 m) so the body keeps
 * its reach instead of snapping to the crosshair plane. The tick runs
 * from input dispatch every frame — including while physics is frozen
 * (P) — so a frozen world is still an arrangement world.
 *
 * SANDBOX S4c: weld-on-drop. Releasing the carried body while its
 * covering sphere overlaps another body pieces the two into ONE rigid
 * object_composite via the public MIG API physics_world_add_composite —
 * GUI-only orchestration, zero engine changes. Both source bodies are
 * expanded into a world-frame mig_part array (primitive = one part at
 * the body pose; composite = its pool slice rotated out to world), the
 * engine computes the COM and re-anchors, then the originals are
 * removed (higher index first) and the selection hands to the new
 * body's id. Per-part friction/restitution are carried post-hoc: the
 * add path stamps body_defaults onto every part, and mig_part has no
 * material fields. Statics/kinematics refuse the weld — one rigid
 * composite has a single static_state, so welding to an infinite-mass
 * anchor would have to bring the anchor to life; the Situation screen
 * toggles that first.
 *
 * 2026-11-10 audit rulings (mig_reviews.txt):
 *  - NO TRUNCATING WELDS: the total ACTIVE part count of both sources is
 *    checked BEFORE anything is built; over the engine cap the weld is
 *    REFUSED and both originals survive. The old collector kept its
 *    first 16 parts and the weld then DELETED both sources — silent
 *    geometry destruction (a 16-part composite plus a dropped sphere
 *    lost the sphere).
 *  - WELD TRIGGER USES THE COVERING SPHERE: broadphase_bounding_radius,
 *    not rb->radius — a cylinder's rb->radius is its DISC radius, so two
 *    deep-interpenetrating coaxial cylinders used to refuse the weld.
 *  - NO CUSTOM-BODY GUESSING: object_custom has no primitive part
 *    mapping; the weld refuses instead of pretending its half-extents
 *    are a cube.
 *  - MOMENTUM SEMANTICS PINNED: the fused body is born AT REST. The weld
 *    is an AUTHORING act — assemble an object from carried pieces (the
 *    carried body's velocity is zeroed every frame while held anyway) —
 *    not an impact event. Conservation-based fusion (v = Σmv/M plus
 *    angular-momentum bookkeeping about the new COM) would be a
 *    different, named feature; it is deliberately not half-added here. */
#include "drag_weld.h"
#include "../mpe_engine.h"
#include "../physics/broadphase.h"
#include "object_selector.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define WELD_PART_CAP 16        /* the engine's composite part cap */
#define WELD_OVERLAP_EPS 0.02f  /* covering spheres must overlap by this */

static bool drag_active = false;
static float grab_distance = 1.0f;
static uint32_t dragged_id = 0;

static int weld_type_to_mig (object_type t) {
    return t == object_sphere ? mig_part_sphere : t == object_cylinder ? mig_part_cylinder : mig_part_cube;
}
/* Pre-flight: count the ACTIVE parts a source would contribute and flag
 * unsupported types (object_custom has no primitive part mapping). Nothing
 * is mutated here — the weld refuses BEFORE any array is built. */
static int weld_count_parts (physics_world *w, rigidbody *src, bool *unsupported) {
    if (src -> type != object_composite) {
        if ((src -> type != object_sphere) && (src -> type != object_cube) && (src -> type != object_cylinder)) {
            *unsupported = true;
        }
        return 1;
    }
    int n = 0;
    for (int pi = 0; pi < src -> composite_part_count; pi++) {
        long slot = (long) src -> part_base + (long) pi;
        if ((slot < 0) || (slot >= (long) w -> composite_part_count)) { continue; }
        if (w -> composite_parts [slot].active) { n++; }
    }
    return n;
} /* Expand one source body into world-frame mig_parts appended to the
   * caller's array. fr_* run parallel to the part array so the materials
   * can be carried post-hoc (the add path stamps body_defaults). */
static void weld_collect_parts (physics_world *w, rigidbody *src, mig_part parts [], int *count,
                                float fr_s [], float fr_k [], float rest []) {
    if (src -> type != object_composite) {
        if (*count >= WELD_PART_CAP) { return; }
        mig_part *p = &parts [*count];
        p -> type = (mig_part_type) weld_type_to_mig (src -> type);
        p -> radius = (src -> type == object_cube) ? 0.0f : src -> radius;
        p -> cylinder_half_length = src -> cylinder_half_length;
        p -> half_extensions = src -> half_extensions;
        p -> position = src -> position;   /* WORLD frame: engine rebases to COM */
        p -> orientation = src -> orientation; /* part->assembly = part->world */
        p -> mass_mode = mig_mass_mode_explicit;
        p -> mass = src -> mass;
        fr_s [*count] = src -> friction_static;
        fr_k [*count] = src -> friction_kinetic;
        rest [*count] = src -> restitution;
        *count += 1;
        return;
    }
    /* Composite source: expand its pool slice out to world frame, the
     * exact pose math the renderer draws with (new_render.c:383). */
    for (int pi = 0; pi < src -> composite_part_count; pi++) {
        if (*count >= WELD_PART_CAP) {
            printf ("[weld] part cap %d reached: tail parts dropped\n", WELD_PART_CAP);
            return;
        }
        long slot = (long) src -> part_base + (long) pi;
        if ((slot < 0) || (slot >= (long) w -> composite_part_count)) { continue; }
        const composite_part *cp = &w -> composite_parts [slot];
        if (!cp -> active) { continue; }
        mig_part *p = &parts [*count];
        p -> type = (mig_part_type) weld_type_to_mig (cp -> type);
        p -> radius = (cp -> type == object_cube) ? 0.0f : cp -> radius;
        p -> cylinder_half_length = cp -> cylinder_half_length;
        p -> half_extensions = cp -> half_extensions;
        p -> position = vector3_addition (src -> position,
                                          vector4_rotate_to_vector3 (src -> orientation, cp -> local_position));
        p -> orientation = vector4_normalisation (vector4_multiplication (src -> orientation,
                                                                          cp -> local_orientation));
        p -> mass_mode = mig_mass_mode_explicit;
        p -> mass = cp -> mass;
        fr_s [*count] = cp -> friction_static;
        fr_k [*count] = cp -> friction_kinetic;
        rest [*count] = cp -> restitution;
        *count += 1;
    }
} static int weld_find_target (physics_world *w, int dragged_idx, bool *static_overlap) {
    rigidbody *a = &w -> bodies [dragged_idx];
    int best = -1;
    float best_d = 1e30f;
    /* Covering spheres, not the raw radius: a cylinder's rb->radius is
     * its DISC radius (the covering sphere is sqrt(r^2 + hl^2)), so the
     * old test refused welds between visibly interpenetrating cylinders.
     * broadphase_bounding_radius is the authored covering bound for
     * composites and the honest envelope for every primitive. */
    const float ra = broadphase_bounding_radius (a);
    bool saw_static = false;
    for (int i = 0; i < w -> body_count; i++) {
        if (i == dragged_idx) { continue; }
        rigidbody *b = &w -> bodies [i];
        float d = vector3_length (vector3_subtraction (b -> position, a -> position));
        if (d >= ((ra + broadphase_bounding_radius (b)) - WELD_OVERLAP_EPS)) { continue; }
        if (b -> static_state || b -> kinematic) { saw_static = true; continue; }
        if (d < best_d) { best_d = d; best = i; }
    }
    /* Usability ruling (review 9.4 tail): the nearest overlapping body is
     * not weldable while it is static/kinematic, but a DYNAMIC overlapping
     * body behind it still is — the search skips the refusal candidates
     * instead of dying on the nearest hit. *static_overlap reports that
     * some overlap existed but every candidate was static/kinematic, so
     * the caller can explain the plain drop. */
    *static_overlap = saw_static;
    return best;
} static void weld_try (int dragged_idx) {
    physics_world *w = physics_world_get_primary ();
    if ((!w) || (dragged_idx < 0) || (dragged_idx >= w -> body_count)) { return; }
    rigidbody *a = &w -> bodies [dragged_idx];
    if (a -> static_state || a -> kinematic) {
        printf ("[weld] refused: carried body is static/kinematic (toggle its Situation first)\n");
        return;
    }
    bool static_overlap = false;
    int target_idx = weld_find_target (w, dragged_idx, &static_overlap);
    if (target_idx < 0) {
        if (static_overlap) {
            printf ("[weld] refused: overlapping body is static/kinematic (toggle its Situation first)\n");
        }
        return; /* plain drop, nothing weldable overlapped */
    }
    rigidbody *b = &w -> bodies [target_idx];
    /* 2026-11-10 audit (review 9.1): count BEFORE building — a weld that
     * would exceed the engine cap is REFUSED with both originals kept.
     * The old flow collected through the truncating collector, welded the
     * truncated 16, and then DELETED both sources: silent geometry loss. */
    bool unsupported = false;
    const int total_parts = weld_count_parts (w, a, &unsupported) + weld_count_parts (w, b, &unsupported);
    if (unsupported) {
        printf ("[weld] refused: custom/plugin bodies have no primitive part mapping\n");
        return;
    }
    if (total_parts > WELD_PART_CAP) {
        printf ("[weld] refused: %d active parts exceed the %d-part cap — both bodies kept\n",
                total_parts, WELD_PART_CAP);
        return;
    }
    mig_part parts [WELD_PART_CAP];
    float fr_s [WELD_PART_CAP], fr_k [WELD_PART_CAP], rest [WELD_PART_CAP];
    int count = 0;
    weld_collect_parts (w, a, parts, &count, fr_s, fr_k, rest);
    weld_collect_parts (w, b, parts, &count, fr_s, fr_k, rest);
    if (count < 2) { return; }
    /* MOMENTUM SEMANTICS (pinned 2026-11-10, review 9.5): the fused body
     * is born AT REST — an authoring act, not an impact event. */
    /* Body-level materials of the carrier, captured before removals. */
    float a_fs = a -> friction_static, a_fk = a -> friction_kinetic, a_r = a -> restitution;
    int new_index = physics_world_add_composite (w, parts, count);
    if (new_index < 0) {
        printf ("[weld] refused: engine rejected the assembly (mass window / degenerate)\n");
        return;
    }
    uint32_t new_id = w -> bodies [new_index].object_id;
    /* Carry per-part materials: the add path stamped body_defaults. */
    rigidbody *nb = &w -> bodies [new_index];
    for (int i = 0; i < nb -> composite_part_count; i++) {
        long slot = (long) nb -> part_base + (long) i;
        if ((slot < 0) || (slot >= (long) w -> composite_part_count)) { continue; }
        composite_part *cp = &w -> composite_parts [slot];
        cp -> friction_static = fr_s [i];
        cp -> friction_kinetic = fr_k [i];
        cp -> restitution = rest [i];
    }
    /* Remove both originals, HIGHER index first — removal compacts the
     * array, so the lower index must stay meaningful. */
    if (target_idx > dragged_idx) {
        scene_remove_object_by_index (target_idx);
        scene_remove_object_by_index (dragged_idx);
    } else {
        scene_remove_object_by_index (dragged_idx);
        scene_remove_object_by_index (target_idx);
    }
    /* Hand the selection to the welded body (its index moved down during
     * the removals; the id is stable). */
    int sel = scene_find_object_index_by_id (new_id);
    if (sel >= 0) {
        select_object_by_index (sel);
        w -> bodies [sel].friction_static = a_fs;
        w -> bodies [sel].friction_kinetic = a_fk;
        w -> bodies [sel].restitution = a_r;
        printf ("[weld] fused %d parts -> new body [%d] (id %u)\n", count, sel, (unsigned) new_id);
    }
}

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
    int idx = scene_find_object_index_by_id (dragged_id);
    drag_active = false;
    dragged_id = 0;
    /* SANDBOX S4c: overlap at release = weld the two into one composite. */
    if (idx >= 0) { weld_try (idx); }
    else { printf ("[drag] dropped\n"); }
}

bool drag_weld_is_active (void) {
    return drag_active;
}

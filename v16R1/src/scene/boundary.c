/* GTK4-PREP: zero GUI headers in scene. */
#include "boundary.h"
#include "../core/rigidbody.h"
#include "../core/physics_world.h"
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include <math.h>
#include <stdbool.h>
/* MPE_TASK_08_FLOOR_EMERGENCY_TUNING_BEGIN */
/* MPE_TASK_08_FLOOR_EMERGENCY_TUNING_END */
// Helper: Get lowest point of OBB along an axis
/* TRUTH: exact support function per shape. The OBB formula
 * S(d) = SUM he_i*|axis_i.d| is exact for boxes (corners) but OVERESTIMATES
 * cylinders by up to r*(|a|+|b|-sqrt(a^2+b^2)) (peaks when the axle is
 * diagonal to d): the phantom depth tripped the floor clamp on ROLLING
 * wheels (+8.8cm teleport into the air, then Poisson bounce off the slam —
 * the "erratic cylinder" instability). Cylinders use their exact support
 * S_cyl(d) = r*|d-(d.ax)ax| + h*|d.ax| (same form as the CCD support). */
static float body_support_along_axis (rigidbody *rigid_body, vector3 axis) {
    if (rigid_body -> type == object_sphere) { return rigid_body -> radius; }
    if (rigid_body -> type == object_cylinder) {
        /* TRUTH: never trust cached_axes blindly (zero/NaN axle from corrupt
         * state). Degraded axle degrades to the bounding radius (conservative
         * clamp: may hold slightly high, never lets escape). */
        vector3 axle = rigid_body -> cached_axes [0];
        float l2 = vector3_length_squared (axle);
        if (!isfinite (l2) || l2 < 1e-8f || l2 > 4.0f) { return broadphase_bounding_radius (rigid_body); }
        axle = vector3_scaling (axle, 1.0f / sqrtf (l2));
        float along = vector3_dot (axle, axis);
        vector3 radial_vec = vector3_subtraction (axis, vector3_scaling (axle, along));
        return rigid_body -> radius * vector3_length (radial_vec) + rigid_body -> cylinder_half_length * fabsf (along);
    } vector3 *axes = rigid_body -> cached_axes;
    return rigid_body -> half_extensions.x * fabsf (vector3_dot (axes [0], axis)) +
           rigid_body -> half_extensions.y * fabsf (vector3_dot (axes [1], axis)) +
           rigid_body -> half_extensions.z * fabsf (vector3_dot (axes [2], axis));
}
static float get_obb_min_along_axis (rigidbody *rigid_body, vector3 axis) {
    /* MPE_TASK_16_BOUNDARY_CACHED_AXES_MIN_BEGIN */
    float projection = body_support_along_axis (rigid_body, axis);
    /* MPE_TASK_16_BOUNDARY_CACHED_AXES_MIN_END */
    return vector3_dot (rigid_body -> position, axis) - projection;
} // Helper: Get highest point of OBB along an axis
static float get_obb_max_along_axis (rigidbody *rigid_body, vector3 axis) {
    /* MPE_TASK_16_BOUNDARY_CACHED_AXES_MAX_BEGIN */
    float projection = body_support_along_axis (rigid_body, axis);
    /* MPE_TASK_16_BOUNDARY_CACHED_AXES_MAX_END */
    return vector3_dot (rigid_body -> position, axis) + projection;
}
/* ---- MIG WP2 2.8 (build plan §4): composite-aware boundary extrema. ----
 * The world-edge safety net read a composite through its BODY-LEVEL
 * shape fields, and a composite is sphere-initialised (radius =
 * authored covering bound, half_extensions = (r, r, r) so OBB readers
 * see a bounding box) before its retype to object_composite. The cube
 * branch therefore projected a phantom box of half-extent r = the whole
 * assembly's bound: a 10-cube stack composite (bound 5.375 m, true
 * lowest part at 0) read min_y = 5.009 − 5.375 = −0.366, so the net
 * clamped it to COVERING-SPHERE REST — the assembly hovered 0.366 m
 * above the floor in a perpetual clamp/fall cycle and could never
 * sleep (the mig_composite_stack gate caught this on its first run).
 * The covering sphere is a physics ENVELOPE, not geometry — the same
 * truth 2.6 enforced for the static-plane and CCD paths. The exact
 * per-part extrema below walk the world's part pool exactly like the
 * 2.6 CCD support (ccd_composite_support_point): part world pose =
 * body ∘ part-local, per-part support exact per shape (sphere r;
 * cylinder axle-local-X exact form; cube |axis·he| through the part's
 * OWN frame — the axis is conjugated by the part's local_orientation
 * too, not just the body's: 2026-11-10 audit fix, the gate
 * mig_boundary_rotated_part pins it). min/max are asymmetric for a
 * composite, so both extrema
 * are computed, not a symmetric radius. World-less callers (NULL) keep
 * the legacy body-level projection — the documented degraded mode, the
 * CCD's low-memory precedent. */
/* ---- MIG WP2.6 2.6.5: hull-aware boundary extrema. ----
 * The world-edge safety net read an object_hull body through its
 * BODY-LEVEL shape fields. A hull is sphere-initialised (radius = the
 * bounding circumradius, half_extensions = (r, r, r) mirroring it), so
 * the net's OBB readers saw a phantom cube of half-extent r: a pyr4
 * resting at COM h/4 = 0.5 read min_y = 0.5 − 1.5 = −1.0, the net
 * clamped it to COVERING-SPHERE REST and the body hovered a metre above
 * the floor in a perpetual clamp/fall cycle (same failure class the
 * 2.8 composite fix removed for assemblies — the covering sphere is a
 * physics ENVELOPE, not geometry). The exact extrema below walk the
 * world's hull-vertex pool (the same slice the narrowphase consumes):
 * the stored vertices are COM-local, so the axis is conjugated into the
 * body frame and both supports are taken in one pass (a hull is convex,
 * but its ±axis supports differ — min = −S(−axis), max = +S(+axis)).
 * World-less or defective-slice callers return 0 and the caller keeps
 * the legacy body-level projection — the documented degraded mode,
 * conservative in the same sense as the composite NULL-world path. */
static int boundary_hull_extrema_along_axis (const physics_world *world, const rigidbody *rigid_body,
                                             vector3 axis, float *min_out, float *max_out) {
    if ((!world) || (!rigid_body) || (!min_out) || (!max_out)) { return 0; }
    if (rigid_body -> type != object_hull) { return 0; }
    if ((rigid_body -> hull_vert_base < 0) || (rigid_body -> hull_vert_count <= 0)) { return 0; }
    if ((long) rigid_body -> hull_vert_base + (long) rigid_body -> hull_vert_count >
        (long) world -> hull_vert_count) { return 0; }
    if (!isfinite (axis.x) || !isfinite (axis.y) || !isfinite (axis.z)) { return 0; }
    const vector4 q = rigid_body -> orientation;
    if (!isfinite (q.w) || !isfinite (q.x) || !isfinite (q.y) || !isfinite (q.z)) { return 0; }
    const vector4 inv = {q.w, -q.x, -q.y, -q.z};
    const vector3 axis_local = vector4_rotate_to_vector3 (inv, axis);
    float s_plus = -INFINITY;  /* max dot(v, +axis_local) */
    float s_minus = -INFINITY; /* max dot(v, -axis_local) */
    for (int i = 0; i < rigid_body -> hull_vert_count; i++) {
        const mig_hull_point *v = &world -> hull_verts [rigid_body -> hull_vert_base + i];
        const vector3 lv = {(float) v -> x, (float) v -> y, (float) v -> z};
        if (!isfinite (lv.x) || !isfinite (lv.y) || !isfinite (lv.z)) { return 0; }
        const float d = vector3_dot (lv, axis_local);
        if (!isfinite (d)) { return 0; }
        if (d > s_plus) { s_plus = d; }
        if (-d > s_minus) { s_minus = -d; }
    }
    const float along = vector3_dot (rigid_body -> position, axis);
    const float lo = along - s_minus;
    const float hi = along + s_plus;
    if ((!isfinite (lo)) || (!isfinite (hi))) { return 0; }
    *min_out = lo;
    *max_out = hi;
    return 1;
}
static int boundary_composite_extrema_along_axis (const physics_world *world, const rigidbody *rigid_body,
                                                  vector3 axis, float *min_out, float *max_out) {
    if ((!world) || (!rigid_body) || (!min_out) || (!max_out)) { return 0; }
    if (rigid_body -> type != object_composite) { return 0; }
    if ((rigid_body -> composite_part_count <= 0) || (rigid_body -> part_base < 0)) { return 0; }
    long last = (long) rigid_body -> part_base + (long) rigid_body -> composite_part_count;
    if (last > (long) world -> composite_part_count) { return 0; }
    /* Unit-quaternion inverse = conjugate (the compose validated the
     * part orientations; sanitize owns the body's). */
    vector4 inv = {rigid_body -> orientation.w, -rigid_body -> orientation.x,
                   -rigid_body -> orientation.y, -rigid_body -> orientation.z};
    float best_min = +INFINITY;
    float best_max = -INFINITY;
    for (long i = rigid_body -> part_base; i < last; i++) {
        const composite_part *part = &world -> composite_parts [i];
        if (!part -> active) { continue; }
        vector3 part_world = vector3_addition (rigid_body -> position,
                                               vector4_rotate_to_vector3 (rigid_body -> orientation, part -> local_position));
        /* 2026-11-10 audit fix: the axis must reach the part's OWN frame —
         * body frame first (inv), THEN the part's local_orientation
         * conjugate. The original code stopped at the body frame, so a
         * part posed with a non-identity local_orientation (first-class
         * state: drag_weld, save/load and the ghost view all produce it;
         * collision_ccd.c composes the same pair for its support) was
         * projected as if its primitive were axis-aligned to the BODY:
         * a cylinder with its axle along body +Y read S_y = r instead of
         * hl, letting the safety net sink 0.7 m before firing on an
         * r=0.3/hl=1.0 part. Quaternion identity used below:
         * rot(conj(q_local) ⊗ conj(q_body), axis) ==
         * rot(conj(q_local), rot(conj(q_body), axis)). */
        vector4 pinv = {part -> local_orientation.w, -part -> local_orientation.x,
                        -part -> local_orientation.y, -part -> local_orientation.z};
        vector3 local_axis = vector4_rotate_to_vector3 (pinv, vector4_rotate_to_vector3 (inv, axis));
        float support = 0.0f;
        if (part -> type == object_sphere) {
            support = part -> radius;
        } else if (part -> type == object_cylinder) {
            /* Axle local X (MPE_FTC_090): exact cylinder support, the same
             * form body_support_along_axis uses for whole cylinders. */
            vector3 radial = vector3_subtraction (local_axis, vector3_scaling ((vector3) {1.0f, 0.0f, 0.0f}, local_axis.x));
            support = part -> radius * vector3_length (radial) + part -> cylinder_half_length * fabsf (local_axis.x);
        } else {
            support = part -> half_extensions.x * fabsf (local_axis.x) +
                      part -> half_extensions.y * fabsf (local_axis.y) +
                      part -> half_extensions.z * fabsf (local_axis.z);
        }
        float along = vector3_dot (part_world, axis);
        if (along - support < best_min) { best_min = along - support; }
        if (along + support > best_max) { best_max = along + support; }
    }
    if ((!isfinite (best_min)) || (!isfinite (best_max))) { return 0; }
    *min_out = best_min;
    *max_out = best_max;
    return 1;
}
/* World-aware extrema: composite parts consulted when a world is given
 * (the caller owns the part pool the body indexes into), legacy
 * body-level projection otherwise. */
static float get_obb_min_along_axis_w (const physics_world *world, const rigidbody *rigid_body, vector3 axis) {
    float cmin = 0.0f, cmax = 0.0f;
    if (boundary_hull_extrema_along_axis (world, rigid_body, axis, &cmin, &cmax)) { return cmin; }
    if (boundary_composite_extrema_along_axis (world, rigid_body, axis, &cmin, &cmax)) { return cmin; }
    return get_obb_min_along_axis ((rigidbody *) rigid_body, axis);
}
static float get_obb_max_along_axis_w (const physics_world *world, const rigidbody *rigid_body, vector3 axis) {
    float cmin = 0.0f, cmax = 0.0f;
    if (boundary_hull_extrema_along_axis (world, rigid_body, axis, &cmin, &cmax)) { return cmax; }
    if (boundary_composite_extrema_along_axis (world, rigid_body, axis, &cmin, &cmax)) { return cmax; }
    return get_obb_max_along_axis ((rigidbody *) rigid_body, axis);
} /* World-edge safety net: PERFECTLY PLASTIC positional clamp.
 *
 * Bounce (restitution) belongs to material contacts in the solver, which
 * combines both bodies' properties. The boundary is not a material, so it
 * never reflects velocity and never applies magic damping: it repositions
 * bodies inside the playable volume and kills only the inward (escaping)
 * velocity component. Deep escape still wakes the body so it rejoins. */
void boundary_apply_floor (rigidbody *rigid_body, float floor_y_level) {boundary_apply_floor_cfg (rigid_body, floor_y_level, NULL);}
void boundary_apply_floor_cfg (rigidbody *rigid_body, float floor_y_level, const mpe_config_t *cfg) {
    if (!rigid_body) { return; }
    if (rigid_body -> static_state || rigid_body -> kinematic) { return; }
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    float min_y = get_obb_min_along_axis (rigid_body, (vector3) {0, 1, 0});
    if (min_y < (floor_y_level - C -> boundary.floor_emergency_slop)) {
        rigid_body -> position.y += (floor_y_level - min_y);
        if (rigid_body -> velocity.y < 0.0f) { rigid_body -> velocity.y = 0.0f; }
        rigidbody_wake (rigid_body);
    }
}
void boundary_apply_box (rigidbody *rigid_body, vector3 min_bounds, vector3 maximum_bounds) {boundary_apply_box_cfg (rigid_body, min_bounds, maximum_bounds, NULL);}
void boundary_apply_box_cfg (rigidbody *rigid_body, vector3 min_bounds, vector3 max_bounds, const mpe_config_t *cfg) {
    boundary_apply_box_world_cfg (rigid_body, min_bounds, max_bounds, cfg, NULL);
}
/* MIG WP2 2.8: world-aware variant — a composite's extrema consult the
 * stepped world's part pool (the body indexes THAT pool, so the caller
 * owns the right one; physics_world_get_primary would lie for twin test
 * worlds). NULL world = the legacy body-level projection, byte-identical
 * for world-less callers.
 * TRUTH: X/Z walls use the same emergency slop as Y (documented asymmetry
 * removed: a 1cm X-escape used to snap while 1cm Y-penetration was ignored;
 * the safety net must be uniform). */
void boundary_apply_box_world_cfg (rigidbody *rigid_body, vector3 min_bounds, vector3 max_bounds,
                                   const mpe_config_t *cfg, const physics_world *world) {
    if (!rigid_body) { return; }
    if (rigid_body -> static_state || rigid_body -> kinematic) { return; }
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    float es = C -> boundary.floor_emergency_slop;
    if (!isfinite (es) || es < 0.0f) { es = 0.0f; }
    // X axis (uniform emergency slop on every face)
    float min_x = get_obb_min_along_axis_w (world, rigid_body, (vector3) {1, 0, 0});
    if (min_x < min_bounds.x - es) {
        rigid_body -> position.x += (min_bounds.x - min_x);
        rigidbody_wake (rigid_body);
        if (rigid_body -> velocity.x < 0) { rigid_body -> velocity.x = 0.0f; }
    }
    float max_x = get_obb_max_along_axis_w (world, rigid_body, (vector3) {1, 0, 0});
    if (max_x > max_bounds.x + es) {
        rigid_body -> position.x -= (max_x - max_bounds.x);
        rigidbody_wake (rigid_body);
        if (rigid_body -> velocity.x > 0) { rigid_body -> velocity.x = 0.0f; }
    } // Y axis
    float min_y = get_obb_min_along_axis_w (world, rigid_body, (vector3) {0, 1, 0});
    if (min_y < (min_bounds.y - es)) {
        rigid_body -> position.y += (min_bounds.y - min_y);
        rigidbody_wake (rigid_body);
        if (rigid_body -> velocity.y < 0.0f) { rigid_body -> velocity.y = 0.0f; }
    }
    float max_y = get_obb_max_along_axis_w (world, rigid_body, (vector3) {0, 1, 0});
    if (max_y > max_bounds.y + es) {
        rigid_body -> position.y -= (max_y - max_bounds.y);
        rigidbody_wake (rigid_body);
        if (rigid_body -> velocity.y > 0) { rigid_body -> velocity.y = 0.0f; }
    } // Z axis
    float min_z = get_obb_min_along_axis_w (world, rigid_body, (vector3) {0, 0, 1});
    if (min_z < min_bounds.z - es) {
        rigid_body -> position.z += (min_bounds.z - min_z);
        rigidbody_wake (rigid_body);
        if (rigid_body -> velocity.z < 0) { rigid_body -> velocity.z = 0.0f; }
    }
    float max_z = get_obb_max_along_axis_w (world, rigid_body, (vector3) {0, 0, 1});
    if (max_z > max_bounds.z + es) {
        rigid_body -> position.z -= (max_z - max_bounds.z);
        rigidbody_wake (rigid_body);
        if (rigid_body -> velocity.z > 0) { rigid_body -> velocity.z = 0.0f; }
    }
}

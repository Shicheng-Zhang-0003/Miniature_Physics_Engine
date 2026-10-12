/* Purple predicted-trajectory line (see ftc_traj.h).
 *
 * The preview is a FORWARD SIMULATION, not a formula. We build a throwaway
 * physics_world that mirrors everything the ball can actually strike in the
 * primary world (the static plane, the tile slab, the perimeter walls and
 * the suspended bucket — i.e. every static body), give it ONE ball with the
 * real carried ball's mass/radius/restitution/friction, place that ball at
 * the exact muzzle pose the shooter will use (via the SAME pure
 * intake_launch_state_compute the shooter calls, resolved from the bundle),
 * then step the real solver at the engine dt, applying the real drag+Magnus
 * aero (intake_balls_aero_step, also from the bundle) each tick. The
 * recorded centres are the purple polyline. Bounces, rim catches and floor
 * bounces fall out of the same collision code the live shot uses, so the
 * line cannot disagree with the shot.
 *
 * This lives in the engine (ui_input), not the bundle: it reaches FTC only
 * through dlsym and owns a private throwaway world; it never touches the
 * primary world or its constraint pool. */
#include "ftc_traj.h"
#include "../core/mpe_platform.h"
#include "../core/mpe_loader.h"
#include "../core/physics_world.h"
#include "../core/rigidbody.h"
#include "../physics/constraint.h"
#include "../config/mpe_config.h"
#include "../ecosystem/mfs/modules/ftc/submodules/intake.h"
#include "../ecosystem/mfs/modules/ftc/submodules/robot.h"
#include <math.h>
#include <string.h>
/* Bundle identity mirrors term_ftc.c / gamepad_drive.c / ftc_view.c. */
#define FTRJ_HANDLE_PRIMARY "mfs-simulator"
#define FTRJ_HANDLE_MODULE "ecosystem/mfs/plugins/mpe_ftc.so"
static void *ftrj_sym (const char *sym) {
    void *p = mpe_loader_symbol (FTRJ_HANDLE_PRIMARY, sym);
    if (p)
        return p;
    return mpe_loader_symbol (FTRJ_HANDLE_MODULE, sym);
}
/* Prediction horizon / sampling. A lob at INTAKE_LAUNCH_SPEED peaks ~0.7 s
 * and lands ~1 s; 3 s at every tick captures the arc plus a bounce or two. */
#define FTRJ_DT (1.0f / 60.0f)
#define FTRJ_MAX_TICKS 180
#define FTRJ_MIN_INTERVAL 0.25f /* s between recomputes while driving */
/* Recompute when the shot would actually differ: aim moves > this yaw, the
 * chassis moves > this distance, or carried count changes. */
#define FTRJ_HEADING_EPS_DEG 2.0f
#define FTRJ_POS_EPS 0.05f
#define FTRJ_MAX_PTS FTRJ_MAX_TICKS
static vector3 s_pts [FTRJ_MAX_PTS];
static int s_npts = 0;
/* cache key */
static vector3 s_last_chassis_pos;
static float s_last_yaw_deg = 1e9f;
static int s_last_carried = -1;
static float s_since_recompute = 1e9f;
static int s_have_key = 0;
/* Reusable throwaway world + a flag for whether it is currently live. */
static physics_world s_pred;
static int s_pred_live = 0;
/* Persistent config snapshot bound to the throwaway world (a pointer to a
 * stack copy would dangle). Rebound from the primary world on every rebuild. */
static mpe_config_t s_pred_cfg;
/* Copy one static body from src world into the prediction world so the ball
 * bounces off the same geometry (floor slab, walls, bucket). Statics are
 * immovable, so their state is frozen by definition. */
static void ftrj_copy_static (physics_world *dst, const rigidbody *sb) {
    if (!sb)
        return;
    int di = -1;
    if (sb -> type == object_cube) {
        di = physics_world_add_cube (dst, sb -> position, sb -> half_extensions, 0.0f);
    } else if (sb -> type == object_sphere) {
        di = physics_world_add_sphere (dst, sb -> radius, 0.0f, sb -> position);
    } else if (sb -> type == object_cylinder) {
        di = physics_world_add_cylinder (dst, sb -> radius, sb -> cylinder_half_length, 0.0f, sb -> position);
    }
    if (di < 0)
        return;
    rigidbody *db = &dst -> bodies [di];
    db -> static_state = true;
    db -> inverse_mass = 0.0f;
    db -> friction_static = sb -> friction_static;
    db -> friction_kinetic = sb -> friction_kinetic;
    db -> restitution = sb -> restitution;
    db -> orientation = sb -> orientation;
    db -> colour = sb -> colour;
    db -> kinematic = false;
    db -> is_sleeping = false;
    rigidbody_update_axes (db);
}
/* Is this body a static the ball can strike? Excludes the robot's own parts
 * (dynamic) and the ball registry (dynamic). Static floor/walls/bucket only. */
static int ftrj_is_static (const rigidbody *b) {
    return (b -> static_state || b -> mass == 0.0f) && !b -> kinematic && !b -> no_collide;
}
void ftc_traj_update (float frame_delta_time) {
    if (!isfinite (frame_delta_time))
        frame_delta_time = FTRJ_DT;
    s_since_recompute += frame_delta_time;
    physics_world *w = physics_world_get_primary ();
    ftc_robot * (*p_get) (struct physics_world *, int) =
        (ftc_robot * (*) (struct physics_world *, int)) ftrj_sym ("ftc_fleet_get");
    void (*p_launch) (vector3, vector4, vector3, intake_launch_state *) =
        (void (*) (vector3, vector4, vector3, intake_launch_state *)) ftrj_sym ("intake_launch_state_compute");
    void (*p_aero) (physics_world *, const int *, int) =
        (void (*) (physics_world *, const int *, int)) ftrj_sym ("intake_balls_aero_step");
    ftc_robot *r = (w && p_get) ? p_get (w, 0) : NULL;
    /* Nothing to preview without a robot that is actually carrying a ball. */
    if (!w || !p_launch || !p_aero || !r || r -> chassis_body < 0 || r -> chassis_body >= w -> body_count ||
        r -> intake_count <= 0) {
        s_npts = 0;
        s_have_key = 0;
        if (s_pred_live) { physics_world_cleanup (&s_pred); s_pred_live = 0; }
        return;
    }
    rigidbody *ch = &w -> bodies [r -> chassis_body];
    /* Aim key: chassis forward yaw (deg) + position + carried count. Recompute
     * only when the preview would visibly change, else at most every interval. */
    vector3 fwd = vector4_rotate_to_vector3 (ch -> orientation, (vector3) {0.0f, 0.0f, 1.0f});
    float yaw_deg = atan2f (fwd.z, fwd.x) * 57.29578f;
    int moved = 1;
    if (s_have_key && s_last_carried == r -> intake_count && s_since_recompute < FTRJ_MIN_INTERVAL) {
        float dyaw = fabsf (yaw_deg - s_last_yaw_deg);
        if (dyaw > 180.0f)
            dyaw = 360.0f - dyaw;
        float dp = vector3_length (vector3_subtraction (ch -> position, s_last_chassis_pos));
        moved = (dyaw > FTRJ_HEADING_EPS_DEG || dp > FTRJ_POS_EPS || s_last_carried != r -> intake_count);
    }
    if (s_have_key && !moved) {
        return; /* preview still valid */
    }
    /* The ball about to fire is the oldest carried one. */
    int ball_id = r -> intake_balls [0];
    int ball_index = physics_world_index_by_id (w, (uint32_t) ball_id);
    if (ball_index < 0 || ball_index >= w -> body_count) {
        s_npts = 0;
        s_have_key = 0;
        return;
    }
    rigidbody *ball = &w -> bodies [ball_index];
    /* ---- throwaway world that mirrors the statics the ball can hit ---- */
    if (s_pred_live) { physics_world_cleanup (&s_pred); s_pred_live = 0; }
    physics_world_init (&s_pred);
    constraint_pool_init (&s_pred);
    s_pred_live = 1;
    /* Same solver/gravity/restitution settings as the live world, so bounces
     * and flight match. physics_world_set_config takes a mutable pointer; the
     * primary world exposes its own snapshot, copy it then bind. */
    {
        s_pred_cfg = *mpe_world_cfg (w);
        physics_world_set_config (&s_pred, &s_pred_cfg);
    }
    /* Mirror the static plane (neutral restitution floor) when live. */
    if (w -> static_plane_enabled) {
        s_pred.static_plane_enabled = true;
        s_pred.static_plane_body.friction_static = w -> static_plane_body.friction_static;
        s_pred.static_plane_body.friction_kinetic = w -> static_plane_body.friction_kinetic;
        s_pred.static_plane_body.restitution = w -> static_plane_body.restitution;
    }
    /* Copy every static body (floor slab, perimeter walls, bucket). */
    for (int i = 0; i < w -> body_count; i++) {
        const rigidbody *b = &w -> bodies [i];
        if (ftrj_is_static (b)) {
            ftrj_copy_static (&s_pred, b);
        }
    }
    /* Add the ball with its REAL radius/mass/material so drag (frontal area),
     * bounce height and rim catch all use the shipped piece (arcade ball or
     * larger BIOBUZZ nectar/pollen). */
    float brad = (isfinite (ball -> radius) && ball -> radius > 0.001f) ? ball -> radius : INTAKE_BALL_RADIUS;
    float bmass = (isfinite (ball -> mass) && ball -> mass > 0.0f) ? ball -> mass : INTAKE_BALL_MASS;
    int pi = physics_world_add_sphere (&s_pred, brad, bmass, (vector3) {0.0f, -50.0f, 0.0f});
    if (pi < 0) {
        s_npts = 0;
        s_have_key = 0;
        return;
    }
    rigidbody *pb = &s_pred.bodies [pi];
    pb -> restitution = ball -> restitution;
    pb -> friction_static = ball -> friction_static;
    pb -> friction_kinetic = ball -> friction_kinetic;
    /* Launch state from the SAME helper the shooter uses (inherit chassis vel). */
    intake_launch_state st;
    p_launch (ch -> position, ch -> orientation, ch -> velocity, &st);
    pb -> position = st.muzzle;
    pb -> velocity = st.velocity;
    pb -> angular_velocity = st.spin;
    pb -> kinematic = false;
    pb -> static_state = false;
    rigidbody_wake (pb);
    rigidbody_update_axes (pb);
    /* ---- forward-simulate and record the polyline ---- */
    int pred_id = (int) pb -> object_id;
    s_npts = 0;
    for (int k = 0; k < FTRJ_MAX_TICKS; k++) {
        p_aero (&s_pred, &pred_id, 1);
        physics_world_step (&s_pred, FTRJ_DT);
        vector3 p = pb -> position;
        if (!isfinite (p.x) || !isfinite (p.y) || !isfinite (p.z))
            break;
        s_pts [s_npts++] = p;
        /* Stop once the piece has settled well below its rest height and is
         * slow: further path is the ball lying still. */
        if (p.y <= brad * 0.6f && vector3_length_squared (pb -> velocity) < 0.01f && k > 10) {
            s_pts [s_npts++] = p;
            break;
        }
    }
    /* Refresh cache key. */
    s_last_chassis_pos = ch -> position;
    s_last_yaw_deg = yaw_deg;
    s_last_carried = r -> intake_count;
    s_have_key = 1;
    s_since_recompute = 0.0f;
}
int ftc_traj_enabled (void) {return s_npts > 1;}
int ftc_traj_points (const vector3 **out_pts) {
    if (out_pts)
        *out_pts = s_pts;
    return s_npts;
}
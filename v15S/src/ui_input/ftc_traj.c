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
 * COST (this is the whole reason for the state machine below): one 180-tick
 * prediction measured 45-146 ms depending on solver iterations — 3-9x the
 * entire 16.7 ms frame budget, which froze the GUI outright. So the solve is
 * AMORTIZED: it keeps its world alive across frames and runs a fixed tick
 * budget per frame (a few ms), growing the polyline as it goes, instead of
 * one blocking burst. A recompute is only (re)started when the shot actually
 * changes (aim heading, carried count, a real translation), never per frame.
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
/* ---- prediction horizon / sampling ----
 * A 9 m/s lob at 50 deg peaks ~0.7 s and lands ~1 s; 2.5 s at every tick
 * captures the arc plus the first bounces. Every 2nd tick is recorded (the
 * line stays smooth, the vertex count halves). */
#define FTRJ_DT (1.0f / 60.0f)
#define FTRJ_MAX_TICKS 150
#define FTRJ_RECORD_EVERY 2
#define FTRJ_MAX_PTS (FTRJ_MAX_TICKS / FTRJ_RECORD_EVERY + 2)
/* Per-frame tick budget. ~0.25 ms/tick measured with the ball in contact, so
 * this is ~2-3 ms/frame while a prediction is growing — well inside the
 * 16.7 ms budget, and the line completes in ~0.2 s (it draws itself in). */
#define FTRJ_TICKS_PER_FRAME 12
/* The prediction world has ONE ball and static geometry: no revolute joints,
 * no 40:1 chassis/wheel mass ratio (the reason the live world needs 128).
 * Measured: trajectory deviation vs the live 128-iteration shot is 0.000000 m
 * at 8 iterations (and at every count down to 2), so this is free accuracy. */
#define FTRJ_ITERATIONS 8
/* Restart triggers: aim turned, a real translation, or the carried set
 * changed. Position alone moves every frame while driving, so it needs a
 * generous threshold (or the line would restart faster than it can finish). */
#define FTRJ_HEADING_EPS_DEG 3.0f
#define FTRJ_POS_EPS 0.25f
#define FTRJ_RESTART_COOLDOWN 0.35f
static vector3 s_pts [FTRJ_MAX_PTS];
static int s_npts = 0;
/* cache key (the aim the CURRENT prediction was started from) */
static vector3 s_key_pos;
static float s_key_yaw_deg = 0.0f;
static int s_key_carried = -1;
static int s_have_key = 0;
/* amortisation state */
static int s_ticks_done = 0;   /* ticks simulated for the current prediction */
static int s_ball_index = -1;  /* index of the predicted ball in s_pred.bodies */
static float s_since_restart = 1e9f;
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
/* (Re)start a prediction: rebuild the throwaway world from the current aim and
 * reset the tick cursor. Returns the ball body index, or -1. */
static int ftrj_begin (physics_world *w, ftc_robot *r, rigidbody *ch) {
    if (s_pred_live) { physics_world_cleanup (&s_pred); s_pred_live = 0; }
    physics_world_init (&s_pred);
    constraint_pool_init (&s_pred);
    s_pred_live = 1;
    /* Same solver/gravity/restitution settings as the live world (a copy
     * bound to the throwaway world; a stack copy would dangle). Only the
     * iteration count is reduced, and that is measured fidelity-neutral. */
    s_pred_cfg = *mpe_world_cfg (w);
    s_pred_cfg.timestep.solver_iterations = FTRJ_ITERATIONS;
    physics_world_set_config (&s_pred, &s_pred_cfg);
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
    /* The ball about to fire is the oldest carried one. */
    int ball_index = physics_world_index_by_id (w, (uint32_t) r -> intake_balls [0]);
    if (ball_index < 0 || ball_index >= w -> body_count) {
        physics_world_cleanup (&s_pred);
        s_pred_live = 0;
        return -1;
    }
    const rigidbody *ball = &w -> bodies [ball_index];
    /* Add the ball with its REAL radius/mass/material so drag (frontal area),
     * bounce height and rim catch all use the shipped piece (arcade ball or
     * larger BIOBUZZ nectar/pollen). */
    float brad = (isfinite (ball -> radius) && ball -> radius > 0.001f) ? ball -> radius : INTAKE_BALL_RADIUS;
    float bmass = (isfinite (ball -> mass) && ball -> mass > 0.0f) ? ball -> mass : INTAKE_BALL_MASS;
    int pi = physics_world_add_sphere (&s_pred, brad, bmass, (vector3) {0.0f, -50.0f, 0.0f});
    if (pi < 0) {
        physics_world_cleanup (&s_pred);
        s_pred_live = 0;
        return -1;
    }
    rigidbody *pb = &s_pred.bodies [pi];
    pb -> restitution = ball -> restitution;
    pb -> friction_static = ball -> friction_static;
    pb -> friction_kinetic = ball -> friction_kinetic;
    /* Launch state from the SAME helper the shooter uses (inherit chassis vel). */
    intake_launch_state st;
    /* Resolved by the caller (ftrj_begin is static; it re-resolves via the
     * symbol so it stays self-contained). */
    void (*p_launch) (vector3, vector4, vector3, intake_launch_state *) =
        (void (*) (vector3, vector4, vector3, intake_launch_state *)) ftrj_sym ("intake_launch_state_compute");
    if (!p_launch) {
        physics_world_cleanup (&s_pred);
        s_pred_live = 0;
        return -1;
    }
    p_launch (ch -> position, ch -> orientation, ch -> velocity, &st);
    pb -> position = st.muzzle;
    pb -> velocity = st.velocity;
    pb -> angular_velocity = st.spin;
    pb -> kinematic = false;
    pb -> static_state = false;
    rigidbody_wake (pb);
    rigidbody_update_axes (pb);
    s_ticks_done = 0;
    s_npts = 0;
    s_ball_index = pi;
    s_since_restart = 0.0f;
    return pi;
}
void ftc_traj_update (float frame_delta_time) {
    if (!isfinite (frame_delta_time) || frame_delta_time < 0.0f)
        frame_delta_time = FTRJ_DT;
    s_since_restart += frame_delta_time;
    physics_world *w = physics_world_get_primary ();
    ftc_robot * (*p_get) (struct physics_world *, int) =
        (ftc_robot * (*) (struct physics_world *, int)) ftrj_sym ("ftc_fleet_get");
    void (*p_aero) (physics_world *, const int *, int) =
        (void (*) (physics_world *, const int *, int)) ftrj_sym ("intake_balls_aero_step");
    ftc_robot *r = (w && p_get) ? p_get (w, 0) : NULL;
    /* Nothing to preview without a robot that is actually carrying a ball. */
    if (!w || !p_aero || !r || r -> chassis_body < 0 || r -> chassis_body >= w -> body_count ||
        r -> intake_count <= 0) {
        s_npts = 0;
        s_have_key = 0;
        s_ticks_done = 0;
        s_ball_index = -1;
        if (s_pred_live) { physics_world_cleanup (&s_pred); s_pred_live = 0; }
        return;
    }
    rigidbody *ch = &w -> bodies [r -> chassis_body];
    vector3 fwd = vector4_rotate_to_vector3 (ch -> orientation, (vector3) {0.0f, 0.0f, 1.0f});
    float yaw_deg = atan2f (fwd.z, fwd.x) * 57.29578f;
    /* Does the current aim differ enough from the one the running prediction
     * was started from to be worth restarting? */
    int want_restart = 1;
    if (s_have_key && s_pred_live) {
        float dyaw = fabsf (yaw_deg - s_key_yaw_deg);
        if (dyaw > 180.0f)
            dyaw = 360.0f - dyaw;
        float dp = vector3_length (vector3_subtraction (ch -> position, s_key_pos));
        want_restart = (dyaw > FTRJ_HEADING_EPS_DEG || dp > FTRJ_POS_EPS || s_key_carried != r -> intake_count);
    }
    if (want_restart) {
        /* Rate-limit restarts: a prediction takes ~0.2 s to draw, so a faster
         * trigger would starve it (this is what made the first version
         * recompute every single frame). If the cooldown is still running we
         * KEEP ADVANCING the current prediction instead of freezing it. */
        if (!s_pred_live || s_since_restart >= FTRJ_RESTART_COOLDOWN) {
            int pi = ftrj_begin (w, r, ch);
            s_have_key = (pi >= 0);
            s_key_pos = ch -> position;
            s_key_yaw_deg = yaw_deg;
            s_key_carried = r -> intake_count;
        }
    }
    if (!s_pred_live || s_ball_index < 0) {
        return; /* build failed */
    }
    if (s_ticks_done >= FTRJ_MAX_TICKS) {
        return; /* finished; a fresh aim will restart it */
    }
    /* Advance the simulation by a fixed tick budget and extend the polyline. */
    const int pi = s_ball_index;
    int pred_id = (int) s_pred.bodies [pi].object_id;
    if (pred_id < 0) {
        return;
    }
    for (int k = 0; k < FTRJ_TICKS_PER_FRAME && s_ticks_done < FTRJ_MAX_TICKS; k++, s_ticks_done++) {
        p_aero (&s_pred, &pred_id, 1);
        physics_world_step (&s_pred, FTRJ_DT);
        vector3 p = s_pred.bodies [pi].position;
        if (!isfinite (p.x) || !isfinite (p.y) || !isfinite (p.z))
            break;
        if ((s_ticks_done % FTRJ_RECORD_EVERY) == 0 && s_npts < FTRJ_MAX_PTS) {
            s_pts [s_npts++] = p;
        }
        /* Stop once the piece has settled on the floor and stopped: further
         * path is the ball lying still. */
        if (s_ticks_done > 10 && p.y <= s_pred.bodies [pi].radius * 0.6f &&
            vector3_length_squared (s_pred.bodies [pi].velocity) < 0.05f) {
            if (s_npts < FTRJ_MAX_PTS)
                s_pts [s_npts++] = p;
            s_ticks_done = FTRJ_MAX_TICKS;
            break;
        }
    }
}
int ftc_traj_enabled (void) {return s_npts > 1;}
int ftc_traj_points (const vector3 **out_pts) {
    if (out_pts)
        *out_pts = s_pts;
    return s_npts;
}
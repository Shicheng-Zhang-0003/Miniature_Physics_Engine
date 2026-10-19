/* See intake.h: fictional intake + shooter-feed + ball aero. */
#include "intake.h"
#include "core/det_math.h"
#include <math.h>
void intake_mouth_pose (vector3 chassis_pos, vector4 chassis_ori, vector3 *out_center, vector3 *out_half) {
    vector3 fwd = vector4_rotate_to_vector3 (chassis_ori, (vector3) {0.0f, 0.0f, 1.0f});
    vector3 up = vector4_rotate_to_vector3 (chassis_ori, (vector3) {0.0f, 1.0f, 0.0f});
    vector3 c = vector3_addition (chassis_pos, vector3_scaling (fwd, INTAKE_MOUTH_FWD));
    c = vector3_addition (c, vector3_scaling (up, INTAKE_MOUTH_UP));
    if (out_center) { *out_center = c; }
    if (out_half) {
        *out_half = (vector3) {INTAKE_MOUTH_HALF_X, INTAKE_MOUTH_HALF_Y, INTAKE_MOUTH_HALF_Z};
    }
}
static int intake_ball_owned (const int *carried, int carried_n, int id) {
    for (int i = 0; i < carried_n; i++) {
        if (carried [i] == id) { return 1; }
    } return 0;
}
int intake_update (physics_world *world, vector3 chassis_pos, vector4 chassis_ori, vector3 chassis_vel, int intake_on,
                   int carried [INTAKE_MAX_CARRIED], int *carried_n, const int *ball_ids, int ball_n) {
    if (!world || !world -> bodies || !carried || !carried_n) { return 0; }
    intake_prune (world, carried, carried_n);
    if (!intake_on) { return 0; }
    if (*carried_n >= INTAKE_MAX_CARRIED) { return 0; } /* hopper full */
    vector3 mc, mh;
    intake_mouth_pose (chassis_pos, chassis_ori, &mc, &mh);
    int got = 0;
    for (int b = 0; b < ball_n && *carried_n < INTAKE_MAX_CARRIED; b++) {
        if (!ball_ids) { break; }
        int id = ball_ids [b];
        if (id < 0 || intake_ball_owned (carried, *carried_n, id)) { continue; }
        int bi = physics_world_index_by_id (world, (uint32_t) id);
        if (bi < 0 || bi >= world -> body_count) { continue; }
        rigidbody *ball = &world -> bodies [bi];
        if (ball -> kinematic || ball -> static_state) { continue; } /* scored/frozen or floor: not ours */
        vector3 d = vector3_subtraction (ball -> position, mc);
        /* Mouth box is world-axis-aligned (the chassis drives near level;
         * yaw misalignment at large heading is accepted as arcade — the box
         * is generous and capture re-checks every tick). */
        float dx = (d.x > 0.0f) ? d.x : -d.x;
        float dy = (d.y > 0.0f) ? d.y : -d.y;
        float dz = (d.z > 0.0f) ? d.z : -d.z;
        if (dx > mh.x || dy > mh.y || dz > mh.z) { continue; }
        vector3 rel = vector3_subtraction (ball -> velocity, chassis_vel);
        float rsp = sqrtf (vector3_length_squared (rel));
        if (!(rsp < INTAKE_CAPTURE_SPEED) || !isfinite (rsp)) { continue; }
        rigidbody_set_kinematic (ball, true);
        rigidbody_wake (ball);
        ball -> velocity = chassis_vel;
        carried [(*carried_n)++] = id;
        got++;
    } return got;
}
/* Single source of truth for the shot: muzzle pose, launch direction, initial
 * velocity (chassis velocity inherited) and backspin. intake_shoot() applies
 * it to the real ball; the trajectory predictor (ftc_traj.c) simulates a copy
 * through the identical state so the purple preview line cannot drift from the
 * shot that actually fires. No side effects, no world access. */
void intake_launch_state_compute (vector3 chassis_pos, vector4 chassis_ori, vector3 chassis_vel,
                                  intake_launch_state *out) {
    intake_launch_state_compute_elev (chassis_pos, chassis_ori, chassis_vel, INTAKE_LAUNCH_ELEV_DEG, out);
}
/* Elevation-parameterized core. intake_launch_state_compute() above is the
 * 50-degree specialization, so the predictor's resolved symbol and every
 * full-power caller are bit-identical to before (same ops, same order,
 * same values — function boundary changes nothing with FP contraction
 * off). Aimed shots come through here with their elevation. */
void intake_launch_state_compute_elev (vector3 chassis_pos, vector4 chassis_ori, vector3 chassis_vel, float elev_deg,
                                       intake_launch_state *out) {
    if (!out) { return; }
    if (!isfinite (elev_deg)) { elev_deg = INTAKE_LAUNCH_ELEV_DEG; }
    if (elev_deg < INTAKE_LAUNCH_ELEV_MIN) { elev_deg = INTAKE_LAUNCH_ELEV_MIN; }
    if (elev_deg > INTAKE_LAUNCH_ELEV_MAX) { elev_deg = INTAKE_LAUNCH_ELEV_MAX; }
    vector3 fwd = vector4_rotate_to_vector3 (chassis_ori, (vector3) {0.0f, 0.0f, 1.0f});
    /* Launch dir: chassis-forward tilted up by elevation. det_cos/det_sin
     * (fixed-coefficient, bit-identical) — shots stay inside the
     * determinism contract. */
    float el = elev_deg * 3.14159265f / 180.0f;
    float ce = (float) det_cos ((double) el), se = (float) det_sin ((double) el);
    /* World-frame elevation (DESPOT-2026-10-12): the muzzle inherits
     * chassis pitch (braking dips the nose several degrees), which swung
     * apex height ±1 m run to run through sin(). Aim is yaw-only by
     * design, so elevation is measured off world-up with only the yaw of
     * the chassis forward — pitch/roll never enter the shot. */
    vector3 yawfwd = {fwd.x, 0.0f, fwd.z};
    float yl = sqrtf (vector3_length_squared (yawfwd));
    if (!(yl > 1e-6f) || !isfinite (yl)) { yawfwd = (vector3) {0.0f, 0.0f, 1.0f}; } else {
        yawfwd = vector3_scaling (yawfwd, 1.0f / yl);
    }
    vector3 dir = vector3_addition (vector3_scaling (yawfwd, ce), (vector3) {0.0f, se, 0.0f});
    float dl = sqrtf (vector3_length_squared (dir));
    if (dl > 1e-6f && isfinite (dl)) { dir = vector3_scaling (dir, 1.0f / dl); } else { dir = (vector3) {0.0f, 1.0f, 0.0f}; }
    vector3 muzzle = vector3_addition (chassis_pos, vector3_scaling (yawfwd, INTAKE_MUZZLE_FWD));
    muzzle = vector3_addition (muzzle, (vector3) {0.0f, INTAKE_MUZZLE_UP, 0.0f});
    out->muzzle = muzzle;
    out->dir = dir;
    out->velocity = vector3_addition (chassis_vel, vector3_scaling (dir, INTAKE_LAUNCH_SPEED));
    /* Backspin about the chassis lateral axis for Magnus lift (yaw-projected
     * like the launch dir, so roll/pitch never tilt it). */
    vector3 lat = vector4_rotate_to_vector3 (chassis_ori, (vector3) {1.0f, 0.0f, 0.0f});
    lat.y = 0.0f;
    float ll = sqrtf (vector3_length_squared (lat));
    if (ll > 1e-6f && isfinite (ll)) { out->spin = vector3_scaling (lat, -INTAKE_LAUNCH_SPIN / ll); } else { out->spin = vector3_zero (); }
}
int intake_shoot (physics_world *world, vector3 chassis_pos, vector4 chassis_ori, vector3 chassis_vel,
                  int carried [INTAKE_MAX_CARRIED], int *carried_n) {
    return intake_shoot_power (world, chassis_pos, chassis_ori, chassis_vel, carried, carried_n, 1.0f);
}
int intake_shoot_power (physics_world *world, vector3 chassis_pos, vector4 chassis_ori, vector3 chassis_vel,
                        int carried [INTAKE_MAX_CARRIED], int *carried_n, float power) {
    return intake_shoot_aimed (world, chassis_pos, chassis_ori, chassis_vel, carried, carried_n, power,
                               INTAKE_LAUNCH_ELEV_DEG);
}
int intake_shoot_aimed (physics_world *world, vector3 chassis_pos, vector4 chassis_ori, vector3 chassis_vel,
                        int carried [INTAKE_MAX_CARRIED], int *carried_n, float power, float elev_deg) {
    if (!world || !world -> bodies || !carried || !carried_n || *carried_n <= 0) { return -1; }
    if (!isfinite (power)) { power = 1.0f; }
    if (power < INTAKE_LAUNCH_POWER_MIN) { power = INTAKE_LAUNCH_POWER_MIN; }
    if (power > 1.0f) { power = 1.0f; }
    if (!isfinite (elev_deg)) { elev_deg = INTAKE_LAUNCH_ELEV_DEG; }
    if (elev_deg < INTAKE_LAUNCH_ELEV_MIN) { elev_deg = INTAKE_LAUNCH_ELEV_MIN; }
    if (elev_deg > INTAKE_LAUNCH_ELEV_MAX) { elev_deg = INTAKE_LAUNCH_ELEV_MAX; }
    int id = carried [0];
    for (int i = 1; i < *carried_n; i++) { carried [i - 1] = carried [i]; }
    (*carried_n)--;
    int bi = physics_world_index_by_id (world, (uint32_t) id);
    if (bi < 0 || bi >= world -> body_count) { return -1; }
    rigidbody *ball = &world -> bodies [bi];
    intake_launch_state st;
    intake_launch_state_compute_elev (chassis_pos, chassis_ori, chassis_vel, elev_deg, &st);
    rigidbody_set_kinematic (ball, false);
    rigidbody_wake (ball);
    ball -> position = st.muzzle;
    if (power == 1.0f && elev_deg == INTAKE_LAUNCH_ELEV_DEG) {
        /* Exact legacy path (see header): no arithmetic on the shot. */
        ball -> velocity = st.velocity;
        ball -> angular_velocity = st.spin;
    } else {
        /* Scale the robot-relative component only; chassis inheritance is
         * untouched (a moving robot's throw still carries its velocity). */
        vector3 rel = vector3_subtraction (st.velocity, chassis_vel);
        ball -> velocity = vector3_addition (chassis_vel, vector3_scaling (rel, power));
        ball -> angular_velocity = vector3_scaling (st.spin, power);
    }
    rigidbody_update_axes (ball);
    return id;
}
int intake_eject (physics_world *world, vector3 chassis_vel, vector3 chassis_fwd, int carried [INTAKE_MAX_CARRIED],
                  int *carried_n) {
    if (!world || !world -> bodies || !carried || !carried_n) { return 0; }
    int n = 0;
    while (*carried_n > 0) {
        int id = carried [--(*carried_n)];
        int bi = physics_world_index_by_id (world, (uint32_t) id);
        if (bi < 0 || bi >= world -> body_count) { continue; }
        rigidbody *ball = &world -> bodies [bi];
        rigidbody_set_kinematic (ball, false);
        rigidbody_wake (ball);
        ball -> velocity = vector3_addition (chassis_vel, vector3_scaling (chassis_fwd, 1.0f));
        n++;
    } return n;
}
void intake_prune (physics_world *world, int carried [INTAKE_MAX_CARRIED], int *carried_n) {
    if (!world || !carried || !carried_n) { return; }
    int w = 0;
    for (int i = 0; i < *carried_n; i++) {
        if (physics_world_index_by_id (world, (uint32_t) carried [i]) >= 0) { carried [w++] = carried [i]; }
    } *carried_n = w;
}
/* Carry step: hold every possessed ball at its mouth slot (kinematic
 * follow). Runs every tick the intake owns balls, so carried balls ride
 * jumps, turns and slopes instead of being left behind. Slots spread
 * laterally across the mouth in chassis frame. */
void intake_carry_step (physics_world *world, vector3 chassis_pos, vector4 chassis_ori, vector3 chassis_vel,
                        int carried [INTAKE_MAX_CARRIED], int carried_n) {
    if (!world || !world -> bodies || !carried || carried_n <= 0) { return; }
    vector3 mc, mh;
    (void) mh;
    intake_mouth_pose (chassis_pos, chassis_ori, &mc, NULL);
    vector3 lat = vector4_rotate_to_vector3 (chassis_ori, (vector3) {1.0f, 0.0f, 0.0f});
    float ll = sqrtf (vector3_length_squared (lat));
    if (!(ll > 1e-6f) || !isfinite (ll)) { lat = (vector3) {1.0f, 0.0f, 0.0f}; } else {
        lat = vector3_scaling (lat, 1.0f / ll);
    }
    float spacing_default = 2.0f * INTAKE_BALL_RADIUS + 0.004f;
    for (int i = 0; i < carried_n && i < INTAKE_MAX_CARRIED; i++) {
        int bi = physics_world_index_by_id (world, (uint32_t) carried [i]);
        if (bi < 0 || bi >= world -> body_count) { continue; }
        rigidbody *ball = &world -> bodies [bi];
        /* Slot pitch follows the piece radius (BIOBUZZ nectar/pollen are
         * larger than the arcade ball): read it from the body so mixed
         * loads do not overlap. */
        float radius = (isfinite (ball -> half_extensions.x) && ball -> half_extensions.x > 0.001f)
                           ? ball -> half_extensions.x : INTAKE_BALL_RADIUS;
        float spacing = 2.0f * radius + 0.004f;
        if (!(spacing > 0.0f)) { spacing = spacing_default; }
        vector3 slot = vector3_addition (mc, vector3_scaling (lat, ((float) i - (float) (carried_n - 1) * 0.5f) * spacing));
        ball -> position = slot;
        ball -> velocity = chassis_vel;
        rigidbody_wake (ball);
    }
}
void intake_balls_aero_step (physics_world *world, const int *ball_ids, int ball_n) {
    if (!world || !world -> bodies || !ball_ids) { return; }
    /* Same air as module_1's ball step (1.225 kg/m^3, sphere Cd 0.47,
     * Magnus S = 0.1, spin gate 10 rad/s) — cited, separate owner. */
    const float rho = 1.225f, cd = 0.47f;
    for (int b = 0; b < ball_n; b++) {
        int bi = physics_world_index_by_id (world, (uint32_t) ball_ids [b]);
        if (bi < 0 || bi >= world -> body_count) { continue; }
        rigidbody *ball = &world -> bodies [bi];
        if (ball -> kinematic || ball -> static_state || ball -> is_sleeping) { continue; }
        /* Frontal area from the body's own radius (BIOBUZZ nectar/pollen
         * are larger than the arcade ball). */
        float radius = (isfinite (ball -> half_extensions.x) && ball -> half_extensions.x > 0.001f)
                           ? ball -> half_extensions.x : INTAKE_BALL_RADIUS;
        const float area = 3.14159265f * radius * radius;
        float speed = sqrtf (vector3_length_squared (ball -> velocity));
        if (speed > 0.1f && isfinite (speed)) {
            float f = 0.5f * rho * cd * area * speed * speed;
            ball -> force_accumulator = vector3_addition (
                ball -> force_accumulator, vector3_scaling (ball -> velocity, -f / speed));
            ball -> ext_force_this_tick = 1;
        }
        float spin = sqrtf (vector3_length_squared (ball -> angular_velocity));
        if (spin > 10.0f && isfinite (spin) && speed > 0.001f) {
            vector3 sd = vector3_scaling (ball -> angular_velocity, 1.0f / spin);
            vector3 vd = vector3_scaling (ball -> velocity, 1.0f / speed);
            float m = 0.5f * rho * area * spin * radius * speed * 0.1f;
            if (isfinite (m)) {
                ball -> force_accumulator =
                    vector3_addition (ball -> force_accumulator, vector3_scaling (vector3_cross (sd, vd), m));
                ball -> ext_force_this_tick = 1;
            }
        }
    }
}

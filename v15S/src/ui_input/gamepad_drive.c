/* See gamepad_drive.h. */
#include "gamepad_drive.h"
#include "../mpe_engine.h"
#include "../core/mpe_platform.h"
#include "../core/mpe_loader.h"
#include "../core/physics_world.h"
#include "../ecosystem/mfs/modules/ftc/submodules/robot.h"
#include "../ecosystem/mfs/modules/ftc/submodules/drivetrain.h"
#include "../ecosystem/mfs/modules/module_1/submodules/gamepad/gamepad.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* Bundle identity mirrors term_ftc.c (ecosystem bundle first, then the
 * single-module plugin path). */
#define GPD_HANDLE_PRIMARY "mfs-simulator"
#define GPD_HANDLE_MODULE "plugins/mpe_ftc.so"
static void *gpd_sym (const char *sym) {
    void *p = mpe_loader_symbol (GPD_HANDLE_PRIMARY, sym);
    if (p)
        return p;
    return mpe_loader_symbol (GPD_HANDLE_MODULE, sym);
}
/* Cached bundle entry points (resolved lazily; NULL = bundle absent). */
static ftc_robot * (*s_fleet_get) (struct physics_world *, int) = NULL;
static void (*s_mecanum) (ftc_robot *, float, float, float) = NULL;
static bool (*s_pad_init) (gamepad_state *, const char *) = NULL;
static void (*s_pad_poll) (gamepad_state *) = NULL;
static float (*s_pad_axis) (const gamepad_state *, int) = NULL;
static bool (*s_pad_button) (const gamepad_state *, int) = NULL;
static bool (*s_pad_connected) (const gamepad_state *) = NULL;
static void (*s_pad_deadzone) (gamepad_state *, float) = NULL;
static int s_syms_resolved = 0;
static gamepad_state s_pad;
static int s_pad_open_attempted = 0;
static int s_frames_since_retry = 0;
static int s_control_enabled = 1; /* START toggles; default ON like module_1 */
static int s_prev_start = 0;
static int s_was_commanding = 0;
static int s_active = 0;
static int s_reported_stage = -1;
static void gpd_report_stage (int stage) {
    /* One line per transition so a dead controller always says why (bundle,
     * pad, robot) instead of silently no-op'ing. */
    if (stage == s_reported_stage)
        return;
    s_reported_stage = stage;
    switch (stage) {
        case 0:
        fprintf (stderr, "[gamepad] no bundle (mod load ecosystem/mfs/mfs_ecosystem.so first)\n");
        break;
        case 1:
        fprintf (stderr,
                 "[gamepad] bundle loaded, waiting for pad (F310 switch X, /dev/input/js0) and robot (ftc spawn)\n");
        break;
        case 2:
        fprintf (stderr, "[gamepad] pad connected, waiting for robot (ftc spawn)\n");
        break;
        case 3:
        fprintf (stderr, "[gamepad] driving robot 0 (START toggles, LB+RB e-stop)\n");
        break;
        default:
        break;
    }
    fflush (stderr);
}
static void gpd_resolve (void) {
    /* DESPOT-FIX 2026-10-04: this ran once and cached NULLs when the first
     * frame ticked before `mod load` — every later tick then silently
     * no-op'd with a bundle loaded, a pad plugged and a robot spawned. The
     * controller was dead until engine restart, with no diagnostic. Retry
     * until the fleet entry points resolve; the rest may legitimately stay
     * NULL only when the bundle is absent (checked per-tick below). */
    if (s_syms_resolved && s_fleet_get && s_mecanum)
        return;
    s_fleet_get = (ftc_robot * (*) (struct physics_world *, int)) gpd_sym ("ftc_fleet_get");
    s_mecanum = (void (*) (ftc_robot *, float, float, float)) gpd_sym ("drivetrain_mecanum");
    s_pad_init = (bool (*) (gamepad_state *, const char *)) gpd_sym ("gamepad_init");
    s_pad_poll = (void (*) (gamepad_state *)) gpd_sym ("gamepad_poll");
    s_pad_axis = (float (*) (const gamepad_state *, int)) gpd_sym ("gamepad_get_axis");
    s_pad_button = (bool (*) (const gamepad_state *, int)) gpd_sym ("gamepad_get_button");
    s_pad_connected = (bool (*) (const gamepad_state *)) gpd_sym ("gamepad_is_connected");
    s_pad_deadzone = (void (*) (gamepad_state *, float)) gpd_sym ("gamepad_set_deadzone");
    if (s_fleet_get && s_mecanum)
        s_syms_resolved = 1;
}
void gamepad_drive_init (void) {
    gpd_resolve ();
    if (!s_pad_init || s_pad_open_attempted)
        return;
    s_pad_open_attempted = 1;
    memset (&s_pad, 0, sizeof (s_pad));
    s_pad.fd = -1;
    if (s_pad_init (&s_pad, NULL)) {
        if (s_pad_deadzone)
            s_pad_deadzone (&s_pad, 0.15f);
        fprintf (stderr, "[gamepad] opened %s\n", s_pad.device_path [0] ? s_pad.device_path : "(default device)");
        fflush (stderr);
    }
    /* Missing device is not an error: headless rigs, unplugged pad, or
     * MPE_GAMEPAD_DEVICE=disabled all land here; tick retries below. */
}
static void gpd_stop_robot (ftc_robot *r) {
    if (s_mecanum && r)
        s_mecanum (r, 0.0f, 0.0f, 0.0f);
}
void gamepad_drive_tick (void) {
    gpd_resolve ();
    if (!s_fleet_get || !s_mecanum || !s_pad_poll || !s_pad_axis || !s_pad_button || !s_pad_connected) {
        s_active = 0;
        gpd_report_stage (0);
        return; /* no bundle: silent except the one-time hint above */
    }
    if (!s_pad_open_attempted) {
        gamepad_drive_init ();
        if (!s_pad_open_attempted) {
            s_active = 0;
            return;
        }
    }
    /* Hot-plug retry while disconnected (~5 s at 60 fps). */
    if (!s_pad_connected (&s_pad)) {
        s_active = 0;
        gpd_report_stage (1);
        if (s_was_commanding) {
            physics_world *w = physics_world_get_primary ();
            ftc_robot *r = (w && s_fleet_get) ? s_fleet_get (w, 0) : NULL;
            gpd_stop_robot (r);
            s_was_commanding = 0;
        }
        if (++s_frames_since_retry > 300) {
            s_frames_since_retry = 0;
            s_pad_open_attempted = 0; /* re-run init/open next tick */
        }
        return;
    }
    s_frames_since_retry = 0;
    s_pad_poll (&s_pad);
    /* START toggles control even while disabled (else latch-dead). */
    int start_now = s_pad_button (&s_pad, gamepad_button_start) ? 1 : 0;
    if (start_now && !s_prev_start)
        s_control_enabled = !s_control_enabled;
    s_prev_start = start_now;
    physics_world *w = physics_world_get_primary ();
    ftc_robot *r = (w && s_fleet_get) ? s_fleet_get (w, 0) : NULL;
    if (!r) {
        s_active = 0;
        s_was_commanding = 0;
        gpd_report_stage (s_pad_connected (&s_pad) ? 2 : 1);
        return; /* no robot: nothing to command (`ftc spawn` first) */
    }
    if (!s_control_enabled) {
        s_active = 0;
        if (s_was_commanding) {
            gpd_stop_robot (r);
            s_was_commanding = 0;
        }
        return;
    }
    float fwd = -s_pad_axis (&s_pad, gamepad_axis_left_y);
    float str = s_pad_axis (&s_pad, gamepad_axis_left_x);
    float rot = s_pad_axis (&s_pad, gamepad_axis_right_x);
    if (s_pad_button (&s_pad, gamepad_button_lb) && s_pad_button (&s_pad, gamepad_button_rb)) {
        fwd = str = rot = 0.0f; /* LB+RB e-stop */
    }
    s_mecanum (r, fwd, str, rot);
    s_was_commanding = 1;
    s_active = 1;
    gpd_report_stage (3);
}
int gamepad_drive_active (void) {
    return s_active;
}
/* ---- joint watchdog (see header) ---- */
#define FTC_WD_MAXW 8
static int s_wd_tagged = 0;
static float s_wd_mount [FTC_WD_MAXW];
static int s_wd_mount_seen [FTC_WD_MAXW];
static int s_wd_tilt_hot [FTC_WD_MAXW];
static int s_wd_jump_hot [FTC_WD_MAXW];
void ftc_watchdog_tick (void) {
    if (!s_fleet_get) {
        return; /* resolve with the driver (no bundle, no watch) */
    }
    if (!s_wd_tagged) {
        s_wd_tagged = 1;
        const mpe_config_t *wc = NULL;
        physics_world *pw0 = physics_world_get_primary ();
        if (pw0) {
            wc = mpe_world_cfg (pw0);
        }
        fprintf (stderr, "[ftc-watchdog] build %s (%s %s) iters=%d sleep=%d (primary cfg)\n", a3_version_string,
                 __DATE__, __TIME__, wc ? wc -> timestep.solver_iterations : -1, wc ? wc -> sleep.enable : -1);
        fflush (stderr);
    }
    physics_world *w = physics_world_get_primary ();
    ftc_robot *r = (w && s_fleet_get) ? s_fleet_get (w, 0) : NULL;
    if (!r || r -> chassis_body < 0 || r -> chassis_body >= w -> body_count) {
        return;
    }
    rigidbody *ch = &w -> bodies [r -> chassis_body];
    vector3 chx = ch -> cached_axes [0];
    for (int i = 0; i < r -> wheel_count && i < FTC_WD_MAXW; i++) {
        int bi = r -> wheel_bodies [i];
        if (bi < 0 || bi >= w -> body_count) {
            continue;
        }
        rigidbody *wh = &w -> bodies [bi];
        int bad = (!isfinite (wh -> position.x)) || (!isfinite (wh -> position.y)) || (!isfinite (wh -> position.z)) ||
                  (!isfinite (wh -> orientation.w)) || (!isfinite (wh -> orientation.x)) ||
                  (!isfinite (wh -> orientation.y)) || (!isfinite (wh -> orientation.z)) ||
                  (!isfinite (wh -> angular_velocity.x)) || (!isfinite (wh -> angular_velocity.y)) ||
                  (!isfinite (wh -> angular_velocity.z));
        vector3 d = vector3_subtraction (wh -> position, ch -> position);
        float md = vector3_length (d);
        if (!s_wd_mount_seen [i] && isfinite (md)) {
            s_wd_mount_seen [i] = 1;
            s_wd_mount [i] = md;
        }
        float dot = vector3_dot (wh -> cached_axes [0], chx);
        if (dot > 1.0f) {
            dot = 1.0f;
        }
        if (dot < -1.0f) {
            dot = -1.0f;
        }
        float tilt = acosf (dot) * 57.29578f;
        float wsp = vector3_dot (wh -> angular_velocity, wh -> cached_axes [0]);
        if (bad) {
            fprintf (stderr, "[ftc-watchdog] wheel %d NON-FINITE state (pos/ori/vel)\n", i);
            fflush (stderr);
        }
        if (s_wd_mount_seen [i] && isfinite (md) && fabsf (md - s_wd_mount [i]) > 0.05f && !s_wd_jump_hot [i]) {
            s_wd_jump_hot [i] = 1;
            fprintf (stderr,
                     "[ftc-watchdog] wheel %d MOUNT JUMP mount=%.3f base=%.3f tilt=%.1f w=%+.1f "
                     "chpos=(%+.2f,%+.2f,%+.2f)\n",
                     i, md, s_wd_mount [i], tilt, wsp, ch -> position.x, ch -> position.y, ch -> position.z);
            fflush (stderr);
        } else if (s_wd_jump_hot [i] && fabsf (md - s_wd_mount [i]) < 0.025f) {
            s_wd_jump_hot [i] = 0;
        }
        if (tilt > 10.0f && !s_wd_tilt_hot [i]) {
            s_wd_tilt_hot [i] = 1;
            fprintf (stderr, "[ftc-watchdog] wheel %d TILT %.1f deg (mount=%.3f base=%.3f w=%+.1f)\n", i, tilt, md,
                     s_wd_mount [i], wsp);
            fflush (stderr);
        } else if (s_wd_tilt_hot [i] && tilt < 5.0f) {
            s_wd_tilt_hot [i] = 0;
        }
    }
}

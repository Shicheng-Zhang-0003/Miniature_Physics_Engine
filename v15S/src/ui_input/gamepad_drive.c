/* See gamepad_drive.h. */
#include "gamepad_drive.h"
#include "../mpe_engine.h"
#include "../core/mpe_platform.h"
#include "../core/mpe_loader.h"
#include "../core/physics_world.h"
#include "../ecosystem/mfs/modules/ftc/submodules/robot.h"
#include "../ecosystem/mfs/modules/ftc/submodules/drivetrain.h"
#include "../ecosystem/mfs/modules/module_1/submodules/gamepad/gamepad.h"
#include "keydrive.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* Bundle identity mirrors term_ftc.c (ecosystem bundle first, then the
 * single-module plugin path). */
#define GPD_HANDLE_PRIMARY "mfs-simulator"
#define GPD_HANDLE_MODULE "ecosystem/mfs/plugins/mpe_ftc.so"
static void *gpd_sym (const char *sym) {
    void *p = mpe_loader_symbol (GPD_HANDLE_PRIMARY, sym);
    if (p)
        return p;
    return mpe_loader_symbol (GPD_HANDLE_MODULE, sym);
} /* Cached bundle entry points (resolved lazily; NULL = bundle absent). */
static ftc_robot * (*s_fleet_get) (struct physics_world *, int) = NULL;
static void (*s_mecanum) (ftc_robot *, float, float, float) = NULL;
static bool (*s_pad_init) (gamepad_state *, const char *) = NULL;
static void (*s_pad_poll) (gamepad_state *) = NULL;
static float (*s_pad_axis) (const gamepad_state *, int) = NULL;
static float (*s_pad_trigger) (const gamepad_state *, int) = NULL;
static bool (*s_pad_button) (const gamepad_state *, int) = NULL;
static bool (*s_pad_connected) (const gamepad_state *) = NULL;
static void (*s_pad_deadzone) (gamepad_state *, float) = NULL;
static int s_syms_resolved = 0;
static gamepad_state s_pad;
static int s_pad_open_attempted = 0;
static int s_frames_since_retry = 0;
static int s_control_enabled = 1; /* START toggles; default ON like module_1 */
static int s_prev_start = 0;
static int s_prev_back = 0;
static int s_kb_reported = 0;
static int s_kb_intake_last = 0;
static int s_prev_period = 0;
static int s_prev_a = 0;
static int s_prev_y = 0;
static int s_prev_b = 0;
static int s_prev_lt = 0;
/* RT charge: peak trigger held, fired on release so the analog value the
 * player chose is the value that fires (press-edge sampling used to fire at
 * the first frame over 0.5, i.e. ~60% power — the "dying battery" shot). */
static int s_rt_charging = 0;
static float s_rt_charge_peak = 1.0f;
/* Hive-aim solve state (DESPOT-2026-10-22): last solved pair + when/where
 * (frame throttle 96 frames + >0.4 m move or a rotate-stick nudge force a
 * fresh solve; the fleet-side pose cache makes stationary re-asks free). */
static int s_solved_ok = 0;
static float s_solvedPower = 0.0f, s_solvedElev = 0.0f;
static int s_solve_frames = 9999;
static vector3 s_last_solve_pos;
static int s_solve_yaw_dirty = 0;
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
        fprintf (stderr,
                 "[gamepad] driving selected robot (START toggles, BACK cycles robots, LB+RB e-stop, LT intake RT shoot)\n");
        break;
        default:
        break;
    } fflush (stderr);
}
static void gpd_resolve (void) {
    /* DESPOT-FIX 2026-10-04: this ran once and cached NULLs when the first
     * frame ticked before `mod load` — every later tick then silently
     * no-op'd with a bundle loaded, a pad plugged and a robot spawned. The
     * controller was dead until engine restart, with no diagnostic. Retry
     * until the fleet entry points resolve; the rest may legitimately stay
     * NULL only when the bundle is absent (checked per-tick below).
     * DESPOT-2026-10-07 P0-2: cached dlsym pointers were never invalidated
     * on `mod unload` — next tick called into unmapped code (use-after-
     * dlclose). Fix: re-resolve EVERY tick (cheap: two hash lookups) and
     * treat NULL as "bundle absent". Never hold a function pointer across
     * an unload boundary. */
    s_fleet_get = (ftc_robot * (*) (struct physics_world *, int)) gpd_sym ("ftc_fleet_get");
    s_mecanum = (void (*) (ftc_robot *, float, float, float)) gpd_sym ("drivetrain_mecanum");
    s_pad_init = (bool (*) (gamepad_state *, const char *)) gpd_sym ("gamepad_init");
    s_pad_poll = (void (*) (gamepad_state *)) gpd_sym ("gamepad_poll");
    s_pad_axis = (float (*) (const gamepad_state *, int)) gpd_sym ("gamepad_get_axis");
    s_pad_trigger = (float (*) (const gamepad_state *, int)) gpd_sym ("gamepad_get_trigger");
    s_pad_button = (bool (*) (const gamepad_state *, int)) gpd_sym ("gamepad_get_button");
    s_pad_connected = (bool (*) (const gamepad_state *)) gpd_sym ("gamepad_is_connected");
    s_pad_deadzone = (void (*) (gamepad_state *, float)) gpd_sym ("gamepad_set_deadzone");
    if (s_fleet_get && s_mecanum)
        s_syms_resolved = 1;
    else
        s_syms_resolved = 0;
} /* DESPOT-2026-10-07 P0-2: explicit invalidation for unload paths.
 * Loader/terminal unload hooks call this BEFORE dlclose so no stale
 * pointer survives even one tick. Re-resolve above makes this redundant
 * but harmless — defence in depth. */
void gamepad_drive_invalidate (void) {
    s_fleet_get = NULL;
    s_mecanum = NULL;
    s_pad_init = NULL;
    s_pad_poll = NULL;
    s_pad_axis = NULL;
    s_pad_trigger = NULL;
    s_pad_button = NULL;
    s_pad_connected = NULL;
    s_pad_deadzone = NULL;
    s_syms_resolved = 0;
    s_active = 0;
    s_prev_back = 0;
    s_kb_reported = 0;
    s_kb_intake_last = 0;
    s_prev_period = 0;
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
/* Keyboard fallback drive (G V B N C M): the pad owns motion while
 * connected; when no pad is present this drives the SELECTED robot from
 * main_inputs so the game is playable with no controller at all. Text
 * entry (debug terminal, microvim) suppresses it — typing 'g' must not
 * drive — and the same no-input brake the pad applies holds here, so
 * release stops and holds instead of coasting. Returns 1 when it drove
 * or braked this tick (caller skips the pad-disconnect stop). */
static int gpd_keyboard_tick (void) {
    physics_world *w = physics_world_get_primary ();
    if (!w || !s_fleet_get || !s_mecanum)
        return 0;
    if (debug_terminal_is_open ())
        return 0;
    if (microvim_is_active ())
        return 0;
    int g = main_inputs.g_key_pressed ? 1 : 0;
    int v = main_inputs.v_key_pressed ? 1 : 0;
    int b = main_inputs.b_key_pressed ? 1 : 0;
    int n = main_inputs.n_key_pressed ? 1 : 0;
    int c = main_inputs.c_key_pressed ? 1 : 0;
    int m = main_inputs.m_key_pressed ? 1 : 0;
    int (*s_sel) (struct physics_world *) =
        (int (*) (struct physics_world *)) gpd_sym ("ftc_fleet_selected");
    int sel = (s_sel) ? s_sel (w) : 0;
    if (sel < 0)
        sel = 0;
    ftc_robot *r = s_fleet_get (w, sel);
    if (!r)
        return 0;
    /* Intake (`<` hold) + shoot (`>` press edge) on the selected robot.
     * Runs whenever the keyboard path runs — including standing still
     * (stationary collection is valid play). State is edge/level driven
     * with last-sent memory, so holding `.` fires exactly once and a
     * reopened terminal/microvim cannot leave a stuck intake: release
     * always lands as off on return. */
    {
        int want_in = mpe_keydrive_intake (main_inputs.comma_key_pressed ? 1 : 0);
        int cur_sh = main_inputs.period_key_pressed ? 1 : 0;
        int (*k_set) (struct physics_world *, int, int) =
            (int (*) (struct physics_world *, int, int)) gpd_sym ("ftc_intake_set");
        int (*k_shoot) (struct physics_world *, int) =
            (int (*) (struct physics_world *, int)) gpd_sym ("ftc_shoot");
        if (want_in != s_kb_intake_last && k_set) {
            if (k_set (w, sel, want_in) >= 0) { s_kb_intake_last = want_in; }
        }
        if (mpe_keydrive_shoot_edge (s_prev_period, cur_sh) && k_shoot) { k_shoot (w, sel); }
        s_prev_period = cur_sh;
    }
    void (*s_brk) (ftc_robot *, int) = (void (*) (ftc_robot *, int)) gpd_sym ("ftc_robot_set_brake");
    /* Brake on resultant-zero (covers release AND opposing-pair cancels —
     * holding G+B is a finger slip, and must stop-and-hold exactly like a
     * centered stick, never coast with the brake off). */
    if (mpe_keydrive_idle (g, v, b, n, c, m)) {
        /* No keys: brake + zero exactly like pad release, but only if WE
         * were the ones commanding (never yank a pad-driven robot). */
        if (s_was_commanding) {
            if (s_brk) { s_brk (r, 1); } else { gpd_stop_robot (r); }
            s_was_commanding = 0;
        }
        return 0;
    }
    if (!s_kb_reported) {
        s_kb_reported = 1;
        fprintf (stderr, "[gamepad] no pad — keyboard drive active (G/V/B/N/C/M) on robot %d\n", sel);
        fflush (stderr);
    }
    float fwd = 0.0f, strafe = 0.0f, rot = 0.0f;
    mpe_keydrive_axes (g, v, b, n, c, m, &fwd, &strafe, &rot);
    if (s_brk) { s_brk (r, 0); }
    s_mecanum (r, fwd, strafe, rot);
    s_was_commanding = 1;
    s_active = 1;
    return 1;
} /* Release-brake: brake command proportional to a velocity component,
 * dead-zoned under 0.05 m/s and capped at 0.2 power units. */
static float gpd_brake (float v, float k) {
    if (fabsf (v) < 0.05f) return 0.0f;
    float b = -k * v;
    if (b > 0.2f) { b = 0.2f; }
    if (b < -0.2f) { b = -0.2f; }
    return b;
}
void gamepad_drive_tick (void) {
    gpd_resolve ();
    if (!s_fleet_get || !s_mecanum || !s_pad_poll || !s_pad_axis || !s_pad_button || !s_pad_connected) {
        s_active = 0;
        gpd_report_stage (0);
        return; /* no bundle: silent except the one-time hint above */
    }
    /* Controlled-robot resolution (fleet selection when the bundle has it,
     * else robot 0 — the pre-selection behavior). -1 (no robots) flows
     * into the existing no-robot paths via fleet_get NULL. Declared up
     * here so the disconnect-stop path below already follows selection. */
    int (*s_sel) (struct physics_world *) =
        (int (*) (struct physics_world *)) gpd_sym ("ftc_fleet_selected");
    physics_world *wg_sel = physics_world_get_primary ();
    int sel = (s_sel && wg_sel) ? s_sel (wg_sel) : 0;
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
        /* No pad: keyboard fallback owns motion (G V B N C M). It brakes
         * on release itself; the stop below is the safety net for when
         * the keyboard path is suppressed (terminal/microvim open) — and
         * must NOT run on a tick the keyboard just drove. */
        int kb_drove = gpd_keyboard_tick ();
        if (!kb_drove && s_was_commanding) {
            physics_world *w = physics_world_get_primary ();
            ftc_robot *r = (w && s_fleet_get) ? s_fleet_get (w, sel) : NULL;
            gpd_stop_robot (r);
            s_was_commanding = 0;
        }
        if (++s_frames_since_retry > 300) {
            s_frames_since_retry = 0;
            s_pad_open_attempted = 0; /* re-run init/open next tick */
        } return;
    } s_frames_since_retry = 0;
    s_pad_poll (&s_pad);
    /* START toggles control even while disabled (else latch-dead). */
    int start_now = s_pad_button (&s_pad, gamepad_button_start) ? 1 : 0;
    if (start_now && !s_prev_start)
        s_control_enabled = !s_control_enabled;
    s_prev_start = start_now;
    /* BACK cycles the controlled robot (edge-triggered). The deselected
     * robot is commanded to stop FIRST: drive commands persist until
     * changed, so switching without stopping strands a runaway. */
    int back_now = s_pad_button (&s_pad, gamepad_button_back) ? 1 : 0;
    if (back_now && !s_prev_back) {
        int (*s_count) (struct physics_world *) = (int (*) (struct physics_world *)) gpd_sym ("ftc_fleet_count");
        int (*s_select) (struct physics_world *, int) =
            (int (*) (struct physics_world *, int)) gpd_sym ("ftc_fleet_select");
        if (s_count && s_select && wg_sel) {
            int n = s_count (wg_sel);
            int cur = (s_sel) ? s_sel (wg_sel) : 0;
            if (n > 1 && cur >= 0) {
                int nxt = (cur + 1) % n;
                if (s_fleet_get && s_mecanum) {
                    ftc_robot *old = s_fleet_get (wg_sel, cur);
                    if (old) { s_mecanum (old, 0.0f, 0.0f, 0.0f); }
                }
                if (s_select (wg_sel, nxt) == nxt) {
                    sel = nxt;
                    fprintf (stderr, "[gamepad] control -> robot %d/%d\n", nxt, n);
                    fflush (stderr);
                }
            }
        }
    }
    s_prev_back = back_now;
    /* F310 arcade profile (user config): left trigger = intake (hold),
     * right trigger = shoot (press). A/Y/B kept as toggles/fallback.
     * Edge/level-triggered; symbols resolve from the bundle and may be NULL
     * on older builds — then the buttons quietly do nothing. */
    {
        float lt = s_pad_trigger ? s_pad_trigger (&s_pad, gamepad_axis_left_trigger) : 0.0f;
        float rt = s_pad_trigger ? s_pad_trigger (&s_pad, gamepad_axis_right_trigger) : 0.0f;
        int a_now = s_pad_button (&s_pad, gamepad_button_a) ? 1 : 0;
        int y_now = s_pad_button (&s_pad, gamepad_button_y) ? 1 : 0;
        int b_now = s_pad_button (&s_pad, gamepad_button_b) ? 1 : 0;
        /* Re-resolved every tick (P0-2: never hold dlsym'd pointers across
         * an unload boundary). */
        int (*s_toggle) (struct physics_world *, int) =
            (int (*) (struct physics_world *, int)) gpd_sym ("ftc_intake_toggle");
        int (*s_set) (struct physics_world *, int, int) =
            (int (*) (struct physics_world *, int, int)) gpd_sym ("ftc_intake_set");
        int (*s_shoot) (struct physics_world *, int) =
            (int (*) (struct physics_world *, int)) gpd_sym ("ftc_shoot");
        int (*s_shoot_pwr) (struct physics_world *, int, float) =
            (int (*) (struct physics_world *, int, float)) gpd_sym ("ftc_shoot_power");
        int (*s_shoot_aim) (struct physics_world *, int, float, float) =
            (int (*) (struct physics_world *, int, float, float)) gpd_sym ("ftc_shoot_aimed");
        int (*s_aim_set) (struct physics_world *, float, float) =
            (int (*) (struct physics_world *, float, float)) gpd_sym ("ftc_aim_params_set");
        int (*s_aim_solve) (struct physics_world *, float *, float *) =
            (int (*) (struct physics_world *, float *, float *)) gpd_sym ("ftc_aim_params_solve");
        int (*s_eject) (struct physics_world *, int) =
            (int (*) (struct physics_world *, int)) gpd_sym ("ftc_eject");
        physics_world *wg = physics_world_get_primary ();
        int lt_now = (lt > 0.3f) ? 1 : 0; /* LT: hold = intake, release = off */
        if (lt_now && !s_prev_lt && s_set && wg) { s_set (wg, sel, 1); }
        if (!lt_now && s_prev_lt && s_set && wg) { s_set (wg, sel, 0); }
        /* RT: pull to charge, RELEASE to fire (soft lobs for hive mouths,
         * full for range). The old press-edge fire sampled the trigger at
         * the first frame over 0.5, so a fast pull fired at ~60% power while
         * the purple line previewed the previous full-power shot. Charging
         * holds the PEAK trigger depth and the live aim is published every
         * tick (ftc_aim_params_set), so the line matches the shot. Elevation
         * rides the right stick Y (up = steeper, 50 deg centred, 20..80):
         * right-X still rotates, so aiming is twin-stick. Older bundles fall
         * back down the chain (aimed -> power -> full). */
        float ry = s_pad_axis ? s_pad_axis (&s_pad, gamepad_axis_right_y) : 0.0f;
        /* Centred default 65 deg (was 50): flat arcs clip the tilted mouth's
         * underside (the "belly" shooter); steeper starts drop inside.
         * Stick deflection still swings 35..95, clamped to the 20..80 band. */
        float elev = 65.0f - 30.0f * ry;
        if (!(elev >= 20.0f) || !(elev <= 80.0f)) { elev = 65.0f; }
        if (rt > 0.2f) {
            if (!s_rt_charging) { s_rt_charging = 1; s_rt_charge_peak = rt; }
            else if (rt > s_rt_charge_peak) { s_rt_charge_peak = rt; }
        }
        /* DESPOT-2026-10-22 hive-aim: the entering band is elevation 55-75
         * at power 0.62-0.82 (13% of the shipped aim space) and invisible
         * to eye-aiming — stick-centred 50 deg always clips the tilted
         * mouth's underside. While charging with the right stick centred
         * (|ry| < 0.08), the aim is the fleet's SOLVED (power, elevation)
         * pair (ftc_aim_params_solve: caches on pose, so re-asks while
         * stationary are free; solves re-run from the throttle gate below
         * while moving). Touching the stick returns to raw twin-stick aim
         * (this block only ever changes the centred-stick default). The
         * trigger charge still halves as the fire commit — with the stick
         * centred the solved arc previews and fires as-is; deflected
         * sticks keep the old charge+elevation semantics. */
        int stick_centred = fabsf (ry) < 0.08f;
        float pubPow = s_rt_charge_peak, pubElev = elev;
        if (rt > 0.2f) {
            /* chassis pose for the re-solve throttle (second dlsym, same
             * per-tick pattern: never held across an unload boundary) */
            ftc_robot *(*p_gg2) (struct physics_world *, int) =
                (ftc_robot * (*) (struct physics_world *, int)) gpd_sym ("ftc_fleet_get");
            ftc_robot *cr = (wg && p_gg2) ? p_gg2 (wg, sel) : NULL;
            vector3 chp = (cr && cr -> chassis_body >= 0) ? wg -> bodies [cr -> chassis_body].position
                                                              : (vector3) {0.0f, 0.0f, 0.0f};
            /* re-solve throttle: >=1.6 s (96 frames) or a >0.4 m move or a
             * rotate-stick nudge; the fleet-side pose cache makes
             * stationary re-asks free */
            s_solve_frames++; /* the timer must tick while charging, or a
                                 stationary hold never re-asks after the
                                 first solve (moved mouth on a tip re-pose) */
            float pdx = chp.x - s_last_solve_pos.x, pdz = chp.z - s_last_solve_pos.z;
            int need = (s_solve_frames >= 96) || (pdx * pdx + pdz * pdz > 0.4f * 0.4f) || s_solve_yaw_dirty;
            if (need && s_aim_solve && wg && stick_centred && cr) {
                float sp = 0.0f, se = 0.0f;
                int rv = s_aim_solve (wg, &sp, &se);
                if (rv == 0) {
                    s_solved_ok = 1; s_solvedPower = sp; s_solvedElev = se;
                } else {
                    s_solved_ok = 0; /* pose is outside the entering corridor */
                }
                s_solve_frames = 0;
                s_last_solve_pos = chp;
                s_solve_yaw_dirty = 0;
            }
            if (stick_centred && s_solved_ok) {
                pubPow = s_solvedPower;
                pubElev = s_solvedElev;
            }
        } else {
            s_solve_frames = 9999; /* idle: force a solve at the next charge */
        }
        if (s_aim_set && wg) { s_aim_set (wg, pubPow, pubElev); }
        if (s_rt_charging && rt <= 0.2f) {
            s_rt_charging = 0;
            if (s_rt_charge_peak < 0.3f) { s_rt_charge_peak = 0.3f; }
            /* fire the PUBLISHED aim (solved pair while the stick was
             * centred, raw charge+stick-elevation otherwise) */
            if (s_shoot_aim && wg) { s_shoot_aim (wg, sel, pubPow, pubElev); }
            else if (s_shoot_pwr && wg) { s_shoot_pwr (wg, sel, pubPow); }
            else if (s_shoot && wg) { s_shoot (wg, sel); }
        }
        if (a_now && !s_prev_a && s_toggle && wg) { s_toggle (wg, sel); }
        if (y_now && !s_prev_y && wg) {
            s_rt_charge_peak = 1.0f;
            if (s_aim_set) { s_aim_set (wg, 1.0f, elev); }
            if (s_shoot_aim) { s_shoot_aim (wg, sel, 1.0f, elev); }
            else if (s_shoot) { s_shoot (wg, sel); }
        }
        if (b_now && !s_prev_b && s_eject && wg) { s_eject (wg, sel); }
        s_prev_lt = lt_now;
        s_prev_a = a_now;
        s_prev_y = y_now;
        s_prev_b = b_now;
    }
    physics_world *w = physics_world_get_primary ();
    ftc_robot *r = (w && s_fleet_get) ? s_fleet_get (w, sel) : NULL;
    if (!r) {
        s_active = 0;
        s_was_commanding = 0;
        gpd_report_stage (s_pad_connected (&s_pad) ? 2 : 1);
        return; /* no robot: nothing to command (`ftc spawn` first) */
    }
    if (!s_control_enabled) {
        s_active = 0;
        /* Control disabled: motor power to BRAKE (not just a command zero). */
        void (*s_brk) (ftc_robot *, int) = (void (*) (ftc_robot *, int)) gpd_sym ("ftc_robot_set_brake");
        if (s_brk) { s_brk (r, 1); } else { gpd_stop_robot (r); }
        s_was_commanding = 0;
        return;
    } float fwd = -s_pad_axis (&s_pad, gamepad_axis_left_y);
    float str = s_pad_axis (&s_pad, gamepad_axis_left_x);
    float rot = s_pad_axis (&s_pad, gamepad_axis_right_x);
    if (fabsf (rot) > 0.1f) { s_solve_yaw_dirty = 1; } /* rotate -> stale solve */
    /* DESPOT-2026-10-14 (handling, expo): linear sticks hand the driver the
     * full yaw runaway (pre-governor plant spins past 13 rad/s) with
     * millimeter precision at centre. Expo (x^1.5, endpoints preserved)
     * gives fine centre control without capping full deflection — the
     * governor, not the stick, owns the top end. UI-side only: headless
     * gates drive the API directly and never see this. Buttons untouched. */
    {
        float ax [3] = {fwd, str, rot};
        for (int ai = 0; ai < 3; ai++) {
            float v = ax [ai];
            if (isfinite (v)) {
                float m = (v < 0.0f) ? -v : v;
                ax [ai] = ((v < 0.0f) ? -1.0f : 1.0f) * m * sqrtf (m);
            }
        }
        fwd = ax [0];
        str = ax [1];
        rot = ax [2];
    }
    /* BRAKE on no stick input (user rule): with every drive axis at rest
     * (or LB+RB e-stop), motor power is BRAKE — command 0 + forced regen
     * idle brake — so the wheels stop and hold instead of running on. */
    int estop = (s_pad_button (&s_pad, gamepad_button_lb) && s_pad_button (&s_pad, gamepad_button_rb)) ? 1 : 0;
    int no_input = estop || (fabsf (fwd) < 0.05f && fabsf (str) < 0.05f && fabsf (rot) < 0.05f);
    void (*s_brk) (ftc_robot *, int) = (void (*) (ftc_robot *, int)) gpd_sym ("ftc_robot_set_brake");
    if (s_brk) {
        s_brk (r, no_input ? 1 : 0);
    }
    if (no_input) {
        fwd = str = rot = 0.0f;
        if (!s_brk) { s_mecanum (r, 0.0f, 0.0f, 0.0f); } /* fallback with an older bundle */
    } else {
        /* RELEASE-BRAKE: a released axis still coasts for seconds because
         * the held axis keeps every wheel "commanded", so the idle regen
         * never engages (measured: forward 1.0 -> 0.15 in ~2.8 s under pure
         * rotate; full release snaps to a stop in ~0.5 s). When an axis is
         * at rest, command a brake proportional to the chassis body-frame
         * velocity along that axis: common-mode command (real motor torque,
         * not an injected force) that rides the held axis without fighting
         * it. +strafe -> robot right (-X); +rotate -> omega_y negative
         * (drive_directions steady-state -5.68 rad/s per unit). */
        rigidbody *ch = (r && r->chassis_body >= 0 && r->chassis_body < w->body_count)
                            ? &w->bodies[r->chassis_body]: NULL;
        if (ch) {
            if (fabsf (fwd) < 0.05f) {
                float vf = vector3_dot (ch->velocity, ch->cached_axes[2]);
                fwd += gpd_brake (vf, 0.15f);
            }
            if (fabsf (str) < 0.05f) {
                float vs = -vector3_dot (ch->velocity, ch->cached_axes[0]);
                str += gpd_brake (vs, 0.15f);
            }
            if (fabsf (rot) < 0.05f) {
                float vr = -ch->angular_velocity.y / 5.68f;
                rot += gpd_brake (vr, 0.9f);
            }
        }
        s_mecanum (r, fwd, str, rot);
    }
    s_was_commanding = 1;
    s_active = 1;
    gpd_report_stage (3);
}
int gamepad_drive_active (void) {return s_active;}
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
        if (pw0) { wc = mpe_world_cfg (pw0); }
        fprintf (stderr, "[ftc-watchdog] build %s (%s %s) iters=%d sleep=%d (primary cfg)\n", a3_version_string,
                 __DATE__, __TIME__, wc ? wc -> timestep.solver_iterations : -1, wc ? wc -> sleep.enable : -1);
        fflush (stderr);
    } physics_world *w = physics_world_get_primary ();
    /* Watch the controlled robot (re-resolved per call; never held). */
    int (*w_sel) (struct physics_world *) = (int (*) (struct physics_world *)) gpd_sym ("ftc_fleet_selected");
    int ws = (w_sel && w) ? w_sel (w) : 0;
    ftc_robot *r = (w && s_fleet_get) ? s_fleet_get (w, ws) : NULL;
    if (!r || r -> chassis_body < 0 || r -> chassis_body >= w -> body_count) { return; }
    rigidbody *ch = &w -> bodies [r -> chassis_body];
    vector3 chx = ch -> cached_axes [0];
    for (int i = 0; i < r -> wheel_count && i < FTC_WD_MAXW; i++) {
        int bi = r -> wheel_bodies [i];
        if (bi < 0 || bi >= w -> body_count) { continue; }
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
        } float dot = vector3_dot (wh -> cached_axes [0], chx);
        if (dot > 1.0f) { dot = 1.0f; }
        if (dot < -1.0f) { dot = -1.0f; }
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
        } else if (s_wd_jump_hot [i] && fabsf (md - s_wd_mount [i]) < 0.025f) { s_wd_jump_hot [i] = 0; }
        if (tilt > 10.0f && !s_wd_tilt_hot [i]) {
            s_wd_tilt_hot [i] = 1;
            fprintf (stderr, "[ftc-watchdog] wheel %d TILT %.1f deg (mount=%.3f base=%.3f w=%+.1f)\n", i, tilt, md,
                     s_wd_mount [i], wsp);
            fflush (stderr);
        } else if (s_wd_tilt_hot [i] && tilt < 5.0f) { s_wd_tilt_hot [i] = 0; }
    }
}

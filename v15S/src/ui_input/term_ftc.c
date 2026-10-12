/* `ftc` terminal command — spawn and inspect the single robot, plus the
 * arcade game league (fictional intake, balls, suspended bucket).
 *
 * Streamlined 2026-10-04 (v15S release blocker): one field, one robot, one
 * controller, full mecanum drive — and the controller is the physical F310
 * gamepad (see ui_input/gamepad_drive.c), NOT the terminal. Verbs:
 *   ftc spawn [x y z]          spawn THE mecanum robot + game bucket (refuses when one exists)
 *   ftc telemetry               pose, odometry, battery, wheels, game state
 *   ftc intake on|off           fictional intake capture on/off
 *   ftc eject                   drop carried balls (no launch)
 *   ftc shoot                   launch oldest carried ball at the bucket arc
 *   ftc balls [n]               scatter n arcade balls ahead (default 3)
 *   ftc pieces [nectar pollen]  stage BIOBUZZ pieces (default 4 nectar + 4 pollen)
 *   ftc bucket [x y z]          spawn/move the suspended bucket (default 0 1.0 4.0)
 *   ftc score                   carried / balls / scored / bucket readout
 *   ftc view [0|1|2]            camera: 0 free, 1 field-centric follow, 2 robot-centric
 * Requires the bundle: `mod load ecosystem/mfs/mfs_ecosystem.so` first
 * (symbols resolve from the loaded handle; the engine never links FTC).
 * Spawn auto-attaches the ftc-fleet module to the primary world, ensures the
 * tile field with perimeter walls (balls can't roll off the grid), drops the
 * game bucket, and stages regulation-size BIOBUZZ nectar and pollen pieces.
 *
 * PARKED (not deleted): `drive`/`stop` (the gamepad owns all motion now),
 * `list`/`preset` inventory, tank drive, preset selection, multi-robot
 * indices. The fleet API, the bundle `command()` surface (`eco command
 * mfs-simulator ...`), tank drivetrain code and all tests keep the full
 * surface — this command is spawn+inspect, not the API ceiling. Game module
 * (intake/shooter/balls) and the GTK robot registry are parked with it;
 * see README_MFS.md.
 */
#include "../core/mpe_platform.h"
#include "term_priv.h"
#include "../core/mpe_loader.h"
#include "../core/mpe_registry.h"
#include "../core/physics_world.h"
#include "ftc_view.h"
#include "../ecosystem/mfs/modules/ftc/ftc_fleet.h"
#include "../ecosystem/mfs/modules/ftc/submodules/robot.h"
#include "../ecosystem/mfs/modules/ftc/submodules/drivetrain.h"
#include "../ecosystem/mfs/modules/ftc/submodules/motor_presets.h"
#include "../ecosystem/mfs/modules/ftc/submodules/battery.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
/* Bundle identity for symbol resolution (ecosystem name registered by
 * the loader; falls back to the module .so path scheme). */
#define FTC_HANDLE_PRIMARY "mfs-simulator"
#define FTC_HANDLE_MODULE "ecosystem/mfs/plugins/mpe_ftc.so"
static void *ftc_sym (const char *sym) {
    void *p = mpe_loader_symbol (FTC_HANDLE_PRIMARY, sym);
    if (p)
        return p;
    return mpe_loader_symbol (FTC_HANDLE_MODULE, sym);
}
static float ftc_argf (char **argv, int i, int argc, float dflt) {
    if (i < argc && argv [i]) {
        char *end = NULL;
        double v = strtod (argv [i], &end);
        if (end != argv [i] && isfinite (v))
            return (float) v;
    } return dflt;
} /* Tile floor guarantee: robots need frictional contact (the frictionless
 * emergency backstop yields slip-regime artifacts). Adds a 60x60 tile
 * slab (top y=0, mu 1.0/0.8) only when no static floor-like body already
 * covers the origin. Reported, never silent.
 * DESPOT-2026-10-04b: was 20x20 — a robot driving ~1 m/s reaches the edge
 * in ~10 s and drops a wheel off it (matches a live "wheel snapped"
 * report timeline exactly). 60x60 matches the proven F10 validation slab
 * and stays inside the broadphase cell-span budget. */
static int ftc_ensure_floor (physics_world *w) {
    if (!w)
        return -1;
    for (int i = 0; i < w -> body_count; i++) {
        rigidbody *b = &w -> bodies [i];
        if (!b -> static_state && !(b -> mass == 0.0f))
            continue;
        float top = b -> position.y + b -> half_extensions.y;
        if (top > -0.05f && top < 0.05f && fabsf (b -> position.x) < 5.0f && fabsf (b -> position.z) < 5.0f &&
            b -> half_extensions.x >= 5.0f && b -> half_extensions.z >= 5.0f) {
            return 0;
        }
    }
    int f = physics_world_add_cube (w, (vector3) {0.0f, -0.5f, 0.0f}, (vector3) {30.0f, 0.5f, 30.0f}, 0.0f);
    if (f < 0)
        return -1;
    w -> bodies [f].friction_static = 1.0f;
    w -> bodies [f].friction_kinetic = 0.8f;
    /* NEUTRAL restitution (min-combine => body e), matching the engine's
     * infinite static plane (physics_world.c: static_plane_body.restitution
     * = 1.0, "neutral"). A hard e=0 slab here overrode every ball's own
     * bounce via the fminf combine in collision_solver.c:942, so arcade
     * pieces (e=0.65) landed dead. Robots carry e=0 themselves, so min()
     * still kills their bounce — only pieces rebound. */
    w -> bodies [f].restitution = 1.0f;
    term_out ("mpe: ftc: tile floor added (robots need frictional contact)\n");
    return 1;
} /* Perimeter walls: static thin boxes on the four edges of the tile field
 * (user config — "grid with walls") so arcade balls (pollen/nectar) and the
 * robot cannot roll off. Idempotent: skips if a long thin static box already
 * sits near an edge. Returns 0 ok, -1 error. */
static int ftc_ensure_walls (physics_world *w, float half, float height) {
    if (!w || half <= 0.0f || height <= 0.0f)
        return -1;
    float hh = 0.5f * height;
    const vector3 plan [4] = {{half, hh, 0.0f}, {-half, hh, 0.0f}, {0.0f, hh, half}, {0.0f, hh, -half}};
    const vector3 size [4] = {{0.2f, hh, half}, {0.2f, hh, half}, {half, hh, 0.2f}, {half, hh, 0.2f}};
    for (int p = 0; p < 4; p++) {
        int have = 0;
        for (int i = 0; i < w -> body_count; i++) {
            rigidbody *b = &w -> bodies [i];
            if (!b -> static_state && !(b -> mass == 0.0f))
                continue;
            if (fabsf (b -> position.x - plan [p].x) < 0.5f && fabsf (b -> position.z - plan [p].z) < 0.5f &&
                (b -> half_extensions.x < 1.0f || b -> half_extensions.z < 1.0f)) {
                have = 1;
                break;
            }
        }
        if (have)
            continue;
        int wall = physics_world_add_cube (w, plan [p], size [p], 0.0f);
        if (wall < 0)
            return -1;
        w -> bodies [wall].friction_static = 1.0f;
        w -> bodies [wall].friction_kinetic = 0.8f;
        /* Neutral restitution: pieces bounce off the perimeter like off the
         * floor (min-combine => body e). e=0 here made every wall a dead
         * stop; robots still lose their bounce via their own e=0. */
        w -> bodies [wall].restitution = 1.0f;
    } return 0;
} /* Ensure the ftc-fleet tick module drives the primary world (idempotent).
 * Returns 0 when driving, -1 when the bundle is missing. Engine APIs
 * (registry, attach) are called directly — this TU links into the
 * engine; only FTC APIs resolve through the bundle handle. */
static int ftc_ensure_driving (void) {
    const mpe_module_desc_t *dmod = mpe_find_module ("ftc-fleet");
    if (!dmod) {
        /* Bundle path: the inner FTC descriptor is still dlsym-able. */
        dmod = (const mpe_module_desc_t *) ftc_sym ("mpe_module_desc");
    }
    if (!dmod) {
        term_err ("mpe: ftc: bundle not loaded (from v15S/src: mod load ecosystem/mfs/mfs_ecosystem.so, or mod load "
                  "ecosystem/mfs/plugins/mpe_ftc.so)\n");
        return -1;
    } physics_world *w = physics_world_get_primary ();
    if (!w) {
        term_err ("mpe: ftc: no primary world\n");
        return -1;
    }
    for (int i = 0; i < w -> tick_module_count; i++) {
        if (w -> tick_modules [i] == dmod)
            return 0;
    }
    if (physics_world_attach_module (w, dmod) < 0) {
        term_err ("mpe: ftc: attach failed\n");
        return -1;
    } return 0;
} /* Iteration guarantee: the 40:1 chassis/wheel stacked mass ratio cannot
 * converge below 128 sequential-impulse iterations (every MFS test pins
 * 128; the MFS suite documents that default 64 cannot converge it). A
 * robot spawned into a 64-iteration world wobbles its revolute axles
 * loose within seconds (measured 15° tilt) and never settles. Raise with
 * a loud report, never silently; already-adequate configs are untouched. */
static void ftc_ensure_iterations (void) {
    if (g_cfg.timestep.solver_iterations < 128) {
        g_cfg.timestep.solver_iterations = 128;
        term_out ("mpe: ftc: solver iterations raised to 128 (robot joints need it; was lower)\n");
    }
}
void cmd_ftc (int argc, char **argv) {
    if (argc < 2) {
        term_err ("mpe: ftc: usage: ftc spawn [x y z] (spawns bucket + BIOBUZZ pieces) | telemetry | intake on|off | eject | shoot | balls [n] | pieces [nectar pollen] | bucket [x y z] | score | view [0|1|2]\n");
        return;
    } physics_world *w = physics_world_get_primary ();
    /* Single-robot accessors (index 0 implied; bundle API keeps indices). */
    ftc_robot * (*p_get) (struct physics_world *, int) =
        (ftc_robot * (*) (struct physics_world *, int)) ftc_sym ("ftc_fleet_get");
    int (*p_count) (struct physics_world *) = (int (*) (struct physics_world *)) ftc_sym ("ftc_fleet_count");
    if (term_str_eq (argv [1], "spawn")) {
        if (ftc_ensure_driving () != 0)
            return;
        int (*p_spawn) (struct physics_world *, float, float, float, motor_preset_id, ftc_drivetrain_type) =
            (int (*) (struct physics_world *, float, float, float, motor_preset_id, ftc_drivetrain_type)) ftc_sym (
                "ftc_fleet_spawn");
        float (*p_rest) (void) = (float (*) (void)) ftc_sym ("ftc_robot_rest_height");
        const char * (*p_name) (motor_preset_id) = (const char * (*) (motor_preset_id)) ftc_sym ("motor_preset_name");
        if (!p_spawn || !p_rest || !p_name || !p_count || !w) {
            term_err ("mpe: ftc: bundle API incomplete\n");
            return;
        }
        /* One robot: refuse a second instead of striding a fleet. Fleet
         * siblings stay available through the bundle/API surface. */
        if (p_count (w) > 0) {
            term_err ("mpe: ftc: one robot already (scene_clear to reset the field)\n");
            return;
        }
        /* Default: first preset matching 5203 26.9 (223 RPM workhorse). */
        int preset = 0;
        for (int id = 0; id < 512; id++) {
            const char *nm = p_name ((motor_preset_id) id);
            if (!nm || strcmp (nm, "unknown") == 0)
                break;
            if (strstr (nm, "5203") && strstr (nm, "26.9")) {
                preset = id;
                break;
            }
        } float x = ftc_argf (argv, 2, argc, 0.0f);
        float y = ftc_argf (argv, 3, argc, p_rest ());
        float z = ftc_argf (argv, 4, argc, 0.0f);
        if (ftc_ensure_floor (w) < 0) {
            term_err ("mpe: ftc: could not ensure floor\n");
            return;
        } ftc_ensure_iterations ();
        if (ftc_ensure_walls (w, 30.0f, 1.0f) < 0) {
            term_err ("mpe: ftc: could not ensure perimeter walls\n");
            return;
        }
        int idx = p_spawn (w, x, y, z, (motor_preset_id) preset, FTC_DRIVETRAIN_MECANUM);
        if (idx < 0) {
            term_err ("mpe: ftc: spawn failed\n");
            return;
        }
        char buf [224];
        /* Spawn the default suspended bucket with the robot so the game is
         * playable immediately (user config: `ftc spawn` = full field). */
        int (*p_bk) (struct physics_world *, float, float, float) =
            (int (*) (struct physics_world *, float, float, float)) ftc_sym ("ftc_bucket_spawn");
        if (p_bk && p_bk (w, 0.0f, 1.0f, 4.0f) == 0) {
            term_out ("mpe: ftc: bucket at (0.00,1.00,4.00) (ftc bucket [x y z] to move)\n");
        } else if (p_bk) {
            term_err ("mpe: ftc: bucket spawn failed\n");
        }
        /* Stage BIOBUZZ scoring elements by default (regulation sizes) so
         * the driver can intake and launch immediately. */
        int (*p_piece) (struct physics_world *, int, int) =
            (int (*) (struct physics_world *, int, int)) ftc_sym ("ftc_biobuzz_spawn");
        if (p_piece) {
            int np = p_piece (w, 4, 4);
            snprintf (buf, sizeof (buf),
                      "mpe: ftc: %d BIOBUZZ piece(s) staged (nectar 9.1cm, pollen 7.1cm)\n", np);
            term_out (buf);
        }
        snprintf (buf, sizeof (buf), "mpe: ftc: robot at (%.2f,%.2f,%.2f) %s mecanum\n", x, y, z,
                  p_name ((motor_preset_id) preset));
        term_ok (buf);
        return;
    }
    if (term_str_eq (argv [1], "telemetry")) {
        if (!p_get) {
            term_err ("mpe: ftc: bundle not loaded\n");
            return;
        } float (*p_volt) (const battery *, float) = (float (*) (const battery *, float)) ftc_sym ("battery_get_voltage");
        int (*p_fuse) (const battery *) = (int (*) (const battery *)) ftc_sym ("battery_fuse_tripped");
        ftc_robot *r = p_get (w, 0);
        if (!r) {
            term_err ("mpe: ftc: no robot (ftc spawn first)\n");
            return;
        } char buf [256];
        float px = 0, py = 0, pz = 0;
        if (r -> chassis_body >= 0 && r -> chassis_body < w -> body_count) {
            px = w -> bodies [r -> chassis_body].position.x;
            py = w -> bodies [r -> chassis_body].position.y;
            pz = w -> bodies [r -> chassis_body].position.z;
        }
        snprintf (buf, sizeof (buf), "mpe: ftc: robot pos=(%.3f,%.3f,%.3f) odom=(%.3f,%.3f,%.3f)%s\n", px, py, pz,
                  r -> odom_x, r -> odom_z, r -> odom_theta, r -> odom_slip ? " SLIP" : "");
        term_out (buf);
        if (p_volt && p_fuse) {
            float isum = 0.0f;
            for (int k = 0; k < r -> wheel_count; k++)
                isum += r -> wheel_motors [k].current;
            snprintf (buf, sizeof (buf), "mpe: ftc: battery %.2fV (%.0f%%)%s\n", p_volt (&r -> battery, isum),
                      r -> battery.charge_fraction * 100.0f, p_fuse (&r -> battery) ? " FUSE-TRIPPED" : "");
            term_out (buf);
        }
        for (int k = 0; k < r -> wheel_count; k++) {
            snprintf (buf, sizeof (buf), "mpe: ftc: wheel %d cmd=%+.2f rpm=%+.0f I=%+.2fA\n", k,
                      r -> wheel_motors [k].command, r -> wheel_motors [k].rpm, r -> wheel_motors [k].current);
            term_out (buf);
        }
        {
            int (*p_stats) (struct physics_world *, int *, int *, int *, int *) =
                (int (*) (struct physics_world *, int *, int *, int *, int *)) ftc_sym ("ftc_game_stats");
            if (p_stats) {
                int carried = 0, balls = 0, scored = 0, bucket = 0;
                if (p_stats (w, &carried, &balls, &scored, &bucket) == 0) {
                    snprintf (buf, sizeof (buf), "mpe: ftc: game carried=%d balls=%d scored=%d bucket=%s intake=%s\n",
                              carried, balls, scored, bucket ? "up" : "--",
                              r -> intake_on ? "on" : "off");
                    term_out (buf);
                }
            }
        }
        return;
    }
    /* Arcade game verbs: all dlsym'd (bundle holds the implementation). */
    if (term_str_eq (argv [1], "intake")) {
        int (*p_set) (struct physics_world *, int, int) =
            (int (*) (struct physics_world *, int, int)) ftc_sym ("ftc_intake_set");
        if (!p_set || !w) {
            term_err ("mpe: ftc: bundle API incomplete\n");
            return;
        }
        int on = (argc > 2 && (term_str_eq (argv [2], "off") || term_str_eq (argv [2], "0"))) ? 0 : 1;
        int rc = p_set (w, 0, on);
        if (rc < 0) {
            term_err ("mpe: ftc: no robot (ftc spawn first)\n");
            return;
        }
        char buf [128];
        snprintf (buf, sizeof (buf), "mpe: ftc: intake %s\n", on ? "on" : "off");
        term_ok (buf);
        return;
    }
    if (term_str_eq (argv [1], "eject")) {
        int (*p_ej) (struct physics_world *, int) = (int (*) (struct physics_world *, int)) ftc_sym ("ftc_eject");
        if (!p_ej || !w) {
            term_err ("mpe: ftc: bundle API incomplete\n");
            return;
        }
        int n = p_ej (w, 0);
        if (n < 0) {
            term_err ("mpe: ftc: no robot (ftc spawn first)\n");
            return;
        }
        char buf [128];
        snprintf (buf, sizeof (buf), "mpe: ftc: ejected %d ball(s)\n", n);
        term_ok (buf);
        return;
    }
    if (term_str_eq (argv [1], "shoot")) {
        int (*p_sh) (struct physics_world *, int) = (int (*) (struct physics_world *, int)) ftc_sym ("ftc_shoot");
        if (!p_sh || !w) {
            term_err ("mpe: ftc: bundle API incomplete\n");
            return;
        }
        int id = p_sh (w, 0);
        if (id < 0) {
            term_err ("mpe: ftc: nothing carried (intake balls first)\n");
            return;
        }
        char buf [128];
        snprintf (buf, sizeof (buf), "mpe: ftc: fired ball id=%d\n", id);
        term_ok (buf);
        return;
    }
    if (term_str_eq (argv [1], "balls")) {
        int (*p_b) (struct physics_world *, int) = (int (*) (struct physics_world *, int)) ftc_sym ("ftc_balls_spawn");
        if (!p_b || !w) {
            term_err ("mpe: ftc: bundle API incomplete\n");
            return;
        }
        int n = (argc > 2) ? (int) ftc_argf (argv, 2, argc, 3.0f) : 3;
        int made = p_b (w, n);
        if (made < 0) {
            term_err ("mpe: ftc: balls failed (no fleet: eco attach mfs-simulator first)\n");
            return;
        }
        char buf [128];
        snprintf (buf, sizeof (buf), "mpe: ftc: %d ball(s) on the field\n", made);
        term_ok (buf);
        return;
    }
    if (term_str_eq (argv [1], "pieces")) {
        int (*p_piece) (struct physics_world *, int, int) =
            (int (*) (struct physics_world *, int, int)) ftc_sym ("ftc_biobuzz_spawn");
        if (!p_piece || !w) {
            term_err ("mpe: ftc: bundle API incomplete\n");
            return;
        }
        int nectar_n = (argc > 2) ? (int) ftc_argf (argv, 2, argc, 4.0f) : 4;
        int pollen_n = (argc > 3) ? (int) ftc_argf (argv, 3, argc, 4.0f) : 4;
        int made = p_piece (w, nectar_n, pollen_n);
        if (made < 0) {
            term_err ("mpe: ftc: pieces failed (no fleet: eco attach mfs-simulator first)\n");
            return;
        }
        char buf [160];
        snprintf (buf, sizeof (buf), "mpe: ftc: staged %d BIOBUZZ piece(s) (nectar 9.1cm, pollen 7.1cm)\n", made);
        term_ok (buf);
        return;
    }
    if (term_str_eq (argv [1], "bucket")) {
        int (*p_bk) (struct physics_world *, float, float, float) =
            (int (*) (struct physics_world *, float, float, float)) ftc_sym ("ftc_bucket_spawn");
        if (!p_bk || !w) {
            term_err ("mpe: ftc: bundle API incomplete\n");
            return;
        }
        float x = ftc_argf (argv, 2, argc, 0.0f);
        float y = ftc_argf (argv, 3, argc, 1.0f);
        float z = ftc_argf (argv, 4, argc, 4.0f);
        if (p_bk (w, x, y, z) != 0) {
            term_err ("mpe: ftc: bucket failed (no fleet: eco attach mfs-simulator first)\n");
            return;
        }
        char buf [160];
        snprintf (buf, sizeof (buf), "mpe: ftc: bucket at (%.2f,%.2f,%.2f) — shoot into it\n", x, y, z);
        term_ok (buf);
        return;
    }
    if (term_str_eq (argv [1], "score")) {
        int (*p_stats) (struct physics_world *, int *, int *, int *, int *) =
            (int (*) (struct physics_world *, int *, int *, int *, int *)) ftc_sym ("ftc_game_stats");
        if (!p_stats || !w) {
            term_err ("mpe: ftc: bundle API incomplete\n");
            return;
        }
        int carried = 0, balls = 0, scored = 0, bucket = 0;
        if (p_stats (w, &carried, &balls, &scored, &bucket) != 0) {
            term_err ("mpe: ftc: no fleet (eco attach mfs-simulator first)\n");
            return;
        }
        char buf [160];
        snprintf (buf, sizeof (buf), "mpe: ftc: carried=%d balls=%d SCORED=%d bucket=%s\n", carried, balls, scored,
                  bucket ? "up" : "--");
        term_out (buf);
        return;
    }
    if (term_str_eq (argv [1], "view")) {
        int mode = (argc > 2) ? (int) ftc_argf (argv, 2, argc, 1.0f) : 1;
        if (mode < 0)
            mode = 0;
        if (mode > 2)
            mode = 2;
        ftc_view_set (mode);
        char buf [160];
        const char *nm = (mode == 0) ? "free" : (mode == 1) ? "field-centric (follows robot, fixed field yaw)" : "robot-centric (chase behind robot)";
        snprintf (buf, sizeof (buf), "mpe: ftc: view %d = %s\n", mode, nm);
        term_ok (buf);
        return;
    }
    term_err ("mpe: ftc: unknown subcommand (spawn|telemetry|intake|eject|shoot|balls|pieces|bucket|score|view)\n");
}

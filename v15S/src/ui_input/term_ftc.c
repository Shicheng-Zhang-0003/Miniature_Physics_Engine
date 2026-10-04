/* `ftc` terminal command — spawn and inspect the single robot.
 *
 * Streamlined 2026-10-04 (v15S release blocker): one field, one robot, one
 * controller, full mecanum drive — and the controller is the physical F310
 * gamepad (see ui_input/gamepad_drive.c), NOT the terminal. Exactly two
 * verbs remain:
 *   ftc spawn [x y z]          spawn THE mecanum robot (refuses when one exists)
 *   ftc telemetry               pose, odometry, battery, wheels
 * Requires the bundle: `mod load ecosystem/mfs/mfs_ecosystem.so` first
 * (symbols resolve from the loaded handle; the engine never links FTC).
 * Spawn auto-attaches the ftc-fleet module to the primary world and ensures
 * the tile field.
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
#define FTC_HANDLE_MODULE "plugins/mpe_ftc.so"

static void *ftc_sym(const char *sym) {
    void *p = mpe_loader_symbol(FTC_HANDLE_PRIMARY, sym);
    if (p) return p;
    return mpe_loader_symbol(FTC_HANDLE_MODULE, sym);
}

static float ftc_argf(char **argv, int i, int argc, float dflt) {
    if (i < argc && argv[i]) {
        char *end = NULL;
        double v = strtod(argv[i], &end);
        if (end != argv[i] && isfinite(v)) return (float)v;
    }
    return dflt;
}

/* Tile floor guarantee: robots need frictional contact (the frictionless
 * emergency backstop yields slip-regime artifacts). Adds a 60x60 tile
 * slab (top y=0, mu 1.0/0.8) only when no static floor-like body already
 * covers the origin. Reported, never silent.
 * DESPOT-2026-10-04b: was 20x20 — a robot driving ~1 m/s reaches the edge
 * in ~10 s and drops a wheel off it (matches a live "wheel snapped"
 * report timeline exactly). 60x60 matches the proven F10 validation slab
 * and stays inside the broadphase cell-span budget. */
static int ftc_ensure_floor(physics_world *w) {
    if (!w) return -1;
    for (int i = 0; i < w->body_count; i++) {
        rigidbody *b = &w->bodies[i];
        if (!b->static_state && !(b->mass == 0.0f)) continue;
        float top = b->position.y + b->half_extensions.y;
        if (top > -0.05f && top < 0.05f && fabsf(b->position.x) < 5.0f &&
            fabsf(b->position.z) < 5.0f && b->half_extensions.x >= 5.0f &&
            b->half_extensions.z >= 5.0f) {
            return 0;
        }
    }
    int f = physics_world_add_cube(w, (vector3){0.0f, -0.5f, 0.0f},
                                   (vector3){30.0f, 0.5f, 30.0f}, 0.0f);
    if (f < 0) return -1;
    w->bodies[f].friction_static = 1.0f;
    w->bodies[f].friction_kinetic = 0.8f;
    w->bodies[f].restitution = 0.0f;
    term_out("mpe: ftc: tile floor added (robots need frictional contact)\n");
    return 1;
}

/* Ensure the ftc-fleet tick module drives the primary world (idempotent).
 * Returns 0 when driving, -1 when the bundle is missing. Engine APIs
 * (registry, attach) are called directly — this TU links into the
 * engine; only FTC APIs resolve through the bundle handle. */
static int ftc_ensure_driving(void) {
    const mpe_module_desc_t *dmod = mpe_find_module("ftc-fleet");
    if (!dmod) {
        /* Bundle path: the inner FTC descriptor is still dlsym-able. */
        dmod = (const mpe_module_desc_t *)ftc_sym("mpe_module_desc");
    }
    if (!dmod) {
        term_err("mpe: ftc: bundle not loaded (from v15S/src: mod load ecosystem/mfs/mfs_ecosystem.so, or mod load plugins/mpe_ftc.so)\n");
        return -1;
    }
    physics_world *w = physics_world_get_primary();
    if (!w) {
        term_err("mpe: ftc: no primary world\n");
        return -1;
    }
    for (int i = 0; i < w->tick_module_count; i++) {
        if (w->tick_modules[i] == dmod) return 0;
    }
    if (physics_world_attach_module(w, dmod) < 0) {
        term_err("mpe: ftc: attach failed\n");
        return -1;
    }
    return 0;
}

/* Iteration guarantee: the 40:1 chassis/wheel stacked mass ratio cannot
 * converge below 128 sequential-impulse iterations (every MFS test pins
 * 128; the MFS suite documents that default 64 cannot converge it). A
 * robot spawned into a 64-iteration world wobbles its revolute axles
 * loose within seconds (measured 15° tilt) and never settles. Raise with
 * a loud report, never silently; already-adequate configs are untouched. */
static void ftc_ensure_iterations(void) {
    if (g_cfg.timestep.solver_iterations < 128) {
        g_cfg.timestep.solver_iterations = 128;
        term_out("mpe: ftc: solver iterations raised to 128 (robot joints need it; was lower)\n");
    }
}

void cmd_ftc(int argc, char **argv) {
    if (argc < 2) {
        term_err("mpe: ftc: usage: ftc spawn [x y z] | telemetry\n");
        return;
    }
    physics_world *w = physics_world_get_primary();

    /* Single-robot accessors (index 0 implied; bundle API keeps indices). */
    ftc_robot *(*p_get)(struct physics_world *, int) =
        (ftc_robot * (*)(struct physics_world *, int)) ftc_sym("ftc_fleet_get");
    int (*p_count)(struct physics_world *) =
        (int (*)(struct physics_world *))ftc_sym("ftc_fleet_count");

    if (term_str_eq(argv[1], "spawn")) {
        if (ftc_ensure_driving() != 0) return;
        int (*p_spawn)(struct physics_world *, float, float, float, motor_preset_id,
                       ftc_drivetrain_type) =
            (int (*)(struct physics_world *, float, float, float, motor_preset_id,
                     ftc_drivetrain_type))ftc_sym("ftc_fleet_spawn");
        float (*p_rest)(void) = (float (*)(void))ftc_sym("ftc_robot_rest_height");
        const char *(*p_name)(motor_preset_id) =
            (const char *(*)(motor_preset_id))ftc_sym("motor_preset_name");
        if (!p_spawn || !p_rest || !p_name || !p_count || !w) {
            term_err("mpe: ftc: bundle API incomplete\n");
            return;
        }
        /* One robot: refuse a second instead of striding a fleet. Fleet
         * siblings stay available through the bundle/API surface. */
        if (p_count(w) > 0) {
            term_err("mpe: ftc: one robot already (scene_clear to reset the field)\n");
            return;
        }
        /* Default: first preset matching 5203 26.9 (223 RPM workhorse). */
        int preset = 0;
        for (int id = 0; id < 512; id++) {
            const char *nm = p_name((motor_preset_id)id);
            if (!nm || strcmp(nm, "unknown") == 0) break;
            if (strstr(nm, "5203") && strstr(nm, "26.9")) {
                preset = id;
                break;
            }
        }
        float x = ftc_argf(argv, 2, argc, 0.0f);
        float y = ftc_argf(argv, 3, argc, p_rest());
        float z = ftc_argf(argv, 4, argc, 0.0f);
        if (ftc_ensure_floor(w) < 0) {
            term_err("mpe: ftc: could not ensure floor\n");
            return;
        }
        ftc_ensure_iterations();
        int idx = p_spawn(w, x, y, z, (motor_preset_id)preset, FTC_DRIVETRAIN_MECANUM);
        if (idx < 0) {
            term_err("mpe: ftc: spawn failed\n");
            return;
        }
        char buf[224];
        snprintf(buf, sizeof(buf), "mpe: ftc: robot at (%.2f,%.2f,%.2f) %s mecanum\n", x, y, z,
                 p_name((motor_preset_id)preset));
        term_ok(buf);
        return;
    }

    if (term_str_eq(argv[1], "telemetry")) {
        if (!p_get) {
            term_err("mpe: ftc: bundle not loaded\n");
            return;
        }
        float (*p_volt)(const battery *, float) =
            (float (*)(const battery *, float))ftc_sym("battery_get_voltage");
        int (*p_fuse)(const battery *) =
            (int (*)(const battery *))ftc_sym("battery_fuse_tripped");
        ftc_robot *r = p_get(w, 0);
        if (!r) {
            term_err("mpe: ftc: no robot (ftc spawn first)\n");
            return;
        }
        char buf[256];
        float px = 0, py = 0, pz = 0;
        if (r->chassis_body >= 0 && r->chassis_body < w->body_count) {
            px = w->bodies[r->chassis_body].position.x;
            py = w->bodies[r->chassis_body].position.y;
            pz = w->bodies[r->chassis_body].position.z;
        }
        snprintf(buf, sizeof(buf), "mpe: ftc: robot pos=(%.3f,%.3f,%.3f) odom=(%.3f,%.3f,%.3f)%s\n",
                 px, py, pz, r->odom_x, r->odom_z, r->odom_theta,
                 r->odom_slip ? " SLIP" : "");
        term_out(buf);
        if (p_volt && p_fuse) {
            float isum = 0.0f;
            for (int k = 0; k < r->wheel_count; k++) isum += r->wheel_motors[k].current;
            snprintf(buf, sizeof(buf), "mpe: ftc: battery %.2fV (%.0f%%)%s\n",
                     p_volt(&r->battery, isum), r->battery.charge_fraction * 100.0f,
                     p_fuse(&r->battery) ? " FUSE-TRIPPED" : "");
            term_out(buf);
        }
        for (int k = 0; k < r->wheel_count; k++) {
            snprintf(buf, sizeof(buf), "mpe: ftc: wheel %d cmd=%+.2f rpm=%+.0f I=%+.2fA\n", k,
                     r->wheel_motors[k].command, r->wheel_motors[k].rpm,
                     r->wheel_motors[k].current);
            term_out(buf);
        }
        return;
    }

    term_err("mpe: ftc: unknown subcommand (spawn|telemetry)\n");
}

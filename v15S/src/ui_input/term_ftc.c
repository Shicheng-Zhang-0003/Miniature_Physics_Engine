/* `ftc` terminal command — drive/configure FTC robots live.
 * Usage:
 *   ftc spawn [preset-substr] [mecanum|tank] [x y z]
 *   ftc list
 *   ftc drive <i> tank <l> <r> | mecanum <f> <s> <r> | stop
 *   ftc telemetry [i]
 *   ftc preset [substr]
 * Requires the bundle: `mod load ecosystem/mfs/mfs_ecosystem.so` first
 * (symbols resolve from the loaded handle; the engine never links FTC).
 * Spawn auto-attaches the ftc-fleet module to the primary world.
 * Drive commands persist until changed (motors hold their command).
 */
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
 * emergency backstop yields slip-regime artifacts). Adds a 20x20 tile
 * slab (top y=0, mu 1.0/0.8) only when no static floor-like body already
 * covers the origin. Reported, never silent. */
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
                                   (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
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
        term_err("mpe: ftc: bundle not loaded (mod load ecosystem/mfs/mfs_ecosystem.so)\n");
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

void cmd_ftc(int argc, char **argv) {
    if (argc < 2) {
        term_err("mpe: ftc: usage: ftc spawn|list|drive|telemetry|preset\n");
        return;
    }
    physics_world *w = physics_world_get_primary();

    if (term_str_eq(argv[1], "preset")) {
        const char *(*p_name)(motor_preset_id) =
            (const char *(*)(motor_preset_id))ftc_sym("motor_preset_name");
        if (!p_name) {
            term_err("mpe: ftc: bundle not loaded\n");
            return;
        }
        char buf[160];
        const char *flt = (argc >= 3) ? argv[2] : "";
        for (int id = 0; id < 512; id++) {
            const char *nm = p_name((motor_preset_id)id);
            if (!nm || strcmp(nm, "unknown") == 0) break;
            if (!flt[0] || strstr(nm, flt)) {
                snprintf(buf, sizeof(buf), "  [%d] %s\n", id, nm);
                term_out(buf);
            }
        }
        return;
    }

    if (term_str_eq(argv[1], "spawn")) {
        if (ftc_ensure_driving() != 0) return;
        int (*p_spawn)(struct physics_world *, float, float, float, motor_preset_id,
                       ftc_drivetrain_type) =
            (int (*)(struct physics_world *, float, float, float, motor_preset_id,
                     ftc_drivetrain_type))ftc_sym("ftc_fleet_spawn");
        float (*p_rest)(void) = (float (*)(void))ftc_sym("ftc_robot_rest_height");
        const char *(*p_name)(motor_preset_id) =
            (const char *(*)(motor_preset_id))ftc_sym("motor_preset_name");
        if (!p_spawn || !p_rest || !p_name) {
            term_err("mpe: ftc: bundle API incomplete\n");
            return;
        }
        int preset = -1, ai = 2;
        ftc_drivetrain_type dtype = FTC_DRIVETRAIN_MECANUM;
        if (argc >= 3 && argv[2]) {
            if (strcmp(argv[2], "tank") == 0) {
                dtype = FTC_DRIVETRAIN_TANK;
                ai = 3;
            } else if (strcmp(argv[2], "mecanum") == 0) {
                ai = 3;
            } else {
                for (int id = 0; id < 512; id++) {
                    const char *nm = p_name((motor_preset_id)id);
                    if (!nm || strcmp(nm, "unknown") == 0) break;
                    if (strstr(nm, argv[2])) {
                        preset = id;
                        break;
                    }
                }
                if (preset < 0) {
                    char b[192];
                    snprintf(b, sizeof(b), "mpe: ftc: unknown preset '%s'\n", argv[2]);
                    term_err(b);
                    return;
                }
                if (ai < argc && argv[ai]) {
                    if (strcmp(argv[ai], "tank") == 0) dtype = FTC_DRIVETRAIN_TANK;
                    else if (strcmp(argv[ai], "mecanum") != 0) {
                        term_err("mpe: ftc: drive type must be mecanum|tank\n");
                        return;
                    }
                    ai++;
                }
            }
        }
        if (preset < 0) {
            /* Default: first preset matching 5203 26.9 (223 RPM workhorse). */
            for (int id = 0; id < 512; id++) {
                const char *nm = p_name((motor_preset_id)id);
                if (!nm || strcmp(nm, "unknown") == 0) break;
                if (strstr(nm, "5203") && strstr(nm, "26.9")) {
                    preset = id;
                    break;
                }
            }
            if (preset < 0) preset = 0;
        }
        float x = ftc_argf(argv, ai, argc, 0.0f);
        float y = ftc_argf(argv, ai + 1, argc, p_rest());
        float z = ftc_argf(argv, ai + 2, argc, 0.0f);
        if (ftc_ensure_floor(w) < 0) {
            term_err("mpe: ftc: could not ensure floor\n");
            return;
        }
        int idx = p_spawn(w, x, y, z, (motor_preset_id)preset, dtype);
        if (idx < 0) {
            term_err("mpe: ftc: spawn failed\n");
            return;
        }
        char buf[224];
        snprintf(buf, sizeof(buf), "mpe: ftc: robot %d at (%.2f,%.2f,%.2f) %s %s\n", idx, x, y, z,
                 p_name((motor_preset_id)preset),
                 dtype == FTC_DRIVETRAIN_MECANUM ? "mecanum" : "tank");
        term_ok(buf);
        return;
    }

    if (term_str_eq(argv[1], "list")) {
        int (*p_count)(struct physics_world *) =
            (int (*)(struct physics_world *))ftc_sym("ftc_fleet_count");
        ftc_robot *(*p_get)(struct physics_world *, int) =
            (ftc_robot * (*)(struct physics_world *, int)) ftc_sym("ftc_fleet_get");
        if (!p_count || !p_get) {
            term_err("mpe: ftc: bundle not loaded\n");
            return;
        }
        int n = p_count(w);
        char buf[224];
        snprintf(buf, sizeof(buf), "mpe: ftc: %d robot(s)\n", n);
        term_out(buf);
        for (int i = 0; i < n; i++) {
            ftc_robot *r = p_get(w, i);
            if (!r) continue;
            float px = 0, py = 0, pz = 0;
            if (r->chassis_body >= 0 && r->chassis_body < w->body_count) {
                px = w->bodies[r->chassis_body].position.x;
                py = w->bodies[r->chassis_body].position.y;
                pz = w->bodies[r->chassis_body].position.z;
            }
            snprintf(buf, sizeof(buf), "  [%d] %s at (%.2f,%.2f,%.2f) odom=(%.2f,%.2f,%.2f)%s\n",
                     i, r->drivetrain_type == FTC_DRIVETRAIN_MECANUM ? "mecanum" : "tank", px, py,
                     pz, r->odom_x, r->odom_z, r->odom_theta, r->odom_slip ? " SLIP" : "");
            term_out(buf);
        }
        return;
    }

    if (term_str_eq(argv[1], "drive")) {
        if (argc < 4) {
            term_err("mpe: ftc: usage: ftc drive <i> tank <l> <r> | mecanum <f> <s> <r> | stop\n");
            return;
        }
        if (ftc_ensure_driving() != 0) return;
        ftc_robot *(*p_get)(struct physics_world *, int) =
            (ftc_robot * (*)(struct physics_world *, int)) ftc_sym("ftc_fleet_get");
        if (!p_get) {
            term_err("mpe: ftc: bundle not loaded\n");
            return;
        }
        int idx = (int)ftc_argf(argv, 2, argc, -1.0f);
        ftc_robot *r = p_get(w, idx);
        if (!r) {
            char b[96];
            snprintf(b, sizeof(b), "mpe: ftc: no robot %d\n", idx);
            term_err(b);
            return;
        }
        if (term_str_eq(argv[3], "stop")) {
            void (*p_tank)(ftc_robot *, float, float) =
                (void (*)(ftc_robot *, float, float))ftc_sym("drivetrain_tank");
            if (p_tank) p_tank(r, 0.0f, 0.0f);
            term_ok("mpe: ftc: stopped\n");
            return;
        }
        if (term_str_eq(argv[3], "tank") && argc >= 6) {
            void (*p_tank)(ftc_robot *, float, float) =
                (void (*)(ftc_robot *, float, float))ftc_sym("drivetrain_tank");
            if (!p_tank) {
                term_err("mpe: ftc: bundle API incomplete\n");
                return;
            }
            p_tank(r, ftc_argf(argv, 4, argc, 0), ftc_argf(argv, 5, argc, 0));
            term_ok("mpe: ftc: driving\n");
            return;
        }
        if (term_str_eq(argv[3], "mecanum") && argc >= 7) {
            void (*p_mec)(ftc_robot *, float, float, float) =
                (void (*)(ftc_robot *, float, float, float))ftc_sym("drivetrain_mecanum");
            if (!p_mec) {
                term_err("mpe: ftc: bundle API incomplete\n");
                return;
            }
            p_mec(r, ftc_argf(argv, 4, argc, 0), ftc_argf(argv, 5, argc, 0),
                  ftc_argf(argv, 6, argc, 0));
            term_ok("mpe: ftc: driving\n");
            return;
        }
        term_err("mpe: ftc: usage: ftc drive <i> tank <l> <r> | mecanum <f> <s> <r> | stop\n");
        return;
    }

    if (term_str_eq(argv[1], "telemetry")) {
        ftc_robot *(*p_get)(struct physics_world *, int) =
            (ftc_robot * (*)(struct physics_world *, int)) ftc_sym("ftc_fleet_get");
        float (*p_volt)(const battery *, float) =
            (float (*)(const battery *, float))ftc_sym("battery_get_voltage");
        int (*p_fuse)(const battery *) =
            (int (*)(const battery *))ftc_sym("battery_fuse_tripped");
        if (!p_get) {
            term_err("mpe: ftc: bundle not loaded\n");
            return;
        }
        int idx = (argc >= 3) ? (int)ftc_argf(argv, 2, argc, 0.0f) : 0;
        ftc_robot *r = p_get(w, idx);
        if (!r) {
            char b[96];
            snprintf(b, sizeof(b), "mpe: ftc: no robot %d\n", idx);
            term_err(b);
            return;
        }
        char buf[256];
        float px = 0, py = 0, pz = 0;
        if (r->chassis_body >= 0 && r->chassis_body < w->body_count) {
            px = w->bodies[r->chassis_body].position.x;
            py = w->bodies[r->chassis_body].position.y;
            pz = w->bodies[r->chassis_body].position.z;
        }
        snprintf(buf, sizeof(buf), "mpe: ftc: robot %d pos=(%.3f,%.3f,%.3f) odom=(%.3f,%.3f,%.3f)%s\n",
                 idx, px, py, pz, r->odom_x, r->odom_z, r->odom_theta,
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

    term_err("mpe: ftc: unknown subcommand (spawn|list|drive|telemetry|preset)\n");
}

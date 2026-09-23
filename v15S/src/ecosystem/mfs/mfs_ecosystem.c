/* MFS Ecosystem — "mfs-simulator" overarching descriptor.
 *
 * The single home for everything MFS-wise: it bundles every module the
 * MFS tree ships as internal modules sharing one per-world state slot.
 * Modules live in modules/<name>/, their support libs in
 * modules/<name>/submodules/ (see README_MFS.md for the map).
 *
 * Architecture:
 * - MFS Ecosystem (this file) manages internal modules
 * - modules/module_1 (mfs_module_1.c) implements the BioBuzz simulation
 * - modules/ftc (ftc_module.c) implements the ftc-fleet tick module,
 *   built on its submodules/ robot stack
 * - Internal module system (mfs_internal.c) manages module lifecycle
 */

#include "ecosystem/mpe_ecosystem.h"
#include "mfs_internal.h"
#include "modules/module_1/mfs_module_1.h"
#include "modules/ftc/ftc_fleet.h"
#include "modules/ftc/submodules/drivetrain.h"
#include "modules/ftc/submodules/motor_presets.h"
#include "modules/ftc/submodules/battery.h"
#include "core/physics_world.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ================================================================
 * Ecosystem State
 * ================================================================ */

#define MFS_ECO_MAX_WORLDS 8

typedef struct {
    bool modules_initialized;
    bool modules_attached;
    /* Worlds this bundle instance is attached to (terminal commands
     * operate on worlds[0]; the fleet itself is per-world). */
    mpe_world_t *worlds[MFS_ECO_MAX_WORLDS];
    int nworlds;
} mfs_ecosystem_state_t;

static void eco_track_world(mfs_ecosystem_state_t *state, mpe_world_t *world) {
    if (!state || !world) return;
    for (int i = 0; i < state->nworlds; i++) {
        if (state->worlds[i] == world) return;
    }
    if (state->nworlds < MFS_ECO_MAX_WORLDS) {
        state->worlds[state->nworlds++] = world;
    }
}

static void eco_untrack_world(mfs_ecosystem_state_t *state, mpe_world_t *world) {
    if (!state) return;
    for (int i = 0; i < state->nworlds; i++) {
        if (state->worlds[i] == world || !world) {
            for (int j = i; j + 1 < state->nworlds; j++) {
                state->worlds[j] = state->worlds[j + 1];
            }
            state->nworlds--;
            if (world) break;
        }
    }
}

static mpe_world_t *eco_primary_world(mfs_ecosystem_state_t *state) {
    if (!state || state->nworlds <= 0 || !state->worlds[0]) return NULL;
    return state->worlds[0];
}

/* ================================================================
 * Ecosystem Lifecycle
 * ================================================================ */

static int mfs_ecosystem_attach(mpe_world_t *world, void **eco_state) {
    if (!world || !eco_state) return -1;

    mfs_ecosystem_state_t *state = calloc(1, sizeof(mfs_ecosystem_state_t));
    if (!state) return -1;

    /* Initialize internal module registry once only */
    static int s_registry_initialized = 0;
    if (!s_registry_initialized) {
        mfs_internal_registry_init();
        s_registry_initialized = 1;
    }
    /* Ensure-registered (idempotent): repeat attaches to other worlds
     * must not fail on duplicate registration. */
    extern const mpe_module_desc_t mfs_module_1_desc;
    if (!mfs_internal_module_registered(MFS_MODULE_1_NAME) &&
        mfs_internal_module_register(&mfs_module_1_desc) < 0) {
        free(state);
        return -1;
    }
    /* NOTE: ftc_module.c exports the loader-reserved symbol
     * mpe_module_desc (dlsym'd by `mod load` for mpe_ftc.so); it IS the
     * ftc-fleet descriptor, so the static bundle references it directly
     * instead of duplicating the struct. */
    extern const mpe_module_desc_t mpe_module_desc;
    if (!mfs_internal_module_registered("ftc-fleet") &&
        mfs_internal_module_register(&mpe_module_desc) < 0) {
        free(state);
        return -1;
    }

    /* Attach internal modules */
    if (mfs_internal_module_attach("mfs-simulator", (mpe_world_t*)world) < 0) {
        free(state);
        return -1;
    }
    if (mfs_internal_module_attach("ftc-fleet", (mpe_world_t*)world) < 0) {
        free(state);
        return -1;
    }
    
    state->modules_initialized = true;
    state->modules_attached = true;
    eco_track_world(state, world);

    *eco_state = state;
    return 0;
}

static void mfs_ecosystem_detach(mpe_world_t *world, void *eco_state) {
    if (!eco_state) return;

    mfs_ecosystem_state_t *state = (mfs_ecosystem_state_t *)eco_state;

    /* Detach all internal modules */
    mfs_internal_modules_detach_all((mpe_world_t*)world);
    eco_untrack_world(state, world);

    free(state);
}

/* ================================================================
 * Ecosystem Step Hooks
 * ================================================================ */

static void mfs_ecosystem_pre_step(mpe_world_t *world, float dt, void *eco_state) {
    (void)eco_state;
    /* Run pre_step for all attached internal modules */
    mfs_internal_modules_pre_step((mpe_world_t*)world, dt);
}

static void mfs_ecosystem_post_step(mpe_world_t *world, float dt, void *eco_state) {
    (void)eco_state;
    mfs_internal_modules_post_step((mpe_world_t*)world, dt);
}

/* ================================================================
 * Configuration Interface
 * ================================================================ */

static int mfs_ecosystem_config_get(void *eco_state, const char *key, char *out, int maxlen) {
    (void)eco_state;
    if (!key || !out || maxlen <= 0) return -1;
    if (strcmp(key, "shooter_rpm") == 0) {
        mfs_module_1_state *ms =
            (mfs_module_1_state *)mfs_internal_module_state(MFS_MODULE_1_NAME);
        if (!ms) return -1;
        snprintf(out, (size_t)maxlen, "%.1f", (double)ms->shooter_rpm);
        return 0;
    }
    return -1; /* unsupported key (honest: no silent default) */
}

static int mfs_ecosystem_config_set(void *eco_state, const char *key, const char *value) {
    (void)eco_state;
    if (!key || !value) return -1;
    /* No settable keys exist yet (shooter target goes through the
     * module_1 gamepad/shooter API, not the bundle). -1 = unsupported. */
    (void)key;
    (void)value;
    return -1;
}

/* Preset lookup by substring ("5203-26.9" or "26.9"); -1 when unknown. */
static int eco_preset_by_name(const char *want) {
    if (!want || !*want) return -1;
    for (int id = 0; id < MOTOR_COUNT; id++) {
        const char *nm = motor_preset_name((motor_preset_id)id);
        if (nm && strstr(nm, want)) return id;
    }
    return -1;
}

/* Tile floor guarantee (same contract as the terminal spawn path:
 * robots need frictional contact; reported, never silent). */
static int eco_ensure_floor(physics_world *w) {
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
    printf("mfs: tile floor added (robots need frictional contact)\n");
    return 1;
}

static float eco_argf(char **argv, int i, int argc, float dflt) {
    if (i < argc && argv[i]) {
        char *end = NULL;
        double v = strtod(argv[i], &end);
        if (end != argv[i] && isfinite(v)) return (float)v;
    }
    return dflt;
}

/* Bundle command surface (drives the terminal `eco command` path):
 *   help
 *   spawn [preset-substr] [mecanum|tank] [x y z]
 *   drive <i> tank <l> <r> | mecanum <f> <s> <r> | stop
 *   list
 *   telemetry [i]
 * Operates on the first attached world. Prints human-readable output;
 * returns 0 on success, -1 on usage/lookup failure. */
static int mfs_ecosystem_command(void *eco_state, int argc, char **argv) {
    mfs_ecosystem_state_t *state = (mfs_ecosystem_state_t *)eco_state;
    if (!state || argc < 1 || !argv || !argv[0]) return -1;
    mpe_world_t *world = eco_primary_world(state);
    if (!world) {
        printf("mfs: not attached to any world\n");
        return -1;
    }
    physics_world *w = (physics_world *)world;

    if (strcmp(argv[0], "help") == 0) {
        printf("mfs commands: spawn [preset] [mecanum|tank] [x y z] | "
               "drive <i> tank <l> <r> | drive <i> mecanum <f> <s> <r> | "
               "drive <i> stop | list | telemetry [i]\n");
        return 0;
    }
    if (strcmp(argv[0], "spawn") == 0) {
        int preset = (int)MOTOR_GB_5203_26_9;
        ftc_drivetrain_type dtype = FTC_DRIVETRAIN_MECANUM;
        int ai = 1;
        /* argv[1] may be a preset substring OR a drive type. */
        if (ai < argc && argv[ai]) {
            if (strcmp(argv[ai], "tank") == 0) {
                dtype = FTC_DRIVETRAIN_TANK;
                ai++;
            } else if (strcmp(argv[ai], "mecanum") == 0) {
                ai++;
            } else {
                int p = eco_preset_by_name(argv[ai]);
                if (p < 0) {
                    printf("mfs: unknown preset '%s'\n", argv[ai]);
                    return -1;
                }
                preset = p;
                ai++;
                if (ai < argc && argv[ai]) {
                    if (strcmp(argv[ai], "tank") == 0) dtype = FTC_DRIVETRAIN_TANK;
                    else if (strcmp(argv[ai], "mecanum") != 0) {
                        printf("mfs: drive type must be mecanum|tank\n");
                        return -1;
                    }
                    ai++;
                }
            }
        }
        float x = eco_argf(argv, ai, argc, 0.0f);
        float y = eco_argf(argv, ai + 1, argc, ftc_robot_rest_height());
        float z = eco_argf(argv, ai + 2, argc, 0.0f);
        if (eco_ensure_floor(w) < 0) {
            printf("mfs: could not ensure floor\n");
            return -1;
        }
        int idx = ftc_fleet_spawn(w, x, y, z, (motor_preset_id)preset, dtype);
        if (idx < 0) {
            printf("mfs: spawn failed (no fleet attached? attach the bundle first)\n");
            return -1;
        }
        printf("mfs: robot %d spawned at (%.2f,%.2f,%.2f) preset=%s %s\n", idx, x, y, z,
               motor_preset_name((motor_preset_id)preset),
               dtype == FTC_DRIVETRAIN_MECANUM ? "mecanum" : "tank");
        return 0;
    }
    if (strcmp(argv[0], "drive") == 0) {
        if (argc < 3) {
            printf("mfs: usage: drive <i> tank <l> <r> | mecanum <f> <s> <r> | stop\n");
            return -1;
        }
        int idx = (int)eco_argf(argv, 1, argc, -1.0f);
        ftc_robot *r = ftc_fleet_get(w, idx);
        if (!r) {
            printf("mfs: no robot %d\n", idx);
            return -1;
        }
        if (strcmp(argv[2], "stop") == 0) {
            float z[4] = {0, 0, 0, 0};
            ftc_robot_set_wheel_commands(r, z, 4);
            printf("mfs: robot %d stopped\n", idx);
            return 0;
        }
        if (strcmp(argv[2], "tank") == 0) {
            if (argc < 5) {
                printf("mfs: usage: drive <i> tank <l> <r>\n");
                return -1;
            }
            drivetrain_tank(r, eco_argf(argv, 3, argc, 0), eco_argf(argv, 4, argc, 0));
            printf("mfs: robot %d tank l=%.2f r=%.2f\n", idx, eco_argf(argv, 3, argc, 0),
                   eco_argf(argv, 4, argc, 0));
            return 0;
        }
        if (strcmp(argv[2], "mecanum") == 0) {
            if (argc < 6) {
                printf("mfs: usage: drive <i> mecanum <f> <s> <r>\n");
                return -1;
            }
            drivetrain_mecanum(r, eco_argf(argv, 3, argc, 0), eco_argf(argv, 4, argc, 0),
                               eco_argf(argv, 5, argc, 0));
            printf("mfs: robot %d mecanum f=%.2f s=%.2f r=%.2f\n", idx, eco_argf(argv, 3, argc, 0),
                   eco_argf(argv, 4, argc, 0), eco_argf(argv, 5, argc, 0));
            return 0;
        }
        printf("mfs: drive mode must be tank|mecanum|stop\n");
        return -1;
    }
    if (strcmp(argv[0], "list") == 0) {
        int n = ftc_fleet_count(w);
        printf("mfs: %d robot(s)\n", n);
        for (int i = 0; i < n; i++) {
            ftc_robot *r = ftc_fleet_get(w, i);
            if (!r) continue;
            float px = 0, py = 0, pz = 0;
            ftc_robot_get_position(w, r, &px, &py, &pz);
            printf("  [%d] %s at (%.2f,%.2f,%.2f) odom=(%.2f,%.2f,%.2f)%s\n", i,
                   r->drivetrain_type == FTC_DRIVETRAIN_MECANUM ? "mecanum" : "tank", px, py, pz,
                   r->odom_x, r->odom_z, r->odom_theta, r->odom_slip ? " SLIP" : "");
        }
        return 0;
    }
    if (strcmp(argv[0], "telemetry") == 0) {
        int idx = (argc > 1) ? (int)eco_argf(argv, 1, argc, 0.0f) : 0;
        ftc_robot *r = ftc_fleet_get(w, idx);
        if (!r) {
            printf("mfs: no robot %d\n", idx);
            return -1;
        }
        float px = 0, py = 0, pz = 0;
        ftc_robot_get_position(w, r, &px, &py, &pz);
        float isum = 0.0f;
        for (int k = 0; k < r->wheel_count; k++) isum += r->wheel_motors[k].current;
        printf("mfs: robot %d pos=(%.3f,%.3f,%.3f) odom=(%.3f,%.3f,%.3f)%s\n", idx, px, py, pz,
               r->odom_x, r->odom_z, r->odom_theta, r->odom_slip ? " SLIP" : "");
        printf("mfs: battery %.2fV (%.0f%%)%s\n", battery_get_voltage(&r->battery, isum),
               r->battery.charge_fraction * 100.0f,
               battery_fuse_tripped(&r->battery) ? " FUSE-TRIPPED" : "");
        for (int k = 0; k < r->wheel_count; k++) {
            printf("mfs: wheel %d cmd=%+.2f rpm=%+.0f I=%+.2fA\n", k, r->wheel_motors[k].command,
                   r->wheel_motors[k].rpm, r->wheel_motors[k].current);
        }
        return 0;
    }
    printf("mfs: unknown command '%s' (try help)\n", argv[0]);
    return -1;
}

/* ================================================================
 * Ecosystem Descriptor
 * ================================================================ */

__attribute__((used)) const mpe_ecosystem_desc_t mpe_ecosystem_desc = {
    .abi = 1,
    .name = "mfs-simulator",
    .version = "1.0",
    .author = "MFS Team",
    .description = "MFS overarching ecosystem: bundles modules/module_1 (BioBuzz sim) and modules/ftc (ftc-fleet) with their submodules",
    /* PHYSICS-TRUTH: bundles a non-deterministic module (see above). */
    .deterministic = false,
    .attach = mfs_ecosystem_attach,
    .detach = mfs_ecosystem_detach,
    .pre_step = mfs_ecosystem_pre_step,
    .post_step = mfs_ecosystem_post_step,
    .config_get = mfs_ecosystem_config_get,
    .config_set = mfs_ecosystem_config_set,
    .command = mfs_ecosystem_command,
};
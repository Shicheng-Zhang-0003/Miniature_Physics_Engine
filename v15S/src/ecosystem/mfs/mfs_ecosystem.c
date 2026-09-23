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
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ================================================================
 * Ecosystem State
 * ================================================================ */

typedef struct {
    bool modules_initialized;
    bool modules_attached;
} mfs_ecosystem_state_t;

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
    
    *eco_state = state;
    return 0;
}

static void mfs_ecosystem_detach(mpe_world_t *world, void *eco_state) {
    (void)world;
    if (!eco_state) return;
    
    mfs_ecosystem_state_t *state = (mfs_ecosystem_state_t *)eco_state;
    
    /* Detach all internal modules */
    mfs_internal_modules_detach_all((mpe_world_t*)world);
    
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

static int mfs_ecosystem_command(void *eco_state, int argc, char **argv) {
    (void)eco_state;
    if (argc < 0 || !argv) return -1;
    /* No bundle commands exist yet. -1 = unsupported. */
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
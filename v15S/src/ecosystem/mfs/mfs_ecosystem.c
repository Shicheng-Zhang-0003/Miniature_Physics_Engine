/* MFS Ecosystem — BioBuzz Robotics/Field Simulation
 * 
 * Provides a complete robotics/field simulation ecosystem that bundles
 * the MFS Module 1 as an internal module. This ecosystem manages the
 * internal module lifecycle and provides the ecosystem descriptor.
 * 
 * Architecture:
 * - MFS Ecosystem (this file) manages internal modules
 * - MFS Module 1 (mfs_module_1.c) implements the actual simulation logic
 * - Internal module system (mfs_internal.c) manages module lifecycle
 */

#include "../mpe_ecosystem.h"
#include "mfs_internal.h"
#include "mfs_module_1.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

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

static int mfs_ecosystem_attach(struct mpe_world *world, void **eco_state) {
    if (!world || !eco_state) return -1;
    
    mfs_ecosystem_state_t *state = calloc(1, sizeof(mfs_ecosystem_state_t));
    if (!state) return -1;
    
    /* Initialize internal module registry */
    mfs_internal_registry_init();
    
    /* Register internal modules */
    extern const mpe_module_desc_t mfs_module_1_desc;
    if (mfs_internal_module_register(&mfs_module_1_desc) < 0) {
        free(state);
        return -1;
    }
    
    /* Attach internal modules */
    if (mfs_internal_module_attach("mfs-simulator", (struct mpe_world*)world) < 0) {
        free(state);
        return -1;
    }
    
    state->modules_initialized = true;
    state->modules_attached = true;
    
    *eco_state = state;
    return 0;
}

static void mfs_ecosystem_detach(struct mpe_world *world, void *eco_state) {
    (void)world;
    if (!eco_state) return;
    
    mfs_ecosystem_state_t *state = (mfs_ecosystem_state_t *)eco_state;
    
    /* Detach all internal modules */
    mfs_internal_modules_detach_all((struct mpe_world*)world);
    
    free(state);
}

/* ================================================================
 * Ecosystem Step Hooks
 * ================================================================ */

static void mfs_ecosystem_pre_step(struct mpe_world *world, float dt, void *eco_state) {
    (void)eco_state;
    /* Run pre_step for all attached internal modules */
    mfs_internal_modules_pre_step((struct mpe_world*)world, dt);
}

static void mfs_ecosystem_post_step(struct mpe_world *world, float dt, void *eco_state) {
    (void)eco_state;
    mfs_internal_modules_post_step((struct mpe_world*)world, dt);
}

/* ================================================================
 * Configuration Interface
 * ================================================================ */

static int mfs_ecosystem_config_get(void *eco_state, const char *key, char *out, int maxlen) {
    (void)eco_state;
    if (strcmp(key, "shooter_rpm") == 0) {
        /* Would need to get state from module */
        return -1;
    }
    return -1;
}

static int mfs_ecosystem_config_set(void *eco_state, const char *key, const char *value) {
    (void)eco_state;
    return -1;
}

static int mfs_ecosystem_command(void *eco_state, int argc, char **argv) {
    (void)eco_state; (void)argc; (void)argv;
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
    .description = "BioBuzz robotics/field simulation ecosystem with bundled MFS Module 1",
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
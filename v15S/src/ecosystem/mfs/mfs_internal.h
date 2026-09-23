#ifndef mfs_internal_h
#define mfs_internal_h
/* MFS Internal Module System
 * 
 * Provides a way to register and manage internal modules within the ecosystem
 * without requiring dlopen. Modules are compiled directly into the ecosystem.
 */

#include <stdint.h>
#include <stdbool.h>
#include "core/physics_world.h"
#include "core/mpe_module.h"

#define MFS_MAX_INTERNAL_MODULES 8

/* Internal module registry entry */
typedef struct {
    const mpe_module_desc_t *desc;
    void *state;
    bool attached;
} mfs_internal_module_t;

/* Internal module registry */
typedef struct {
    mfs_internal_module_t modules[8];
    int count;
} mfs_module_registry_t;

/* Initialize internal module registry */
void mfs_internal_registry_init(void);

/* Register an internal module (compiled directly into ecosystem) */
int mfs_internal_module_register(const mpe_module_desc_t *desc);

/* 1 if name is registered, else 0 (idempotent ensure-register). */
int mfs_internal_module_registered(const char *name);

/* Attach an internal module by name */
int mfs_internal_module_attach(const char *name, physics_world *world);

/* Detach an internal module by name */
int mfs_internal_module_detach(const char *name, physics_world *world);

/* Get internal module state by name */
void *mfs_internal_module_state(const char *name);

/* Run pre_step for all attached internal modules */
void mfs_internal_modules_pre_step(physics_world *world, float dt);

/* Run post_step for all attached internal modules */
void mfs_internal_modules_post_step(physics_world *world, float dt);

/* Detach all internal modules */
void mfs_internal_modules_detach_all(physics_world *world);

#endif
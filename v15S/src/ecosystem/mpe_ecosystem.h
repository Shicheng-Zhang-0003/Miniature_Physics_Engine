#ifndef mpe_ecosystem_h
#define mpe_ecosystem_h
/* MPE Ecosystem Interface (MEI) — for module ecosystems like MFS.
 * 
 * An ecosystem is a self-contained collection of modules, configuration,
 * and runtime state that provides a complete simulation domain
 * (e.g. MFS - robotics/field simulation, BioBuzz - ball shooter game).
 * 
 * Ecosystems are loaded as .so files and can attach/detach to worlds.
 * Multiple ecosystems can coexist in different worlds.
 */

#include <stdbool.h>
#include <stdint.h>
/* NOTE: canonical module ABI lives in core/ (src/module/ holds only a
 * stale divergent copy — do not include from there). */
#include "../core/mpe_module.h"

#define MPE_ECOSYSTEM_ABI 1

struct mpe_ecosystem;
typedef struct mpe_ecosystem mpe_ecosystem_t;

/* Ecosystem descriptor — exported by each ecosystem via mpe_ecosystem_desc symbol */
typedef struct {
    uint32_t abi;                   /* must equal MPE_ECOSYSTEM_ABI */
    const char *name;               /* e.g. "mfs-simulator" */
    const char *version;            /* e.g. "1.0" */
    const char *author;             /* e.g. "MFS Team" */
    const char *description;        /* human-readable description */
    bool deterministic;             /* true if bit-identical across IEEE targets */
    
    /* Ecosystem lifecycle */
    int (*attach)(mpe_world_t *world, void **eco_state);
    void (*detach)(mpe_world_t *world, void *eco_state);
    
    /* Per-tick simulation */
    void (*pre_step)(mpe_world_t *world, float dt, void *eco_state);
    void (*post_step)(mpe_world_t *world, float dt, void *eco_state);
    
    /* Ecosystem configuration (key=value pairs) */
    int (*config_get)(void *eco_state, const char *key, char *out, int maxlen);
    int (*config_set)(void *eco_state, const char *key, const char *value);
    
    /* Ecosystem-specific commands (optional) */
    int (*command)(void *eco_state, int argc, char **argv);
    
} mpe_ecosystem_desc_t;

/* Ecosystem registry API */
int mpe_ecosystem_register(const mpe_ecosystem_desc_t *desc);
int mpe_ecosystem_unregister(const char *name);
const mpe_ecosystem_desc_t *mpe_ecosystem_find(const char *name);
int mpe_ecosystem_count(void);
const mpe_ecosystem_desc_t *mpe_ecosystem_at(int index);
/* Detach everywhere (loader unload path; hooks run pre-dlclose). */
void mpe_ecosystem_detach_everywhere(const char *eco_name);
/* Per-world state for terminal-driven commands (NULL when detached). */
void *mpe_ecosystem_state(mpe_world_t *world, const char *eco_name);

/* Ecosystem management */
int mpe_ecosystem_attach(mpe_world_t *world, const char *eco_name);
int mpe_ecosystem_detach(mpe_world_t *world, const char *eco_name);
void mpe_ecosystem_pre_step(mpe_world_t *world, float dt);
void mpe_ecosystem_post_step(mpe_world_t *world, float dt);

/* Register built-in ecosystems */
void mpe_register_ecosystems(void);

#endif
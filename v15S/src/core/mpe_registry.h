#ifndef mpe_registry_h
#define mpe_registry_h
/* Static + dynamic registry for shapes, broadphase, solver, generic modules.
 * Built-ins self-register at startup (mpe_register_builtins).
 * .so plugins register via mpe_loader (calls same functions).
 * Pair dispatch: (type_a, type_b) with object_custom support.
 *
 * NOTE: the registry tables are intentionally process-global (a type /
 * plugin repository, not simulation state). All mutable SIMULATION state
 * lives in physics_world instances; worlds never share solver data. */
#include "mpe_module.h"
#include <stddef.h>
struct rigidbody;

#define MPE_MAX_PAIR_HANDLERS 64
#define MPE_MAX_MODULES 64

typedef struct {
    int type_a; /* object_type or -1 wildcard; 3 == object_custom */
    int type_b;
    int custom_a; /* custom_shape id or -1 wildcard */
    int custom_b;
    mpe_collide_fn fn;
    char name[64]; /* owned copy: never dangles into .so rodata */
} mpe_pair_entry_t;

void mpe_register_builtins(void);
/* Explicit lock for multi-step loader transactions (snapshot + dlopen +
 * rollback). Dispatch-time find/register calls lock internally; do NOT
 * hold this across a physics step. */
void mpe_registry_lock(void);
void mpe_registry_unlock(void);
int mpe_register_pair_handler(int type_a, int type_b, int custom_a, int custom_b,
                              mpe_collide_fn fn, const char *name);
mpe_collide_fn mpe_find_pair_handler(int type_a, int type_b, int custom_a, int custom_b);
/* Truncation rollback for loader failure paths (never below builtins). */
int mpe_registry_pair_count(void);
void mpe_registry_truncate_pairs(int keep);
int mpe_registry_broadphase_count(void);
void mpe_registry_truncate_broadphase(int keep);
int mpe_registry_solver_count(void);
void mpe_registry_truncate_solvers(int keep);
/* Indexed pair-fn snapshot for loader purge walks. 0 + *out set on success. */
int mpe_registry_pair_fn_at(int idx, mpe_collide_fn *out);
/* Human-readable pair-table row for `mod ls`. 0 on success. */
int mpe_registry_pair_describe(int idx, int *ta, int *tb, int *ca, int *cb, char *name, int namelen);

/* broadphase / solver actives are per-world (world->broadphase_if etc).
 * Global defaults live here.
 * Register returns index on success, -1 on NULL/full/overlong-name, and
 * ALSO -1 when refusing a silent takeover of a builtin ("hash",
 * "seq-impulse", the six builtin pairs). Unregister returns 0/-1 and
 * resets every live world that referenced the entry (no dangling). */
int mpe_register_broadphase(const char *name, const mpe_broadphase_if_t *iface);
int mpe_register_solver(const char *name, const mpe_solver_if_t *iface);
int mpe_unregister_broadphase(const char *name);
int mpe_unregister_solver(const char *name);
int mpe_unregister_pair_handler(mpe_collide_fn fn);
const mpe_broadphase_if_t *mpe_find_broadphase(const char *name);
const mpe_solver_if_t *mpe_find_solver(const char *name);

/* generic tick modules */
int mpe_register_module(const mpe_module_desc_t *desc);
int mpe_unregister_module(const char *name);
/* Provenance for loader/static disambiguation (same name, two images). */
int mpe_registry_set_origin(const char *name, const char *path);
const char *mpe_registry_module_origin(const mpe_module_desc_t *desc);
int mpe_unregister_module_origin(const char *name, const char *path);
int mpe_module_count(void);
const mpe_module_desc_t *mpe_module_at(int i);
const mpe_module_desc_t *mpe_find_module(const char *name);

#endif

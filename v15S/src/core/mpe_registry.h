#ifndef mpe_registry_h
#define mpe_registry_h
/* Static + dynamic registry for shapes, broadphase, solver, generic modules.
 * Built-ins self-register at startup (mpe_register_builtins).
 * .so plugins register via mpe_loader (calls same functions).
 * Pair dispatch: (type_a, type_b) with object_custom support.
 */
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
    const char *name;
} mpe_pair_entry_t;

void mpe_register_builtins(void);
int mpe_register_pair_handler(int type_a, int type_b, int custom_a, int custom_b,
                              mpe_collide_fn fn, const char *name);
mpe_collide_fn mpe_find_pair_handler(int type_a, int type_b, int custom_a, int custom_b);

/* broadphase / solver actives are per-world (world->broadphase_if etc).
 * Global defaults live here. */
void mpe_register_broadphase(const char *name, const mpe_broadphase_if_t *iface);
void mpe_register_solver(const char *name, const mpe_solver_if_t *iface);
const mpe_broadphase_if_t *mpe_find_broadphase(const char *name);
const mpe_solver_if_t *mpe_find_solver(const char *name);

/* generic tick modules */
int mpe_register_module(const mpe_module_desc_t *desc);
int mpe_module_count(void);
const mpe_module_desc_t *mpe_module_at(int i);
const mpe_module_desc_t *mpe_find_module(const char *name);

#endif

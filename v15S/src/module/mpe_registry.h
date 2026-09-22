#ifndef mpe_registry_h
#define mpe_registry_h
/* MPE Module Registry — internal registry for pair handlers, broadphase, solver, and modules */
#include "mpe_module.h"
#include "mpe_module_types.h"
#include <stddef.h>

/* Pair handler entry */
typedef struct {
    int type_a, type_b;
    int custom_a, custom_b;
    mpe_collide_fn fn;
    const char *name;
} mpe_pair_entry_t;

/* Registry API */
int mpe_register_pair_handler(int ta, int tb, int ca, int cb, mpe_collide_fn fn, const char *name);
mpe_collide_fn mpe_find_pair_handler(int ta, int tb, int ca, int cb);

void mpe_register_broadphase(const char *name, const mpe_broadphase_if_t *iface);
void mpe_register_solver(const char *name, const mpe_solver_if_t *iface);
const mpe_broadphase_if_t *mpe_find_broadphase(const char *name);
const mpe_solver_if_t *mpe_find_solver(const char *name);

int mpe_register_module(const mpe_module_desc_t *desc);
int mpe_module_count(void);
const mpe_module_desc_t *mpe_module_at(int index);
const mpe_module_desc_t *mpe_find_module(const char *name);

/* Register built-in pair handlers and stage backends */
void mpe_register_builtins(void);

#endif
#ifndef mpe_module_h
#define mpe_module_h
/* MPE Module Interface (MPI) — Phase 2 modularisation.
 * Stable C ABI for hot plug-and-play physics components.
 * Rules:
 *  - modules never touch g_cfg / g_physics_world / bodies[] directly;
 *    use mpe_world_* accessors + per-world cfg (mpe_world_cfg).
 *  - per-module per-world state via void* (attach stores, detach frees).
 *  - deterministic modules must use det_math.h, never bare sin/cos/pow.
 */
#include <stdbool.h>
#include <stdint.h>
#include "../core/math3d.h"
#include "../core/rigidbody.h"
#include "../physics/broadphase.h"

#define MPE_MODULE_ABI 1

struct physics_world;
typedef struct physics_world mpe_world_t;

typedef struct {
    uint32_t abi;          /* must equal MPE_MODULE_ABI */
    const char *name;      /* e.g. "capsule-shape" */
    const char *version;   /* e.g. "1.0" */
    const char *kind;      /* "shape" | "broadphase" | "solver" | "forcefield" | "constraint" | "generic" */
    bool deterministic;    /* true if bit-identical across IEEE targets */
    int (*attach)(mpe_world_t *world, void **mod_state);
    void (*detach)(mpe_world_t *world, void *mod_state);
    void (*pre_step)(mpe_world_t *world, float dt, void *mod_state);
    void (*post_step)(mpe_world_t *world, float dt, void *mod_state);
} mpe_module_desc_t;

/* Shape pair handler: collide A vs B into manifold_out (collision_data*).
 * Return true on contact (even slop-only friction contact). */
typedef bool (*mpe_collide_fn)(rigidbody *a, rigidbody *b,
                               void *manifold_out, mpe_world_t *world);

/* Broadphase interface: typed pair buffer, world for config/scratch.
 * Return value: pair count (0 = degraded tick). NULL entry = builtin. */
typedef struct {
    int (*generate)(mpe_world_t *world, broadphase_pair *pairs_out, int max_pairs, float dt,
                    void *mod_state);
} mpe_broadphase_if_t;

/* Solver stage interface: every hook optional (NULL = builtin).
 * Hooks receive the owning world (per-world config + scratch) and the
 * module state pointer from attach. Builtin semantics: sequential
 * impulses, Poisson restitution, Catto split, rolling resistance. */
typedef struct {
    float (*resolve)(mpe_world_t *world, void *manifold, float dt, bool friction_only, int iter,
                     void *mod_state);
    void (*poisson)(mpe_world_t *world, void *manifolds, int n, void *mod_state);
    void (*rolling)(mpe_world_t *world, void *manifolds, int n, float dt, void *mod_state);
    void (*split)(mpe_world_t *world, void *manifolds, int n, float dt, void *mod_state);
} mpe_solver_if_t;

#endif

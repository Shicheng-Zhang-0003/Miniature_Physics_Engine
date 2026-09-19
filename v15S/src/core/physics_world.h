/* MPE_FTC_055: Real physics world — pure simulation state. No camera/input/UI. */
#ifndef physics_world_h
#define physics_world_h

#include "rigidbody.h"
#include "../config/mpe_constants.h" /* MFS_131 */
#include "../config/mpe_config.h"
#include "mpe_module.h"
#include "../physics/spring_joint_types.h"
#include "../physics/constraint.h"
#include "../physics/broadphase.h"
#include "../physics/collision_mechanics.h"
#include <stdint.h>

/* MFS_131A: warm-start contact cache entry. Moved here from
 * collision_mechanics.c so physics_world can own a per-world cache
 * (Milestone 3, item 101). */
typedef struct {
    uint32_t object_id_a;
    uint32_t object_id_b;
    vector3 local_position_a;
    vector3 local_position_b;
    float accumulated_normal_impulse;
    float accumulated_tangent_impulse;
    /* NOTE: the second Coulomb-disc tangent is deliberately NOT cached (see
     * prepare): t2 = n×t1 is frame-derived each tick and re-converges in
     * the relaxation sweeps. */
    /* Tangent frame memory: lets a resting contact keep its stick direction
     * across ticks (true Coulomb stick needs direction persistence). */
    vector3 tangent_dir;
    uint32_t property_stamp_a;
    uint32_t property_stamp_b;
    /* Warm-start hash chain link (index into the same cache array, -1 end).
     * Chains are rebuilt every save in array order; lookups walk them
     * instead of linear-scanning the whole cache per contact (was O(n^2)
     * per tick in contact-heavy scenes). */
    int32_t hash_next;
} cached_contact;

typedef struct physics_world {
    rigidbody *bodies;
    int body_count;
    int body_capacity;
    uint32_t next_object_id;
    /* Phase-1 modularisation: per-world config. Defaults to &g_cfg
     * (global singleton, back-compat). Call physics_world_set_config()
     * to bind a foreign/isolated mpe_config_t. Hot path must read
     * via mpe_world_cfg(world), never g_cfg directly, so two worlds
     * can hold different gravity/iters/slop. */
    mpe_config_t *cfg;
    /* MFS_131A: per-world warm-start cache. Heap-allocated in
     * physics_world_init (an inline array would be ~3 MB and would
     * overflow the stack of tests that declare worlds locally).
     * NULL-world callers fall back to the global cache. */
    cached_contact *world_contact_cache;
    int world_contact_cache_count;
    /* Growable pools: bodies start small and double to the compile-time
     * ceilings (mpe_max_bodies / max_cached_contacts), so an empty world
     * costs kilobytes instead of megabytes. Caps are ceilings, not
     * preallocations; overflow still degrades gracefully, never crashes. */
    int world_contact_cache_capacity;
    /* Warm-start hash heads (4096 buckets, -1 empty), heap-allocated with
     * the cache. Rebuilt on every save; see collision_mechanics.c. */
    int32_t *contact_hash_head;
    /* Joint pools (per-world state; migrated from file-scope globals).
     * Spring joints (Hooke point-to-point) and generic constraints
     * (revolute hinges today). Counts track active entries. */
    spring_joint spring_joints[mpe_max_joints];
    int spring_joint_count;
    constraint revolute_constraints[mpe_max_joints];
    int revolute_constraint_count;
    /* Solver scratch (per-world heap; migrated from file-scope statics so
     * worlds never share mutable solver state).
     * pair_buffer/manifolds: broadphase pairs + narrowphase manifolds for
     * the current tick. awake/order: per-manifold island flags + sweep
     * order. sort_keys: manifold height keys. pair_skipped: sleep-wake
     * revisit marks. island_*: union-find labels + wake flags. */
    broadphase_workspace *broadphase;
    broadphase_pair *pair_buffer;
    collision_data *manifolds;
    unsigned char *manifold_awake;
    int *manifold_order;
    float *manifold_sort_keys;
    unsigned char *pair_skipped;
    int *island_parent;
    int *island_label;
    unsigned char *island_awake_flags;
    rigidbody *island_base;
    int island_body_count;
    int island_total;
    /* Contact-cache diagnostics (per-world; was global). */
    int contact_cache_hits;
    int contact_cache_misses;
    /* TRUTH: manifold overflow (pairs dropped when manifolds full) must be
     * visible, not silent. Legacy path counted via debug counter; world path
     * dropped silently and gave false free-flight gravity. */
    int manifold_overflow_count;
    /* TRUTH P0-3: per-body CCD remainder (dt - toi). Allocated mpe_max_bodies
     * floats. CCD pre-clamp consumes toi; post-solve integration must advance
     * only the remainder, else displacement double-counts (toi + dt). */
    float *ccd_time_remaining;
    /* TRUTH P0-1: per-body contact flag for gravity-exactness gating.
     * Verlet +1/2*g*dt^2 applies ONLY to contact-free bodies (free flight =
     * exact parabola); constrained bodies stay pure symplectic Euler (their
     * acceleration is canceled by contact impulses post-solve; correcting
     * with pre-solve gravity pumps them out of slop). Reset each tick.
     * (Wake-on-first-touch novelty intentionally does NOT use a per-body
     * prev array — floor contacts blind it. It probes the warm-start cache
     * per id-pair instead; see contact_cache_has_pair.) */
    unsigned char *has_contact;
    /* Phase-2 modular slots. NULL = built-in default.
     * broadphase_if/solver_if override the corresponding stage;
     * tick_modules[] are generic pre/post-step hooks (forcefields,
     * motors, loggers). States parallel tick_modules[]. */
    const mpe_broadphase_if_t *broadphase_if;
    const mpe_solver_if_t *solver_if;
    const mpe_module_desc_t *tick_modules[16];
    void *tick_module_state[16];
    int tick_module_count;
    /* id->index cache (replaces the old file-static global in
     * constraint.c, which was shared across worlds and threads).
     * Rebuilt lazily when body_revision moves; every lookup verifies
     * against the live array and falls back to linear scan, so a
     * missed bump costs speed, never correctness. */
    uint32_t *id_cache_keys;
    int *id_cache_vals;
    unsigned char *id_cache_valid;
    int id_cache_size;
    uint64_t body_revision;
    uint64_t id_cache_revision;
} physics_world;

void physics_world_init(physics_world *world);
void physics_world_cleanup(physics_world *world);
/* Phase-1: bind per-world config (NULL rebinds global g_cfg).
 * Must be called after physics_world_init; safe any time (takes
 * effect next tick; step snapshots cfg at tick start). */
void physics_world_set_config(physics_world *world, mpe_config_t *cfg);
mpe_config_t *physics_world_get_config(physics_world *world);
/* Hot-path accessor: per-world cfg if bound, else global.
 * Never NULL after mpe_config_init() has run once. */
static inline const mpe_config_t *mpe_world_cfg(const physics_world *world) {
    if (world && world->cfg) return world->cfg;
    return &g_cfg;
}
static inline mpe_config_t *mpe_world_cfg_mut(physics_world *world) {
    if (world && world->cfg) return world->cfg;
    return &g_cfg;
}
int physics_world_add_sphere(physics_world *world, float radius, float mass, vector3 position);
int physics_world_add_cube(physics_world *world, vector3 position, vector3 half_extensions, float mass);
int physics_world_add_cylinder(physics_world *world, float radius, float half_length, float mass, vector3 position); /* MPE_FTC_090 */
int physics_world_add_custom(physics_world *world, int custom_shape, vector3 position, float mass, float radius);
void physics_world_clear(physics_world *world);
void physics_world_step(physics_world *world, float dt);
/* Shared pair pipeline (narrowphase dispatch + 3-gate wake + solver prep).
 * Used by both step paths so GUI and headless ticks stay identical. */
void physics_world_process_pair(physics_world *world, int index_a, int index_b, float dt,
                                int *manifold_count_ptr);
/* Application-owned primary world (defined in core/mpe_primary.c).
 * The kernel holds no simulation globals; this accessor exists for the
 * scene/UI/render layers of a single-world GUI process. Headless and
 * foreign code should own explicit physics_world instances instead. */
physics_world *physics_world_get_primary(void);
/* Phase-2: modular attach/dispatch. */
int physics_world_attach_module(physics_world *world, const mpe_module_desc_t *desc);
int physics_world_detach_module(physics_world *world, const char *name);
void physics_world_set_broadphase(physics_world *world, const mpe_broadphase_if_t *iface);
void physics_world_set_solver(physics_world *world, const mpe_solver_if_t *iface);
/* Pool growth (×2 to ceiling). Used by add_* paths and cache save. */
int physics_world_grow_contact_cache(physics_world *world);
/* Mutation stamp: bumped by every world-owned body-array mutation
 * (add/clear) and by scene-level mutators (remove/load/clear). */
void physics_world_bump_revision(physics_world *world);
/* O(1) id->index via the world cache (linear fallback, always correct).
 * Returns -1 for id 0 / missing. */
int physics_world_index_by_id(physics_world *world, uint32_t id);
rigidbody *physics_world_body_by_id(physics_world *world, uint32_t id);
/* Shape dispatch: registry-first, built-in fallback. Returns true on contact. */
bool mpe_shape_dispatch(physics_world *world, rigidbody *a, rigidbody *b, collision_data *out);
/* R3-07: Add four static wall bodies around the playable area.
 * half_width and half_depth define the playable half-extents.
 * wall_height and wall_thickness define the wall geometry.
 * Returns 0 on success, -1 on failure. */
int physics_world_add_boundary_walls(physics_world *world,
                                     float half_width,
                                     float half_depth,
                                     float wall_height,
                                     float wall_thickness);
#endif

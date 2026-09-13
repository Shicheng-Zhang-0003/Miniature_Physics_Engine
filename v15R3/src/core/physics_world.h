/* MPE_FTC_055: Real physics world — pure simulation state. No camera/input/UI. */
#ifndef physics_world_h
#define physics_world_h

#include "rigidbody.h"
#include "../config/mpe_constants.h" /* MFS_131 */
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
    /* MFS_131A: per-world warm-start cache. Heap-allocated in
     * physics_world_init (an inline array would be ~3 MB and would
     * overflow the stack of tests that declare worlds locally).
     * NULL-world callers fall back to the global cache. */
    cached_contact *world_contact_cache;
    int world_contact_cache_count;
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
} physics_world;

void physics_world_init(physics_world *world);
void physics_world_cleanup(physics_world *world);
int physics_world_add_sphere(physics_world *world, float radius, float mass, vector3 position);
int physics_world_add_cube(physics_world *world, vector3 position, vector3 half_extensions, float mass);
int physics_world_add_cylinder(physics_world *world, float radius, float half_length, float mass, vector3 position); /* MPE_FTC_090 */
void physics_world_clear(physics_world *world);
void physics_world_step(physics_world *world, float dt);
physics_world *physics_world_get_primary(void);
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

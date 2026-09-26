/* GTK4-PREP: zero GUI headers in physics. */
#include "collision_mechanics.h"
#include "../core/physics_world.h"
#include "../core/rigidbody.h"
#include "../core/det_math.h"
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

/* Warm-start hash: 4096 buckets over canonical (min_id, max_id) pairs.
 * Chains live in cached_contact.hash_next and are rebuilt on every save
 * in array order (reverse-prepend), so a lookup walk visits candidates in
 * exactly the array order the old legacy linear scan used: identical
 * first-hit, O(chain) instead of O(cache). Heads live in
 * physics_world.contact_hash_head (heap, per world). The old file-scope
 * global cache is retired; a NULL cache degrades to all-miss. */
#define contact_hash_bits 12
#define contact_hash_size (1 << contact_hash_bits)
#define contact_hash_mask (contact_hash_size - 1)

static inline uint32_t contact_pair_key(uint32_t id_a, uint32_t id_b) {
    uint32_t lo = (id_a < id_b) ? id_a : id_b;
    uint32_t hi = (id_a < id_b) ? id_b : id_a;
    uint64_t key = ((uint64_t) lo << 32) | (uint64_t) hi;
    /* splitmix64 finalizer over the combined key. */
    key ^= key >> 30;
    key *= 0xbf58476d1ce4e5b9ULL;
    key ^= key >> 27;
    key *= 0x94d049bb133111ebULL;
    key ^= key >> 31;
    return (uint32_t) (key & contact_hash_mask);
}

static inline vector4 collision_inverse_orientation(vector4 orientation) {
    return (vector4){orientation.w, -orientation.x, -orientation.y, -orientation.z};
}

static inline vector3 collision_world_offset_to_body_local(rigidbody *body, vector3 world_offset) {
    return vector4_rotate_to_vector3(collision_inverse_orientation(body->orientation), world_offset);
}

static inline vector3 collision_body_local_to_world_offset(rigidbody *body, vector3 local_offset) {
    return vector4_rotate_to_vector3(body->orientation, local_offset);
}

void contact_cache_stats_reset(struct physics_world *world) {
    if (!world) {
        return;
    }
    world->contact_cache_hits = 0;
    world->contact_cache_misses = 0;
}

int contact_cache_get_hits(const struct physics_world *world) {
    if (!world) {
        return 0;
    }
    return world->contact_cache_hits;
}

int contact_cache_get_misses(const struct physics_world *world) {
    if (!world) {
        return 0;
    }
    return world->contact_cache_misses;
}

bool contact_cache_has_pair(struct physics_world *world, uint32_t id_a, uint32_t id_b) {
    /* TRUTH: zero ids mean "unknown", not "seen". Returning true suppressed
     * first-touch wake for id-0 bodies (they never woke sleepers). */
    if (!world || id_a == 0 || id_b == 0) {
        return false;
    }
    cached_contact *cache = world->world_contact_cache;
    int count = world->world_contact_cache_count;
    if (!cache || count <= 0) {
        return false;
    }
    /* O(chain) hash walk over the chains rebuilt by contact_cache_save
     * (same entries, same order semantics as the old O(n) scan; boolean
     * result identical). Degrades to linear only if heads are missing. */
    int32_t *heads = world->contact_hash_head;
    if (heads) {
        uint32_t slot = contact_pair_key(id_a, id_b);
        for (int32_t s = heads[slot], guard = 0; s >= 0 && s < count && guard <= count;
             s = cache[s].hash_next, guard++) {
            uint32_t ca = cache[s].object_id_a;
            uint32_t cb = cache[s].object_id_b;
            if (((ca == id_a) && (cb == id_b)) || ((ca == id_b) && (cb == id_a))) {
                return true;
            }
        }
        return false;
    }
    if (count > world->world_contact_cache_capacity) {
        count = world->world_contact_cache_capacity;
    }
    for (int i = 0; i < count; i++) {
        uint32_t ca = cache[i].object_id_a;
        uint32_t cb = cache[i].object_id_b;
        if (((ca == id_a) && (cb == id_b)) || ((ca == id_b) && (cb == id_a))) {
            return true;
        }
    }
    return false;
}

/* FIX-AUDIT-DESPOT: thin wrapper over the shared stamp in
 * collision_mechanics.h (single source of truth with the solver's match
 * side). See the header note for why match-role predicates stay in the
 * solver TU. */
static uint32_t a3_task05_body_property_stamp(const rigidbody *rigid_body) {
    return a3_contact_cache_body_stamp(rigid_body);
}

void contact_cache_save(struct physics_world *world, collision_data *manifolds, int count) {
    /* Per-world warm-start cache (no global fallback remains). A missing
     * cache degrades to no warm start for the next tick. */
    if ((!world) || (!world->world_contact_cache)) {
        return;
    }
    int *cache_count = &world->world_contact_cache_count;
    cached_contact *cache_array = world->world_contact_cache;
    int32_t *hash_head = world->contact_hash_head;
    *cache_count = 0;
    for (int m = 0; m < count; m++) {
        collision_data *manifold = &manifolds[m];
        for (int i = 0; i < manifold->contact_count; i++) {
            if (*cache_count >= world->world_contact_cache_capacity) {
                if (physics_world_grow_contact_cache(world) != 0) {
                    break;
                }
                cache_array = world->world_contact_cache;
            }
            contact_point_data *cp = &manifold->contacts[i];
            cached_contact *cc = &cache_array[(*cache_count)++];
            cc->object_id_a = (manifold->object_a) ? manifold->object_a->object_id : 0;
            cc->object_id_b = (manifold->object_b) ? manifold->object_b->object_id : 0;
            /* MPE_TASK_05_CACHE_SAVE_STAMP_BEGIN */
            cc->property_stamp_a = a3_task05_body_property_stamp(manifold->object_a);
            cc->property_stamp_b = a3_task05_body_property_stamp(manifold->object_b);
            /* MPE_TASK_05_CACHE_SAVE_STAMP_END */
            cc->local_position_a = cp->local_position_a;
            cc->local_position_b = cp->local_position_b;
            cc->accumulated_normal_impulse = cp->accumulated_normal_impulse;
            cc->accumulated_tangent_impulse = cp->accumulated_tangent_impulse;
            /* (compression fields intentionally unsaved: reserved/ABI, the
             * solver derives compression as normal-minus-base). */
            /* Remember the stick frame for resting contacts next tick. */
            cc->tangent_dir = cp->tangent_vector;
        }
        if (*cache_count >= world->world_contact_cache_capacity) {
            break;
        }
    }
    /* Rebuild the lookup chains in array order (reverse-prepend), so a
     * lookup walk visits candidates in exactly the order the legacy linear
     * scan used: identical first-hit. Zero-id entries never match any
     * predicate and stay unchained. */
    if (hash_head) {
        for (int h = 0; h < contact_hash_size; h++) {
            hash_head[h] = -1;
        }
        for (int c = *cache_count - 1; c >= 0; c--) {
            cached_contact *cc = &cache_array[c];
            if ((cc->object_id_a == 0) || (cc->object_id_b == 0)) {
                cc->hash_next = -1;
                continue;
            }
            uint32_t slot = contact_pair_key(cc->object_id_a, cc->object_id_b);
            cc->hash_next = hash_head[slot];
            hash_head[slot] = c;
        }
    }
}

void contact_cache_clear(struct physics_world *world) {
    if (!world) {
        return;
    }
    world->world_contact_cache_count = 0;
}

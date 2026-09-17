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
    if (!world || id_a == 0 || id_b == 0) {
        return true;
    }
    cached_contact *cache = world->world_contact_cache;
    int count = world->world_contact_cache_count;
    if (!cache || count <= 0) {
        return false;
    }
    if (count > max_cached_contacts) {
        count = max_cached_contacts;
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

static uint32_t a3_task05_mix_u32(uint32_t hash_value, uint32_t input_value) {
    hash_value ^= input_value + 0x9e3779b9u + (hash_value << 6) + (hash_value >> 2);
    return hash_value;
}

static uint32_t a3_task05_float_bits(float value) {
    union {
        float float_value;
        uint32_t integer_value;
    } converter;

    converter.float_value = value;
    return converter.integer_value;
}

static uint32_t a3_task05_body_property_stamp(const rigidbody *rigid_body) {
    if (!rigid_body) {
        return 0;
    }

    uint32_t stamp = 2166136261u;

    stamp = a3_task05_mix_u32(stamp, (uint32_t) rigid_body->type);
    stamp = a3_task05_mix_u32(stamp, rigid_body->static_state ? 1u : 0u);

    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->mass));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->inverse_mass));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->radius));

    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->half_extensions.x));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->half_extensions.y));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->half_extensions.z));

    /* TRUTH: friction/restitution/kinematic affect the solved impulse.
     * Old stamp omitted them: editing friction or toggling kinematic hit a
     * stale acc_n*new_mu (wrong friction cone for a tick). Include. */
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->friction_static));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->friction_kinetic));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->restitution));
    stamp = a3_task05_mix_u32(stamp, rigid_body->kinematic ? 2u : 0u);
    stamp = a3_task05_mix_u32(stamp, rigid_body->is_sleeping ? 4u : 0u);

    /* Orientation quantized to 1e-3: rotation invalidates local-space cache
     * matching. Without this, a body that rotates significantly between
     * frames can false-positive match a stale contact (PHYS-007). Quantizing
     * keeps resting contacts stable while forcing a miss on real rotation. */
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(roundf(rigid_body->orientation.w * 1000.0f) / 1000.0f));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(roundf(rigid_body->orientation.x * 1000.0f) / 1000.0f));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(roundf(rigid_body->orientation.y * 1000.0f) / 1000.0f));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(roundf(rigid_body->orientation.z * 1000.0f) / 1000.0f));

    return stamp;
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
            if (*cache_count >= max_cached_contacts) {
                break;
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
            /* Remember the stick frame for resting contacts next tick. */
            cc->tangent_dir = cp->tangent_vector;
        }
        if (*cache_count >= max_cached_contacts) {
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

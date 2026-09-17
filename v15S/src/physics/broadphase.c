/* GTK4-PREP: zero GUI headers in core/physics. */
#include "broadphase.h"
#include "../core/physics_world.h"
#include "../core/rigidbody.h"
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/* All mutable state lives in the per-world workspace (broadphase_workspace,
 * owned by physics_world). No file-scope state remains; worlds never share
 * broadphase data. Static helpers below take the workspace explicitly. */

int broadphase_get_node_overflow_count(const struct physics_world *world) {
    if ((!world) || (!world->broadphase)) {
        return 0;
    }
    return world->broadphase->node_overflow_count;
}

int broadphase_get_pair_overflow_count(const struct physics_world *world) {
    if ((!world) || (!world->broadphase)) {
        return 0;
    }
    return world->broadphase->pair_overflow_count;
}

/* MPE_TASK_10_PAIR_DEDUPE_GETTER_BEGIN */
int broadphase_get_pair_dedupe_overflow_count(const struct physics_world *world) {
    if ((!world) || (!world->broadphase)) {
        return 0;
    }
    return world->broadphase->pair_dedupe_overflow_count;
}
/* MPE_TASK_10_PAIR_DEDUPE_GETTER_END */

/* MPE_TASK_11_LARGE_OBJECT_CLAMP_GETTER_BEGIN */
int broadphase_get_large_object_clamp_count(const struct physics_world *world) {
    if ((!world) || (!world->broadphase)) {
        return 0;
    }
    return world->broadphase->large_object_clamp_count;
}
/* MPE_TASK_11_LARGE_OBJECT_CLAMP_GETTER_END */

void broadphase_reset_overflow_counts(struct physics_world *world) {
    if ((!world) || (!world->broadphase)) {
        return;
    }
    world->broadphase->node_overflow_count = 0;
    world->broadphase->pair_overflow_count = 0;
    /* MPE_TASK_10_PAIR_DEDUPE_RESET_BEGIN */
    world->broadphase->pair_dedupe_overflow_count = 0;
    /* MPE_TASK_10_PAIR_DEDUPE_RESET_END */
    /* MPE_TASK_11_LARGE_OBJECT_CLAMP_RESET_BEGIN */
    world->broadphase->large_object_clamp_count = 0;
    /* MPE_TASK_11_LARGE_OBJECT_CLAMP_RESET_END */
}

void broadphase_cleanup(struct physics_world *world) {
    if ((!world) || (!world->broadphase)) {
        return;
    }
    if (world->broadphase->node_pool) {
        free(world->broadphase->node_pool);
        world->broadphase->node_pool = NULL;
        world->broadphase->node_pool_capacity = 0;
        world->broadphase->node_count = 0;
    }
}

/* MPE_TASK_17_CELL_SIZE_GETTER_BEGIN */
float broadphase_get_current_cell_size(const struct physics_world *world) {
    if ((!world) || (!world->broadphase)) {
        return 5.0f;
    }
    return world->broadphase->current_cell_size;
}
/* MPE_TASK_17_CELL_SIZE_GETTER_END */

static int hash_coordinate(int x, int y, int z) {
    unsigned int h = ((unsigned int) x) * 73856093u;
    h ^= ((unsigned int) y) * 19349663u;
    h ^= ((unsigned int) z) * 83492791u;
    return (int) (h % hash_table_size);
}

static bool broadphase_ensure_node_capacity(broadphase_workspace *ws) {
    if (!ws) {
        return false;
    }
    /* TRUTH: unbounded doubling lets one huge wall (250m/cell 1m) allocate
     * millions of nodes -> stall/OOM then false negatives. Cap at 1M nodes
     * (~12MB) and fail open with telemetry instead of OOM-killing. */
    const int kMaxNodes = 1 << 20;
    if (ws->node_pool == NULL) {
        int want = max_objects * 8;
        if (want > kMaxNodes) {
            want = kMaxNodes;
        }
        ws->node_pool_capacity = want;
        ws->node_pool = (hash_node *) malloc((size_t) ws->node_pool_capacity * sizeof(hash_node));
        if (ws->node_pool == NULL) {
            ws->node_pool_capacity = 0;
            ws->node_overflow_count++;
            return false;
        }
        return true;
    }
    if (ws->node_count < ws->node_pool_capacity) {
        return true;
    }
    if (ws->node_pool_capacity >= kMaxNodes) {
        ws->node_overflow_count++;
        return false;
    }
    int new_capacity = (ws->node_pool_capacity > 0) ? (ws->node_pool_capacity * 2) : (max_objects * 8);
    if (new_capacity > kMaxNodes) {
        new_capacity = kMaxNodes;
    }
    if (new_capacity <= ws->node_pool_capacity) {
        ws->node_overflow_count++;
        return false;
    }
    hash_node *new_pool = (hash_node *) realloc(ws->node_pool, (size_t) new_capacity * sizeof(hash_node));
    if (new_pool == NULL) {
        ws->node_overflow_count++;
        return false;
    }
    ws->node_pool = new_pool;
    ws->node_pool_capacity = new_capacity;
    return true;
}

static void insert_into_hash(broadphase_workspace *ws, int object_index, int x, int y, int z) {
    if (!broadphase_ensure_node_capacity(ws)) {
        return;
    }
    int hash = hash_coordinate(x, y, z);
    ws->node_pool[ws->node_count].object_index = object_index;
    ws->node_pool[ws->node_count].next_entry = ws->hash_table[hash];
    ws->hash_table[hash] = ws->node_count;
    ws->node_count++;
}

float broadphase_bounding_radius(rigidbody *rb) {
    if (!rb) {
        return 0.0f;
    }
    if (rb->type == object_sphere) {
        return rb->radius;
    }
    if (rb->type == object_cylinder) { /* MPE_FTC_091 */
        return sqrtf(rb->radius * rb->radius +
                     rb->cylinder_half_length * rb->cylinder_half_length);
    }
    return sqrtf(rb->half_extensions.x * rb->half_extensions.x +
                 rb->half_extensions.y * rb->half_extensions.y +
                 rb->half_extensions.z * rb->half_extensions.z);
}

static inline uint64_t a3_broadphase_pair_key(int object_a, int object_b) {
    return ((uint64_t) (uint32_t) object_a << 32) | (uint64_t) (uint32_t) object_b;
}

static inline uint32_t a3_broadphase_pair_hash(uint64_t key) {
    uint64_t mixed_key = key;
    mixed_key ^= mixed_key >> 33;
    mixed_key *= 0xff51afd7ed558ccdULL;
    mixed_key ^= mixed_key >> 33;
    mixed_key *= 0xc4ceb9fe1a85ec53ULL;
    mixed_key ^= mixed_key >> 33;
    return (uint32_t) (mixed_key & a3_pair_hash_mask);
}

static void broadphase_pair_dedupe_begin(broadphase_workspace *ws) {
    if (!ws) {
        return;
    }
    ws->pair_hash_generation++;
    if (ws->pair_hash_generation == 0) {
        for (int i = 0; i < a3_pair_hash_table_size; i++) {
            ws->pair_hash_generations[i] = 0;
        }
        ws->pair_hash_generation = 1;
    }
}

static bool pair_already_checked(broadphase_workspace *ws, int min_obj, int max_obj) {
    uint64_t key = a3_broadphase_pair_key(min_obj, max_obj);
    uint32_t index = a3_broadphase_pair_hash(key);
    for (uint32_t probe = 0; probe < a3_pair_hash_table_size; probe++) {
        uint32_t slot = (index + probe) & a3_pair_hash_mask;
        if (ws->pair_hash_generations[slot] != ws->pair_hash_generation) {
            ws->pair_hash_keys[slot] = key;
            ws->pair_hash_generations[slot] = ws->pair_hash_generation;
            return false;
        }
        if (ws->pair_hash_keys[slot] == key) {
            return true;
        }
    }
    /* FIX-AUDIT: table exhausted. Old code returned true ("already seen")
     * which silently DROPPED new pairs (missed collisions). Return false so
     * the pair is emitted (risk duplicate narrowphase work, never a miss).
     * Overflow is still counted for validation visibility. */
    ws->pair_dedupe_overflow_count++;
    return false;
    /* MPE_TASK_10_PAIR_DEDUPE_FALLBACK_END */
}

/* MPE_TASK_17_CELL_SIZE_FUNCTION_BEGIN */
static void broadphase_update_cell_size(struct physics_world *world, rigidbody *bodies,
                                        int body_count) { /* MPE_FTC_059 */
    broadphase_workspace *ws = world ? world->broadphase : NULL;
    if (!ws) {
        return;
    }
    if (body_count <= 0) {
        ws->current_cell_size = g_cfg.broadphase.cell_size_default;
        ws->cached_body_count = body_count;
        ws->ticks_since_cell_recompute = 0;
        return;
    }

    /* Cache: skip O(n) rescan if population is stable and we recomputed
     * recently. Recompute when count drifts >10% or every 60 ticks.
     * TRUTH: count alone hides same-count size swaps (spheres -> huge walls).
     * Also track average radius; drift >25% forces recompute. */
    ws->ticks_since_cell_recompute++;
    int count_delta = body_count > ws->cached_body_count ? body_count - ws->cached_body_count
                                                          : ws->cached_body_count - body_count;
    bool count_stable = (ws->cached_body_count > 0) && (count_delta * 10 < ws->cached_body_count);
    if (count_stable && ws->ticks_since_cell_recompute < 60 && ws->current_cell_size > 0.0f) {
        /* Cheap size-distribution probe: sample up to 16 bodies for avg radius drift. */
        float probe_sum = 0.0f;
        int probe_n = body_count < 16 ? body_count : 16;
        for (int pi = 0; pi < probe_n; pi++) {
            float pr = broadphase_bounding_radius(&bodies[(pi * body_count) / probe_n]);
            if (isfinite(pr) && pr > 0.0f) {
                probe_sum += pr;
            }
        }
        float probe_avg = probe_n > 0 ? probe_sum / (float) probe_n : 0.0f;
        float cached_avg = ws->current_cell_size / g_cfg.broadphase.cell_size_multiplier;
        if (cached_avg <= 0.0f) {
            cached_avg = 0.5f;
        }
        float drift = fabsf(probe_avg - cached_avg) / cached_avg;
        if (isfinite(drift) && drift < 0.25f) {
            return;
        }
    }
    ws->cached_body_count = body_count;
    ws->ticks_since_cell_recompute = 0;

    float radius_sum = 0.0f;
    float max_radius = 0.0f;

    for (int object_index = 0; object_index < body_count; object_index++) {
        float object_radius = broadphase_bounding_radius(&bodies[object_index]);
        if ((isfinite(object_radius)) && (object_radius > 0.0f)) {
            radius_sum += object_radius;
            if (object_radius > max_radius) {
                max_radius = object_radius;
            }
        }
    }

    float average_radius = radius_sum / (float) body_count;
    if ((!isfinite(average_radius)) || (average_radius <= 0.0f)) {
        average_radius = 0.5f;
    }
    if ((!isfinite(max_radius)) || (max_radius <= 0.0f)) {
        max_radius = 0.5f;
    }

    float desired_cell_size = g_cfg.broadphase.cell_size_multiplier * average_radius;
    float minimum_required_cell_size = (2.0f * max_radius) / (float) g_cfg.broadphase.max_cell_span_per_axis;

    if (desired_cell_size < minimum_required_cell_size) {
        desired_cell_size = minimum_required_cell_size;
    }

    if (desired_cell_size < g_cfg.broadphase.cell_size_min) {
        desired_cell_size = g_cfg.broadphase.cell_size_min;
    }

    if (desired_cell_size > g_cfg.broadphase.cell_size_max) {
        desired_cell_size = g_cfg.broadphase.cell_size_max;
    }

    ws->current_cell_size = desired_cell_size;
}
/* MPE_TASK_17_CELL_SIZE_FUNCTION_END */

int broadphase_generate_pairing(struct physics_world *world, broadphase_pair *collision_pairs_output_array,
                                int maximum_pairs_allowed, float dt) { /* MPE_FTC_059 */
    if ((!world) || (!world->bodies) || (!world->broadphase) || (!collision_pairs_output_array) ||
        (maximum_pairs_allowed <= 0)) {
        return 0;
    }
    rigidbody *bodies = world->bodies;
    int body_count = world->body_count;
    broadphase_workspace *ws = world->broadphase;
    if (!(dt > 0.0f) || !isfinite(dt)) {
        dt = 1.0f / 60.0f;
    }
    /* MPE_TASK_17_CELL_SIZE_CALL_BEGIN */
    if (body_count < 2) {
        ws->current_cell_size = g_cfg.broadphase.cell_size_default;
        ws->cached_body_count = body_count;
        ws->ticks_since_cell_recompute = 0;
        return 0;
    }

    broadphase_update_cell_size(world, bodies, body_count); /* MPE_FTC_059e */
    /* MPE_TASK_17_CELL_SIZE_CALL_END */
    for (int i = 0; i < hash_table_size; i++) {
        ws->hash_table[i] = -1;
    }
    ws->node_count = 0;
    broadphase_pair_dedupe_begin(ws);
    int collision_pair_counter = 0;
    for (int i = 0; i < body_count; i++) {
        rigidbody *rb = &bodies[i];
        float extent_x, extent_y, extent_z;
        if (rb->type == object_sphere) {
            extent_x = extent_y = extent_z = rb->radius;
        } else if (rb->type == object_cylinder) {
            float r = broadphase_bounding_radius(rb);
            extent_x = extent_y = extent_z = r;
        } else {
            vector3 *axes = rb->cached_axes;
            extent_x = fabsf(axes[0].x) * rb->half_extensions.x + fabsf(axes[1].x) * rb->half_extensions.y +
                       fabsf(axes[2].x) * rb->half_extensions.z;
            extent_y = fabsf(axes[0].y) * rb->half_extensions.x + fabsf(axes[1].y) * rb->half_extensions.y +
                       fabsf(axes[2].y) * rb->half_extensions.z;
            extent_z = fabsf(axes[0].z) * rb->half_extensions.x + fabsf(axes[1].z) * rb->half_extensions.y +
                       fabsf(axes[2].z) * rb->half_extensions.z;
        }
        /* Swept AABB: expand by this tick's linear motion so a fast body
         * pairs with everything along its path (CCD needs the pair to
         * exist). Sleeping/static bodies don't move: no expansion.
         * TRUTH P1-19: add angular sweep |w|*R*dt (tip-speed bound). A fast
         * spinner sweeps a disc of radius R; linear-only expansion tunnels
         * rotationally. Conservative: expands all axes uniformly. */
        if ((!rb->static_state) && (!rb->is_sleeping)) {
            float ang_sweep = vector3_length(rb->angular_velocity) * broadphase_bounding_radius(rb) * dt;
            if ((!isfinite(ang_sweep)) || (ang_sweep < 0.0f)) {
                ang_sweep = 0.0f;
            }
            extent_x += fabsf(rb->velocity.x) * dt + ang_sweep;
            extent_y += fabsf(rb->velocity.y) * dt + ang_sweep;
            extent_z += fabsf(rb->velocity.z) * dt + ang_sweep;
        }
        float cell_size = ws->current_cell_size;
        int min_x = (int) floorf((rb->position.x - extent_x) / cell_size);
        int max_x = (int) floorf((rb->position.x + extent_x) / cell_size);
        int min_y = (int) floorf((rb->position.y - extent_y) / cell_size);
        int max_y = (int) floorf((rb->position.y + extent_y) / cell_size);
        int min_z = (int) floorf((rb->position.z - extent_z) / cell_size);
        int max_z = (int) floorf((rb->position.z + extent_z) / cell_size);
        /* FIX-AUDIT: old code SHRANK the occupied interval to max_span,
         * so cells the body truly covers were never inserted -> missed
         * pairs (false negatives). Broadphase must never miss. Keep the
         * full span (correct); count the event for perf visibility. */
        bool a3_large_object_clamped = false;

        if ((max_x - min_x) > g_cfg.broadphase.max_cell_span_per_axis) {
            a3_large_object_clamped = true;
        }

        if ((max_y - min_y) > g_cfg.broadphase.max_cell_span_per_axis) {
            a3_large_object_clamped = true;
        }

        if ((max_z - min_z) > g_cfg.broadphase.max_cell_span_per_axis) {
            a3_large_object_clamped = true;
        }

        if (a3_large_object_clamped) {
            ws->large_object_clamp_count++;
        }
        /* MPE_TASK_11_LARGE_OBJECT_CLAMP_END */
        for (int x = min_x; x <= max_x; x++) {
            for (int y = min_y; y <= max_y; y++) {
                for (int z = min_z; z <= max_z; z++) {
                    insert_into_hash(ws, i, x, y, z);
                }
            }
        }
    }
    for (int i = 0; i < hash_table_size; i++) {
        int node_idx = ws->hash_table[i];
        while (node_idx != -1) {
            int obj_a = ws->node_pool[node_idx].object_index;
            int next_node_idx = ws->node_pool[node_idx].next_entry;
            while (next_node_idx != -1) {
                int obj_b = ws->node_pool[next_node_idx].object_index;
                if (obj_a != obj_b) {
                    int min_obj = obj_a < obj_b ? obj_a : obj_b;
                    int max_obj = obj_a > obj_b ? obj_a : obj_b;
                    if (!pair_already_checked(ws, min_obj, max_obj)) {
                        rigidbody *rb_a = &bodies[min_obj];
                        rigidbody *rb_b = &bodies[max_obj];
                        /* TRUTH: swept-insert then unswept cull tunnels fast bodies.
                         * Cells prove swept-AABB overlap; cull must be swept too.
                         * Expand by relative displacement over dt (linear + tip). */
                        vector3 dp = vector3_subtraction(rb_a->position, rb_b->position);
                        float dist_sq = vector3_length_squared(dp);
                        float rad_sum = broadphase_bounding_radius(rb_a) + broadphase_bounding_radius(rb_b);
                        float sweep = 0.0f;
                        if ((!rb_a->static_state && !rb_a->is_sleeping) ||
                            (!rb_b->static_state && !rb_b->is_sleeping)) {
                            vector3 dv = vector3_subtraction(rb_a->velocity, rb_b->velocity);
                            float vrel = vector3_length(dv);
                            float wa = vector3_length(rb_a->angular_velocity) * broadphase_bounding_radius(rb_a);
                            float wb = vector3_length(rb_b->angular_velocity) * broadphase_bounding_radius(rb_b);
                            if (!isfinite(vrel)) {
                                vrel = 0.0f;
                            }
                            if (!isfinite(wa)) {
                                wa = 0.0f;
                            }
                            if (!isfinite(wb)) {
                                wb = 0.0f;
                            }
                            sweep = (vrel + wa + wb) * dt;
                        }
                        float swept_sum = rad_sum + sweep;
                        if (!isfinite(swept_sum) || swept_sum < 0.0f) {
                            swept_sum = rad_sum;
                        }
                        if (dist_sq <= swept_sum * swept_sum) {
                            if (collision_pair_counter < maximum_pairs_allowed) {
                                collision_pairs_output_array[collision_pair_counter].object_index_a = min_obj;
                                collision_pairs_output_array[collision_pair_counter].object_index_b = max_obj;
                                collision_pair_counter++;
                            } else {
                                ws->pair_overflow_count++;
                            }
                        }
                    }
                }
                next_node_idx = ws->node_pool[next_node_idx].next_entry;
            }
            node_idx = ws->node_pool[node_idx].next_entry;
        }
    }
    return collision_pair_counter;
}

/* MPE_FTC_059C: physics world — full pipeline.
 * Supersedes MPE_FTC_056 (free-body step).
 * Pipeline mirrors the legacy loop in simulation.c, minus:
 *   - sleep staticize hack (sleeping bodies keep real mass here),
 *   - positional depenetration pass,
 *   - joints/constraints (Phase 1).
 * NOTE: the contact warm-start cache is still engine-global. Worlds
 * must seed non-overlapping object_id ranges (see tests/two_world_test.c).
 * A per-world cache is tracked as future work.
 */
#include "physics_world.h"
#include "mpe_registry.h"
#include "../physics/collision_mechanics.h"
#include "../physics/broadphase.h"
#include "../physics/constraint.h" /* MPE_FTC_067 */
#include "../physics/islands.h"
#include "../physics/depenetration.h"
#include "../scene/boundary.h"
#include "det_math.h" /* bit-identical damping factors on all IEEE targets */
/* Canonical spring pass is weakly linked so spring-less headless test
 * binaries (which omit physics/spring_joint.c for its GL dependency)
 * still link; the GUI engine and TUI link it and get real forces. */
void mpe_springs_apply(physics_world *world, float dt) __attribute__((weak));
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include <stdlib.h>
#include <math.h>
#include <string.h> /* MPE_FTC_076a */

/* NOTE: no file-scope simulation state in this TU. The application
 * primary lives in core/mpe_primary.c; all stepping takes explicit
 * worlds (see physics_world_get_primary contract in the header). */

/* Helper: malloc-or-NULL (leaves member NULL on failure; users degrade
 * gracefully — add fns reject NULL bodies, pairing returns 0, solves skip).
 * Worlds that fail init are safely unusable, never crash-prone. */
static void physics_world_free_ptr(void **slot) {
    if (slot && *slot) {
        free(*slot);
        *slot = NULL;
    }
}

void physics_world_init(physics_world *world) {
    if (!world) {
        return;
    }
    det_pin_fp_state();
    memset(world, 0, sizeof(physics_world)); /* MPE_FTC_076a */
    /* Phase-1: default-bind global config (back-compat). Caller may
     * rebind via physics_world_set_config() for isolation. */
    world->cfg = &g_cfg;
    mpe_register_builtins();
    /* Growable pools: start small, double on demand (see growers below).
     * body_capacity tracks the live allocation (not the ceiling). */
    if (!world->bodies) {
        world->bodies = (rigidbody *) malloc((size_t) mpe_initial_bodies * sizeof(rigidbody));
        world->body_capacity = world->bodies ? mpe_initial_bodies : 0;
    }
    if (!world->world_contact_cache) {
        world->world_contact_cache =
            (cached_contact *) malloc((size_t) mpe_initial_contacts * sizeof(cached_contact)); /* MFS_131A */
        world->world_contact_cache_capacity = world->world_contact_cache ? mpe_initial_contacts : 0;
    }
    /* Warm-start hash heads: heap, not stack (worlds are often stack-local;
     * an inline 16 KB array risks overflow next to big frames). */
    if (!world->contact_hash_head) {
        world->contact_hash_head = (int32_t *) malloc((size_t) 4096 * sizeof(int32_t));
        if (world->contact_hash_head) {
            for (int i = 0; i < 4096; i++) {
                world->contact_hash_head[i] = -1;
            }
        }
    }
    /* Solver scratch (heap; worlds are frequently stack-local). Sizes are
     * compile-time caps from mpe_constants.h; pages fault on first touch. */
    if (!world->broadphase) {
        world->broadphase = (broadphase_workspace *) calloc(1, sizeof(broadphase_workspace));
        if (world->broadphase) {
            world->broadphase->current_cell_size = 5.0f;
        }
    }
    if (!world->pair_buffer) {
        world->pair_buffer = (broadphase_pair *) malloc((size_t) mpe_max_broadphase_pairs * sizeof(broadphase_pair));
    }
    if (!world->manifolds) {
        world->manifolds = (collision_data *) malloc((size_t) a3_max_manifolds * sizeof(collision_data));
    }
    if (!world->manifold_awake) {
        world->manifold_awake = (unsigned char *) malloc((size_t) a3_max_manifolds * sizeof(unsigned char));
    }
    if (!world->manifold_order) {
        world->manifold_order = (int *) malloc((size_t) a3_max_manifolds * sizeof(int));
    }
    if (!world->manifold_sort_keys) {
        world->manifold_sort_keys = (float *) malloc((size_t) a3_max_manifolds * sizeof(float));
    }
    if (!world->pair_skipped) {
        world->pair_skipped = (unsigned char *) malloc((size_t) mpe_max_broadphase_pairs * sizeof(unsigned char));
    }
    if (!world->island_parent) {
        world->island_parent = (int *) malloc((size_t) mpe_max_bodies * sizeof(int));
    }
    if (!world->island_label) {
        world->island_label = (int *) malloc((size_t) mpe_max_bodies * sizeof(int));
    }
    if (!world->island_awake_flags) {
        world->island_awake_flags = (unsigned char *) malloc((size_t) mpe_max_bodies * sizeof(unsigned char));
    }
    if (!world->ccd_time_remaining) {
        world->ccd_time_remaining = (float *) malloc((size_t) mpe_max_bodies * sizeof(float));
    }
    if (!world->has_contact) {
        world->has_contact = (unsigned char *) malloc((size_t) mpe_max_bodies * sizeof(unsigned char));
    }
    /* id->index cache (heap; worlds are frequently stack-local). */
    if (!world->id_cache_keys) {
        world->id_cache_size = mpe_id_cache_size;
        world->id_cache_keys = (uint32_t *) calloc((size_t) world->id_cache_size, sizeof(uint32_t));
        world->id_cache_vals = (int *) malloc((size_t) world->id_cache_size * sizeof(int));
        world->id_cache_valid = (unsigned char *) calloc((size_t) world->id_cache_size, sizeof(unsigned char));
        if (!world->id_cache_keys || !world->id_cache_vals || !world->id_cache_valid) {
            physics_world_free_ptr((void **) &world->id_cache_keys);
            physics_world_free_ptr((void **) &world->id_cache_vals);
            physics_world_free_ptr((void **) &world->id_cache_valid);
            world->id_cache_size = 0;
        }
        world->id_cache_revision = 0;
    }
    world->body_count = 0;
    if (world->next_object_id == 0) {
        world->next_object_id = 1;
    }
}

void physics_world_cleanup(physics_world *world) {
    if (!world) {
        return;
    }
    physics_world_free_ptr((void **) &world->bodies);
    physics_world_free_ptr((void **) &world->world_contact_cache);
    physics_world_free_ptr((void **) &world->contact_hash_head);
    broadphase_cleanup(world); /* frees the node pool; struct freed below */
    physics_world_free_ptr((void **) &world->broadphase);
    physics_world_free_ptr((void **) &world->pair_buffer);
    physics_world_free_ptr((void **) &world->manifolds);
    physics_world_free_ptr((void **) &world->manifold_awake);
    physics_world_free_ptr((void **) &world->manifold_order);
    physics_world_free_ptr((void **) &world->manifold_sort_keys);
    physics_world_free_ptr((void **) &world->pair_skipped);
    physics_world_free_ptr((void **) &world->island_parent);
    physics_world_free_ptr((void **) &world->island_label);
    physics_world_free_ptr((void **) &world->island_awake_flags);
    physics_world_free_ptr((void **) &world->ccd_time_remaining);
    physics_world_free_ptr((void **) &world->has_contact);
    physics_world_free_ptr((void **) &world->id_cache_keys);
    physics_world_free_ptr((void **) &world->id_cache_vals);
    physics_world_free_ptr((void **) &world->id_cache_valid);
    world->id_cache_size = 0;
    world->world_contact_cache_count = 0;
    world->body_count = 0;
    world->body_capacity = 0;
    world->cfg = &g_cfg;
}

void physics_world_set_config(physics_world *world, mpe_config_t *cfg) {
    if (!world) return;
    world->cfg = cfg ? cfg : &g_cfg;
}

/* Pool growers: ×2 up to the compile-time ceilings. Manifold pointers
 * into bodies[] are rebuilt every tick, and caches store ids (never
 * pointers), so relocation during add_* (outside any step) is safe. */
static int physics_world_grow_bodies(physics_world *world) {
    if (!world || !world->bodies) {
        return -1;
    }
    if (world->body_capacity >= mpe_max_bodies) {
        return -1;
    }
    int want = world->body_capacity > 0 ? world->body_capacity * 2 : mpe_initial_bodies;
    if (want > mpe_max_bodies) {
        want = mpe_max_bodies;
    }
    rigidbody *grown = (rigidbody *) realloc(world->bodies, (size_t) want * sizeof(rigidbody));
    if (!grown) {
        return -1;
    }
    world->bodies = grown;
    world->body_capacity = want;
    return 0;
}

int physics_world_grow_contact_cache(physics_world *world) {
    if (!world || !world->world_contact_cache) {
        return -1;
    }
    if (world->world_contact_cache_capacity >= max_cached_contacts) {
        return -1;
    }
    int want = world->world_contact_cache_capacity > 0 ? world->world_contact_cache_capacity * 2
                                                       : mpe_initial_contacts;
    if (want > max_cached_contacts) {
        want = max_cached_contacts;
    }
    cached_contact *grown =
        (cached_contact *) realloc(world->world_contact_cache, (size_t) want * sizeof(cached_contact));
    if (!grown) {
        return -1;
    }
    world->world_contact_cache = grown;
    world->world_contact_cache_capacity = want;
    return 0;
}

mpe_config_t *physics_world_get_config(physics_world *world) {
    if (!world || !world->cfg) return &g_cfg;
    return world->cfg;
}

void physics_world_bump_revision(physics_world *world) {
    if (!world) return;
    world->body_revision++;
}

static void physics_world_id_cache_rebuild(physics_world *world) {
    for (int i = 0; i < world->id_cache_size; i++) {
        world->id_cache_valid[i] = 0;
    }
    if (!world->bodies || world->body_count <= 0) {
        world->id_cache_revision = world->body_revision;
        return;
    }
    uint32_t mask = (uint32_t) (world->id_cache_size - 1);
    for (int i = 0; i < world->body_count; i++) {
        uint32_t id = world->bodies[i].object_id;
        if (id == 0 || id == 0xFFFFFFFFu) {
            continue;
        }
        uint32_t h = (id * 2654435761u) & mask;
        for (int probe = 0; probe < 32; probe++) {
            uint32_t s = (h + (uint32_t) probe) & mask;
            if (!world->id_cache_valid[s]) {
                world->id_cache_keys[s] = id;
                world->id_cache_vals[s] = i;
                world->id_cache_valid[s] = 1;
                break;
            }
        }
    }
    world->id_cache_revision = world->body_revision;
}

int physics_world_index_by_id(physics_world *world, uint32_t id) {
    if (!world || !world->bodies || world->body_count <= 0 || id == 0) {
        return -1;
    }
    if (world->id_cache_size > 0 && world->id_cache_keys && world->id_cache_vals && world->id_cache_valid) {
        if (world->id_cache_revision != world->body_revision) {
            physics_world_id_cache_rebuild(world);
        }
        uint32_t mask = (uint32_t) (world->id_cache_size - 1);
        uint32_t h = (id * 2654435761u) & mask;
        for (int probe = 0; probe < 32; probe++) {
            uint32_t s = (h + (uint32_t) probe) & mask;
            if (!world->id_cache_valid[s]) {
                break;
            }
            if (world->id_cache_keys[s] == id) {
                int idx = world->id_cache_vals[s];
                if (idx >= 0 && idx < world->body_count && world->bodies[idx].object_id == id) {
                    return idx;
                }
                break;
            }
        }
    }
    for (int i = 0; i < world->body_count; i++) {
        if (world->bodies[i].object_id == id) {
            return i;
        }
    }
    return -1;
}

rigidbody *physics_world_body_by_id(physics_world *world, uint32_t id) {
    int idx = physics_world_index_by_id(world, id);
    if (idx < 0) {
        return NULL;
    }
    return &world->bodies[idx];
}

void physics_world_set_broadphase(physics_world *world, const mpe_broadphase_if_t *iface) {
    if (!world) return;
    world->broadphase_if = iface;
}

void physics_world_set_solver(physics_world *world, const mpe_solver_if_t *iface) {
    if (!world) return;
    world->solver_if = iface;
}

int physics_world_attach_module(physics_world *world, const mpe_module_desc_t *desc) {
    if (!world || !desc || desc->abi != MPE_MODULE_ABI) return -1;
    for (int i = 0; i < world->tick_module_count; i++)
        if (world->tick_modules[i] == desc) return i;
    if (world->tick_module_count >= 16) return -1;
    void *st = NULL;
    if (desc->attach && desc->attach(world, &st) != 0) return -1;
    world->tick_modules[world->tick_module_count] = desc;
    world->tick_module_state[world->tick_module_count] = st;
    return world->tick_module_count++;
}

int physics_world_detach_module(physics_world *world, const char *name) {
    if (!world || !name) return -1;
    for (int i = 0; i < world->tick_module_count; i++) {
        if (world->tick_modules[i] && world->tick_modules[i]->name &&
            strcmp(world->tick_modules[i]->name, name) == 0) {
            if (world->tick_modules[i]->detach)
                world->tick_modules[i]->detach(world, world->tick_module_state[i]);
            for (int j = i; j + 1 < world->tick_module_count; j++) {
                world->tick_modules[j] = world->tick_modules[j + 1];
                world->tick_module_state[j] = world->tick_module_state[j + 1];
            }
            world->tick_module_count--;
            world->tick_modules[world->tick_module_count] = NULL;
            world->tick_module_state[world->tick_module_count] = NULL;
            return 0;
        }
    }
    return -1;
}

/* Registry-first shape dispatch with built-in fallback.
 * Handles swapped (cube,sphere)/(cyl,sphere)/(cyl,cube) by trying the
 * registered orientation first, then the swapped orientation with a
 * normal flip — mirroring the legacy inline chains. Custom shapes
 * go through the registry only (no built-in fallback). */
bool mpe_shape_dispatch(physics_world *world, rigidbody *a, rigidbody *b, collision_data *out) {
    if (!a || !b || !out) return false;
    mpe_register_builtins();
    int ca = (a->type == object_custom) ? a->custom_shape : -1;
    int cb = (b->type == object_custom) ? b->custom_shape : -1;
    mpe_collide_fn fn = mpe_find_pair_handler((int)a->type, (int)b->type, ca, cb);
    if (fn) return fn(a, b, out, world);
    /* try swapped orientation (registry may hold canonical order only) */
    fn = mpe_find_pair_handler((int)b->type, (int)a->type, cb, ca);
    if (fn) {
        collision_data tmp = {0};
        if (!fn(b, a, &tmp, world)) return false;
        *out = tmp;
        out->normal_vector = vector3_scaling(tmp.normal_vector, -1.0f);
        out->object_a = a;
        out->object_b = b;
        return true;
    }
    return false;
}

int physics_world_add_sphere(physics_world *world, float radius, float mass, vector3 position) {
    if ((!world) || (!world->bodies)) {
        return -1;
    }
    if (world->body_count >= world->body_capacity && physics_world_grow_bodies(world) != 0) {
        return -1;
    }
    rigidbody *rb = &world->bodies[world->body_count];
    rigidbody_initialisation_sphere(rb, radius, mass, position);
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) {
        world->next_object_id = 1;
    }
    rb->object_id = world->next_object_id++;
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) {
        world->next_object_id = 1;
    }
    rb->object_generation = 1;
    rigidbody_sanitize(rb);
    physics_world_bump_revision(world);
    return world->body_count++;
}

int physics_world_add_cube(physics_world *world, vector3 position, vector3 half_extensions, float mass) {
    if ((!world) || (!world->bodies)) {
        return -1;
    }
    if (world->body_count >= world->body_capacity && physics_world_grow_bodies(world) != 0) {
        return -1;
    }
    rigidbody *rb = &world->bodies[world->body_count];
    rigidbody_initialisation_cube(rb, position, half_extensions, mass);
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) {
        world->next_object_id = 1;
    }
    rb->object_id = world->next_object_id++;
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) {
        world->next_object_id = 1;
    }
    rb->object_generation = 1;
    rigidbody_sanitize(rb);
    physics_world_bump_revision(world);
    return world->body_count++;
}

/* MPE_FTC_091 */
int physics_world_add_cylinder(physics_world *world, float radius, float half_length, float mass,
                             vector3 position) {
    if ((!world) || (!world->bodies)) {
        return -1;
    }
    if (world->body_count >= world->body_capacity && physics_world_grow_bodies(world) != 0) {
        return -1;
    }
    rigidbody *rb = &world->bodies[world->body_count];
    rigidbody_initialisation_cylinder(rb, radius, half_length, mass, position);
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) {
        world->next_object_id = 1;
    }
    rb->object_id = world->next_object_id++;
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) {
        world->next_object_id = 1;
    }
    rb->object_generation = 1;
    rigidbody_sanitize(rb);
    physics_world_bump_revision(world);
    return world->body_count++;
}

int physics_world_add_custom(physics_world *world, int custom_shape, vector3 position, float mass, float radius) {
    if ((!world) || (!world->bodies)) return -1;
    if (world->body_count >= world->body_capacity && physics_world_grow_bodies(world) != 0) return -1;
    if (custom_shape < 100) custom_shape = 100;
    rigidbody *rb = &world->bodies[world->body_count];
    /* Backing is a sphere (bounding volume + inertia sane until the
     * plugin overrides); type marks it foreign for dispatch. */
    rigidbody_initialisation_sphere(rb, radius > 0.0f ? radius : 0.5f, mass, position);
    rb->type = object_custom;
    rb->custom_shape = custom_shape;
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) world->next_object_id = 1;
    rb->object_id = world->next_object_id++;
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) world->next_object_id = 1;
    rb->object_generation = 1;
    rigidbody_sanitize(rb);
    rb->type = object_custom; /* sanitize must not reset foreign type */
    rb->custom_shape = custom_shape;
    physics_world_bump_revision(world);
    return world->body_count++;
}

void physics_world_clear(physics_world *world) {
    if (!world) {
        return;
    }
    world->body_count = 0;
    world->world_contact_cache_count = 0; /* MFS_131A */
    world->manifold_overflow_count = 0;
    world->contact_cache_hits = 0;
    world->contact_cache_misses = 0;
    physics_world_bump_revision(world);
    /* TRUTH: next_object_id monotonic wraps at 4G to 0, colliding with
     * cache sentinel id==0 (no match) + floor 0xFFFFFFFF. Skip 0/0xFFFFFFFF
     * on wrap. clear() does NOT reset IDs (stable across clears would alias
     * old cache entries); instead ensure wrap skips sentinels. */
    if (world->next_object_id == 0 || world->next_object_id == 0xFFFFFFFFu) {
        world->next_object_id = 1;
    }
}

/* One broadphase pair through narrowphase + wake-on-contact + solver
 * prep. Shared by the main pair loop and the sleep-wake revisit pass
 * below (extracted verbatim from the former single loop). Non-static:
 * the legacy GUI tick reuses it so both step paths share one wake +
 * dispatch implementation (registry-routed, per-world config). */
void physics_world_process_pair(physics_world *world, int index_a, int index_b, float dt,
                                int *manifold_count_ptr) {
    if ((index_a < 0) || (index_a >= world->body_count)) {
        return;
    }
    if ((index_b < 0) || (index_b >= world->body_count)) {
        return;
    }
    rigidbody *body_a = &world->bodies[index_a];
    rigidbody *body_b = &world->bodies[index_b];
        collision_data narrowphase_collision = {0};
        /* Phase-2: registry-first dispatch (foreign shapes plug in here),
         * built-in table fallback preserves exact legacy behaviour. */
        bool collided = mpe_shape_dispatch(world, body_a, body_b, &narrowphase_collision);
        if (collided) {
            if ((*manifold_count_ptr) >= a3_max_manifolds) {
                world->manifold_overflow_count++;
                return;
            }
            /* TRUTH: three-gate wake (any fires).
             * 1. NEW EDGE (pair novelty via the contact cache): either id
             *    pair has no entry from a prior tick. First touch wakes at
             *    ANY speed — this closes the slow-pusher ghost (creeping
             *    stack partner, 0.05 m/s kinematic platform) that pure
             *    velocity gates miss, and unlike per-body flags it is not
             *    blinded by floor contacts every rester holds. Persistent
             *    resting pairs have entries, so settled stacks proceed to
             *    sleep instead of being kept awake forever (which pumped
             *    the F10 10-stack to 13 m/s via never-sleeping micro-motion).
             * 2. FAST OTHER: original velocity gate (backstop for fast bodies
             *    that already held contacts, e.g. debris sliding along a
             *    stack before striking a sleeper).
             * 3. DEEP: overlap beyond wake_depth_thresh, even vs statics. */
            bool a_was_sleeping = body_a->is_sleeping;
            bool b_was_sleeping = body_b->is_sleeping;

            if ((a_was_sleeping) && (b_was_sleeping)) {
                /* Both sleeping: nothing to wake (depth backstop below still
                 * applies via the revisit/split/depen paths). */
            } else {
                bool new_edge = !contact_cache_has_pair(world, body_a->object_id, body_b->object_id);
                if ((a_was_sleeping) && (!body_b->static_state) && new_edge) {
                    rigidbody_wake(body_a);
                }
                if ((b_was_sleeping) && (!body_a->static_state) && new_edge) {
                    rigidbody_wake(body_b);
                }

                float wake_lin_sq = mpe_world_cfg(world)->sleep.wake_linear_thresh_sq;
                float wake_ang_sq = mpe_world_cfg(world)->sleep.wake_angular_thresh_sq;
                bool a_fast = (!a_was_sleeping) &&
                              ((vector3_length_squared(body_a->velocity) > wake_lin_sq) ||
                               (vector3_length_squared(body_a->angular_velocity) > wake_ang_sq));
                bool b_fast = (!b_was_sleeping) &&
                              ((vector3_length_squared(body_b->velocity) > wake_lin_sq) ||
                               (vector3_length_squared(body_b->angular_velocity) > wake_ang_sq));
                if ((a_was_sleeping) && (!body_b->static_state) && b_fast) {
                    rigidbody_wake(body_a);
                }
                if ((b_was_sleeping) && (!body_a->static_state) && a_fast) {
                    rigidbody_wake(body_b);
                }

                /* Wake on significant overlap growth, even against static
                 * geometry: a sleeper ground into a static surface by a new
                 * persistent force (e.g. a spawned overlap, a tilted stack
                 * settling) must rejoin the solve. Resting contact within
                 * slop never reaches this bar. */
                {
                    float deepest = 0.0f;
                    for (int wi = 0; wi < narrowphase_collision.contact_count; wi++) {
                        if (narrowphase_collision.contacts[wi].penetration > deepest) {
                            deepest = narrowphase_collision.contacts[wi].penetration;
                        }
                    }
                    if (deepest > mpe_world_cfg(world)->depenetration.wake_depth_thresh) {
                        if (a_was_sleeping) {
                            rigidbody_wake(body_a);
                        }
                        if (b_was_sleeping) {
                            rigidbody_wake(body_b);
                        }
                    }
                }

                collision_prepare_solver(world, &narrowphase_collision, &world->manifolds[(*manifold_count_ptr)], dt);
                (*manifold_count_ptr)++;
                /* TRUTH P0-1: contact flag for gravity-exactness gating. */
                if (world->has_contact) {
                    if ((index_a >= 0) && (index_a < world->body_count)) {
                        world->has_contact[index_a] = 1;
                    }
                    if ((index_b >= 0) && (index_b < world->body_count)) {
                        world->has_contact[index_b] = 1;
                    }
                }
            }
        }
}

/* Solver stage dispatch: foreign solver_if hooks override per stage,
 * builtins run on the tick's config snapshot. Keeps the iteration loop
 * readable while making every stage hot-swappable. */
static float mpe_step_resolve(physics_world *world, collision_data *m, float dt, bool friction_only, int iter,
                              const mpe_config_t *cfg) {
    if (world->solver_if && world->solver_if->resolve) {
        return world->solver_if->resolve(world, m, dt, friction_only, iter, NULL);
    }
    return collision_resolve_iterative(m, dt, friction_only, iter, cfg);
}
static void mpe_step_poisson(physics_world *world, collision_data *manifolds, int n, const mpe_config_t *cfg) {
    if (world->solver_if && world->solver_if->poisson) {
        world->solver_if->poisson(world, manifolds, n, NULL);
        return;
    }
    collision_apply_poisson_restitution(manifolds, n, cfg);
}
static void mpe_step_rolling(physics_world *world, collision_data *manifolds, int n, float dt,
                             const mpe_config_t *cfg) {
    if (world->solver_if && world->solver_if->rolling) {
        world->solver_if->rolling(world, manifolds, n, dt, NULL);
        return;
    }
    collision_apply_rolling_resistance(manifolds, n, dt, cfg);
}
static void mpe_step_split(physics_world *world, collision_data *manifolds, int n, float dt,
                           const mpe_config_t *cfg) {
    if (world->solver_if && world->solver_if->split) {
        world->solver_if->split(world, manifolds, n, dt, NULL);
        return;
    }
    collision_apply_split_impulse(manifolds, n, dt, cfg);
}

void physics_world_step(physics_world *world, float dt) {    if ((!world) || (!world->bodies) || (!(dt > 0.0f)) || (!isfinite(dt)) || (world->body_count <= 0)) {
        return;
    }
    if (dt > 0.1f) {
        dt = 0.1f;
    }

    for (int i = 0; i < world->body_count; i++) {
        rigidbody_sanitize(&world->bodies[i]);
    }
    /* TRUTH: snapshot config once per tick. Menu/terminal mutating g_cfg
     * mid-tick (between substeps) would otherwise change behavior halfway
     * through the frame. Hot path uses these locals, never g_cfg directly.
     * Phase-1: snapshot from per-world cfg (defaults to global). */
    const mpe_config_t *step_cfg = mpe_world_cfg(world);
    const float step_gravity = step_cfg->world.gravity;
    const float step_drag = step_cfg->world.drag;
    const float step_ang_scale = step_cfg->world.angular_damping_scale;
    const int step_iterations = step_cfg->timestep.solver_iterations;
    /* TRUTH P0-1: reset contact flags (set below on every manifold). */
    if (world->has_contact) {
        for (int i = 0; i < world->body_count; i++) {
            world->has_contact[i] = 0;
        }
    }

    /* CCD: clamp fast bodies to their time-of-impact pose before pairing,
     * so discrete narrowphase cannot tunnel past thin geometry.
     * TRUTH P0-3: remainder (dt-toi) recorded per body; post-solve
     * integration advances only the remainder (see below). */
    collision_ccd_sweep_clamp_world(world, dt);

    int pair_count = 0;
    if (world->body_count >= 2) {
        if (world->broadphase_if && world->broadphase_if->generate)
            pair_count = world->broadphase_if->generate(world, world->pair_buffer,
                                                        mpe_max_broadphase_pairs, dt, NULL);
        else
            pair_count = broadphase_generate_pairing(world, world->pair_buffer, mpe_max_broadphase_pairs, dt);
    }
    int broadphase_pair_count = pair_count; /* saved for depenetration pass */

    /* Low-memory contract: degraded (pairless) tick instead of a crash when
     * scratch failed to allocate. TRUTH: sort_keys missing left order_out
     * uninitialized -> OOB solve. Check it too. */
    if ((!world->pair_buffer) || (!world->manifolds) || (!world->manifold_awake) || (!world->manifold_order) ||
        (!world->manifold_sort_keys) || (!world->pair_skipped) || (!world->broadphase)) {
        return;
    }
    int manifold_count = 0;
    for (int q = 0; q < pair_count; q++) {
        world->pair_skipped[q] = 0;
    }
    contact_cache_stats_reset(world);
    for (int p = 0; p < pair_count; p++) {
        int index_a = world->pair_buffer[p].object_index_a;
        int index_b = world->pair_buffer[p].object_index_b;
        if ((index_a < 0) || (index_a >= world->body_count) || (index_b < 0) ||
            (index_b >= world->body_count)) {
            continue;
        }
        rigidbody *body_a = &world->bodies[index_a];
        rigidbody *body_b = &world->bodies[index_b];
        if ((body_a->is_sleeping) && (body_b->is_sleeping)) {
            world->pair_skipped[p] = 1;
            continue;
        }
        physics_world_process_pair(world, index_a, index_b, dt, &manifold_count);
    }
    /* Sleep-wake revisit fixpoint: pairs skipped above as both-asleep get
     * re-processed if either endpoint woke during the main loop (a woken
     * body with no manifold free-falls the whole tick otherwise). Each
     * pass only touches newly-active skipped pairs; cascades settle pass
     * by pass. Deterministic (pair order, wake-driven, bounded). */
    for (int revisit_pass = 0; revisit_pass < 16; revisit_pass++) {
        bool revisit_woke = false;
        for (int p = 0; p < pair_count; p++) {
            if (!world->pair_skipped[p]) {
                continue;
            }
            int index_a = world->pair_buffer[p].object_index_a;
            int index_b = world->pair_buffer[p].object_index_b;
            if ((index_a < 0) || (index_a >= world->body_count) || (index_b < 0) ||
                (index_b >= world->body_count)) {
                world->pair_skipped[p] = 0;
                continue;
            }
            rigidbody *body_a = &world->bodies[index_a];
            rigidbody *body_b = &world->bodies[index_b];
            if ((body_a->is_sleeping) && (body_b->is_sleeping)) {
                continue;
            }
            bool slept_a = body_a->is_sleeping;
            bool slept_b = body_b->is_sleeping;
            physics_world_process_pair(world, index_a, index_b, dt, &manifold_count);
            world->pair_skipped[p] = 0;
            if ((slept_a && !body_a->is_sleeping) || (slept_b && !body_b->is_sleeping)) {
                revisit_woke = true;
            }
        }
        if (!revisit_woke) {
            break;
        }
    }

    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];
        if (rb->static_state) {
            continue;
        }
        /* TRUTH: sleeping floor contact must still build a manifold (with
         * eff_inv=0 it solves as no-op) so has_contact is set and deep
         * sink wakes. Old skip left shallow sleepers intersecting floor
         * forever with has_contact=0 (false free-flight). */
        bool was_sleeping = rb->is_sleeping;
        collision_data floor_collision = {0};
        if (collision_static_plane_body(rb, 0.0f, &floor_collision, step_cfg)) {
            if (manifold_count >= a3_max_manifolds) {
                world->manifold_overflow_count++;
                continue;
            }
            float deepest = 0.0f;
            for (int fi = 0; fi < floor_collision.contact_count; fi++) {
                if (floor_collision.contacts[fi].penetration > deepest) {
                    deepest = floor_collision.contacts[fi].penetration;
                }
            }
            if (was_sleeping && deepest > step_cfg->depenetration.wake_depth_thresh) {
                rigidbody_wake(rb);
            }
            collision_prepare_solver(world, &floor_collision, &world->manifolds[manifold_count], dt);
            manifold_count++;
            if (world->has_contact) {
                world->has_contact[i] = 1;
            }
        }
    }

    /* Solver islands: union pairs + joints, then flag each manifold.
     * Fully-sleeping islands skip the iteration/relaxation loops below
     * (exact no-ops: zero velocity, zeroed inverses). */
    islands_build(world, world->pair_buffer, pair_count);
    for (int m = 0; m < manifold_count; m++) {
        rigidbody *ma = world->manifolds[m].object_a;
        rigidbody *mb = world->manifolds[m].object_b;
        world->manifold_awake[m] =
            (unsigned char) (islands_body_awake(world, ma) || islands_body_awake(world, mb));
    }
    collision_manifold_solve_order(world, world->manifolds, manifold_count, world->manifold_order);

    vector3 gravity = {0.0f, step_gravity, 0.0f};
    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];
        if ((rb->static_state) || (rb->is_sleeping) || (rb->kinematic)) {
            continue;
        }
        rb_apply_forces_perfect(rb, vector3_scaling(gravity, rb->mass));
    }

    /* Deterministic retention factors (never libm pow: see det_math.h). */
    float linear_damping = (float) det_pow_retention((double) step_drag, (double) dt);
    /* FIX-AUDIT: angular scale was hardcoded 0.97 (damped even at drag=1).
     * Now a real config param. */
    float angular_damping =
        (float) det_pow_retention((double) (step_drag * step_ang_scale), (double) dt);
    /* Springs before integration (weak-linked canonical entry). */
    if (mpe_springs_apply) {
        mpe_springs_apply(world, dt);
    }
    constraint_apply_motors(world, dt); /* MPE_FTC_067 */
    /* Phase-2: foreign forcefield / motor modules (pre-integration). */
    for (int mi = 0; mi < world->tick_module_count; mi++) {
        if (world->tick_modules[mi] && world->tick_modules[mi]->pre_step)
            world->tick_modules[mi]->pre_step(world, dt, world->tick_module_state[mi]);
    }
    for (int i = 0; i < world->body_count; i++) {
        rb_integrate_velocity(&world->bodies[i], dt, linear_damping, angular_damping);
    }

    /* Sleeping bodies keep real mass/inertia (no staticize mutation).
     * The solver treats them as infinite mass via rigidbody_effective_*
     * helpers, so observable state is never corrupted mid-tick and the
     * path is thread-safe. Zero stale velocities/accumulators only. */
    for (int sleep_index = 0; sleep_index < world->body_count; sleep_index++) {
        rigidbody *sleep_body = &world->bodies[sleep_index];
        if ((sleep_body->is_sleeping) && (!sleep_body->static_state)) {
            sleep_body->velocity = vector3_zero();
            sleep_body->angular_velocity = vector3_zero();
            sleep_body->force_accumulator = vector3_zero();
            sleep_body->torque_accumulator = vector3_zero();
        }
    }

    /* TRUTH P0-2: Poisson gate must see post-force-integration approach
     * speed (prepare ran pre-gravity, stale by g*dt). Refresh vn from
     * current velocities before any iteration touches accumulators. */
    collision_refresh_impact_velocities(world->manifolds, manifold_count);

    int solver_iterations = step_iterations;
    if (solver_iterations < 1) {
        solver_iterations = 1;
    }
    if (solver_iterations > 128) {
        solver_iterations = 128;
    }
    /* TRUTH: joint angle/slide integration once per tick (not per iteration). */
    constraint_pre_step_all(world, dt);
    for (int iter = 0; iter < solver_iterations; iter++) {
        for (int o = 0; o < manifold_count; o++) {
            int m = world->manifold_order[o];
            if (!world->manifold_awake[m]) {
                continue;
            }
            /* Local convergence: a manifold's contacts share bodies
             * (strong rotational coupling), so one visit rarely settles
             * them — the residue pollutes the next level up the stack.
             * A second immediate visit converges the local distribution
             * before propagating, buying back the margin deep stacks
             * need at low global iteration counts. Deterministic. */
            mpe_step_resolve(world, &world->manifolds[m], dt, false, iter, step_cfg);
            mpe_step_resolve(world, &world->manifolds[m], dt, false, iter + 1, step_cfg);
        }
        /* MFS_SOLVER_FIX: solve joints inside the iteration loop so friction
         * impulses properly transfer through revolute constraints to the chassis */
        constraint_solve_all(world, dt);
    }
    /* Positional axis-drift correction: exactly once per tick, never in the
     * loop above (see revolute_joint.c). */
    constraint_correct_axis_drift_all(world, dt);
    /* Poisson restitution: one e-over-compression payment per fresh impact,
     * then short relaxation so friction sees post-bounce velocities. */
    /* TRUTH: joints must see post-bounce velocities too. Poisson without a
     * joint relaxation leaves hinges/welds broken for a tick. */
    mpe_step_poisson(world, world->manifolds, manifold_count, step_cfg);
    constraint_solve_all(world, dt);
    for (int relax_iter = 0; relax_iter < 2; relax_iter++) {
        for (int o = 0; o < manifold_count; o++) {
            int m = world->manifold_order[o];
            if (!world->manifold_awake[m]) {
                continue;
            }
            mpe_step_resolve(world, &world->manifolds[m], dt, true, relax_iter, step_cfg);
        }
    }
    /* Split impulse: positional depenetration with zero velocity change.
     * The velocity solve above is compression-only, so contact impulses
     * (and the Coulomb clamp) stay honest. */
    mpe_step_split(world, world->manifolds, manifold_count, dt, step_cfg);
    /* TRUTH: split can wake sleepers (deep overlap) that were skipped as
     * both-asleep with no manifold. They would integrate with has_contact=0
     * (false free-flight). Re-process newly-awake skipped pairs now so they
     * get a manifold + has_contact before integration. */
    for (int p = 0; p < pair_count; p++) {
        if (!world->pair_skipped[p]) {
            continue;
        }
        int ia = world->pair_buffer[p].object_index_a;
        int ib = world->pair_buffer[p].object_index_b;
        if (ia < 0 || ia >= world->body_count || ib < 0 || ib >= world->body_count) {
            world->pair_skipped[p] = 0;
            continue;
        }
        if (world->bodies[ia].is_sleeping && world->bodies[ib].is_sleeping) {
            continue;
        }
        physics_world_process_pair(world, ia, ib, dt, &manifold_count);
        world->pair_skipped[p] = 0;
    }
    /* Rolling resistance once per tick (uses solved normal impulses). */
    mpe_step_rolling(world, world->manifolds, manifold_count, dt, step_cfg);

    /* Sleeping bodies already hold real mass (no staticize was applied),
     * so no restore is needed. Keep velocities pinned at zero. */
    for (int sleep_restore_index = 0; sleep_restore_index < world->body_count; sleep_restore_index++) {
        rigidbody *sleep_restore_body = &world->bodies[sleep_restore_index];
        if ((sleep_restore_body->is_sleeping) && (!sleep_restore_body->static_state)) {
            sleep_restore_body->velocity = vector3_zero();
            sleep_restore_body->angular_velocity = vector3_zero();
        }
    }

    contact_cache_save(world, world->manifolds, manifold_count);

    /* TRUTH P0-1+P0-3: integrate CCD remainder; gravity-exactness for
     * contact-free bodies ONLY (x += v*rem - 1/2*g*rem^2 is the exact
     * constant-force flow, hence exact parabolas; constrained bodies stay
     * symplectic Euler since contact impulses cancel gravity post-solve). */
    float grav_half = -0.5f * step_gravity;
    for (int i = 0; i < world->body_count; i++) {
        float step_dt = dt;
        if ((world->ccd_time_remaining) && (world->ccd_time_remaining[i] < step_dt) &&
            (world->ccd_time_remaining[i] > 0.0f)) {
            step_dt = world->ccd_time_remaining[i];
        }
        rb_integrate_position(&world->bodies[i], step_dt);
        rigidbody *ib = &world->bodies[i];
        bool free_flight = (world->has_contact) ? (world->has_contact[i] == 0) : true;
        if (free_flight && (!ib->static_state) && (!ib->is_sleeping) && (!ib->kinematic) && (step_dt > 0.0f)) {
            ib->position.y += grav_half * step_dt * step_dt;
        }
        rigidbody_sanitize(&world->bodies[i]);
    }

    /* World-edge safety net, same as the legacy path: perfectly plastic,
     * fires only past emergency slop (solver owns all normal contact). */
    bool a3_boundary_moved_any = false;
    for (int i = 0; i < world->body_count; i++) {
        vector3 a3_pre_boundary_position = world->bodies[i].position;
        boundary_apply_box(&world->bodies[i], (vector3){-250, 0, -250}, (vector3){250, 500, 250});
        if (vector3_length_squared(vector3_subtraction(world->bodies[i].position, a3_pre_boundary_position)) > 0.000001f) {
            a3_boundary_moved_any = true;
        }
    }

    /* Positional depenetration pass (like legacy path). */
    a3_positional_depenetration_pass_dt(world, world->pair_buffer, &broadphase_pair_count, a3_boundary_moved_any,
                                        dt);
    /* Phase-2: foreign post-step modules (loggers, correctors). */
    for (int mi = 0; mi < world->tick_module_count; mi++) {
        if (world->tick_modules[mi] && world->tick_modules[mi]->post_step)
            world->tick_modules[mi]->post_step(world, dt, world->tick_module_state[mi]);
    }
    /* AUDIT: the warm-start cache is deliberately NOT cleared here. Cached
     * contact points are stored in BODY-LOCAL space, which rigid
     * translation (the only thing depenetration/boundary do) preserves
     * exactly — a persistent contact matches the same material point next
     * tick regardless of where the body moved. An earlier revision cleared
     * unconditionally ("stale positions"), which silently disabled warm
     * starting engine-wide (F9 showed hits=0 forever): every tick solved
     * cold, tall stacks crept and collapsed at low iteration counts.
     * Rotation-driven contact migration still misses naturally via the
     * match distance, which is the correct invalidation path. Scene loads
     * (pool invalidation) keep their explicit clears. */
}

/* R3-07: Containment walls.
 *
 * Adds four static cube bodies around the playable area.
 * The walls are placed just outside the half-extents so the
 * playable interior is exactly half_width x half_depth.
 *
 * Wall layout (top view):
 *
 *        north wall
 *   +-----------------+
 *   |                 |
 * w |    playable     | e
 * e |     area        | a
 * s |                 | s
 * t |                 | t
 *   +-----------------+
 *        south wall
 */
int physics_world_add_boundary_walls(physics_world *world,
                                     float half_width,
                                     float half_depth,
                                     float wall_height,
                                     float wall_thickness)
{
    if (!world) {
        return -1;
    }
    if ((half_width <= 0.0f) || (half_depth <= 0.0f) ||
        (wall_height <= 0.0f) || (wall_thickness <= 0.0f)) {
        return -1;
    }

    float hy = wall_height * 0.5f;
    float ht = wall_thickness * 0.5f;

    /* North wall: +Z side */
    int north = physics_world_add_cube(world,
        (vector3){0.0f, hy, half_depth + ht},
        (vector3){half_width + wall_thickness, hy, ht},
        0.0f);

    /* South wall: -Z side */
    int south = physics_world_add_cube(world,
        (vector3){0.0f, hy, -(half_depth + ht)},
        (vector3){half_width + wall_thickness, hy, ht},
        0.0f);

    /* East wall: +X side */
    int east = physics_world_add_cube(world,
        (vector3){half_width + ht, hy, 0.0f},
        (vector3){ht, hy, half_depth + wall_thickness},
        0.0f);

    /* West wall: -X side */
    int west = physics_world_add_cube(world,
        (vector3){-(half_width + ht), hy, 0.0f},
        (vector3){ht, hy, half_depth + wall_thickness},
        0.0f);

    if ((north < 0) || (south < 0) || (east < 0) || (west < 0)) {
        return -1;
    }

    return 0;
}

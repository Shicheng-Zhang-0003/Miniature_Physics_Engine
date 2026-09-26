#include "mpe_registry.h"
#include "../core/rigidbody.h"
#include "../core/physics_world.h"
#include "../physics/broadphase.h"
#include "../physics/collision_mechanics.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>

/* Process-global registry lock: every mutation and every lookup takes it.
 * Dispatch calls find per pair; uncontended mutex cost (~20 ns) is noise
 * next to narrowphase. No lock ordering issues: this is the only
 * registry-side lock; callers must never hold world locks across it. */
static pthread_mutex_t s_reg_lock = PTHREAD_MUTEX_INITIALIZER;

void mpe_registry_lock(void) {
    pthread_mutex_lock(&s_reg_lock);
}

void mpe_registry_unlock(void) {
    pthread_mutex_unlock(&s_reg_lock);
}

static mpe_pair_entry_t s_pairs[MPE_MAX_PAIR_HANDLERS];
static int s_pair_count = 0;
/* Indices [0, s_builtin_pairs) are engine builtins, pinned after the once
 * init below. Overwriting them with foreign code is refused (silent
 * builtin hijack used to be possible with one register call). */
static int s_builtin_pairs = 0;

/* Tombstoned slots: unregister marks dead instead of memmove, so
 * interior pointers already handed out (world slots, find results in
 * flight) never shift onto a different entry. Dead slots are reused by
 * later registers. Purge (forget/detach in all live worlds) still runs
 * on every unregister so nothing calls into unloaded code. */
static struct { char name[64]; mpe_broadphase_if_t iface; int live; } s_broad[8];
static int s_broad_count = 0;

static struct { char name[64]; mpe_solver_if_t iface; int live; } s_solvers[8];
static int s_solver_count = 0;

static mpe_module_desc_t s_modules[MPE_MAX_MODULES];
static char s_module_names[MPE_MAX_MODULES][64];
static char s_module_versions[MPE_MAX_MODULES][32];
static char s_module_kinds[MPE_MAX_MODULES][32];
/* Provenance: plugin .so path that registered the slot, "" for static or
 * direct registrations. Lets the loader tell a static desc and a .so
 * desc of the SAME module name apart (hotload pattern). */
static char s_module_origin[MPE_MAX_MODULES][256];
static int s_module_live[MPE_MAX_MODULES];
static int s_module_count = 0;

#define MPE_BUILTIN_BROADPHASE "hash"
#define MPE_BUILTIN_SOLVER "seq-impulse"

static int name_fits(const char *name, size_t cap) {
    return name && strlen(name) < cap;
}

int mpe_register_pair_handler(int ta, int tb, int ca, int cb, mpe_collide_fn fn, const char *name) {
    if (!fn) return -1;
    if (!name_fits(name ? name : "?", sizeof(s_pairs[0].name))) return -1;
    pthread_mutex_lock(&s_reg_lock);
    int rc = -1;
    for (int i = 0; i < s_pair_count; i++) {
        if (s_pairs[i].type_a == ta && s_pairs[i].type_b == tb &&
            s_pairs[i].custom_a == ca && s_pairs[i].custom_b == cb) {
            if (i < s_builtin_pairs && s_pairs[i].fn != fn) {
                rc = -1; /* builtin slot: refuse silent hijack */
            } else {
                s_pairs[i].fn = fn;
                snprintf(s_pairs[i].name, sizeof(s_pairs[i].name), "%s", name ? name : "?");
                rc = i;
            }
            pthread_mutex_unlock(&s_reg_lock);
            return rc;
        }
    }
    if (s_pair_count >= MPE_MAX_PAIR_HANDLERS) {
        /* FIX-AUDIT-DESPOT: slot-full was a silent -1 (plugin load looked
         * like success downstream). Log so `mod` failures are diagnosable. */
        fprintf(stderr, "[registry] pair table full (%d); refusing '%s'\n",
                MPE_MAX_PAIR_HANDLERS, name ? name : "?");
        pthread_mutex_unlock(&s_reg_lock);
        return -1;
    }
    s_pairs[s_pair_count].type_a = ta;
    s_pairs[s_pair_count].type_b = tb;
    s_pairs[s_pair_count].custom_a = ca;
    s_pairs[s_pair_count].custom_b = cb;
    s_pairs[s_pair_count].fn = fn;
    snprintf(s_pairs[s_pair_count].name, sizeof(s_pairs[s_pair_count].name), "%s", name ? name : "?");
    rc = s_pair_count++;
    pthread_mutex_unlock(&s_reg_lock);
    return rc;
}

int mpe_unregister_pair_handler(mpe_collide_fn fn) {
    if (!fn) return -1;
    /* FIX-AUDIT-DESPOT: missing lock added — every other registry mutation
     * and lookup takes s_reg_lock; the unlocked memmove raced concurrent
     * dispatch (find) and loader purge walks.
     * Tombstones deliberately NOT used here (unlike broadphase/solver/
     * modules): worlds store the pair fn BY VALUE — dispatch copies it per
     * pair per tick via mpe_find_pair_handler (see physics_world_process_pair)
     * and never retains &s_pairs[i] — so memmove under lock cannot dangle a
     * world. Loader purge snapshots fns via mpe_registry_pair_fn_at (also
     * under lock) before calling back in here. */
    pthread_mutex_lock(&s_reg_lock);
    /* By-value invariant: no live world may alias &s_pairs[i]. Pair dispatch
     * is lookup-per-pair (value copy); broadphase/solver/module slots ARE
     * aliased by worlds and therefore use tombstones instead. */
    assert(s_pair_count >= 0 && s_pair_count <= MPE_MAX_PAIR_HANDLERS);
    int removed = 0;
    for (int i = 0; i < s_pair_count;) {
        if (s_pairs[i].fn == fn) {
            /* Pinned builtins [0, s_builtin_pairs) refuse foreign overwrite
             * at register time; explicit unregister of a builtin fn is still
             * honored here (loader purge only ever passes plugin-range fns,
             * verified by fn_in_plugin, so builtins cannot be purged). */
            for (int j = i; j + 1 < s_pair_count; j++) s_pairs[j] = s_pairs[j + 1];
            s_pair_count--; removed++;
        } else i++;
    }
    pthread_mutex_unlock(&s_reg_lock);
    return removed ? 0 : -1;
}

static int match_score(const mpe_pair_entry_t *e, int ta, int tb, int ca, int cb) {
    int s = 0;
    if (e->type_a == ta) s += 4; else if (e->type_a != -1) return -1;
    if (e->type_b == tb) s += 4; else if (e->type_b != -1) return -1;
    if (e->custom_a == ca) s += 2; else if (e->custom_a != -1) return -1;
    if (e->custom_b == cb) s += 2; else if (e->custom_b != -1) return -1;
    return s;
}

/* Human-readable pair-table dump for `mod ls` (types + owned name). */
int mpe_registry_pair_describe(int idx, int *ta, int *tb, int *ca, int *cb, char *name, int namelen) {
    if (idx < 0) return -1;
    pthread_mutex_lock(&s_reg_lock);
    int rc = -1;
    if (idx < s_pair_count) {
        if (ta) *ta = s_pairs[idx].type_a;
        if (tb) *tb = s_pairs[idx].type_b;
        if (ca) *ca = s_pairs[idx].custom_a;
        if (cb) *cb = s_pairs[idx].custom_b;
        if (name && namelen > 0) {
            snprintf(name, (size_t)namelen, "%s", s_pairs[idx].name);
        }
        rc = 0;
    }
    pthread_mutex_unlock(&s_reg_lock);
    return rc;
}

/* Indexed snapshot for loader purge walks (skips tombstone-free table:
 * pairs have no tombstones; memmove under lock is safe). */
int mpe_registry_pair_fn_at(int idx, mpe_collide_fn *out) {
    if (!out) return -1;
    pthread_mutex_lock(&s_reg_lock);
    int rc = -1;
    if (idx >= 0 && idx < s_pair_count) {
        *out = s_pairs[idx].fn;
        rc = 0;
    }
    pthread_mutex_unlock(&s_reg_lock);
    return rc;
}

mpe_collide_fn mpe_find_pair_handler(int ta, int tb, int ca, int cb) {
    pthread_mutex_lock(&s_reg_lock);
    int best = -1, best_score = -1;
    for (int i = 0; i < s_pair_count; i++) {
        int sc = match_score(&s_pairs[i], ta, tb, ca, cb);
        if (sc > best_score) { best_score = sc; best = i; }
    }
    mpe_collide_fn fn = (best < 0) ? 0 : s_pairs[best].fn;
    pthread_mutex_unlock(&s_reg_lock);
    return fn;
}

/* Rollback support for loader failure paths: counts are only ever
 * truncated back to a snapshot taken (under lock) before dlopen. */
int mpe_registry_pair_count(void) {
    pthread_mutex_lock(&s_reg_lock);
    int n = s_pair_count;
    pthread_mutex_unlock(&s_reg_lock);
    return n;
}

void mpe_registry_truncate_pairs(int keep) {
    pthread_mutex_lock(&s_reg_lock);
    if (keep < s_builtin_pairs) keep = s_builtin_pairs;
    if (keep < s_pair_count) s_pair_count = keep;
    pthread_mutex_unlock(&s_reg_lock);
}

int mpe_registry_broadphase_count(void) {
    pthread_mutex_lock(&s_reg_lock);
    int n = s_broad_count;
    pthread_mutex_unlock(&s_reg_lock);
    return n;
}

void mpe_registry_truncate_broadphase(int keep) {
    pthread_mutex_lock(&s_reg_lock);
    if (keep < 1) keep = 1; /* builtin "hash" is index 0, pinned */
    if (keep < s_broad_count) s_broad_count = keep;
    pthread_mutex_unlock(&s_reg_lock);
}

int mpe_registry_solver_count(void) {
    pthread_mutex_lock(&s_reg_lock);
    int n = s_solver_count;
    pthread_mutex_unlock(&s_reg_lock);
    return n;
}

void mpe_registry_truncate_solvers(int keep) {
    pthread_mutex_lock(&s_reg_lock);
    if (keep < 1) keep = 1; /* builtin "seq-impulse" is index 0, pinned */
    if (keep < s_solver_count) s_solver_count = keep;
    pthread_mutex_unlock(&s_reg_lock);
}

static int iface_equal_broad(const mpe_broadphase_if_t *a, const mpe_broadphase_if_t *b) {
    return memcmp(a, b, sizeof(*a)) == 0;
}

static int iface_equal_solver(const mpe_solver_if_t *a, const mpe_solver_if_t *b) {
    return memcmp(a, b, sizeof(*a)) == 0;
}

static int is_builtin_stage(const char *name) {
    return strcmp(name, MPE_BUILTIN_BROADPHASE) == 0 || strcmp(name, MPE_BUILTIN_SOLVER) == 0;
}

int mpe_register_broadphase(const char *name, const mpe_broadphase_if_t *iface) {
    if (!name || !iface) return -1;
    if (!name_fits(name, sizeof(s_broad[0].name))) return -1;
    pthread_mutex_lock(&s_reg_lock);
    int rc = -1;
    for (int i = 0; i < s_broad_count; i++) {
        if (s_broad[i].live && strcmp(s_broad[i].name, name) == 0) {
            if (is_builtin_stage(name) && !iface_equal_broad(&s_broad[i].iface, iface)) {
                rc = -1; /* refuse builtin takeover */
            } else {
                s_broad[i].iface = *iface;
                rc = i;
            }
            pthread_mutex_unlock(&s_reg_lock);
            return rc;
        }
    }
    /* Reuse a tombstoned slot before growing. */
    for (int i = 0; i < s_broad_count; i++) {
        if (!s_broad[i].live) {
            snprintf(s_broad[i].name, sizeof(s_broad[i].name), "%s", name);
            s_broad[i].iface = *iface;
            s_broad[i].live = 1;
            pthread_mutex_unlock(&s_reg_lock);
            return i;
        }
    }
    if (s_broad_count >= 8) {
        /* FIX-AUDIT-DESPOT: silent slot-full -> stderr diagnostic. */
        fprintf(stderr, "[registry] broadphase table full (8); refusing '%s'\n",
                name ? name : "?");
        pthread_mutex_unlock(&s_reg_lock);
        return -1;
    }
    snprintf(s_broad[s_broad_count].name, sizeof(s_broad[s_broad_count].name), "%s", name);
    s_broad[s_broad_count].iface = *iface;
    s_broad[s_broad_count].live = 1;
    rc = s_broad_count++;
    pthread_mutex_unlock(&s_reg_lock);
    return rc;
}

int mpe_unregister_broadphase(const char *name) {
    if (!name) return -1;
    pthread_mutex_lock(&s_reg_lock);
    int rc = -1;
    const mpe_broadphase_if_t *deadp = NULL;
    for (int i = 0; i < s_broad_count; i++) {
        if (s_broad[i].live && strcmp(s_broad[i].name, name) == 0) {
            s_broad[i].live = 0;
            deadp = &s_broad[i].iface;
            rc = 0;
            break;
        }
    }
    pthread_mutex_unlock(&s_reg_lock);
    /* Worlds aliasing the interior pointer must reset to builtin (NULL)
     * before the caller can dlclose. Slot memory stays put (tombstone),
     * so no other world's pointer shifts meaning. */
    if (deadp) physics_world_forget_stage_pointers(deadp, NULL);
    return rc;
}

int mpe_register_solver(const char *name, const mpe_solver_if_t *iface) {
    if (!name || !iface) return -1;
    if (!name_fits(name, sizeof(s_solvers[0].name))) return -1;
    pthread_mutex_lock(&s_reg_lock);
    int rc = -1;
    for (int i = 0; i < s_solver_count; i++) {
        if (s_solvers[i].live && strcmp(s_solvers[i].name, name) == 0) {
            if (is_builtin_stage(name) && !iface_equal_solver(&s_solvers[i].iface, iface)) {
                rc = -1; /* refuse builtin takeover */
            } else {
                s_solvers[i].iface = *iface;
                rc = i;
            }
            pthread_mutex_unlock(&s_reg_lock);
            return rc;
        }
    }
    for (int i = 0; i < s_solver_count; i++) {
        if (!s_solvers[i].live) {
            snprintf(s_solvers[i].name, sizeof(s_solvers[i].name), "%s", name);
            s_solvers[i].iface = *iface;
            s_solvers[i].live = 1;
            pthread_mutex_unlock(&s_reg_lock);
            return i;
        }
    }
    if (s_solver_count >= 8) {
        /* FIX-AUDIT-DESPOT: silent slot-full -> stderr diagnostic. */
        fprintf(stderr, "[registry] solver table full (8); refusing '%s'\n",
                name ? name : "?");
        pthread_mutex_unlock(&s_reg_lock);
        return -1;
    }
    snprintf(s_solvers[s_solver_count].name, sizeof(s_solvers[s_solver_count].name), "%s", name);
    s_solvers[s_solver_count].iface = *iface;
    s_solvers[s_solver_count].live = 1;
    rc = s_solver_count++;
    pthread_mutex_unlock(&s_reg_lock);
    return rc;
}

int mpe_unregister_solver(const char *name) {
    if (!name) return -1;
    pthread_mutex_lock(&s_reg_lock);
    int rc = -1;
    const mpe_solver_if_t *deadp = NULL;
    for (int i = 0; i < s_solver_count; i++) {
        if (s_solvers[i].live && strcmp(s_solvers[i].name, name) == 0) {
            s_solvers[i].live = 0;
            deadp = &s_solvers[i].iface;
            rc = 0;
            break;
        }
    }
    pthread_mutex_unlock(&s_reg_lock);
    if (deadp) physics_world_forget_stage_pointers(NULL, deadp);
    return rc;
}

const mpe_broadphase_if_t *mpe_find_broadphase(const char *name) {
    if (!name) return 0;
    pthread_mutex_lock(&s_reg_lock);
    const mpe_broadphase_if_t *out = 0;
    for (int i = 0; i < s_broad_count; i++)
        if (s_broad[i].live && strcmp(s_broad[i].name, name) == 0) { out = &s_broad[i].iface; break; }
    pthread_mutex_unlock(&s_reg_lock);
    return out;
}

const mpe_solver_if_t *mpe_find_solver(const char *name) {
    if (!name) return 0;
    pthread_mutex_lock(&s_reg_lock);
    const mpe_solver_if_t *out = 0;
    for (int i = 0; i < s_solver_count; i++)
        if (s_solvers[i].live && strcmp(s_solvers[i].name, name) == 0) { out = &s_solvers[i].iface; break; }
    pthread_mutex_unlock(&s_reg_lock);
    return out;
}

static void module_store(int i, const mpe_module_desc_t *desc, int keep_origin) {
    char origin_keep[256];
    snprintf(origin_keep, sizeof(origin_keep), "%s", keep_origin ? s_module_origin[i] : "");
    s_modules[i] = *desc;
    snprintf(s_module_names[i], sizeof(s_module_names[i]), "%s", desc->name);
    snprintf(s_module_versions[i], sizeof(s_module_versions[i]), "%s",
             desc->version ? desc->version : "?");
    snprintf(s_module_kinds[i], sizeof(s_module_kinds[i]), "%s", desc->kind ? desc->kind : "?");
    s_modules[i].name = s_module_names[i];
    s_modules[i].version = s_module_versions[i];
    s_modules[i].kind = s_module_kinds[i];
    snprintf(s_module_origin[i], sizeof(s_module_origin[i]), "%s", origin_keep);
    s_module_live[i] = 1;
}

/* Loader-registered provenance (call right after mpe_register_module). */
int mpe_registry_set_origin(const char *name, const char *path) {
    if (!name || !path) return -1;
    pthread_mutex_lock(&s_reg_lock);
    int rc = -1;
    for (int i = 0; i < s_module_count; i++) {
        if (s_module_live[i] && strcmp(s_module_names[i], name) == 0) {
            snprintf(s_module_origin[i], sizeof(s_module_origin[i]), "%s", path);
            rc = 0;
            break;
        }
    }
    pthread_mutex_unlock(&s_reg_lock);
    return rc;
}

/* Origin of the slot this desc pointer aliases, or NULL (static desc,
 * unknown pointer, or dead slot). */
const char *mpe_registry_module_origin(const mpe_module_desc_t *desc) {
    if (!desc) return NULL;
    pthread_mutex_lock(&s_reg_lock);
    const char *out = NULL;
    for (int i = 0; i < s_module_count; i++) {
        if (s_module_live[i] && desc == &s_modules[i]) {
            out = s_module_origin[i][0] ? s_module_origin[i] : NULL;
            break;
        }
    }
    pthread_mutex_unlock(&s_reg_lock);
    return out;
}

int mpe_register_module(const mpe_module_desc_t *desc) {
    if (!desc || desc->abi != MPE_MODULE_ABI || !desc->name) return -1;
    if (!name_fits(desc->name, sizeof(s_module_names[0]))) return -1;
    pthread_mutex_lock(&s_reg_lock);
    int rc = -1;
    for (int i = 0; i < s_module_count; i++) {
        if (s_module_live[i] && strcmp(s_module_names[i], desc->name) == 0) {
            module_store(i, desc, 1);
            rc = i;
            pthread_mutex_unlock(&s_reg_lock);
            return rc;
        }
    }
    for (int i = 0; i < s_module_count; i++) {
        if (!s_module_live[i]) {
            module_store(i, desc, 0);
            pthread_mutex_unlock(&s_reg_lock);
            return i;
        }
    }
    if (s_module_count >= MPE_MAX_MODULES) {
        /* FIX-AUDIT-DESPOT: silent slot-full -> stderr diagnostic. */
        fprintf(stderr, "[registry] module table full (%d); refusing '%s'\n",
                MPE_MAX_MODULES, desc && desc->name ? desc->name : "?");
        pthread_mutex_unlock(&s_reg_lock);
        return -1;
    }
    module_store(s_module_count, desc, 0);
    rc = s_module_count++;
    pthread_mutex_unlock(&s_reg_lock);
    return rc;
}

int mpe_unregister_module(const char *name) {
    if (!name) return -1;
    pthread_mutex_lock(&s_reg_lock);
    int rc = -1;
    for (int i = 0; i < s_module_count; i++) {
        if (s_module_live[i] && strcmp(s_module_names[i], name) == 0) {
            s_module_live[i] = 0;
            rc = 0;
            break;
        }
    }
    pthread_mutex_unlock(&s_reg_lock);
    /* Attached worlds hold &s_modules[i]: tombstone keeps the address
     * stable; detach everywhere so no world calls a dead module. */
    if (rc == 0) physics_world_detach_module_everywhere(name);
    return rc;
}

/* Origin-scoped removal for loader unload: tombstones only the slot
 * whose origin matches (a static same-named desc is left alone), with NO
 * world-detach side effect (the loader detaches targeted refs first). */
int mpe_unregister_module_origin(const char *name, const char *path) {
    if (!name || !path) return -1;
    pthread_mutex_lock(&s_reg_lock);
    int rc = -1;
    for (int i = 0; i < s_module_count; i++) {
        if (s_module_live[i] && strcmp(s_module_names[i], name) == 0 &&
            strcmp(s_module_origin[i], path) == 0) {
            s_module_live[i] = 0;
            rc = 0;
            break;
        }
    }
    pthread_mutex_unlock(&s_reg_lock);
    return rc;
}

int mpe_module_count(void) {
    pthread_mutex_lock(&s_reg_lock);
    int n = 0;
    for (int i = 0; i < s_module_count; i++) n += s_module_live[i] ? 1 : 0;
    pthread_mutex_unlock(&s_reg_lock);
    return n;
}

const mpe_module_desc_t *mpe_module_at(int i) {
    pthread_mutex_lock(&s_reg_lock);
    const mpe_module_desc_t *out = 0;
    int seen = -1;
    for (int k = 0; k < s_module_count; k++) {
        if (!s_module_live[k]) continue;
        if (++seen == i) { out = &s_modules[k]; break; }
    }
    pthread_mutex_unlock(&s_reg_lock);
    return out;
}

const mpe_module_desc_t *mpe_find_module(const char *name) {
    if (!name) return 0;
    pthread_mutex_lock(&s_reg_lock);
    const mpe_module_desc_t *out = 0;
    for (int i = 0; i < s_module_count; i++)
        if (s_module_live[i] && strcmp(s_modules[i].name, name) == 0) { out = &s_modules[i]; break; }
    pthread_mutex_unlock(&s_reg_lock);
    return out;
}

/* Built-in pair handlers forward the world's config snapshot. */
static const mpe_config_t *wrap_cfg(mpe_world_t *w) {
    return (w && w->cfg) ? w->cfg : &g_cfg;
}
static bool wrap_ss(rigidbody *a, rigidbody *b, void *out, mpe_world_t *w) {
    return collision_dual_sphere(a, b, (collision_data *)out, wrap_cfg(w));
}
static bool wrap_sc(rigidbody *a, rigidbody *b, void *out, mpe_world_t *w) {
    return collision_sphere_cube(a, b, (collision_data *)out, wrap_cfg(w));
}
static bool wrap_cc(rigidbody *a, rigidbody *b, void *out, mpe_world_t *w) {
    return collision_dual_cube(a, b, (collision_data *)out, wrap_cfg(w));
}
static bool wrap_cyl_s(rigidbody *a, rigidbody *b, void *out, mpe_world_t *w) {
    return collision_cylinder_sphere(a, b, (collision_data *)out, wrap_cfg(w));
}
static bool wrap_cyl_c(rigidbody *a, rigidbody *b, void *out, mpe_world_t *w) {
    return collision_cylinder_cube(a, b, (collision_data *)out, wrap_cfg(w));
}
static bool wrap_cyl_cyl(rigidbody *a, rigidbody *b, void *out, mpe_world_t *w) {
    return collision_cylinder_cylinder(a, b, (collision_data *)out, wrap_cfg(w));
}

/* Builtin stage backends: exact current behaviour, registered under
 * canonical names so `mod use-broadphase hash` / `mod use-solver
 * seq-impulse` round-trips to the defaults. */
static int builtin_broadphase_generate(mpe_world_t *world, broadphase_pair *pairs_out, int max_pairs,
                                       float dt, void *mod_state) {
    (void) mod_state;
    return broadphase_generate_pairing(world, pairs_out, max_pairs, dt);
}
static float builtin_solver_resolve(mpe_world_t *world, void *manifold, float dt, bool friction_only, int iter,
                                    void *mod_state) {
    (void) mod_state;
    return collision_resolve_iterative((collision_data *)manifold, dt, friction_only, iter, wrap_cfg(world));
}
static void builtin_solver_poisson(mpe_world_t *world, void *manifolds, int n, void *mod_state) {
    (void) mod_state;
    collision_apply_poisson_restitution((collision_data *)manifolds, n, wrap_cfg(world));
}
static void builtin_solver_rolling(mpe_world_t *world, void *manifolds, int n, float dt, void *mod_state) {
    (void) mod_state;
    collision_apply_rolling_resistance((collision_data *)manifolds, n, dt, wrap_cfg(world));
}
static void builtin_solver_split(mpe_world_t *world, void *manifolds, int n, float dt, void *mod_state) {
    (void) mod_state;
    collision_apply_split_impulse((collision_data *)manifolds, n, dt, wrap_cfg(world));
}
static const mpe_broadphase_if_t s_builtin_broadphase = {builtin_broadphase_generate};
static const mpe_solver_if_t s_builtin_solver = {builtin_solver_resolve, builtin_solver_poisson,
                                                 builtin_solver_rolling, builtin_solver_split};

static pthread_once_t s_builtins_once = PTHREAD_ONCE_INIT;

static void mpe_register_builtins_once(void) {
    /* object_sphere=0, object_cube=1, object_cylinder=2 (see rigidbody.h) */
    mpe_register_pair_handler(0, 0, -1, -1, wrap_ss, "sphere-sphere");
    mpe_register_pair_handler(0, 1, -1, -1, wrap_sc, "sphere-cube");
    mpe_register_pair_handler(1, 1, -1, -1, wrap_cc, "cube-cube");
    mpe_register_pair_handler(2, 0, -1, -1, wrap_cyl_s, "cyl-sphere");
    mpe_register_pair_handler(2, 1, -1, -1, wrap_cyl_c, "cyl-cube");
    mpe_register_pair_handler(2, 2, -1, -1, wrap_cyl_cyl, "cyl-cyl");
    mpe_register_broadphase("hash", &s_builtin_broadphase);
    mpe_register_solver("seq-impulse", &s_builtin_solver);
    /* Pin the builtin pair slots: indices below this count refuse
     * foreign overwrite (hijack guard in mpe_register_pair_handler). */
    pthread_mutex_lock(&s_reg_lock);
    s_builtin_pairs = s_pair_count;
    pthread_mutex_unlock(&s_reg_lock);
}

void mpe_register_builtins(void) {
    pthread_once(&s_builtins_once, mpe_register_builtins_once);
}

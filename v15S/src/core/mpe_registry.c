#include "mpe_registry.h"
#include "../core/rigidbody.h"
#include "../core/physics_world.h"
#include "../physics/broadphase.h"
#include "../physics/collision_mechanics.h"
#include <string.h>

static mpe_pair_entry_t s_pairs[MPE_MAX_PAIR_HANDLERS];
static int s_pair_count = 0;

static struct { const char *name; mpe_broadphase_if_t iface; } s_broad[8];
static int s_broad_count = 0;
static struct { const char *name; mpe_solver_if_t iface; } s_solvers[8];
static int s_solver_count = 0;

static mpe_module_desc_t s_modules[MPE_MAX_MODULES];
static int s_module_count = 0;

int mpe_register_pair_handler(int ta, int tb, int ca, int cb, mpe_collide_fn fn, const char *name) {
    if (!fn) return -1;
    for (int i = 0; i < s_pair_count; i++) {
        if (s_pairs[i].type_a == ta && s_pairs[i].type_b == tb &&
            s_pairs[i].custom_a == ca && s_pairs[i].custom_b == cb) {
            s_pairs[i].fn = fn; s_pairs[i].name = name;
            return i;
        }
    }
    if (s_pair_count >= MPE_MAX_PAIR_HANDLERS) return -1;
    s_pairs[s_pair_count] = (mpe_pair_entry_t){ta, tb, ca, cb, fn, name};
    return s_pair_count++;
}

static int match_score(const mpe_pair_entry_t *e, int ta, int tb, int ca, int cb) {
    int s = 0;
    if (e->type_a == ta) s += 4; else if (e->type_a != -1) return -1;
    if (e->type_b == tb) s += 4; else if (e->type_b != -1) return -1;
    if (e->custom_a == ca) s += 2; else if (e->custom_a != -1) return -1;
    if (e->custom_b == cb) s += 2; else if (e->custom_b != -1) return -1;
    return s;
}

mpe_collide_fn mpe_find_pair_handler(int ta, int tb, int ca, int cb) {
    int best = -1, best_score = -1;
    for (int i = 0; i < s_pair_count; i++) {
        int sc = match_score(&s_pairs[i], ta, tb, ca, cb);
        if (sc > best_score) { best_score = sc; best = i; }
    }
    if (best < 0) return 0;
    return s_pairs[best].fn;
}

void mpe_register_broadphase(const char *name, const mpe_broadphase_if_t *iface) {
    if (!name || !iface || s_broad_count >= 8) return;
    for (int i = 0; i < s_broad_count; i++)
        if (strcmp(s_broad[i].name, name) == 0) { s_broad[i].iface = *iface; return; }
    s_broad[s_broad_count].name = name;
    s_broad[s_broad_count].iface = *iface;
    s_broad_count++;
}

void mpe_register_solver(const char *name, const mpe_solver_if_t *iface) {
    if (!name || !iface || s_solver_count >= 8) return;
    for (int i = 0; i < s_solver_count; i++)
        if (strcmp(s_solvers[i].name, name) == 0) { s_solvers[i].iface = *iface; return; }
    s_solvers[s_solver_count].name = name;
    s_solvers[s_solver_count].iface = *iface;
    s_solver_count++;
}

const mpe_broadphase_if_t *mpe_find_broadphase(const char *name) {
    if (!name) return 0;
    for (int i = 0; i < s_broad_count; i++)
        if (strcmp(s_broad[i].name, name) == 0) return &s_broad[i].iface;
    return 0;
}

const mpe_solver_if_t *mpe_find_solver(const char *name) {
    if (!name) return 0;
    for (int i = 0; i < s_solver_count; i++)
        if (strcmp(s_solvers[i].name, name) == 0) return &s_solvers[i].iface;
    return 0;
}

int mpe_register_module(const mpe_module_desc_t *desc) {
    if (!desc || desc->abi != MPE_MODULE_ABI || !desc->name) return -1;
    for (int i = 0; i < s_module_count; i++)
        if (strcmp(s_modules[i].name, desc->name) == 0) { s_modules[i] = *desc; return i; }
    if (s_module_count >= MPE_MAX_MODULES) return -1;
    s_modules[s_module_count] = *desc;
    return s_module_count++;
}

int mpe_module_count(void) { return s_module_count; }
const mpe_module_desc_t *mpe_module_at(int i) {
    if (i < 0 || i >= s_module_count) return 0;
    return &s_modules[i];
}
const mpe_module_desc_t *mpe_find_module(const char *name) {
    if (!name) return 0;
    for (int i = 0; i < s_module_count; i++)
        if (strcmp(s_modules[i].name, name) == 0) return &s_modules[i];
    return 0;
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

static int s_builtins_done = 0;
void mpe_register_builtins(void) {
    if (s_builtins_done) return;
    s_builtins_done = 1;
    /* object_sphere=0, object_cube=1, object_cylinder=2 (see rigidbody.h) */
    mpe_register_pair_handler(0, 0, -1, -1, wrap_ss, "sphere-sphere");
    mpe_register_pair_handler(0, 1, -1, -1, wrap_sc, "sphere-cube");
    mpe_register_pair_handler(1, 1, -1, -1, wrap_cc, "cube-cube");
    mpe_register_pair_handler(2, 0, -1, -1, wrap_cyl_s, "cyl-sphere");
    mpe_register_pair_handler(2, 1, -1, -1, wrap_cyl_c, "cyl-cube");
    mpe_register_pair_handler(2, 2, -1, -1, wrap_cyl_cyl, "cyl-cyl");
    mpe_register_broadphase("hash", &s_builtin_broadphase);
    mpe_register_solver("seq-impulse", &s_builtin_solver);
}

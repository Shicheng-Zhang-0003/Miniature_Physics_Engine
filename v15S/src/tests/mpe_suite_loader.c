/* MPE Suite v2 — loader/registry lifecycle regression test.
 *
 * Proves the MPI lifetime fixes end to end with a REAL .so
 * (plugins/mpe_capsule.so; needs CWD=v15S/src for confinement —
 * skips gracefully elsewhere):
 *   load -> attach (busy: unload refused -2) -> detach -> unload ok ->
 *   pair handlers purged, module gone.
 * Plus static-only checks (no .so needed): builtin hijack refusal,
 * over-long names, stage reset-to-builtin on unregister, mod_state
 * threading through foreign solver hooks.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "mpe_test.h"
#include "core/mpe_registry.h"
#include "core/mpe_loader.h"

static int saw_state = 0;
static void *saw_ptr = NULL;

static float rec_resolve(mpe_world_t *world, void *manifold, float dt, bool friction_only, int iter,
                         void *mod_state) {
    (void)world;
    (void)manifold;
    (void)dt;
    (void)friction_only;
    (void)iter;
    saw_state = 1;
    saw_ptr = mod_state;
    return 0.0f;
}

static void *saw_bp_state = NULL;

static int fake_generate(mpe_world_t *world, broadphase_pair *pairs_out, int max_pairs, float dt,
                         void *mod_state) {
    saw_bp_state = mod_state;
    /* Delegate to the real backend so manifolds exist for resolve. */
    extern int broadphase_generate_pairing(mpe_world_t *world, broadphase_pair *out, int max, float dt);
    return broadphase_generate_pairing(world, pairs_out, max_pairs, dt);
}

static bool fake_sphere(rigidbody *a, rigidbody *b, void *out, mpe_world_t *w) {
    (void)a;
    (void)b;
    (void)out;
    (void)w;
    return false;
}

int mpe_t_loader_lifecycle(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "loader_lifecycle");
    mpe_config_init();
    mpe_register_builtins();

    /* ---- static-only: builtin hijack refusal + name length ---- */
    {
        mpe_broadphase_if_t evil_bp = {fake_generate};
        MPE_CHECK(&t, mpe_register_broadphase("hash", &evil_bp) < 0);
        mpe_solver_if_t evil_sv = {rec_resolve, NULL, NULL, NULL};
        MPE_CHECK(&t, mpe_register_solver("seq-impulse", &evil_sv) < 0);
        MPE_CHECK(&t, mpe_register_pair_handler(0, 0, -1, -1, fake_sphere, "evil") < 0);
        char longname[128];
        memset(longname, 'x', sizeof(longname) - 1);
        longname[sizeof(longname) - 1] = '\0';
        MPE_CHECK(&t, mpe_register_broadphase(longname, &evil_bp) < 0);
        MPE_CHECK(&t, mpe_find_pair_handler(0, 0, -1, -1) != NULL); /* builtin intact */
        MPE_CHECK(&t, mpe_find_broadphase("hash") != NULL);
        MPE_CHECK(&t, mpe_find_solver("seq-impulse") != NULL);
    }

    /* ---- static-only: stage reset + mod_state threading ---- */
    {
        physics_world w;
        mpe_world_begin(&w);
        mpe_broadphase_if_t fb = {fake_generate};
        MPE_CHECK(&t, mpe_register_broadphase("suite-bp", &fb) >= 0);
        physics_world_set_broadphase(&w, mpe_find_broadphase("suite-bp"));
        int sentinel = 1234;
        physics_world_set_broadphase_state(&w, &sentinel);
        mpe_solver_if_t fs = {rec_resolve, NULL, NULL, NULL};
        MPE_CHECK(&t, mpe_register_solver("suite-sv", &fs) >= 0);
        physics_world_set_solver(&w, mpe_find_solver("suite-sv"));
        physics_world_set_solver_state(&w, &sentinel);
        /* Resting contact -> manifold -> foreign resolve runs with state. */
        MPE_CHECK(&t, mpe_floor_slab(&w, 0.4f, 0.3f, 0.0f) >= 0);
        MPE_CHECK(&t, physics_world_add_sphere(&w, 0.3f, 1.0f, (vector3){0.0f, 0.3f, 0.0f}) >= 0);
        saw_state = 0;
        saw_ptr = NULL;
        saw_bp_state = NULL;
        physics_world_step(&w, 1.0f / 60.0f);
        MPE_CHECK(&t, w.broadphase_if != NULL);
        MPE_CHECK(&t, saw_bp_state == (void *)&sentinel);
        MPE_CHECK(&t, saw_state == 1 && saw_ptr == (void *)&sentinel);
        /* Unregister resets world slots to builtin (NULL). */
        MPE_CHECK(&t, mpe_unregister_broadphase("suite-bp") == 0);
        MPE_CHECK(&t, w.broadphase_if == NULL && w.broadphase_state == NULL);
        MPE_CHECK(&t, mpe_unregister_solver("suite-sv") == 0);
        MPE_CHECK(&t, w.solver_if == NULL && w.solver_state == NULL);
        physics_world_cleanup(&w);
    }

    /* ---- live .so lifecycle (needs CWD=v15S/src) ---- */
    if (access("plugins/mpe_capsule.so", R_OK) != 0) {
        printf("[SKIP] plugins/mpe_capsule.so not visible (run from v15S/src)\n");
        mpe_test_end(&t);
        return t.failures;
    }
    {
        char err[512] = {0};
        MPE_CHECK(&t, mpe_loader_load("plugins/mpe_capsule.so", err, sizeof(err)) == 0);
        MPE_CHECK(&t, mpe_find_module("capsule-shape") != NULL);
        MPE_CHECK(&t, mpe_find_pair_handler(3, 0, 100, -1) != NULL);
        /* Direct analytic check of the rewritten segment capsule.
         * Bounding invariant: R = sqrt(h^2 + rc^2) = sqrt(0.05) for
         * h = 0.2, rc = 0.1. Sphere r 0.1 at (0,0.15,0) -> barrel
         * contact pen 0.05 normal +Y at (0,0.1,0); at (0,0.25,0) ->
         * gap 0.05 > slop -> no contact. */
        {
            mpe_collide_fn capfn = mpe_find_pair_handler(3, 0, 100, -1);
            MPE_CHECK(&t, capfn != NULL);
            if (capfn) {
                physics_world cw;
                mpe_world_begin(&cw);
                int ci = physics_world_add_custom(&cw, 100, (vector3){0.0f, 0.0f, 0.0f}, 1.0f,
                                                  0.2236068f);
                MPE_CHECK(&t, ci >= 0);
                MPE_CHECK_NEAR(&t, cw.bodies[ci].radius, 0.2236068f, 1e-6f, "bounding-kept");
                cw.bodies[ci].cylinder_half_length = 0.2f;
                cw.bodies[ci].orientation = vector4_identity();
                rigidbody_update_axes(&cw.bodies[ci]);
                int si = physics_world_add_sphere(&cw, 0.1f, 1.0f, (vector3){0.0f, 0.15f, 0.0f});
                MPE_CHECK(&t, si >= 0);
                collision_data cd = {0};
                MPE_CHECK(&t, capfn(&cw.bodies[ci], &cw.bodies[si], &cd, &cw));
                MPE_CHECK_NEAR(&t, cd.contacts[0].penetration, 0.05f, 1e-5f, "capsule-pen");
                MPE_CHECK_NEAR(&t, cd.normal_vector.y, 1.0f, 1e-5f, "capsule-normal");
                MPE_CHECK_NEAR(&t, cd.contacts[0].position.y, 0.1f, 1e-5f, "capsule-pos");
                cw.bodies[si].position = (vector3){0.0f, 0.25f, 0.0f};
                memset(&cd, 0, sizeof(cd));
                MPE_CHECK(&t, !capfn(&cw.bodies[ci], &cw.bodies[si], &cd, &cw));
                physics_world_cleanup(&cw);
            }
        }
        physics_world w;
        mpe_world_begin(&w);
        const mpe_module_desc_t *d = mpe_find_module("capsule-shape");
        MPE_CHECK(&t, d != NULL);
        MPE_CHECK(&t, physics_world_attach_module(&w, d) >= 0);
        /* Attached (hookless but pinned): unload must refuse with -2. */
        MPE_CHECK(&t, mpe_loader_unload("plugins/mpe_capsule.so") == -2);
        MPE_CHECK(&t, physics_world_detach_module(&w, "capsule-shape") == 0);
        MPE_CHECK(&t, mpe_loader_unload("plugins/mpe_capsule.so") == 0);
        /* Purged: no module entry, no pair handlers from the .so. */
        MPE_CHECK(&t, mpe_find_module("capsule-shape") == NULL);
        MPE_CHECK(&t, mpe_find_pair_handler(3, 0, 100, -1) == NULL);
        MPE_CHECK(&t, mpe_find_pair_handler(3, 3, 100, 100) == NULL);
        /* Unknown handle still -1 (distinct from busy -2). */
        MPE_CHECK(&t, mpe_loader_unload("plugins/does_not_exist.so") == -1);
        physics_world_cleanup(&w);
        /* Reload works after full unload (slot reuse path). */
        MPE_CHECK(&t, mpe_loader_load("plugins/mpe_capsule.so", err, sizeof(err)) == 0);
        MPE_CHECK(&t, mpe_find_pair_handler(3, 0, 100, -1) != NULL);
        MPE_CHECK(&t, mpe_loader_unload("plugins/mpe_capsule.so") == 0);
        MPE_CHECK(&t, mpe_find_pair_handler(3, 0, 100, -1) == NULL);
    }

    if (t.failures == 0) {
        printf("[PASS] loader lifecycle green\n");
    }
    mpe_test_end(&t);
    return t.failures;
}

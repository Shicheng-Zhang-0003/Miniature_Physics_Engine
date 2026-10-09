/* MPE Suite v2 — loader/registry lifecycle regression test.
 *
 * Proves the MPI lifetime fixes end to end with a REAL .so
 * (ecosystem/capsule/mpe_capsule.so; needs CWD=v15S/src for confinement —
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
#include "core/mpe_platform.h"
#ifndef MPE_OS_WINDOWS
#include <unistd.h>
#endif
#include "mpe_test.h"
#include "core/mpe_registry.h"
#include "core/mpe_loader.h"
#include "core/mpe_diag.h"
/* Windows-aware plugin path: pick existing .so/.dll variant. */
static const char *mpe_pick_plugin (const char *so_path, char *buf, size_t n) {
#ifdef MPE_OS_WINDOWS
    /* so_path like "ecosystem/capsule/mpe_capsule.so": try as-is, then .dll variant. */
    if (access (so_path, R_OK) == 0)
        return so_path;
    size_t L = strlen (so_path);
    if (L > 3 && strcmp (so_path + L - 3, ".so") == 0) {
        snprintf (buf, n, "%.*s.dll", (int) (L - 3), so_path);
        if (access (buf, R_OK) == 0)
            return buf;
    } else if (L > 4 && _stricmp (so_path + L - 4, ".dll") == 0) {
        snprintf (buf, n, "%.*s.so", (int) (L - 4), so_path);
        if (access (buf, R_OK) == 0)
            return buf;
    }
    /* try MPE_PLUGIN_EXT variant of basename */
    return so_path;
#else
    (void) buf;
    (void) n;
    return so_path;
#endif
} static int saw_state = 0;
static void *saw_ptr = NULL;
static float rec_resolve (mpe_world_t *world, void *manifold, float dt, bool friction_only, int iter, void *mod_state) {
    (void) world;
    (void) manifold;
    (void) dt;
    (void) friction_only;
    (void) iter;
    saw_state = 1;
    saw_ptr = mod_state;
    return 0.0f;
} static void *saw_bp_state = NULL;
static int fake_generate (mpe_world_t *world, broadphase_pair *pairs_out, int max_pairs, float dt, void *mod_state) {
    saw_bp_state = mod_state;
    /* Delegate to the real backend so manifolds exist for resolve. */
    extern int broadphase_generate_pairing (mpe_world_t *world, broadphase_pair *out, int max, float dt);
    return broadphase_generate_pairing (world, pairs_out, max_pairs, dt);
}
static bool fake_sphere (rigidbody *a, rigidbody *b, void *out, mpe_world_t *w) {
    (void) a;
    (void) b;
    (void) out;
    (void) w;
    return false;
}
int mpe_t_loader_lifecycle (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "loader_lifecycle");
    mpe_register_builtins ();
    /* ---- static-only: builtin hijack refusal + name length ---- */
    {
        mpe_broadphase_if_t evil_bp = {fake_generate};
        MPE_CHECK (&t, mpe_register_broadphase ("hash", &evil_bp) < 0);
        mpe_solver_if_t evil_sv = {rec_resolve, NULL, NULL, NULL};
        MPE_CHECK (&t, mpe_register_solver ("seq-impulse", &evil_sv) < 0);
        MPE_CHECK (&t, mpe_register_pair_handler (0, 0, -1, -1, fake_sphere, "evil") < 0);
        char longname [128];
        memset (longname, 'x', sizeof (longname) - 1);
        longname [sizeof (longname) - 1] = '\0';
        MPE_CHECK (&t, mpe_register_broadphase (longname, &evil_bp) < 0);
        MPE_CHECK (&t, mpe_find_pair_handler (0, 0, -1, -1) != NULL); /* builtin intact */
        MPE_CHECK (&t, mpe_find_broadphase ("hash") != NULL);
        MPE_CHECK (&t, mpe_find_solver ("seq-impulse") != NULL);
    }
    /* ---- static-only: stage reset + mod_state threading ---- */
    {
        physics_world w;
        mpe_world_begin (&w);
        mpe_broadphase_if_t fb = {fake_generate};
        MPE_CHECK (&t, mpe_register_broadphase ("suite-bp", &fb) >= 0);
        physics_world_set_broadphase (&w, mpe_find_broadphase ("suite-bp"));
        int sentinel = 1234;
        physics_world_set_broadphase_state (&w, &sentinel);
        mpe_solver_if_t fs = {rec_resolve, NULL, NULL, NULL};
        MPE_CHECK (&t, mpe_register_solver ("suite-sv", &fs) >= 0);
        physics_world_set_solver (&w, mpe_find_solver ("suite-sv"));
        physics_world_set_solver_state (&w, &sentinel);
        /* Resting contact -> manifold -> foreign resolve runs with state. */
        MPE_CHECK (&t, mpe_floor_slab (&w, 0.4f, 0.3f, 0.0f) >= 0);
        MPE_CHECK (&t, physics_world_add_sphere (&w, 0.3f, 1.0f, (vector3) {0.0f, 0.3f, 0.0f}) >= 0);
        saw_state = 0;
        saw_ptr = NULL;
        saw_bp_state = NULL;
        physics_world_step (&w, 1.0f / 60.0f);
        MPE_CHECK (&t, w.broadphase_if != NULL);
        MPE_CHECK (&t, saw_bp_state == (void *) &sentinel);
        MPE_CHECK (&t, saw_state == 1 && saw_ptr == (void *) &sentinel);
        /* Unregister resets world slots to builtin (NULL). */
        MPE_CHECK (&t, mpe_unregister_broadphase ("suite-bp") == 0);
        MPE_CHECK (&t, w.broadphase_if == NULL && w.broadphase_state == NULL);
        MPE_CHECK (&t, mpe_unregister_solver ("suite-sv") == 0);
        MPE_CHECK (&t, w.solver_if == NULL && w.solver_state == NULL);
        physics_world_cleanup (&w);
    }
    /* ---- live plugin lifecycle (needs CWD=v15S/src) ---- */
    char cap_buf [1024];
    const char *cap_path = mpe_pick_plugin ("ecosystem/capsule/mpe_capsule.so", cap_buf, sizeof (cap_buf));
    if (access (cap_path, R_OK) != 0) {
        /* DESPOT-2026-09-29: this returned t.failures == 0, so mpe_run_one
         * printed "[PASS] loader_lifecycle" and the summary read 29/29 green
         * while the case had executed nothing. A case that could not run must
         * not report green. Returns MPE_SKIPPED (distinct from pass and from
         * fail) so the summary can report it separately. */
        printf ("[SKIP] ecosystem/capsule/mpe_capsule%s not visible (run from v15S/src)\n", MPE_PLUGIN_EXT);
        mpe_test_end (&t);
        return MPE_SKIPPED;
    }
    {
        char err [512] = {0};
        MPE_CHECK (&t, mpe_loader_load (cap_path, err, sizeof (err)) == 0);
        MPE_CHECK (&t, mpe_find_module ("capsule-shape") != NULL);
        MPE_CHECK (&t, mpe_find_pair_handler (3, 0, 100, -1) != NULL);
        /* Direct analytic check of the rewritten segment capsule.
         * Bounding invariant: R = sqrt(h^2 + rc^2) = sqrt(0.05) for
         * h = 0.2, rc = 0.1. Sphere r 0.1 at (0,0.15,0) -> barrel
         * contact pen 0.05 normal +Y at (0,0.1,0); at (0,0.25,0) ->
         * gap 0.05 > slop -> no contact. */
        {
            mpe_collide_fn capfn = mpe_find_pair_handler (3, 0, 100, -1);
            MPE_CHECK (&t, capfn != NULL);
            if (capfn) {
                physics_world cw;
                mpe_world_begin (&cw);
                int ci = physics_world_add_custom (&cw, 100, (vector3) {0.0f, 0.0f, 0.0f}, 1.0f, 0.2236068f);
                MPE_CHECK (&t, ci >= 0);
                MPE_CHECK_NEAR (&t, cw.bodies [ci].radius, 0.2236068f, 1e-6f, "bounding-kept");
                cw.bodies [ci].cylinder_half_length = 0.2f;
                cw.bodies [ci].orientation = vector4_identity ();
                rigidbody_update_axes (&cw.bodies [ci]);
                int si = physics_world_add_sphere (&cw, 0.1f, 1.0f, (vector3) {0.0f, 0.15f, 0.0f});
                MPE_CHECK (&t, si >= 0);
                collision_data cd = {0};
                MPE_CHECK (&t, capfn (&cw.bodies [ci], &cw.bodies [si], &cd, &cw));
                MPE_CHECK_NEAR (&t, cd.contacts [0].penetration, 0.05f, 1e-5f, "capsule-pen");
                MPE_CHECK_NEAR (&t, cd.normal_vector.y, 1.0f, 1e-5f, "capsule-normal");
                MPE_CHECK_NEAR (&t, cd.contacts [0].position.y, 0.1f, 1e-5f, "capsule-pos");
                cw.bodies [si].position = (vector3) {0.0f, 0.25f, 0.0f};
                memset (&cd, 0, sizeof (cd));
                MPE_CHECK (&t, !capfn (&cw.bodies [ci], &cw.bodies [si], &cd, &cw));
                physics_world_cleanup (&cw);
            }
        } physics_world w;
        mpe_world_begin (&w);
        const mpe_module_desc_t *d = mpe_find_module ("capsule-shape");
        MPE_CHECK (&t, d != NULL);
        MPE_CHECK (&t, physics_world_attach_module (&w, d) >= 0);
        /* Attached (hookless but pinned): unload must refuse with -2. */
        MPE_CHECK (&t, mpe_loader_unload (cap_path) == -2);
        MPE_CHECK (&t, physics_world_detach_module (&w, "capsule-shape") == 0);
        MPE_CHECK (&t, mpe_loader_unload (cap_path) == 0);
        /* Purged: no module entry, no pair handlers from the .so. */
        MPE_CHECK (&t, mpe_find_module ("capsule-shape") == NULL);
        MPE_CHECK (&t, mpe_find_pair_handler (3, 0, 100, -1) == NULL);
        MPE_CHECK (&t, mpe_find_pair_handler (3, 3, 100, 100) == NULL);
        /* Unknown handle still -1 (distinct from busy -2). */
        MPE_CHECK (&t, mpe_loader_unload ("ecosystem/does_not_exist" MPE_PLUGIN_EXT) == -1);
        physics_world_cleanup (&w);
        /* Reload works after full unload (slot reuse path). */
        MPE_CHECK (&t, mpe_loader_load (cap_path, err, sizeof (err)) == 0);
        MPE_CHECK (&t, mpe_find_pair_handler (3, 0, 100, -1) != NULL);
        MPE_CHECK (&t, mpe_loader_unload (cap_path) == 0);
        MPE_CHECK (&t, mpe_find_pair_handler (3, 0, 100, -1) == NULL);
    }
    if (t.failures == 0) { printf ("[PASS] loader lifecycle green\n"); }
    mpe_test_end (&t);
    return t.failures;
}

/* DESPOT-2026-10-09: every rejection above used to collapse into one bare
 * negative int, so the terminal could only print a guess ("table full /
 * attach hook"). An operator was told a guess about a subsystem they could
 * not see, and localising a live failure took days. This gates the contract
 * that replaced it: DISTINCT codes per cause, and a diagnostics record
 * naming the source, the code, the engine call site and the runtime values.
 *
 * It also gates the one thing that is easy to regress silently: an attach
 * hook with side effects must be invoked EXACTLY ONCE per attach attempt.
 * Calling it twice to build a log message would spawn the child process
 * twice and hand back a half-initialised pointer. */
static int diag_refuse_calls = 0;
static int diag_refuse_hook (physics_world *world, void **state) {
    (void) world;
    (void) state;
    diag_refuse_calls++;
    return 7; /* any non-zero refuses; the value must reach the report */
}
int mpe_t_diag_naming (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "diag_naming");
    mpe_diag_clear ();

    physics_world w;
    physics_world_init (&w);

    /* Distinct code per cause. Every one of these used to be -1. */
    MPE_CHECK (&t, physics_world_attach_module (NULL, NULL) == MPE_ATTACH_E_NULL);
    MPE_CHECK (&t, physics_world_detach_module (NULL, "x") == MPE_DETACH_E_NULL);

    mpe_module_desc_t wrong_abi;
    memset (&wrong_abi, 0, sizeof (wrong_abi));
    wrong_abi.name = "wrong-abi";
    wrong_abi.abi = MPE_MODULE_ABI + 7;
    MPE_CHECK (&t, physics_world_attach_module (&w, &wrong_abi) == MPE_ATTACH_E_ABI);

    mpe_module_desc_t noname;
    memset (&noname, 0, sizeof (noname));
    noname.abi = MPE_MODULE_ABI;
    MPE_CHECK (&t, physics_world_attach_module (&w, &noname) == MPE_ATTACH_E_NAME);

    diag_refuse_calls = 0;
    mpe_module_desc_t hook;
    memset (&hook, 0, sizeof (hook));
    hook.name = "diag-refuser";
    hook.abi = MPE_MODULE_ABI;
    hook.attach = diag_refuse_hook;
    MPE_CHECK (&t, physics_world_attach_module (&w, &hook) == MPE_ATTACH_E_HOOK);
    /* The hook has side effects; reporting its refusal must not re-run it. */
    MPE_CHECK (&t, diag_refuse_calls == 1);

    /* Table full, and the report must say WHICH table with WHICH occupants. */
    mpe_module_desc_t fillers [MPE_MAX_TICK_MODULES + 1];
    char names [MPE_MAX_TICK_MODULES + 1][16];
    int full_at = -1;
    for (int i = 0; i <= MPE_MAX_TICK_MODULES; i++) {
        memset (&fillers [i], 0, sizeof (mpe_module_desc_t));
        snprintf (names [i], sizeof (names [i]), "diag-fill-%02d", i);
        fillers [i].name = names [i];
        fillers [i].abi = MPE_MODULE_ABI;
        if (physics_world_attach_module (&w, &fillers [i]) < 0) {
            full_at = i;
            break;
        }
    }
    MPE_CHECK (&t, full_at == MPE_MAX_TICK_MODULES);
    MPE_CHECK (&t, physics_world_detach_module (&w, "not-attached-ever") == MPE_DETACH_E_NOTFOUND);

    /* Loader rejections must name the cause too, and must name a REAL
     * out-of-jail file as a jail violation (not as "no such file"). */
    char err [512];
    MPE_CHECK (&t, mpe_loader_load ("", err, sizeof (err)) != 0);
    MPE_CHECK (&t, mpe_loader_load ("ecosystem/mgb/build/definitely_absent.so", err, sizeof (err)) != 0);
    {
        FILE *outside = fopen ("mpe_diag_outside.so", "wb");
        if (outside) {
            fputs ("not a plugin\n", outside);
            fclose (outside);
            MPE_CHECK (&t, mpe_loader_load ("mpe_diag_outside.so", err, sizeof (err)) != 0);
            remove ("mpe_diag_outside.so");
        }
    }

    /* Now the report itself: every provoked source named, each with the
     * engine file:line that raised it. */
    static char report [16384];
    mpe_diag_render (report, sizeof (report), 200);
    const char *must_appear [] = {"E_NULL",   "E_ABI",  "E_NAME", "E_HOOK",       "E_TABLE",
                                  "E_NOTFOUND", "E_EMPTY", "E_NOFILE", "physics_world.c:", "mpe_loader.c:"};
    for (size_t i = 0; i < sizeof (must_appear) / sizeof (must_appear [0]); i++)
        MPE_CHECK (&t, strstr (report, must_appear [i]) != NULL);
    /* strerror must be specific, never the old two-guess text. */
    MPE_CHECK (&t, strstr (physics_world_attach_strerror (MPE_ATTACH_E_HOOK), "hook") != NULL);
    MPE_CHECK (&t, strstr (physics_world_attach_strerror (MPE_ATTACH_E_TABLE), "full") != NULL);
    MPE_CHECK (&t, strcmp (physics_world_attach_strerror (MPE_ATTACH_E_ABI), physics_world_attach_strerror (MPE_ATTACH_E_TABLE)) != 0);
    /* Per-source totals must exist and be non-zero for the sources raised. */
    int src_seen = 0;
    for (int i = 0; i < mpe_diag_source_total_count (); i++) {
        const mpe_diag_source_total *st = mpe_diag_source_total_at (i);
        if (st && (strcmp (st->source, "attach") == 0 || strcmp (st->source, "loader") == 0) && st->error > 0) { src_seen++; }
    }
    MPE_CHECK (&t, src_seen == 2);

    physics_world_cleanup (&w);
    mpe_diag_clear ();
    if (t.failures == 0) { printf ("[PASS] diagnostics name every provoked failure source\n"); }
    mpe_test_end (&t);
    return t.failures;
}

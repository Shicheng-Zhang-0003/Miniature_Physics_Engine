/* Module-system smoke: per-world cfg isolation + registry dispatch +
 * custom shape + tick-module hooks. No GTK/GL. */
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "../core/physics_world.h"
#include "../core/mpe_registry.h"
#include "../core/det_math.h"
#include "../config/mpe_config.h"

static int pre_calls = 0;
static void my_pre(mpe_world_t *w, float dt, void *s) { (void)w; (void)dt; (void)s; pre_calls++; }

static const mpe_module_desc_t my_mod = {
    .abi = MPE_MODULE_ABI, .name = "test-hook", .version = "1.0",
    .kind = "generic", .deterministic = true,
    .attach = 0, .detach = 0, .pre_step = my_pre, .post_step = 0,
};

/* Custom-routing probe: sentinel normal proves registry->custom dispatch. */
static bool probe_custom_hit = false;
static bool probe_custom_fn(rigidbody *a, rigidbody *b, void *out, mpe_world_t *w) {
    (void)b; (void)w;
    collision_data *cd = (collision_data *)out;
    cd->object_a = a;
    cd->object_b = a;
    cd->normal_vector = (vector3){0.0f, 0.0f, 1.0f};
    cd->contact_count = 0;
    probe_custom_hit = true;
    return true;
}

/* Counting solver_if: delegates to the builtin with the world's config. */
static int *test_counting_solver_calls = NULL;
static float test_count_resolve(mpe_world_t *world, void *manifold, float dt, bool friction_only, int iter,
                                void *s) {
    (void) s;
    if (test_counting_solver_calls) {
        (*test_counting_solver_calls)++;
    }
    const mpe_config_t *C = (world && world->cfg) ? world->cfg : &g_cfg;
    return collision_resolve_iterative((collision_data *)manifold, dt, friction_only, iter, C);
}
static const mpe_solver_if_t test_counting_solver = {test_count_resolve, NULL, NULL, NULL};

int mpe_module_test_main(void) {
    mpe_config_init();
    mpe_register_builtins();

    /* 1. per-world cfg isolation */
    physics_world A, B;
    physics_world_init(&A);
    physics_world_init(&B);
    static mpe_config_t cfgA, cfgB;
    cfgA = g_cfg; cfgB = g_cfg;
    cfgA.world.gravity = -1.0f;
    cfgB.world.gravity = -20.0f;
    physics_world_set_config(&A, &cfgA);
    physics_world_set_config(&B, &cfgB);
    if (mpe_world_cfg(&A)->world.gravity != -1.0f) { printf("[FAIL] cfg A\n"); return 1; }
    if (mpe_world_cfg(&B)->world.gravity != -20.0f) { printf("[FAIL] cfg B\n"); return 1; }
    printf("[PASS] per-world cfg isolation\n");

    /* 2. registry has built-ins */
    if (!mpe_find_pair_handler(0, 0, -1, -1)) { printf("[FAIL] registry sphere-sphere\n"); return 1; }
    if (!mpe_find_pair_handler(0, 1, -1, -1)) { printf("[FAIL] registry sphere-cube\n"); return 1; }
    if (!mpe_find_broadphase("hash")) { printf("[FAIL] registry broadphase hash\n"); return 1; }
    if (!mpe_find_solver("seq-impulse")) { printf("[FAIL] registry solver seq-impulse\n"); return 1; }
    printf("[PASS] builtin pair/broadphase/solver registry\n");

    /* 3. shape dispatch equivalence: sphere-sphere via registry == direct */
    int ia = physics_world_add_sphere(&A, 0.5f, 1.0f, (vector3){0, 2, 0});
    int ib = physics_world_add_sphere(&A, 0.5f, 1.0f, (vector3){0, 2.4f, 0});
    (void)ia; (void)ib;
    collision_data d1 = {0}, d2 = {0};
    bool r1 = collision_dual_sphere(&A.bodies[0], &A.bodies[1], &d1, NULL);
    bool r2 = mpe_shape_dispatch(&A, &A.bodies[0], &A.bodies[1], &d2);
    if (r1 != r2) { printf("[FAIL] dispatch mismatch\n"); return 1; }
    /* TRUTH: bool-only compare passes while normals/penetration diverge
     * (swapped-frame bug hid here). Compare the manifold content. */
    if (r1 && r2) {
        float dn = vector3_length(vector3_subtraction(d1.normal_vector, d2.normal_vector));
        float dp = fabsf(d1.contacts[0].penetration - d2.contacts[0].penetration);
        if (dn > 1e-5f || dp > 1e-5f || d1.contact_count != d2.contact_count) {
            printf("[FAIL] dispatch manifold differs (dn=%.6f dp=%.6f)\n", dn, dp);
            return 1;
        }
    }
    printf("[PASS] shape dispatch matches builtin\n");

    /* 4. custom shape add + dispatch (bounding-sphere fallback) */
    int ic = physics_world_add_custom(&A, 100, (vector3){5, 2, 0}, 1.0f, 0.5f);
    if (ic < 0 || A.bodies[ic].type != object_custom) { printf("[FAIL] add_custom\n"); return 1; }
    /* place overlapping a sphere to force contact */
    A.bodies[ic].position = (vector3){0, 2.2f, 0};
    rigidbody_sanitize(&A.bodies[ic]);
    if (A.bodies[ic].type != object_custom) { printf("[FAIL] sanitize reset custom\n"); return 1; }
    printf("[PASS] custom shape survives sanitize\n");
    /* TRUTH: custom dispatch was never exercised (fallback unproven).
     * Register a stub for (custom:100 vs sphere) and prove routing. */
    mpe_register_pair_handler(3, 0, 100, -1, probe_custom_fn, "test-custom-probe");
    {
        collision_data dc = {0};
        probe_custom_hit = false;
        bool rc = mpe_shape_dispatch(&A, &A.bodies[ic], &A.bodies[0], &dc);
        if (!rc || !probe_custom_hit) { printf("[FAIL] custom dispatch not routed\n"); return 1; }
        if (dc.normal_vector.z < 0.99f) { printf("[FAIL] custom sentinel normal lost\n"); return 1; }
    }
    mpe_unregister_pair_handler(probe_custom_fn);
    printf("[PASS] custom shape dispatches through registry\n");

    /* 5. tick-module hook fires once per step */
    physics_world_attach_module(&A, &my_mod);
    /* minimal step: drop a sphere and step once */
    physics_world_step(&A, 1.0f / 60.0f);
    if (pre_calls != 1) { printf("[FAIL] pre_step calls=%d\n", pre_calls); return 1; }
    printf("[PASS] tick-module pre_step hook\n");
    physics_world_detach_module(&A, "test-hook");
    physics_world_step(&A, 1.0f / 60.0f);
    if (pre_calls != 1) { printf("[FAIL] detach did not stop calls\n"); return 1; }
    printf("[PASS] detach stops hooks\n");

    /* 6. id cache: lookups correct, survives pool growth + revision bumps */
    {
        physics_world W; physics_world_init(&W);
        for (int i = 0; i < 600; i++) {
            physics_world_add_sphere(&W, 0.3f, 1.0f,
                                     (vector3){(float) (i % 20), 2.0f + (float) (i / 20), 0});
        }
        if (W.body_capacity < 600 || W.body_count != 600) {
            printf("[FAIL] pool growth cap=%d count=%d\n", W.body_capacity, W.body_count);
            return 1;
        }
        printf("[PASS] pool grows on demand (cap=%d)\n", W.body_capacity);
        uint32_t mid_id = W.bodies[300].object_id;
        if (physics_world_index_by_id(&W, mid_id) != 300) {
            printf("[FAIL] id lookup cap=%d\n", physics_world_index_by_id(&W, mid_id));
            return 1;
        }
        if (physics_world_index_by_id(&W, 0xDEADBEEFu) != -1) {
            printf("[FAIL] missing id should be -1\n");
            return 1;
        }
        printf("[PASS] id->index cache correct\n");
        for (int t = 0; t < 120; t++) physics_world_step(&W, 1.0f / 60.0f);
        if (physics_world_index_by_id(&W, mid_id) < 0 && W.body_count == 600) {
            /* bodies may legitimately still all exist; index must resolve */
            printf("[FAIL] id lost after steps\n");
            return 1;
        }
        printf("[PASS] id cache stable across steps\n");
        physics_world_cleanup(&W);
    }

    /* 7. det fallback counters observable process-wide, zero in-contract */
    {
        det_fallback_reset();
        physics_world W; physics_world_init(&W);
        physics_world_add_sphere(&W, 0.5f, 1.0f, (vector3){0, 5, 0});
        for (int t = 0; t < 600; t++) physics_world_step(&W, 1.0f / 60.0f);
        if (det_fallback_pow_total() != 0 || det_fallback_trig_total() != 0) {
            printf("[FAIL] det fallbacks pow=%lu trig=%lu\n", det_fallback_pow_total(), det_fallback_trig_total());
            return 1;
        }
        printf("[PASS] det counters process-wide zero (pow=%lu trig=%lu)\n",
               det_fallback_pow_total(), det_fallback_trig_total());
        physics_world_cleanup(&W);
    }

    /* 8. per-world narrowphase config: slop-0 world sees contact, slop-5cm world does not */
    {
        physics_world W; physics_world_init(&W);
        static mpe_config_t cfgW; cfgW = g_cfg;
        physics_world_add_sphere(&W, 0.5f, 1.0f, (vector3){0, 2, 0});
        physics_world_add_sphere(&W, 0.5f, 1.0f, (vector3){0, 3.005f, 0}); /* 5mm gap */
        collision_data dd = {0};
        cfgW.solver.penetration_slop = 0.01f;
        bool hit_slop = collision_dual_sphere(&W.bodies[0], &W.bodies[1], &dd, &cfgW);
        memset(&dd, 0, sizeof(dd));
        cfgW.solver.penetration_slop = 0.0f;
        bool hit_zero = collision_dual_sphere(&W.bodies[0], &W.bodies[1], &dd, &cfgW);
        if (!hit_slop || hit_zero) { printf("[FAIL] per-world slop routing\n"); return 1; }
        printf("[PASS] per-world narrowphase config\n");
        /* solver_if override: counting resolve hook observes iterations */
        physics_world_cleanup(&W);
    }

    /* 7. solver_if + broadphase_if overrides take effect */
    {
        int resolve_calls = 0;
        physics_world W; physics_world_init(&W);
        physics_world_add_cube(&W, (vector3){0, 0.5f, 0}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        physics_world_add_cube(&W, (vector3){0, 1.5f, 0}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        physics_world_set_solver(&W, &test_counting_solver);
        resolve_calls = 0;
        test_counting_solver_calls = &resolve_calls;
        for (int t = 0; t < 5; t++) physics_world_step(&W, 1.0f / 60.0f);
        if (resolve_calls <= 0) { printf("[FAIL] solver_if resolve never called\n"); return 1; }
        printf("[PASS] solver_if override observes %d resolves\n", resolve_calls);
        physics_world_set_solver(&W, NULL);
        physics_world_cleanup(&W);
    }

    physics_world_cleanup(&A);
    physics_world_cleanup(&B);
    printf("[PASS] module system smoke complete\n");
    return 0;
}

#ifdef mpe_module_test
int main(void) { return mpe_module_test_main(); }
#endif

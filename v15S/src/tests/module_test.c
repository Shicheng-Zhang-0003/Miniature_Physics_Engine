/* Module-system smoke: per-world cfg isolation + registry dispatch +
 * custom shape + tick-module hooks. No GTK/GL. */
#include <stdio.h>
#include <assert.h>
#include "../core/physics_world.h"
#include "../core/mpe_registry.h"
#include "../config/mpe_config.h"

static int pre_calls = 0;
static void my_pre(mpe_world_t *w, float dt, void *s) { (void)w; (void)dt; (void)s; pre_calls++; }

static const mpe_module_desc_t my_mod = {
    .abi = MPE_MODULE_ABI, .name = "test-hook", .version = "1.0",
    .kind = "generic", .deterministic = true,
    .attach = 0, .detach = 0, .pre_step = my_pre, .post_step = 0,
};

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
    printf("[PASS] builtin pair registry\n");

    /* 3. shape dispatch equivalence: sphere-sphere via registry == direct */
    int ia = physics_world_add_sphere(&A, 0.5f, 1.0f, (vector3){0, 2, 0});
    int ib = physics_world_add_sphere(&A, 0.5f, 1.0f, (vector3){0, 2.4f, 0});
    (void)ia; (void)ib;
    collision_data d1 = {0}, d2 = {0};
    bool r1 = collision_dual_sphere(&A.bodies[0], &A.bodies[1], &d1);
    bool r2 = mpe_shape_dispatch(&A, &A.bodies[0], &A.bodies[1], &d2);
    if (r1 != r2) { printf("[FAIL] dispatch mismatch\n"); return 1; }
    printf("[PASS] shape dispatch matches builtin\n");

    /* 4. custom shape add + dispatch (bounding-sphere fallback) */
    int ic = physics_world_add_custom(&A, 100, (vector3){5, 2, 0}, 1.0f, 0.5f);
    if (ic < 0 || A.bodies[ic].type != object_custom) { printf("[FAIL] add_custom\n"); return 1; }
    /* place overlapping a sphere to force contact */
    A.bodies[ic].position = (vector3){0, 2.2f, 0};
    rigidbody_sanitize(&A.bodies[ic]);
    if (A.bodies[ic].type != object_custom) { printf("[FAIL] sanitize reset custom\n"); return 1; }
    printf("[PASS] custom shape survives sanitize\n");

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

    physics_world_cleanup(&A);
    physics_world_cleanup(&B);
    printf("[PASS] module system smoke complete\n");
    return 0;
}

#ifdef mpe_module_test
int main(void) { return mpe_module_test_main(); }
#endif

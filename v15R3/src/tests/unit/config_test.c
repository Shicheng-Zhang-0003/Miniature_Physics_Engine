#ifdef MPE_UNIT_CONFIG_TEST

#include "config/mpe_config.h"
#include "tests/unit/test_framework.h"

static void test_defaults(void) {
    printf("--- config defaults ---\n");

    ASSERT_NEAR(g_cfg.world.gravity, -9.81f, 0.001f, "default gravity");
    ASSERT_NEAR(g_cfg.world.drag, 0.99f, 0.001f, "default drag");
    ASSERT_TRUE(g_cfg.timestep.solver_iterations == 16, "default solver iterations");
}

static void test_clamp(void) {
    printf("--- config clamp ---\n");

    const mpe_param *param = mpe_config_find("world.gravity");
    ASSERT_TRUE(param != NULL, "world.gravity exists in registry");

    mpe_config_set_float("world.gravity", 100.0f);
    ASSERT_NEAR(g_cfg.world.gravity, 0.0f, 0.001f, "gravity clamps to max");

    mpe_config_set_float("world.gravity", -100.0f);
    ASSERT_NEAR(g_cfg.world.gravity, -50.0f, 0.001f, "gravity clamps to min");

    mpe_config_set_float("world.gravity", -20.0f);
    ASSERT_NEAR(g_cfg.world.gravity, -20.0f, 0.001f, "gravity accepts valid value");
}

static void test_save_load_round_trip(void) {
    printf("--- config save/load ---\n");

    const char *path = "/tmp/mpe_unit_config_test.cfg";

    mpe_config_set_float("world.drag", 0.42f);
    ASSERT_TRUE(mpe_config_save(path), "config save succeeds");

    mpe_config_reset_defaults();
    ASSERT_NEAR(g_cfg.world.drag, 0.99f, 0.001f, "reset restores default drag");

    ASSERT_TRUE(mpe_config_load(path), "config load succeeds");
    ASSERT_NEAR(g_cfg.world.drag, 0.42f, 0.001f, "loaded drag matches saved drag");

    remove(path);
}

int main(void) {
    mpe_config_init();

    printf("============================================\n");
    printf("MPE Kernel Unit Tests: config\n");
    printf("============================================\n");

    test_defaults();
    test_clamp();
    test_save_load_round_trip();

    return mpe_unit_test_summary("config_test");
}

#endif /* MPE_UNIT_CONFIG_TEST */

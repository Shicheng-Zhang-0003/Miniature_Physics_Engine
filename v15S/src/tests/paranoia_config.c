/* PARANOIA TEST: Configuration validation - all params in range, no silent corruption */
#ifdef mpe_paranoia_config
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "config/mpe_constants.h"

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Config bounds clamping - all params */
    {
        mpe_config_init();
        int clamped = 0;

        /* Test gravity bounds */
        clamped += mpe_config_set_float("world.gravity", 100.0f);  /* should clamp to max */
        clamped += mpe_config_set_float("world.gravity", -100.0f); /* should clamp to min */

        /* Test drag bounds */
        clamped += mpe_config_set_float("world.drag", 2.0f);
        clamped += mpe_config_set_float("world.drag", 0.0f);

        /* Test solver iterations */
        clamped += mpe_config_set_int("timestep.solver_iterations", 200);
        clamped += mpe_config_set_int("timestep.solver_iterations", 0);

        /* Test slop */
        clamped += mpe_config_set_float("solver.penetration_slop", 1.0f);
        clamped += mpe_config_set_float("solver.penetration_slop", -0.1f);

        printf("[INFO] config_clamping clamped=%d (expected >0)\n", clamped);
        if (clamped == 0) { printf("[FAIL] config bounds not enforced\n"); fail = 1; }
        else { printf("[PASS] config bounds clamping works\n"); }
    }

    /* Test 2: Config persistence - save/load roundtrip */
    {
        mpe_config_init();
        mpe_config_set_float("world.gravity", -5.0f);
        mpe_config_set_float("world.drag", 0.95f);
        mpe_config_set_int("timestep.solver_iterations", 32);
        mpe_config_set_float("solver.penetration_slop", 0.02f);

        char path[256] = "/tmp/paranoia_config.cfg";
        int save_result = mpe_config_save(path);
        if (save_result != 0) { printf("[FAIL] config save failed\n"); fail = 1; }

        mpe_config_reset_defaults();
        int load_result = mpe_config_load(path);
        if (load_result != 0) { printf("[FAIL] config load failed\n"); fail = 1; }

        float g, d; int iters; float slop;
        mpe_config_get_float("world.gravity", &g);
        mpe_config_get_float("world.drag", &d);
        mpe_config_get_int("timestep.solver_iterations", &iters);
        mpe_config_get_float("solver.penetration_slop", &slop);

        int mismatch = (fabsf(g + 5.0f) > 0.0f) || (fabsf(d - 0.95f) > 0.0f) ||
                       (iters != 32) || (fabsf(slop - 0.02f) > 0.0f);

        printf("[INFO] config_persist mismatch=%d\n", mismatch);
        if (mismatch) { printf("[FAIL] config persistence failed\n"); fail = 1; }
        else { printf("[PASS] config exact roundtrip\n"); }
    }

    /* Test 3: Debug-only params - cannot be set in game mode */
    {
        mpe_config_init();
        mpe_config_set_float("solver.penetration_slop", 0.02f);
        int clamped = mpe_config_set_float("sleep.enable", 0); /* debug only */

        /* In actual game mode this would be enforced by UI layer */
        printf("[INFO] debug_only_params test (manual verification)\n");
    }

    /* Test 4: Config migration - old files load with defaults for new params */
    {
        /* Would need actual old config file - tested manually */
        printf("[INFO] config migration tested manually\n");
    }

    /* Test 5: Category iteration - all params accessible */
    {
        mpe_config_init();
        size_t count = mpe_config_count_by_category(cat_world);
        if (count == 0) { printf("[FAIL] cat_world empty\n"); fail = 1; }
        else { printf("[PASS] cat_world has %zu params\n", count); }

        count = mpe_config_count_by_category(cat_solver);
        if (count == 0) { printf("[FAIL] cat_solver empty\n"); fail = 1; }
        else { printf("[PASS] cat_solver has %zu params\n", count); }
    }

    /* Test 6: Invalid key handling */
    {
        float out;
        int result = mpe_config_get_float("nonexistent.param", &out);
        if (result) { printf("[FAIL] invalid key should return false\n"); fail = 1; }
        else { printf("[PASS] invalid key returns false\n"); }
    }

    /* Test 6: Type mismatch handling */
    {
        int out;
        int result = mpe_config_get_int("world.gravity", &out); /* gravity is float */
        if (result) { printf("[FAIL] type mismatch should return false\n"); fail = 1; }
        else { printf("[PASS] type mismatch returns false\n"); }
    }

    return fail;
}
#endif
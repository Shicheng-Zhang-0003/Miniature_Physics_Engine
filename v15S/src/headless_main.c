/* MPE_FTC_057: headless entry. Build with -DMPE_HEADLESS via `make headless`. */
#ifdef MPE_HEADLESS
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "config/mpe_config.h"
#include "core/physics_world.h"
#include "core/det_math.h"

int main(int argc, char *argv[]) {
    int ticks = (argc > 1) ? atoi(argv[1]) : 3600;
    /* FIX-AUDIT-DESPOT: pin FP state explicitly in the headless smoke path.
     * physics_world_init also pins, but the smoke binary's determinism
     * contract must not depend on init ordering (headless twins compare
     * bit-identical trajectories only under identical rounding/FTZ/DAZ). */
    det_pin_fp_state();
    mpe_config_init();
    det_fallback_reset();
    physics_world world;
    physics_world_init(&world);
    physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 5.0f, 0.0f});
    physics_world_add_cube(&world, (vector3){2.0f, 5.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 2.0f);
    const float dt = 1.0f / 60.0f;
    for (int t = 0; t < ticks; t++) {
        physics_world_step(&world, dt);
    }
    int invalid = 0;
    for (int i = 0; i < world.body_count; i++) {
        rigidbody *rb = &world.bodies[i];
        if ((!isfinite(rb->position.x)) || (!isfinite(rb->position.y)) || (!isfinite(rb->position.z))) {
            invalid++;
        }
    }
    /* FIX-AUDIT-DESPOT: libm fallbacks (pow/trig) are accurate but NOT
     * bit-deterministic across platforms; a headless PASS with fallbacks is
     * a portability lie. Assert zero fallbacks like the suite's determinism
     * tests (det_fallback_pow_total/trig_total). */
    unsigned long pow_fb = det_fallback_pow_total();
    unsigned long trig_fb = det_fallback_trig_total();
    printf("[headless] ticks=%d bodies=%d invalid=%d fallback_pow=%lu fallback_trig=%lu result=%s\n",
           ticks, world.body_count, invalid, pow_fb, trig_fb,
           (invalid == 0 && pow_fb == 0 && trig_fb == 0) ? "PASS" : "FAIL");
    physics_world_cleanup(&world);
    return (invalid == 0 && pow_fb == 0 && trig_fb == 0) ? 0 : 1;
}
#endif /* MPE_HEADLESS */

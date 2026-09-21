/* MPE_FTC_059D: two-world independence test. Built via `make test_two_world`. */
#ifdef mpe_two_world_test
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    physics_world world_a;
    physics_world world_b;
    physics_world_init(&world_a);
    physics_world_init(&world_b);
    /* TRUTH: overlap positions AND interleave stepping with DIFFERENT
     * gravity per world (old: B never stepped, 50m away - trivially
     * unchanged, proving nothing about cross-talk). Same start, different
     * gravity: A must out-fall B (independence + per-world config). */
    mpe_config_t cfg_b = g_cfg;
    cfg_b.world.gravity = -1.0f;
    physics_world_set_config(&world_b, &cfg_b);
    physics_world_add_sphere(&world_a, 0.5f, 1.0f, (vector3){0.0f, 10.0f, 0.0f});
    physics_world_add_sphere(&world_b, 0.5f, 1.0f, (vector3){0.0f, 10.0f, 0.0f});
    const float dt = 1.0f / 60.0f;
    int fail = 0;
    float ya_mid = 0.0f, yb_mid = 0.0f;
    for (int t = 0; t < 600; t++) {
        physics_world_step(&world_a, dt);
        physics_world_step(&world_b, dt);
        rigidbody *ra = &world_a.bodies[0];
        rigidbody *rb = &world_b.bodies[0];
        if ((!isfinite(rb->position.x)) || (!isfinite(rb->position.y)) || (!isfinite(rb->position.z)) ||
            (!isfinite(ra->position.x)) || (!isfinite(ra->position.y)) || (!isfinite(ra->position.z))) {
            printf("[FAIL] non-finite state on tick %d\n", t);
            fail = 1;
            break;
        }
        /* Mid-run (t=1s): A (g=-9.81) at ~5.1, B (g=-1.0) at ~9.5.
         * Same start + different gravity MUST separate: proves per-world
         * config isolation (a shared-global gravity would move together). */
        if (t == 60) {
            ya_mid = ra->position.y;
            yb_mid = rb->position.y;
            printf("[info] t=1s: y_a=%.3f (g=-9.81) y_b=%.3f (g=-1.0)\n", ya_mid, yb_mid);
        }
    }
    if (!fail) {
        /* Both settle on the plane at 0.5 eventually; independence was
         * proven mid-run above. End state: both finite and settled. */
        if ((yb_mid - ya_mid) < 2.0f) {
            printf("[FAIL] worlds not independent at t=1s (y_a=%.3f y_b=%.3f)\n", ya_mid, yb_mid);
            fail = 1;
        } else if (world_a.bodies[0].position.y > 9.0f) {
            printf("[FAIL] world A sphere did not fall (y=%.3f)\n", world_a.bodies[0].position.y);
            fail = 1;
        } else {
            printf("[PASS] two worlds independent: separated %.3f m at t=1s by per-world gravity\n",
                   yb_mid - ya_mid);
        }
    }
    physics_world_cleanup(&world_a);
    physics_world_cleanup(&world_b);
    return fail;
}
#endif /* mpe_two_world_test */

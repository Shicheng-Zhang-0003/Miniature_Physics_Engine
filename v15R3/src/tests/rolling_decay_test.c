/* MPE rolling-decay truth test: with air drag disabled, contact-patch
 * rolling resistance must still decay a rolling ball (no perpetual roll). */
#ifdef MPE_ROLLING_DECAY_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    g_cfg.world.drag = 1.0f;
    g_cfg.world.angular_damping_scale = 1.0f;
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init();

    int s = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){-8.0f, 0.5f, 0.0f});
    world.bodies[s].velocity = (vector3){2.0f, 0.0f, 0.0f};
    world.bodies[s].angular_velocity = (vector3){0.0f, 0.0f, -4.0f};
    rigidbody_wake(&world.bodies[s]);

    const float dt = 1.0f / 60.0f;
    for (int t = 0; t < 480; t++) {
        physics_world_step(&world, dt);
        if (!isfinite(world.bodies[s].position.x)) {
            printf("[FAIL] NaN during roll\n");
            physics_world_cleanup(&world);
            return 1;
        }
    }
    /* Perpetual roll would travel 16 m. mu_r*g decay (~0.2 m/s^2) gives ~9.7 m. */
    float dist = world.bodies[s].position.x - (-8.0f);
    float v = vector3_length(world.bodies[s].velocity);
    printf("[info] rolled %.3f m in 8 s, end speed %.3f\n", dist, v);
    int fail = 0;
    if (dist > 14.0f) {
        printf("[FAIL] ball barely decayed (%.3f m): no rolling resistance\n", dist);
        fail = 1;
    } else if (dist < 4.0f) {
        printf("[FAIL] ball stopped too fast (%.3f m): resistance overdamped\n", dist);
        fail = 1;
    } else {
        printf("[PASS] rolling decays at contact-patch rate (%.3f m)\n", dist);
    }
    physics_world_cleanup(&world);
    return fail;
}
#endif /* MPE_ROLLING_DECAY_TEST */

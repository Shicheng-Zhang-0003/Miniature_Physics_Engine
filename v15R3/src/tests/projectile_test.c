/* Projectile truth: ballistic apex height/time and horizontal range must
 * match closed-form kinematics (constant-gravity, drag disabled). */
#ifdef MPE_PROJECTILE_TEST
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

    const float vx = 8.0f, vy = 12.0f, g = 9.81f;
    int s = physics_world_add_sphere(&world, 0.2f, 1.0f, (vector3){0.0f, 1.0f, 0.0f});
    world.bodies[s].velocity = (vector3){vx, vy, 0.0f};
    rigidbody_wake(&world.bodies[s]);

    const float dt = 1.0f / 60.0f;
    float apex = 0.0f, t_apex = 0.0f, x_apex = 0.0f;
    for (int t = 0; t < 400; t++) {
        physics_world_step(&world, dt);
        rigidbody *b = &world.bodies[s];
        if (!isfinite(b->position.x)) {
            printf("[FAIL] NaN\n");
            physics_world_cleanup(&world);
            return 1;
        }
        if (b->position.y > apex) {
            apex = b->position.y;
            t_apex = (float)(t + 1) * dt;
            x_apex = b->position.x;
        }
        if (b->position.y < 0.25f) {
            break; /* landed */
        }
    }
    float apex_e = 1.0f + vy * vy / (2.0f * g);
    float t_e = vy / g;
    float x_e = vx * t_e;
    printf("[info] apex=%.4f (expect %.4f) t=%.4f (expect %.4f) x=%.4f (expect %.4f)\n", apex, apex_e, t_apex, t_e,
           x_apex, x_e);
    int fail = 0;
    if (fabsf(apex - apex_e) / apex_e > 0.02f) {
        printf("[FAIL] apex height off\n");
        fail = 1;
    } else {
        printf("[PASS] ballistic apex exact\n");
    }
    if (fabsf(t_apex - t_e) / t_e > 0.05f) {
        printf("[FAIL] time-to-apex off\n");
        fail = 1;
    } else {
        printf("[PASS] time-to-apex exact\n");
    }
    if (fabsf(x_apex - x_e) / x_e > 0.02f) {
        printf("[FAIL] horizontal range off (spurious forces?)\n");
        fail = 1;
    } else {
        printf("[PASS] horizontal motion force-free\n");
    }
    physics_world_cleanup(&world);
    return fail;
}
#endif /* MPE_PROJECTILE_TEST */

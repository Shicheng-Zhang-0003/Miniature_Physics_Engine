/* MPE Phase 1.2-prep #1: wake-on-contact test.
* A sleeping cube is struck by a fast projectile. The cube must wake. */
#ifdef MPE_SLEEP_WAKE_CONTACT_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    constraint_pool_init();
    physics_world world;
    physics_world_init(&world);

    /* Static floor (not strictly needed, kept for realism) */
    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    /* Sleeping target cube, floating well above the floor */
    int target = physics_world_add_cube(&world,
        (vector3){0.0f, 2.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 2.0f);
    world.bodies[target].velocity = vector3_zero();
    world.bodies[target].angular_velocity = vector3_zero();
    world.bodies[target].is_sleeping = true;
    world.bodies[target].sleep_timer = 2.0f;

    /* Fast projectile aimed at the target along -Z */
    int projectile = physics_world_add_sphere(&world, 0.35f, 3.0f,
        (vector3){0.0f, 2.0f, 2.0f});
    world.bodies[projectile].velocity = (vector3){0.0f, 0.0f, -8.0f};

    const float dt = 1.0f / 60.0f;
    int woke = 0;
    int fail = 0;
    for (int t = 0; t < 60 && !fail; t++) {
        physics_world_step(&world, dt);
        for (int i = 0; i < world.body_count; i++) {
            if (!isfinite(world.bodies[i].position.x) ||
                !isfinite(world.bodies[i].position.y) ||
                !isfinite(world.bodies[i].position.z)) {
                printf("[FAIL] NaN at tick %d\n", t);
                fail = 1; break;
            }
        }
        if (!world.bodies[target].is_sleeping) {
            woke = 1;
        }
    }
    if (fail) return 1;
    if (!woke) {
        printf("[FAIL] sleeping cube was never woken by projectile impact\n");
        return 1;
    }
    printf("[PASS] wake-on-contact: sleeping cube woken by impact\n");
    physics_world_cleanup(&world);
    return 0;
}
#endif /* MPE_SLEEP_WAKE_CONTACT_TEST */

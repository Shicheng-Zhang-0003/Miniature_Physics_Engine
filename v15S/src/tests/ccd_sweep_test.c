/* MPE CCD truth test: fast bodies must impact, never tunnel. */
#ifdef mpe_ccd_sweep_test
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    int fail = 0;
    const float dt = 1.0f / 60.0f;

    /* Case 1: 144 m/s sphere at a 0.1 m wall (2.4 m steps straddle it). */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);
        physics_world_add_cube(&world, (vector3){0, 5.0f, 0}, (vector3){0.05f, 5.0f, 5.0f}, 0.0f);
        int s = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){-5.7f, 5.0f, 0});
        world.bodies[s].velocity = (vector3){144.0f, 0.0f, 0};
        world.bodies[s].restitution = 0.0f;
        rigidbody_wake(&world.bodies[s]);
        for (int t = 0; t < 30; t++) {
            physics_world_step(&world, dt);
        }
        float x = world.bodies[s].position.x;
        float vx = world.bodies[s].velocity.x;
        printf("[info] wall case: final_x=%.3f (face contact at -0.55)\n", x);
        /* TRUTH: two-sided. A frozen body at -5.7 or a 0.09m penetration
         * both passed the old one-sided gate. Demand stopped AT the face
         * with spent velocity. */
        if (x > -0.45f) {
            printf("[FAIL] sphere tunneled the thin wall\n");
            fail = 1;
        } else if (x < -0.75f) {
            printf("[FAIL] sphere never reached/stopped at the wall (x=%.3f)\n", x);
            fail = 1;
        } else if (fabsf(vx) > 5.0f) {
            printf("[FAIL] sphere still flying after wall impact (vx=%.3f)\n", vx);
            fail = 1;
        } else {
            printf("[PASS] 144 m/s sphere stopped at thin wall\n");
        }
        physics_world_cleanup(&world);
    }

    /* Case 2: 60 m/s sphere straight down at the floor. */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);
        int s = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0, 5.0f, 0});
        world.bodies[s].velocity = (vector3){0, -60.0f, 0};
        world.bodies[s].restitution = 0.0f;
        rigidbody_wake(&world.bodies[s]);
        float min_y = 1e9f;
        for (int t = 0; t < 120; t++) {
            physics_world_step(&world, dt);
            if (world.bodies[s].position.y < min_y) {
                min_y = world.bodies[s].position.y;
            }
        }
        printf("[info] floor case: min_center_y=%.4f rest_y=%.3f\n", min_y, world.bodies[s].position.y);
        /* TRUTH: two-sided. Old min_y<-0.55 allowed 1.04m penetration
         * (center -0.54, fully through) to pass. Demand no deep tunnel
         * AND settled rest at radius height. */
        float rest_y = world.bodies[s].position.y;
        if (min_y < 0.40f) {
            printf("[FAIL] sphere tunneled the floor (min_y=%.4f)\n", min_y);
            fail = 1;
        } else if (rest_y < 0.45f || rest_y > 0.55f) {
            printf("[FAIL] sphere did not settle at rest height (rest_y=%.4f)\n", rest_y);
            fail = 1;
        } else {
            printf("[PASS] 60 m/s sphere lands on floor\n");
        }
        physics_world_cleanup(&world);
    }

    if (fail == 0) {
        printf("[PASS] ccd sweep truth complete\n");
    }
    return fail;
}
#endif /* mpe_ccd_sweep_test */

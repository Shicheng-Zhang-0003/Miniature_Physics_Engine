/* Friction-stop truth: a sliding block must stop after exactly v^2/(2*mu*g)
 * (work-energy: kinetic energy dissipated by Coulomb friction). */
#ifdef mpe_friction_stop_test
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    /* The infinite solver floor is coplanar with the slab (both y=0) and
     * shares support: its friction must equal the declared test surface
     * (mu=0.3) or min-combine drags the effective coefficient toward the
     * floor default (0.2/0.1) and the block slides ~3x far. Same class of
     * setup-material fix as the bounce test's floor restitution. */
    g_cfg.world.floor_friction_s = 0.3f;
    g_cfg.world.floor_friction_k = 0.3f;
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(&world);

    /* Static floor slab, top at y=0, body friction 0.3. */
    int floor = physics_world_add_cube(&world, (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    world.bodies[floor].friction_static = 0.3f;
    world.bodies[floor].friction_kinetic = 0.3f;
    int box = physics_world_add_cube(&world, (vector3){-6.0f, 0.55f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    world.bodies[box].friction_static = 0.3f;
    world.bodies[box].friction_kinetic = 0.3f;
    world.bodies[box].velocity = (vector3){4.0f, 0.0f, 0.0f};
    rigidbody_wake(&world.bodies[box]);

    const float dt = 1.0f / 60.0f;
    for (int t = 0; t < 60; t++) {
        physics_world_step(&world, dt); /* settle onto the slab */
    }
    float x0 = world.bodies[box].position.x;
    float v0 = vector3_length(world.bodies[box].velocity);
    for (int t = 0; t < 600; t++) {
        physics_world_step(&world, dt);
        if (!isfinite(world.bodies[box].position.x)) {
            printf("[FAIL] NaN\n");
            physics_world_cleanup(&world);
            return 1;
        }
        if (vector3_length(world.bodies[box].velocity) < 0.005f) {
            break;
        }
    }
    float dist = world.bodies[box].position.x - x0;
    float analytic = v0 * v0 / (2.0f * 0.3f * 9.81f);
    printf("[info] stop distance=%.4f (expect %.4f from v0=%.3f)\n", dist, analytic, v0);
    int fail = 0;
    if (fabsf(dist - analytic) / analytic > 0.15f) {
        printf("[FAIL] friction dissipates the wrong energy\n");
        fail = 1;
    } else {
        printf("[PASS] Coulomb friction stops at v^2/(2*mu*g)\n");
    }
    physics_world_cleanup(&world);
    return fail;
}
#endif /* mpe_friction_stop_test */

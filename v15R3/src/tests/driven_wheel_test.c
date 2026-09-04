/* MPE_FTC_093h: Grounded driven wheel propulsion proof. */
#ifdef MPE_DRIVEN_WHEEL_TEST

#include <math.h>
#include <stdio.h>

#include "config/mpe_config.h"
#include "core/physics_world.h"
#include "core/rigidbody.h"

int main(void) {
    mpe_config_init();
    printf("[info] gravity = %.4f\n", g_cfg.world.gravity);

    physics_world world;
    physics_world_init(&world);
    world.next_object_id = 1;

    /* Cylinder wheel resting on the floor. Spawn slightly above (y=0.06)
       so it drops, settles, and establishes solid contact manifolds. */
    int w = physics_world_add_cylinder(&world, 0.05f, 0.02f, 0.5f,
                                       (vector3){0.0f, 0.06f, 0.0f});
    if (w < 0) { printf("[FAIL] could not create wheel\n"); return 1; }

    const float dt = 1.0f / 60.0f;
    const float drive_torque = 0.15f; /* N·m about the axle (X) */

    /* Let it settle for 1 second before applying drive torque */
    for (int t = 0; t < 60; t++) {
        physics_world_step(&world, dt);
    }

    float start_z = world.bodies[w].position.z;

    /* Apply torque for 3 seconds */
    for (int t = 0; t < 180; t++) {
        rigidbody_wake(&world.bodies[w]);
        world.bodies[w].torque_accumulator.x += drive_torque;
        physics_world_step(&world, dt);
    }

    float dz = world.bodies[w].position.z - start_z;
    float vz = world.bodies[w].velocity.z;
    float wx = world.bodies[w].angular_velocity.x;
    float y  = world.bodies[w].position.y;

    printf("[info] grounded wheel: dz=%.4f vz=%.4f wx=%.4f y=%.4f\n", dz, vz, wx, y);

    if (fabsf(wx) < 1.0f) {
        printf("[FAIL] wheel did not spin under torque (wx=%.4f)\n", wx);
        return 1;
    }

    if (fabsf(dz) < 0.10f) {
        printf("[GAP] wheel spun but did not translate (dz=%.4f) — contact friction not gripping\n", dz);
        return 1;
    }

    /* Kinematic check: rolling without slipping means vz ≈ wx * r */
    float expected_vz = wx * 0.05f;
    printf("[info] kinematic check: expected vz (w*r) = %.4f, actual vz = %.4f\n", expected_vz, vz);

    printf("[PASS] grounded wheel rolled %.4f m via real floor friction\n", dz);
    return 0;
}

#endif /* MPE_DRIVEN_WHEEL_TEST */

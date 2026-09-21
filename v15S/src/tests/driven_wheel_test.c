/* MPE_FTC_093h: Grounded driven wheel propulsion proof. */
#ifdef mpe_driven_wheel_test

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
    /* TRUTH: torque sized INSIDE the traction envelope. Max traction torque
     * is mu_k*N*r = 0.1*4.905*0.05 = 0.0245 N·m (kinetic floor friction
     * 0.1, weight 0.5 kg, radius 0.05 m). Above it traction saturates and
     * the wheel MUST burnout-spin: no equilibrium exists (constant drive vs
     * saturating damping), spin crosses pi rad/tick where discrete contact
     * is undefined for ANY scheme, and the run becomes a sanitize-restart
     * lottery (measured: mid-run explosion to 1e21 with y=204, masked only
     * if a restart lands calm before the final gates). The old 0.080 value
     * exceeded traction 3x and "passed" only via such luck. 0.020 holds
     * traction with margin (static 0.049), stays in the resolvable regime
     * (~1 rad/tick), and rolls deterministically. */
    const float drive_torque = 0.020f; /* N·m about the axle (X) */
    {
        float traction_limit =
            g_cfg.world.floor_friction_k * (0.5f * 9.81f) * 0.05f;
        if (!(drive_torque < traction_limit)) {
            printf("[FAIL] drive torque %.4f exceeds traction limit %.4f\n",
                   drive_torque, traction_limit);
            return 1;
        }
    }

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

    if (!isfinite(dz) || !isfinite(vz) || !isfinite(wx) || !isfinite(y)) {
        printf("[FAIL] non-finite wheel state\n");
        return 1;
    }

    if (fabsf(wx) < 5.0f) {
        printf("[FAIL] wheel did not spin up under torque (wx=%.4f)\n", wx);
        return 1;
    }

    if (fabsf(wx) > 100.0f) {
        printf("[FAIL] wheel spun past the resolvable regime (wx=%.4f > 100 ~ 1.7 rad/tick)\n", wx);
        return 1;
    }

    /* Grounded: the wheel radius is 0.05; liftoff lunacy reached y=147+. */
    if (fabsf(y - 0.05f) > 0.02f) {
        printf("[FAIL] wheel left the ground (y=%.4f, expected ~0.05)\n", y);
        return 1;
    }

    if (fabsf(dz) < 1.5f) {
        printf("[GAP] wheel spun but did not translate (dz=%.4f) — contact friction not gripping\n", dz);
        return 1;
    }

    /* Rolling coupling: propulsion direction must match spin, and slip must
     * stay physical — vz tracks wx*r (pure roll) within slip tolerance.
     * TRUTH: 0.20-1.15 admitted 80% slip (pure spin + creep) as rolling;
     * near-unity coupling is what rolling MEANS. */
    float expected_vz = wx * 0.05f;
    printf("[info] kinematic check: expected vz (w*r) = %.4f, actual vz = %.4f\n", expected_vz, vz);
    if ((vz * wx) < 0.0f) {
        printf("[FAIL] translation opposes spin (vz=%.4f, wx=%.4f) — wrong propulsion direction\n", vz, wx);
        return 1;
    }
    float coupling = fabsf(vz) / (fabsf(expected_vz) + 1e-6f);
    if (coupling < 0.70f || coupling > 1.10f) {
        printf("[FAIL] unphysical slip (vz/wr=%.3f, need 0.70..1.10)\n", coupling);
        return 1;
    }

    printf("[PASS] grounded wheel rolled %.4f m via real floor friction\n", dz);
    return 0;
}

#endif /* mpe_driven_wheel_test */

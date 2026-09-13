/* Angular momentum truth: a torque-free tumbling asymmetric body must keep
 * its world-frame angular momentum vector exactly constant. This exercises
 * the inertia-tensor rotation and the gyroscopic term simultaneously. */
#ifdef MPE_ANGMOM_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

static vector3 body_L(rigidbody *rb) {
    math3 R = vector4_to_math3(rb->orientation);
    math3 Rt = math3_transposition(R);
    math3 Iw = math3_multiplication(R, math3_multiplication(rb->inertia_tensor_local, Rt));
    return math3_multiplication_vector3(Iw, rb->angular_velocity);
}

int main(void) {
    mpe_config_init();
    g_cfg.world.drag = 1.0f;
    g_cfg.world.angular_damping_scale = 1.0f;
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init();

    /* Distinct half extents => fully populated inertia tensor. */
    int b = physics_world_add_cube(&world, (vector3){0.0f, 50.0f, 0.0f}, (vector3){0.3f, 0.5f, 0.7f}, 2.0f);
    world.bodies[b].angular_velocity = (vector3){1.0f, 3.0f, 2.0f};
    rigidbody_wake(&world.bodies[b]);

    vector3 L0 = body_L(&world.bodies[b]);
    float L0n = vector3_length(L0);
    const float dt = 1.0f / 60.0f;
    float max_err = 0.0f;
    for (int t = 0; t < 120; t++) {
        physics_world_step(&world, dt);
        vector3 L = body_L(&world.bodies[b]);
        float err = vector3_length(vector3_subtraction(L, L0)) / L0n;
        if (err > max_err) {
            max_err = err;
        }
        if (!isfinite(err)) {
            printf("[FAIL] NaN in angular momentum\n");
            physics_world_cleanup(&world);
            return 1;
        }
    }
    printf("[info] max |L-L0|/|L0| over 2 s tumble: %.5f\n", max_err);
    int fail = 0;
    if (max_err > 0.03f) {
        printf("[FAIL] angular momentum drifts (%.4f)\n", max_err);
        fail = 1;
    } else {
        printf("[PASS] torque-free angular momentum conserved\n");
    }
    physics_world_cleanup(&world);
    return fail;
}
#endif /* MPE_ANGMOM_TEST */

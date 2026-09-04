/* MPE_FTC_068: revolute pendulum test. Built via `make test_revolute`. */
#ifdef MPE_REVOLUTE_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    physics_world world;
    physics_world_init(&world);

    int pivot_index = physics_world_add_cube(&world, (vector3){0.0f, 10.0f, 0.0f}, (vector3){0.2f, 0.2f, 0.2f}, 1.0f);
    rigidbody_set_static(&world.bodies[pivot_index], true);
    uint32_t pivot_id = world.bodies[pivot_index].object_id;

    int bob_index = physics_world_add_sphere(&world, 0.3f, 2.0f, (vector3){1.0f, 8.0f, 0.0f});
    uint32_t bob_id = world.bodies[bob_index].object_id;

    vector3 pivot_point = {0.0f, 10.0f, 0.0f};
    float rod_length = vector3_length(vector3_subtraction(pivot_point, world.bodies[bob_index].position));
    vector3 start_position = world.bodies[bob_index].position;

    constraint_pool_init();
    vector3 anchor_a = {0.0f, 0.0f, 0.0f}; /* pivot centre -> world (0,10,0) */
    vector3 anchor_b = {-1.0f, 2.0f, 0.0f}; /* bob-local -> world (0,10,0)   */
    vector3 axis = {0.0f, 0.0f, 1.0f}; /* swing in the x-y plane        */
    int joint_index = constraint_add_revolute(pivot_id, bob_id, anchor_a, anchor_b, axis);
    if (joint_index < 0) {
        printf("[FAIL] could not add revolute joint\n");
        return 1;
    }

    const float dt = 1.0f / 60.0f;
    int fail = 0;
    float max_drift = 0.0f;
    for (int t = 0; t < 600; t++) {
        physics_world_step(&world, dt);
        rigidbody *bob = &world.bodies[bob_index];
        if ((!isfinite(bob->position.x)) || (!isfinite(bob->position.y)) || (!isfinite(bob->position.z))) {
            printf("[FAIL] bob went non-finite at tick %d\n", t);
            fail = 1;
            break;
        }
        float dist = vector3_length(vector3_subtraction(pivot_point, bob->position));
        float drift = fabsf(dist - rod_length);
        if (drift > max_drift) {
            max_drift = drift;
        }
    }
    if (!fail) {
        rigidbody *bob = &world.bodies[bob_index];
        float moved = vector3_length(vector3_subtraction(bob->position, start_position));
        printf("[info] rod=%.4f max_drift=%.4f moved=%.4f bob=(%.3f,%.3f,%.3f)\n", rod_length, max_drift, moved,
               bob->position.x, bob->position.y, bob->position.z);
        if (max_drift > 0.15f) {
            printf("[FAIL] anchor drift %.4f too large — revolute not holding\n", max_drift);
            fail = 1;
        } else if (moved < 0.05f) {
            printf("[FAIL] bob did not move — gravity or joint not acting\n");
            fail = 1;
        } else {
            printf("[PASS] revolute pendulum holds (max drift %.4f) and swings under gravity\n", max_drift);
        }
    }
    physics_world_cleanup(&world);
    return fail;
}
#endif /* MPE_REVOLUTE_TEST */

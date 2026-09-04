/* MPE_FTC_059D: two-world independence test. Built via `make test_two_world`. */
#ifdef MPE_TWO_WORLD_TEST
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
    world_b.next_object_id = 1000; /* keep contact-cache IDs disjoint */
    physics_world_add_sphere(&world_a, 0.5f, 1.0f, (vector3){0.0f, 10.0f, 0.0f});
    physics_world_add_cube(&world_b, (vector3){50.0f, 10.0f, 50.0f}, (vector3){0.5f, 0.5f, 0.5f}, 2.0f);
    vector3 b_initial_position = world_b.bodies[0].position;
    const float dt = 1.0f / 60.0f;
    int fail = 0;
    for (int t = 0; t < 600; t++) {
        physics_world_step(&world_a, dt);
        rigidbody *rb_b = &world_b.bodies[0];
        float drift = vector3_length(vector3_subtraction(rb_b->position, b_initial_position));
        if (drift > 0.0001f) {
            printf("[FAIL] world B drifted on tick %d (drift=%.6f)\n", t, drift);
            fail = 1;
            break;
        }
        if ((!isfinite(rb_b->position.x)) || (!isfinite(rb_b->position.y)) || (!isfinite(rb_b->position.z))) {
            printf("[FAIL] world B went non-finite on tick %d\n", t);
            fail = 1;
            break;
        }
    }
    if (!fail) {
        if (world_a.bodies[0].position.y < 9.0f) {
            printf("[PASS] two worlds independent: A stepped (sphere y=%.3f), B untouched\n",
                   world_a.bodies[0].position.y);
        } else {
            printf("[FAIL] world A sphere did not fall (y=%.3f)\n", world_a.bodies[0].position.y);
            fail = 1;
        }
    }
    physics_world_cleanup(&world_a);
    physics_world_cleanup(&world_b);
    return fail;
}
#endif /* MPE_TWO_WORLD_TEST */

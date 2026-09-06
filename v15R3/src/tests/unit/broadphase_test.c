#ifdef MPE_UNIT_BROADPHASE_TEST

#include "core/rigidbody.h"
#include "physics/broadphase.h"
#include "config/mpe_config.h"
#include "tests/unit/test_framework.h"

static void test_pair_generation(void) {
    printf("--- broadphase pair generation ---\n");

    rigidbody bodies[3];

    rigidbody_initialisation_sphere(&bodies[0], 1.0f, 1.0f, (vector3){0.0f, 0.0f, 0.0f});
    rigidbody_initialisation_sphere(&bodies[1], 1.0f, 1.0f, (vector3){1.5f, 0.0f, 0.0f});
    rigidbody_initialisation_sphere(&bodies[2], 1.0f, 1.0f, (vector3){100.0f, 0.0f, 0.0f});

    bodies[0].object_id = 1;
    bodies[1].object_id = 2;
    bodies[2].object_id = 3;

    broadphase_pair pairs[64];
    int count = broadphase_generate_pairing(bodies, 3, pairs, 64);

    ASSERT_TRUE(count >= 1, "near spheres generate at least one pair");

    bool found_0_1 = false;
    for (int i = 0; i < count; i++) {
        if ((pairs[i].object_index_a == 0) && (pairs[i].object_index_b == 1)) {
            found_0_1 = true;
        }
    }
    ASSERT_TRUE(found_0_1, "expected near-sphere pair 0-1");
}

static void test_separated_no_pair(void) {
    printf("--- broadphase separated bodies ---\n");

    rigidbody bodies[2];

    rigidbody_initialisation_sphere(&bodies[0], 1.0f, 1.0f, (vector3){0.0f, 0.0f, 0.0f});
    rigidbody_initialisation_sphere(&bodies[1], 1.0f, 1.0f, (vector3){10.0f, 0.0f, 0.0f});

    bodies[0].object_id = 1;
    bodies[1].object_id = 2;

    broadphase_pair pairs[64];
    int count = broadphase_generate_pairing(bodies, 2, pairs, 64);

    ASSERT_TRUE(count == 0, "far spheres generate no pair");
}

int main(void) {
    mpe_config_init();

    printf("============================================\n");
    printf("MPE Kernel Unit Tests: broadphase\n");
    printf("============================================\n");

    test_pair_generation();
    test_separated_no_pair();

    return mpe_unit_test_summary("broadphase_test");
}

#endif /* MPE_UNIT_BROADPHASE_TEST */

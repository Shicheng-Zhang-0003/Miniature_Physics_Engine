#ifdef MPE_UNIT_SOLVER_TEST

#include "core/rigidbody.h"
#include "physics/collision_mechanics.h"
#include "config/mpe_config.h"
#include "tests/unit/test_framework.h"

static rigidbody make_sphere(float radius, float mass, vector3 position, uint32_t id) {
    rigidbody rb;
    rigidbody_initialisation_sphere(&rb, radius, mass, position);
    rb.object_id = id;
    rb.object_generation = 1;
    return rb;
}

static void test_floor_solver_stability(void) {
    printf("--- solver floor contact ---\n");

    contact_cache_clear(NULL);

    rigidbody sphere = make_sphere(0.5f, 1.0f, (vector3){0.0f, 0.4f, 0.0f}, 1);
    sphere.velocity = (vector3){0.0f, -1.0f, 0.0f};

    collision_data floor_collision = (collision_data){0};
    bool hit = collision_static_plane_body(&sphere, 0.0f, &floor_collision);
    ASSERT_TRUE(hit, "floor collision detected before solver prep");

    collision_data manifold = (collision_data){0};
    collision_prepare_solver(&floor_collision, &manifold);

    for (int i = 0; i < g_cfg.timestep.solver_iterations; i++) {
        collision_resolve_iterative(&manifold);
    }

    ASSERT_TRUE(isfinite(sphere.velocity.y), "solver leaves finite velocity");
    ASSERT_TRUE(sphere.velocity.y > -1.0f, "solver applies corrective normal impulse");
    ASSERT_TRUE(manifold.contact_count > 0, "solver manifold has contacts");
    ASSERT_TRUE(manifold.contacts[0].accumulated_normal_impulse >= 0.0f,
                "accumulated normal impulse is non-negative");
}

int main(void) {
    mpe_config_init();

    printf("============================================\n");
    printf("MPE Kernel Unit Tests: solver\n");
    printf("============================================\n");

    test_floor_solver_stability();

    return mpe_unit_test_summary("solver_test");
}

#endif /* MPE_UNIT_SOLVER_TEST */

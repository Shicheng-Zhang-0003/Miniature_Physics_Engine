#ifdef MPE_UNIT_INTEGRATION_TEST

#include "core/rigidbody.h"
#include "config/mpe_config.h"
#include "tests/unit/test_framework.h"

static rigidbody make_sphere(float radius, float mass, vector3 position, uint32_t id) {
    rigidbody rb;
    rigidbody_initialisation_sphere(&rb, radius, mass, position);
    rb.object_id = id;
    rb.object_generation = 1;
    return rb;
}

static void test_linear_integration(void) {
    printf("--- linear integration ---\n");

    const float dt = 1.0f / 60.0f;

    rigidbody rb = make_sphere(0.5f, 2.0f, (vector3){0.0f, 10.0f, 0.0f}, 1);
    rb.velocity = vector3_zero();

    vector3 gravity = {0.0f, -9.81f, 0.0f};
    rb_apply_forces_perfect(&rb, vector3_scaling(gravity, rb.mass));

    rb_integrate_velocity(&rb, dt, 1.0f, 1.0f);
    ASSERT_NEAR(rb.velocity.y, -9.81f * dt, 0.001f, "gravity produces expected velocity");

    float y_before = rb.position.y;
    rb_integrate_position(&rb, dt);
    ASSERT_TRUE(rb.position.y < y_before, "position integrates downward under gravity");
}

static void test_angular_integration(void) {
    printf("--- angular integration ---\n");

    const float dt = 1.0f / 60.0f;

    rigidbody rb = make_sphere(0.5f, 2.0f, (vector3){0.0f, 10.0f, 0.0f}, 1);
    rb.angular_velocity = (vector3){0.0f, 10.0f, 0.0f};

    rb_integrate_position(&rb, dt);

    float qlen = sqrtf(rb.orientation.w * rb.orientation.w +
                       rb.orientation.x * rb.orientation.x +
                       rb.orientation.y * rb.orientation.y +
                       rb.orientation.z * rb.orientation.z);

    ASSERT_NEAR(qlen, 1.0f, 0.001f, "orientation remains normalized after integration");
}

static void test_sanitize(void) {
    printf("--- sanitize ---\n");

    rigidbody rb = make_sphere(0.5f, 2.0f, (vector3){0.0f, 10.0f, 0.0f}, 1);
    rb.position = (vector3){NAN, 0.0f, 0.0f};

    rigidbody_sanitize(&rb);
    ASSERT_TRUE(isfinite(rb.position.x), "sanitize repairs NaN position");
    ASSERT_TRUE(isfinite(rb.position.y), "sanitize leaves finite y");
    ASSERT_TRUE(isfinite(rb.position.z), "sanitize leaves finite z");
}

int main(void) {
    mpe_config_init();

    printf("============================================\n");
    printf("MPE Kernel Unit Tests: integration\n");
    printf("============================================\n");

    test_linear_integration();
    test_angular_integration();
    test_sanitize();

    return mpe_unit_test_summary("integration_test");
}

#endif /* MPE_UNIT_INTEGRATION_TEST */

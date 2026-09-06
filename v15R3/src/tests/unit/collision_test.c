#ifdef MPE_UNIT_COLLISION_TEST

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

static rigidbody make_cube(vector3 position, vector3 half_extensions, float mass, uint32_t id) {
    rigidbody rb;
    rigidbody_initialisation_cube(&rb, position, half_extensions, mass);
    rb.object_id = id;
    rb.object_generation = 1;
    return rb;
}

static rigidbody make_cylinder(float radius, float half_length, float mass, vector3 position, uint32_t id) {
    rigidbody rb;
    rigidbody_initialisation_cylinder(&rb, radius, half_length, mass, position);
    rb.object_id = id;
    rb.object_generation = 1;
    return rb;
}

static void test_sphere_sphere(void) {
    printf("--- sphere-sphere ---\n");

    rigidbody a = make_sphere(1.0f, 1.0f, (vector3){0.0f, 0.0f, 0.0f}, 1);
    rigidbody b = make_sphere(1.0f, 1.0f, (vector3){1.5f, 0.0f, 0.0f}, 2);
    collision_data out = (collision_data){0};

    bool hit = collision_dual_sphere(&a, &b, &out);
    ASSERT_TRUE(hit, "overlapping spheres collide");
    ASSERT_NEAR(out.contacts[0].penetration, 0.5f, 0.001f, "sphere penetration depth");

    b.position = (vector3){3.0f, 0.0f, 0.0f};
    out = (collision_data){0};
    hit = collision_dual_sphere(&a, &b, &out);
    ASSERT_FALSE(hit, "separated spheres do not collide");

    b.position = a.position;
    out = (collision_data){0};
    hit = collision_dual_sphere(&a, &b, &out);
    ASSERT_TRUE(hit, "coincident spheres collide");
    ASSERT_NEAR(out.contacts[0].penetration, 2.0f, 0.001f, "coincident sphere penetration");
}

static void test_sphere_cube(void) {
    printf("--- sphere-cube ---\n");

    rigidbody cube = make_cube((vector3){0.0f, 0.0f, 0.0f},
                               (vector3){0.5f, 0.5f, 0.5f},
                               1.0f, 2);
    rigidbody sphere = make_sphere(0.5f, 1.0f, (vector3){0.9f, 0.0f, 0.0f}, 1);
    collision_data out = (collision_data){0};

    bool hit = collision_sphere_cube(&sphere, &cube, &out);
    ASSERT_TRUE(hit, "sphere penetrating cube face collides");
    ASSERT_TRUE(out.contact_count == 1, "sphere-cube produces one contact");
    ASSERT_NEAR(out.contacts[0].penetration, 0.1f, 0.02f, "sphere-cube penetration");

    sphere.position = (vector3){2.0f, 0.0f, 0.0f};
    out = (collision_data){0};
    hit = collision_sphere_cube(&sphere, &cube, &out);
    ASSERT_FALSE(hit, "separated sphere-cube does not collide");

    sphere.position = (vector3){0.0f, 0.0f, 0.0f};
    out = (collision_data){0};
    hit = collision_sphere_cube(&sphere, &cube, &out);
    ASSERT_TRUE(hit, "sphere inside cube collides");
    ASSERT_TRUE(out.contacts[0].penetration > 0.0f, "inside-case penetration positive");
}

static void test_cube_cube(void) {
    printf("--- cube-cube ---\n");

    rigidbody a = make_cube((vector3){0.0f, 0.0f, 0.0f},
                            (vector3){0.5f, 0.5f, 0.5f},
                            1.0f, 1);
    rigidbody b = make_cube((vector3){0.8f, 0.0f, 0.0f},
                            (vector3){0.5f, 0.5f, 0.5f},
                            1.0f, 2);
    collision_data out = (collision_data){0};

    bool hit = collision_dual_cube(&a, &b, &out);
    ASSERT_TRUE(hit, "overlapping axis-aligned cubes collide");
    ASSERT_TRUE(out.contact_count >= 1, "cube-cube produces contacts");
    ASSERT_NEAR(out.contacts[0].penetration, 0.2f, 0.03f, "cube-cube penetration depth");

    vector3 a_to_b = vector3_subtraction(b.position, a.position);
    ASSERT_TRUE(vector3_dot(out.normal_vector, a_to_b) > 0.0f,
                "cube-cube normal points from A toward B");

    b.position = (vector3){2.0f, 0.0f, 0.0f};
    out = (collision_data){0};
    hit = collision_dual_cube(&a, &b, &out);
    ASSERT_FALSE(hit, "separated cubes do not collide");
}

static void test_cylinder_collisions(void) {
    printf("--- cylinder collisions ---\n");

    rigidbody cyl = make_cylinder(0.05f, 0.02f, 1.0f, (vector3){0.0f, 0.0f, 0.0f}, 1);
    rigidbody sph = make_sphere(0.08f, 1.0f, (vector3){0.0f, 0.0f, 0.10f}, 2);
    collision_data out = (collision_data){0};

    bool hit = collision_cylinder_sphere(&cyl, &sph, &out);
    ASSERT_TRUE(hit, "cylinder-sphere overlap collides");
    ASSERT_TRUE(out.contacts[0].penetration > 0.0f, "cylinder-sphere penetration positive");

    sph.position = (vector3){0.0f, 0.0f, 0.5f};
    out = (collision_data){0};
    hit = collision_cylinder_sphere(&cyl, &sph, &out);
    ASSERT_FALSE(hit, "separated cylinder-sphere does not collide");

    rigidbody cube = make_cube((vector3){0.0f, 0.0f, 0.5f},
                               (vector3){0.5f, 0.25f, 0.1f},
                               0.0f, 3);
    rigidbody cyl2 = make_cylinder(0.05f, 0.02f, 1.0f, (vector3){0.0f, 0.0f, 0.38f}, 4);
    out = (collision_data){0};
    hit = collision_cylinder_cube(&cyl2, &cube, &out);
    ASSERT_TRUE(hit, "cylinder near cube wall collides");

    cyl2.position = (vector3){0.0f, 0.0f, -2.0f};
    out = (collision_data){0};
    hit = collision_cylinder_cube(&cyl2, &cube, &out);
    ASSERT_FALSE(hit, "far cylinder-cube does not collide");

    rigidbody c1 = make_cylinder(0.05f, 0.02f, 1.0f, (vector3){0.0f, 0.0f, -0.03f}, 5);
    rigidbody c2 = make_cylinder(0.05f, 0.02f, 1.0f, (vector3){0.0f, 0.0f, 0.03f}, 6);
    out = (collision_data){0};
    hit = collision_cylinder_cylinder(&c1, &c2, &out);
    ASSERT_TRUE(hit, "overlapping cylinders collide");

    c1.position = (vector3){0.0f, 0.0f, -0.5f};
    c2.position = (vector3){0.0f, 0.0f, 0.5f};
    out = (collision_data){0};
    hit = collision_cylinder_cylinder(&c1, &c2, &out);
    ASSERT_FALSE(hit, "separated cylinders do not collide");
}

static void test_floor_collisions(void) {
    printf("--- floor collisions ---\n");

    rigidbody sphere = make_sphere(0.5f, 1.0f, (vector3){0.0f, 0.4f, 0.0f}, 1);
    collision_data out = (collision_data){0};
    bool hit = collision_static_plane_body(&sphere, 0.0f, &out);
    ASSERT_TRUE(hit, "sphere below floor collides");
    ASSERT_NEAR(out.contacts[0].penetration, 0.1f, 0.02f, "sphere floor penetration");

    rigidbody cube = make_cube((vector3){0.0f, 0.4f, 0.0f},
                               (vector3){0.5f, 0.5f, 0.5f},
                               1.0f, 2);
    out = (collision_data){0};
    hit = collision_static_plane_body(&cube, 0.0f, &out);
    ASSERT_TRUE(hit, "cube below floor collides");
    ASSERT_TRUE(out.contact_count >= 1, "cube floor produces contacts");

    rigidbody cyl = make_cylinder(0.05f, 0.02f, 1.0f, (vector3){0.0f, 0.04f, 0.0f}, 3);
    out = (collision_data){0};
    hit = collision_static_plane_body(&cyl, 0.0f, &out);
    ASSERT_TRUE(hit, "cylinder below floor collides");
    ASSERT_TRUE(out.contact_count >= 1, "cylinder floor produces contacts");
}

int main(void) {
    mpe_config_init();

    printf("============================================\n");
    printf("MPE Kernel Unit Tests: collision\n");
    printf("============================================\n");

    test_sphere_sphere();
    test_sphere_cube();
    test_cube_cube();
    test_cylinder_collisions();
    test_floor_collisions();

    return mpe_unit_test_summary("collision_test");
}

#endif /* MPE_UNIT_COLLISION_TEST */

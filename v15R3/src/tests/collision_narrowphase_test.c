/* Kernel collision narrowphase unit tests. Build: make test_collision_narrowphase */
#ifdef MPE_COLLISION_NARROWPHASE_TEST
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "core/rigidbody.h"
#include "physics/collision_mechanics.h"

static int tests_run = 0, tests_passed = 0, tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  [PASS] %s\n", msg); } \
    else { tests_failed++; printf("  [FAIL] %s\n", msg); } \
} while(0)

#define ASSERT_NEAR(a, b, eps, msg) do { \
    tests_run++; \
    if (fabsf((float)(a) - (float)(b)) < (eps)) { tests_passed++; } \
    else { tests_failed++; printf("  [FAIL] %s: got %.6f, expected %.6f\n", msg, (float)(a), (float)(b)); } \
} while(0)

/* ------------------------------------------------------------------
* Test 1: Sphere-Sphere collision
* ------------------------------------------------------------------ */
static void test_sphere_sphere(void) {
    printf("--- Sphere-Sphere ---\n");

    rigidbody a, b;
    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));
    a.type = object_sphere;
    b.type = object_sphere;
    a.radius = 1.0f;
    b.radius = 1.0f;
    a.object_id = 1;
    b.object_id = 2;

    /* Overlapping: centers 1.5 apart, radii sum = 2.0 */
    a.position = (vector3){0.0f, 0.0f, 0.0f};
    b.position = (vector3){1.5f, 0.0f, 0.0f};

    collision_data out = {0};
    bool hit = collision_dual_sphere(&a, &b, &out);
    TEST_ASSERT(hit, "overlapping spheres collide");
    ASSERT_NEAR(out.contacts[0].penetration, 0.5f, 0.01f, "penetration = 0.5");
    ASSERT_NEAR(out.normal_vector.x, 1.0f, 0.01f, "normal points A->B");
    ASSERT_NEAR(out.normal_vector.y, 0.0f, 0.01f, "normal y = 0");

    /* Separated: centers 3.0 apart, radii sum = 2.0 */
    b.position = (vector3){3.0f, 0.0f, 0.0f};
    memset(&out, 0, sizeof(out));
    hit = collision_dual_sphere(&a, &b, &out);
    TEST_ASSERT(!hit, "separated spheres don't collide");

    /* Coincident centers */
    b.position = a.position;
    memset(&out, 0, sizeof(out));
    hit = collision_dual_sphere(&a, &b, &out);
    TEST_ASSERT(hit, "coincident spheres collide");
    ASSERT_NEAR(out.contacts[0].penetration, 2.0f, 0.01f, "coincident penetration = 2*r");

    /* Different radii */
    a.radius = 0.5f;
    b.radius = 2.0f;
    a.position = (vector3){0.0f, 0.0f, 0.0f};
    b.position = (vector3){2.0f, 0.0f, 0.0f};
    memset(&out, 0, sizeof(out));
    hit = collision_dual_sphere(&a, &b, &out);
    TEST_ASSERT(hit, "different radii overlap (0.5+2.0=2.5 > 2.0)");
    ASSERT_NEAR(out.contacts[0].penetration, 0.5f, 0.01f, "penetration = 2.5-2.0 = 0.5");
}

/* ------------------------------------------------------------------
* Test 2: Sphere-Cube collision
* ------------------------------------------------------------------ */
static void test_sphere_cube(void) {
    printf("--- Sphere-Cube ---\n");

    rigidbody sphere, cube;
    memset(&sphere, 0, sizeof(sphere));
    memset(&cube, 0, sizeof(cube));
    sphere.type = object_sphere;
    cube.type = object_cube;
    sphere.radius = 0.5f;
    cube.half_extensions = (vector3){1.0f, 1.0f, 1.0f};
    sphere.object_id = 1;
    cube.object_id = 2;
    cube.orientation = vector4_identity();
    rigidbody_update_axes(&cube);

    /* Sphere touching cube face: sphere center at (2.0, 0, 0), cube at origin */
    /* Cube face at x=1.0, sphere center at 2.0, radius 0.5 -> gap = 0.5 */
    sphere.position = (vector3){2.0f, 0.0f, 0.0f};
    cube.position = (vector3){0.0f, 0.0f, 0.0f};

    collision_data out = {0};
    bool hit = collision_sphere_cube(&sphere, &cube, &out);
    TEST_ASSERT(!hit, "sphere 0.5 away from cube face doesn't collide");

    /* Sphere penetrating cube face */
    sphere.position = (vector3){1.2f, 0.0f, 0.0f};
    memset(&out, 0, sizeof(out));
    hit = collision_sphere_cube(&sphere, &cube, &out);
    TEST_ASSERT(hit, "sphere penetrating cube face collides");
    ASSERT_NEAR(out.contacts[0].penetration, 0.3f, 0.02f, "penetration ~0.3");
    ASSERT_NEAR(out.normal_vector.x, -1.0f, 0.02f, "normal points from sphere toward cube (B)");

    /* Sphere inside cube */
    sphere.position = (vector3){0.0f, 0.0f, 0.0f};
    memset(&out, 0, sizeof(out));
    hit = collision_sphere_cube(&sphere, &cube, &out);
    TEST_ASSERT(hit, "sphere inside cube collides");
    TEST_ASSERT(out.contacts[0].penetration > 0.0f, "penetration positive inside");
}

/* ------------------------------------------------------------------
* Test 3: Cube-Cube collision (SAT)
* ------------------------------------------------------------------ */
static void test_cube_cube(void) {
    printf("--- Cube-Cube (SAT) ---\n");

    rigidbody a, b;
    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));
    a.type = object_cube;
    b.type = object_cube;
    a.half_extensions = (vector3){0.5f, 0.5f, 0.5f};
    b.half_extensions = (vector3){0.5f, 0.5f, 0.5f};
    a.object_id = 1;
    b.object_id = 2;
    a.orientation = vector4_identity();
    b.orientation = vector4_identity();
    rigidbody_update_axes(&a);
    rigidbody_update_axes(&b);

    /* Axis-aligned cubes overlapping along X */
    a.position = (vector3){0.0f, 0.0f, 0.0f};
    b.position = (vector3){0.8f, 0.0f, 0.0f};

    collision_data out = {0};
    bool hit = collision_dual_cube(&a, &b, &out);
    TEST_ASSERT(hit, "axis-aligned overlapping cubes collide");
    TEST_ASSERT(out.contact_count >= 1, "at least 1 contact point");
    ASSERT_NEAR(out.contacts[0].penetration, 0.2f, 0.02f, "penetration ~0.2");

    /* Separated cubes */
    b.position = (vector3){2.0f, 0.0f, 0.0f};
    memset(&out, 0, sizeof(out));
    hit = collision_dual_cube(&a, &b, &out);
    TEST_ASSERT(!hit, "separated cubes don't collide");

    /* Stacked cubes: one on top of the other */
    a.position = (vector3){0.0f, 0.5f, 0.0f};
    b.position = (vector3){0.0f, -0.5f, 0.0f};
    memset(&out, 0, sizeof(out));
    hit = collision_dual_cube(&a, &b, &out);
    TEST_ASSERT(hit, "stacked cubes collide");

    /* Edge-on contact: one cube rotated 45 degrees */
    a.position = (vector3){0.0f, 0.0f, 0.0f};
    b.position = (vector3){1.0f, 0.0f, 0.0f};
    /* Rotate b by 45 degrees around Y axis */
    vector4 rot45 = vector4_from_axis_with_angle((vector3){0.0f, 1.0f, 0.0f}, 0.785398f);
    b.orientation = rot45;
    rigidbody_update_axes(&b);
    memset(&out, 0, sizeof(out));
    hit = collision_dual_cube(&a, &b, &out);
    TEST_ASSERT(hit, "rotated cube overlapping collides");
}

/* ------------------------------------------------------------------
* Test 4: Cylinder-Sphere collision
* ------------------------------------------------------------------ */
static void test_cylinder_sphere(void) {
    printf("--- Cylinder-Sphere ---\n");

    rigidbody cyl, sph;
    memset(&cyl, 0, sizeof(cyl));
    memset(&sph, 0, sizeof(sph));
    cyl.type = object_cylinder;
    sph.type = object_sphere;
    cyl.radius = 0.05f;
    cyl.cylinder_half_length = 0.02f;
    sph.radius = 0.08f;
    cyl.object_id = 1;
    sph.object_id = 2;
    cyl.orientation = vector4_identity();
    rigidbody_update_axes(&cyl);

    /* Sphere approaching cylinder along Z */
    cyl.position = (vector3){0.0f, 0.0f, 0.0f};
    sph.position = (vector3){0.0f, 0.0f, 0.10f}; /* gap = 0.10 - 0.05 - 0.08 = -0.03 -> overlap */

    collision_data out = {0};
    bool hit = collision_cylinder_sphere(&cyl, &sph, &out);
    TEST_ASSERT(hit, "sphere overlapping cylinder collides");
    TEST_ASSERT(out.contacts[0].penetration > 0.0f, "positive penetration");

    /* Separated */
    sph.position = (vector3){0.0f, 0.0f, 0.5f};
    memset(&out, 0, sizeof(out));
    hit = collision_cylinder_sphere(&cyl, &sph, &out);
    TEST_ASSERT(!hit, "separated sphere-cylinder don't collide");
}

/* ------------------------------------------------------------------
* Test 5: Cylinder-Cube collision
* ------------------------------------------------------------------ */
static void test_cylinder_cube(void) {
    printf("--- Cylinder-Cube ---\n");

    rigidbody cyl, cube;
    memset(&cyl, 0, sizeof(cyl));
    memset(&cube, 0, sizeof(cube));
    cyl.type = object_cylinder;
    cube.type = object_cube;
    cyl.radius = 0.05f;
    cyl.cylinder_half_length = 0.02f;
    cube.half_extensions = (vector3){0.5f, 0.25f, 0.1f};
    cyl.object_id = 1;
    cube.object_id = 2;
    cyl.orientation = vector4_identity();
    cube.orientation = vector4_identity();
    rigidbody_update_axes(&cyl);
    rigidbody_update_axes(&cube);

    /* Cylinder near cube wall */
    cube.position = (vector3){0.0f, 0.25f, 0.5f};
    cyl.position = (vector3){0.0f, 0.06f, 0.33f}; /* close to cube face at z=0.4 */

    collision_data out = {0};
    bool hit = collision_cylinder_cube(&cyl, &cube, &out);
    TEST_ASSERT(hit, "cylinder near cube wall collides");
    TEST_ASSERT(out.contacts[0].penetration > 0.0f, "positive penetration");

    /* Far away */
    cyl.position = (vector3){0.0f, 0.06f, -2.0f};
    memset(&out, 0, sizeof(out));
    hit = collision_cylinder_cube(&cyl, &cube, &out);
    TEST_ASSERT(!hit, "far cylinder-cube don't collide");
}

/* ------------------------------------------------------------------
* Test 6: Cylinder-Cylinder collision
* ------------------------------------------------------------------ */
static void test_cylinder_cylinder(void) {
    printf("--- Cylinder-Cylinder ---\n");

    rigidbody c1, c2;
    memset(&c1, 0, sizeof(c1));
    memset(&c2, 0, sizeof(c2));
    c1.type = object_cylinder;
    c2.type = object_cylinder;
    c1.radius = 0.05f;
    c2.radius = 0.05f;
    c1.cylinder_half_length = 0.02f;
    c2.cylinder_half_length = 0.02f;
    c1.object_id = 1;
    c2.object_id = 2;
    c1.orientation = vector4_identity();
    c2.orientation = vector4_identity();
    rigidbody_update_axes(&c1);
    rigidbody_update_axes(&c2);

    /* Overlapping along Z */
    c1.position = (vector3){0.0f, 0.0f, -0.03f};
    c2.position = (vector3){0.0f, 0.0f, 0.03f};

    collision_data out = {0};
    bool hit = collision_cylinder_cylinder(&c1, &c2, &out);
    TEST_ASSERT(hit, "overlapping cylinders collide");
    TEST_ASSERT(out.contacts[0].penetration > 0.0f, "positive penetration");

    /* Separated */
    c1.position = (vector3){0.0f, 0.0f, -0.5f};
    c2.position = (vector3){0.0f, 0.0f, 0.5f};
    memset(&out, 0, sizeof(out));
    hit = collision_cylinder_cylinder(&c1, &c2, &out);
    TEST_ASSERT(!hit, "separated cylinders don't collide");
}

/* ------------------------------------------------------------------
* Test 7: Floor collision
* ------------------------------------------------------------------ */
static void test_floor_collision(void) {
    printf("--- Floor Collision ---\n");

    /* Sphere on floor */
    rigidbody sphere;
    memset(&sphere, 0, sizeof(sphere));
    sphere.type = object_sphere;
    sphere.radius = 0.5f;
    sphere.object_id = 1;

    sphere.position = (vector3){0.0f, 0.3f, 0.0f}; /* bottom at y=-0.2, below floor y=0 */
    collision_data out = {0};
    bool hit = collision_static_plane_body(&sphere, 0.0f, &out);
    TEST_ASSERT(hit, "sphere below floor collides");
    ASSERT_NEAR(out.contacts[0].penetration, 0.2f, 0.02f, "sphere penetration = 0.2");

    /* Sphere resting on floor (bottom exactly at y=0) */
    sphere.position = (vector3){0.0f, 0.5f, 0.0f};
    memset(&out, 0, sizeof(out));
    hit = collision_static_plane_body(&sphere, 0.0f, &out);
    TEST_ASSERT(!hit, "sphere resting exactly on floor doesn't penetrate");

    /* Cube on floor */
    rigidbody cube;
    memset(&cube, 0, sizeof(cube));
    cube.type = object_cube;
    cube.half_extensions = (vector3){0.5f, 0.5f, 0.5f};
    cube.object_id = 2;
    cube.orientation = vector4_identity();
    rigidbody_update_axes(&cube);

    cube.position = (vector3){0.0f, 0.4f, 0.0f}; /* bottom face at y=-0.1, below floor */
    memset(&out, 0, sizeof(out));
    hit = collision_static_plane_body(&cube, 0.0f, &out);
    TEST_ASSERT(hit, "cube below floor collides");
    TEST_ASSERT(out.contact_count >= 1, "cube has contact points");

    /* Cylinder on floor */
    rigidbody cyl;
    memset(&cyl, 0, sizeof(cyl));
    cyl.type = object_cylinder;
    cyl.radius = 0.05f;
    cyl.cylinder_half_length = 0.02f;
    cyl.object_id = 3;
    cyl.orientation = vector4_identity();
    rigidbody_update_axes(&cyl);

    cyl.position = (vector3){0.0f, 0.04f, 0.0f}; /* axle at y=0.04, bottom at y=-0.01 */
    memset(&out, 0, sizeof(out));
    hit = collision_static_plane_body(&cyl, 0.0f, &out);
    TEST_ASSERT(hit, "cylinder below floor collides");
    TEST_ASSERT(out.contact_count >= 1, "cylinder has contact points");
}

/* ------------------------------------------------------------------
* Main
* ------------------------------------------------------------------ */
int main(void) {
    printf("============================================\n");
    printf("MPE Collision Narrowphase Unit Tests\n");
    printf("============================================\n");

    test_sphere_sphere();
    test_sphere_cube();
    test_cube_cube();
    test_cylinder_sphere();
    test_cylinder_cube();
    test_cylinder_cylinder();
    test_floor_collision();

    printf("\n============================================\n");
    printf("SUMMARY: %d run, %d passed, %d failed\n", tests_run, tests_passed, tests_failed);
    printf("============================================\n");
    return tests_failed > 0 ? 1 : 0;
}
#endif /* MPE_COLLISION_NARROWPHASE_TEST */

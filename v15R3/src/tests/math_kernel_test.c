/* Kernel math unit tests. Build: make test_math_kernel */
#ifdef MPE_MATH_KERNEL_TEST
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "core/math3D.h"
#include "core/math4_special.h"

static int tests_run = 0, tests_passed = 0, tests_failed = 0;

#define ASSERT_NEAR(a, b, eps, msg) do { \
    tests_run++; \
    if (fabsf((a) - (b)) < (eps)) { tests_passed++; } \
    else { tests_failed++; printf("  [FAIL] %s: got %.6f, expected %.6f\n", msg, (float)(a), (float)(b)); } \
} while(0)

#define ASSERT_TRUE(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; } \
    else { tests_failed++; printf("  [FAIL] %s\n", msg); } \
} while(0)

/* ---- vector3 tests ---- */
static void test_vector3_basic(void) {
    printf("--- vector3 basics ---\n");
    vector3 a = {1.0f, 2.0f, 3.0f};
    vector3 b = {4.0f, 5.0f, 6.0f};

    vector3 sum = vector3_addition(a, b);
    ASSERT_NEAR(sum.x, 5.0f, 0.001f, "add x");
    ASSERT_NEAR(sum.y, 7.0f, 0.001f, "add y");
    ASSERT_NEAR(sum.z, 9.0f, 0.001f, "add z");

    vector3 diff = vector3_subtraction(a, b);
    ASSERT_NEAR(diff.x, -3.0f, 0.001f, "sub x");

    vector3 scaled = vector3_scaling(a, 2.0f);
    ASSERT_NEAR(scaled.x, 2.0f, 0.001f, "scale x");
    ASSERT_NEAR(scaled.z, 6.0f, 0.001f, "scale z");

    float dot = vector3_dot(a, b);
    ASSERT_NEAR(dot, 32.0f, 0.001f, "dot product");

    vector3 cross = vector3_cross(a, b);
    ASSERT_NEAR(cross.x, -3.0f, 0.001f, "cross x");
    ASSERT_NEAR(cross.y, 6.0f, 0.001f, "cross y");
    ASSERT_NEAR(cross.z, -3.0f, 0.001f, "cross z");

    float len = vector3_length(a);
    ASSERT_NEAR(len, sqrtf(14.0f), 0.001f, "length");

    vector3 norm = vector3_normalisation(a);
    ASSERT_NEAR(vector3_length(norm), 1.0f, 0.001f, "normalized length");

    vector3 zero = vector3_zero();
    vector3 norm_zero = vector3_normalisation(zero);
    ASSERT_NEAR(vector3_length(norm_zero), 0.0f, 0.001f, "normalize zero vector");
}

/* ---- math3 inverse tests ---- */
static void test_math3_inverse(void) {
    printf("--- math3_inverse ---\n");

    /* Identity */
    math3 identity = math3_identity();
    math3 inv_identity = math3_inverse(identity);
    ASSERT_NEAR(inv_identity.matrix[0][0], 1.0f, 0.001f, "inv(I)[0][0]");
    ASSERT_NEAR(inv_identity.matrix[1][1], 1.0f, 0.001f, "inv(I)[1][1]");
    ASSERT_NEAR(inv_identity.matrix[2][2], 1.0f, 0.001f, "inv(I)[2][2]");
    ASSERT_NEAR(inv_identity.matrix[0][1], 0.0f, 0.001f, "inv(I)[0][1]");

    /* Diagonal */
    math3 diag = {{{2.0f, 0, 0}, {0, 4.0f, 0}, {0, 0, 8.0f}}};
    math3 inv_diag = math3_inverse(diag);
    ASSERT_NEAR(inv_diag.matrix[0][0], 0.5f, 0.001f, "inv(diag)[0][0]");
    ASSERT_NEAR(inv_diag.matrix[1][1], 0.25f, 0.001f, "inv(diag)[1][1]");
    ASSERT_NEAR(inv_diag.matrix[2][2], 0.125f, 0.001f, "inv(diag)[2][2]");

    /* Small inertia tensor (cylinder wheel case from MPE_FTC_093g) */
    math3 small = {{{0.000625f, 0, 0}, {0, 0.000379f, 0}, {0, 0, 0.000379f}}};
    math3 inv_small = math3_inverse(small);
    ASSERT_NEAR(inv_small.matrix[0][0], 1600.0f, 1.0f, "inv(small)[0][0]");
    ASSERT_NEAR(inv_small.matrix[1][1], 2638.5f, 1.0f, "inv(small)[1][1]");

    /* Round-trip: A * A^-1 = I */
    math3 product = math3_multiplication(diag, inv_diag);
    ASSERT_NEAR(product.matrix[0][0], 1.0f, 0.01f, "A*A^-1 [0][0]");
    ASSERT_NEAR(product.matrix[1][1], 1.0f, 0.01f, "A*A^-1 [1][1]");
    ASSERT_NEAR(product.matrix[0][1], 0.0f, 0.01f, "A*A^-1 [0][1]");

    /* Singular matrix returns zero */
    math3 singular = {{{0}}};
    math3 inv_singular = math3_inverse(singular);
    ASSERT_NEAR(inv_singular.matrix[0][0], 0.0f, 0.001f, "inv(singular) = 0");
}

/* ---- quaternion tests ---- */
static void test_quaternion(void) {
    printf("--- quaternion ---\n");

    vector4 ident = vector4_identity();
    ASSERT_NEAR(ident.w, 1.0f, 0.001f, "identity w");
    ASSERT_NEAR(ident.x, 0.0f, 0.001f, "identity x");

    /* Normalize */
    vector4 q = {2.0f, 0.0f, 0.0f, 0.0f};
    vector4 qn = vector4_normalisation(q);
    ASSERT_NEAR(qn.w, 1.0f, 0.001f, "normalize w");

    /* Rotation: 90 degrees around Y axis */
    vector3 axis = {0.0f, 1.0f, 0.0f};
    float angle = math_pi / 2.0f;
    vector4 rot = vector4_from_axis_with_angle(axis, angle);

    /* Rotate X axis by 90 around Y -> should give Z axis */
    vector3 x_axis = {1.0f, 0.0f, 0.0f};
    vector3 rotated = vector4_rotate_to_vector3(rot, x_axis);
    ASSERT_NEAR(rotated.x, 0.0f, 0.01f, "rot90y x->z: x");
    ASSERT_NEAR(rotated.z, -1.0f, 0.01f, "rot90y x->z: z");

    /* Quaternion multiplication: two 90-degree rotations = 180 degrees */
    vector4 rot2 = vector4_multiplication(rot, rot);
    vector3 rotated2 = vector4_rotate_to_vector3(rot2, x_axis);
    ASSERT_NEAR(rotated2.x, -1.0f, 0.01f, "rot180y x: x");
    ASSERT_NEAR(rotated2.z, 0.0f, 0.02f, "rot180y x: z");
}

/* ---- math4 tests ---- */
static void test_math4(void) {
    printf("--- math4 ---\n");

    math4 ident = math4_identity();
    ASSERT_NEAR(ident.matrix[0][0], 1.0f, 0.001f, "I[0][0]");
    ASSERT_NEAR(ident.matrix[3][3], 1.0f, 0.001f, "I[3][3]");

    /* Translation */
    math4 trans = math4_translation((vector3){5.0f, 10.0f, 15.0f});
    ASSERT_NEAR(trans.matrix[3][0], 5.0f, 0.001f, "trans x");
    ASSERT_NEAR(trans.matrix[3][1], 10.0f, 0.001f, "trans y");
    ASSERT_NEAR(trans.matrix[3][2], 15.0f, 0.001f, "trans z");

    /* Scaling */
    math4 scale = math4_scaling((vector3){2.0f, 3.0f, 4.0f});
    ASSERT_NEAR(scale.matrix[0][0], 2.0f, 0.001f, "scale x");
    ASSERT_NEAR(scale.matrix[1][1], 3.0f, 0.001f, "scale y");
    ASSERT_NEAR(scale.matrix[2][2], 4.0f, 0.001f, "scale z");

    /* Quaternion to matrix: identity quaternion -> identity matrix */
    vector4 q_ident = vector4_identity();
    math4 m_from_q = vector4_to_math4(q_ident);
    ASSERT_NEAR(m_from_q.matrix[0][0], 1.0f, 0.001f, "q->m [0][0]");
    ASSERT_NEAR(m_from_q.matrix[0][1], 0.0f, 0.001f, "q->m [0][1]");
}

/* ---- collision: sphere-sphere ---- */
#include "physics/collision_mechanics.h"

static void test_sphere_sphere_collision(void) {
    printf("--- collision_dual_sphere ---\n");

    rigidbody a, b;
    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));
    a.type = object_sphere;
    b.type = object_sphere;
    a.radius = 1.0f;
    b.radius = 1.0f;
    a.position = (vector3){0.0f, 0.0f, 0.0f};
    b.position = (vector3){1.5f, 0.0f, 0.0f};
    a.object_id = 1;
    b.object_id = 2;

    collision_data out = {0};
    bool hit = collision_dual_sphere(&a, &b, &out);
    ASSERT_TRUE(hit, "overlapping spheres collide");
    ASSERT_NEAR(out.contacts[0].penetration, 0.5f, 0.01f, "penetration depth");
    ASSERT_NEAR(out.normal_vector.x, 1.0f, 0.01f, "normal points A->B");

    /* Non-overlapping */
    b.position = (vector3){3.0f, 0.0f, 0.0f};
    memset(&out, 0, sizeof(out));
    hit = collision_dual_sphere(&a, &b, &out);
    ASSERT_TRUE(!hit, "separated spheres don't collide");

    /* Coincident centers */
    b.position = (vector3){0.0f, 0.0f, 0.0f};
    memset(&out, 0, sizeof(out));
    hit = collision_dual_sphere(&a, &b, &out);
    ASSERT_TRUE(hit, "coincident spheres collide");
    ASSERT_NEAR(out.contacts[0].penetration, 2.0f, 0.01f, "coincident penetration");
}

int main(void) {
    printf("============================================\n");
    printf("MPE Kernel Math Unit Tests\n");
    printf("============================================\n");

    test_vector3_basic();
    test_math3_inverse();
    test_quaternion();
    test_math4();
    test_sphere_sphere_collision();

    printf("\n============================================\n");
    printf("SUMMARY: %d run, %d passed, %d failed\n", tests_run, tests_passed, tests_failed);
    printf("============================================\n");
    return tests_failed > 0 ? 1 : 0;
}
#endif /* MPE_MATH_KERNEL_TEST */

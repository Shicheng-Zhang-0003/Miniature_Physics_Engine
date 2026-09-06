#ifdef MPE_UNIT_MATH_TEST

#include "core/math3D.h"
#include "core/math4_special.h"
#include "tests/unit/test_framework.h"

static void test_vector3(void) {
    printf("--- vector3 ---\n");

    vector3 a = {1.0f, 2.0f, 3.0f};
    vector3 b = {4.0f, 5.0f, 6.0f};

    vector3 sum = vector3_addition(a, b);
    ASSERT_FLOAT_EQ(sum.x, 5.0f, 0.0001f, "vector3 addition x");
    ASSERT_FLOAT_EQ(sum.y, 7.0f, 0.0001f, "vector3 addition y");
    ASSERT_FLOAT_EQ(sum.z, 9.0f, 0.0001f, "vector3 addition z");

    float dot = vector3_dot(a, b);
    ASSERT_FLOAT_EQ(dot, 32.0f, 0.0001f, "vector3 dot product");

    vector3 cross = vector3_cross(a, b);
    ASSERT_FLOAT_EQ(cross.x, -3.0f, 0.0001f, "vector3 cross x");
    ASSERT_FLOAT_EQ(cross.y, 6.0f, 0.0001f, "vector3 cross y");
    ASSERT_FLOAT_EQ(cross.z, -3.0f, 0.0001f, "vector3 cross z");

    float len = vector3_length(a);
    ASSERT_FLOAT_EQ(len, sqrtf(14.0f), 0.0001f, "vector3 length");

    vector3 norm = vector3_normalisation(a);
    ASSERT_FLOAT_EQ(vector3_length(norm), 1.0f, 0.0001f, "vector3 normalisation length");

    vector3 zero = vector3_zero();
    vector3 norm_zero = vector3_normalisation(zero);
    ASSERT_FLOAT_EQ(vector3_length(norm_zero), 0.0f, 0.0001f, "zero vector normalisation safe");
}

static void test_quaternion(void) {
    printf("--- quaternion ---\n");

    vector4 identity = vector4_identity();
    ASSERT_FLOAT_EQ(identity.w, 1.0f, 0.0001f, "quaternion identity w");
    ASSERT_FLOAT_EQ(identity.x, 0.0f, 0.0001f, "quaternion identity x");
    ASSERT_FLOAT_EQ(identity.y, 0.0f, 0.0001f, "quaternion identity y");
    ASSERT_FLOAT_EQ(identity.z, 0.0f, 0.0001f, "quaternion identity z");

    vector3 axis = {0.0f, 1.0f, 0.0f};
    vector4 rot90 = vector4_from_axis_with_angle(axis, math_pi / 2.0f);
    vector3 x_axis = {1.0f, 0.0f, 0.0f};
    vector3 rotated = vector4_rotate_to_vector3(rot90, x_axis);

    ASSERT_FLOAT_EQ(vector3_length(rotated), 1.0f, 0.001f, "rotated vector remains unit length");
    ASSERT_TRUE(fabsf(rotated.y) < 0.01f, "90-degree Y rotation keeps X axis in XZ plane");
    ASSERT_TRUE(fabsf(fabsf(rotated.z) - 1.0f) < 0.01f, "90-degree Y rotation maps X toward Z axis");

    vector4 rot180 = vector4_multiplication(rot90, rot90);
    vector3 rotated180 = vector4_rotate_to_vector3(rot180, x_axis);
    ASSERT_TRUE(fabsf(fabsf(rotated180.x) - 1.0f) < 0.01f, "180-degree Y rotation flips X axis");
    ASSERT_TRUE(fabsf(rotated180.z) < 0.02f, "180-degree Y rotation has near-zero Z");
}

static void test_math3_inverse(void) {
    printf("--- math3_inverse ---\n");

    math3 identity = math3_identity();
    math3 inv_identity = math3_inverse(identity);
    ASSERT_FLOAT_EQ(inv_identity.matrix[0][0], 1.0f, 0.0001f, "inverse identity [0][0]");
    ASSERT_FLOAT_EQ(inv_identity.matrix[1][1], 1.0f, 0.0001f, "inverse identity [1][1]");
    ASSERT_FLOAT_EQ(inv_identity.matrix[2][2], 1.0f, 0.0001f, "inverse identity [2][2]");
    ASSERT_FLOAT_EQ(inv_identity.matrix[0][1], 0.0f, 0.0001f, "inverse identity [0][1]");

    math3 diagonal = {{{2.0f, 0.0f, 0.0f},
                       {0.0f, 4.0f, 0.0f},
                       {0.0f, 0.0f, 8.0f}}};
    math3 inv_diagonal = math3_inverse(diagonal);
    ASSERT_FLOAT_EQ(inv_diagonal.matrix[0][0], 0.5f, 0.0001f, "inverse diagonal [0][0]");
    ASSERT_FLOAT_EQ(inv_diagonal.matrix[1][1], 0.25f, 0.0001f, "inverse diagonal [1][1]");
    ASSERT_FLOAT_EQ(inv_diagonal.matrix[2][2], 0.125f, 0.0001f, "inverse diagonal [2][2]");

    math3 product = math3_multiplication(diagonal, inv_diagonal);
    ASSERT_FLOAT_EQ(product.matrix[0][0], 1.0f, 0.001f, "A * inv(A) [0][0]");
    ASSERT_FLOAT_EQ(product.matrix[1][1], 1.0f, 0.001f, "A * inv(A) [1][1]");
    ASSERT_FLOAT_EQ(product.matrix[2][2], 1.0f, 0.001f, "A * inv(A) [2][2]");
    ASSERT_FLOAT_EQ(product.matrix[0][1], 0.0f, 0.001f, "A * inv(A) [0][1]");

    math3 small_inertia = {{{0.000625f, 0.0f, 0.0f},
                            {0.0f, 0.000379f, 0.0f},
                            {0.0f, 0.0f, 0.000379f}}};
    math3 inv_small = math3_inverse(small_inertia);
    ASSERT_NEAR(inv_small.matrix[0][0], 1600.0f, 1.0f, "small tensor inverse [0][0]");
    ASSERT_NEAR(inv_small.matrix[1][1], 2638.5f, 2.0f, "small tensor inverse [1][1]");

    math3 singular = {{{0.0f}}};
    math3 inv_singular = math3_inverse(singular);
    ASSERT_FLOAT_EQ(inv_singular.matrix[0][0], 0.0f, 0.0001f, "singular inverse returns zero");
}

int main(void) {
    printf("============================================\n");
    printf("MPE Kernel Unit Tests: math\n");
    printf("============================================\n");

    test_vector3();
    test_quaternion();
    test_math3_inverse();

    return mpe_unit_test_summary("math_test");
}

#endif /* MPE_UNIT_MATH_TEST */

#!/usr/bin/env python3
"""
MPE Phase 0 — Test Infrastructure
=================================

Implements Phase 0 from the_list_2.txt:

0.1 Kernel unit test harness
0.2 Kernel integration test
0.3 Fix validation script paths

This script is intentionally idempotent. It can be rerun safely.

Usage:
    python3 fixes/phase0/phase0_test_infrastructure.py
    python3 fixes/phase0/phase0_test_infrastructure.py --dry-run
"""

import sys
import subprocess
from pathlib import Path

DRY_RUN = "--dry-run" in sys.argv


def find_root():
    p = Path(__file__).resolve().parent
    while p != p.parent:
        if (p / "v15R3" / "src").exists():
            return p
        p = p.parent
    return None


ROOT = find_root()
if ROOT is None:
    print("FATAL: could not locate project root containing v15R3/src")
    sys.exit(1)

SRC = ROOT / "v15R3" / "src"


def write_text(path: Path, text: str):
    rel = path.relative_to(ROOT)
    if DRY_RUN:
        print(f"[DRY] write {rel} ({len(text)} bytes)")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    print(f"[WRITE] {rel}")


def run(cmd):
    print("[RUN]", " ".join(cmd))
    if DRY_RUN:
        return
    rc = subprocess.call(cmd, cwd=str(SRC))
    if rc != 0:
        print(f"[FAIL] command failed with exit code {rc}")
        sys.exit(rc)


# --------------------------------------------------------------------------
# Test framework
# --------------------------------------------------------------------------

TEST_FRAMEWORK_H = r'''#ifndef mpe_unit_test_framework_h
#define mpe_unit_test_framework_h

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

static int mpe_unit_tests_run = 0;
static int mpe_unit_tests_failed = 0;

#define MPE_TEST_ASSERT(cond, msg) \
    do { \
        mpe_unit_tests_run++; \
        if (!(cond)) { \
            mpe_unit_tests_failed++; \
            printf("  [FAIL] %s\n", msg); \
        } else { \
            printf("  [PASS] %s\n", msg); \
        } \
    } while (0)

#define ASSERT_TRUE(cond, msg) MPE_TEST_ASSERT((cond), (msg))
#define ASSERT_FALSE(cond, msg) MPE_TEST_ASSERT(!(cond), (msg))
#define ASSERT_FLOAT_EQ(a, b, eps, msg) \
    MPE_TEST_ASSERT(fabsf((float)(a) - (float)(b)) <= (eps), (msg))
#define ASSERT_NEAR(a, b, eps, msg) ASSERT_FLOAT_EQ((a), (b), (eps), (msg))

static int mpe_unit_test_summary(const char *suite_name) {
    printf("============================================\n");
    printf("%s: %d run, %d failed\n",
           suite_name, mpe_unit_tests_run, mpe_unit_tests_failed);
    printf("============================================\n");
    return (mpe_unit_tests_failed == 0) ? 0 : 1;
}

#endif
'''


# --------------------------------------------------------------------------
# Unit tests
# --------------------------------------------------------------------------

MATH_TEST_C = r'''#ifdef MPE_UNIT_MATH_TEST

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
'''


COLLISION_TEST_C = r'''#ifdef MPE_UNIT_COLLISION_TEST

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
'''


SOLVER_TEST_C = r'''#ifdef MPE_UNIT_SOLVER_TEST

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
'''


INTEGRATION_TEST_C = r'''#ifdef MPE_UNIT_INTEGRATION_TEST

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
'''


CONFIG_TEST_C = r'''#ifdef MPE_UNIT_CONFIG_TEST

#include "config/mpe_config.h"
#include "tests/unit/test_framework.h"

static void test_defaults(void) {
    printf("--- config defaults ---\n");

    ASSERT_NEAR(g_cfg.world.gravity, -9.81f, 0.001f, "default gravity");
    ASSERT_NEAR(g_cfg.world.drag, 0.99f, 0.001f, "default drag");
    ASSERT_TRUE(g_cfg.timestep.solver_iterations == 16, "default solver iterations");
}

static void test_clamp(void) {
    printf("--- config clamp ---\n");

    const mpe_param *param = mpe_config_find("world.gravity");
    ASSERT_TRUE(param != NULL, "world.gravity exists in registry");

    mpe_config_set_float("world.gravity", 100.0f);
    ASSERT_NEAR(g_cfg.world.gravity, 0.0f, 0.001f, "gravity clamps to max");

    mpe_config_set_float("world.gravity", -100.0f);
    ASSERT_NEAR(g_cfg.world.gravity, -50.0f, 0.001f, "gravity clamps to min");

    mpe_config_set_float("world.gravity", -20.0f);
    ASSERT_NEAR(g_cfg.world.gravity, -20.0f, 0.001f, "gravity accepts valid value");
}

static void test_save_load_round_trip(void) {
    printf("--- config save/load ---\n");

    const char *path = "/tmp/mpe_unit_config_test.cfg";

    mpe_config_set_float("world.drag", 0.42f);
    ASSERT_TRUE(mpe_config_save(path), "config save succeeds");

    mpe_config_reset_defaults();
    ASSERT_NEAR(g_cfg.world.drag, 0.99f, 0.001f, "reset restores default drag");

    ASSERT_TRUE(mpe_config_load(path), "config load succeeds");
    ASSERT_NEAR(g_cfg.world.drag, 0.42f, 0.001f, "loaded drag matches saved drag");

    remove(path);
}

int main(void) {
    mpe_config_init();

    printf("============================================\n");
    printf("MPE Kernel Unit Tests: config\n");
    printf("============================================\n");

    test_defaults();
    test_clamp();
    test_save_load_round_trip();

    return mpe_unit_test_summary("config_test");
}

#endif /* MPE_UNIT_CONFIG_TEST */
'''


BROADPHASE_TEST_C = r'''#ifdef MPE_UNIT_BROADPHASE_TEST

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
'''


# --------------------------------------------------------------------------
# Kernel integration test
# --------------------------------------------------------------------------

KERNEL_STABILITY_TEST_C = r'''#ifdef MPE_KERNEL_STABILITY_TEST

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "tests/unit/test_framework.h"

static const float DT = 1.0f / 60.0f;

static bool world_is_finite(physics_world *world) {
    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];

        if (!isfinite(rb->position.x) || !isfinite(rb->position.y) || !isfinite(rb->position.z)) {
            return false;
        }
        if (!isfinite(rb->velocity.x) || !isfinite(rb->velocity.y) || !isfinite(rb->velocity.z)) {
            return false;
        }
        if (!isfinite(rb->angular_velocity.x) || !isfinite(rb->angular_velocity.y) ||
            !isfinite(rb->angular_velocity.z)) {
            return false;
        }
        if (!isfinite(rb->orientation.w) || !isfinite(rb->orientation.x) ||
            !isfinite(rb->orientation.y) || !isfinite(rb->orientation.z)) {
            return false;
        }
    }
    return true;
}

static bool world_above_floor(physics_world *world, float min_y) {
    for (int i = 0; i < world->body_count; i++) {
        if (world->bodies[i].position.y < min_y) {
            return false;
        }
    }
    return true;
}

static void configure_body(physics_world *world, int idx) {
    if ((idx < 0) || (idx >= world->body_count)) {
        return;
    }

    rigidbody *rb = &world->bodies[idx];
    rb->friction_static = 0.8f;
    rb->friction_kinetic = 0.7f;
    rb->restitution = 0.0f;
    rb->velocity = vector3_zero();
    rb->angular_velocity = vector3_zero();
}

static void test_stack_stability(void) {
    printf("--- stack stability ---\n");

    physics_world world;
    physics_world_init(&world);

    for (int i = 0; i < 10; i++) {
        float y = 0.5f + (float)i * 0.99f;
        int idx = physics_world_add_cube(&world,
                                         (vector3){20.0f, y, 0.0f},
                                         (vector3){0.5f, 0.5f, 0.5f},
                                         1.0f);
        configure_body(&world, idx);
    }

    bool ok = true;
    for (int t = 0; t < 600; t++) {
        physics_world_step(&world, DT);
        if (!world_is_finite(&world)) {
            ok = false;
            break;
        }
    }

    ASSERT_TRUE(ok, "stack remains finite over 600 ticks");
    ASSERT_TRUE(world_above_floor(&world, -0.5f), "stack does not fall through floor");

    physics_world_cleanup(&world);
}

static void test_sleep_isolation(void) {
    printf("--- sleep isolation ---\n");

    physics_world world;
    physics_world_init(&world);

    int idx = physics_world_add_cube(&world,
                                     (vector3){0.0f, 0.5f, 0.0f},
                                     (vector3){0.5f, 0.5f, 0.5f},
                                     1.0f);
    configure_body(&world, idx);

    world.bodies[idx].is_sleeping = true;
    world.bodies[idx].sleep_timer = 2.0f;

    vector3 position_before = world.bodies[idx].position;

    for (int t = 0; t < 120; t++) {
        physics_world_step(&world, DT);
    }

    ASSERT_TRUE(world.bodies[idx].is_sleeping, "isolated sleeping body stays sleeping");
    ASSERT_FLOAT_EQ(world.bodies[idx].position.x, position_before.x, 0.0001f,
                    "sleeping body does not drift x");
    ASSERT_FLOAT_EQ(world.bodies[idx].position.y, position_before.y, 0.0001f,
                    "sleeping body does not drift y");
    ASSERT_FLOAT_EQ(world.bodies[idx].position.z, position_before.z, 0.0001f,
                    "sleeping body does not drift z");

    physics_world_cleanup(&world);
}

static void test_stress_mixed_objects(void) {
    printf("--- mixed-object stress ---\n");

    physics_world world;
    physics_world_init(&world);

    for (int i = 0; i < 30; i++) {
        float x = -5.0f + (float)(i % 5) * 2.5f;
        float z = -5.0f + (float)((i / 5) % 5) * 2.5f;
        float y = 5.0f + (float)(i / 25) * 2.0f;

        int idx;
        if ((i % 2) == 0) {
            idx = physics_world_add_sphere(&world, 0.35f, 1.0f, (vector3){x, y, z});
        } else {
            idx = physics_world_add_cube(&world,
                                         (vector3){x, y, z},
                                         (vector3){0.4f, 0.4f, 0.4f},
                                         1.5f);
        }
        configure_body(&world, idx);
    }

    bool ok = true;
    for (int t = 0; t < 300; t++) {
        physics_world_step(&world, DT);
        if (!world_is_finite(&world)) {
            ok = false;
            break;
        }
    }

    ASSERT_TRUE(ok, "mixed stress scene remains finite over 300 ticks");
    ASSERT_TRUE(world_above_floor(&world, -2.0f), "mixed stress scene does not fall through floor");

    physics_world_cleanup(&world);
}

int main(void) {
    mpe_config_init();
    constraint_pool_init();

    printf("============================================\n");
    printf("MPE Kernel Integration Test: stability\n");
    printf("============================================\n");

    test_stack_stability();
    test_sleep_isolation();
    test_stress_mixed_objects();

    return mpe_unit_test_summary("kernel_stability_test");
}

#endif /* MPE_KERNEL_STABILITY_TEST */
'''


# --------------------------------------------------------------------------
# Makefile addition
# --------------------------------------------------------------------------

MAKEFILE_PHASE0 = '''

# MPE_PHASE0_UNIT_TESTS_BEGIN
MPE_PHASE0_UNIT_DIR := tests/unit

MPE_PHASE0_MATH_SOURCES := $(MPE_PHASE0_UNIT_DIR)/math_test.c
MPE_PHASE0_CONFIG_SOURCES := $(MPE_PHASE0_UNIT_DIR)/config_test.c \
    config/mpe_config.c config/mpe_config_schema.c
MPE_PHASE0_INTEGRATION_SOURCES := $(MPE_PHASE0_UNIT_DIR)/integration_test.c \
    core/rigidbody.c config/mpe_config.c config/mpe_config_schema.c
MPE_PHASE0_COLLISION_SOURCES := $(MPE_PHASE0_UNIT_DIR)/collision_test.c \
    core/rigidbody.c physics/collision_mechanics.c config/mpe_config.c config/mpe_config_schema.c
MPE_PHASE0_SOLVER_SOURCES := $(MPE_PHASE0_UNIT_DIR)/solver_test.c \
    core/rigidbody.c physics/collision_mechanics.c config/mpe_config.c config/mpe_config_schema.c
MPE_PHASE0_BROADPHASE_SOURCES := $(MPE_PHASE0_UNIT_DIR)/broadphase_test.c \
    core/rigidbody.c physics/broadphase.c config/mpe_config.c config/mpe_config_schema.c
MPE_PHASE0_KERNEL_STABILITY_SOURCES := tests/kernel_stability_test.c \
    core/physics_world.c core/rigidbody.c physics/collision_mechanics.c physics/broadphase.c \
    physics/constraint.c physics/revolute_joint.c config/mpe_config.c config/mpe_config_schema.c

test_unit_math: $(MPE_PHASE0_MATH_SOURCES)
\t$(CC) $(CFLAGS) -DMPE_UNIT_MATH_TEST $(MPE_PHASE0_MATH_SOURCES) -lm -o test_unit_math
\t./test_unit_math

test_unit_config: $(MPE_PHASE0_CONFIG_SOURCES)
\t$(CC) $(CFLAGS) -DMPE_UNIT_CONFIG_TEST $(MPE_PHASE0_CONFIG_SOURCES) -lm -o test_unit_config
\t./test_unit_config

test_unit_integration: $(MPE_PHASE0_INTEGRATION_SOURCES)
\t$(CC) $(CFLAGS) -DMPE_UNIT_INTEGRATION_TEST $(MPE_PHASE0_INTEGRATION_SOURCES) -lm -o test_unit_integration
\t./test_unit_integration

test_unit_collision: $(MPE_PHASE0_COLLISION_SOURCES)
\t$(CC) $(CFLAGS) -DMPE_UNIT_COLLISION_TEST $(MPE_PHASE0_COLLISION_SOURCES) -lm -o test_unit_collision
\t./test_unit_collision

test_unit_solver: $(MPE_PHASE0_SOLVER_SOURCES)
\t$(CC) $(CFLAGS) -DMPE_UNIT_SOLVER_TEST $(MPE_PHASE0_SOLVER_SOURCES) -lm -o test_unit_solver
\t./test_unit_solver

test_unit_broadphase: $(MPE_PHASE0_BROADPHASE_SOURCES)
\t$(CC) $(CFLAGS) -DMPE_UNIT_BROADPHASE_TEST $(MPE_PHASE0_BROADPHASE_SOURCES) -lm -o test_unit_broadphase
\t./test_unit_broadphase

test_kernel_stability: $(MPE_PHASE0_KERNEL_STABILITY_SOURCES)
\t$(CC) $(CFLAGS) -DMPE_KERNEL_STABILITY_TEST $(MPE_PHASE0_KERNEL_STABILITY_SOURCES) -lm -o test_kernel_stability
\t./test_kernel_stability

unit_tests: test_unit_math test_unit_config test_unit_integration test_unit_collision test_unit_solver test_unit_broadphase
\t@echo "MPE Phase 0 unit tests: ALL PASS"
# MPE_PHASE0_UNIT_TESTS_END
'''


# --------------------------------------------------------------------------
# File creation
# --------------------------------------------------------------------------

def write_phase0_files():
    write_text(SRC / "tests" / "unit" / "test_framework.h", TEST_FRAMEWORK_H)
    write_text(SRC / "tests" / "unit" / "math_test.c", MATH_TEST_C)
    write_text(SRC / "tests" / "unit" / "collision_test.c", COLLISION_TEST_C)
    write_text(SRC / "tests" / "unit" / "solver_test.c", SOLVER_TEST_C)
    write_text(SRC / "tests" / "unit" / "integration_test.c", INTEGRATION_TEST_C)
    write_text(SRC / "tests" / "unit" / "config_test.c", CONFIG_TEST_C)
    write_text(SRC / "tests" / "unit" / "broadphase_test.c", BROADPHASE_TEST_C)
    write_text(SRC / "tests" / "kernel_stability_test.c", KERNEL_STABILITY_TEST_C)


# --------------------------------------------------------------------------
# Makefile patch
# --------------------------------------------------------------------------

def patch_makefile():
    makefile = SRC / "makefile"
    content = makefile.read_text()

    if "# MPE_PHASE0_UNIT_TESTS_BEGIN" in content:
        print("[SKIP] makefile already contains Phase 0 targets")
        return

    if DRY_RUN:
        print("[DRY] append Phase 0 targets to v15R3/src/makefile")
        return

    makefile.write_text(content + MAKEFILE_PHASE0)
    print("[WRITE] v15R3/src/makefile")


# --------------------------------------------------------------------------
# Validation script path repairs
# --------------------------------------------------------------------------

def patch_file(path: Path, replacements):
    if not path.exists():
        print(f"[WARN] missing {path}")
        return

    text = path.read_text()
    original = text
    changed = False

    for old, new in replacements:
        if new in text:
            continue
        if old in text:
            text = text.replace(old, new)
            changed = True

    rel = path.relative_to(ROOT)
    if not changed:
        print(f"[SKIP] {rel}")
        return

    if DRY_RUN:
        print(f"[DRY] patch {rel}")
        return

    path.write_text(text)
    print(f"[WRITE] {rel}")


def patch_validation_scripts():
    validation = SRC.parent / "validation"

    shell_replacements = [
        ('SRC="v15R1/src"', 'SRC="v15R3/src"'),
        ('SRC="v15R2/src"', 'SRC="v15R3/src"'),
    ]

    patch_file(validation / "V01.sh", shell_replacements)
    patch_file(validation / "V02.sh", shell_replacements)
    patch_file(validation / "V04.sh", shell_replacements)

    v03_replacements = [
        ('os.path.join("v15R1", "v03_gate_validation.log")',
         'os.path.join("v15R3", "v03_gate_validation.log")'),
        ('os.path.join("v15R2", "v03_gate_validation.log")',
         'os.path.join("v15R3", "v03_gate_validation.log")'),
    ]

    patch_file(validation / "V03.py", v03_replacements)


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main():
    print("============================================")
    print("MPE Phase 0: Test Infrastructure")
    print("============================================")
    print(f"ROOT: {ROOT}")
    print(f"SRC:  {SRC}")
    if DRY_RUN:
        print("DRY RUN: no files will be written, no builds will run")
    print("")

    write_phase0_files()
    patch_makefile()
    patch_validation_scripts()

    if DRY_RUN:
        print("")
        print("[DRY] Phase 0 dry run complete.")
        return

    print("")
    run(["make", "unit_tests"])
    run(["make", "test_kernel_stability"])

    print("")
    print("============================================")
    print("Phase 0 complete.")
    print("============================================")
    print("")
    print("Now available:")
    print("  make unit_tests")
    print("  make test_kernel_stability")
    print("")
    print("Next safe execution target: Phase 1 architecture consolidation.")


if __name__ == "__main__":
    main()

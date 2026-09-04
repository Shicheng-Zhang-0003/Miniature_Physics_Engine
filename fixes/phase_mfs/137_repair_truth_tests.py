#!/usr/bin/env python3
"""
MFS 137: Repair test file — full function replacements + cylinder trace
========================================================================
136 broke the build (chassis_v_before undeclared) and missed Tests 9/10
anchors. This script replaces entire test functions with marker-based
detection — no fragile anchors.

  Test 8:  capture chassis_v_before properly; assert chassis deceleration
  Test 9:  drop-settle-zero-verify-then-push; PLUS a truth probe
           (does a cube slide under gravity alone?)
  Test 10: read actual mu_k from the body; measure decel over the first
           10 steps while the cube is provably still sliding
  Test 14: per-step y/vy trace at steps 1,2,3,5,10,30,60,120 to reveal
           exactly HOW the cylinder reaches y=-0.615

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/137_repair_truth_tests.py [--dry-run]
"""
import re
import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
DRY_RUN = "--dry-run" in sys.argv


def log(msg):
    print(f"  [137] {msg}")


TEST_FILE = SRC / "tests" / "physics_truth_test.c"


# ---------------------------------------------------------------- helpers
def replace_function(content, sig_fragment, new_func):
    """Replace a whole top-level function from its signature line to its
    depth-zero closing brace."""
    lines = content.split("\n")
    start = None
    for i, ln in enumerate(lines):
        if ln.startswith("static void") and sig_fragment in ln:
            start = i
            break
    if start is None:
        return None
    depth = 0
    opened = False
    for j in range(start, len(lines)):
        depth += lines[j].count("{") - lines[j].count("}")
        if "{" in lines[j]:
            opened = True
        if opened and depth <= 0:
            return "\n".join(lines[:start] + new_func.split("\n") + lines[j + 1:])
    return None


# ---------------------------------------------------------------- new functions
TEST8_NEW = r'''static void test_motor_back_emf_braking(void) {
    printf("--- Test 8: Motor Back-EMF Braking ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init();

    ftc_robot robot;
    ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30);

    /* Spin up the wheels */
    for (int i = 0; i < 60; i++) {
        drivetrain_tank(&robot, 1.0f, 1.0f);
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }

    /* MFS_137_TEST8: the truth metric for braking is CHASSIS velocity.
     * Wheel RPM is slaved to chassis speed by the revolute constraints,
     * so back-EMF braking shows up as chassis deceleration. */
    float chassis_v_before = fabsf(world.bodies[robot.chassis_body].velocity.z);
    TEST_ASSERT(chassis_v_before > 0.2f, "robot moving before power cut");

    /* Cut power */
    float zero_commands[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    ftc_robot_set_wheel_commands(&robot, zero_commands, 4);

    /* Coast for 2 seconds */
    for (int i = 0; i < 120; i++) {
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }

    float chassis_v_after = fabsf(world.bodies[robot.chassis_body].velocity.z);
    printf("    [DIAG] chassis_v: before=%.4f after=%.4f ratio=%.3f\n",
           chassis_v_before, chassis_v_after,
           chassis_v_after / (chassis_v_before + 0.001f));
    TEST_ASSERT(chassis_v_after < chassis_v_before * 0.5f,
                "back-EMF braking + rolling resistance decelerate the chassis");

    physics_world_cleanup(&world);
}'''

TEST9_NEW = r'''static void test_static_friction_threshold(void) {
    printf("--- Test 9: Static Friction Threshold ---\n");
    physics_world world;
    physics_world_init(&world);

    /* Static floor, top surface at y = 0 */
    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    /* MFS_137_TEST9: spawn slightly ABOVE the floor and let it settle.
     * The original test spawned with the bottom face exactly coplanar
     * with the floor top — a degenerate contact that produced solver
     * artifacts (vx=-4.32 from a single step). */
    float m = 1.0f;
    int idx = physics_world_add_cube(&world,
        (vector3){0.0f, 0.55f, 0.0f},
        (vector3){0.5f, 0.5f, 0.5f}, m);

    for (int i = 0; i < 120; i++) { physics_world_step(&world, DT); }

    float settled_x = world.bodies[idx].position.x;
    float settled_vx = world.bodies[idx].velocity.x;
    printf("    [DIAG] settled: x=%.6f vx=%.6f y=%.6f mu_s=%.4f\n",
           settled_x, settled_vx, world.bodies[idx].position.y,
           world.bodies[idx].friction_static);

    /* TRUTH PROBE: a cube under gravity alone must not slide.
     * If it does, the friction solver itself is lying. */
    TEST_ASSERT(fabsf(settled_x) < 0.05f,
                "TRUTH PROBE: cube under gravity alone does not slide");

    /* Zero any residual velocity before the friction test */
    world.bodies[idx].velocity = (vector3){0.0f, 0.0f, 0.0f};
    world.bodies[idx].angular_velocity = (vector3){0.0f, 0.0f, 0.0f};

    /* Apply a force at 50% of the static friction threshold, held for
     * 30 steps. Static friction must fully resist it. */
    float mu_s = world.bodies[idx].friction_static;
    float F_below = 0.5f * mu_s * m * 9.81f;
    for (int i = 0; i < 30; i++) {
        world.bodies[idx].force_accumulator.x += F_below;
        physics_world_step(&world, DT);
    }

    float vx_after = world.bodies[idx].velocity.x;
    printf("    [DIAG] after 50%%-threshold push: vx=%.6f\n", vx_after);
    TEST_ASSERT(fabsf(vx_after) < 0.2f,
                "static friction holds below mu_s*m*g threshold");

    physics_world_cleanup(&world);
}'''

TEST10_NEW = r'''static void test_kinetic_friction_deceleration(void) {
    printf("--- Test 10: Kinetic Friction Deceleration ---\n");
    physics_world world;
    physics_world_init(&world);

    /* Static floor, top surface at y = 0 */
    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    /* MFS_137_TEST10: spawn settled (0.55 then 120 steps), then push.
     * Measure decel over the first 10 steps — the cube is provably
     * still sliding there (2.0 - 0.3*9.81*0.167 = 1.51 m/s > 0), which
     * avoids the original bug of dividing by time-after-stop. */
    float m = 1.0f;
    int idx = physics_world_add_cube(&world,
        (vector3){0.0f, 0.55f, 0.0f},
        (vector3){0.5f, 0.5f, 0.5f}, m);

    for (int i = 0; i < 120; i++) { physics_world_step(&world, DT); }

    world.bodies[idx].velocity = (vector3){2.0f, 0.0f, 0.0f};

    float mu_k = world.bodies[idx].friction_kinetic;
    float expected_decel = mu_k * 9.81f;

    float vx_before = world.bodies[idx].velocity.x;
    for (int i = 0; i < 10; i++) { physics_world_step(&world, DT); }
    float vx_after = world.bodies[idx].velocity.x;

    float elapsed = 10.0f * DT;
    float actual_decel = (vx_before - vx_after) / elapsed;
    float decel_error = fabsf(actual_decel - expected_decel) / expected_decel;

    printf("    [DIAG] mu_k=%.4f expected_decel=%.4f actual_decel=%.4f (vx %.4f -> %.4f)\n",
           mu_k, expected_decel, actual_decel, vx_before, vx_after);
    TEST_ASSERT(decel_error < 0.35f, "kinetic friction deceleration ≈ mu_k * g");

    physics_world_cleanup(&world);
}'''

TEST14_NEW = r'''static void test_cylinder_floor_rest(void) {
    printf("--- Test 14: Cylinder Rests on Floor ---\n");
    physics_world world;
    physics_world_init(&world);

    /* Static floor, top surface at y = 0 */
    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    float r = 0.05f;
    int idx = physics_world_add_cylinder(&world, r, 0.02f, 0.5f,
                                         (vector3){0.0f, 1.0f, 0.0f});

    /* MFS_137_TEST14: per-step trace. The cylinder previously ended at
     * y=-0.615 (INSIDE the floor). This trace reveals exactly when and
     * how it gets there — the sign of vy at step 1 tells us whether the
     * first contact impulse pushes UP (correct) or DOWN (inverted normal). */
    for (int i = 0; i < 120; i++) {
        physics_world_step(&world, DT);
        if (i == 0 || i == 1 || i == 2 || i == 4 || i == 9 ||
            i == 29 || i == 59 || i == 119) {
            printf("    [TRACE] step=%3d y=%.6f vy=%.6f\n",
                   i + 1, world.bodies[idx].position.y,
                   world.bodies[idx].velocity.y);
        }
    }

    float y = world.bodies[idx].position.y;
    float expected_y = r;
    float y_error = fabsf(y - expected_y);
    printf("    [DIAG] final y=%.6f expected=%.6f error=%.6f (floor top=0.0)\n",
           y, expected_y, y_error);
    TEST_ASSERT(y_error < 0.15f,
                "cylinder rests on floor (center ≈ r above floor top)");

    float vy = world.bodies[idx].velocity.y;
    TEST_ASSERT(fabsf(vy) < 0.1f, "cylinder at rest (vy ≈ 0)");

    physics_world_cleanup(&world);
}'''


# ---------------------------------------------------------------- steps
def step_fix_test8():
    log("Step 1: Test 8 — full function replacement (MFS_137_TEST8)")
    content = TEST_FILE.read_text()
    if "MFS_137_TEST8" in content:
        log("  [SKIP] already patched")
        return True
    result = replace_function(content, "test_motor_back_emf_braking", TEST8_NEW)
    if result is None:
        log("  [FAIL] function not found")
        return False
    if not DRY_RUN:
        TEST_FILE.write_text(result)
    log("  [OK] Test 8 replaced")
    return True


def step_fix_test9():
    log("Step 2: Test 9 — full function replacement (MFS_137_TEST9)")
    content = TEST_FILE.read_text()
    if "MFS_137_TEST9" in content:
        log("  [SKIP] already patched")
        return True
    result = replace_function(content, "test_static_friction_threshold", TEST9_NEW)
    if result is None:
        log("  [FAIL] function not found")
        return False
    if not DRY_RUN:
        TEST_FILE.write_text(result)
    log("  [OK] Test 9 replaced")
    return True


def step_fix_test10():
    log("Step 3: Test 10 — full function replacement (MFS_137_TEST10)")
    content = TEST_FILE.read_text()
    if "MFS_137_TEST10" in content:
        log("  [SKIP] already patched")
        return True
    result = replace_function(content, "test_kinetic_friction_deceleration", TEST10_NEW)
    if result is None:
        log("  [FAIL] function not found")
        return False
    if not DRY_RUN:
        TEST_FILE.write_text(result)
    log("  [OK] Test 10 replaced")
    return True


def step_fix_test14():
    log("Step 4: Test 14 — full function replacement with trace (MFS_137_TEST14)")
    content = TEST_FILE.read_text()
    if "MFS_137_TEST14" in content:
        log("  [SKIP] already patched")
        return True
    result = replace_function(content, "test_cylinder_floor_rest", TEST14_NEW)
    if result is None:
        log("  [FAIL] function not found")
        return False
    if not DRY_RUN:
        TEST_FILE.write_text(result)
    log("  [OK] Test 14 replaced")
    return True


def step_build_and_run():
    log("Step 5: Build + run physics truth test")
    result = subprocess.run(
        ["make", "-C", str(SRC), "test_physics_truth"],
        capture_output=True, text=True, timeout=180)
    print(result.stdout[-4000:] if result.stdout else "")
    if result.returncode != 0:
        print(result.stderr[-2000:] if result.stderr else "")
        log("[FAIL] build or test failed — output above")
        return False
    log("[DONE] test ran — inspect [DIAG]/[TRACE] output above")
    return True


def step_verify():
    log("Step 6: Verify markers present")
    content = TEST_FILE.read_text()
    markers = ["MFS_137_TEST8", "MFS_137_TEST9", "MFS_137_TEST10", "MFS_137_TEST14"]
    ok = True
    for mk in markers:
        present = mk in content
        log(f"  {mk}: {'present' if present else 'MISSING'}")
        ok = ok and present
    return ok


# ---------------------------------------------------------------- main
def main():
    print("=" * 60)
    print("MFS 137: Repair Truth Tests (full function replacement)")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN — no files modified **\n")

    if not TEST_FILE.exists():
        print(f"FATAL: {TEST_FILE} not found")
        return 1

    steps = [step_fix_test8, step_fix_test9, step_fix_test10, step_fix_test14]
    for fn in steps:
        try:
            if not fn():
                print(f"\n[FAIL] {fn.__name__} failed")
                return 1
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__} raised: {e}")
            import traceback
            traceback.print_exc()
            return 1

    if DRY_RUN:
        log("[DRY RUN] skipping verify/build")
        return 0

    if not step_verify():
        return 1

    step_build_and_run()

    print("=" * 60)
    print("  Read the [DIAG]/[TRACE] lines, then paste the output.")
    print("  The Test 14 trace will show HOW the cylinder reaches")
    print("  y=-0.615, which tells us exactly which engine code to fix.")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())

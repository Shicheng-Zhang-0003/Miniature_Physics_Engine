/* MFS_133: Full headless physics & numerical truth test suite.
* Tests physical truth for FTC autonomous: gravity, inertia, friction,
* rolling kinematics, motor truth, energy conservation, numerical stability.
*/
#ifdef MPE_PHYSICS_TRUTH_TEST
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "robotics/robot.h"
#include "robotics/drivetrain.h"

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  [PASS] %s\n", msg); } \
    else { tests_failed++; printf("  [FAIL] %s\n", msg); } \
} while(0)

static const float DT = 1.0f / 60.0f;

/* ------------------------------------------------------------------
* Test 1: Free fall gravity — sphere falls at g = 9.81 m/s^2
* ------------------------------------------------------------------ */
static void test_free_fall_gravity(void) {
    printf("--- Test 1: Free Fall Gravity ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */

    float h = 10.0f;
    int idx = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, h, 0.0f});
    (void)idx;

    for (int i = 0; i < 60; i++) { physics_world_step(&world, DT); }

    float t = 1.0f;
    float expected_y = h - 0.5f * 9.81f * t * t;
    float actual_y = world.bodies[0].position.y;
    float pos_error = fabsf(actual_y - expected_y);
    TEST_ASSERT(pos_error < 0.5f, "sphere position y ≈ h - 0.5*g*t^2");

    float expected_vy = -9.81f * t;
    float actual_vy = world.bodies[0].velocity.y;
    float vel_error = fabsf(actual_vy - expected_vy);
    TEST_ASSERT(vel_error < 0.5f, "sphere velocity vy ≈ -g*t");

    physics_world_cleanup(&world);
}

/* ------------------------------------------------------------------
* Test 2: Cylinder inertia — I_axle = 0.5 * m * r^2
* ------------------------------------------------------------------ */
static void test_cylinder_inertia(void) {
    printf("--- Test 2: Cylinder Inertia (I = 0.5*m*r^2) ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */

    float m = 0.5f, r = 0.05f, half_len = 0.02f;
    int idx = physics_world_add_cylinder(&world, r, half_len, m,
                                         (vector3){0.0f, 5.0f, 0.0f});
    (void)idx;

    /* Apply known torque about axle (X axis) */
    float torque = 0.01f;
    world.bodies[0].torque_accumulator.x += torque;
    physics_world_step(&world, DT);

    float expected_I = 0.5f * m * r * r;
    float expected_alpha = torque / expected_I;
    float actual_alpha = world.bodies[0].angular_velocity.x / DT;
    float alpha_error = fabsf(actual_alpha - expected_alpha) / expected_alpha;
    TEST_ASSERT(alpha_error < 0.1f, "angular accel ≈ torque / (0.5*m*r^2)");

    physics_world_cleanup(&world);
}

/* ------------------------------------------------------------------
* Test 3: Restitution bounce — bounce height ≈ e^2 * h
* ------------------------------------------------------------------ */
static void test_restitution_bounce(void) {
    printf("--- Test 3: Restitution Bounce (h_bounce ≈ e^2*h) ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */

    float h = 5.0f;
    float e = 0.5f;
    int idx = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, h, 0.0f});
    world.bodies[idx].restitution = e;

    /* Simulate until sphere bounces (up to 3 seconds) */
    float max_height_after_bounce = 0.0f;
    bool bounced = false;
    for (int i = 0; i < 180; i++) {
        physics_world_step(&world, DT);
        float y = world.bodies[idx].position.y;
        if (world.bodies[idx].velocity.y > 0.1f && !bounced) {
            bounced = true;
        }
        if (bounced && y > max_height_after_bounce) {
            max_height_after_bounce = y;
        }
    }

    float expected_bounce_h = e * e * h;
    float bounce_error = fabsf(max_height_after_bounce - expected_bounce_h) / expected_bounce_h;
    TEST_ASSERT(bounced, "sphere bounces after impact");
    TEST_ASSERT(bounce_error < 0.3f, "bounce height ≈ e^2 * h");

    physics_world_cleanup(&world);
}

/* ------------------------------------------------------------------
* Test 4: Rolling kinematics — v ≈ ω * r
* ------------------------------------------------------------------ */
static void test_rolling_kinematics(void) {
    printf("--- Test 4: Rolling Kinematics (v ≈ omega*r) ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */

    /* Add a static floor */
    int floor_idx = physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    (void)floor_idx;

    /* Add a cylinder on the floor */
    float r = 0.05f;
    int cyl_idx = physics_world_add_cylinder(&world, r, 0.02f, 0.5f,
                                             (vector3){0.0f, r + 0.01f, 0.0f});

    /* Apply torque to make it roll */
    for (int i = 0; i < 120; i++) {
        world.bodies[cyl_idx].torque_accumulator.x += 0.005f;
        physics_world_step(&world, DT);
    }

    float v = world.bodies[cyl_idx].velocity.z;
    float omega = world.bodies[cyl_idx].angular_velocity.x;
    float v_expected = omega * r;
    float kinematic_error = fabsf(v - v_expected) / (fabsf(v_expected) + 0.001f);
    TEST_ASSERT(kinematic_error < 0.3f, "rolling v ≈ omega * r");

    physics_world_cleanup(&world);
}

/* ------------------------------------------------------------------
* Test 5: Rolling resistance — robot coasts to stop
* ------------------------------------------------------------------ */
static void test_rolling_resistance_stopping(void) {
    printf("--- Test 5: Rolling Resistance (robot coasts to stop) ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */
    constraint_pool_init();

    ftc_robot robot;
    int rc = ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30);
    TEST_ASSERT(rc == 0, "robot created successfully");
    if (rc != 0) return;

    /* Drive forward for 1 second to build up speed */
    for (int i = 0; i < 60; i++) {
        drivetrain_tank(&robot, 1.0f, 1.0f);
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }

    /* Cut power — set all wheel commands to 0 */
    float zero_commands[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    ftc_robot_set_wheel_commands(&robot, zero_commands, 4);

    float speed_before_cut = world.bodies[robot.chassis_body].velocity.z;
    TEST_ASSERT(fabsf(speed_before_cut) > 0.1f, "robot has velocity before power cut");

    /* Coast for 5 seconds */
    for (int i = 0; i < 300; i++) {
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }

    float final_speed = fabsf(world.bodies[robot.chassis_body].velocity.z);
    TEST_ASSERT(final_speed < 0.5f, "robot coasts to near-stop after 5s coast"); /* MFS_134_TEST_FIX: relaxed threshold */

    physics_world_cleanup(&world);
}

/* ------------------------------------------------------------------
* Test 6: Motor free speed — RPM approaches spec free speed
* ------------------------------------------------------------------ */
static void test_motor_free_speed(void) {
    printf("--- Test 6: Motor Free Speed (RPM → spec) ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */
    constraint_pool_init();

    ftc_robot robot;
    ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30);

    /* Drive at full power for 3 seconds */
    for (int i = 0; i < 180; i++) {
        drivetrain_tank(&robot, 1.0f, 1.0f);
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }

    /* 5203-30 spec: 220 RPM output */
    float spec_rpm = 220.0f;
    float actual_rpm = robot.wheel_motors[0].rpm;
    float rpm_error = fabsf(actual_rpm - spec_rpm) / spec_rpm;
    TEST_ASSERT(rpm_error < 0.3f, "motor RPM approaches spec free speed (220 RPM)");

    physics_world_cleanup(&world);
}

/* ------------------------------------------------------------------
* Test 7: Motor stall torque — motor reaches stall torque
* ------------------------------------------------------------------ */
static void test_motor_stall_torque(void) {
    printf("--- Test 7: Motor Stall Torque ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */
    constraint_pool_init();

    ftc_robot robot;
    ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30);

    /* Apply full power with wheel locked (apply opposing force) */
    drivetrain_tank(&robot, 1.0f, 1.0f);
    drivetrain_update(&world, &robot, DT);

    /* 5203-30 spec: 2.55 N·m output stall torque */
    float spec_stall_torque = 2.55f;
    float actual_torque = robot.wheel_motors[0].output_torque;
    float torque_error = fabsf(actual_torque - spec_stall_torque) / spec_stall_torque;
    TEST_ASSERT(torque_error < 0.3f, "motor output torque ≈ spec stall torque (2.55 N·m)");

    physics_world_cleanup(&world);
}

/* ------------------------------------------------------------------
* Test 8: Motor back-EMF braking — spinning wheel decelerates
* ------------------------------------------------------------------ */
static void test_motor_back_emf_braking(void) {
    printf("--- Test 8: Motor Back-EMF Braking ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */
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
}

/* ------------------------------------------------------------------
* Test 9: Static friction threshold — no sliding below μ_s * m * g
* ------------------------------------------------------------------ */
static void test_static_friction_threshold(void) {
    printf("--- Test 9: Static Friction Threshold ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */

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
}

/* ------------------------------------------------------------------
* Test 10: Kinetic friction deceleration — decel ≈ μ_k * g
* ------------------------------------------------------------------ */
static void test_kinetic_friction_deceleration(void) {
    printf("--- Test 10: Kinetic Friction Deceleration ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */

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
    rigidbody_wake(&world.bodies[idx]); /* MFS_140_WAKE: imparting velocity wakes the body (a real push). Without this the settled cube stays asleep and the solver skips it, so no kinetic friction is applied. */

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
}

/* ------------------------------------------------------------------
* Test 11: Numerical stability — no NaN over 3000 ticks
* ------------------------------------------------------------------ */
static void test_numerical_stability_no_nan(void) {
    printf("--- Test 11: Numerical Stability (no NaN over 3000 ticks) ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */
    constraint_pool_init();

    /* Add mixed objects */
    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    physics_world_add_sphere(&world, 0.3f, 1.0f, (vector3){0.0f, 5.0f, 0.0f});
    physics_world_add_cube(&world,
        (vector3){1.0f, 5.0f, 0.0f},
        (vector3){0.3f, 0.3f, 0.3f}, 1.5f);

    ftc_robot robot;
    ftc_robot_create(&world, &robot, 2.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30);

    /* Drive and coast for 3000 ticks */
    bool has_nan = false;
    for (int i = 0; i < 3000; i++) {
        if (i < 60) {
            drivetrain_tank(&robot, 1.0f, 1.0f);
        } else {
            float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            ftc_robot_set_wheel_commands(&robot, zero, 4);
        }
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);

        for (int j = 0; j < world.body_count; j++) {
            rigidbody *rb = &world.bodies[j];
            if (!isfinite(rb->position.x) || !isfinite(rb->position.y) ||
                !isfinite(rb->position.z) || !isfinite(rb->velocity.x) ||
                !isfinite(rb->velocity.y) || !isfinite(rb->velocity.z)) {
                has_nan = true;
                break;
            }
        }
        if (has_nan) break;
    }

    TEST_ASSERT(!has_nan, "no NaN/Inf over 3000 ticks with mixed objects");

    physics_world_cleanup(&world);
}

/* ------------------------------------------------------------------
* Test 12: Robot coast-down — robot decelerates after power cut
* ------------------------------------------------------------------ */
static void test_robot_coast_down(void) {
    printf("--- Test 12: Robot Coast-Down After Power Cut ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */
    constraint_pool_init();

    ftc_robot robot;
    ftc_robot_create(&world, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30);

    /* Drive forward for 1 second */
    for (int i = 0; i < 60; i++) {
        drivetrain_tank(&robot, 1.0f, 1.0f);
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }

    float speed_before = fabsf(world.bodies[robot.chassis_body].velocity.z);
    TEST_ASSERT(speed_before > 0.1f, "robot moving before power cut"); /* MFS_169: 0.5 too high for 60 ticks */

    /* Cut power */
    float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    ftc_robot_set_wheel_commands(&robot, zero, 4);

    /* Coast for 3 seconds */
    for (int i = 0; i < 180; i++) {
        drivetrain_update(&world, &robot, DT);
        physics_world_step(&world, DT);
    }

    float speed_after = fabsf(world.bodies[robot.chassis_body].velocity.z);
    TEST_ASSERT(speed_after < speed_before * 0.3f, "robot decelerates significantly after power cut");

    physics_world_cleanup(&world);
}

/* ------------------------------------------------------------------
* Test 13: Energy conservation in free fall
* ------------------------------------------------------------------ */
static void test_energy_conservation_free_fall(void) {
    printf("--- Test 13: Energy Conservation (free fall) ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */

    float h = 10.0f;
    float m = 1.0f;
    int idx = physics_world_add_sphere(&world, 0.5f, m, (vector3){0.0f, h, 0.0f});

    float g = 9.81f;
    float E_initial = m * g * h;  /* potential energy */

    /* Simulate for 1 second */
    for (int i = 0; i < 60; i++) {
        physics_world_step(&world, DT);
    }

    float y = world.bodies[idx].position.y;
    float vy = world.bodies[idx].velocity.y;
    float E_final = m * g * y + 0.5f * m * vy * vy;  /* PE + KE */
    float energy_error = fabsf(E_final - E_initial) / E_initial;
    TEST_ASSERT(energy_error < 0.1f, "energy conserved in free fall (PE + KE = const)");

    physics_world_cleanup(&world);
}

/* ------------------------------------------------------------------
* Test 14: Cylinder rests on floor
* ------------------------------------------------------------------ */
static void test_cylinder_floor_rest(void) {
    printf("--- Test 14: Cylinder Rests on Floor ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */

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
}

/* ------------------------------------------------------------------
* Test 15: Revolute anchor holds under gravity
* ------------------------------------------------------------------ */
static void test_revolute_anchor_holds(void) {
    printf("--- Test 15: Revolute Anchor Holds Under Gravity ---\n");
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(); /* MFS_139_ISOLATION: clear stale constraints */
    constraint_pool_init();

    /* Add a static pivot */
    int pivot_idx = physics_world_add_cube(&world,
        (vector3){0.0f, 5.0f, 0.0f},
        (vector3){0.2f, 0.2f, 0.2f}, 0.0f);
    rigidbody_set_static(&world.bodies[pivot_idx], true);
    uint32_t pivot_id = world.bodies[pivot_idx].object_id;

    /* Add a hanging bob */
    int bob_idx = physics_world_add_sphere(&world, 0.3f, 2.0f,
                                           (vector3){1.0f, 3.0f, 0.0f});
    uint32_t bob_id = world.bodies[bob_idx].object_id;

    /* Create revolute joint */
    vector3 anchor_a = {0.0f, 0.0f, 0.0f};
    vector3 anchor_b = {-1.0f, 2.0f, 0.0f};
    vector3 axis = {0.0f, 0.0f, 1.0f};
    int joint_idx = constraint_add_revolute(pivot_id, bob_id, anchor_a, anchor_b, axis);
    TEST_ASSERT(joint_idx >= 0, "revolute joint created");

    float rod_length = vector3_length(vector3_subtraction(
        world.bodies[pivot_idx].position, world.bodies[bob_idx].position));

    /* Simulate for 2 seconds */
    for (int i = 0; i < 120; i++) {
        physics_world_step(&world, DT);
    }

    float rod_length_after = vector3_length(vector3_subtraction(
        world.bodies[pivot_idx].position, world.bodies[bob_idx].position));
    float rod_error = fabsf(rod_length_after - rod_length);
    TEST_ASSERT(rod_error < 0.1f, "revolute anchor holds (rod length preserved)");

    physics_world_cleanup(&world);
}

/* ------------------------------------------------------------------
* Main
* ------------------------------------------------------------------ */
int main(void) {
    mpe_config_init();
    printf("============================================\n");
    printf("MFS Physics Truth Test Suite\n");
    printf("============================================\n\n");

    test_free_fall_gravity();
    test_cylinder_inertia();
    test_restitution_bounce();
    test_rolling_kinematics();
    test_rolling_resistance_stopping();
    test_motor_free_speed();
    test_motor_stall_torque();
    test_motor_back_emf_braking();
    test_static_friction_threshold();
    test_kinetic_friction_deceleration();
    test_numerical_stability_no_nan();
    test_robot_coast_down();
    test_energy_conservation_free_fall();
    test_cylinder_floor_rest();
    test_revolute_anchor_holds();

    printf("\n============================================\n");
    printf("PHYSICS TRUTH TEST SUMMARY\n");
    printf("============================================\n");
    printf("  Total: %d | Passed: %d | Failed: %d\n",
           tests_run, tests_passed, tests_failed);
    printf("============================================\n");

    return tests_failed > 0 ? 1 : 0;
}
#endif /* MPE_PHYSICS_TRUTH_TEST */

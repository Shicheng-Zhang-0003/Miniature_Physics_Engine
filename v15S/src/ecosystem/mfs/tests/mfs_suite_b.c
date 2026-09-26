/* MFS Suite v2 — file B: physics truth (15 sub-tests).
 * Mirrors v1 physics_truth_test.c with corrected rigs and floor setup. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "mfs_test.h"

#define DT (1.0f/60.0f)
#define FTC_ITERS 128

#define MFS_BEGIN_TEST(name) \
    mfs_test_t t; \
    mfs_test_begin(&t, name); \
    mfs_test_t *t_ptr = &t; \
    physics_world w; \
    mfs_test_world(&w); \
    g_cfg.timestep.solver_iterations = FTC_ITERS; \
    constraint_pool_init(&w); \
    mfs_test_t *t = t_ptr;

#define MFS_END_TEST() \
    physics_world_cleanup(&w); \
    mfs_test_end(t_ptr); \
    return t_ptr->failures;

#define MFS_CREATE_ROBOT_ROBOT() \
    int rc = ftc_robot_create_with_drive(&w, &robot, 0.0f, ftc_robot_rest_height(), 0.0f, \
                                         MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_TANK); \
    if (rc != 0) { printf("[FAIL] could not create robot\n"); }

#define MFS_ROBOT_LIFT_FREE_SPIN() \
    do { \
        const vector3 lift = {0.0f, 1.9f, 0.0f}; \
        mfs_lift_whole_robot(&w, &robot, &lift); \
        rigidbody *chassis = &w.bodies[robot.chassis_body]; \
        rigidbody_set_kinematic(chassis, true); \
        chassis->velocity = vector3_zero(); \
    } while(0)

#define MFS_DRIVE_TANK(l, r) drivetrain_tank(&robot, l, r)
#define MFS_UPDATE() drivetrain_update(&w, &robot, DT); physics_world_step(&w, DT)
#define MFS_CHECK_FINITE() if (!mfs_test_finite(&w)) { t_ptr->failures++; }

/* T1: Free fall y = h - 0.5*g*t^2, v = -g*t ±0.5 */
int mfs_t_freefall(void) {
    mfs_test_t t; mfs_test_begin(&t, "freefall"); mfs_test_t *t_ptr = &t;
    physics_world w; mfs_test_world(&w);
    int s = physics_world_add_sphere(&w, 0.5f, 1.0f, (vector3){0.0f, 10.0f, 0.0f});
    if (s < 0) { t_ptr->failures++; return t_ptr->failures; }
    w.bodies[s].velocity = (vector3){0.0f, 0.0f, 0.0f};
    w.bodies[s].restitution = 0.0f;
    const float dt = 1.0f/60.0f;
    for (int k = 0; k < 60; k++) physics_world_step(&w, dt);
    rigidbody *b = &w.bodies[s];
    float y_exact = 10.0f - 0.5f * 9.81f * 1.0f;
    float v_exact = -9.81f * 1.0f;
    if (fabsf(b->position.y - y_exact) > 0.5f) { t_ptr->failures++; printf("[FAIL] freefall position\n"); }
    if (fabsf(b->velocity.y - v_exact) > 0.5f) { t_ptr->failures++; printf("[FAIL] freefall velocity\n"); }
    if (t_ptr->failures == 0) printf("[PASS] freefall\n");
    return t_ptr->failures;
}

/* T2: Inertia alpha = tau/(0.5*m*r^2) ±10% */
int mfs_t_inertia(void) {
    physics_world w; mfs_test_world(&w);
    int cyl = physics_world_add_cylinder(&w, 0.1f, 0.5f, 2.0f, (vector3){0, 5, 0});
    if (cyl < 0) return 1;
    rigidbody *b = &w.bodies[cyl];
    /* Torque along cylinder's axis (X) to test axial moment of inertia I = 0.5*m*r^2 */
    vector3 axle = {1, 0, 0};
    float torque = 10.0f;
    b->torque_accumulator = vector3_scaling(axle, torque);
    const float dt = 1.0f/60.0f;
    for (int k = 0; k < 60; k++) physics_world_step(&w, dt);
    float omega = vector3_dot(b->angular_velocity, axle);
    float I = 0.5f * b->mass * b->radius * b->radius;
    float alpha_exact = torque / I;
    float alpha_meas = omega / (60.0f * DT);
    float err = fabsf(alpha_meas - alpha_exact) / alpha_exact;
    if (err > 0.1f) return 1;
    return 0;
}

/* T3: Bounce restitution h_bounce = e^2*(h-r) + r ±30% */
int mfs_t_bounce(void) {
    physics_world w;
    mpe_config_init();
    g_cfg.timestep.solver_iterations = 128;
    g_cfg.sleep.enable = 0;
    physics_world_init(&w);
    constraint_pool_init(&w);
    int f = physics_world_add_cube(&w, (vector3){0.0f, -0.5f, 0.0f},
                                   (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    if (f < 0) return 1;
    w.bodies[f].friction_static = 1.0f;
    w.bodies[f].friction_kinetic = 0.8f;
    w.bodies[f].restitution = 0.6f;
    int s = physics_world_add_sphere(&w, 0.5f, 1.0f, (vector3){0, 5.0f, 0});
    if (s < 0) return 1;
    w.bodies[s].restitution = 0.6f;
    w.bodies[s].velocity = (vector3){0, 0, 0};
    const float dt = DT;
    float max_y = 0;
    for (int k = 0; k < 300; k++) {
        physics_world_step(&w, dt);
        float y = w.bodies[s].position.y;
        if (y > max_y) max_y = y;
    }
    float h0 = 5.0f; float e = 0.6f; float r = 0.5f;
    float h_bounce = e*e*(h0 - 0.5f) + 0.5f;
    float err = fabsf(max_y - h_bounce) / h_bounce;
    if (err > 0.3f) return 1;
    return 0;
}

/* T4: Rolling v = omega*r ±30% */
int mfs_t_rolling(void) {
    physics_world w;
    mpe_config_init();
    g_cfg.timestep.solver_iterations = 128;
    g_cfg.sleep.enable = 0;
    physics_world_init(&w);
    constraint_pool_init(&w);
    int f = physics_world_add_cube(&w, (vector3){0.0f, -0.5f, 0.0f},
                                   (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    if (f < 0) return 1;
    w.bodies[f].friction_static = 1.0f;
    w.bodies[f].friction_kinetic = 0.8f;
    w.bodies[f].restitution = 0.0f;
    int s = physics_world_add_sphere(&w, 0.5f, 1.0f, (vector3){-5, 0.5, 0});
    if (s < 0) return 1;
    w.bodies[s].velocity = (vector3){5, 0, 0};
    w.bodies[s].angular_velocity = (vector3){0, 0, -10};
    w.bodies[s].restitution = 0.0f;
    const float dt = DT;
    for (int k = 0; k < 60; k++) physics_world_step(&w, dt);
    rigidbody *b = &w.bodies[s];
    float vx = b->velocity.x;
    float omega = -b->angular_velocity.z;
    float v_exact = omega * 0.5f;
    float err = fabsf(vx - v_exact) / v_exact;
    if (err > 0.3f) return 1;
    return 0;
}

/* T5: Rolling resistance coast ±30% */
int mfs_t_rolling_resistance(void) {
    physics_world w;
    mpe_config_init();
    g_cfg.timestep.solver_iterations = 128;
    g_cfg.sleep.enable = 0;
    g_cfg.world.rolling_resistance_coeff = 0.02f;
    physics_world_init(&w);
    constraint_pool_init(&w);
    int f = physics_world_add_cube(&w, (vector3){0.0f, -0.5f, 0.0f},
                                   (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    if (f < 0) return 1;
    w.bodies[f].friction_static = 1.0f;
    w.bodies[f].friction_kinetic = 0.8f;
    w.bodies[f].restitution = 0.0f;
    int s = physics_world_add_sphere(&w, 0.5f, 1.0f, (vector3){0, 0.5, 0});
    if (s < 0) return 1;
    w.bodies[s].velocity = (vector3){10, 0, 0};
    w.bodies[s].angular_velocity = (vector3){0, 0, -20};
    w.bodies[s].restitution = 0.0f;
    const float dt = DT;
    for (int k = 0; k < 60; k++) physics_world_step(&w, dt);
    float v_mid = w.bodies[s].velocity.x;
    for (int k = 0; k < 300; k++) physics_world_step(&w, dt);
    float v_end = w.bodies[s].velocity.x;
    float decel_ratio = (v_mid > 0.001f) ? v_end / v_mid : 0;
    if (v_mid <= 0.1f) return 1;
    if (v_end <= 0.0f || v_end >= 0.5f * v_mid) return 1;
    return 0;
}

/* T6: Motor free speed ±30% */
int mfs_t_motor_free_speed(void) {
    physics_world w;
    mpe_config_init();
    g_cfg.timestep.solver_iterations = 128;
    g_cfg.sleep.enable = 0;
    g_cfg.world.drag = 1.0f;
    g_cfg.world.angular_damping_scale = 1.0f;
    physics_world_init(&w);
    constraint_pool_init(&w);
    int f = physics_world_add_cube(&w, (vector3){0.0f, -0.5f, 0.0f},
                                   (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    if (f < 0) return 1;
    w.bodies[f].friction_static = 1.0f;
    w.bodies[f].friction_kinetic = 0.8f;
    w.bodies[f].restitution = 0.0f;
    g_cfg.world.drag = 1.0f;
    g_cfg.world.angular_damping_scale = 1.0f;
    ftc_robot robot;
    int rc = ftc_robot_create_with_drive(&w, &robot, 0.0f, ftc_robot_rest_height(), 0.0f,
                                         MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_TANK);
    if (rc != 0) return 1;
    const vector3 lift = {0.0f, 1.9f, 0.0f};
    rigidbody *chassis = &w.bodies[robot.chassis_body];
    chassis->position = vector3_addition(chassis->position, lift);
    chassis->velocity = vector3_zero(); chassis->angular_velocity = vector3_zero();
    rigidbody_set_kinematic(chassis, true);
    for (int w_idx = 0; w_idx < robot.wheel_count; w_idx++) {
        int wi = robot.wheel_bodies[w_idx];
        if (wi >= 0 && wi < w.body_count) {
            rigidbody *wb = &w.bodies[wi];
            wb->position = vector3_addition(wb->position, lift);
            wb->velocity = vector3_zero(); wb->angular_velocity = vector3_zero();
            rigidbody_update_axes(wb);
            for (int k = 0; k < robot.roller_count[w_idx]; k++) {
                int rb = robot.roller_bodies[w_idx][k];
                if (rb >= 0 && rb < w.body_count) {
                    rigidbody *rbb = &w.bodies[rb];
                    rbb->position = vector3_addition(rbb->position, lift);
                    rbb->velocity = vector3_zero(); rbb->angular_velocity = vector3_zero();
                    rigidbody_update_axes(rbb);
                }
            }
        }
    }
    rigidbody_update_axes(chassis);
    for (int w_idx = 0; w_idx < robot.wheel_count; w_idx++) motor_reset_observer(&robot.wheel_motors[w_idx]);
    for (int i = 0; i < 180; i++) {
        drivetrain_tank(&robot, 1.0f, 1.0f);
        drivetrain_update(&w, &robot, DT);
        physics_world_step(&w, DT);
    }
    float spec_rpm = 223.0f;
    float actual_rpm = robot.wheel_motors[0].rpm;
    float rpm_error = fabsf(actual_rpm - spec_rpm) / spec_rpm;
    if (rpm_error > 0.3f) return 1;
    return 0;
}

/* T7: Motor stall torque ±30% */
int mfs_t_motor_stall(void) {
    physics_world w; mfs_test_world(&w);
    ftc_robot robot;
    int rc = ftc_robot_create_with_drive(&w, &robot, 0.0f, ftc_robot_rest_height(), 0.0f,
                                         MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_TANK);
    if (rc != 0) return 1;
    drivetrain_tank(&robot, 1.0f, 1.0f);
    for (int t = 0; t < 10; t++) {
        for (int w_idx = 0; w_idx < robot.wheel_count; w_idx++) {
            int wi = robot.wheel_bodies[w_idx];
            if (wi >= 0 && wi < w.body_count) {
                w.bodies[wi].angular_velocity = (vector3){0, 0, 0};
            }
        }
        drivetrain_tank(&robot, 1.0f, 1.0f);
        drivetrain_update(&w, &robot, DT);
        physics_world_step(&w, DT);
    }
    float stall_spec = 3.7265f;
    float actual = robot.wheel_motors[0].output_torque;
    float err = fabsf(actual - stall_spec) / stall_spec;
    if (err > 0.3f) return 1;
    return 0;
}

/* T8: Back-EMF braking */
int mfs_t_back_emf(void) {
    physics_world w; mfs_test_world(&w);
    ftc_robot robot;
    int rc = ftc_robot_create_with_drive(&w, &robot, 0.0f, ftc_robot_rest_height(), 0.0f,
                                         MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_TANK);
    if (rc != 0) return 1;
    drivetrain_tank(&robot, 1.0f, 1.0f);
    for (int t = 0; t < 60; t++) {
        drivetrain_update(&w, &robot, DT);
        physics_world_step(&w, DT);
    }
    float v_before = robot.wheel_motors[0].output_torque;
    drivetrain_tank(&robot, 0.0f, 0.0f);
    for (int t = 0; t < 10; t++) {
        drivetrain_update(&w, &robot, DT);
        physics_world_step(&w, DT);
    }
    float v_after = robot.wheel_motors[0].output_torque;
    if (fabsf(v_after) >= 0.5f * fabsf(v_before)) return 1;
    return 0;
}
/* T9: Static friction hold */
int mfs_t_static_friction(void) {
    physics_world w;
    mpe_config_init();
    g_cfg.timestep.solver_iterations = 128;
    g_cfg.sleep.enable = 0;
    physics_world_init(&w);
    constraint_pool_init(&w);
    int f = physics_world_add_cube(&w, (vector3){0, -0.5f, 0}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
    if (f < 0) return 1;
    w.bodies[f].friction_static = 1.0f;
    w.bodies[f].friction_kinetic = 0.8f;
    w.bodies[f].restitution = 0.0f;
    ftc_robot robot;
    int rc = ftc_robot_create_with_drive(&w, &robot, 0.0f, ftc_robot_rest_height(), 0.0f,
                                         MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_TANK);
    if (rc != 0) return 1;
    int s = physics_world_add_cube(&w, (vector3){0, -0.5f, 0}, (vector3){10, 0.5, 10}, 0);
    w.bodies[s].friction_static = 1.0f;
    w.bodies[s].friction_kinetic = 0.8f;
    w.bodies[s].restitution = 0.0f;
    drivetrain_tank(&robot, 0.5f, 0.5f);
    for (int t = 0; t < 300; t++) {
        drivetrain_update(&w, &robot, DT);
        physics_world_step(&w, DT);
    }
    rigidbody *ch = &w.bodies[robot.chassis_body];
    if (fabsf(ch->position.x) >= 0.05f) return 1;
    if (fabsf(ch->velocity.x) >= 0.2f) return 1;
    return 0;
}
/* T10: Kinetic friction a = mu*g ±35% */
int mfs_t_kinetic_friction(void) {
    physics_world w;
    mpe_config_init();
    g_cfg.timestep.solver_iterations = 128;
    g_cfg.sleep.enable = 0;
    physics_world_init(&w);
    constraint_pool_init(&w);
    int s = physics_world_add_cube(&w, (vector3){-10, -0.5f, 0}, (vector3){10, 0.5, 10}, 0);
    if (s < 0) return 1;
    w.bodies[s].friction_static = 1.0f;
    w.bodies[s].friction_kinetic = 0.3f;
    w.bodies[s].restitution = 0.0f;
    int b = physics_world_add_cube(&w, (vector3){0, 0.5f, 0}, (vector3){0.5, 0.5, 0.5}, 1.0f);
    if (b < 0) return 1;
    w.bodies[b].velocity = (vector3){5, 0, 0};
    w.bodies[b].friction_static = 0.3f;
    w.bodies[b].friction_kinetic = 0.3f;
    w.bodies[b].restitution = 0.0f;
    const float dt = 1.0f/60.0f;
    float a_sum = 0;
    int n = 0;
    for (int t = 0; t < 120; t++) {
        physics_world_step(&w, dt);
        if (t >= 10 && n < 10) {
            float a = (w.bodies[b].velocity.x - 5.0f) / (dt * 10.0f);
            a_sum += a; n++;
        }
    }
    float a_avg = n ? a_sum / n : 0;
    float a_expect = 0.3f * 9.81f;
    float err = fabsf(a_avg - a_expect) / a_expect;
    if (err > 0.35f) return 1;
    return 0;
}

/* T11: 3000 ticks no NaN */
int mfs_t_stability(void) {
    physics_world w; mfs_test_world(&w);
    ftc_robot robot;
    int rc = ftc_robot_create_with_drive(&w, &robot, 0.0f, ftc_robot_rest_height(), 0.0f,
                                         MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_TANK);
    if (rc != 0) return 1;
    const float dt = 1.0f/60.0f;
    for (int t = 0; t < 3000; t++) {
        drivetrain_tank(&robot, 0.5f, 0.5f);
        drivetrain_update(&w, &robot, dt);
        physics_world_step(&w, dt);
        if (!mfs_test_finite(&w)) return 1;
    }
    return 0;
}

/* T12: Coast-down after power cut */
int mfs_t_coast_down(void) {
    physics_world w; mfs_test_world(&w);
    ftc_robot robot;
    int rc = ftc_robot_create_with_drive(&w, &robot, 0.0f, ftc_robot_rest_height(), 0.0f,
                                         MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_TANK);
    if (rc != 0) return 1;
    drivetrain_tank(&robot, 1.0f, 1.0f);
    for (int t = 0; t < 60; t++) {
        drivetrain_update(&w, &robot, DT);
        physics_world_step(&w, DT);
    }
    float v_before = w.bodies[robot.chassis_body].velocity.x;
    drivetrain_tank(&robot, 0.0f, 0.0f);
    for (int t = 0; t < 300; t++) {
        drivetrain_update(&w, &robot, DT);
        physics_world_step(&w, DT);
    }
    float v_after = w.bodies[robot.chassis_body].velocity.x;
    if (v_after >= 0.3f * v_before) return 1;
    return 0;
}

/* T13: Energy conservation ±10% */
int mfs_t_energy(void) {
    physics_world w; mfs_test_world(&w);
    int s = physics_world_add_sphere(&w, 0.5f, 1.0f, (vector3){0, 10.0f, 0});
    if (s < 0) return 1;
    w.bodies[s].restitution = 0.0f;
    const float dt = 1.0f/60.0f;
    float E0 = 1.0f * 9.81f * 10.0f;
    for (int k = 0; k < 60; k++) physics_world_step(&w, dt);
    rigidbody *b = &w.bodies[s];
    float PE = b->mass * 9.81f * b->position.y;
    float KE = 0.5f * b->mass * vector3_length_squared(b->velocity) +
               0.5f * vector3_dot(b->angular_velocity,
                                  math3_multiplication_vector3(b->inertia_tensor_local, b->angular_velocity));
    float E = PE + KE;
    float err = fabsf(E - E0) / E0;
    if (err > 0.1f) return 1;
    return 0;
}

/* T14: Cylinder rests on floor ±0.03m, v<0.1 */
int mfs_t_cylinder_rest(void) {
    physics_world w; mfs_test_world(&w);
    int c = physics_world_add_cylinder(&w, 0.05f, 0.5f, 1.0f, (vector3){0, 0.55f, 0});
    if (c < 0) return 1;
    const float dt = 1.0f/60.0f;
    for (int k = 0; k < 300; k++) physics_world_step(&w, dt);
    rigidbody *b = &w.bodies[c];
    float y_err = fabsf(b->position.y - 0.05f);
    if (y_err >= 0.03f) return 1;
    if (fabsf(b->velocity.y) >= 0.1f) return 1;
    return 0;
}

/* T15: Revolute anchor holds under gravity */
int mfs_t_revolute_anchor(void) {
    physics_world w;
    mpe_config_init();
    g_cfg.timestep.solver_iterations = 128;
    g_cfg.sleep.enable = 0;
    physics_world_init(&w);
    constraint_pool_init(&w);
    int a = physics_world_add_cube(&w, (vector3){0, 3, 0}, (vector3){0.1, 0.1, 0.1}, 0);
    int b = physics_world_add_cylinder(&w, 0.05f, 0.5f, 1.0f, (vector3){0, 1.5f, 0});
    w.bodies[b].restitution = 0.0f;
    uint32_t ida = w.bodies[a].object_id;
    uint32_t idb = w.bodies[b].object_id;
    int j = constraint_add_revolute(&w, ida, idb, (vector3){0, -1.5f, 0},
                                    (vector3){0, 0, 0}, (vector3){1, 0, 0});
    if (j < 0) return 1;
    const float dt = 1.0f/60.0f;
    for (int k = 0; k < 300; k++) physics_world_step(&w, dt);
    float len = w.bodies[1].position.y - w.bodies[0].position.y;
    float len0 = 1.5f;
    if (fabsf(len - len0) >= 0.01f) return 1;
    return 0;
}


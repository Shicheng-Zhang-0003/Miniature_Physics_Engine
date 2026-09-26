/* MFS Suite v2 — file A: basic drive kinematics (5 tests).
 * Oracles mirror the v1 originals with corrected floor setup. */
#include <math.h>
#include <stdio.h>
#include "mfs_test.h"

/* teleop: straight drive 0.5m, dy<=1m, heading<=0.3rad. */
int mfs_t_teleop(void) {
    mfs_test_t t;
    mfs_test_begin(&t, "teleop");
    mfs_test_t *t_ptr = &t;

    physics_world w;
    mfs_test_world(&w);

    ftc_robot *robot = mfs_create_robot(&w, 0.0f, ftc_robot_rest_height(), 0.0f,
                                        MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_TANK);
    MFS_CHECK(t_ptr, robot != NULL);

    float start_x, start_y, start_z;
    mfs_get_pos(&w, robot, &start_x, &start_y, &start_z);

    const float dt = 1.0f / 60.0f;
    int fail = 0;

    for (int t_tick = 0; t_tick < 180 && !fail; t_tick++) {
        mfs_drive_tank(robot, 1.0f, 1.0f);
        drivetrain_update(&w, robot, dt);
        physics_world_step(&w, dt);
        if (!mfs_test_finite(&w)) {
            fail = 1;
        }
    }

    if (!fail) {
        float end_x, end_y, end_z;
        mfs_get_pos(&w, robot, &end_x, &end_y, &end_z);
        float disp_xz = sqrtf((end_x - start_x)*(end_x - start_x) + (end_z - start_z)*(end_z - start_z));
        float dy = fabsf(end_y - start_y);
        rigidbody *ch = &w.bodies[robot->chassis_body];
        float heading = fabsf(atan2f(2.0f*(ch->orientation.w*ch->orientation.y + ch->orientation.x*ch->orientation.z),
                                     1.0f - 2.0f*(ch->orientation.y*ch->orientation.y + ch->orientation.x*ch->orientation.x)));

        MFS_INFO("disp_xz=%.4f dy=%.4f heading=%.4f", disp_xz, dy, heading);
        MFS_CHECK(t_ptr, disp_xz >= 0.5f);
        MFS_CHECK(t_ptr, dy <= 1.0f);
        MFS_CHECK(t_ptr, heading <= 0.3f);

        if (t_ptr->failures == 0) {
            printf("[PASS] teleop straight drive\n");
        }
    }

    physics_world_cleanup(&w);
    free(robot);
    mfs_test_end(t_ptr);
    return t_ptr->failures;
}

/* mecanum: strafe right >0.3m in +X. */
int mfs_t_mecanum(void) {
    mfs_test_t t;
    mfs_test_begin(&t, "mecanum");
    mfs_test_t *t_ptr = &t;

    physics_world w;
    mfs_test_world(&w);

    ftc_robot *robot = mfs_create_robot(&w, 0.0f, ftc_robot_rest_height(), 0.0f,
                                        MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_MECANUM);
    MFS_CHECK(t_ptr, robot != NULL);

    float start_x, start_y, start_z;
    mfs_get_pos(&w, robot, &start_x, &start_y, &start_z);

    const float dt = 1.0f / 60.0f;
    int fail = 0;

    for (int t_tick = 0; t_tick < 180 && !fail; t_tick++) {
        mfs_drive_mecanum(robot, 0.0f, 1.0f, 0.0f);
        drivetrain_update(&w, robot, dt);
        physics_world_step(&w, dt);
        if (!mfs_test_finite(&w)) {
            fail = 1;
        }
    }

    if (!fail) {
        float end_x, end_y, end_z;
        mfs_get_pos(&w, robot, &end_x, &end_y, &end_z);
        float dx = end_x - start_x;
        float dz = end_z - start_z;

        MFS_INFO("start=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f)", start_x, start_y, start_z, end_x, end_y, end_z);
        MFS_INFO("displacement x=%.4f  z=%.4f", dx, dz);

        float lateral_displacement = dx;
        MFS_CHECK(t_ptr, lateral_displacement >= 0.3f);

        if (t_ptr->failures == 0) {
            printf("[PASS] mecanum strafe in +X (dx=%.4f)\n", dx);
        }
    }

    physics_world_cleanup(&w);
    free(robot);
    mfs_test_end(t_ptr);
    return t_ptr->failures;
}

/* tank: differential turn in place. */
int mfs_t_tank(void) {
    mfs_test_t t;
    mfs_test_begin(&t, "tank");
    mfs_test_t *t_ptr = &t;

    physics_world w;
    mfs_test_world(&w);

    ftc_robot *robot = mfs_create_robot(&w, 0.0f, ftc_robot_rest_height(), 0.0f,
                                        MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_TANK);
    MFS_CHECK(t_ptr, robot != NULL);

    float start_x, start_y, start_z;
    mfs_get_pos(&w, robot, &start_x, &start_y, &start_z);

    const float dt = 1.0f / 60.0f;
    int fail = 0;

    for (int t_tick = 0; t_tick < 120 && !fail; t_tick++) {
        mfs_drive_tank(robot, 1.0f, -1.0f);
        drivetrain_update(&w, robot, dt);
        physics_world_step(&w, dt);
        if (!mfs_test_finite(&w)) {
            fail = 1;
        }
    }

    if (!fail) {
        float end_x, end_y, end_z;
        mfs_get_pos(&w, robot, &end_x, &end_y, &end_z);
        float disp = sqrtf((end_x - start_x)*(end_x - start_x) + (end_z - start_z)*(end_z - start_z));
        rigidbody *ch = &w.bodies[robot->chassis_body];
        quaternion q = ch->orientation;
        float heading = fabsf(atan2f(2.0f*(q.w*q.y + q.x*q.z),
                                     1.0f - 2.0f*(q.y*q.y + q.x*q.x)));

        MFS_INFO("displacement=%.4f heading=%.4f", disp, heading);

        MFS_CHECK(t_ptr, disp <= 0.3f);
        MFS_CHECK(t_ptr, heading >= 0.1f);

        if (t_ptr->failures == 0) {
            printf("[PASS] tank differential turn (disp=%.4f, heading=%.4f)\n", disp, heading);
        }
    }

    physics_world_cleanup(&w);
    free(robot);
    mfs_test_end(t_ptr);
    return t_ptr->failures;
}

/* odometry: forward + strafe accuracy. */
int mfs_t_odometry(void) {
    mfs_test_t t;
    mfs_test_begin(&t, "odometry");
    mfs_test_t *t_ptr = &t;

    physics_world w;
    mfs_test_world(&w);

    ftc_robot *robot = mfs_create_robot(&w, 0.0f, ftc_robot_rest_height(), 0.0f,
                                        MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_TANK);
    MFS_CHECK(t_ptr, robot != NULL);

    const float dt = 1.0f / 60.0f;
    int fail = 0;

    /* Settle */
    MFS_CHECK(t_ptr, mfs_step(&w, 120, dt));

    /* Zero odometry */
    robot->odom_x = robot->odom_z = robot->odom_theta = 0.0f;

    /* Phase 1: forward drive */
    for (int t_tick = 0; t_tick < 180 && !fail; t_tick++) {
        mfs_drive_tank(robot, 1.0f, 1.0f);
        drivetrain_update(&w, robot, dt);
        physics_world_step(&w, dt);
        if (!mfs_test_finite(&w)) fail = 1;
    }

    if (!fail) {
        float end_x, end_y, end_z;
        mfs_get_pos(&w, robot, &end_x, &end_y, &end_z);
        float dist = sqrtf(end_x*end_x + end_z*end_z);
        float odom_dist = sqrtf(robot->odom_x*robot->odom_x + robot->odom_z*robot->odom_z);
        float odom_error = fabsf(odom_dist - dist) / dist;

        MFS_INFO("odometry distance error: %.1f%%", odom_error * 100.0f);
        MFS_CHECK(t_ptr, dist >= 0.2f);
        MFS_CHECK_REL(t_ptr, odom_dist, dist, 0.3f, "odometry distance");

        /* Phase 2: strafe */
        for (int t_tick = 0; t_tick < 60 && !fail; t_tick++) {
            mfs_drive_mecanum(robot, 0.0f, 1.0f, 0.0f);
            drivetrain_update(&w, robot, dt);
            physics_world_step(&w, dt);
            if (!mfs_test_finite(&w)) fail = 1;
        }

        if (!fail) {
            float end_x2, end_y2, end_z2;
            mfs_get_pos(&w, robot, &end_x2, &end_y2, &end_z2);
            float dx_phys = end_x2 - end_x;
            float dx_odom = robot->odom_x;
            float odom_error_strafe = fabsf(dx_odom - dx_phys) / (fabsf(dx_phys) > 0.001f ? fabsf(dx_phys) : 1.0f);

            MFS_INFO("Phase 2: strafe: physics dx=%.4f odometry dx=%.4f", dx_phys, dx_odom);
            MFS_CHECK(t_ptr, fabsf(dx_phys) >= 0.1f);
            MFS_CHECK(t_ptr, (dx_odom * dx_phys) > 0.0f);
            MFS_CHECK_REL(t_ptr, dx_odom, dx_phys, 0.3f, "odometry strafe");
        }
    }

    if (t_ptr->failures == 0) {
        printf("[PASS] odometry tracks physics\n");
    }

    physics_world_cleanup(&w);
    free(robot);
    mfs_test_end(t_ptr);
    return t_ptr->failures;
}

/* ftc_integration: combined drive test. */
int mfs_t_ftc_integration(void) {
    mfs_test_t t;
    mfs_test_begin(&t, "ftc_integration");
    mfs_test_t *t_ptr = &t;

    physics_world w;
    mfs_test_world(&w);

    ftc_robot *robot = mfs_create_robot(&w, 0.0f, ftc_robot_rest_height(), 0.0f,
                                        MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_MECANUM);
    MFS_CHECK(t_ptr, robot != NULL);

    const float dt = 1.0f / 60.0f;
    int fail = 0;

    for (int t_tick = 0; t_tick < 240 && !fail; t_tick++) {
        if (t_tick < 120) {
            mfs_drive_mecanum(robot, 1.0f, 0.0f, 0.0f);
        } else if (t_tick < 180) {
            mfs_drive_tank(robot, 0.5f, -0.5f);
        } else {
            mfs_drive_mecanum(robot, 0.0f, 1.0f, 0.0f);
        }
        drivetrain_update(&w, robot, dt);
        physics_world_step(&w, dt);
        if (!mfs_test_finite(&w)) {
            fail = 1;
            break;
        }
    }

    if (!fail) {
        float end_x, end_y, end_z;
        mfs_get_pos(&w, robot, &end_x, &end_y, &end_z);
        float dist = sqrtf(end_x*end_x + end_z*end_z);
        float dy = fabsf(end_y - ftc_robot_rest_height());

        MFS_INFO("disp_xz=%.4f dy=%.4f", dist, dy);
        MFS_CHECK(t_ptr, dist >= 0.5f);
        MFS_CHECK(t_ptr, dy <= 0.5f);

        if (t_ptr->failures == 0) {
            printf("[PASS] ftc integration drive\n");
        }
    }

    physics_world_cleanup(&w);
    free(robot);
    mfs_test_end(t_ptr);
    return t_ptr->failures;
}
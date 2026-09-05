/* MPE_FTC_073: FTC robot object implementation */
/* MPE_FTC_094_CLEANUP: wheel_traction removed — real cylinder friction */
#include "robot.h"
#include "../physics/constraint.h"
#include <math.h>
#include <string.h>

/* Robot dimensions (metres, approximate FTC 18" x 18" chassis) */
#define CHASSIS_HALF_X 0.225f
#define CHASSIS_HALF_Y 0.075f
#define CHASSIS_HALF_Z 0.225f
#define CHASSIS_MASS 8.0f /* ~18 lb robot */
#define WHEEL_RADIUS 0.05f /* 100mm wheels */
#define WHEEL_MASS 0.2f
#define WHEEL_HALF_WIDTH 0.02f /* 40mm wide wheels */
#define WHEEL_OFFSET_X 0.24f /* slightly outside chassis */
#define WHEEL_OFFSET_Z 0.20f
#define WHEEL_Y_OFFSET (-CHASSIS_HALF_Y - WHEEL_RADIUS + 0.01f)

/* MPE_FTC_095: chassis-centre height where the wheels just touch floor y=0 */
float ftc_robot_rest_height(void) {
    return WHEEL_RADIUS - WHEEL_Y_OFFSET;
}

int ftc_robot_create_with_drive(physics_world *world, ftc_robot *robot, float x, float y, float z,
                                motor_preset_id preset, ftc_drivetrain_type drivetrain_type) {
/* MFS_161_NULL_FIX: null-check FIRST, before any dereference */
if ((!world) || (!robot)) {
return 1;
}
memset(robot, 0, sizeof(ftc_robot));
/* memset zeroes odom_x/z/theta and wheel_radians — no separate init needed */
    robot->motor_preset = preset;
    robot->drivetrain_type = drivetrain_type;
    robot->axle_axis_x = 1.0f; /* axles point along X (left-right) */
    robot->axle_axis_y = 0.0f;
    robot->axle_axis_z = 0.0f;
    battery_init(&robot->battery);

    /* Chassis: a box at the given position */
    robot->chassis_body = physics_world_add_cube(
        world, (vector3){x, y, z}, (vector3){CHASSIS_HALF_X, CHASSIS_HALF_Y, CHASSIS_HALF_Z}, CHASSIS_MASS);
    if (robot->chassis_body < 0) {
        return 1;
    }

    uint32_t chassis_id = world->bodies[robot->chassis_body].object_id;

    /* 4 wheels at corners */
    float wheel_positions[4][3] = {
        {x - WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z - WHEEL_OFFSET_Z}, /* front-left */
        {x + WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z - WHEEL_OFFSET_Z}, /* front-right */
        {x - WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z + WHEEL_OFFSET_Z}, /* back-left */
        {x + WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z + WHEEL_OFFSET_Z}, /* back-right */
    };
    robot->wheel_count = 4;

    for (int i = 0; i < robot->wheel_count; i++) {
        /* Create wheel as a sphere (rolling approximation) */
        robot->wheel_bodies[i] =
            physics_world_add_cylinder(world, WHEEL_RADIUS, WHEEL_HALF_WIDTH, WHEEL_MASS,
                                     (vector3){wheel_positions[i][0], wheel_positions[i][1], wheel_positions[i][2]});
        if (robot->wheel_bodies[i] < 0) {
            return 1;
        }

        uint32_t wheel_id = world->bodies[robot->wheel_bodies[i]].object_id;

        /* Revolute joint: chassis (body_a) to wheel (body_b), axle along X */
        vector3 anchor_on_chassis = {wheel_positions[i][0] - x, WHEEL_Y_OFFSET, wheel_positions[i][2] - z};
        vector3 anchor_on_wheel = {0.0f, 0.0f, 0.0f}; /* wheel centre */
        vector3 axle_axis = {robot->axle_axis_x, robot->axle_axis_y, robot->axle_axis_z};

        robot->wheel_joints[i] =
            constraint_add_revolute(chassis_id, wheel_id, anchor_on_chassis, anchor_on_wheel, axle_axis);
        if (robot->wheel_joints[i] < 0) {
            return 1;
        }

        /* MFS_MECANUM_REAL: Mark wheel as mecanum with roller angle.
         * Standard layout: front-left +45°, front-right -45°, back-left -45°, back-right +45° */
        float roller_angle = 0.0f;
        if (i == 0) roller_angle = 0.785398f;       /* front-left: +45° */
        if (i == 1) roller_angle = -0.785398f;      /* front-right: -45° */
        if (i == 2) roller_angle = -0.785398f;      /* back-left: -45° */
        if (i == 3) roller_angle = 0.785398f;       /* back-right: +45° */
        
        if (robot->drivetrain_type == FTC_DRIVETRAIN_MECANUM) {
                rigidbody_set_mecanum(&world->bodies[robot->wheel_bodies[i]], true, roller_angle);
            } else {
                rigidbody_set_mecanum(&world->bodies[robot->wheel_bodies[i]], false, 0.0f);
            }

        /* Set up motor for this wheel */
        motor_preset_apply(&robot->wheel_motors[i], preset);
    }

    return 0;
}


int ftc_robot_create(physics_world *world, ftc_robot *robot, float x, float y, float z, motor_preset_id preset) {
    return ftc_robot_create_with_drive(world, robot, x, y, z, preset, FTC_DRIVETRAIN_MECANUM);
}

void ftc_robot_update(physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {
        return;
    }

    /* Sum currents for battery sag */
    float total_current = 0.0f;
    for (int i = 0; i < robot->wheel_count; i++) {
        total_current += fabsf(robot->wheel_motors[i].current);
    }
    float terminal_voltage = battery_get_voltage(&robot->battery, total_current);
    battery_drain(&robot->battery, total_current, dt);

    /* Update each wheel motor */
    for (int i = 0; i < robot->wheel_count; i++) {
        int wheel_idx = robot->wheel_bodies[i];
        if ((wheel_idx < 0) || (wheel_idx >= world->body_count)) {
            continue;
        }
        rigidbody *wheel = &world->bodies[wheel_idx];

        /* Read wheel angular velocity about the actual rotated axle axis in world space */
        vector3 axle = wheel->cached_axes[0];
        if (vector3_length_squared(axle) < 0.0001f) {
            axle = vector4_rotate_to_vector3(wheel->orientation, (vector3){1.0f, 0.0f, 0.0f});
        }
        float wheel_speed = vector3_dot(wheel->angular_velocity, axle);

        /* Update motor electrical state */
        motor_update(&robot->wheel_motors[i], wheel_speed, dt, terminal_voltage);

        /* Apply motor torque along the actual physical axle in world space */
        float torque = robot->wheel_motors[i].output_torque;
        /* MFS_145_IDLE_BRAKE: back-EMF braking is a damper — it brings a coasting
         * wheel to rest and can never reverse it (no back-EMF once stopped).
         * At idle, clamp the braking torque to the amount that stops the wheel
         * within this timestep. Without this, the stall-clamped back-EMF torque
         * (~2.17 N·m) reverses the light wheel every step -> ±25 rad/s idle spin. */
        if ((fabsf(robot->wheel_motors[i].command) < 0.05f) && ((torque * wheel_speed) < 0.0f)) {
            float mfs_i_axle = 0.5f * wheel->mass * wheel->radius * wheel->radius;
            if (mfs_i_axle > 0.0f) {
                float mfs_max_brake = mfs_i_axle * fabsf(wheel_speed) / dt;
                if (fabsf(torque) > mfs_max_brake) {
                    torque = (torque > 0.0f) ? mfs_max_brake : -mfs_max_brake;
                }
            }
        }
        wheel->torque_accumulator = vector3_addition(
            wheel->torque_accumulator,
            vector3_scaling(axle, torque));
        rigidbody_wake(wheel); /* MPE_FTC_078: keep driven wheels awake so motor torque is applied */
    }
}

void ftc_robot_set_wheel_commands(ftc_robot *robot, const float *commands, int count) {
    if (!robot) {
        return;
    }
    int n = (count < robot->wheel_count) ? count : robot->wheel_count;
    for (int i = 0; i < n; i++) {
        float cmd = commands[i];
        if (cmd > 1.0f) {
            cmd = 1.0f;
        }
        if (cmd < -1.0f) {
            cmd = -1.0f;
        }
        robot->wheel_motors[i].command = cmd;
    }
}

void ftc_robot_get_position(physics_world *world, ftc_robot *robot, float *px, float *py, float *pz) {
    if ((!world) || (!robot)) {
        return;
    }
    int idx = robot->chassis_body;
    if ((idx < 0) || (idx >= world->body_count)) {
        return;
    }
    if (px) {
        *px = world->bodies[idx].position.x;
    }
    if (py) {
        *py = world->bodies[idx].position.y;
    }
    if (pz) {
        *pz = world->bodies[idx].position.z;
    }
}

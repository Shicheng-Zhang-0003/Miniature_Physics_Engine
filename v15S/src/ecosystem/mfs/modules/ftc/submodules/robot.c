/* MPE_FTC_073: FTC robot object implementation */
/* MPE_FTC_094_CLEANUP: wheel_traction removed — real cylinder friction */
#include "robot.h"
#include "physics/constraint.h"
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
#define WHEEL_Y_OFFSET (-CHASSIS_HALF_Y - WHEEL_RADIUS - 0.005f) /* MFS_PORT_V15S: 5mm ground clearance. The old +0.01f tucked wheel tops 10mm INSIDE the chassis box, so the contact solver fought the joint-held pose every tick (sinking, chatter, pitch-unload). Joints hold anchors, not volumes — interpenetrating rest poses are solver poison. */
#define WHEEL_PRELOAD 0.002f /* MFS_PORT_V15S: joint anchors sit 2mm BELOW exact touch so P2P preloads wheels into persistent floor contact. Inside slop (10mm): no positional fight, but the manifold never grazes out to a hover-skid (which starves odometry). Real suspensions run droop/preload the same way. */

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
    /* Traction scales start open (1.0); memset leaves 0.0 = fully cut. */
    for (int i = 0; i < FTC_MAX_WHEELS; i++) {
        robot->wheel_traction_scale[i] = 1.0f;
    }
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
        /* Grippy rubber on tile (was engine defaults ~0.3/0.2: glassy).
         * Contact mu = min(wheel, floor); the drivetrain budgets against
         * these same wheel materials (see drivetrain_update). */
        world->bodies[robot->wheel_bodies[i]].friction_static = 0.9f;
        world->bodies[robot->wheel_bodies[i]].friction_kinetic = 0.7f;
        world->bodies[robot->wheel_bodies[i]].restitution = 0.0f;

        uint32_t wheel_id = world->bodies[robot->wheel_bodies[i]].object_id;

        /* Revolute joint: chassis (body_a) to wheel (body_b), axle along X.
         * Anchor sits WHEEL_PRELOAD below exact touch (see above). */
        vector3 anchor_on_chassis = {wheel_positions[i][0] - x, WHEEL_Y_OFFSET - WHEEL_PRELOAD,
                                     wheel_positions[i][2] - z};
        vector3 anchor_on_wheel = {0.0f, 0.0f, 0.0f}; /* wheel centre */
        vector3 axle_axis = {robot->axle_axis_x, robot->axle_axis_y, robot->axle_axis_z};

        robot->wheel_joints[i] =
            constraint_add_revolute(world, chassis_id, wheel_id, anchor_on_chassis, anchor_on_wheel,
                                    axle_axis);
        if (robot->wheel_joints[i] < 0) {
            return 1;
        }

        /* MFS_PORT_V15S: roller geometry is robot-local state now (the
         * parked rigidbody is_mecanum/roller_angle_rad fields are gone
         * from the core). Standard layout: FL +45°, FR -45°, BL -45°,
         * BR +45°. Tank robots get 0/false (plain cylinders). */
        float roller_angle = 0.0f;
        bool is_mecanum = false;
        if (robot->drivetrain_type == FTC_DRIVETRAIN_MECANUM) {
            is_mecanum = true;
            if (i == 0) roller_angle = 0.785398f;       /* front-left: +45° */
            else if (i == 1) roller_angle = -0.785398f; /* front-right: -45° */
            else if (i == 2) roller_angle = -0.785398f; /* back-left: -45° */
            else if (i == 3) roller_angle = 0.785398f;  /* back-right: +45° */
        }
        robot->wheel_roller_angle[i] = roller_angle;
        robot->wheel_is_mecanum[i] = is_mecanum;

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

    /* A commanded robot is awake by definition. Wake the chassis when any
     * wheel is commanded: motor vibration and driver intent keep a real
     * robot active, and the velocity integrator drains forces for sleeping
     * bodies, so a sleeping chassis would swallow traction forever after
     * an idle settle. Wheels are woken below per-wheel. */
    int any_command = 0;
    for (int i = 0; i < robot->wheel_count; i++) {
        if (fabsf(robot->wheel_motors[i].command) > 0.01f) { any_command = 1; break; }
    }
    if (any_command && robot->chassis_body >= 0 && robot->chassis_body < world->body_count) {
        rigidbody_wake(&world->bodies[robot->chassis_body]);
    }
    /* A sleeping chassis with real velocity is inconsistent state: the
     * velocity integrator drains forces for sleepers, so motion freezes
     * mid-drift (measured T8 hold failure). Truly settled bodies sit
     * below the sleep thresholds; anything above wakes. */
    if (robot->chassis_body >= 0 && robot->chassis_body < world->body_count) {
        rigidbody *chb = &world->bodies[robot->chassis_body];
        if (chb->is_sleeping) {
            const mpe_config_t *sleep_cfg = mpe_world_cfg(world);
            float lv2 = vector3_length_squared(chb->velocity);
            float av2 = vector3_length_squared(chb->angular_velocity);
            if (lv2 > sleep_cfg->sleep.linear_thresh_sq ||
                av2 > sleep_cfg->sleep.angular_thresh_sq) {
                rigidbody_wake(chb);
            }
        }
    }

    /* Sum currents for battery sag.
     * FIX-AUDIT: old fabs() sum doubled sag in turn-in-place (opposing
     * currents cancel on a real pack) and made regen always drain. Use
     * signed sum for sag; drain only net positive (regen credited with
     * 50% efficiency, SoC clamped in battery_drain path). */
    float total_current_signed = 0.0f;
    for (int i = 0; i < robot->wheel_count; i++) {
        total_current_signed += robot->wheel_motors[i].current;
    }
    /* Pack fuse sees the absolute bus load (stall sums, regen doesn't
     * cool the fuse). */
    float fuse_load = 0.0f;
    for (int i = 0; i < robot->wheel_count; i++) {
        float c = robot->wheel_motors[i].current;
        fuse_load += (c > 0.0f) ? c : -c;
    }
    battery_fuse_step(&robot->battery, fuse_load, dt);
    float terminal_voltage = battery_get_voltage(&robot->battery, total_current_signed);
    float drain_current = (total_current_signed > 0.0f) ? total_current_signed : 0.5f * total_current_signed;
    if (drain_current < 0.0f && robot->battery.charge_fraction >= 1.0f) {
        drain_current = 0.0f;
    }
    battery_drain(&robot->battery, drain_current, dt);

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
        /* Disturbance observer: external load = measured net torque effect
         * minus last tick's explicit motor torque. At lock this converges
         * to -stall (full stall held); free, to 0. Clamped; NaN-safe. */
        float axle_I = 0.5f * wheel->mass * wheel->radius * wheel->radius;
        if (robot->wheel_motors[i].wprev_valid && axle_I > 0.0f && dt > 0.0f &&
            isfinite(wheel_speed)) {
            float tau_l = axle_I * (wheel_speed - robot->wheel_motors[i].w_prev) / dt -
                          robot->wheel_motors[i].tau_exp_prev;
            if (!isfinite(tau_l)) {
                tau_l = 0.0f;
            } else if (tau_l > 100.0f) {
                tau_l = 100.0f;
            } else if (tau_l < -100.0f) {
                tau_l = -100.0f;
            }
            robot->wheel_motors[i].load_torque = tau_l;
        }
        robot->wheel_motors[i].w_prev = isfinite(wheel_speed) ? wheel_speed : 0.0f;
        robot->wheel_motors[i].wprev_valid = 1;

        /* MFS_TRACTION_CONTROL: compare against the rolling speed the
         * chassis motion demands at this wheel (rigid-body velocity at
         * the wheel center, projected on the rolling direction). A wheel
         * spinning far from demand is slipping: cut its torque so kinetic
         * friction re-captures it instead of sliding forever. Scales
         * recover toward open when slip clears (hysteresis via margin). */
        /* No contact → no slip: airborne wheels (free-spin rigs, jumps)
         * must not trip the traction cut. Same 0.05 clearance the
         * traction loop uses. */
        float wheel_bottom = wheel->position.y - wheel->radius;
        if (robot->chassis_body >= 0 && robot->chassis_body < world->body_count &&
            wheel->radius > 0.001f && wheel_bottom <= 0.05f) {
            rigidbody *chassis = &world->bodies[robot->chassis_body];
            vector3 r_ch_wh = vector3_subtraction(wheel->position, chassis->position);
            vector3 v_contact = vector3_addition(
                chassis->velocity, vector3_cross(chassis->angular_velocity, r_ch_wh));
            vector3 roll_dir = vector3_cross(axle, (vector3){0.0f, 1.0f, 0.0f});
            float w_expected = 0.0f;
            if (vector3_length_squared(roll_dir) > 1e-6f) {
                roll_dir = vector3_scaling(roll_dir, 1.0f / sqrtf(vector3_length_squared(roll_dir)));
                w_expected = vector3_dot(v_contact, roll_dir) / wheel->radius;
            }
            float slip = wheel_speed - w_expected;
            /* Cut ONLY overspeed (wheel outrunning travel = burnout).
             * Under-speed (skid/drag) keeps full torque so the wheel
             * spins UP to rolling speed; cutting there deadlocks the
             * wheel at zero while traction drags the chassis. */
            float dir = 0.0f;
            if (w_expected > 1e-3f) {
                dir = 1.0f;
            } else if (w_expected < -1e-3f) {
                dir = -1.0f;
            }
            float *scale = &robot->wheel_traction_scale[i];
            float over = slip * dir;
            if (dir != 0.0f && over > 4.0f) {
                *scale = 0.15f;
            } else if (*scale < 1.0f && over < 2.0f) {
                *scale += 0.2f;
                if (*scale > 1.0f) {
                    *scale = 1.0f;
                }
            }
        }

        /* Update motor electrical state (implicit-in-speed: stable for
         * light wheels; same stall/free endpoints as explicit). */
        float axle_inertia = 0.5f * wheel->mass * wheel->radius * wheel->radius;
        motor_update_load(&robot->wheel_motors[i], wheel_speed, dt, terminal_voltage, axle_inertia);

        /* Traction cut applies to delivered torque (both the axle drive
         * below and the traction loop in drivetrain_update read
         * output_torque). Electrical readings (current/rpm) stay
         * unscaled: they report the commanded state. */
        robot->wheel_motors[i].output_torque *= robot->wheel_traction_scale[i];

        /* Apply motor torque along the actual physical axle in world space.
         * Free-speed governor: a motor cannot push its wheel past free
         * speed under its own power (measured pathology: +248 rad/s in
         * ONE tick at 10.6x free speed). Below free speed torque is
         * untouched, preserving full stall for breakaway grip; only the
         * overshoot past 1.1x free speed is clipped to land on the bound.
         * (An old no-overshoot-everywhere clamp is NOT used: it capped
         * torque below the static-grip cone and stalled breakaway.) */
        float torque = robot->wheel_motors[i].output_torque;
        {
            float wfree = fabsf(robot->wheel_motors[i].free_speed_rad_s) * 1.1f;
            float iaxle = 0.5f * wheel->mass * wheel->radius * wheel->radius;
            if (iaxle > 0.0f && wfree > 0.0f && dt > 0.0f) {
                float wpred = wheel_speed + torque * dt / iaxle;
                if (fabsf(wpred) > wfree && (wpred * torque) > 0.0f) {
                    torque = ((wpred > 0.0f) ? wfree : -wfree) - wheel_speed;
                    torque *= iaxle / dt;
                }
            }
        }
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
        /* MFS_PORT_V15S: the parked core wheel-lock loop (and its
         * driven_this_tick gate) is gone; driven wheels are kept awake
         * directly below instead. */
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

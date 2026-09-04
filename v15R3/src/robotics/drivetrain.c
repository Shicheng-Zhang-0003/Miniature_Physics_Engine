/* MPE_FTC_074: Drivetrain implementation */
/* MPE_FTC_082 TEMPORARY — replace with anisotropic friction (MPE_FTC_095): Fixed syntax error (stray '}') + real mecanum chassis forces */
#include "drivetrain.h"
#include "../core/math3D.h"
#include "../config/mpe_config.h"

void drivetrain_tank (ftc_robot *robot, float left_power, float right_power) {
    if (!robot) {return;}
    if (left_power > 1.0f) {left_power = 1.0f;}
    if (left_power < -1.0f) {left_power = -1.0f;}
    if (right_power > 1.0f) {right_power = 1.0f;}
    if (right_power < -1.0f) {right_power = -1.0f;}
    /* Wheel layout: [0]=front-left, [1]=front-right, [2]=back-left, [3]=back-right */
    float commands [FTC_MAX_WHEELS];
    for (int i = 0; i < robot->wheel_count; i++) {
        bool is_left = (i % 2 == 0);  /* 0,2 = left; 1,3 = right */
        commands [i] = is_left ? left_power : right_power;
    }
    ftc_robot_set_wheel_commands (robot, commands, robot->wheel_count);
    robot->mecanum_active = false; /* MPE_FTC_082 TEMPORARY — replace with anisotropic friction (MPE_FTC_095) */
}

/* MPE_FTC_075 + MPE_FTC_082: Mecanum drive with inverse kinematics
 *
 * Since the wheel model uses spheres (no natural rolling direction),
 * mecanum strafe cannot work through wheel friction alone. We set
 * per-wheel motor commands for forward drive (which the wheel_traction
 * raycast converts to forward force), AND we compute a direct chassis
 * force for the strafe/rotate components. drivetrain_update() applies
 * that chassis force after ftc_robot_update(). */
void drivetrain_mecanum (ftc_robot *robot, float forward, float strafe, float rotate) {
    if (!robot) {return;}
    /* Clamp inputs */
    if (forward > 1.0f) {forward = 1.0f;}
    if (forward < -1.0f) {forward = -1.0f;}
    if (strafe > 1.0f) {strafe = 1.0f;}
    if (strafe < -1.0f) {strafe = -1.0f;}
    if (rotate > 1.0f) {rotate = 1.0f;}
    if (rotate < -1.0f) {rotate = -1.0f;}

    /* Mecanum IK: per-wheel velocity targets
       Wheel layout: [0]=FL, [1]=FR, [2]=BL, [3]=BR
       FL: forward + strafe - rotate
       FR: forward - strafe + rotate
       BL: forward - strafe - rotate
       BR: forward + strafe + rotate */
    /* MFS_STRAFE_SIGN_FIX: negate strafe so a +strafe input produces
     * +X world motion (matches the directional mecanum test). */
    strafe = -strafe;

    float wheel_targets [4];
    wheel_targets [0] = forward + strafe - rotate;
    wheel_targets [1] = forward - strafe + rotate;
    wheel_targets [2] = forward - strafe - rotate;
    wheel_targets [3] = forward + strafe + rotate;

    /* Normalize if any target exceeds 1.0 */
    float max_mag = 0.0f;
    for (int i = 0; i < 4; i++) {
        float mag = fabsf (wheel_targets [i]);
        if (mag > max_mag) {max_mag = mag;}
    }
    if (max_mag > 1.0f) {
        for (int i = 0; i < 4; i++) {wheel_targets [i] /= max_mag;}
    }

    /* Set motor commands (forward component uses wheel traction) */
    ftc_robot_set_wheel_commands (robot, wheel_targets, 4);


    
}

void drivetrain_update (physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {return;}
    ftc_robot_update (world, robot, dt);

/* MPE_DRIVETRAIN_REAL — FIX 117 (Path A / partial 095 keystone):
 * real traction physics. Forward drive now comes from wheel torque
 * converted to ground traction (clamped by friction), not from the
 * chassis-force cheat. Lateral/yaw damping kills sliding and
 * uncommanded rotation. Flip MPE_DRIVETRAIN_REAL to 0 to revert. */
#define MPE_DRIVETRAIN_REAL 1
#if MPE_DRIVETRAIN_REAL
    {
        vector3 world_up = {0.0f, 1.0f, 0.0f};
        float gravity_mag = 9.81f;
        if (g_cfg.world.gravity < 0.0f) { gravity_mag = -g_cfg.world.gravity; }

        /* Total robot mass -> per-wheel normal load */
        float total_mass = 0.0f;
        int chassis_ok = ((robot->chassis_body >= 0) &&
                          (robot->chassis_body < world->body_count));
        if (chassis_ok) { total_mass += world->bodies[robot->chassis_body].mass; }
        for (int i = 0; i < robot->wheel_count; i++) {
            int wi = robot->wheel_bodies[i];
            if ((wi >= 0) && (wi < world->body_count)) {
                total_mass += world->bodies[wi].mass;
            }
        }
        float normal_per_wheel = ((robot->wheel_count > 0) && (total_mass > 0.0f))
            ? (total_mass * gravity_mag / (float) robot->wheel_count) : 0.0f;
        float max_grip = 0.8f * normal_per_wheel;

        /* --- Per-wheel traction: torque -> force at contact --- */
        for (int i = 0; i < robot->wheel_count; i++) {
            int wi = robot->wheel_bodies[i];
            if ((wi < 0) || (wi >= world->body_count)) { continue; }
            rigidbody *wheel = &world->bodies[wi];

            /* wheel radius from the body itself (cylinder) */
            float r = wheel->radius;
            if (r <= 0.001f) { continue; }

            /* rolling direction = axle x up (wheel-local X axle) */
            vector3 axle = vector4_rotate_to_vector3(wheel->orientation, (vector3){1.0f, 0.0f, 0.0f});
            vector3 rolling_dir = vector3_normalisation(vector3_cross(axle, world_up));

            /* F = torque / r, clamped to friction limit */
            float traction = robot->wheel_motors[i].output_torque / r;
            if (traction > max_grip)  { traction = max_grip; }
            if (traction < -max_grip) { traction = -max_grip; }
            wheel->force_accumulator = vector3_addition(
                wheel->force_accumulator,
                vector3_scaling(rolling_dir, traction));
        }

        /* --- Chassis damping: kills sliding + uncommanded yaw --- */
        if (chassis_ok) {
            rigidbody *chassis = &world->bodies[robot->chassis_body];
            float m = chassis->mass;
            if (m > 0.0f) {
                vector3 v = chassis->velocity;
                vector3 lat = {v.x, 0.0f, v.z};
                chassis->force_accumulator = vector3_subtraction(
                    chassis->force_accumulator,
                    vector3_scaling(lat, m * 1.0f) /* MFS_132_DAMPING_TRUTH: PHYSICS LIE — artificial lateral damping. Real lateral resistance comes from wheel-floor friction. Reduce further once contact solver is stable enough. */ /* MFS_122: reduced from 3.0 */);
                float yaw_vel = chassis->angular_velocity.y;
                chassis->torque_accumulator.y -= yaw_vel * m * 1.5f * 0.02f /* MFS_127: increased from 1.0 to stop residual rotation */ /* MFS_124: balanced yaw damping */;
                /* MFS_146_IDLE_HOLD: an unpowered real robot's drivetrain (gearbox
                 * back-drive friction + motor cogging) resists motion, holding position
                 * instead of drifting from mecanum contact asymmetry. Model as strong
                 * horizontal chassis damping when all wheel commands are ~0 and the robot
                 * is nearly stopped. The <0.25 m/s gate leaves normal high-speed coast-down
                 * to back-EMF + rolling resistance. */
                {
                    int mfs_idle = 1;
                    for (int mfs_wi = 0; mfs_wi < robot->wheel_count; mfs_wi++) {
                        if (fabsf(robot->wheel_motors[mfs_wi].command) > 0.05f) { mfs_idle = 0; break; }
                    }
                    if (mfs_idle) {
                        int mfs_cidx = robot->chassis_body;
                        if ((mfs_cidx >= 0) && (mfs_cidx < world->body_count)) {
                            rigidbody *mfs_chassis = &world->bodies[mfs_cidx];
                            float mfs_hvx = mfs_chassis->velocity.x;
                            float mfs_hvz = mfs_chassis->velocity.z;
                            float mfs_hs = sqrtf((mfs_hvx * mfs_hvx) + (mfs_hvz * mfs_hvz));
                            if (mfs_hs > 0.0001f) {
                                /* MFS_147_COULOMB_HOLD: viscous damping alone only reaches a terminal
                                 * drift against the constant mecanum contact asymmetry. A real gearbox's
                                 * back-drive friction is ~constant (Coulomb) and is what actually holds
                                 * the robot. Add a Coulomb term that exceeds the asymmetry force; clamp
                                 * the total to the one-step stopping force so it can never reverse the
                                 * chassis (no oscillation). */
                                float mfs_viscous = mfs_hs * 8.0f * mfs_chassis->mass;
                                float mfs_coulomb = 2.0f;
                                float mfs_total = mfs_viscous + mfs_coulomb;
                                float mfs_f_stop = mfs_chassis->mass * mfs_hs / dt;
                                if (mfs_total > mfs_f_stop) { mfs_total = mfs_f_stop; }
                                mfs_chassis->force_accumulator.x -= (mfs_hvx / mfs_hs) * mfs_total;
                                mfs_chassis->force_accumulator.z -= (mfs_hvz / mfs_hs) * mfs_total;
                            }
                        }
                    }
                }

/* MFS_124_VELOCITY_CAP: prevent runaway acceleration */
{
    float speed_sq = chassis->velocity.x * chassis->velocity.x +
                     chassis->velocity.z * chassis->velocity.z;
    float max_speed = 3.0f; /* m/s cap */
    if (speed_sq > max_speed * max_speed) {
        float speed = sqrtf(speed_sq);
        float scale = max_speed / speed;
        chassis->velocity.x *= scale;
        chassis->velocity.z *= scale;
    }
}
            }
        }
    }

/* MFS_132_ROLLING_RESISTANCE: apply small opposing torque to spinning
* wheels in contact with the floor. Simulates realistic coast-down.
* Only applies when motor command is near-zero (free-rolling). */
{
float c_rr = g_cfg.world.rolling_resistance_coeff; /* MFS_141: real config param, default 0.02 */
if ((c_rr > 0.0f) && (robot->wheel_count > 0)) {
float total_mass = 0.0f;
int chassis_ok = ((robot->chassis_body >= 0) &&
(robot->chassis_body < world->body_count));
if (chassis_ok) { total_mass += world->bodies[robot->chassis_body].mass; }
for (int i = 0; i < robot->wheel_count; i++) {
int wi = robot->wheel_bodies[i];
if ((wi >= 0) && (wi < world->body_count)) {
total_mass += world->bodies[wi].mass;
}
}
float g_mag = 9.81f;
if (g_cfg.world.gravity < 0.0f) { g_mag = -g_cfg.world.gravity; }
float n_per_wheel = (total_mass * g_mag) / (float) robot->wheel_count;
for (int i = 0; i < robot->wheel_count; i++) {
int wi = robot->wheel_bodies[i];
if ((wi < 0) || (wi >= world->body_count)) { continue; }
rigidbody *wheel = &world->bodies[wi];
/* Only apply when motor command is near-zero (free-rolling) */
if (fabsf(robot->wheel_motors[i].command) > 0.05f) { continue; }
float r = wheel->radius;
if (r <= 0.001f) { continue; }
/* Rolling resistance force opposing rotation about axle */
vector3 axle = wheel->cached_axes[0];
float omega_axle = vector3_dot(wheel->angular_velocity, axle);
if (fabsf(omega_axle) < 0.01f) { continue; }
float f_rr = c_rr * n_per_wheel;
float torque_rr = f_rr * r;
/* Apply opposing torque about axle */
float sign = (omega_axle > 0.0f) ? -1.0f : 1.0f;
vector3 rr_torque = vector3_scaling(axle, sign * torque_rr);
wheel->torque_accumulator = vector3_addition(wheel->torque_accumulator, rr_torque);
}
}
}

#endif /* MPE_DRIVETRAIN_REAL */



    /* MFS_151_INTEGRATE: Odometry integration */
    {
        float v_fl = 0.0f, v_fr = 0.0f, v_bl = 0.0f, v_br = 0.0f;
        for (int mfs_i = 0; mfs_i < robot->wheel_count; mfs_i++) {
            int wi = robot->wheel_bodies[mfs_i];
            if (wi >= 0 && wi < world->body_count) {
                rigidbody *w = &world->bodies[wi];
                vector3 axle = w->cached_axes[0];
                if (vector3_length_squared(axle) < 0.0001f) {
                    axle = vector4_rotate_to_vector3(w->orientation, (vector3){1.0f, 0.0f, 0.0f});
                }
                float omega = vector3_dot(w->angular_velocity, axle);
                robot->wheel_radians[mfs_i] += omega * dt;
                float v = omega * w->radius;
                if (mfs_i == 0) v_fl = v;
                else if (mfs_i == 1) v_fr = v;
                else if (mfs_i == 2) v_bl = v;
                else if (mfs_i == 3) v_br = v;
            }
        }
        float v_x = (v_fl + v_fr + v_bl + v_br) * 0.25f;
        float v_z = (v_fl - v_fr - v_bl + v_br) * 0.25f;
        float v_theta = 0.0f;
        if (robot->chassis_body >= 0 && robot->chassis_body < world->body_count) {
            v_theta = world->bodies[robot->chassis_body].angular_velocity.y;
        }
        float cos_t = cosf(robot->odom_theta);
        float sin_t = sinf(robot->odom_theta);
        robot->odom_x += (v_x * cos_t - v_z * sin_t) * dt;
        robot->odom_z += (v_x * sin_t + v_z * cos_t) * dt;
        robot->odom_theta += v_theta * dt;
    }

}

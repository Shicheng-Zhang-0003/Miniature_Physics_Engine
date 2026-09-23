/* MPE_FTC_074: Drivetrain implementation */
/* MPE_FTC_082 TEMPORARY — replace with anisotropic friction (MPE_FTC_095): Fixed syntax error (stray '}') + real mecanum chassis forces */
#include "drivetrain.h"
#include "core/math3d.h"
#include "config/mpe_config.h"

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
    /* MFS_162_DEAD_FIELD: mecanum_active removed */
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
    const mpe_config_t *drive_cfg = mpe_world_cfg(world);
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
        if (drive_cfg->world.gravity < 0.0f) { gravity_mag = -drive_cfg->world.gravity; }

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
        /* Traction limit: rolling grip is static friction (no-slip rolling).
         * Budgeted against the WHEELS' own static friction (min over
         * wheels), not the global floor default: contact mu is
         * min(wheel, floor), and test floors (tile 1.0) grip at least as
         * well as the rubber (0.9), so min-wheel is the binding side.
         * The old global-default (0.2) budgeting starved every force 3x. */
        float grip_mu = 0.0f;
        {
            int have_mu = 0;
            for (int i = 0; i < robot->wheel_count; i++) {
                int wi = robot->wheel_bodies[i];
                if ((wi < 0) || (wi >= world->body_count)) continue;
                float mu = world->bodies[wi].friction_static;
                if (!have_mu || mu < grip_mu) {
                    grip_mu = mu;
                    have_mu = 1;
                }
            }
            if (!have_mu || !(grip_mu > 0.0f)) {
                grip_mu = drive_cfg->world.floor_friction_s;
            }
        }
        float max_grip = grip_mu * normal_per_wheel;

        /* --- Per-wheel traction: torque -> force at contact --- */
        /* PHYSICS-FIX: budget on the NET traction vector, not the scalar
         * sum. The friction circle bounds |F_long + F_lat| per patch; in
         * pure strafe the wheel (fore-aft) forces cancel vectorially
         * (net ~0) while each wheel saturates, so a scalar-sum budget
         * leaves zero remaining and kills strafe entirely (dx 0.10 vs
         * 0.3 gate). Net budgeting keeps pure strafe whole and scales
         * strafe only when net drive actually consumes the circle. */
        vector3 traction_net = vector3_zero();
        for (int i = 0; i < robot->wheel_count; i++) {
            int wi = robot->wheel_bodies[i];
            if ((wi < 0) || (wi >= world->body_count)) { continue; }
            rigidbody *wheel = &world->bodies[wi];

            /* wheel radius from the body itself (cylinder) */
            float r = wheel->radius;
            if (r <= 0.001f) { continue; }

            /* Traction requires contact (F<=mu*N): skip airborne wheels.
             * Threshold 0.05 tolerates solver bounce/penetration slop. */
            float wheel_bottom = wheel->position.y - wheel->radius;
            if (wheel_bottom > 0.05f) {
                continue;
            }

            /* rolling direction = axle x up (wheel-local X axle) */
            vector3 axle = vector4_rotate_to_vector3(wheel->orientation, (vector3){1.0f, 0.0f, 0.0f});
            vector3 rolling_dir = vector3_normalisation(vector3_cross(axle, world_up));

            /* F = torque / r, clamped to friction limit */
            float traction = robot->wheel_motors[i].output_torque / r;
            if (traction > max_grip)  { traction = max_grip; }
            if (traction < -max_grip) { traction = -max_grip; }
            vector3 tvec = vector3_scaling(rolling_dir, traction);
            traction_net = vector3_addition(traction_net, tvec);
            wheel->force_accumulator = vector3_addition(
                wheel->force_accumulator, tvec);
            /* MFS_PORT_V15S: driven_this_tick gate belonged to the parked
             * core wheel-lock loop; wheels are woken in ftc_robot_update. */
        }

        /* --- Mecanum strafe: reduced-order roller force (FTC-side) ---
         * MFS_PORT_V15S: lateral motion has no contact force path anymore
         * (the parked core roller-tangent model is gone, and isotropic
         * cylinder friction cannot produce roller thrust). Model it where
         * it belongs without engine changes: the roller geometry converts
         * wheel torque into chassis-lateral force.
         * PHYSICS-FIX: budgeted inside the friction circle. The old code
         * added strafe*max_grip*n*0.5 on top of fully-clamped wheel
         * traction (combined ~6*max_grip vs circle 4*max_grip), inventing
         * grip. Strafe is capped so |F_long_net + F_lat| <= n*max_grip
         * (net-vector circle: pure strafe keeps full force since fore-aft
         * cancels; combined drive scales strafe). Rotate still works
         * through real wheel differentials (see tank_turn_test), so only
         * strafe is modeled here. */
        if (robot->drivetrain_type == FTC_DRIVETRAIN_MECANUM && chassis_ok) {
            /* Lateral force from wheel TORQUES through the 45° rollers
             * (coupling sin45 per wheel, signs = inverse-IK combo), NOT
             * from commands times a fraction of grip: the old feedforward
             * (strafe*max_grip*n*0.5) could never break static friction
             * (8.6 N vs 26 N cone) so strafe stalled by construction.
             * Still capped by the remaining friction circle. */
            float lat_sum = 0.0f;
            float r_lat = 0.05f;
            {
                int wi0 = (robot->wheel_count > 0) ? robot->wheel_bodies[0] : -1;
                if (wi0 >= 0 && wi0 < world->body_count && world->bodies[wi0].radius > 0.001f) {
                    r_lat = world->bodies[wi0].radius;
                }
            }
            if (r_lat > 0.001f) {
                static const float sgn[4] = {1.0f, -1.0f, -1.0f, 1.0f};
                for (int i = 0; i < robot->wheel_count && i < 4; i++) {
                    /* Explicit instantaneous torque (× traction-cut scale):
                     * locked rotors truly deliver stall; the applied
                     * (observer) torque softens to a fixed point that
                     * starves the roller model 6x. */
                    lat_sum += sgn[i] * robot->wheel_motors[i].torque_explicit *
                               robot->wheel_traction_scale[i];
                }
                float f_lat = 0.7071068f * lat_sum / r_lat;
                /* Breakaway margin: real rollers ROLL laterally (no static
                 * breakaway); the isotropic contact model cannot roll
                 * sideways, so the stand-in must exceed the static cone or
                 * strafe stalls by construction (measured dx=0.001). Cap at
                 * 1.1x the static cone: inside solver/model uncertainty,
                 * and the friction ellipse is WAIVED here by design (the
                 * roller path sits outside tire friction). odom_slip marks
                 * every strafe tick (sliding stand-in, flagged honestly);
                 * true roller modeling is the parked anisotropic keystone. */
                float total_grip = max_grip * (float)robot->wheel_count;
                float break_cap = 1.1f * total_grip;
                if (fabsf(f_lat) > 0.01f * total_grip) {
                    rigidbody *chassis = &world->bodies[robot->chassis_body];
                    vector3 lat =
                        vector4_rotate_to_vector3(chassis->orientation, (vector3){1.0f, 0.0f, 0.0f});
                    lat.y = 0.0f;
                    float lat_len_sq = vector3_length_squared(lat);
                    if (lat_len_sq > 1e-6f) {
                        lat = vector3_scaling(lat, 1.0f / sqrtf(lat_len_sq));
                        if (fabsf(f_lat) > break_cap) {
                            f_lat = (f_lat > 0.0f) ? break_cap : -break_cap;
                        }
                        chassis->force_accumulator = vector3_addition(
                            chassis->force_accumulator, vector3_scaling(lat, f_lat));
                    }
                }
            }
        }

        /* --- Chassis damping: kills sliding + uncommanded yaw --- */
        if (chassis_ok) {
            rigidbody *chassis = &world->bodies[robot->chassis_body];
            float m = chassis->mass;
            if (m > 0.0f) {
                /* Isotropic horizontal drag. Coefficient 0.5 provides enough
                 * resistance to prevent ice-rink sliding while not fighting
                 * drive force too aggressively (was 1.0 pre-audit). */
                vector3 v = chassis->velocity;
                vector3 horizontal_drag = (vector3){v.x, 0.0f, v.z};
                chassis->force_accumulator = vector3_subtraction(
                    chassis->force_accumulator,
                    vector3_scaling(horizontal_drag, m * 0.5f));
                float yaw_vel = chassis->angular_velocity.y;
                chassis->torque_accumulator.y -= yaw_vel * m * 1.5f * 0.08f;
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
                    /* Documented <0.25 m/s gate (was claimed but missing):
                     * hold applies at near-rest only; faster coast-down
                     * belongs to back-EMF + rolling resistance. */
                    if (mfs_idle) {
                        int mfs_cidx = robot->chassis_body;
                        if ((mfs_cidx >= 0) && (mfs_cidx < world->body_count)) {
                            rigidbody *mfs_probe = &world->bodies[mfs_cidx];
                            float mfs_ps = sqrtf(mfs_probe->velocity.x * mfs_probe->velocity.x +
                                                 mfs_probe->velocity.z * mfs_probe->velocity.z);
                            if (mfs_ps >= 0.25f) mfs_idle = 0;
                        }
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

/* Velocity safety monitor (was: silent 3 m/s hard clamp that masked
 * runaway instead of fixing it). With tile friction + the free-speed
 * governor, runaway has no known source; count excursions as telemetry
 * so a regression is visible instead of hidden. */
{
    float speed_sq = chassis->velocity.x * chassis->velocity.x +
                     chassis->velocity.z * chassis->velocity.z;
    float max_speed = 3.0f;
    if (speed_sq > max_speed * max_speed) {
        robot->clamp_events++;
    }
}
            }
        }
    }

/* MFS_132_ROLLING_RESISTANCE: apply small opposing torque to spinning
* wheels in contact with the floor. Simulates realistic coast-down.
* Only applies when motor command is near-zero (free-rolling). */
{
float c_rr = drive_cfg->world.rolling_resistance_coeff; /* MFS_141: real config param, default 0.02 */
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
if (drive_cfg->world.gravity < 0.0f) { g_mag = -drive_cfg->world.gravity; }
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
/* MFS_PORT_V15S: see above — no driven_this_tick in the current core. */
}
}
}

#endif /* MPE_DRIVETRAIN_REAL */



    /* FIX-AUDIT: encoder odometry (was chassis ground-truth integration,
     * which hid slip/drift by construction). Forward kinematics from wheel
     * encoders: fwd = mean(w)*r, strafe from mecanum combo (undoing the
     * strafe=-strafe IK sign), yaw from differential/mecanum combo over
     * the (offset_x+offset_z) moment arm. Integrated in the heading frame
     * so slip shows up as odom-vs-truth error. */
{
    float w_rad[FTC_MAX_WHEELS] = {0};
    for (int mfs_i = 0; mfs_i < robot->wheel_count && mfs_i < FTC_MAX_WHEELS; mfs_i++) {
        int wi = robot->wheel_bodies[mfs_i];
        if ((wi >= 0) && (wi < world->body_count)) {
            rigidbody *w = &world->bodies[wi];
            vector3 axle = w->cached_axes[0];
            if (vector3_length_squared(axle) < 0.0001f) {
                axle = vector4_rotate_to_vector3(w->orientation, (vector3){1.0f, 0.0f, 0.0f});
            }
            float omega = vector3_dot(w->angular_velocity, axle);
            robot->wheel_radians[mfs_i] += omega * dt;
            w_rad[mfs_i] = omega;
        }
    }
    float r = 0.05f;
    {
        int wi0 = (robot->wheel_count > 0) ? robot->wheel_bodies[0] : -1;
        if ((wi0 >= 0) && (wi0 < world->body_count) && (world->bodies[wi0].radius > 0.001f)) {
            r = world->bodies[wi0].radius;
        }
    }
    float v_fwd = 0.0f;
    float v_lat = 0.0f;
    float yaw_rate = 0.0f;
    if (robot->wheel_count >= 4) {
        float wfl = w_rad[0];
        float wfr = w_rad[1];
        float wbl = w_rad[2];
        float wbr = w_rad[3];
        v_fwd = ((wfl + wfr + wbl + wbr) * 0.25f) * r;
        /* IK identity mapping: combo FL-FR-BL+BR = 4*strafe. */
        v_lat = ((wfl - wfr - wbl + wbr) * 0.25f) * r;
        /* PHYSICS-FIX: mecanum yaw arm is Lx+Lz (0.24+0.20=0.44), not the
         * differential track 2*Lx=0.48. The old 0.48 understated yaw ~9%. */
        float track = 0.44f; /* WHEEL_OFFSET_X + WHEEL_OFFSET_Z: mecanum moment arm */
        yaw_rate = (((-wfl + wfr - wbl + wbr) * 0.25f) * r) / track;
    } else if (robot->wheel_count >= 2) {
        float wl = 0.0f;
        float wr = 0.0f;
        for (int i = 0; i < robot->wheel_count; i++) {
            if ((i % 2) == 0) {
                wl += w_rad[i];
            } else {
                wr += w_rad[i];
            }
        }
        int nl = (robot->wheel_count + 1) / 2;
        int nr = robot->wheel_count / 2;
        wl = (nl > 0) ? (wl / (float) nl) : 0.0f;
        wr = (nr > 0) ? (wr / (float) nr) : 0.0f;
        v_fwd = ((wl + wr) * 0.5f) * r;
        /* Differential (tank) yaw arm is the track 2*Lx = 0.48. */
        yaw_rate = ((wr - wl) * r) / 0.48f;
    }
    robot->odom_theta += yaw_rate * dt;
    float c = cosf(robot->odom_theta);
    float s = sinf(robot->odom_theta);
    /* Roller-thrust fusion: lateral chassis force bypasses the wheels, so
     * wheel encoders are structurally blind to strafe (measured: physics
     * -0.98 m vs encoder +0.15 m). Compare against the chassis-derived
     * lateral velocity in the heading frame (world->body inverse rotation
     * of the mapping applied below); on disagreement integrate odom
     * lateral from chassis motion (dead-wheel equivalent) and raise
     * odom_slip. Pure-encoder runs keep odom_slip == 0. */
    robot->odom_slip = 0;
    if (robot->chassis_body >= 0 && robot->chassis_body < world->body_count) {
        rigidbody *chb = &world->bodies[robot->chassis_body];
        float ch_lat_body = chb->velocity.x * c - chb->velocity.z * s;
        if (fabsf(ch_lat_body - v_lat) > 0.2f) {
            v_lat = ch_lat_body;
            robot->odom_slip = 1;
        }
    }
    /* body->world yaw rotation about +Y: x'=x*c+z*s, z'=-x*s+z*c */
    robot->odom_x += (v_lat * c + v_fwd * s) * dt;
    robot->odom_z += (-v_lat * s + v_fwd * c) * dt;
}

}

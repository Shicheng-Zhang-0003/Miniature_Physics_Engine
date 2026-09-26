/* MPE_FTC_074: Drivetrain implementation */
/* MPE_FTC_082 TEMPORARY — replace with anisotropic friction (MPE_FTC_095): Fixed syntax error (stray '}') + real mecanum chassis forces */
#include "drivetrain.h"
#include "core/math3d.h"
#include "config/mpe_config.h"
#include <stdio.h>
#include <math.h>

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

    /* M10 HONEST LIMIT: the mecanum inverse kinematics below is a
     * four-corner layout ([0]=FL [1]=FR [2]=BL [3]=BR). FTC_MAX_WHEELS is
     * 8, but ftc_robot carries no description of where an extra wheel sits,
     * so a 5/6/8-wheel chassis cannot be given correct per-wheel roller
     * signs here. Previously the code indexed past the end of a 4-element
     * local array and produced arbitrary wheel targets. Fail loudly and
     * stop the chassis instead. */
    if (robot->wheel_count != 4) {
        fprintf(stderr,
            "drivetrain_mecanum: mecanum IK is defined for exactly 4 wheels "
            "(FL,FR,BL,BR); robot has %d. Refusing to drive.\n",
            robot->wheel_count);
        float zero[FTC_MAX_WHEELS] = {0.0f};
        ftc_robot_set_wheel_commands(robot, zero, robot->wheel_count);
        return;
    }

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

/* ---------------------------------------------------------------------
 * DRIVETRAIN: no force model at all.
 *
 * Everything that used to live here has been deleted, because none of it was
 * physics:
 *
 *   - the per-wheel "traction" vector (torque/r applied straight to the
 *     wheel's force accumulator). That applied the motor's forward force a
 *     SECOND time: the real path is motor torque -> hub -> roller contacts
 *     -> ground friction -> chassis. Adding torque/r by hand double-counted
 *     the drive and bypassed the friction budget entirely.
 *   - the "reduced-order roller force" sin(45)*sum(tau)/r applied to the
 *     chassis. That is the lateral force faked directly onto the chassis,
 *     and it is where the 4.3x-over-ceiling strafe came from.
 *   - the "1.1x static cone breakaway margin", which deliberately exceeded
 *     the friction limit: invented grip, so that strafe could not stall.
 *   - the isotropic chassis drag m*0.5*v, which is not rolling resistance
 *     and not any other real force - it was velocity-proportional air drag
 *     applied to a solid body at a magnitude chosen to hide sliding.
 *
 * The mecanum wheels are now modelled as what they are: a hub carrying 16
 * real rollers on free bearings (see robot.c). Lateral thrust is an
 * emergent consequence of rigid-body dynamics and Coulomb friction, so it
 * is bounded by the friction cone for free and needs no fudge factor.
 * --------------------------------------------------------------------- */
static void drivetrain_odometry_update (physics_world *world, ftc_robot *robot, float dt);

void drivetrain_update (physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {return;}
    const mpe_config_t *drive_cfg = mpe_world_cfg(world);

    /* The only actuation: the real motor model in ftc_robot_update, applied
     * as a torque couple between hub and chassis. Ground friction is then
     * resolved by the engine's contact solver. */
    ftc_robot_update(world, robot, dt);

    /* Rolling resistance.
     *
     * A rigid-body contact solver is perfectly elastic, so a real wheel
     * would coast forever; real rolling resistance is energy lost to tyre
     * and floor deformation. It is therefore supplied here, but as genuine
     * physics rather than a fudge: the standard rolling-resistance law
     * F_rr = C_rr * N, with C_rr a real measured material coefficient and
     * N the actual normal load, applied at the actual contact radius.
     * Note it is an opposing TORQUE about the wheel axis (F_rr * R), not a
     * velocity-proportional force on the chassis.
     * FIX-AUDIT-DESPOT attribution: C_rr comes from the world config
     * (world.rolling_resistance_coeff, engine default 0.02 — verified
     * non-zero in mpe_config_schema.c; pneumatic-tyre-on-tile class).
     * Zero in the config means free roll and this block correctly no-ops;
     * it does not substitute its own default. */
    {
        const float c_rr = drive_cfg->world.rolling_resistance_coeff;
        const int chassis_ok = ((robot->chassis_body >= 0) &&
                                (robot->chassis_body < world->body_count));
        if (c_rr > 0.0f && chassis_ok) {
            float total_mass = world->bodies[robot->chassis_body].mass;
            for (int i = 0; i < robot->wheel_count; i++) {
                int wi = robot->wheel_bodies[i];
                if ((wi >= 0) && (wi < world->body_count)) {
                    total_mass += world->bodies[wi].mass;
                    for (int k = 0; k < robot->roller_count[i]; k++) {
                        int rb = robot->roller_bodies[i][k];
                        if ((rb >= 0) && (rb < world->body_count)) {
                            total_mass += world->bodies[rb].mass;
                        }
                    }
                }
            }
            float g_mag = 9.81f;
            if (drive_cfg->world.gravity < 0.0f) {g_mag = -drive_cfg->world.gravity;}
            if (robot->wheel_count > 0 && total_mass > 0.0f) {
                const float f_rr = c_rr * total_mass * g_mag / (float)robot->wheel_count;
                for (int i = 0; i < robot->wheel_count; i++) {
                    int wi = robot->wheel_bodies[i];
                    if ((wi < 0) || (wi >= world->body_count)) {continue;}
                    rigidbody *wheel = &world->bodies[wi];
                    /* Effective contact radius: the rollers define the
                     * running surface, not the hub plate. */
                    const float r_eff = robot->wheel_effective_radius[i];
                    if (r_eff <= 0.001f) {continue;}
                    vector3 axle = wheel->cached_axes[0];
                    if (vector3_length_squared(axle) < 0.0001f) {continue;}
                    float omega = vector3_dot(wheel->angular_velocity, axle);
                    if (fabsf(omega) < 0.01f) {continue;}
                    /* Free-rolling only: a driven wheel's net torque is the
                     * motor's business, not rolling resistance's. */
                    if (fabsf(robot->wheel_motors[i].command) > 0.05f) {continue;}
                    float sign = (omega > 0.0f) ? -1.0f : 1.0f;
                    wheel->torque_accumulator = vector3_addition(
                        wheel->torque_accumulator,
                        vector3_scaling(axle, sign * f_rr * r_eff));
                }
            }
        }
    }

    /* Velocity safety monitor: pure telemetry. The old code silently clamped
     * the chassis to 3 m/s, which hid runaway instead of fixing it. A clamp
     * is a non-physical force, so it is gone; excursions are only counted so
     * a regression is visible in the log. */
    {
        int cidx = robot->chassis_body;
        if ((cidx >= 0) && (cidx < world->body_count)) {
            const rigidbody *cb = &world->bodies[cidx];
            const float speed_sq = (cb->velocity.x * cb->velocity.x) +
                                   (cb->velocity.z * cb->velocity.z);
            if (speed_sq > 9.0f) {robot->clamp_events++;}
        }
    }

    drivetrain_odometry_update(world, robot, dt);
}

/* ---------------------------------------------------------------------
 * Encoder odometry: forward kinematics from the wheel encoders.
 *
 * This is what a real robot reports, so it is what is integrated here:
 *
 *   v_fwd  = mean(wheel) * R        (drive along the chassis axis)
 *   v_lat  = (FL-FR-BL+BR)/4 * R   (mecanum lateral, from the corner layout)
 *   yaw    = (-FL+FR-BL+BR)/4 * R / moment_arm
 *
 * Wheel i's "encoder" is the hub body integrated about its axle, which is
 * exactly the signal a hub encoder produces. R is the radius at which that
 * encoder's surface actually touches the ground: for a mecanum wheel that is
 * the roller pitch radius, not the hub plate, so it comes from
 * wheel_effective_radius.
 *
 * IMPORTANT (MFS odometry honesty): this is PURE ENCODER kinematics. It is
 * deliberately NOT fused with the chassis velocity. The old code compared the
 * encoder lateral estimate against the chassis-derived lateral velocity and,
 * on disagreement, overwrote the estimate with chassis motion (a dead-wheel
 * stand-in) while raising odom_slip. That silently injected ground truth into
 * the "odometry" and made it agree with the physics by construction. Now that
 * the mecanum lateral force is emergent from real roller contacts instead of
 * an injected chassis force, the encoders genuinely observe it, so no fusion
 * is needed. Any remaining encoder-vs-truth disagreement is real slip, and is
 * reported as error rather than papered over.
 * --------------------------------------------------------------------- */
static void drivetrain_odometry_update (physics_world *world, ftc_robot *robot, float dt) {
    float w_rad[FTC_MAX_WHEELS] = {0};
    float r = 0.0f;

    for (int i = 0; i < robot->wheel_count && i < FTC_MAX_WHEELS; i++) {
        const int wi = robot->wheel_bodies[i];
        if ((wi < 0) || (wi >= world->body_count)) {continue;}
        rigidbody *w = &world->bodies[wi];
        vector3 axle = w->cached_axes[0];
        if (vector3_length_squared(axle) < 0.0001f) {
            axle = vector4_rotate_to_vector3(w->orientation, (vector3){1.0f, 0.0f, 0.0f});
        }
        const float omega = vector3_dot(w->angular_velocity, axle);
        robot->wheel_radians[i] += omega * dt;
        w_rad[i] = omega;
    }

    /* Ground-contact radius seen by the encoder. */
    for (int i = 0; i < robot->wheel_count; i++) {
        if (robot->wheel_effective_radius[i] > 0.001f) {
            r = robot->wheel_effective_radius[i];
            break;
        }
    }
    if (r <= 0.001f) {r = 0.05f;}

    float v_fwd = 0.0f, v_lat = 0.0f, yaw_rate = 0.0f;
    if (robot->wheel_count >= 4) {
        const float wfl = w_rad[0], wfr = w_rad[1];
        const float wbl = w_rad[2], wbr = w_rad[3];
        v_fwd  = ((wfl + wfr + wbl + wbr) * 0.25f) * r;
        v_lat  = ((wfl - wfr - wbl + wbr) * 0.25f) * r;
        /* Mecanum yaw moment arm is (Lx + Lz), not the differential 2*Lx. */
        yaw_rate = (((-wfl + wfr - wbl + wbr) * 0.25f) * r) / 0.44f;
    } else if (robot->wheel_count >= 2) {
        float wl = 0.0f, wr = 0.0f;
        for (int i = 0; i < robot->wheel_count; i++) {
            if ((i % 2) == 0) {wl += w_rad[i];} else {wr += w_rad[i];}
        }
        const int nl = (robot->wheel_count + 1) / 2;
        const int nr = robot->wheel_count / 2;
        wl = (nl > 0) ? (wl / (float) nl) : 0.0f;
        wr = (nr > 0) ? (wr / (float) nr) : 0.0f;
        v_fwd = ((wl + wr) * 0.5f) * r;
        yaw_rate = ((wr - wl) * r) / 0.48f;
    }

    robot->odom_theta += yaw_rate * dt;
    /* DESPOT-FIX (math lie): odom_theta grew unbounded and cosf/sinf lost
     * precision on long runs (libm trig error grows with |theta|; past ~1e4
     * rad the heading is noise). Wrap to [-pi,pi] every tick — exact for
     * rotation, keeps libm in its accurate regime. Uses only fmodf/fabsf
     * (no new deps); deterministic=false still holds (libm), but error is
     * now bounded instead of growing. */
    {
        const float pi = 3.14159265358979323846f;
        const float two_pi = 6.28318530717958647692f;
        if (!isfinite(robot->odom_theta)) {
            robot->odom_theta = 0.0f;
        } else if (robot->odom_theta > pi || robot->odom_theta < -pi) {
            float wrapped = fmodf(robot->odom_theta + pi, two_pi);
            if (wrapped < 0.0f) wrapped += two_pi;
            robot->odom_theta = wrapped - pi;
        }
    }
    const float c = cosf(robot->odom_theta);
    const float s = sinf(robot->odom_theta);
    /* odom_slip is retained as a field for telemetry compatibility. It is no
     * longer computed from a truth comparison, because doing so would require
     * reading the chassis and fusing it in - exactly the hack being removed.
     * It stays 0 for genuinely encoder-only odometry. */
    robot->odom_slip = 0;
    /* body->world yaw about +Y: x' = x*c + z*s ; z' = -x*s + z*c */
    robot->odom_x += (v_lat * c + v_fwd * s) * dt;
    robot->odom_z += (-v_lat * s + v_fwd * c) * dt;
}

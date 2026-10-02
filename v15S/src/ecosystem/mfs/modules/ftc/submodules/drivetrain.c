/* MPE_FTC_074: Drivetrain implementation */
/* MPE_FTC_082 TEMPORARY — replace with anisotropic friction (MPE_FTC_095): Fixed syntax error (stray '}') + real mecanum chassis forces */
#include "drivetrain.h"
#include "core/math3d.h"
#include "core/det_math.h"
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

    /* Mecanum IK: per-wheel velocity targets.
       Wheel layout: [0]=FL, [1]=FR, [2]=BL, [3]=BR
       FL: forward + strafe - rotate
       FR: forward - strafe + rotate
       BL: forward - strafe - rotate
       BR: forward + strafe + rotate

       DESPOT-2026-10-02 (audited; signs CONFIRMED CORRECT by measurement,
       not by the algebra that first appeared to indict them):
       the anti-diagonal pairing (FL=f+s-R, FR=f-s+R, BL=f-s-R, BR=f+s+R)
       was A/B tested against the diagonal pairing over a steady-state
       window (ticks 180..240, after 180 ticks of drive). Pure rotate, the
       shipped mixer gives +2.289 rad/s for r=+1 and -2.030 rad/s for r=-1:
       correct sign, correct anti-symmetry, ~131 deg/s — a healthy mecanum
       turn. The diagonal pairing (FL=f+s+R, FR=f-s-R, BL=f-s+R, BR=f+s-R)
       gives -2.030 for r=+1 and +2.289 for r=-1: the same magnitude,
       INVERTED. Both pairings produce equal-and-opposite pure torque on
       paper (each is a zero-net-force, nonzero-torque combination of the
       four rail forces), which is exactly why algebra alone cannot choose
       between them: the sign is fixed by the convention, and the
       convention is fixed by measurement.

       Recorded because the first pass got this backwards. A 3.0 s
       end-to-end average reported r=+1 producing +0.029 rad and r=-1
       producing +0.221 rad, which looks like a sign error; it is not. It
       is the spin-up transient: yaw rate builds over ~2 s, so an average
       over the first 3 s is dominated by the ramp and is not comparable
       between the two commands. The steady-state window is. Any future
       judgement about drivetrain directionality must be made on the
       steady-state window, never on a short-horizon displacement average
       — the same trap made 'rotate barely works' look true when it does
       not. Pinned by mfs_t_drive_directions in tests/mfs_suite_a.c. */

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
 * The mecanum wheels are now modelled as what they are: a hub carrying 8
 * real rollers on free revolute bearings (see robot.c). Lateral thrust
 * comes from rigid-body contacts through the engine's Coulomb solver, so
 * the CONTACT FORCE is cone-bounded by the engine for free and no chassis
 * force is ever injected here (verified: no sin45/chassis-force code
 * remains). MFS itself computes no cone budget — traction is regulated by
 * the slip-threshold loop in robot.c, not by a friction-circle model.
 * The roller BEARING spin is quasi-static (see robot.c DESPOT-2026-09-26):
 * stiff-DOF-slaved, honestly labeled, not "fully emergent".
 * --------------------------------------------------------------------- */
static void drivetrain_odometry_update (physics_world *world, ftc_robot *robot, float dt);

/* ---------------------------------------------------------------------
 * MFS-STRAFE-A: analytic mecanum roller force.
 *
 * Replaces 32 articulated roller bodies + 32 bearing joints + the
 * quasi-static spin prescription (the 5-link ground->roller->bearing->hub
 * chain that GS-128/512 cannot converge: strafe wandered 0.03-0.17 m
 * chaotically with iteration count, never reaching 0.30 m). Analytic
 * mode builds NO roller bodies (hub contacts the floor directly; 6
 * bodies / 4 joints, trivially converged) and the roller geometry
 * survives here as the per-wheel axle direction.
 *
 * Why this is physics and not the retired cheat (grep-clean: no sin45
 * torque term, no chassis force anywhere in this TU):
 *   - Applied to each WHEEL's force/torque accumulators at the contact
 *     patch (force) and about the wheel centre (r×F reaction torque,
 *     which honestly loads the motor) — never to the chassis.
 *   - Coulomb-capped: |F| <= MU*N with N the wheel's static-share normal
 *     (documented approximation: ignores dynamic load transfer) and MU
 *     the wheel-rubber kinetic coefficient. It cannot exceed the cone.
 *   - Dissipative: F always opposes the measured axle-slip velocity
 *     (smoothed Coulomb, linear viscous region below VREF so standstill
 *     holds without sign chatter). Zero slip gives zero force — unlike
 *     the retired sin45*sum(tau)/r term, which pushed at full stall with
 *     no motion and no cone and ran 4.3x over the ceiling.
   *   - Contact-gated at TRUE-CONTACT scale (patch bottom within 1 cm of
   *     the floor — the engine slop scale): airborne wheels get nothing.
   *     DESPOT-2026-09-28: was 0.05 m "same as the traction loop" — a full
   *     wheel radius of hover still drew the full Coulomb cone. The two
   *     gates are now deliberately different: FORCE needs contact (1 cm),
   *     while the traction CUT keeps its conservative 0.05 m (stays engaged
   *     near ground — safe direction).
 *   - Single tangential model: analytic-mode hubs ship zero isotropic
 *     friction (see robot.c), so engine contact supplies the normal only
 *     and this is the SOLE tangential force — never double-counted.
 *   - Frame-honest: the axle lives in the CHASSIS (mount) frame, not the
 *     spinning hub (a hub-local axle would sweep with the wheel and the
 *     contact would have no fixed rail — the same bug class the parked
 *     rail model fixed with its frame id).
 * Deterministic: det_sin/det_cos + arithmetic + sqrtf only.
 * --------------------------------------------------------------------- */
static void drivetrain_mecanum_analytic (physics_world *world, ftc_robot *robot) {
    if ((!world) || (!robot)) {return;}
    if (!robot->mecanum_analytic) {return;}
    if (robot->drivetrain_type != FTC_DRIVETRAIN_MECANUM) {return;}
    if (robot->wheel_count <= 0) {return;}
    int ci = robot->chassis_body;
    if ((ci < 0) || (ci >= world->body_count)) {return;}
    rigidbody *ch = &world->bodies[ci];
    float total_mass = ch->mass;
    for (int i = 0; i < robot->wheel_count; i++) {
        int wi = robot->wheel_bodies[i];
        if ((wi >= 0) && (wi < world->body_count)) {
            total_mass += world->bodies[wi].mass;
        }
    }
    if (!(total_mass > 0.0f) || !isfinite(total_mass)) {return;}
    /* DESPOT-2026-10-02 (mathematical lie: gravity was hardcoded, and the
     * fallback was wrong in the one case that matters).
     *
     * Normal load is N = m*g, and BOTH the analytic lateral cap
     * (f_max = MU*N) and rolling resistance are proportional to it, so
     * whatever g is used here scales the entire traction budget.
     *
     * The old code was:
     *     float g_mag = 9.81f;
     *     if (cfg->world.gravity < 0.0f) { g_mag = -cfg->world.gravity; }
     * Two defects in two lines:
     *
     *   1. A world with gravity DISABLED (world.gravity == 0, a legitimate
     *      and supported configuration) still got g_mag = 9.81, so the
     *      mecanum lateral force was capped at MU*m*9.81 and the rolling
     *      resistance at Crr*m*9.81 — inventing a normal load, and hence a
     *      friction budget, out of nothing. A robot in free fall would push
     *      against an imaginary floor.
     *   2. 9.81 is not the standard value of gravity. The standard
     *      acceleration of gravity is g_n = 9.80665 m/s^2, exact by
     *      definition (CGPM 1901; CODATA 2022), so 9.81 carries a
     *      systematic +0.0341% bias into every traction budget, and a
     *      different magic number than the engine's own -9.81 default, so
     *      the two could not be reasoned about together.
     *
     * Fix: take |g| from the world config and nothing else, with no
     * fallback constant at all. Gravity is a required config field; if it
     * is absent or non-finite the honest response is to bail (the callers
     * already do) rather than invent a value. A zero-g world now correctly
     * yields N = 0 and therefore no analytic lateral force, because a free
     * roller with no normal load genuinely cannot push. */
    const mpe_config_t *cfg = mpe_world_cfg(world);
    const float g_mag = fabsf(cfg->world.gravity);
    if (!(g_mag >= 0.0f) || !isfinite(g_mag)) {return;}
    float n_per_wheel = total_mass * g_mag / (float)robot->wheel_count;
    float f_max = MFS_MECANUM_ANALYTIC_MU * n_per_wheel;
    if (!(f_max >= 0.0f) || !isfinite(f_max)) {return;}
    for (int i = 0; i < robot->wheel_count; i++) {
        int wi = robot->wheel_bodies[i];
        if ((wi < 0) || (wi >= world->body_count)) {continue;}
        rigidbody *wheel = &world->bodies[wi];
        float r_run = 0.0f;
        if ((i >= 0) && (i < FTC_MAX_WHEELS) && (robot->wheel_effective_radius[i] > 0.001f)) {
            r_run = robot->wheel_effective_radius[i];
        }
        if (!(r_run > 0.001f) || !isfinite(r_run)) {continue;}
        /* DESPOT-2026-09-28: was 0.05 m — hover force. True-contact scale. */
        if (wheel->position.y - r_run > 0.01f) {continue;} /* airborne */
        float theta = robot->wheel_roller_angle[i];
        if (!isfinite(theta)) {continue;}
        vector3 a_local = {(float)det_sin((double)theta), 0.0f, (float)det_cos((double)theta)};
        vector3 a_world = vector4_rotate_to_vector3(ch->orientation, a_local);
        a_world.y = 0.0f; /* contact plane */
        float a_len_sq = vector3_length_squared(a_world);
        if (!(a_len_sq > 1e-12f) || !isfinite(a_len_sq)) {continue;}
        a_world = vector3_scaling(a_world, 1.0f / sqrtf(a_len_sq));
        /* Contact-patch slip velocity (bottom point; planar only — the
         * normal solver owns Y). */
        vector3 r_c = {0.0f, -r_run, 0.0f};
        vector3 v_c = vector3_addition(wheel->velocity, vector3_cross(wheel->angular_velocity, r_c));
        v_c.y = 0.0f;
        float v_a = vector3_dot(v_c, a_world);
        if (!isfinite(v_a)) {continue;}
        float u = v_a / MFS_MECANUM_ANALYTIC_VREF;
        if (u > 1.0f) {u = 1.0f;} else if (u < -1.0f) {u = -1.0f;}
        vector3 F = vector3_scaling(a_world, -f_max * u);
        wheel->force_accumulator = vector3_addition(wheel->force_accumulator, F);
        wheel->torque_accumulator = vector3_addition(wheel->torque_accumulator, vector3_cross(r_c, F));
    }
}

void drivetrain_update (physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {return;}
    const mpe_config_t *drive_cfg = mpe_world_cfg(world);

    /* The only actuation: the real motor model in ftc_robot_update, applied
     * as a torque couple between hub and chassis. Ground friction is then
     * resolved by the engine's contact solver. */
    ftc_robot_update(world, robot, dt);

    /* MFS-STRAFE-A analytic lateral (mecanum only, contact-level, cone-
     * capped — see function header). Lands in the accumulators alongside
     * motor torque, pre-integration, same tick. */
    drivetrain_mecanum_analytic(world, robot);

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
            /* DESPOT-2026-10-02: same correction as the analytic lateral
             * above — g comes from the world config with no fallback
             * constant, so a disabled-gravity world has zero rolling
             * resistance (correct: F_rr = Crr*N and N = 0) instead of
             * Crr*m*9.81 invented out of nothing. See the long note at the
             * drivetrain_mecanum_analytic() g_mag for the 9.81 vs the exact
             * g_n = 9.80665 argument. */
            const float g_mag = fabsf(drive_cfg->world.gravity);
            if (robot->wheel_count > 0 && total_mass > 0.0f && isfinite(g_mag)) {
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

    /* DESPOT-2026-09-26: encoder quantization. Real hub encoders report
     * integer counts (base_ppr * gear_ratio per output rev). Previously the
     * PPR table existed but odometry differentiated the true continuous
     * omega, so a creeping wheel (0.05 rad/s) still reported smooth motion.
     * Now: integrate true angle, quantize to counts, differentiate the
     * QUANTIZED angle for odometry. Low-speed motion staircases / sticks at
     * zero exactly like hardware. */
    float counts_per_rev = 0.0f;
    {
        int base_ppr = motor_preset_base_encoder_ppr(robot->motor_preset);
        float gear = (robot->wheel_count > 0) ? robot->wheel_motors[0].gear_ratio : 1.0f;
        if (base_ppr > 0 && gear > 0.0f && isfinite(gear)) {
            counts_per_rev = (float)base_ppr * gear;
        }
    }
    const float quant_on = (counts_per_rev > 1.0f && dt > 0.0f) ? 1.0f : 0.0f;

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
        if (quant_on > 0.5f) {
            /* True angle -> integer counts -> quantized angle.
             * DESPOT-2026-09-28: was floor() — negative creep reported -1
             * immediately while positive needed a full count (half-count
             * directional bias on reversal; hardware quadrature is
             * symmetric). Round-half-away-from-zero. */
            const float revs = robot->wheel_radians[i] * 0.15915494309189535f; /* /2pi */
            const float exact = revs * counts_per_rev;
            int new_count = (int)(exact >= 0.0f ? floorf(exact + 0.5f) : ceilf(exact - 0.5f));
            if (!isfinite((float)new_count)) new_count = robot->wheel_encoder_counts[i];
            int old_count = robot->wheel_encoder_counts[i];
            robot->wheel_encoder_counts[i] = new_count;
            float q_angle = (float)new_count * 6.283185307179586f / counts_per_rev;
            float dq = q_angle - robot->wheel_radians_quant[i];
            robot->wheel_radians_quant[i] = q_angle;
            w_rad[i] = (old_count == new_count) ? 0.0f : (dq / dt);
        } else {
            robot->wheel_radians_quant[i] = robot->wheel_radians[i];
            w_rad[i] = omega;
        }
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
     * rotation, keeps trig in its accurate regime. Uses only fmodf/fabsf
     * (no new deps).
     * TRUTH (determinism): heading frame now uses det_sin/det_cos
     * (core/det_math.h, IEEE-exact polynomials, no libm). Theta is wrapped
     * to [-pi,pi] above, so |x|<1e15 contract always holds — zero trig
     * fallback in practice. deterministic=false still declared: other libm
     * uses remain in this TU (sqrtf/fabsf/floorf/fmodf), so the module as a
     * whole is not yet provably fallback-free. */
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
    const float c = (float)det_cos((double)robot->odom_theta);
    const float s = (float)det_sin((double)robot->odom_theta);
    /* body->world yaw about +Y: x' = x*c + z*s ; z' = -x*s + z*c */
    robot->odom_x += (v_lat * c + v_fwd * s) * dt;
    robot->odom_z += (-v_lat * s + v_fwd * c) * dt;
    /* DESPOT-2026-09-26: odom_slip REPORT (no fusion — odom_* above are
     * never corrected). Compare encoder-implied planar/yaw rates against
     * the true chassis body rates. Beyond 0.25 m/s planar or 0.35 rad/s yaw
     * disagreement the encoders are slipping/peeled: flag 1, else 0. */
    {
        int slip = 0;
        int ci = robot->chassis_body;
        if (ci >= 0 && ci < world->body_count) {
            const rigidbody *ch = &world->bodies[ci];
            float tvx = ch->velocity.x, tvz = ch->velocity.z;
            /* Encoder-implied world velocity (same rotation as above). */
            float evx = v_lat * c + v_fwd * s;
            float evz = -v_lat * s + v_fwd * c;
            float ex = evx - tvx, ez = evz - tvz;
            float planar_err = sqrtf(ex * ex + ez * ez);
            float yaw_err = fabsf(yaw_rate - ch->angular_velocity.y);
            if ((isfinite(planar_err) && planar_err > 0.25f) ||
                (isfinite(yaw_err) && yaw_err > 0.35f)) {
                slip = 1;
            }
        }
        robot->odom_slip = slip;
    }
}

/* MPE_FTC_070: DC motor electrical model implementation */
#include "motor.h"
#include <math.h>
#define MOTOR_RPM_TO_RAD_S 0.104719755f /* 2*pi/60 */
void motor_from_spec (motor *m, float stall_torque_nm, float free_speed_rpm, float stall_current_a,
                      float nominal_voltage, float gear_ratio, float efficiency) {
    if (!m) {
        return;
    }
    m -> stall_current = stall_current_a;
    m -> free_speed_rad_s = free_speed_rpm * MOTOR_RPM_TO_RAD_S;
    m -> gear_ratio = (gear_ratio > 0.0f) ? gear_ratio : 1.0f;
    m -> efficiency = (efficiency > 0.0f && efficiency <= 1.0f) ? efficiency : 0.85f;
    /* Kt = motor-shaft stall torque / stall_current.
     * FIX-AUDIT: preset stall is OUTPUT-shaft (post-gearbox, includes loss)
     * but output applied eff again -> stall*eff (15-20% low). Derive the
     * ideal motor-shaft torque as stall/(gear*eff) so output == spec. */
    float motor_stall_torque = stall_torque_nm / (m -> gear_ratio * m -> efficiency);
    m -> kt = (stall_current_a > 0.0f) ? (motor_stall_torque / stall_current_a) : 0.0f;
    /* R = V_nominal / stall_current */
    m -> resistance = (stall_current_a > 0.0f) ? (nominal_voltage / stall_current_a) : 1.0f;
    /* Kv: at free speed, current ~ 0, so BackEMF ~ V_nominal */
    /* Kv = V / omega_free  (motor shaft, before gearing) */
    float motor_free_speed = m -> free_speed_rad_s * m -> gear_ratio;
    m -> kv = (motor_free_speed > 0.0f) ? (nominal_voltage / motor_free_speed) : 0.0f;
    m -> command = 0.0f;
    m -> current = 0.0f;
    m -> back_emf = 0.0f;
    m -> torque = 0.0f;
    m -> output_torque = 0.0f;
    m -> torque_explicit = 0.0f;
    m -> rpm = 0.0f;
    m -> temperature = 25.0f;
    m -> load_torque = 0.0f;
    m -> w_prev = 0.0f;
    m -> tau_exp_prev = 0.0f;
    m -> wprev_valid = 0;
}
void motor_update (motor *m, float wheel_angular_vel, float dt, float battery_voltage) {
    if ((!m) || (dt <= 0.0f)) {
        return;
    }
    /* DESPOT-2026-10-01: fail-closed on non-finite bus voltage (NULL-battery
     * NAN, corrupt pack). Old code let NAN through: current/torque/temperature
     * all latched NAN and the pack bricked for the run. Clamp to OCV floor. */
    if (!isfinite (battery_voltage)) {
        battery_voltage = 12.0f;
    }
    if (!isfinite (wheel_angular_vel)) {
        return;
    }
    float motor_shaft_vel = wheel_angular_vel * m -> gear_ratio;
    /* BackEMF opposes applied voltage */
    m -> back_emf = m -> kv * motor_shaft_vel;
    /* Applied voltage from command */
    float applied_voltage = battery_voltage * m -> command;
    /* Copper thermal derating: winding resistance rises with the modeled
     * temperature (was write-only telemetry).
     * DESPOT-2026-09-28 (math truth): the old "small at FTC currents"
     * claim was false — equilibrium is dT_eq = 10*I^2*r (thermal mass
     * 10 J/K vs 0.01/K cooling), so a 2 A cruise at r~=1.3 ohm settles
     * +52 C (+20% R). Sustained stall is contained in practice by the
     * pack fuse browning out (~1 s at 4-motor stall), not by this model,
     * so a 150 C magnet ceiling clamps the integrator below. */
    float r_eff = m -> resistance * (1.0f + 0.00393f * (m -> temperature - 25.0f));
    if (!(r_eff > 0.0f) || !isfinite (r_eff))
        r_eff = m -> resistance;
    /* Current = (V - BackEMF) / R, clamped to stall */
    float raw_current = (applied_voltage - m -> back_emf) / r_eff;
    if (raw_current > m -> stall_current) {
        raw_current = m -> stall_current;
    }
    if (raw_current < -m -> stall_current) {
        raw_current = -m -> stall_current;
    }
    m -> current = raw_current;
    /* Torque = Kt * I (motor-shaft ideal); efficiency applied once at the
     * gearbox output below. NOTE: Kt and Kv are fit independently to the
     * output-shaft stall/free-speed spec endpoints, so their ratio absorbs
     * gearbox friction + no-load current (Kt != Ke is the friction budget,
     * not a units error): motor-shaft Kt==Ke is deliberately not enforced.
     * FIX-AUDIT-DESPOT: the old comment cited a CoreHex gear=1.0 preset
     * row that no longer exists (Core Hex is 72:1 in the preset table);
     * removed as stale, kept the Kt!=Ke rationale. */
    m -> torque = m -> kt * m -> current;
    /* Output torque at wheel (after gearing, minus gearbox loss) */
    m -> output_torque = m -> torque * m -> gear_ratio * m -> efficiency; /* MFS_122: restore gearing */
    m -> torque_explicit = m -> output_torque; /* explicit path: applied == instantaneous */
    /* DESPOT-2026-10-04 (stale-reference handshake): the observer reads
     * tau_exp_prev every tick (motor_observe), but only the implicit path
     * published it — an idle spell on the explicit path froze the
     * drive-phase reference, so re-drive planned against a phantom stall
     * (measured: post-idle strafe died 10x with ±stall chatter from tick
     * 0). Whichever path runs publishes its expectation; the observer is
     * then consistent across path switches by construction. */
    m -> tau_exp_prev = m -> torque_explicit;
    /* Speed tracking (signed: reverse reads negative). */
    m -> rpm = wheel_angular_vel / MOTOR_RPM_TO_RAD_S;
    /* FIX-AUDIT-DESPOT: heating lived only in motor_update_load, so the
     * explicit path (fallback + direct callers) never warmed or derated.
     * DESPOT-2026-09-26: old comment claimed "nominal R, matching the
     * implicit path exactly" — false. Implicit heats with r_eff (hot
     * copper), explicit heated with nominal R: 0.39%/°C divergence that
     * grows under sustained stall. Both now heat with r_eff. */
    {
        float heat_generated = m -> current * m -> current * r_eff * dt;
        float cooling = (m -> temperature - 25.0f) * 0.01f * dt;
        m -> temperature += heat_generated * 0.1f - cooling;
        if (m -> temperature < 25.0f) {
            m -> temperature = 25.0f;
        }
        if (m -> temperature > 150.0f) {
            m -> temperature = 150.0f; /* magnet ceiling (see derating note) */
        }
    }
}
void motor_update_load (motor *m, float wheel_angular_vel, float dt, float battery_voltage, float axle_inertia) {
    if ((!m) || (dt <= 0.0f)) {
        return;
    }
    if (!isfinite (battery_voltage)) {
        battery_voltage = 12.0f;
    }
    if (!(axle_inertia > 0.0f) || !isfinite (axle_inertia)) {
        motor_update (m, wheel_angular_vel, dt, battery_voltage);
        return;
    }
    /* Implicit Euler on the electrical dynamics with disturbance
     * observer for external load (joints/contacts):
     *   tau = A*(V - B*w_end),  w_end = w + (tau + tau_L)*dt/I
     * => w_end = (w + (A*V + tau_L)*dt/I) / (1 + A*B*dt/I)
     * with A = Kt*gear*eff/R, B = kv*gear. tau_L is last tick's measured
     * discrepancy (I*(w_now - w_pred)/dt). Without it the solve assumes
     * no load and starves locked wheels ~10x (turn/strafe die while free
     * spin converges). Stall/free endpoints identical to explicit. */
    float applied_voltage = battery_voltage * m -> command;
    /* DESPOT-FIX (math lie): old code derived A from nominal R but computed
     * current from nominal R too, while motor_update() used copper-derated
     * r_eff — the two paths disagreed by 0.39%/C and diverged under heat.
     * Both now use the same r_eff so stall/free endpoints AND transients
     * match across paths. */
    float r_eff = m -> resistance * (1.0f + 0.00393f * (m -> temperature - 25.0f));
    if (!(r_eff > 0.0f) || !isfinite (r_eff))
        r_eff = m -> resistance;
    float A = m -> kt * m -> gear_ratio * m -> efficiency / r_eff;
    float B = m -> kv * m -> gear_ratio;
    if (!(r_eff > 0.0f) || !isfinite (A) || !isfinite (B)) {
        motor_update (m, wheel_angular_vel, dt, battery_voltage);
        return;
    }
    float tau_L = (m -> wprev_valid && isfinite (m -> load_torque)) ? m -> load_torque : 0.0f;
    float w_end =
        (wheel_angular_vel + (A * applied_voltage + tau_L) * dt / axle_inertia) / (1.0f + A * B * dt / axle_inertia);
    if (!isfinite (w_end)) {
        motor_update (m, wheel_angular_vel, dt, battery_voltage);
        return;
    }
    /* Clamp the predicted end-of-tick speed to the motor's own no-load speed
     * AT THE CURRENT BUS VOLTAGE.
     *
     * The implicit solve above linearises back-EMF, so on a lightly loaded
     * wheel it badly OVER-predicts: at 26.9:1 with I=2.5e-4 kg.m^2 it
     * predicted ~63 rad/s in a single 1/60 s tick against a 23.35 rad/s
     * free speed. Because m->back_emf is derived from w_end, that
     * over-prediction also inflated the reported back-EMF and current, and
     * callers that trusted w_end had to bolt on an external free-speed
     * governor (which throttled per-tick torque to ~0.04 N.m and became the
     * accidental speed limiter for the whole drivetrain).
     *
     * A motor cannot exceed its own no-load speed on its own power: the
     * no-load point of the V-w line is w_free(V) = V/(kv*gear), which is
     * also where back-EMF exactly cancels the applied voltage and current
     * (hence torque) goes to zero. So clamping to that bound is not a
     * fudge - it is the physically exact saturation of this model, and it
     * makes the current/torque endpoints correct instead of merely bounded.
     *
     * DESPOT-2026-09-28 (math lie, was min(spec, V-line)): the old bound
     * took min(spec_free, V/(kv*gear)) and called it "physically exact".
     * At a fresh pack (12.8 V) the V-line sits 6.7% ABOVE spec, so min()
     * pinned the implicit path to spec while the explicit path correctly
     * reached the voltage-scaled speed — the "identical endpoints" claim
     * was false off-nominal. The bound is now the V-line itself (the true
     * no-load point at this voltage); the 12 V spec value survives only
     * as the fallback when Kv is degenerate. Explicit/implicit endpoints
     * agree at ANY bus voltage now.
     *
     * The clamp is deliberately ONE-SIDED with respect to the measured speed:
     * never clamp below |w_measured|, or a wheel already turning faster than
     * free speed would see less back-EMF than it should, regenerative
     * braking would silently switch off, and the wheel would run away
     * instead of being pulled back down.
     */
    {
        float w_lim = m -> free_speed_rad_s; /* 12 V spec point: fallback only */
        if (m -> kv > 0.0f && m -> gear_ratio > 0.0f && isfinite (battery_voltage) && battery_voltage > 0.0f) {
            w_lim = battery_voltage / (m -> kv * m -> gear_ratio);
        }
        if (!isfinite (w_lim) || (w_lim < 0.0f)) {
            w_lim = m -> free_speed_rad_s;
        }
        if (fabsf (wheel_angular_vel) > w_lim) {
            w_lim = fabsf (wheel_angular_vel); /* keep full braking authority */
        }
        if (w_end > w_lim) {
            w_end = w_lim;
        }
        if (w_end < -w_lim) {
            w_end = -w_lim;
        }
    }
    m -> back_emf = m -> kv * (w_end * m -> gear_ratio);
    float raw_current = (applied_voltage - m -> back_emf) / r_eff;
    if (raw_current > m -> stall_current) {
        raw_current = m -> stall_current;
    }
    if (raw_current < -m -> stall_current) {
        raw_current = -m -> stall_current;
    }
    m -> current = raw_current;
    m -> torque = m -> kt * m -> current;
    m -> output_torque = m -> torque * m -> gear_ratio * m -> efficiency;
    /* Explicit instantaneous twin (see header): correct locked-rotor
     * force sizing at the measured speed. */
    {
        float exp_i = (applied_voltage - m -> kv * (wheel_angular_vel * m -> gear_ratio)) / r_eff;
        if (exp_i > m -> stall_current)
            exp_i = m -> stall_current;
        else if (exp_i < -m -> stall_current)
            exp_i = -m -> stall_current;
        m -> torque_explicit = m -> kt * exp_i * m -> gear_ratio * m -> efficiency;
        m -> tau_exp_prev = m -> torque_explicit;
    }
    m -> rpm = w_end / MOTOR_RPM_TO_RAD_S;
    float heat_generated = m -> current * m -> current * r_eff * dt;
    float cooling = (m -> temperature - 25.0f) * 0.01f * dt;
    m -> temperature += heat_generated * 0.1f - cooling;
    if (m -> temperature < 25.0f) {
        m -> temperature = 25.0f;
    }
    if (m -> temperature > 150.0f) {
        m -> temperature = 150.0f; /* magnet ceiling (see derating note) */
    }
} /* FIX-AUDIT-DESPOT: teleport/back-button paths move bodies discontinuously,
 * which the disturbance observer reads as an infinite load spike
 * (I*dw/dt across a warp). Reset the observer on every teleport so the
 * next tick starts from "no load information" instead of a phantom stall. */
/* DESPOT-2026-09-29: own the disturbance observer here, in the same module
 * as the gate that consumes it.
 *
 * Before this, the load estimate lived inline in ftc_robot_update() while
 * motor_update_load() gated on `m->wprev_valid` — a flag motor.c READ but
 * never SET. That worked only because exactly one caller remembered to set
 * it. Every other consumer (the plugin path, a future submodule, the
 * standalone build) silently got tau_L == 0: a dead disturbance observer,
 * no warning, and no way to detect it from the motor's own state. It cost me
 * a wrong diagnosis — the new gated stall test read 0.708 N.m (-81%) purely
 * because the harness had not performed the handshake.
 *
 * Now the estimate and its validity flag are set here, by the module that
 * reads them, and callers cannot get it wrong. */
void motor_observe (motor *m, float wheel_angular_vel, float dt, float axle_inertia) {
    if (!m) {
        return;
    }
    /* w_prev is the PREVIOUS sample; the estimate is a difference against it,
     * so the new reading is stored only after the difference is taken. */
    if (!(axle_inertia > 0.0f) || !(dt > 0.0f) || !isfinite (wheel_angular_vel)) {
        /* Degenerate input: no observation is possible. Hold the last state
         * rather than claiming a fresh sample, so the next tick can still
         * difference against a real one. */
        return;
    }
    if (!m -> wprev_valid) {
        /* First sample: record it and arm the difference. Publishing a load
         * now would be inventing a number from one point, so publish none.
         * (Setting the flag here is what lets the SECOND tick observe; the
         * pre-refactor code did this unconditionally and my first attempt
         * cleared the flag instead, which silently kept the observer dead
         * for the whole run -- 5 of 9 suite cases failed.) */
        m -> w_prev = wheel_angular_vel;
        m -> load_torque = 0.0f;
        m -> wprev_valid = 1;
        return;
    }
    float stall_out = m -> stall_current * m -> kt * m -> gear_ratio * m -> efficiency;
    if (!(stall_out > 0.5f) || !isfinite (stall_out)) {
        stall_out = 1.0f;
    }
    float tau_cap = 2.0f * stall_out;
    float tau_l = axle_inertia * (wheel_angular_vel - m -> w_prev) / dt - m -> tau_exp_prev;
    if (!isfinite (tau_l)) {
        tau_l = 0.0f;
    } else if (tau_l > tau_cap) {
        tau_l = tau_cap;
    } else if (tau_l < -tau_cap) {
        tau_l = -tau_cap;
    }
    /* Blocked-rotor gate: a motor at its torque limit whose shaft is not
     * turning is, by definition, transmitting its full stall torque. This
     * evaluates the model where its answer is known rather than inventing a
     * gain. Only when saturated AND not turning, so free-spin convergence
     * (tau_l -> 0) is untouched. */
    {
        float tau_ref = m -> tau_exp_prev;
        const float w_blk = 0.5f;
        if (isfinite (tau_ref) && fabsf (tau_ref) >= 0.95f * stall_out && isfinite (wheel_angular_vel) &&
            fabsf (wheel_angular_vel) < w_blk) {
            tau_l = (tau_ref >= 0.0f) ? -stall_out : stall_out;
        }
    }
    m -> load_torque = tau_l;
    m -> w_prev = wheel_angular_vel;
    m -> wprev_valid = 1;
}
void motor_reset_observer (motor *m) {
    if (!m) {
        return;
    }
    m -> wprev_valid = 0;
    m -> load_torque = 0.0f;
    m -> w_prev = 0.0f;
    m -> tau_exp_prev = 0.0f;
}

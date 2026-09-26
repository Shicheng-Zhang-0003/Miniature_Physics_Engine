/* MPE_FTC_070: DC motor electrical model implementation */
#include "motor.h"
#include <math.h>

#define MOTOR_RPM_TO_RAD_S 0.104719755f /* 2*pi/60 */

void motor_from_spec(motor *m, float stall_torque_nm, float free_speed_rpm, float stall_current_a,
                     float nominal_voltage, float gear_ratio, float efficiency) {
    if (!m) {
        return;
    }
    m->stall_current = stall_current_a;
    m->free_speed_rad_s = free_speed_rpm * MOTOR_RPM_TO_RAD_S;
    m->gear_ratio = (gear_ratio > 0.0f) ? gear_ratio : 1.0f;
    m->efficiency = (efficiency > 0.0f && efficiency <= 1.0f) ? efficiency : 0.85f;

    /* Kt = motor-shaft stall torque / stall_current.
     * FIX-AUDIT: preset stall is OUTPUT-shaft (post-gearbox, includes loss)
     * but output applied eff again -> stall*eff (15-20% low). Derive the
     * ideal motor-shaft torque as stall/(gear*eff) so output == spec. */
    float motor_stall_torque = stall_torque_nm / (m->gear_ratio * m->efficiency);
    m->kt = (stall_current_a > 0.0f) ? (motor_stall_torque / stall_current_a) : 0.0f;

    /* R = V_nominal / stall_current */
    m->resistance = (stall_current_a > 0.0f) ? (nominal_voltage / stall_current_a) : 1.0f;

    /* Kv: at free speed, current ~ 0, so BackEMF ~ V_nominal */
    /* Kv = V / omega_free  (motor shaft, before gearing) */
    float motor_free_speed = m->free_speed_rad_s * m->gear_ratio;
    m->kv = (motor_free_speed > 0.0f) ? (nominal_voltage / motor_free_speed) : 0.0f;

    m->command = 0.0f;
    m->current = 0.0f;
    m->back_emf = 0.0f;
    m->torque = 0.0f;
    m->output_torque = 0.0f;
    m->torque_explicit = 0.0f;
    m->rpm = 0.0f;
    m->temperature = 25.0f;
    m->load_torque = 0.0f;
    m->w_prev = 0.0f;
    m->tau_exp_prev = 0.0f;
    m->wprev_valid = 0;
}

void motor_update(motor *m, float wheel_angular_vel, float dt, float battery_voltage) {
    if ((!m) || (dt <= 0.0f)) {
        return;
    }

    float motor_shaft_vel = wheel_angular_vel * m->gear_ratio;

    /* BackEMF opposes applied voltage */
    m->back_emf = m->kv * motor_shaft_vel;

    /* Applied voltage from command */
    float applied_voltage = battery_voltage * m->command;

    /* Copper thermal derating: winding resistance rises with the modeled
     * temperature (was write-only telemetry). Small at FTC currents. */
    float r_eff = m->resistance * (1.0f + 0.00393f * (m->temperature - 25.0f));
    if (!(r_eff > 0.0f) || !isfinite(r_eff)) r_eff = m->resistance;
    /* Current = (V - BackEMF) / R, clamped to stall */
    float raw_current = (applied_voltage - m->back_emf) / r_eff;
    if (raw_current > m->stall_current) {
        raw_current = m->stall_current;
    }
    if (raw_current < -m->stall_current) {
        raw_current = -m->stall_current;
    }
    m->current = raw_current;

    /* Torque = Kt * I (motor-shaft ideal); efficiency applied once at the
     * gearbox output below. NOTE: Kt and Kv are fit independently to the
     * output-shaft stall/free-speed spec endpoints, so their ratio absorbs
     * gearbox friction + no-load current (Kt != Ke is the friction budget,
     * not a units error): motor-shaft Kt==Ke is deliberately not enforced.
     * FIX-AUDIT-DESPOT: the old comment cited a CoreHex gear=1.0 preset
     * row that no longer exists (Core Hex is 72:1 in the preset table);
     * removed as stale, kept the Kt!=Ke rationale. */
    m->torque = m->kt * m->current;

    /* Output torque at wheel (after gearing, minus gearbox loss) */
    m->output_torque = m->torque * m->gear_ratio * m->efficiency; /* MFS_122: restore gearing */
    m->torque_explicit = m->output_torque; /* explicit path: applied == instantaneous */

    /* Speed tracking (signed: reverse reads negative). */
    m->rpm = wheel_angular_vel / MOTOR_RPM_TO_RAD_S;

    /* FIX-AUDIT-DESPOT: heating lived only in motor_update_load, so the
     * explicit path (fallback + direct callers) never warmed or derated.
     * Same copper-loss update in both paths (nominal R, matching the
     * implicit path exactly); not dead code, unified. */
    {
        float heat_generated = m->current * m->current * m->resistance * dt;
        float cooling = (m->temperature - 25.0f) * 0.01f * dt;
        m->temperature += heat_generated * 0.1f - cooling;
        if (m->temperature < 25.0f) {
            m->temperature = 25.0f;
        }
    }
}

void motor_update_load(motor *m, float wheel_angular_vel, float dt, float battery_voltage,
                       float axle_inertia) {
    if ((!m) || (dt <= 0.0f)) {
        return;
    }
    if (!(axle_inertia > 0.0f) || !isfinite(axle_inertia)) {
        motor_update(m, wheel_angular_vel, dt, battery_voltage);
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
    float applied_voltage = battery_voltage * m->command;
    /* DESPOT-FIX (math lie): old code derived A from nominal R but computed
     * current from nominal R too, while motor_update() used copper-derated
     * r_eff — the two paths disagreed by 0.39%/C and diverged under heat.
     * Both now use the same r_eff so stall/free endpoints AND transients
     * match across paths. */
    float r_eff = m->resistance * (1.0f + 0.00393f * (m->temperature - 25.0f));
    if (!(r_eff > 0.0f) || !isfinite(r_eff)) r_eff = m->resistance;
    float A = m->kt * m->gear_ratio * m->efficiency / r_eff;
    float B = m->kv * m->gear_ratio;
    if (!(r_eff > 0.0f) || !isfinite(A) || !isfinite(B)) {
        motor_update(m, wheel_angular_vel, dt, battery_voltage);
        return;
    }
    float tau_L = (m->wprev_valid && isfinite(m->load_torque)) ? m->load_torque : 0.0f;
    float w_end = (wheel_angular_vel + (A * applied_voltage + tau_L) * dt / axle_inertia) /
                  (1.0f + A * B * dt / axle_inertia);
    if (!isfinite(w_end)) {
        motor_update(m, wheel_angular_vel, dt, battery_voltage);
        return;
    }
    /* Clamp the predicted end-of-tick speed to the motor's own no-load speed.
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
     * no-load point of the V-w line is w_free = V/(kv*gear), which is also
     * where back-EMF exactly cancels the applied voltage and current (hence
     * torque) goes to zero. So clamping to that bound is not a fudge - it is
     * the physically exact saturation of this model, and it makes the
     * current/torque endpoints correct instead of merely bounded.
     *
     * The clamp is deliberately ONE-SIDED with respect to the measured speed:
     * never clamp below |w_measured|, or a wheel already turning faster than
     * free speed would see less back-EMF than it should, regenerative
     * braking would silently switch off, and the wheel would run away
     * instead of being pulled back down.
     */
    {
        float w_lim = m->free_speed_rad_s;
        if (m->kv > 0.0f && m->gear_ratio > 0.0f) {
            float w_v = battery_voltage / (m->kv * m->gear_ratio);
            if (w_v < w_lim) {
                w_lim = w_v;
            }
        }
        if (!isfinite(w_lim) || (w_lim < 0.0f)) {
            w_lim = m->free_speed_rad_s;
        }
        if (fabsf(wheel_angular_vel) > w_lim) {
            w_lim = fabsf(wheel_angular_vel); /* keep full braking authority */
        }
        if (w_end > w_lim) {
            w_end = w_lim;
        }
        if (w_end < -w_lim) {
            w_end = -w_lim;
        }
    }
    m->back_emf = m->kv * (w_end * m->gear_ratio);
    float raw_current = (applied_voltage - m->back_emf) / r_eff;
    if (raw_current > m->stall_current) {
        raw_current = m->stall_current;
    }
    if (raw_current < -m->stall_current) {
        raw_current = -m->stall_current;
    }
    m->current = raw_current;
    m->torque = m->kt * m->current;
    m->output_torque = m->torque * m->gear_ratio * m->efficiency;
    /* Explicit instantaneous twin (see header): correct locked-rotor
     * force sizing at the measured speed. */
    {
        float exp_i = (applied_voltage - m->kv * (wheel_angular_vel * m->gear_ratio)) / r_eff;
        if (exp_i > m->stall_current) exp_i = m->stall_current;
        else if (exp_i < -m->stall_current) exp_i = -m->stall_current;
        m->torque_explicit = m->kt * exp_i * m->gear_ratio * m->efficiency;
        m->tau_exp_prev = m->torque_explicit;
    }
    m->rpm = w_end / MOTOR_RPM_TO_RAD_S;
    float heat_generated = m->current * m->current * r_eff * dt;
    float cooling = (m->temperature - 25.0f) * 0.01f * dt;
    m->temperature += heat_generated * 0.1f - cooling;
    if (m->temperature < 25.0f) {
        m->temperature = 25.0f;
    }
}

/* FIX-AUDIT-DESPOT: teleport/back-button paths move bodies discontinuously,
 * which the disturbance observer reads as an infinite load spike
 * (I*dw/dt across a warp). Reset the observer on every teleport so the
 * next tick starts from "no load information" instead of a phantom stall. */
void motor_reset_observer(motor *m) {
    if (!m) {
        return;
    }
    m->wprev_valid = 0;
    m->load_torque = 0.0f;
    m->w_prev = 0.0f;
    m->tau_exp_prev = 0.0f;
}

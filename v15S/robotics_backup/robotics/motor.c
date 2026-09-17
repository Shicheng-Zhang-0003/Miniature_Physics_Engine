/* MPE_FTC_070: DC motor electrical model implementation */
#include "motor.h"
#include <math.h>

#define MOTOR_RPM_TO_RAD_S 0.10472f /* 2*pi/60 */

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
    m->rpm = 0.0f;
    m->temperature = 25.0f;
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

    /* Current = (V - BackEMF) / R, clamped to stall */
    float raw_current = (applied_voltage - m->back_emf) / m->resistance;
    if (raw_current > m->stall_current) {
        raw_current = m->stall_current;
    }
    if (raw_current < -m->stall_current) {
        raw_current = -m->stall_current;
    }
    m->current = raw_current;

    /* Torque = Kt * I (motor-shaft ideal); efficiency applied once at the
     * gearbox output below. NOTE: Kt and Kv are fit independently to the
     * output-shaft stall/free-speed spec endpoints; their ratio absorbs
     * gearbox friction + no-load current, so motor-shaft Kt==Ke is not
     * enforced. See audit: CoreHex gear=1.0 mislabels output as shaft. */
    m->torque = m->kt * m->current;

    /* Output torque at wheel (after gearing, minus gearbox loss) */
    m->output_torque = m->torque * m->gear_ratio * m->efficiency; /* MFS_122: restore gearing */

    /* Speed tracking (signed: reverse reads negative). */
    m->rpm = wheel_angular_vel / MOTOR_RPM_TO_RAD_S;

    /* Simplified thermal: heat from I^2*R, cooling to ambient */
    float heat_generated = m->current * m->current * m->resistance * dt;
    float cooling = (m->temperature - 25.0f) * 0.01f * dt;
    m->temperature += heat_generated * 0.1f - cooling;
    if (m->temperature < 25.0f) {
        m->temperature = 25.0f;
    }
}

/* MPE_FTC_071: FTC motor presets
 *
 * Derived from published spec-sheet values:
 *   stall_torque (N·m), free_speed (RPM), stall_current (A)
 *   nominal_voltage = 12.8V (fresh FTC battery)
 *
 * NOTE: Verify against current-season spec sheets.
 * These are representative values for simulation tuning.
 */
#include "motor_presets.h"

typedef struct {
    motor_preset_id id;
    const char *name;
    float stall_torque;
    float free_speed_rpm;
    float stall_current;
    float gear_ratio;
    float efficiency;
} motor_preset_spec;

static const motor_preset_spec presets[MOTOR_COUNT] = {
    {MOTOR_GB_5203_19_2, "goBILDA 5203 19.2:1", 1.63f, 340.0f, 17.0f, 19.2f, 0.85f},
    {MOTOR_GB_5203_30, "goBILDA 5203 30:1", 2.55f, 220.0f, 17.0f, 30.0f, 0.85f},
    {MOTOR_GB_5203_43_7, "goBILDA 5203 43.7:1", 3.72f, 150.0f, 17.0f, 43.7f, 0.85f},
    {MOTOR_GB_5203_71, "goBILDA 5203 71:1", 6.04f, 92.0f, 17.0f, 71.0f, 0.85f},
    {MOTOR_REV_CORE_HEX, "REV Core Hex", 1.40f, 60.0f, 10.0f, 1.0f, 0.80f},
};

void motor_preset_apply(motor *m, motor_preset_id id) {
    if ((!m) || (id < 0) || (id >= MOTOR_COUNT)) {
        return;
    }
    const motor_preset_spec *spec = &presets[id];
    motor_from_spec(m, spec->stall_torque, spec->free_speed_rpm, spec->stall_current, 12.8f, spec->gear_ratio,
                    spec->efficiency);
}

const char *motor_preset_name(motor_preset_id id) {
    if ((id < 0) || (id >= MOTOR_COUNT)) {
        return "unknown";
    }
    return presets[id].name;
}

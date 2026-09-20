/* MPE_FTC_071: FTC motor presets — verified spec-sheet values.
 *
 * SOURCES (full table + URLs in robotics_backup/FTC_SPECS.md):
 * - goBILDA 5203 series pages (RS-555 base, steel planetary, 12 VDC,
 *   0.25 A no-load, 9.2 A stall). kg.cm -> N-m at 0.0980665.
 * - AndyMark NeveRest Classic 40/60 + Orbital pages (am-3104 base:
 *   6000 RPM, 0.062 N-m stall, 11.5 A). Classic stalls published
 *   (350 oz-in, 3.707 N-m); Orbital 20 stall ideal-derived.
 * - REV DUO docs: Core Hex (125 RPM, 3.2 N-m, 4.4 A, 72:1); HD Hex
 *   base (6000 RPM, 0.105 N-m, 8.5 A). Geared HD/UP outputs are
 *   ideal-derived (base x ratio) and marked as such.
 * - Pitsco TorqueNADO sheet: 8.7 A stall; 700/466/233 oz-in.
 *
 * SUPERSEDES (2026-09): the previous table mixed NeveRest-class RPMs
 * with invented torques/currents under goBILDA names (e.g. 340 RPM /
 * 1.63 N-m / 17 A matches NO published sheet; true 19.2:1 is 312 RPM /
 * 2.383 N-m / 9.2 A). Stall current 17 A matches no FTC motor on record.
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
    /* goBILDA 5203 Yellow Jacket planetary — published output specs */
    {MOTOR_GB_5203_5_2, "goBILDA 5203 5.2:1", 0.7747f, 1150.0f, 9.2f, 5.2f, 0.85f},
    {MOTOR_GB_5203_19_2, "goBILDA 5203 19.2:1", 2.3830f, 312.0f, 9.2f, 19.2f, 0.85f},
    {MOTOR_GB_5203_26_9, "goBILDA 5203 26.9:1", 3.7265f, 223.0f, 9.2f, 26.9f, 0.85f},
    {MOTOR_GB_5203_50_9, "goBILDA 5203 50.9:1", 6.7077f, 117.0f, 9.2f, 50.9f, 0.85f},
    {MOTOR_GB_5203_71_2, "goBILDA 5203 71.2:1", 9.1790f, 84.0f, 9.2f, 71.2f, 0.85f},
    {MOTOR_GB_5203_99_5, "goBILDA 5203 99.5:1", 13.0625f, 60.0f, 9.2f, 99.5f, 0.85f},
    {MOTOR_GB_5203_188, "goBILDA 5203 188:1", 24.5166f, 30.0f, 9.2f, 188.0f, 0.85f},
    /* AndyMark NeveRest — published output specs (Classic) / base-derived (Orbital) */
    {MOTOR_NR_CLASSIC_40, "NeveRest Classic 40", 2.4715f, 160.0f, 11.5f, 40.0f, 0.80f},
    {MOTOR_NR_CLASSIC_60, "NeveRest Classic 60", 3.707f, 105.0f, 11.5f, 60.0f, 0.80f},
    {MOTOR_NR_ORBITAL_20, "NeveRest Orbital 20", 1.1904f, 344.0f, 11.5f, 19.2f, 0.85f},
    /* REV Robotics — published (Core Hex, HD base) / ideal-derived (geared) */
    {MOTOR_REV_CORE_HEX, "REV Core Hex", 3.2f, 125.0f, 4.4f, 72.0f, 0.80f},
    {MOTOR_REV_HD_HEX, "REV HD Hex (bare)", 0.105f, 6000.0f, 8.5f, 1.0f, 0.85f},
    {MOTOR_REV_HD_HEX_20, "REV HD Hex 20:1", 2.10f, 300.0f, 8.5f, 20.0f, 0.80f},
    {MOTOR_REV_HD_HEX_40, "REV HD Hex 40:1", 4.20f, 150.0f, 8.5f, 40.0f, 0.80f},
    {MOTOR_REV_UP_12, "REV UltraPlanetary 12:1", 1.26f, 500.0f, 8.5f, 12.0f, 0.85f},
    {MOTOR_REV_UP_60, "REV UltraPlanetary 60:1", 6.30f, 100.0f, 8.5f, 60.0f, 0.85f},
    /* Pitsco TETRIX TorqueNADO — published output specs */
    {MOTOR_TN_20, "TorqueNADO 20:1", 1.6453f, 300.0f, 8.7f, 20.0f, 0.80f},
    {MOTOR_TN_40, "TorqueNADO 40:1", 3.2907f, 150.0f, 8.7f, 40.0f, 0.80f},
    {MOTOR_TN_60, "TorqueNADO 60:1", 4.9431f, 100.0f, 8.7f, 60.0f, 0.80f},
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

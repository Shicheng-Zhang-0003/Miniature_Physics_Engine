/* MPE_FTC_071: FTC motor presets — verified against manufacturer spec sheets.
 *
 * Every entry below is traceable to a published sheet (see FTC_SPECS.md
 * for the full table with SKUs and sources). Conventions:
 * - stall_torque: OUTPUT-SHAFT stall, N·m, at nominal 12 VDC.
 * - free_speed: OUTPUT-SHAFT no-load RPM at 12 VDC.
 * - stall_current: amps at 12 VDC stall.
 * - gear_ratio: nominal reduction (used for encoder math, not torque —
 *   the model derives Kt from the published OUTPUT stall directly).
 * - efficiency: nominal gearbox figure. NOTE: in this motor model it
 *   cancels exactly (output = stall * I/Istall), so it documents the
 *   build, not the physics. 0.85 planetary / 0.80 spur nominal.
 *
 * "ideal-derived" marks values computed as base-motor spec × ratio
 * (used only where vendors publish the base motor but not the geared
 * output — REV HD Hex / UltraPlanetary). Everything else is published.
 */
#ifndef motor_presets_h
#define motor_presets_h
#include "motor.h"

typedef enum {
    /* goBILDA 5203 Yellow Jacket planetary (RS-555 base, 9.2 A stall) */
    MOTOR_GB_5203_5_2,   /* 5203-2402-0005: 5.2:1, 1150 RPM */
    MOTOR_GB_5203_19_2,  /* 5203-2402-0019: 19.2:1, 312 RPM */
    MOTOR_GB_5203_26_9,  /* 5203-2402-0027: 26.9:1, 223 RPM (drive workhorse) */
    MOTOR_GB_5203_50_9,  /* 5203-2402-0051: 50.9:1, 117 RPM */
    MOTOR_GB_5203_71_2,  /* 5203-2402-0071: 71.2:1, 84 RPM */
    MOTOR_GB_5203_99_5,  /* 5203-2402-0100: 99.5:1, 60 RPM */
    MOTOR_GB_5203_188,   /* 5203-2402-0188: 188:1, 30 RPM */
    /* AndyMark NeveRest (am-3104 base: 6000 RPM, 0.062 N-m, 11.5 A) */
    MOTOR_NR_CLASSIC_40, /* am-2964: 40:1, 160 RPM */
    MOTOR_NR_CLASSIC_60, /* am-3103: 60:1, 105 RPM */
    MOTOR_NR_ORBITAL_20, /* 19.2:1, ~344 RPM */
    /* REV Robotics (HD Hex base REV-41-1291: 6000 RPM, 0.105 N-m, 8.5 A) */
    MOTOR_REV_CORE_HEX,  /* REV-41-1300: 72:1, 125 RPM */
    MOTOR_REV_HD_HEX,    /* REV-41-1291 bare: 1:1, 6000 RPM */
    MOTOR_REV_HD_HEX_20, /* 20:1 spur, 300 RPM (ideal-derived) */
    MOTOR_REV_HD_HEX_40, /* 40:1 spur, 150 RPM (ideal-derived) */
    MOTOR_REV_UP_12,     /* UltraPlanetary 3x4 stack: 12:1, 500 RPM (ideal-derived) */
    MOTOR_REV_UP_60,     /* UltraPlanetary 3x4x5 stack: 60:1, 100 RPM (ideal-derived) */
    /* Pitsco TETRIX TorqueNADO (spur, 8.7 A stall, 324 g) */
    MOTOR_TN_20,         /* 20:1, 300 RPM */
    MOTOR_TN_40,         /* 40:1, 150 RPM */
    MOTOR_TN_60,         /* 60:1, 100 RPM */
    MOTOR_COUNT
} motor_preset_id;

void motor_preset_apply(motor *m, motor_preset_id id);
const char *motor_preset_name(motor_preset_id id);

#endif /* motor_presets_h */

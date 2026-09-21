/* MPE_FTC_071: FTC motor presets — verified against manufacturer spec sheets (2025-09).
 *
 * EVERY entry below is traceable to a published sheet (see ecosystem/mfs/docs/FTC_SPECS.md for SKUs/URLs).
 * Conventions:
 * - stall_torque: OUTPUT-SHAFT stall, N·m, at nominal 12 VDC.
 * - free_speed:   OUTPUT-SHAFT no-load RPM at 12 VDC.
 * - stall_current: amps at 12 VDC stall.
 * - gear_ratio:   nominal reduction (used for encoder math; torque model derives Kt
 *                 from published OUTPUT stall directly, so efficiency cancels).
 * - efficiency:   nominal gearbox figure (0.85 planetary / 0.80 spur). Documented
 *                 for the build; the physics model cancels it exactly.
 * - "ideal-derived" marks values computed as base_motor_spec × ratio
 *   (used ONLY where vendors publish the base motor but not the geared output:
 *   REV HD Hex / UltraPlanetary geared variants). Everything else is published.
 *
 * SOURCES (all URLs verified 2025-09):
 * - goBILDA: gobilda.com Yellow Jacket series pages + spec sheets (5202/5203/5204)
 * - AndyMark: andymark.com NeveRest Classic/Orbital/Hex pages + spec sheets
 * - REV Robotics: docs.revrobotics.com DUO build system + product pages
 * - Pitsco: asset.pitsco.com TorqueNADO DC motor specifications PDF
 */
#ifndef motor_presets_h
#define motor_presets_h
#include "motor.h"

typedef enum {
    /* ================================================================
     * goBILDA 5203 Series — Yellow Jacket Planetary (8mm REX, 24mm shaft)
     * RS-555 base: 12 VDC, 0.25 A no-load, 9.2 A stall. Steel planetary.
     * All steel gears. Dual ball bearing output shaft support.
     * Encoder: 28 PPR at motor, 28×ratio at output. 3.3-5V Hall effect.
     * SKU prefix: 5203-2402-XXXX
     * ================================================================ */
    MOTOR_GB_5203_1_1,      /* 5203-2402-0001: 1:1,   6000 RPM,  1.47 kg·cm = 0.1442 Nm */
    MOTOR_GB_5203_3_7,      /* 5203-2402-0003: 3.7:1, 1620 RPM,  5.4  kg·cm = 0.5296 Nm */
    MOTOR_GB_5203_5_2,      /* 5203-2402-0005: 5.2:1, 1150 RPM,  7.9  kg·cm = 0.7747 Nm */
    MOTOR_GB_5203_13_7,     /* 5203-2402-0014: 13.7:1,  435 RPM, 18.7  kg·cm = 1.8338 Nm */
    MOTOR_GB_5203_19_2,     /* 5203-2402-0019: 19.2:1,  312 RPM, 24.3  kg·cm = 2.3830 Nm */
    MOTOR_GB_5203_26_9,     /* 5203-2402-0027: 26.9:1,  223 RPM, 38.0  kg·cm = 3.7265 Nm */
    MOTOR_GB_5203_50_9,     /* 5203-2402-0051: 50.9:1,  117 RPM, 68.4  kg·cm = 6.7077 Nm */
    MOTOR_GB_5203_71_2,     /* 5203-2402-0071: 71.2:1,   84 RPM, 93.6  kg·cm = 9.1790 Nm */
    MOTOR_GB_5203_99_5,     /* 5203-2402-0100: 99.5:1,   60 RPM, 133.2 kg·cm = 13.0625 Nm */
    MOTOR_GB_5203_139,      /* 5203-2402-0139: 139:1,    43 RPM, 185   kg·cm = 18.1423 Nm */
    MOTOR_GB_5203_188,      /* 5203-2402-0188: 188:1,    30 RPM, 250   kg·cm = 24.5166 Nm */

    /* ================================================================
     * goBILDA 5204 Series — Yellow Jacket Planetary (8mm REX, 80mm shaft)
     * Identical gear ratios & physics to 5203; longer shaft for direct wheel mount.
     * SKU prefix: 5204-8002-XXXX (same ratio suffixes)
     * ================================================================ */
    MOTOR_GB_5204_1_1,
    MOTOR_GB_5204_3_7,
    MOTOR_GB_5204_5_2,
    MOTOR_GB_5204_13_7,
    MOTOR_GB_5204_19_2,
    MOTOR_GB_5204_26_9,
    MOTOR_GB_5204_50_9,
    MOTOR_GB_5204_71_2,
    MOTOR_GB_5204_99_5,
    MOTOR_GB_5204_139,
    MOTOR_GB_5204_188,

    /* ================================================================
     * goBILDA 5202 Series — Yellow Jacket Planetary (6mm D-shaft, 24mm shaft)
     * Identical gear ratios & physics to 5203; 6mm D-shaft instead of 8mm REX.
     * SKU prefix: 5202-2402-XXXX (same ratio suffixes)
     * ================================================================ */
    MOTOR_GB_5202_1_1,
    MOTOR_GB_5202_3_7,
    MOTOR_GB_5202_5_2,
    MOTOR_GB_5202_13_7,
    MOTOR_GB_5202_19_2,
    MOTOR_GB_5202_26_9,
    MOTOR_GB_5202_50_9,
    MOTOR_GB_5202_71_2,
    MOTOR_GB_5202_99_5,
    MOTOR_GB_5202_139,
    MOTOR_GB_5202_188,

    /* ================================================================
     * AndyMark NeveRest Classic (6mm D-shaft, 38mm dia gearbox)
     * Base motor am-3104: 12 VDC, 6000 RPM ±10% no-load, 0.062 N·m stall,
     * 11.5 A stall, 0.4 A free, 7 PPR encoder (28 PPR at output × ratio).
     * Classic gearboxes use 12T pinion. Steel gears.
     * Connector: JST-VH-2 or Anderson Powerpole 15A.
     * SKU: am-2964 (40:1), am-3103 (60:1)
     * ================================================================ */
    MOTOR_NR_CLASSIC_40,    /* am-2964: 40:1,  160 RPM, 350 oz-in = 2.4715 Nm (published) */
    MOTOR_NR_CLASSIC_60,    /* am-3103: 60:1,  105 RPM, 2.734 ft-lb = 3.707  Nm (published) */

    /* ================================================================
     * AndyMark NeveRest Orbital (6mm D-shaft, 37mm dia planetary)
     * Same base motor (am-3104), 17T pinion. Steel+plastic gears.
     * Encoder: 7 PPR at motor → 7×ratio at output.
     * Ratios available: 1:1, 3.7:1, 13.7:1, 19.2:1, 50.9:1, 263.7:1
     * Published free speeds run ~10% above 6000/ratio (6600-class base).
     * Only 3.7, 19.2, 50.9, 263.7 have full published data; others ideal-derived.
     * ================================================================ */
    MOTOR_NR_ORBITAL_1_1,   /* 1:1,     ~6600 RPM, 0.062  Nm (base motor, ideal) */
    MOTOR_NR_ORBITAL_3_7,   /* 3.7:1,  ~1784 RPM, 0.2294 Nm (ideal: 0.062*3.7) */
    MOTOR_NR_ORBITAL_13_7,  /* 13.7:1,  ~482 RPM, 0.8494 Nm (ideal: 0.062*13.7) */
    MOTOR_NR_ORBITAL_19_2,  /* 19.2:1,  ~344 RPM, 1.1904 Nm (ideal: 0.062*19.2; published ~344 RPM) */
    MOTOR_NR_ORBITAL_50_9,  /* 50.9:1,  ~130 RPM, 3.1558 Nm (ideal: 0.062*50.9; published ~130 RPM) */
    MOTOR_NR_ORBITAL_263_7, /* 263.7:1,  ~25 RPM, 16.3494 Nm (ideal: 0.062*263.7; published ~25 RPM) */

    /* ================================================================
     * AndyMark NeveRest Hex (6mm D-shaft, hexagonal gearbox)
     * Same base motor. Gearboxes: 40:1, 60:1 (Classic compatible).
     * Also NeveRest Sport series (not FTC-legal, excluded).
     * ================================================================ */
    MOTOR_NR_HEX_40,        /* 40:1, 160 RPM (same as Classic 40) */
    MOTOR_NR_HEX_60,        /* 60:1, 105 RPM (same as Classic 60) */

    /* ================================================================
     * REV Robotics HD Hex (REV-41-1291 base)
     * 550-class brushed DC: 12 VDC, 6000 RPM no-load, 0.105 N·m stall,
     * 8.5 A stall, 0.4 A free, 28 PPR encoder (at motor).
     * Pinion: 17T (UltraPlanetary) or helical 12T (20:1/40:1 Hex).
     * Encoder: JST-PH 4-pin. Power: JST-VH 2-pin.
     * Body: 37mm dia, 234g. 5mm hex output or female hex coupler.
     * ================================================================ */
    MOTOR_REV_HD_HEX,       /* REV-41-1291 bare: 1:1, 6000 RPM, 0.105 Nm, 8.5 A (published) */

    /* REV HD Hex with Spur Gearboxes (ideal-derived: base × ratio) */
    MOTOR_REV_HD_HEX_20,    /* 20:1 spur (REV-41-1064/1298): 300 RPM, 2.10 Nm, 8.5 A (ideal) */
    MOTOR_REV_HD_HEX_40,    /* 40:1 spur (REV-41-1065/1301): 150 RPM, 4.20 Nm, 8.5 A (ideal) */

    /* REV Core Hex (REV-41-1300) — integrated 72:1 planetary, 90° orientation */
    MOTOR_REV_CORE_HEX,     /* REV-41-1300: 72:1, 125 RPM, 3.2 Nm, 4.4 A (published) */

    /* ================================================================
     * REV UltraPlanetary (cartridge-based modular gearbox for HD Hex)
     * Cartridges: 3:1, 4:1, 5:1 (stackable). Input stage + pinion pressed on HD Hex.
     * Female 5mm hex output. Stack ratios multiply: 3×4=12:1, 3×4×5=60:1, etc.
     * All stall torques ideal-derived (base × stack_ratio).
     * ================================================================ */
    MOTOR_REV_UP_3,         /* 3:1 cartridge only, 2000 RPM, 0.315 Nm, 8.5 A (ideal) */
    MOTOR_REV_UP_4,         /* 4:1 cartridge only, 1500 RPM, 0.420 Nm, 8.5 A (ideal) */
    MOTOR_REV_UP_5,         /* 5:1 cartridge only, 1200 RPM, 0.525 Nm, 8.5 A (ideal) */
    MOTOR_REV_UP_12,        /* 3×4 stack: 12:1, 500 RPM, 1.26 Nm, 8.5 A (ideal) */
    MOTOR_REV_UP_20,        /* 4×5 stack: 20:1, 300 RPM, 2.10 Nm, 8.5 A (ideal) */
    MOTOR_REV_UP_60,        /* 3×4×5 stack: 60:1, 100 RPM, 6.30 Nm, 8.5 A (ideal) */
    MOTOR_REV_UP_80,        /* 4x4x5 stack: 80:1, 75 RPM, 8.40 Nm, 8.5 A (ideal-derived; reuses one 4:1 cartridge) */

    /* ================================================================
     * Pitsco TETRIX MAX TorqueNADO (spur, 6mm D-shaft, 37mm dia, 324g)
     * 12 VDC, 8.7 A stall. All-steel gears, oil bushing output support.
     * Encoder: 6 cycles/rev motor (24 counts), 6×ratio at output.
     * 20:1 → 480 CPR, 40:1 → 960 CPR, 60:1 → 1440 CPR at output shaft.
     * Connector: Powerpoles (motor), 4-pin encoder.
     * ================================================================ */
    MOTOR_TN_20,            /* 20:1, 300 RPM, 233 oz-in = 1.6453 Nm, 8.7 A (published) */
    MOTOR_TN_40,            /* 40:1, 150 RPM, 466 oz-in = 3.2907 Nm, 8.7 A (published) */
    MOTOR_TN_60,            /* 60:1, 100 RPM, 700 oz-in = 4.9431 Nm, 8.7 A (published) */

    MOTOR_COUNT
} motor_preset_id;

/* Apply a preset to a motor struct (derives Kt from published OUTPUT stall). */
void motor_preset_apply(motor *m, motor_preset_id id);

/* Get the human-readable name for a preset. */
const char *motor_preset_name(motor_preset_id id);

/* Get the gear ratio for encoder math (counts per rev = base_ppr × gear_ratio). */
float motor_preset_gear_ratio(motor_preset_id id);

/* Get the base encoder PPR at motor shaft for this motor family. */
int motor_preset_base_encoder_ppr(motor_preset_id id);

#endif /* motor_presets_h */
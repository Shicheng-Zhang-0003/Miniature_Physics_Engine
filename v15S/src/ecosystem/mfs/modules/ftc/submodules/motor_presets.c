/* MPE_FTC_071: FTC motor presets — verified spec-sheet values.
 *
 * SOURCES (full table + URLs in ecosystem/mfs/docs/FTC_SPECS.md):
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
    int base_encoder_ppr;  /* PPR at motor shaft (for encoder math) */
} motor_preset_spec;

static const motor_preset_spec presets[MOTOR_COUNT] = {
    /* ================================================================
     * goBILDA 5203 Yellow Jacket planetary — published output specs
     * RS-555 base, steel planetary, 9.2 A stall, 28 PPR encoder at motor
     * ================================================================ */
    {MOTOR_GB_5203_1_1,      "goBILDA 5203 1:1",       0.1442f,  6000.0f, 9.2f, 1.0f,   0.85f, 28},
    {MOTOR_GB_5203_3_7,      "goBILDA 5203 3.7:1",     0.5296f,  1620.0f, 9.2f, 3.7f,   0.85f, 28},
    {MOTOR_GB_5203_5_2,      "goBILDA 5203 5.2:1",     0.7747f,  1150.0f, 9.2f, 5.2f,   0.85f, 28},
    {MOTOR_GB_5203_13_7,     "goBILDA 5203 13.7:1",    1.8338f,   435.0f, 9.2f, 13.7f,  0.85f, 28},
    {MOTOR_GB_5203_19_2,     "goBILDA 5203 19.2:1",    2.3830f,   312.0f, 9.2f, 19.2f,  0.85f, 28},
    {MOTOR_GB_5203_26_9,     "goBILDA 5203 26.9:1",    3.7265f,   223.0f, 9.2f, 26.9f,  0.85f, 28},
    {MOTOR_GB_5203_50_9,     "goBILDA 5203 50.9:1",    6.7077f,   117.0f, 9.2f, 50.9f,  0.85f, 28},
    {MOTOR_GB_5203_71_2,     "goBILDA 5203 71.2:1",    9.1790f,    84.0f, 9.2f, 71.2f,  0.85f, 28},
    {MOTOR_GB_5203_99_5,     "goBILDA 5203 99.5:1",   13.0625f,   60.0f, 9.2f, 99.5f,  0.85f, 28},
    {MOTOR_GB_5203_139,      "goBILDA 5203 139:1",    18.1423f,   43.0f, 9.2f, 139.0f, 0.85f, 28},
    {MOTOR_GB_5203_188,      "goBILDA 5203 188:1",    24.5166f,   30.0f, 9.2f, 188.0f, 0.85f, 28},

    /* ================================================================
     * goBILDA 5204 Series — identical ratios/physics, 80mm shaft
     * ================================================================ */
    {MOTOR_GB_5204_1_1,      "goBILDA 5204 1:1",       0.1442f,  6000.0f, 9.2f, 1.0f,   0.85f, 28},
    {MOTOR_GB_5204_3_7,      "goBILDA 5204 3.7:1",     0.5296f,  1620.0f, 9.2f, 3.7f,   0.85f, 28},
    {MOTOR_GB_5204_5_2,      "goBILDA 5204 5.2:1",     0.7747f,  1150.0f, 9.2f, 5.2f,   0.85f, 28},
    {MOTOR_GB_5204_13_7,     "goBILDA 5204 13.7:1",    1.8338f,   435.0f, 9.2f, 13.7f,  0.85f, 28},
    {MOTOR_GB_5204_19_2,     "goBILDA 5204 19.2:1",    2.3830f,   312.0f, 9.2f, 19.2f,  0.85f, 28},
    {MOTOR_GB_5204_26_9,     "goBILDA 5204 26.9:1",    3.7265f,   223.0f, 9.2f, 26.9f,  0.85f, 28},
    {MOTOR_GB_5204_50_9,     "goBILDA 5204 50.9:1",    6.7077f,   117.0f, 9.2f, 50.9f,  0.85f, 28},
    {MOTOR_GB_5204_71_2,     "goBILDA 5204 71.2:1",    9.1790f,    84.0f, 9.2f, 71.2f,  0.85f, 28},
    {MOTOR_GB_5204_99_5,     "goBILDA 5204 99.5:1",   13.0625f,   60.0f, 9.2f, 99.5f,  0.85f, 28},
    {MOTOR_GB_5204_139,      "goBILDA 5204 139:1",    18.1423f,   43.0f, 9.2f, 139.0f, 0.85f, 28},
    {MOTOR_GB_5204_188,      "goBILDA 5204 188:1",    24.5166f,   30.0f, 9.2f, 188.0f, 0.85f, 28},

    /* ================================================================
     * goBILDA 5202 Series — identical ratios/physics, 6mm D-shaft
     * ================================================================ */
    {MOTOR_GB_5202_1_1,      "goBILDA 5202 1:1",       0.1442f,  6000.0f, 9.2f, 1.0f,   0.85f, 28},
    {MOTOR_GB_5202_3_7,      "goBILDA 5202 3.7:1",     0.5296f,  1620.0f, 9.2f, 3.7f,   0.85f, 28},
    {MOTOR_GB_5202_5_2,      "goBILDA 5202 5.2:1",     0.7747f,  1150.0f, 9.2f, 5.2f,   0.85f, 28},
    {MOTOR_GB_5202_13_7,     "goBILDA 5202 13.7:1",    1.8338f,   435.0f, 9.2f, 13.7f,  0.85f, 28},
    {MOTOR_GB_5202_19_2,     "goBILDA 5202 19.2:1",    2.3830f,   312.0f, 9.2f, 19.2f,  0.85f, 28},
    {MOTOR_GB_5202_26_9,     "goBILDA 5202 26.9:1",    3.7265f,   223.0f, 9.2f, 26.9f,  0.85f, 28},
    {MOTOR_GB_5202_50_9,     "goBILDA 5202 50.9:1",    6.7077f,   117.0f, 9.2f, 50.9f,  0.85f, 28},
    {MOTOR_GB_5202_71_2,     "goBILDA 5202 71.2:1",    9.1790f,    84.0f, 9.2f, 71.2f,  0.85f, 28},
    {MOTOR_GB_5202_99_5,     "goBILDA 5202 99.5:1",   13.0625f,   60.0f, 9.2f, 99.5f,  0.85f, 28},
    {MOTOR_GB_5202_139,      "goBILDA 5202 139:1",    18.1423f,   43.0f, 9.2f, 139.0f, 0.85f, 28},
    {MOTOR_GB_5202_188,      "goBILDA 5202 188:1",    24.5166f,   30.0f, 9.2f, 188.0f, 0.85f, 28},

    /* ================================================================
     * AndyMark NeveRest Classic — published output specs
     * Base am-3104: 6000 RPM, 0.062 N-m stall, 11.5 A, 7 PPR encoder
     * Classic gearboxes: 12T pinion, 0.80 nominal efficiency (spur)
     * ================================================================ */
    {MOTOR_NR_CLASSIC_40,    "NeveRest Classic 40",    2.4715f,  160.0f, 11.5f, 40.0f, 0.80f, 7},
    {MOTOR_NR_CLASSIC_60,    "NeveRest Classic 60",    3.7070f,  105.0f, 11.5f, 60.0f, 0.80f, 7},

    /* ================================================================
     * AndyMark NeveRest Orbital — published + ideal-derived
     * Same base motor (am-3104, 17T pinion), 7 PPR encoder.
     * Orbital gearboxes: planetary, 0.85 nominal efficiency.
     * Published free speeds run ~10% above 6000/ratio (6600-class base).
     * PHYSICS-FIX: stalls are ideal-derived base*ratio (0.062 Nm base):
     * the old table ran ~4.3x high (e.g. 3.7:1 1.1904 vs 0.229 ideal),
     * overstating wheel force 4x and guaranteeing burnout/slip.
     * ================================================================ */
    {MOTOR_NR_ORBITAL_1_1,   "NeveRest Orbital 1:1",   0.0620f, 6600.0f, 11.5f, 1.0f,  0.85f, 7},
    {MOTOR_NR_ORBITAL_3_7,   "NeveRest Orbital 3.7:1", 0.2294f, 1784.0f, 11.5f, 3.7f,  0.85f, 7},
    {MOTOR_NR_ORBITAL_13_7,  "NeveRest Orbital 13.7:1",0.8494f,  482.0f, 11.5f, 13.7f, 0.85f, 7},
    {MOTOR_NR_ORBITAL_19_2,  "NeveRest Orbital 19.2:1",1.1904f,  344.0f, 11.5f, 19.2f, 0.85f, 7},
    {MOTOR_NR_ORBITAL_50_9,  "NeveRest Orbital 50.9:1",3.1558f, 130.0f, 11.5f, 50.9f, 0.85f, 7},
    {MOTOR_NR_ORBITAL_263_7, "NeveRest Orbital 263.7:1",16.3494f,  25.0f, 11.5f, 263.7f, 0.85f, 7},

    /* ================================================================
     * AndyMark NeveRest Hex — same as Classic ratios
     * ================================================================ */
    {MOTOR_NR_HEX_40,        "NeveRest Hex 40:1",      2.4715f,  160.0f, 11.5f, 40.0f, 0.80f, 7},
    {MOTOR_NR_HEX_60,        "NeveRest Hex 60:1",      3.7070f,  105.0f, 11.5f, 60.0f, 0.80f, 7},

    /* ================================================================
     * REV Robotics HD Hex (REV-41-1291 base)
     * 550-class: 6000 RPM, 0.105 N-m stall, 8.5 A, 28 PPR encoder.
     * Geared variants are ideal-derived (base × ratio) — vendors
     * publish base motor specs but not geared output for HD Hex.
     * ================================================================ */
    {MOTOR_REV_HD_HEX,       "REV HD Hex (bare)",      0.1050f, 6000.0f, 8.5f, 1.0f,  0.85f, 28},
    {MOTOR_REV_HD_HEX_20,    "REV HD Hex 20:1 spur",   2.1000f,  300.0f, 8.5f, 20.0f, 0.80f, 28},  /* ideal-derived */
    {MOTOR_REV_HD_HEX_40,    "REV HD Hex 40:1 spur",   4.2000f,  150.0f, 8.5f, 40.0f, 0.80f, 28},  /* ideal-derived */

    /* REV Core Hex (REV-41-1300) — integrated 72:1 planetary */
    {MOTOR_REV_CORE_HEX,     "REV Core Hex",           3.2000f,  125.0f, 4.4f, 72.0f, 0.80f, 28},  /* published */

    /* ================================================================
     * REV UltraPlanetary (cartridge stacks on HD Hex base)
     * All ideal-derived from HD Hex base (6000 RPM, 0.105 Nm, 8.5 A).
     * Cartridge ratios multiply: 3, 4, 5.
     * ================================================================ */
    {MOTOR_REV_UP_3,         "REV UltraPlanetary 3:1", 0.3150f, 2000.0f, 8.5f, 3.0f,  0.85f, 28},
    {MOTOR_REV_UP_4,         "REV UltraPlanetary 4:1", 0.4200f, 1500.0f, 8.5f, 4.0f,  0.85f, 28},
    {MOTOR_REV_UP_5,         "REV UltraPlanetary 5:1", 0.5250f, 1200.0f, 8.5f, 5.0f,  0.85f, 28},
    {MOTOR_REV_UP_12,        "REV UltraPlanetary 12:1", 1.2600f,  500.0f, 8.5f, 12.0f, 0.85f, 28},
    {MOTOR_REV_UP_20,        "REV UltraPlanetary 20:1", 2.1000f,  300.0f, 8.5f, 20.0f, 0.85f, 28},
    {MOTOR_REV_UP_60,        "REV UltraPlanetary 60:1", 6.3000f,  100.0f, 8.5f, 60.0f, 0.85f, 28},
    /* PHYSICS-FIX: MOTOR_REV_UP_80 had an enum entry but no table row, so
     * every TorqueNADO initializer slid one slot down (TN_20 got TN_40
     * data, TN_60 read zero -> dead motor + NULL name). 80:1 = 4x4x5
     * stack (reuse one 4:1 cartridge); ideal-derived like the other
     * stacks: 0.105*80 = 8.40 Nm, 6000/80 = 75 RPM. */
    {MOTOR_REV_UP_80,        "REV UltraPlanetary 80:1", 8.4000f,   75.0f, 8.5f, 80.0f, 0.85f, 28},

    /* ================================================================
     * Pitsco TETRIX MAX TorqueNADO — published output specs
     * Spur gearbox, 8.7 A stall, 6 cycles/rev motor (24 counts).
     * 20:1 → 480 CPR, 40:1 → 960 CPR, 60:1 → 1440 CPR at output.
     * ================================================================ */
    {MOTOR_TN_20,            "TorqueNADO 20:1",        1.6453f,  300.0f, 8.7f, 20.0f, 0.80f, 6},
    {MOTOR_TN_40,            "TorqueNADO 40:1",        3.2907f,  150.0f, 8.7f, 40.0f, 0.80f, 6},
    {MOTOR_TN_60,            "TorqueNADO 60:1",        4.9431f,  100.0f, 8.7f, 60.0f, 0.80f, 6},
};

void motor_preset_apply(motor *m, motor_preset_id id) {
    /* PHYSICS-GUARD: table and enum must stay in lockstep. The missing
     * UP_80 row once slid every TorqueNADO entry one slot down (dead
     * motor + NULL name). Fail the build, not the robot, on drift. */
    _Static_assert(sizeof(presets) / sizeof(presets[0]) == MOTOR_COUNT,
                   "motor preset table length must equal MOTOR_COUNT");
    if ((!m) || (id < 0) || (id >= MOTOR_COUNT)) {
        return;
    }
    const motor_preset_spec *spec = &presets[id];
    /* PHYSICS-FIX: derive at the 12.0 V spec voltage, not the 12.8 V fresh
     * pack voltage. R/Kv biased +6.7% when derived at 12.8 V; the battery
     * model supplies the fresh-pack voltage at runtime. */
    motor_from_spec(m, spec->stall_torque, spec->free_speed_rpm, spec->stall_current, 12.0f, spec->gear_ratio,
                    spec->efficiency);
}

const char *motor_preset_name(motor_preset_id id) {
    if ((id < 0) || (id >= MOTOR_COUNT)) {
        return "unknown";
    }
    return presets[id].name;
}

float motor_preset_gear_ratio(motor_preset_id id) {
    if ((id < 0) || (id >= MOTOR_COUNT)) {
        return 1.0f;
    }
    return presets[id].gear_ratio;
}

int motor_preset_base_encoder_ppr(motor_preset_id id) {
    if ((id < 0) || (id >= MOTOR_COUNT)) {
        return 0;
    }
    return presets[id].base_encoder_ppr;
}
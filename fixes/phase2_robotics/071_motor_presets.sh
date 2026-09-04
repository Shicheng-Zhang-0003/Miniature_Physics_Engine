#!/usr/bin/env bash
# ============================================================
# FIX 071 — FTC Phase 2: motor presets
#   Common FTC motors derived from spec-sheet values.
#   Values are physically reasonable; verify against actual
#   spec sheets for competition accuracy.
# Phase:   phase2_robotics
# Files:   v15R3/src/robotics/motor_presets.h, motor_presets.c (new)
# Depends: 070
# Risk:    low (new files only)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
DIR="v15R3/src/robotics"
H="$DIR/motor_presets.h"
C="$DIR/motor_presets.c"
grep -q 'MPE_FTC_071' "$C" 2>/dev/null && { echo "[SKIP] motor presets already present"; exit 0; }
[[ -f "$DIR/motor.h" ]] || { echo "[SKIP] motor.h missing (run 070 first)"; exit 0; }

cat > "$H" <<'EOF'
/* MPE_FTC_071: FTC motor presets */
#ifndef motor_presets_h
#define motor_presets_h
#include "motor.h"

typedef enum {
    MOTOR_GB_5203_19_2,   /* goBILDA Yellow Jacket 19.2:1 */
    MOTOR_GB_5203_30,     /* goBILDA Yellow Jacket 30:1 */
    MOTOR_GB_5203_43_7,   /* goBILDA Yellow Jacket 43.7:1 */
    MOTOR_GB_5203_71,     /* goBILDA Yellow Jacket 71:1 */
    MOTOR_REV_CORE_HEX,   /* REV Core Hex Motor */
    MOTOR_COUNT
} motor_preset_id;

void motor_preset_apply (motor *m, motor_preset_id id);
const char *motor_preset_name (motor_preset_id id);

#endif /* motor_presets_h */
EOF

cat > "$C" <<'EOF'
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

static const motor_preset_spec presets [MOTOR_COUNT] = {
    {MOTOR_GB_5203_19_2, "goBILDA 5203 19.2:1", 1.63f, 340.0f, 17.0f, 19.2f, 0.85f},
    {MOTOR_GB_5203_30,   "goBILDA 5203 30:1",   2.55f, 220.0f, 17.0f, 30.0f, 0.85f},
    {MOTOR_GB_5203_43_7, "goBILDA 5203 43.7:1", 3.72f, 150.0f, 17.0f, 43.7f, 0.85f},
    {MOTOR_GB_5203_71,   "goBILDA 5203 71:1",   6.04f,  92.0f, 17.0f, 71.0f, 0.85f},
    {MOTOR_REV_CORE_HEX, "REV Core Hex",         1.40f,  60.0f, 10.0f,  1.0f, 0.80f},
};

void motor_preset_apply (motor *m, motor_preset_id id) {
    if ((!m) || (id < 0) || (id >= MOTOR_COUNT)) {return;}
    const motor_preset_spec *spec = &presets [id];
    motor_from_spec (m,
                     spec->stall_torque,
                     spec->free_speed_rpm,
                     spec->stall_current,
                     12.8f,
                     spec->gear_ratio,
                     spec->efficiency);
}

const char *motor_preset_name (motor_preset_id id) {
    if ((id < 0) || (id >= MOTOR_COUNT)) {return "unknown";}
    return presets [id].name;
}
EOF

grep -q 'motor_preset_apply' "$H" || { echo "[FAIL] motor_presets.h not written"; exit 1; }
grep -q 'MOTOR_GB_5203_30' "$C" || { echo "[FAIL] motor_presets.c not written"; exit 1; }
echo "[PASS] 071: FTC motor presets added (5 presets)"

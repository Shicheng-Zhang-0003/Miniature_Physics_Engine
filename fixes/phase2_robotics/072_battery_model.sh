#!/usr/bin/env bash
# ============================================================
# FIX 072 — FTC Phase 2: battery model with voltage sag
#   V_terminal = V_nominal * charge - R_internal * I_total
# Phase:   phase2_robotics
# Files:   v15R2/src/robotics/battery.h, battery.c (new)
# Depends: none
# Risk:    low (new files only)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
DIR="v15R2/src/robotics"
H="$DIR/battery.h"
C="$DIR/battery.c"
grep -q 'MPE_FTC_072' "$C" 2>/dev/null && { echo "[SKIP] battery model already present"; exit 0; }
mkdir -p "$DIR"

cat > "$H" <<'EOF'
/* MPE_FTC_072: Battery model with voltage sag */
#ifndef battery_h
#define battery_h

typedef struct {
    float nominal_voltage;      /* V (12.8 fresh) */
    float internal_resistance;  /* ohms (~0.05 for FTC battery) */
    float capacity_ah;          /* amp-hours */
    float charge_fraction;      /* 0..1 */
} battery;

void battery_init (battery *b);
/* Returns terminal voltage under load. total_current = sum of all motor currents. */
float battery_get_voltage (const battery *b, float total_current_draw);
/* Drain battery over time based on current draw. */
void battery_drain (battery *b, float total_current_draw, float dt);

#endif /* battery_h */
EOF

cat > "$C" <<'EOF'
/* MPE_FTC_072: Battery model implementation */
#include "battery.h"

void battery_init (battery *b) {
    if (!b) {return;}
    b->nominal_voltage = 12.8f;
    b->internal_resistance = 0.05f;
    b->capacity_ah = 3.0f;
    b->charge_fraction = 1.0f;
}

float battery_get_voltage (const battery *b, float total_current_draw) {
    if (!b) {return 12.8f;}
    float open_circuit = b->nominal_voltage * b->charge_fraction;
    float sag = b->internal_resistance * total_current_draw;
    float terminal = open_circuit - sag;
    if (terminal < 0.0f) {terminal = 0.0f;}
    return terminal;
}

void battery_drain (battery *b, float total_current_draw, float dt) {
    if ((!b) || (dt <= 0.0f)) {return;}
    float amp_hours_used = (total_current_draw * dt) / 3600.0f;
    b->charge_fraction -= amp_hours_used / b->capacity_ah;
    if (b->charge_fraction < 0.0f) {b->charge_fraction = 0.0f;}
}
EOF

grep -q 'battery_get_voltage' "$H" || { echo "[FAIL] battery.h not written"; exit 1; }
grep -q 'battery_drain' "$C" || { echo "[FAIL] battery.c not written"; exit 1; }
echo "[PASS] 072: battery model with voltage sag added"

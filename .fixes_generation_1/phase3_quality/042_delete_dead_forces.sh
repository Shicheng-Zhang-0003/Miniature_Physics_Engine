#!/usr/bin/env bash
# ============================================================
# FIX 042 — ARCH-013: delete unused define_forces functions
# Phase:   phase3_quality
# Files:   v15R3/src/physics/define_forces.c, define_forces.h
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET_C="v15R3/src/physics/define_forces.c"
TARGET_H="v15R3/src/physics/define_forces.h"

if [[ ! -f "$TARGET_C" ]]; then
    echo "[SKIP] $TARGET_C not found"
    exit 0
fi

if ! grep -q 'force_applicant_universal_gravity' "$TARGET_C"; then
    echo "[SKIP] define_forces functions already removed"
    exit 0
fi

cp "$TARGET_C" "${TARGET_C}.pre_042"
cp "$TARGET_H" "${TARGET_H}.pre_042"

# Replace define_forces.h with a minimal stub
cat > "$TARGET_H" << 'EOF'
#ifndef forces_h
#define forces_h
/* MPE_TASK_042: All legacy force applicant functions removed.
 * The engine uses rb_apply_forces_perfect() for gravity and
 * the impulse solver handles friction internally.
 * This header is retained for include compatibility. */
#endif
EOF

# Replace define_forces.c with a minimal stub
cat > "$TARGET_C" << 'EOF'
#include "../mpe_engine.h"
#include "define_forces.h"
/* MPE_TASK_042: Legacy force applicants removed.
 * gravity     -> rb_apply_forces_perfect() in simulation.c
 * friction    -> impulse solver in collision_mechanics.c
 * springs     -> spring_joint.c
 * All other force models (universal gravity, rolling friction,
 * string tension, vertical anchor) were never called. */
EOF

# Postflight
if grep -q 'force_applicant_universal_gravity' "$TARGET_C"; then
    echo "[FAIL] Functions still present in define_forces.c"
    exit 1
fi

if grep -q 'force_applicant_universal_gravity' "$TARGET_H"; then
    echo "[FAIL] Declarations still present in define_forces.h"
    exit 1
fi

echo "[PASS] 042: ARCH-013 fixed — unused define_forces functions removed"

#!/usr/bin/env bash
# ============================================================
# FIX 043 — ARCH-014: delete no-op adjust_joints_after_deletion
# Phase:   phase3_quality
# Files:   v15R2/src/physics/spring_joint.c, spring_joint.h,
#          v15R2/src/scene/scene_init.c
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET_C="v15R2/src/physics/spring_joint.c"
TARGET_H="v15R2/src/physics/spring_joint.h"
CALLER="v15R2/src/scene/scene_init.c"

if [[ ! -f "$TARGET_C" ]]; then
    echo "[SKIP] $TARGET_C not found"
    exit 0
fi

if ! grep -q 'adjust_joints_after_deletion' "$TARGET_C"; then
    echo "[SKIP] adjust_joints_after_deletion already removed"
    exit 0
fi

cp "$TARGET_C" "${TARGET_C}.pre_043"
cp "$TARGET_H" "${TARGET_H}.pre_043"
cp "$CALLER" "${CALLER}.pre_043"

# Remove the function body from spring_joint.c
awk '
/^void adjust_joints_after_deletion/ { skip=1 }
/^void joint_init_pool/ { skip=0 }
!skip { print }
' "$TARGET_C" > "${TARGET_C}.tmp" && mv "${TARGET_C}.tmp" "$TARGET_C"

# Remove the declaration from spring_joint.h
sed -i '/^void adjust_joints_after_deletion (int deleted_object_index);$/d' "$TARGET_H"

# Remove the call site in scene_init.c
sed -i '/adjust_joints_after_deletion (object_index);/d' "$CALLER"

# Postflight
if grep -q 'adjust_joints_after_deletion' "$TARGET_C"; then
    echo "[FAIL] Function still in spring_joint.c"
    exit 1
fi

if grep -q 'adjust_joints_after_deletion' "$TARGET_H"; then
    echo "[FAIL] Declaration still in spring_joint.h"
    exit 1
fi

if grep -q 'adjust_joints_after_deletion' "$CALLER"; then
    echo "[FAIL] Call site still in scene_init.c"
    exit 1
fi

echo "[PASS] 043: ARCH-014 fixed — no-op adjust_joints_after_deletion removed"

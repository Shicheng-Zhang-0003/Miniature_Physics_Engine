#!/usr/bin/env bash
# ============================================================
# FIX 093d — Repair: add cylinder branch to inertia dispatch
#   rigidbody_sanitize and rigidbody_set_static dispatch inertia
#   recalc as sphere-vs-cube only. A cylinder that triggers a recalc
#   gets the CUBE formula -> wrong/zero inertia -> torque can't spin
#   it. Add a cylinder branch calling rigidbody_update_inertia_cylinder.
#
#   NOTE: previous attempt used awk \s which mawk does not support.
#   This version matches the CALL rigidbody_update_inertia_sphere(rigid_body);
#   explicitly and uses portable [ \t] character classes.
#
# Phase:   phase3_sensors (cylinder keystone)
# Files:   v15R3/src/core/rigidbody.c
# Depends: 091
# Risk:    low (additive branch)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

RB_C="v15R3/src/core/rigidbody.c"
[[ -f "$RB_C" ]] || { echo "[SKIP] $RB_C not found"; exit 0; }
grep -q 'MPE_FTC_093d' "$RB_C" && { echo "[SKIP] 093d already applied"; exit 0; }

cp "$RB_C" "${RB_C}.pre_093d"

# Transform every:
#       rigidbody_update_inertia_sphere(rigid_body);
#   } else {
# into:
#       rigidbody_update_inertia_sphere(rigid_body);
#   } else if (rigid_body->type == object_cylinder) {
#       rigidbody_update_inertia_cylinder(rigid_body);
#   } else {
awk '
/rigidbody_update_inertia_sphere[ \t]*\(rigid_body\);/ {
    print
    just_sphere = 1
    next
}
just_sphere == 1 && /\}[ \t]*else[ \t]*\{/ {
    match($0, /^[ \t]*/)
    indent = substr($0, 1, RLENGTH)
    print indent "} else if (rigid_body->type == object_cylinder) { /* MPE_FTC_093d */"
    print indent "    rigidbody_update_inertia_cylinder(rigid_body);"
    print indent "} else {"
    just_sphere = 0
    next
}
{ print }
' "$RB_C" > "${RB_C}.awk_tmp" && mv "${RB_C}.awk_tmp" "$RB_C"

# Postflight
if ! grep -q 'MPE_FTC_093d' "$RB_C"; then
    echo "[DIAG] no replacement made. Current dispatch sites:"
    grep -n 'rigidbody_update_inertia_sphere\|rigidbody_update_inertia_cube' "$RB_C"
    echo "[FAIL] cylinder branch not added"
    exit 1
fi

cd v15R3/src
if make > /tmp/build_093d.log 2>&1; then
    echo "[PASS] 093d: cylinder branch added to inertia dispatch"
else
    tail -10 /tmp/build_093d.log
    echo "[FAIL] 093d: build failed"
    exit 1
fi

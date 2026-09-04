#!/usr/bin/env bash
# ============================================================
# FIX 093 — FTC Phase 3: Cylinder-floor narrowphase (THE KEYSTONE)
#   Implements collision_static_plane_cylinder() and wires it into
#   the collision_static_plane_body() dispatcher. After this, wheels
#   physically rest on the ground instead of falling through it.
#
#   Method: treat the cylinder as its axle segment + radius. The two
#   axle endpoints act like spheres of radius r; each that penetrates
#   the plane yields a contact point. Two contacts = stable wheel.
#   Normal matches the sphere-floor convention: (0,-1,0).
#
# Phase:   phase3_sensors (cylinder keystone)
# Files:   v15R3/src/physics/collision_mechanics.c
# Depends: 091, 092a
# Risk:    medium (new narrowphase + dispatcher edit)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

CM_C="v15R3/src/physics/collision_mechanics.c"

[[ -f "$CM_C" ]] || { echo "[SKIP] $CM_C not found"; exit 0; }
grep -q 'collision_static_plane_body' "$CM_C" || { echo "[FAIL] dispatcher collision_static_plane_body not found"; exit 1; }
grep -q 'MPE_FTC_093' "$CM_C" && { echo "[SKIP] 093 already applied"; exit 0; }

cp "$CM_C" "${CM_C}.pre_093"

# ---- Part A: add cylinder branch to the dispatcher ----
awk '
/bool collision_static_plane_body[ (]/ && $0 !~ /_proxy/ && done_dispatch == 0 {
    line = $0
    p1 = ""; p2 = ""; p3 = ""
    if (match(line, /rigidbody *\* *[A-Za-z_][A-Za-z0-9_]*/))      { p1 = substr(line, RSTART, RLENGTH); sub(/^rigidbody *\* */, "", p1) }
    if (match(line, /float +[A-Za-z_][A-Za-z0-9_]*/))              { p2 = substr(line, RSTART, RLENGTH); sub(/^float +/, "", p2) }
    if (match(line, /collision_data *\* *[A-Za-z_][A-Za-z0-9_]*/)) { p3 = substr(line, RSTART, RLENGTH); sub(/^collision_data *\* */, "", p3) }
    if ((p1 != "") && (p2 != "") && (p3 != "")) {
        print "/* MPE_FTC_093: cylinder floor contact */"
        print "bool collision_static_plane_cylinder(rigidbody *cyl, float plane_y, collision_data *collision_output_data);"
        print line
        if (line ~ /\{/) {
            print "    if (" p1 "->type == object_cylinder) {return collision_static_plane_cylinder(" p1 ", " p2 ", " p3 ");}"
            done_dispatch = 1
        } else {
            pend = 1
            done_dispatch = 1
        }
    } else {
        print line
    }
    next
}
pend == 1 && $0 ~ /\{/ {
    print
    print "    if (" p1 "->type == object_cylinder) {return collision_static_plane_cylinder(" p1 ", " p2 ", " p3 ");}"
    pend = 0
    next
}
{ print }
' "$CM_C" > "${CM_C}.awk_tmp" && mv "${CM_C}.awk_tmp" "$CM_C"

# ---- Part B: append the cylinder-plane collision function ----
cat >> "$CM_C" << 'CYL_EOF'

/* MPE_FTC_093: Cylinder vs static floor plane.
 * Models the cylinder as axle segment + radius. Each axle endpoint acts
 * like a sphere of radius r; an endpoint below the plane yields a contact.
 * Two contacts (one per axle end) give a stable resting wheel.
 * Normal matches the sphere-floor convention: (0,-1,0). */
bool collision_static_plane_cylinder(rigidbody *cyl, float plane_y, collision_data *collision_output_data) {
    if (cyl->type != object_cylinder) {return false;}
    vector3 axis = cyl->cached_axes[0]; /* axle = local X in world space */
    float r = cyl->radius;
    float h = cyl->cylinder_half_length;
    vector3 axle_offset = vector3_scaling(axis, h);
    vector3 e1 = vector3_subtraction(cyl->position, axle_offset);
    vector3 e2 = vector3_addition(cyl->position, axle_offset);
    rigidbody *plane_body = collision_static_plane_body_proxy(plane_y);
    collision_output_data->object_a = cyl;
    collision_output_data->object_b = plane_body;
    collision_output_data->normal_vector = (vector3){0.0f, -1.0f, 0.0f};
    collision_output_data->contact_count = 0;
    float pen1 = plane_y - (e1.y - r);
    if ((pen1 > 0.0f) && (collision_output_data->contact_count < 2)) {
        contact_point_data *cp = &collision_output_data->contacts[collision_output_data->contact_count];
        cp->position = (vector3){e1.x, e1.y - r, e1.z};
        cp->penetration = pen1;
        collision_output_data->contact_count++;
    }
    float pen2 = plane_y - (e2.y - r);
    if ((pen2 > 0.0f) && (collision_output_data->contact_count < 2)) {
        contact_point_data *cp = &collision_output_data->contacts[collision_output_data->contact_count];
        cp->position = (vector3){e2.x, e2.y - r, e2.z};
        cp->penetration = pen2;
        collision_output_data->contact_count++;
    }
    return collision_output_data->contact_count > 0;
}
CYL_EOF

# ---- Postflight ----
grep -q 'collision_static_plane_cylinder' "$CM_C" || { echo "[FAIL] cylinder function not added"; exit 1; }
grep -q 'object_cylinder) {return collision_static_plane_cylinder' "$CM_C" || { echo "[FAIL] dispatcher not wired"; exit 1; }

cd v15R3/src
if make test_cylinder_drop > /tmp/cyl_093.log 2>&1; then
    grep -E '\[info\]|\[PASS\]' /tmp/cyl_093.log || true
    echo "[PASS] 093: cylinder now rests on the floor — wheels touch ground"
else
    tail -20 /tmp/cyl_093.log
    echo "[FAIL] 093: cylinder still not resting (see output above)"
    exit 1
fi

#!/usr/bin/env bash
# ============================================================
# FIX 091 — FTC Phase 3: Cylinder implementation
#   Implements cylinder initialisation, inertia tensor, sanitize,
#   set_static, physics_world_add_cylinder, and broadphase radius.
#
#   Inertia (axle along local X):
#     I_xx = 0.5 * m * r^2
#     I_yy = I_zz = m * (3r^2 + h^2) / 12   where h = 2*half_length
#
#   Broadphase bounding radius: sqrt(r^2 + half_length^2)
#
# Phase:   phase3_sensors (cylinder keystone)
# Files:   v15R3/src/core/rigidbody.c
#          v15R3/src/core/physics_world.c
#          v15R3/src/physics/broadphase.c
# Depends: 090
# Risk:    medium (multiple file edits, additive logic)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

RB_C="v15R3/src/core/rigidbody.c"
PW_C="v15R3/src/core/physics_world.c"
BP_C="v15R3/src/physics/broadphase.c"
RB_H="v15R3/src/core/rigidbody.h"

for f in "$RB_C" "$PW_C" "$BP_C"; do
    [[ -f "$f" ]] || { echo "[SKIP] $f not found"; exit 0; }
done

grep -q 'object_cylinder' "$RB_H" || { echo "[SKIP] 090 not applied yet"; exit 0; }
grep -q 'MPE_FTC_091' "$RB_C" && { echo "[SKIP] 091 already applied"; exit 0; }

cp "$RB_C" "${RB_C}.pre_091"
cp "$PW_C" "${PW_C}.pre_091"
cp "$BP_C" "${BP_C}.pre_091"

# ============================================================
# STEP 1: Add rigidbody_update_inertia_cylinder to rigidbody.c
#         Insert after rigidbody_update_inertia_cube function.
# ============================================================
awk '
/^} \/\/ Helper to update inertia tensor after mass\/radius change$/ && done_cube {
    print
    print ""
    print "/* MPE_FTC_091: Cylinder inertia tensor (axle along local X) */"
    print "void rigidbody_update_inertia_cylinder(rigidbody *rigid_body) {"
    print "    float m = rigid_body->mass;"
    print "    float r = rigid_body->radius;"
    print "    float h = rigid_body->cylinder_half_length * 2.0f; /* full height */"
    print "    float i_axle = 0.5f * m * r * r; /* about axle (X) */"
    print "    float i_perp = m * (3.0f * r * r + h * h) / 12.0f; /* about Y and Z */"
    print "    rigid_body->inertia_tensor_local = (math3){{{0}}};"
    print "    rigid_body->inertia_tensor_local.matrix[0][0] = i_axle;"
    print "    rigid_body->inertia_tensor_local.matrix[1][1] = i_perp;"
    print "    rigid_body->inertia_tensor_local.matrix[2][2] = i_perp;"
    print "    if (m > 0) {"
    print "        rigid_body->inverse_inertia_tensor_local = math3_inverse(rigid_body->inertia_tensor_local);"
    print "        rigid_body->inverse_inertia_system = rigid_body->inverse_inertia_tensor_local;"
    print "    } else {"
    print "        rigid_body->inverse_inertia_tensor_local = (math3){{{0}}};"
    print "        rigid_body->inverse_inertia_system = (math3){{{0}}};"
    print "    }"
    print "}"
    next
}
/^void rigidbody_update_inertia_cube/ { done_cube = 1 }
{ print }
' "$RB_C" > "${RB_C}.tmp" && mv "${RB_C}.tmp" "$RB_C"

# ============================================================
# STEP 2: Add rigidbody_initialisation_cylinder to rigidbody.c
#         Insert after rigidbody_initialisation_cube function end.
# ============================================================
awk '
/^} \/\/ Force & Torque accumulators$/ && done_init_cube {
    print
    print ""
    print "/* MPE_FTC_091: Cylinder initialisation */"
    print "void rigidbody_initialisation_cylinder(rigidbody *rigid_body, float radius, float half_length,"
    print "                                     float mass, vector3 position_input) {"
    print "    rigid_body->position = position_input;"
    print "    rigid_body->velocity = vector3_zero();"
    print "    rigid_body->acceleration = vector3_zero();"
    print "    rigid_body->orientation = vector4_identity();"
    print "    rigid_body->angular_velocity = vector3_zero();"
    print "    rigid_body->angular_acceleration = vector3_zero();"
    print "    rigid_body->colour = (vector3){0.2f, 0.8f, 0.8f}; /* cyan for cylinders */"
    print "    rigid_body->type = object_cylinder;"
    print "    rigidbody_update_axes(rigid_body);"
    print "    rigid_body->mass = mass;"
    print "    if (mass > 0) {"
    print "        rigid_body->inverse_mass = 1.0f / mass;"
    print "    } else {"
    print "        rigid_body->inverse_mass = 0.0f;"
    print "    }"
    print "    rigid_body->radius = radius;"
    print "    rigid_body->cylinder_half_length = half_length;"
    print "    rigid_body->restitution = 0.3f;"
    print "    rigid_body->static_state = (mass == 0);"
    print "    rigid_body->is_sleeping = false;"
    print "    rigid_body->sleep_timer = 0.0f;"
    print "    rigid_body->nice_value = 0;"
    print "    rigid_body->friction_static = 0.8f; /* high grip for wheels */"
    print "    rigid_body->friction_kinetic = 0.6f;"
    print "    rigidbody_update_inertia_cylinder(rigid_body);"
    print "    rigid_body->force_accumulator = vector3_zero();"
    print "    rigid_body->torque_accumulator = vector3_zero();"
    print "}"
    next
}
/^void rigidbody_initialisation_cube/ { done_init_cube = 1 }
{ print }
' "$RB_C" > "${RB_C}.tmp" && mv "${RB_C}.tmp" "$RB_C"

# ============================================================
# STEP 3: Update rigidbody_sanitize to handle cylinders.
#         The existing else branch handles cubes. We add a
#         cylinder branch before the else.
# ============================================================
awk '
/^} else \{$/ && in_type_check && !cylinder_done {
    print "} else if (rigid_body->type == object_cylinder) {"
    print "    if (!isfinite(rigid_body->radius) || (rigid_body->radius <= 0.0f)) {"
    print "        rigid_body->radius = 0.01f;"
    print "        needs_inertia_recalc = true;"
    print "    }"
    print "    if (!isfinite(rigid_body->cylinder_half_length) || (rigid_body->cylinder_half_length <= 0.0f)) {"
    print "        rigid_body->cylinder_half_length = 0.01f;"
    print "        needs_inertia_recalc = true;"
    print "    }"
    print "} else {"
    cylinder_done = 1
    next
}
/if \(rigid_body->type == object_sphere\)/ { in_type_check = 1 }
{ print }
' "$RB_C" > "${RB_C}.tmp" && mv "${RB_C}.tmp" "$RB_C"

# ============================================================
# STEP 4: Update rigidbody_set_static to handle cylinder inertia.
#         Find the sphere/cube branch and add cylinder.
# ============================================================
sed -i 's/if (rigid_body->type == object_sphere) {\n        rigidbody_update_inertia_sphere(rigid_body);\n    } else {\n        rigidbody_update_inertia_cube(rigid_body);\n    }/if (rigid_body->type == object_sphere) {\n        rigidbody_update_inertia_sphere(rigid_body);\n    } else if (rigid_body->type == object_cylinder) { \/* MPE_FTC_091 *\/\n        rigidbody_update_inertia_cylinder(rigid_body);\n    } else {\n        rigidbody_update_inertia_cube(rigid_body);\n    }/' "$RB_C"

# Fallback: if the multi-line sed didn't match (whitespace differences), use awk
if ! grep -q 'rigidbody_update_inertia_cylinder' "$RB_C"; then
    awk '
    /rigidbody_update_inertia_sphere\(rigid_body\);/ && in_set_static {
        print
        getline nextline
        if (nextline ~ /} else {/) {
            print "    } else if (rigid_body->type == object_cylinder) { /* MPE_FTC_091 */"
            print "        rigidbody_update_inertia_cylinder(rigid_body);"
            print "    } else {"
            next
        }
        print nextline
        next
    }
    /void rigidbody_set_static/ { in_set_static = 1 }
    { print }
    ' "$RB_C" > "${RB_C}.tmp" && mv "${RB_C}.tmp" "$RB_C"
fi

# ============================================================
# STEP 5: Add physics_world_add_cylinder to physics_world.c
#         Insert after physics_world_add_cube function.
# ============================================================
awk '
/^}$/ && done_add_cube && !added_cylinder {
    print
    print ""
    print "/* MPE_FTC_091 */"
    print "int physics_world_add_cylinder(physics_world *world, float radius, float half_length, float mass,"
    print "                             vector3 position) {"
    print "    if ((!world) || (!world->bodies) || (world->body_count >= world->body_capacity)) {"
    print "        return -1;"
    print "    }"
    print "    rigidbody *rb = &world->bodies[world->body_count];"
    print "    rigidbody_initialisation_cylinder(rb, radius, half_length, mass, position);"
    print "    rb->object_id = world->next_object_id++;"
    print "    rb->object_generation = 1;"
    print "    rigidbody_sanitize(rb);"
    print "    return world->body_count++;"
    print "}"
    added_cylinder = 1
    next
}
/^int physics_world_add_cube/ { done_add_cube = 1 }
{ print }
' "$PW_C" > "${PW_C}.tmp" && mv "${PW_C}.tmp" "$PW_C"

# ============================================================
# STEP 6: Update broadphase_bounding_radius to handle cylinders.
#         bounding_radius = sqrt(r^2 + half_length^2)
# ============================================================
awk '
/^}$/ && in_bounding_radius && !cylinder_added {
    print "    if (rb->type == object_cylinder) { /* MPE_FTC_091 */"
    print "        return sqrtf(rb->radius * rb->radius +"
    print "                     rb->cylinder_half_length * rb->cylinder_half_length);"
    print "    }"
    print
    cylinder_added = 1
    next
}
/^static inline float broadphase_bounding_radius/ { in_bounding_radius = 1 }
{ print }
' "$BP_C" > "${BP_C}.tmp" && mv "${BP_C}.tmp" "$BP_C"

# ============================================================
# POSTFLIGHT
# ============================================================
grep -q 'rigidbody_initialisation_cylinder' "$RB_C" || { echo "[FAIL] init not in rigidbody.c"; exit 1; }
grep -q 'rigidbody_update_inertia_cylinder' "$RB_C" || { echo "[FAIL] inertia not in rigidbody.c"; exit 1; }
grep -q 'object_cylinder' "$RB_C" || { echo "[FAIL] object_cylinder not referenced in rigidbody.c"; exit 1; }
grep -q 'physics_world_add_cylinder' "$PW_C" || { echo "[FAIL] add_cylinder not in physics_world.c"; exit 1; }
grep -q 'object_cylinder' "$BP_C" || { echo "[FAIL] cylinder not in broadphase.c"; exit 1; }

# Build check
cd v15R3/src
if make > /tmp/build_091.log 2>&1; then
    echo "  Build: PASS"
else
    echo "  Build: FAIL"
    tail -10 /tmp/build_091.log
    echo "[FAIL] 091: build failed after cylinder implementation"
    exit 1
fi

echo "[PASS] 091: cylinder implementation complete"

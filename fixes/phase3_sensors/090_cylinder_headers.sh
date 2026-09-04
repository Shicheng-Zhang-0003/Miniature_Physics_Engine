#!/usr/bin/env bash
# ============================================================
# FIX 090 — FTC Phase 3: Cylinder type headers
#   Adds object_cylinder to the type enum, cylinder_half_length
#   to the rigidbody struct, and declares init/inertia/add functions.
#   Axle convention: local X axis (matches robot.c axle_axis_x = 1.0)
#
# Phase:   phase3_sensors (cylinder keystone)
# Files:   v15R3/src/core/rigidbody.h
#          v15R3/src/core/physics_world.h
# Depends: 082 (indentation)
# Risk:    low (additive header changes)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

RB_H="v15R3/src/core/rigidbody.h"
PW_H="v15R3/src/core/physics_world.h"

for f in "$RB_H" "$PW_H"; do
    [[ -f "$f" ]] || { echo "[SKIP] $f not found"; exit 0; }
done

grep -q 'MPE_FTC_090' "$RB_H" && { echo "[SKIP] 090 already applied"; exit 0; }

cp "$RB_H" "${RB_H}.pre_090"
cp "$PW_H" "${PW_H}.pre_090"

# 1. Add object_cylinder to the object_type enum
sed -i 's/typedef enum { object_sphere, object_cube } object_type;/typedef enum { object_sphere, object_cube, object_cylinder } object_type; \/* MPE_FTC_090 *\//' "$RB_H"

# 2. Add cylinder_half_length field after the radius field
sed -i '/^    float radius;$/a\    float cylinder_half_length; \/* MPE_FTC_090: half-length along axle (X) *\//' "$RB_H"

# 3. Add cylinder function declarations after rigidbody_update_inertia_cube declaration
sed -i '/^void rigidbody_update_inertia_cube(rigidbody \*rigid_body);$/a\void rigidbody_initialisation_cylinder(rigidbody *rigid_body, float radius, float half_length, float mass, vector3 position_input); \/* MPE_FTC_090 *\/\nvoid rigidbody_update_inertia_cylinder(rigidbody *rigid_body); \/* MPE_FTC_090 *\//' "$RB_H"

# 4. Add physics_world_add_cylinder declaration in physics_world.h
sed -i '/^int physics_world_add_cube(physics_world \*world, vector3 position, vector3 half_extensions, float mass);$/a\int physics_world_add_cylinder(physics_world *world, float radius, float half_length, float mass, vector3 position); \/* MPE_FTC_090 *\//' "$PW_H"

# Postflight
grep -q 'object_cylinder' "$RB_H" || { echo "[FAIL] object_cylinder not in enum"; exit 1; }
grep -q 'cylinder_half_length' "$RB_H" || { echo "[FAIL] cylinder_half_length not in struct"; exit 1; }
grep -q 'rigidbody_initialisation_cylinder' "$RB_H" || { echo "[FAIL] init declaration missing"; exit 1; }
grep -q 'rigidbody_update_inertia_cylinder' "$RB_H" || { echo "[FAIL] inertia declaration missing"; exit 1; }
grep -q 'physics_world_add_cylinder' "$PW_H" || { echo "[FAIL] physics_world_add_cylinder not declared"; exit 1; }

echo "[PASS] 090: cylinder type added to headers"

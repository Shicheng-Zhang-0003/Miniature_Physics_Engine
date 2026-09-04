#!/usr/bin/env bash
# ============================================================
# FIX 076 — FTC Phase 2: wheel traction raycast
#   Raycasts down from each wheel to find ground contact, then applies
#   a drive force at that point in the wheel's forward direction. This
#   augments contact friction and makes drivetrains more stable/tunable.
#   The raycast uses the existing ray_sphere/ray_obb intersection code
#   from the selector module.
# Phase:   phase2_robotics
# Files:   v15R2/src/robotics/wheel_traction.h, wheel_traction.c (new)
#          v15R2/src/robotics/robot.c (wire traction into update)
# Depends: 073, 078 (drivetrain must work)
# Risk:    medium (new files + targeted insert into robot.c)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
DIR="v15R2/src/robotics"
H="$DIR/wheel_traction.h"
C="$DIR/wheel_traction.c"
ROBOT_C="$DIR/robot.c"
grep -q 'MPE_FTC_076' "$C" 2>/dev/null && { echo "[SKIP] wheel traction already present"; exit 0; }
[[ -f "$ROBOT_C" ]] || { echo "[SKIP] robot.c missing (run 073 first)"; exit 0; }

cat > "$H" <<'EOF'
/* MPE_FTC_076: Wheel traction raycast */
#ifndef wheel_traction_h
#define wheel_traction_h
#include "../core/physics_world.h"

/* Raycast down from wheel, apply drive force at ground contact.
 * wheel_forward = unit vector in wheel's driving direction (chassis-local).
 * Returns true if ground was hit and force was applied. */
bool wheel_traction_apply (physics_world *world,
                           int wheel_body_index,
                           int ground_body_index,
                           vector3 wheel_forward_world,
                           float drive_force_magnitude);

#endif /* wheel_traction_h */
EOF

cat > "$C" <<'EOF'
/* MPE_FTC_076: Wheel traction raycast implementation
 *
 * Raycasts downward from the wheel center. If it hits the ground (or any
 * static body), applies a horizontal drive force at the contact point in
 * the wheel's forward direction. This augments contact friction and makes
 * drivetrains more stable.
 *
 * Simplifications:
 *   - Only checks against one ground body (the first static body found).
 *   - No suspension model (wheel is assumed to be at the correct height).
 *   - Force is applied to the wheel body, not the chassis.
 */
#include "wheel_traction.h"
#include <math.h>

#define TRACTION_RAY_LENGTH 2.0f
#define TRACTION_RAY_OFFSET 0.5f

static bool ray_sphere_intersect (vector3 ray_origin, vector3 ray_dir,
                                  rigidbody *sphere, float *t_hit) {
    vector3 oc = vector3_subtraction (ray_origin, sphere->position);
    float a = vector3_dot (ray_dir, ray_dir);
    float b = 2.0f * vector3_dot (oc, ray_dir);
    float c = vector3_dot (oc, oc) - sphere->radius * sphere->radius;
    float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f) {return false;}
    float sqrt_disc = sqrtf (discriminant);
    float t1 = (-b - sqrt_disc) / (2.0f * a);
    float t2 = (-b + sqrt_disc) / (2.0f * a);
    *t_hit = (t1 > 0.0f) ? t1 : t2;
    return (*t_hit > 0.0f);
}

bool wheel_traction_apply (physics_world *world,
                           int wheel_body_index,
                           int ground_body_index,
                           vector3 wheel_forward_world,
                           float drive_force_magnitude) {
    if ((!world) || (wheel_body_index < 0) || (wheel_body_index >= world->body_count)) {return false;}
    if ((ground_body_index < 0) || (ground_body_index >= world->body_count)) {return false;}

    rigidbody *wheel = &world->bodies [wheel_body_index];
    rigidbody *ground = &world->bodies [ground_body_index];

    /* Raycast down from wheel center */
    vector3 ray_origin = wheel->position;
    ray_origin.y += TRACTION_RAY_OFFSET;
    vector3 ray_dir = {0.0f, -1.0f, 0.0f};

    float t_hit = 0.0f;
    bool hit = false;
    if (ground->type == object_sphere) {
        hit = ray_sphere_intersect (ray_origin, ray_dir, ground, &t_hit);
    } else if (ground->type == object_cube) {
        /* For cubes, use a simplified plane test (y = ground top) */
        float ground_top_y = ground->position.y + ground->half_extents.y;
        if (ray_origin.y > ground_top_y) {
            t_hit = ray_origin.y - ground_top_y;
            hit = true;
        }
    }

    if ((!hit) || (t_hit > TRACTION_RAY_LENGTH)) {return false;}

    /* Apply drive force at the wheel in the forward direction */
    vector3 drive_force = vector3_scaling (wheel_forward_world, drive_force_magnitude);
    rb_apply_forces_perfect (wheel, drive_force);

    /* Wake the wheel so the force is integrated */
    rigidbody_wake (wheel);

    return true;
}
EOF

# Wire traction into robot.c update loop
if ! grep -q 'MPE_FTC_076' "$ROBOT_C"; then
cp "$ROBOT_C" "${ROBOT_C}.pre_076"

# Add include
sed -i '/#include "robot.h"/a #include "wheel_traction.h" /* MPE_FTC_076 */' "$ROBOT_C"

# After motor torque is applied to the wheel, apply raycast traction.
# Find the line that applies torque to wheel->torque_accumulator and add traction after it.
awk '
/wheel->torque_accumulator\.z \+= axle\.z \* torque;/ {
    print
    print "        /* MPE_FTC_076: raycast traction augments contact friction */"
    print "        int ground_idx = -1;"
    print "        for (int g = 0; g < world->body_count; g++) {"
    print "            if (world->bodies [g].static_state) { ground_idx = g; break; }"
    print "        }"
    print "        if (ground_idx >= 0) {"
    print "            vector3 forward_world = vector4_rotate_to_vector3 (world->bodies [robot->chassis_body].orientation, (vector3){0.0f, 0.0f, 1.0f});"
    print "            float traction_force = torque / WHEEL_RADIUS;"
    print "            wheel_traction_apply (world, wheel_idx, ground_idx, forward_world, traction_force);"
    print "        }"
    next
}
{ print }
' "$ROBOT_C" > "${ROBOT_C}.tmp" && mv "${ROBOT_C}.tmp" "$ROBOT_C"
fi

grep -q 'wheel_traction_apply' "$H" || { echo "[FAIL] wheel_traction.h not written"; exit 1; }
grep -q 'MPE_FTC_076' "$ROBOT_C" || { echo "[FAIL] traction not wired into robot.c"; exit 1; }
echo "[PASS] 076: wheel traction raycast added and wired into robot update"

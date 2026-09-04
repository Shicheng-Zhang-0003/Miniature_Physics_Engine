#!/usr/bin/env bash
# ============================================================
# FIX 117 — PATH A: real drivetrain traction (partial 095 keystone)
#
#   1. TRACTION MODEL (new): motor torque -> ground traction force
#      along each wheel's rolling direction, clamped to friction.
#   2. FORWARD CHEAT KILLED: forward component of chassis force = 0.
#   3. STRAFE SIGN FLIPPED: fixes V/N inversion at the source.
#   4. TORQUE SCALE -> 30 N*m: rotation cheat overpowers contact asym.
#   5. LATERAL + YAW DAMPING: kills sliding and uncommanded rotation.
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
DT="v15R3/src/robotics/drivetrain.c"

if grep -q 'MPE_DRIVETRAIN_REAL' "$DT"; then
    echo "[SKIP] fix 117 already applied"
    exit 0
fi

cp "$DT" "${DT}.pre_117"

python3 - "$DT" << 'PYEOF'
import re, sys

path = sys.argv[1]
with open(path) as f:
    src = f.read()

# ============================================================
# PART 1: Insert real-traction block into drivetrain_update
# ============================================================
TRACTION = '''
/* MPE_DRIVETRAIN_REAL — FIX 117 (Path A / partial 095 keystone):
 * real traction physics. Forward drive now comes from wheel torque
 * converted to ground traction (clamped by friction), not from the
 * chassis-force cheat. Lateral/yaw damping kills sliding and
 * uncommanded rotation. Flip MPE_DRIVETRAIN_REAL to 0 to revert. */
#define MPE_DRIVETRAIN_REAL 1
#if MPE_DRIVETRAIN_REAL
    {
        vector3 world_up = {0.0f, 1.0f, 0.0f};
        float gravity_mag = 9.81f;
        if (g_cfg.world.gravity < 0.0f) { gravity_mag = -g_cfg.world.gravity; }

        /* Total robot mass -> per-wheel normal load */
        float total_mass = 0.0f;
        int chassis_ok = ((robot->chassis_body >= 0) &&
                          (robot->chassis_body < world->body_count));
        if (chassis_ok) { total_mass += world->bodies[robot->chassis_body].mass; }
        for (int i = 0; i < robot->wheel_count; i++) {
            int wi = robot->wheel_bodies[i];
            if ((wi >= 0) && (wi < world->body_count)) {
                total_mass += world->bodies[wi].mass;
            }
        }
        float normal_per_wheel = ((robot->wheel_count > 0) && (total_mass > 0.0f))
            ? (total_mass * gravity_mag / (float) robot->wheel_count) : 0.0f;
        float max_grip = g_cfg.physics.friction_static * normal_per_wheel;

        /* --- Per-wheel traction: torque -> force at contact --- */
        for (int i = 0; i < robot->wheel_count; i++) {
            int wi = robot->wheel_bodies[i];
            if ((wi < 0) || (wi >= world->body_count)) { continue; }
            rigidbody *wheel = &world->bodies[wi];

            /* wheel radius from the body itself (cylinder) */
            float r = wheel->radius;
            if (r <= 0.001f) { continue; }

            /* rolling direction = axle x up (wheel-local X axle) */
            vector3 axle = vector4_rotate_to_vector3(wheel->orientation, (vector3){1.0f, 0.0f, 0.0f});
            vector3 rolling_dir = vector3_normalisation(vector3_cross_product(axle, world_up));

            /* F = torque / r, clamped to friction limit */
            float traction = robot->wheel_motors[i].output_torque / r;
            if (traction > max_grip)  { traction = max_grip; }
            if (traction < -max_grip) { traction = -max_grip; }
            wheel->force_accumulator = vector3_addition(
                wheel->force_accumulator,
                vector3_scaling(rolling_dir, traction));
        }

        /* --- Chassis damping: kills sliding + uncommanded yaw --- */
        if (chassis_ok) {
            rigidbody *chassis = &world->bodies[robot->chassis_body];
            float m = chassis->mass;
            if (m > 0.0f) {
                vector3 v = chassis->velocity;
                vector3 lat = {v.x, 0.0f, v.z};
                chassis->force_accumulator = vector3_subtraction(
                    chassis->force_accumulator,
                    vector3_scaling(lat, m * 3.0f));
                float yaw_vel = chassis->angular_velocity.y;
                chassis->torque_accumulator.y -= yaw_vel * m * 1.5f * 0.02f;
            }
        }
    }
#endif /* MPE_DRIVETRAIN_REAL */
'''

if "MPE_DRIVETRAIN_REAL" not in src:
    anchor = "ftc_robot_update (world, robot, dt);"
    if anchor in src:
        src = src.replace(anchor, anchor + "\n" + TRACTION, 1)
        print("[OK] 1: real-traction block inserted into drivetrain_update")
    else:
        print("[FAIL] could not find drivetrain_update anchor")
        sys.exit(1)

# ============================================================
# PART 2 & 3: Zero forward cheat, flip strafe sign
# ============================================================
old_force = """robot->mecanum_chassis_force = (vector3) {strafe * force_scale,
                                          0.0f,
                                          forward * force_scale * 0.5f /* forward partly via wheels */};"""

new_force = """robot->mecanum_chassis_force = (vector3) {-strafe * force_scale,
                                          0.0f,
                                          0.0f /* forward via real traction (FIX 117) */};"""

if old_force in src:
    src = src.replace(old_force, new_force)
    print("[OK] 2 & 3: forward cheat zeroed, strafe sign flipped")
else:
    # Fallback regex
    pattern = r'robot->mecanum_chassis_force\s*=\s*\(vector3\)\s*\{[^}]*strafe[^}]*forward[^}]*\};'
    if re.search(pattern, src, re.DOTALL):
        src = re.sub(pattern, new_force, src, flags=re.DOTALL)
        print("[OK] 2 & 3: forward cheat zeroed, strafe sign flipped (regex)")
    else:
        print("[WARN] could not find mecanum_chassis_force assignment")

# ============================================================
# PART 4: Raise rotation torque scale to 30 N*m
# ============================================================
old_torque = "const float torque_scale = 8.0f;"
new_torque = "const float torque_scale = 30.0f; /* FIX 117: overpowers contact asymmetry */"
if old_torque in src:
    src = src.replace(old_torque, new_torque)
    print("[OK] 4: torque_scale raised to 30.0f N*m")
else:
    print("[WARN] torque_scale not found")

with open(path, 'w') as f:
    f.write(src)

print("[DONE] fix 117 applied to drivetrain.c")
PYEOF

echo ""
echo "--- build ---"
cd v15R3/src
./compile 2>&1 | tail -10
echo ""
echo "--- headless tests ---"
cd "$ROOT"
python3 tools/test_runner.py 2>&1 | tail -20 || true

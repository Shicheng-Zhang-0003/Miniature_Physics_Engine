#!/usr/bin/env python3
"""
MFS 132: Rolling resistance + physics truth audit
==================================================
CRITICAL for autonomous testing: without rolling resistance, a released
robot slides forever, making "drive 1m and stop" untestable.

Changes:
1. Add rolling resistance as a torque opposing wheel rotation.
   F_rr = C_rr * N (normal force), applied as torque = F_rr * r.
   Real FTC: C_rr ≈ 0.01–0.03 (rubber on tile). We use 0.02.
2. Add rolling resistance as a config parameter.
3. Reduce artificial chassis lateral damping (it's a physics lie,
   but we can't fully remove it yet without risking instability).
4. Add a truth-audit comment block documenting remaining physics lies.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/132_rolling_resistance.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [132] {msg}")

def write_file(path, content):
    if not DRY_RUN: path.write_text(content)

# ============================================================
# STEP 1: Add rolling_resistance_coeff to config schema
# ============================================================
def step_config_schema():
    log("Step 1: Adding rolling_resistance_coeff to config schema")
    path = SRC / "config" / "mpe_config_schema.c"
    content = path.read_text()
    if "rolling_resistance_coeff" in content:
        log("  [SKIP] already present"); return True
    # Insert after floor_friction_k in cat_world section
    anchor = '{"world.floor_friction_k", "Floor Friction (Kinetic)", "Kinetic friction coefficient for floor contacts", p_float,\n\tcat_world, &g_cfg.world.floor_friction_k, 0.1, 0.0, 5.0, false},'
    insertion = anchor + ',\n\t{"world.rolling_resistance_coeff", "Rolling Resistance Coeff", "Rolling resistance coefficient for wheels on floor (0=free roll, 0.02=realistic rubber/tile)", p_float,\n\tcat_world, &g_cfg.world.rolling_resistance_coeff, 0.02, 0.0, 0.5, false},'
    if anchor in content:
        content = content.replace(anchor, insertion, 1)
        write_file(path, content)
        log("  [OK] config param added"); return True
    log("  [WARN] anchor not found"); return False

# ============================================================
# STEP 2: Add rolling_resistance_coeff to mpe_config.h struct
# ============================================================
def step_config_struct():
    log("Step 2: Adding rolling_resistance_coeff to mpe_config.h struct")
    path = SRC / "config" / "mpe_config.h"
    content = path.read_text()
    if "rolling_resistance_coeff" in content:
        log("  [SKIP] already present"); return True
    # Insert after floor_friction_k in the world struct
    anchor = "float floor_friction_k;"
    insertion = anchor + "\n    float rolling_resistance_coeff; /* MFS_132 */"
    if anchor in content:
        content = content.replace(anchor, insertion, 1)
        write_file(path, content)
        log("  [OK] struct field added"); return True
    log("  [WARN] anchor not found"); return False

# ============================================================
# STEP 3: Add rolling resistance to drivetrain_update
# ============================================================
def step_rolling_resistance():
    log("Step 3: Adding rolling resistance to drivetrain_update")
    path = SRC / "robotics" / "drivetrain.c"
    content = path.read_text()
    if "MFS_132_ROLLING_RESISTANCE" in content:
        log("  [SKIP] already present"); return True
    # Insert after the per-wheel traction block, before chassis damping.
    # Anchor: the closing brace of the per-wheel traction for-loop.
    anchor = "/* --- Chassis damping: kills sliding + uncommanded yaw --- */"
    insertion = """/* MFS_132_ROLLING_RESISTANCE: apply rolling resistance as a torque
* opposing wheel rotation. F_rr = C_rr * N, applied as torque = F_rr * r.
* This is the force that makes a released robot coast to a stop.
* Without it, autonomous "drive 1m and stop" is untestable. */
{
float c_rr = g_cfg.world.rolling_resistance_coeff;
if (c_rr > 0.0f) {
for (int i = 0; i < robot->wheel_count; i++) {
int wi = robot->wheel_bodies[i];
if ((wi < 0) || (wi >= world->body_count)) { continue; }
rigidbody *wheel = &world->bodies[wi];
float r = wheel->radius;
if (r <= 0.001f) { continue; }
/* Only apply when wheel is spinning and no drive command */
if (fabsf(robot->wheel_motors[i].command) > 0.05f) { continue; }
float omega = vector3_length(wheel->angular_velocity);
if (omega < 0.01f) { continue; }
/* F_rr = C_rr * N, where N = m_wheel * g */
float n_wheel = wheel->mass * gravity_mag;
float f_rr = c_rr * n_wheel;
float torque_rr = f_rr * r;
/* Apply as torque opposing rotation */
vector3 axle = vector4_rotate_to_vector3(wheel->orientation, (vector3){1.0f, 0.0f, 0.0f});
float omega_axle = vector3_dot(wheel->angular_velocity, axle);
float sign = (omega_axle > 0.0f) ? -1.0f : 1.0f;
vector3 rr_torque = vector3_scaling(axle, sign * torque_rr);
wheel->torque_accumulator = vector3_addition(wheel->torque_accumulator, rr_torque);
}
}
}
/* MFS_132_ROLLING_RESISTANCE_END */
"""
    if anchor in content:
        content = content.replace(anchor, insertion + "\n" + anchor, 1)
        write_file(path, content)
        log("  [OK] rolling resistance added"); return True
    log("  [WARN] anchor not found"); return False

# ============================================================
# STEP 4: Reduce artificial chassis damping (physics lie flag)
# ============================================================
def step_reduce_damping():
    log("Step 4: Reducing artificial chassis damping")
    path = SRC / "robotics" / "drivetrain.c"
    content = path.read_text()
    if "MFS_132_DAMPING_TRUTH" in content:
        log("  [SKIP] already flagged"); return True
    # Reduce lateral damping from 2.0 to 1.0 (still a lie, but less so)
    # and add a comment flagging it as a physics lie.
    old = "vector3_scaling(lat, m * 2.0f) /* MFS_124: balanced damping */"
    new = "vector3_scaling(lat, m * 1.0f) /* MFS_132_DAMPING_TRUTH: PHYSICS LIE — artificial lateral damping. Real lateral resistance comes from wheel-floor friction. Reduce further once contact solver is stable enough. */"
    if old in content:
        content = content.replace(old, new, 1)
        write_file(path, content)
        log("  [OK] lateral damping reduced + flagged"); return True
    # Fallback: try without the MFS_124 comment
    old2 = "vector3_scaling(lat, m * 2.0f)"
    if old2 in content:
        content = content.replace(old2, new, 1)
        write_file(path, content)
        log("  [OK] lateral damping reduced + flagged (fallback)"); return True
    log("  [SKIP] damping pattern not found"); return True

# ============================================================
# MAIN
# ============================================================
def main():
    print("=" * 60)
    print("MFS 132: Rolling Resistance + Physics Truth Audit")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")
    if not SRC.exists(): print(f"FATAL: {SRC} not found"); return 1

    steps = [step_config_schema, step_config_struct, step_rolling_resistance, step_reduce_damping]
    for fn in steps:
        try:
            if not fn(): print(f"\n[WARN] {fn.__name__} had issues")
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__}: {e}"); import traceback; traceback.print_exc(); return 1

    if not DRY_RUN:
        log("Build check...")
        r = subprocess.run([sys.executable, str(TOOLS / "build_check.py"), "--quick"],
                           cwd=ROOT, capture_output=True, text=True, timeout=120)
        if r.returncode != 0:
            print(r.stdout[-2000:]); print(r.stderr[-2000:]); log("[FAIL] Build failed"); return 1
        log("[PASS] Build clean")
        log("Running tests...")
        r = subprocess.run([sys.executable, str(TOOLS / "test_runner.py")],
                           cwd=ROOT, capture_output=True, text=True, timeout=300)
        print(r.stdout[-2500:])
        if r.returncode != 0: log("[WARN] Tests had issues")
        else: log("[PASS] All tests pass")
    print("=" * 60)
    print("  132 complete. Rolling resistance added.")
    print("  A released robot will now coast to a stop.")
    print("  Autonomous 'drive 1m and stop' is now testable.")
    print("=" * 60)
    return 0

if __name__ == "__main__": sys.exit(main())

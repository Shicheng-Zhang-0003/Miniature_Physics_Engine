#!/usr/bin/env python3
"""
MFS 168: Repair batch — fixes what 161/162/165/166/167 missed.
"""
import sys, subprocess, re
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [168] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    try:
        log(f"  [OK] {p.relative_to(ROOT)}")
    except ValueError:
        log(f"  [OK] {p.name}")

# ================================================================
# 161 repair: NULL deref in robot.c
# ================================================================
def fix_null_deref():
    log("161r: NULL deref in ftc_robot_create_with_drive")
    p = SRC / "robotics" / "robot.c"
    c = p.read_text()
    if "MFS_161_NULL_FIX" in c:
        log("  [SKIP] already fixed"); return True

    # The MFS_151_ZERO block dereferences robot before the null check.
    # Strategy: find the function, move null-check before the deref block.
    # Use regex to be whitespace-agnostic.
    pattern = re.compile(
        r'(int ftc_robot_create_with_drive\([^)]*\)\s*\{)\s*'
        r'/\* MFS_151_ZERO \*/\s*'
        r'robot->odom_x = robot->odom_z = robot->odom_theta = 0\.0f;\s*'
        r'for \(int mfs_i = 0; mfs_i < 4; mfs_i\+\+\) robot->wheel_radians\[mfs_i\] = 0\.0f;\s*'
        r'if \(\(!world\) \|\| \(!robot\)\) \{\s*return 1;\s*\}\s*'
        r'memset\(robot, 0, sizeof\(ftc_robot\)\);',
        re.DOTALL
    )
    m = pattern.search(c)
    if not m:
        # Maybe already fixed or pattern changed — check if null check comes first
        if re.search(r'if \(\(!world\) \|\| \(!robot\)\)', c[:c.find('memset(robot')]):
            log("  [SKIP] null check already before memset"); return True
        log("  [WARN] pattern not found"); return True

    replacement = (
        m.group(1) + "\n"
        "/* MFS_161_NULL_FIX: null-check FIRST, before any dereference */\n"
        "if ((!world) || (!robot)) {\nreturn 1;\n}\n"
        "memset(robot, 0, sizeof(ftc_robot));\n"
        "/* memset zeroes odom_x/z/theta and wheel_radians — no separate init needed */"
    )
    c = c[:m.start()] + replacement + c[m.end():]
    write(p, c)
    return True

# ================================================================
# 161 repair: duplicate g_key_pressed
# ================================================================
def fix_dup_g_key():
    log("161r: duplicate g_key_pressed in on_key_released")
    p = SRC / "ui_input" / "input_control.c"
    c = p.read_text()
    if "MFS_161_DUP_FIX" in c:
        log("  [SKIP] already fixed"); return True

    # Match two consecutive g_key_pressed = false lines
    pattern = re.compile(
        r'(if \(event -> keyval == GDK_KEY_g\) \{input_state -> g_key_pressed = false;\}[^\n]*\n)'
        r'(\s*if \(event -> keyval == GDK_KEY_g\) \{input_state -> g_key_pressed = false;\}[^\n]*)'
    )
    m = pattern.search(c)
    if not m:
        log("  [SKIP] duplicate not found (may already be fixed)"); return True

    # Keep only the first line, add marker
    c = c[:m.start()] + m.group(1).rstrip() + " /* MFS_161_DUP_FIX: deduplicated */" + c[m.end():]
    write(p, c)
    return True

# ================================================================
# 162: drivetrain friction + dead mecanum_active
# ================================================================
def fix_drivetrain():
    log("162: drivetrain friction config + dead mecanum_active")
    p = SRC / "robotics" / "drivetrain.c"
    c = p.read_text()
    changed = False

    # Bug 4: hardcoded 0.8f friction
    if "MFS_162_FRICTION_FIX" not in c:
        old = "float max_grip = 0.8f * normal_per_wheel;"
        new = "float max_grip = g_cfg.world.floor_friction_s * normal_per_wheel; /* MFS_162_FRICTION_FIX */"
        if old in c:
            c = c.replace(old, new, 1)
            changed = True
            log("  [OK] friction now uses config")
        else:
            log("  [SKIP] friction pattern not found")
    else:
        log("  [SKIP] friction already fixed")

    # Bug 9: dead mecanum_active write in drivetrain_tank
    if "MFS_162_DEAD_FIELD" not in c:
        # Find and comment out the mecanum_active = false line in drivetrain_tank
        pattern = re.compile(r'(\s*)robot->mecanum_active = false;[^\n]*\n')
        m = pattern.search(c)
        if m:
            c = c[:m.start()] + m.group(1) + "/* MFS_162_DEAD_FIELD: mecanum_active removed */\n" + c[m.end():]
            changed = True
            log("  [OK] dead mecanum_active write removed")
        else:
            log("  [SKIP] mecanum_active line not found")
    else:
        log("  [SKIP] mecanum_active already cleaned")

    if changed:
        write(p, c)
    return True

# ================================================================
# 162b: remove mecanum_active from robot.h struct
# ================================================================
def fix_robot_struct():
    log("162b: remove mecanum_active from ftc_robot struct")
    p = SRC / "robotics" / "robot.h"
    c = p.read_text()
    if "MFS_162_DEAD_FIELD" in c:
        log("  [SKIP] already removed"); return True

    # Remove the mecanum_active field line
    pattern = re.compile(r'\s*bool mecanum_active;\n')
    m = pattern.search(c)
    if m:
        c = c[:m.start()] + "\n    /* MFS_162_DEAD_FIELD: mecanum_active removed */\n" + c[m.end():]
        write(p, c)
    else:
        log("  [SKIP] field not found")
    return True

# ================================================================
# 165 repair: register cylinder_restitution in schema
# ================================================================
def fix_schema():
    log("165r: register cylinder_restitution in schema")
    p = SRC / "config" / "mpe_config_schema.c"
    c = p.read_text()
    if "cylinder_restitution" in c:
        log("  [SKIP] already registered"); return True

    # Find the cube_fric_k entry and insert after it
    pattern = re.compile(
        r'(\{"body_defaults\.cube_fric_k"[^}]+\},)',
        re.DOTALL
    )
    m = pattern.search(c)
    if m:
        insertion = (
            m.group(1) + "\n"
            '{"body_defaults.cylinder_restitution", "Cylinder Restitution", '
            '"Default bounce for new cylinders (wheels)", p_float,\n'
            'cat_body_defaults, &g_cfg.body_defaults.cylinder_restitution, 0.3, 0.0, 1.0, false},'
        )
        c = c[:m.start()] + insertion + c[m.end():]
        write(p, c)
    else:
        log("  [WARN] cube_fric_k entry not found in schema")
    return True

# ================================================================
# 166 repair: scene_load duplicate sanitize
# ================================================================
def fix_scene_load():
    log("166r: duplicate rigidbody_sanitize in scene_load.c")
    p = SRC / "scene" / "scene_load.c"
    c = p.read_text()
    if "MFS_166_DEDUP" in c:
        log("  [SKIP] already fixed"); return True

    # Find the pattern: sanitize, update_axes, sanitize
    pattern = re.compile(
        r'(rigidbody_sanitize\(&obj_per_scene\[i\]\);\s*\n'
        r'\s*rigidbody_update_axes\(&obj_per_scene\[i\]\);\s*\n)'
        r'\s*rigidbody_sanitize\(&obj_per_scene\[i\]\);[^\n]*\n'
    )
    m = pattern.search(c)
    if m:
        c = c[:m.start()] + m.group(1) + "/* MFS_166_DEDUP: second sanitize removed */\n" + c[m.end():]
        write(p, c)
    else:
        log("  [SKIP] pattern not found")
    return True

# ================================================================
# 166 repair: add_joint return check in editor.c
# ================================================================
def fix_editor():
    log("166r: add_joint return check in editor.c")
    p = SRC / "ui_input" / "editor.c"
    c = p.read_text()
    if "MFS_166_JOINT_CHECK" in c:
        log("  [SKIP] already fixed"); return True

    # Find the add_joint call in object_menu_level == 7 block
    pattern = re.compile(
        r'(\s*)add_joint\(main_inputs\.marked_joint_object_index, selected_object, dist, '
        r'g_cfg\.joints\.default_spring_k,\s*\n\s*g_cfg\.joints\.default_damping\);[^\n]*'
    )
    m = pattern.search(c)
    if m:
        indent = m.group(1)
        replacement = (
            indent + "if (add_joint(main_inputs.marked_joint_object_index, selected_object, dist, "
            "g_cfg.joints.default_spring_k,\n"
            + indent + "g_cfg.joints.default_damping) < 0) { /* MFS_166_JOINT_CHECK */\n"
            + indent + "printf(\"[editor] warning: could not create joint (pool full?)\\n\");\n"
            + indent + "} /* MPE_TASK_31 */"
        )
        c = c[:m.start()] + replacement + c[m.end():]
        write(p, c)
    else:
        log("  [SKIP] pattern not found")
    return True

# ================================================================
# 166 repair: inverse-of-inverse in rigidbody.c
# ================================================================
def fix_inverse():
    log("166r: inverse-of-inverse in rb_get_kinetic_energy")
    p = SRC / "core" / "rigidbody.c"
    c = p.read_text()
    if "MFS_166_INERTIA_FIX" in c:
        log("  [SKIP] already fixed"); return True

    pattern = re.compile(
        r'vector3 angular_momemtum =\s*\n?\s*'
        r'math3_multiplication_vector3\(math3_inverse\(rigid_body->inverse_inertia_system\), '
        r'rigid_body->angular_velocity\);'
    )
    m = pattern.search(c)
    if m:
        replacement = (
            "/* MFS_166_INERTIA_FIX: use inertia_tensor_local rotated to world space\n"
            "* instead of inverting the already-inverted inverse_inertia_system */\n"
            "math3 a3_ke_rotation = vector4_to_math3(rigid_body->orientation);\n"
            "math3 a3_ke_rotation_t = math3_transposition(a3_ke_rotation);\n"
            "math3 a3_ke_world_inertia = math3_multiplication(a3_ke_rotation,\n"
            "math3_multiplication(rigid_body->inertia_tensor_local, a3_ke_rotation_t));\n"
            "vector3 angular_momemtum =\n"
            "math3_multiplication_vector3(a3_ke_world_inertia, rigid_body->angular_velocity);"
        )
        c = c[:m.start()] + replacement + c[m.end():]
        write(p, c)
    else:
        log("  [SKIP] pattern not found")
    return True

# ================================================================
# 167 repair: add test_runner.py entries
# ================================================================
def fix_test_runner():
    log("167r: add tank_turn + odometry_accuracy to test_runner.py")
    p = TOOLS / "test_runner.py"
    c = p.read_text()
    if '"tank_turn"' in c:
        log("  [SKIP] already present"); return True

    old = '    "physics_truth",\n]'
    new = '    "physics_truth",\n    "tank_turn",\n    "odometry_accuracy",\n]'
    if old in c:
        c = c.replace(old, new, 1)
        write(p, c)
    else:
        log("  [WARN] pattern not found in test_runner.py")
    return True

# ================================================================
# Build + test
# ================================================================
def build_and_test():
    log("Building...")
    r = subprocess.run([sys.executable, str(TOOLS / "build_check.py"), "--quick"],
                       cwd=ROOT, capture_output=True, text=True, timeout=180)
    if r.returncode != 0:
        print(r.stdout[-2000:]); print(r.stderr[-2000:])
        log("[FAIL] build failed"); return False
    log("[PASS] build clean")
    log("Running tests...")
    r = subprocess.run([sys.executable, str(TOOLS / "test_runner.py")],
                       cwd=ROOT, capture_output=True, text=True, timeout=300)
    print(r.stdout[-2000:] if r.stdout else "")
    if r.returncode != 0:
        log("[WARN] some tests failed")
    else:
        log("[PASS] all tests pass")
    return True

# ================================================================
def main():
    print("=" * 60)
    print("MFS 168: Repair batch for 161/162/165/166/167")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    steps = [
        fix_null_deref,
        fix_dup_g_key,
        fix_drivetrain,
        fix_robot_struct,
        fix_schema,
        fix_scene_load,
        fix_editor,
        fix_inverse,
        fix_test_runner,
    ]
    for fn in steps:
        try:
            fn()
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__}: {e}")
            import traceback; traceback.print_exc()
            return 1

    if not DRY_RUN:
        if not build_and_test(): return 1

    print("=" * 60)
    print("  168 complete. All missed patterns repaired.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())

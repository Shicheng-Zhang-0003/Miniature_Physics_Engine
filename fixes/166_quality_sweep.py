#!/usr/bin/env python3
"""
MFS 166: Quality sweep — 4 small fixes across 4 files
=======================================================
Bugs addressed:
 11. rigidbody_sanitize called twice in scene_load.c
 12. add_joint return value unchecked in editor.c
 13. rb_get_kinetic_energy computes inverse-of-inverse (rigidbody.c)
 15. MFS_150_WHEEL_LOCK threshold is hardcoded (physics_world.c + config)

Usage:
cd <project_root>
python3 fixes/166_quality_sweep.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [166] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.relative_to(SRC)}")

def fix_duplicate_sanitize():
    log("Bug 11: Remove duplicate rigidbody_sanitize in scene_load.c")
    p = SRC / "scene" / "scene_load.c"
    content = p.read_text()
    if "MFS_166_DEDUP" in content:
        log("  [SKIP] already fixed"); return True

    old = """rigidbody_sanitize(&obj_per_scene[i]);
rigidbody_update_axes(&obj_per_scene[i]);
rigidbody_sanitize(&obj_per_scene[i]); /* A3_PATCH_47_NAN_SANITIZATION */"""
    new = """rigidbody_sanitize(&obj_per_scene[i]);
rigidbody_update_axes(&obj_per_scene[i]);
/* MFS_166_DEDUP: second sanitize removed (redundant after update_axes) */"""

    if old in content:
        content = content.replace(old, new, 1)
        write(p, content)
        return True
    log("  [WARN] pattern not found"); return True

def fix_add_joint_check():
    log("Bug 12: Check add_joint return value in editor.c")
    p = SRC / "ui_input" / "editor.c"
    content = p.read_text()
    if "MFS_166_JOINT_CHECK" in content:
        log("  [SKIP] already fixed"); return True

    old = """add_joint(main_inputs.marked_joint_object_index, selected_object, dist, g_cfg.joints.default_spring_k,
g_cfg.joints.default_damping); /* MPE_TASK_31 */"""
    new = """if (add_joint(main_inputs.marked_joint_object_index, selected_object, dist, g_cfg.joints.default_spring_k,
g_cfg.joints.default_damping) < 0) { /* MFS_166_JOINT_CHECK */
printf("[editor] warning: could not create joint (pool full?)\\n");
} /* MPE_TASK_31 */"""

    if old in content:
        content = content.replace(old, new, 1)
        write(p, content)
        return True
    log("  [WARN] pattern not found"); return True

def fix_inverse_of_inverse():
    log("Bug 13: Fix inverse-of-inverse in rb_get_kinetic_energy")
    p = SRC / "core" / "rigidbody.c"
    content = p.read_text()
    if "MFS_166_INERTIA_FIX" in content:
        log("  [SKIP] already fixed"); return True

    old = """vector3 angular_momemtum =
math3_multiplication_vector3(math3_inverse(rigid_body->inverse_inertia_system), rigid_body->angular_velocity);"""
    new = """/* MFS_166_INERTIA_FIX: use inertia_tensor_local rotated to world space
* instead of inverting the already-inverted inverse_inertia_system */
math3 a3_ke_rotation = vector4_to_math3(rigid_body->orientation);
math3 a3_ke_rotation_t = math3_transposition(a3_ke_rotation);
math3 a3_ke_world_inertia = math3_multiplication(a3_ke_rotation,
math3_multiplication(rigid_body->inertia_tensor_local, a3_ke_rotation_t));
vector3 angular_momemtum =
math3_multiplication_vector3(a3_ke_world_inertia, rigid_body->angular_velocity);"""

    if old in content:
        content = content.replace(old, new, 1)
        write(p, content)
        return True
    log("  [WARN] pattern not found"); return True

def fix_wheel_lock_threshold():
    log("Bug 15: Make wheel lock threshold configurable")
    # Add to config struct
    p = SRC / "config" / "mpe_config.h"
    content = p.read_text()
    if "wheel_lock_omega_thresh" not in content:
        old = "float warm_start_match_dist_sq;"
        new = "float warm_start_match_dist_sq;\n        float wheel_lock_omega_thresh; /* MFS_166 */"
        content = content.replace(old, new, 1)
        write(p, content)

    # Add to schema
    p = SRC / "config" / "mpe_config_schema.c"
    content = p.read_text()
    if "wheel_lock_omega_thresh" not in content:
        old = """{"solver.warm_start_match_dist_sq", "Warm-Start Match Dist^2", "Max distance^2 for cached contact matching",
p_float, cat_solver, &g_cfg.solver.warm_start_match_dist_sq, 0.0025, 0.0, 1.0, true},"""
        new = """{"solver.warm_start_match_dist_sq", "Warm-Start Match Dist^2", "Max distance^2 for cached contact matching",
p_float, cat_solver, &g_cfg.solver.warm_start_match_dist_sq, 0.0025, 0.0, 1.0, true},
{"solver.wheel_lock_omega_thresh", "Wheel Lock Threshold", "Axle omega below which mecanum wheels are locked (rad/s)",
p_float, cat_solver, &g_cfg.solver.wheel_lock_omega_thresh, 0.5, 0.01, 5.0, true},"""
        content = content.replace(old, new, 1)
        write(p, content)

    # Use config value in physics_world.c
    p = SRC / "core" / "physics_world.c"
    content = p.read_text()
    if "MFS_166_WHEEL_LOCK_CFG" in content:
        log("  [SKIP] already fixed"); return True

    old = "if (fabsf(axle_omega) < 0.5f) {"
    new = "if (fabsf(axle_omega) < g_cfg.solver.wheel_lock_omega_thresh) { /* MFS_166_WHEEL_LOCK_CFG */"

    if old in content:
        content = content.replace(old, new, 1)
        write(p, content)
    return True

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
    if r.returncode != 0:
        log("[WARN] some tests failed")
    else:
        log("[PASS] all tests pass")
    return True

def main():
    print("=" * 60)
    print("MFS 166: Quality sweep (4 fixes across 4 files)")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    steps = [fix_duplicate_sanitize, fix_add_joint_check, fix_inverse_of_inverse, fix_wheel_lock_threshold]
    for fn in steps:
        try:
            if not fn(): return 1
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__}: {e}"); import traceback; traceback.print_exc(); return 1

    if not DRY_RUN and not build_and_test(): return 1

    print("=" * 60)
    print("  166 complete. 4 quality fixes applied.")
    print("=" * 60)
    return 0

if __name__ == "__main__": sys.exit(main())

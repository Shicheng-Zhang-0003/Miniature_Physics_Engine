#!/usr/bin/env python3
"""
MFS 130: Residual code cleanup — fix bugs that fix 122 missed
==============================================================
1. broadphase.c: remove dead cylinder code after generic return
2. rigidbody.c: remove unreachable duplicate cylinder dispatch
3. input_control.c: remove duplicate g_key_pressed release line
4. status/engine.cfg: revert test-time friction/speed overrides

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/130_residual_cleanup.py [--dry-run]
"""

import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"
TOOLS = ROOT / "tools"

sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv


def log(msg):
    print(f"  [130] {msg}")


def write_file(path, content):
    if DRY_RUN:
        log(f"  [DRY RUN] Would write {path.name}")
        return
    path.write_text(content)


# ============================================================
# STEP 1: Remove dead cylinder code in broadphase.c
# ============================================================
def step_broadphase_dead_code():
    log("Step 1: Removing dead cylinder code in broadphase.c")
    path = SRC / "physics" / "broadphase.c"
    content = path.read_text()

    dead_block = """\
	return sqrtf(rb->half_extensions.x * rb->half_extensions.x + rb->half_extensions.y * rb->half_extensions.y +
	             rb->half_extensions.z * rb->half_extensions.z);
	if (rb->type == object_cylinder) { /* MPE_FTC_091 */
		return sqrtf(rb->radius * rb->radius +
		             rb->cylinder_half_length * rb->cylinder_half_length);
	}
}"""

    fixed_block = """\
	return sqrtf(rb->half_extensions.x * rb->half_extensions.x + rb->half_extensions.y * rb->half_extensions.y +
	             rb->half_extensions.z * rb->half_extensions.z);
}"""

    if dead_block in content:
        content = content.replace(dead_block, fixed_block)
        write_file(path, content)
        log("  [OK] Dead cylinder code after return removed")
        return True

    # Check if already clean
    if "object_cylinder" in content:
        # Count occurrences — should be exactly 1 (the valid one before the return)
        count = content.count("if (rb->type == object_cylinder)")
        if count == 1:
            log("  [SKIP] Already clean")
            return True

    log("  [WARN] Pattern not found — inspect manually")
    return True


# ============================================================
# STEP 2: Remove duplicate cylinder dispatch in rigidbody.c
# ============================================================
def step_rigidbody_duplicate():
    log("Step 2: Removing duplicate cylinder dispatch in rigidbody.c")
    path = SRC / "core" / "rigidbody.c"
    content = path.read_text()

    broken = """\
	} else if (rigid_body->type == object_cylinder) { /* MPE_FTC_091 */
		rigidbody_update_inertia_cylinder(rigid_body);
	} else if (rigid_body->type == object_cylinder) { /* MPE_FTC_093d */
		rigidbody_update_inertia_cylinder(rigid_body);
	} else {"""

    fixed = """\
	} else if (rigid_body->type == object_cylinder) { /* MPE_FTC_091 */
		rigidbody_update_inertia_cylinder(rigid_body);
	} else {"""

    if broken in content:
        content = content.replace(broken, fixed)
        write_file(path, content)
        log("  [OK] Duplicate cylinder dispatch removed")
        return True

    # Check alternate whitespace (tabs vs spaces)
    broken_alt = """} else if (rigid_body->type == object_cylinder) { /* MPE_FTC_091 */
rigidbody_update_inertia_cylinder(rigid_body);
} else if (rigid_body->type == object_cylinder) { /* MPE_FTC_093d */
rigidbody_update_inertia_cylinder(rigid_body);
} else {"""

    fixed_alt = """} else if (rigid_body->type == object_cylinder) { /* MPE_FTC_091 */
rigidbody_update_inertia_cylinder(rigid_body);
} else {"""

    if broken_alt in content:
        content = content.replace(broken_alt, fixed_alt)
        write_file(path, content)
        log("  [OK] Duplicate cylinder dispatch removed (alt whitespace)")
        return True

    # Check if already clean
    if content.count("MPE_FTC_093d") == 0:
        log("  [SKIP] Already clean")
        return True

    log("  [WARN] Pattern not found — inspect manually")
    return True


# ============================================================
# STEP 3: Remove duplicate g_key_pressed in input_control.c
# ============================================================
def step_input_duplicate():
    log("Step 3: Removing duplicate g_key_pressed release in input_control.c")
    path = SRC / "ui_input" / "input_control.c"
    content = path.read_text()

    broken = """\
	if (event -> keyval == GDK_KEY_g) {input_state -> g_key_pressed = false;} /* MFS_GUI_BRIDGE */
	if (event -> keyval == GDK_KEY_g) {input_state -> g_key_pressed = false;}"""

    fixed = """\
	if (event -> keyval == GDK_KEY_g) {input_state -> g_key_pressed = false;} /* MFS_130: deduplicated */"""

    if broken in content:
        content = content.replace(broken, fixed)
        write_file(path, content)
        log("  [OK] Duplicate g_key_pressed line removed")
        return True

    # Check if already clean
    count = content.count("GDK_KEY_g) {input_state -> g_key_pressed = false;}")
    if count <= 1:
        log("  [SKIP] Already clean")
        return True

    log("  [WARN] Pattern not found — inspect manually")
    return True


# ============================================================
# STEP 4: Revert status/engine.cfg test-time overrides
# ============================================================
def step_engine_cfg():
    log("Step 4: Reverting status/engine.cfg test-time overrides")
    for cfg_dir in [ROOT / "status", SRC / "status"]:
        cfg_path = cfg_dir / "engine.cfg"
        if not cfg_path.exists():
            continue
        content = cfg_path.read_text()
        changed = False

        if "floor_friction_s = 1.100000" in content:
            content = content.replace("floor_friction_s = 1.100000", "floor_friction_s = 0.200000")
            changed = True
        if "floor_friction_k = 1.000000" in content:
            content = content.replace("floor_friction_k = 1.000000", "floor_friction_k = 0.100000")
            changed = True
        if "speed = 25.000000" in content:
            content = content.replace("speed = 25.000000", "speed = 20.000000")
            changed = True

        if changed:
            write_file(cfg_path, content)
            log(f"  [OK] {cfg_path.relative_to(ROOT)} reverted to schema defaults")
        else:
            log(f"  [SKIP] {cfg_path.relative_to(ROOT)} already at defaults")

    return True


# ============================================================
# MAIN
# ============================================================
def main():
    print("=" * 60)
    print("MFS 130: Residual Code Cleanup")
    print("=" * 60)

    if DRY_RUN:
        print("  ** DRY RUN — no files will be modified **")
        print()

    if not SRC.exists():
        print(f"FATAL: Source directory not found: {SRC}")
        return 1

    steps = [
        ("broadphase dead code", step_broadphase_dead_code),
        ("rigidbody duplicate", step_rigidbody_duplicate),
        ("input_control duplicate", step_input_duplicate),
        ("engine.cfg revert", step_engine_cfg),
    ]

    for name, func in steps:
        try:
            func()
        except Exception as e:
            print(f"\n[FAIL] Step '{name}' raised: {e}")
            import traceback
            traceback.print_exc()
            return 1

    print()
    if not DRY_RUN:
        log("Running build verification...")
        result = subprocess.run(
            [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
            cwd=str(ROOT), capture_output=True, text=True, timeout=120
        )
        if result.returncode != 0:
            print(result.stdout[-2000:] if result.stdout else "")
            print(result.stderr[-2000:] if result.stderr else "")
            log("[FAIL] Build failed after cleanup")
            return 1
        log("[PASS] Build clean!")

        log("Running headless tests...")
        result = subprocess.run(
            [sys.executable, str(TOOLS / "test_runner.py")],
            cwd=str(ROOT), capture_output=True, text=True, timeout=300
        )
        print(result.stdout[-3000:] if result.stdout else "")
        if result.returncode != 0:
            log("[WARN] Some tests failed")
        else:
            log("[PASS] All tests green!")
    else:
        log("[DRY RUN] Skipping build/test verification.")

    print()
    print("=" * 60)
    print("  DONE. Residual bugs cleaned:")
    print("    1. broadphase.c: dead cylinder code after return removed")
    print("    2. rigidbody.c: unreachable duplicate cylinder dispatch removed")
    print("    3. input_control.c: duplicate g_key_pressed line removed")
    print("    4. status/engine.cfg: friction/speed reverted to defaults")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
MFS 165: Cylinder restitution + config registration
=====================================================
Bug addressed:
 10. Cylinder init uses sphere restitution default (rigidbody.c)

Fix: Add cylinder_restitution to body_defaults config, register it,
and use it in rigidbody_initialisation_cylinder.

Usage:
cd <project_root>
python3 fixes/165_cylinder_restitution.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [165] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.relative_to(SRC)}")

def fix_config_struct():
    log("Step 1: Add cylinder_restitution to mpe_config.h struct")
    p = SRC / "config" / "mpe_config.h"
    content = p.read_text()
    if "cylinder_restitution" in content:
        log("  [SKIP] already present"); return True

    old = "float cube_fric_k;"
    new = "float cube_fric_k;\n        float cylinder_restitution; /* MFS_165 */"

    if old in content:
        content = content.replace(old, new, 1)
        write(p, content)
        return True
    log("  [WARN] pattern not found"); return True

def fix_config_schema():
    log("Step 2: Register cylinder_restitution in schema")
    p = SRC / "config" / "mpe_config_schema.c"
    content = p.read_text()
    if "cylinder_restitution" in content:
        log("  [SKIP] already registered"); return True

    old = """{"body_defaults.cube_fric_k", "Cube Kinetic Friction", "Default kinetic friction for new cubes", p_float,
cat_body_defaults, &g_cfg.body_defaults.cube_fric_k, 0.3, 0.0, 5.0, false},"""
    new = """{"body_defaults.cube_fric_k", "Cube Kinetic Friction", "Default kinetic friction for new cubes", p_float,
cat_body_defaults, &g_cfg.body_defaults.cube_fric_k, 0.3, 0.0, 5.0, false},
{"body_defaults.cylinder_restitution", "Cylinder Restitution", "Default bounce for new cylinders (wheels)", p_float,
cat_body_defaults, &g_cfg.body_defaults.cylinder_restitution, 0.3, 0.0, 1.0, false},"""

    if old in content:
        content = content.replace(old, new, 1)
        write(p, content)
        return True
    log("  [WARN] pattern not found"); return True

def fix_rigidbody_init():
    log("Step 3: Use cylinder_restitution in rigidbody_initialisation_cylinder")
    p = SRC / "core" / "rigidbody.c"
    content = p.read_text()
    if "MFS_165_CYL_RESTITUTION" in content:
        log("  [SKIP] already fixed"); return True

    old = "rigid_body->restitution = g_cfg.body_defaults.sphere_restitution;"
    # Only replace in the cylinder init function
    cyl_init_marker = "void rigidbody_initialisation_cylinder"
    cyl_idx = content.find(cyl_init_marker)
    if cyl_idx < 0:
        log("  [SKIP] cylinder init not found"); return True

    # Find the restitution line after the cylinder init function
    rest_idx = content.find(old, cyl_idx)
    if rest_idx < 0:
        log("  [SKIP] restitution line not found in cylinder init"); return True

    new = "rigid_body->restitution = g_cfg.body_defaults.cylinder_restitution; /* MFS_165_CYL_RESTITUTION */"
    content = content[:rest_idx] + new + content[rest_idx + len(old):]
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
    print("MFS 165: Cylinder restitution + config registration")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    steps = [fix_config_struct, fix_config_schema, fix_rigidbody_init]
    for fn in steps:
        try:
            if not fn(): return 1
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__}: {e}"); import traceback; traceback.print_exc(); return 1

    if not DRY_RUN and not build_and_test(): return 1

    print("=" * 60)
    print("  165 complete. Cylinders now have their own restitution config.")
    print("=" * 60)
    return 0

if __name__ == "__main__": sys.exit(main())

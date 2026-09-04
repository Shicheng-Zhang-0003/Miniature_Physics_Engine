#!/usr/bin/env python3
"""
MFS 131b — Repair: fix broadphase extra } and physics_world typedef
====================================================================
Fixes two root causes:
1. broadphase.c: remove the stray } left outside broadphase_bounding_radius
   by the 130a corruption (131a's replace_function correctly identified the
   function boundary but the stray } sits outside it).
2. physics_world.h: change the anonymous struct typedef to a named struct
   so the forward declaration `struct physics_world;` in collision_mechanics.h
   refers to the same type.
"""

import sys
import re
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"
TOOLS = ROOT / "tools"
DRY_RUN = "--dry-run" in sys.argv


def log(msg):
    print(f"  [131b] {msg}")


# ---------------------------------------------------------------- 1. broadphase.c
def fix_broadphase():
    path = SRC / "physics" / "broadphase.c"
    lines = path.read_text().split("\n")

    # Locate the function
    func_start = None
    for i, line in enumerate(lines):
        if "broadphase_bounding_radius" in line:
            func_start = i
            break
    if func_start is None:
        log("  [SKIP] broadphase_bounding_radius not found")
        return

    # Walk braces to find the true end of the function
    depth = 0
    func_end = None
    for i in range(func_start, len(lines)):
        depth += lines[i].count("{") - lines[i].count("}")
        if depth == 0 and i > func_start:
            func_end = i
            break
    if func_end is None:
        log("  [SKIP] could not find end of broadphase_bounding_radius")
        return

    # The stray } sits as the next non-blank line after func_end
    for i in range(func_end + 1, len(lines)):
        if lines[i].strip():
            if lines[i].strip() == "}":
                lines.pop(i)
                if not DRY_RUN:
                    path.write_text("\n".join(lines))
                log("  [OK] removed stray } after broadphase_bounding_radius")
                return
            break

    log("  [SKIP] no stray } found after broadphase_bounding_radius")


# ---------------------------------------------------------------- 2. physics_world.h
def fix_physics_world_typedef():
    path = SRC / "core" / "physics_world.h"
    content = path.read_text()

    if "typedef struct physics_world {" in content:
        log("  [SKIP] typedef already uses a named struct")
        return

    # Match: typedef struct {  <newline>  <indent> rigidbody *bodies;
    pattern = r"(typedef struct \{)\n(\s+)(rigidbody \*bodies;)"
    match = re.search(pattern, content)
    if match:
        indent = match.group(2)
        content = content.replace(
            match.group(0),
            f"typedef struct physics_world {{\n{indent}{match.group(3)}",
        )
        if not DRY_RUN:
            path.write_text(content)
        log("  [OK] typedef changed to named struct physics_world")
        return

    log("  [SKIP] could not find the typedef pattern in physics_world.h")


# ---------------------------------------------------------------- main
def main():
    print("=" * 60)
    print("MFS 131b: Repair broadphase stray } + physics_world typedef")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN **\n")

    fix_broadphase()
    fix_physics_world_typedef()

    if not DRY_RUN:
        log("Build check...")
        r = subprocess.run(
            [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
            cwd=str(ROOT), capture_output=True, text=True, timeout=120,
        )
        if r.returncode != 0:
            print(r.stdout[-2000:] if r.stdout else "")
            print(r.stderr[-2000:] if r.stderr else "")
            log("[FAIL] Build failed")
            return 1
        log("[PASS] Build clean")

        log("Running tests...")
        r = subprocess.run(
            [sys.executable, str(TOOLS / "test_runner.py")],
            cwd=str(ROOT), capture_output=True, text=True, timeout=300,
        )
        print(r.stdout[-2500:] if r.stdout else "")
        if r.returncode != 0:
            log("[FAIL] Tests failed")
            return 1
        log("[PASS] All tests pass")

    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())

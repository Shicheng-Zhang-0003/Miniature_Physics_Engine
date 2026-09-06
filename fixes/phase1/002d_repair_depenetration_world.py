#!/usr/bin/env python3
"""
MPE Phase 1.2-prep #2d: Repair depenetration_world integration
==============================================================

Fixes three surviving stale integration points:

1. physics_world.c still calls the old 4-argument signature:
       physics_world_depenetration_pass(world, world_pairs, &pair_count, false);
   The new depenetration_world.h declares:
       physics_world_depenetration_pass(world, world_pairs, pair_count);

2. physics_world.c may still include the old depenetration.h include marker.

3. Makefile test source lists may still link physics/depenetration.c,
   which references legacy globals obj_per_scene/object_count.

This script converts the call to the new signature, removes the stale include,
and replaces old depenetration.c links with depenetration_world.c in all
*_SOURCES makefile variables.

Run:
    python3 fixes/phase1/002d_repair_depenetration_world.py
"""

import sys
import re
import subprocess
from pathlib import Path

DRY_RUN = "--dry-run" in sys.argv


def find_root():
    p = Path(__file__).resolve().parent
    while p != p.parent:
        if (p / "v15R3" / "src").exists():
            return p
        p = p.parent
    return None


ROOT = find_root()
if ROOT is None:
    print("FATAL: cannot locate project root containing v15R3/src")
    sys.exit(1)

SRC = ROOT / "v15R3" / "src"

OLD_INCLUDE_PATTERN = re.compile(
    r'^[ \t]*#include "\.\./physics/depenetration\.h"[^\n]*MPE_PHASE1_DEPENETRATION_INCLUDE[^\n]*\n?',
    re.MULTILINE
)

OLD_CALL_PATTERN = re.compile(
    r'physics_world_depenetration_pass\s*\(\s*world\s*,\s*world_pairs\s*,\s*&pair_count\s*,\s*(?:false|true|rebuild_broadphase)\s*\)\s*;'
)

NEW_CALL = "physics_world_depenetration_pass(world, world_pairs, pair_count);"

POSITION_LOOP_PATTERN = re.compile(
    r'(for\s*\(\s*int\s+i\s*=\s*0\s*;\s*i\s*<\s*world->body_count\s*;\s*i\+\+\s*\)\s*\{\s*'
    r'rb_integrate_position\s*\(\s*&world->bodies\[i\]\s*,\s*dt\s*\)\s*;\s*'
    r'rigidbody_sanitize\s*\(\s*&world->bodies\[i\]\s*\)\s*;\s*\})',
    re.DOTALL
)


def write_if_changed(path: Path, old: str, new: str) -> bool:
    if old == new:
        print(f"[SKIP] {path.relative_to(ROOT)} already clean")
        return False

    if DRY_RUN:
        print(f"[DRY] would patch {path.relative_to(ROOT)}")
        return True

    path.write_text(new)
    print(f"[WRITE] {path.relative_to(ROOT)}")
    return True


def patch_physics_world_c():
    path = SRC / "core" / "physics_world.c"
    if not path.exists():
        print("[FAIL] core/physics_world.c not found")
        return False

    content = path.read_text()
    original = content

    # 1. Remove stale old depenetration.h include marker if present.
    content = OLD_INCLUDE_PATTERN.sub("", content)

    # 2. Ensure new depenetration_world.h include exists.
    if '#include "../physics/depenetration_world.h"' not in content:
        anchor = '#include "../config/mpe_constants.h"'
        if anchor not in content:
            print("[FAIL] could not find include anchor for depenetration_world.h")
            return False

        content = content.replace(
            anchor,
            anchor + '\n#include "../physics/depenetration_world.h" /* MPE_PHASE1_DEPENETRATION_WORLD */',
            1
        )
        print("[OK] ensured depenetration_world.h include")

    # 3. Convert any old 4-argument call to the new 3-argument call.
    if OLD_CALL_PATTERN.search(content):
        content = OLD_CALL_PATTERN.sub(NEW_CALL, content)
        print("[OK] converted old 4-argument depenetration call to new signature")

    # 4. If no valid call exists, insert one after the final integrate/sanitize loop.
    if NEW_CALL not in content:
        match = POSITION_LOOP_PATTERN.search(content)
        if not match:
            print("[FAIL] could not find depenetration call or final integration loop")
            return False

        insert = (
            "\n    /* MPE_PHASE1_DEPENETRATION_CALL */\n    "
            + NEW_CALL
        )
        content = content[:match.end()] + insert + content[match.end():]
        print("[OK] inserted new depenetration call after position integration loop")

    # 5. If duplicate calls somehow exist, keep only the first.
    if content.count(NEW_CALL) > 1:
        lines = content.splitlines(keepends=True)
        seen = False
        cleaned = []

        for line in lines:
            if NEW_CALL in line:
                if seen:
                    continue
                seen = True
            cleaned.append(line)

        content = "".join(cleaned)
        print("[OK] removed duplicate depenetration calls")

    return write_if_changed(path, original, content)


def keep_first_token(block: str, token: str) -> str:
    """Keep only the first occurrence of token inside a makefile source block."""
    matches = list(re.finditer(re.escape(token), block))
    if len(matches) <= 1:
        return block

    for match in reversed(matches[1:]):
        start = match.start()
        end = match.end()

        # Remove preceding spaces/tabs so we do not leave double spacing.
        while start > 0 and block[start - 1] in " \t":
            start -= 1

        block = block[:start] + block[end:]

    return block


def process_makefile_block(block: str) -> str:
    original = block

    # Replace old legacy depenetration.c with the new physics_world version.
    if "physics/depenetration.c" in block:
        block = block.replace("physics/depenetration.c", "physics/depenetration_world.c")

    # If this source list links physics_world.c, it must link depenetration_world.c.
    if "core/physics_world.c" in block and "physics/depenetration_world.c" not in block:
        if block.endswith("\n"):
            block = block[:-1].rstrip() + " physics/depenetration_world.c\n"
        else:
            block = block.rstrip() + " physics/depenetration_world.c"

    # Remove duplicate depenetration_world.c entries.
    block = keep_first_token(block, "physics/depenetration_world.c")

    # Tidy accidental double spaces inside source lists.
    block = re.sub(r"[ \t]{2,}", " ", block)

    return block if block != original else original


def patch_makefile():
    path = SRC / "makefile"
    if not path.exists():
        print("[FAIL] makefile not found")
        return False

    content = path.read_text()
    original = content

    lines = content.splitlines(keepends=True)
    out = []
    i = 0

    source_var_pattern = re.compile(r"^[A-Za-z0-9_]+_SOURCES\s*[:+]?=")

    while i < len(lines):
        line = lines[i]

        if source_var_pattern.match(line):
            block = line

            # Consume backslash-continuation lines.
            while block.rstrip("\n").endswith("\\") and i + 1 < len(lines):
                i += 1
                block += lines[i]

            new_block = process_makefile_block(block)
            out.append(new_block)
        else:
            out.append(line)

        i += 1

    content = "".join(out)
    return write_if_changed(path, original, content)


def build_and_test():
    print("\n[RUN] make test_kernel_stability")
    rc = subprocess.call(["make", "test_kernel_stability"], cwd=str(SRC))
    if rc != 0:
        print("[FAIL] test_kernel_stability failed")
        return False

    print("\n[RUN] make test_sleep_wake_contact")
    rc = subprocess.call(["make", "test_sleep_wake_contact"], cwd=str(SRC))
    if rc != 0:
        print("[FAIL] test_sleep_wake_contact failed")
        return False

    print("\n[RUN] make unit_tests")
    rc = subprocess.call(["make", "unit_tests"], cwd=str(SRC))
    if rc != 0:
        print("[FAIL] unit_tests failed")
        return False

    return True


def main():
    print("=" * 60)
    print("MPE Phase 1.2-prep #2d: Repair depenetration_world wiring")
    print("=" * 60)

    if DRY_RUN:
        print("  ** DRY RUN **\n")

    ok = patch_physics_world_c()
    if not ok:
        return 1

    ok = patch_makefile()
    if not ok:
        return 1

    if DRY_RUN:
        print("\n[DRY] no build executed")
        return 0

    if not build_and_test():
        return 1

    print("\n[PASS] depenetration_world integration repaired and verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())

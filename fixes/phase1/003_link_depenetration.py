#!/usr/bin/env python3
"""
MPE Phase 1.2-prep #3: Link depenetration.c into physics_world test targets
============================================================================
physics_world.c now calls physics_world_depenetration_pass (defined in
physics/depenetration.c). Every test target that links core/physics_world.c
must also link physics/depenetration.c, or the linker fails with:
    undefined reference to `physics_world_depenetration_pass'

This script inserts physics/depenetration.c into every test source list
that ends with the common tail `physics/revolute_joint.c config/mpe_config.c`.
Idempotent: once depenetration.c is present, the replace is a no-op.

Run: python3 fixes/phase1/003_link_depenetration.py
"""
import sys
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

MAKEFILE = ROOT / "v15R3" / "src" / "makefile"

# The common tail shared by every test target that links physics_world.c.
# Inserting depenetration.c here covers kernel_stability, sleep_wake_contact,
# and all cylinder / two_world / revolute targets in one idempotent pass.
FIND    = "physics/revolute_joint.c config/mpe_config.c"
REPLACE = "physics/revolute_joint.c physics/depenetration.c config/mpe_config.c"

def main():
    content = MAKEFILE.read_text()

    count = content.count(FIND)
    if count == 0:
        if "physics/depenetration.c config/mpe_config.c" in content:
            print("[SKIP] depenetration.c already linked into all test targets")
            return 0
        print("[FAIL] could not locate the test-source tail to patch")
        return 1

    if DRY_RUN:
        print(f"[DRY] would patch {count} test source list(s)")
        return 0

    content = content.replace(FIND, REPLACE)
    MAKEFILE.write_text(content)
    print(f"[OK] added physics/depenetration.c to {count} test source list(s)")
    return 0

if __name__ == "__main__":
    sys.exit(main())

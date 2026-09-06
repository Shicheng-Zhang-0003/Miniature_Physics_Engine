#!/usr/bin/env python3
"""
MPE Phase 1.2-prep #2: Depenetration pass for physics_world
============================================================
Adds physics_world_depenetration_pass to depenetration.c and wires it
into physics_world_step after position integration.

Corrected from previous attempt: uses regex for anchor matching instead
of exact whitespace matching.

Idempotent. Run: python3 fixes/phase1/002_depenetration_pass.py
"""
import sys, re, subprocess
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

MARKER_FUNC = "MPE_PHASE1_DEPENETRATION_PASS"
MARKER_CALL = "MPE_PHASE1_DEPENETRATION_CALL"
MARKER_INCLUDE = "MPE_PHASE1_DEPENETRATION_INCLUDE"

# ── New function to append to depenetration.c ──────────────────────
DEPENETRATION_FUNC = '''
/* MPE_PHASE1_DEPENETRATION_PASS: physics_world positional depenetration.
* Standalone extraction from the legacy a3_positional_depenetration_pass.
* Resolves residual penetration between body pairs after the impulse solve.
* Mirrors the legacy implementation but operates on a physics_world. */
void physics_world_depenetration_pass(physics_world *world, broadphase_pair *pair_buffer,
                                       int *pair_count_pointer, bool rebuild_broadphase) {
    if ((!world) || (!world->bodies) || (world->body_count < 2) ||
        (!pair_buffer) || (!pair_count_pointer)) {
        return;
    }
    int pair_count = *pair_count_pointer;
    if (rebuild_broadphase) {
        pair_count = broadphase_generate_pairing(world->bodies, world->body_count,
                                                  pair_buffer, mpe_max_broadphase_pairs);
        *pair_count_pointer = pair_count;
    }
    int depenetration_iterations = rebuild_broadphase ? g_cfg.depenetration.rebuild_iterations : 1;
    for (int dep_iteration = 0; dep_iteration < depenetration_iterations; dep_iteration++) {
        for (int pair_index = 0; pair_index < pair_count; pair_index++) {
            int index_a = pair_buffer[pair_index].object_index_a;
            int index_b = pair_buffer[pair_index].object_index_b;
            if ((index_a < 0) || (index_a >= world->body_count)) { continue; }
            if ((index_b < 0) || (index_b >= world->body_count)) { continue; }
            rigidbody *body_a = &world->bodies[index_a];
            rigidbody *body_b = &world->bodies[index_b];
            collision_data depenetration_collision = {0};
            if (a3_depenetration_dispatch(body_a, body_b, &depenetration_collision)) {
                a3_positional_depenetrate_manifold(&depenetration_collision);
            }
        }
        for (int object_index = 0; object_index < world->body_count; object_index++) {
            rigidbody *rigid_body = &world->bodies[object_index];
            if (rigid_body->static_state) { continue; }
            collision_data floor_collision = {0};
            if (collision_static_plane_body(rigid_body, 0.0f, &floor_collision)) {
                a3_positional_depenetrate_manifold(&floor_collision);
            }
        }
    }
}
'''

# ── Declaration to add to depenetration.h ──────────────────────────
DEPENETRATION_DECL = '''
/* MPE_PHASE1_DEPENETRATION_PASS */
struct physics_world;
void physics_world_depenetration_pass(struct physics_world *world, broadphase_pair *pair_buffer,
                                       int *pair_count_pointer, bool rebuild_broadphase);
'''


def patch_depenetration_c():
    path = SRC / "physics" / "depenetration.c"
    content = path.read_text()
    if MARKER_FUNC in content:
        print("[SKIP] depenetration function already present")
        return
    content += DEPENETRATION_FUNC
    if not DRY_RUN:
        path.write_text(content)
    print("[OK] added physics_world_depenetration_pass to depenetration.c")


def patch_depenetration_h():
    path = SRC / "physics" / "depenetration.h"
    content = path.read_text()
    if MARKER_FUNC in content:
        print("[SKIP] declaration already present")
        return
    anchor = "#endif"
    if anchor not in content:
        print("[FAIL] #endif not found in depenetration.h")
        sys.exit(1)
    content = content.replace(anchor, DEPENETRATION_DECL + "\n" + anchor, 1)
    if not DRY_RUN:
        path.write_text(content)
    print("[OK] added declaration to depenetration.h")


def patch_physics_world():
    path = SRC / "core" / "physics_world.c"
    content = path.read_text()
    changed = False

    # 1. Add include if not present
    if MARKER_INCLUDE not in content:
        anchor = '#include "../config/mpe_constants.h"'
        if anchor not in content:
            print("[FAIL] mpe_constants.h include not found in physics_world.c")
            sys.exit(1)
        content = content.replace(
            anchor,
            anchor + '\n#include "../physics/depenetration.h" /* MPE_PHASE1_DEPENETRATION_INCLUDE */',
            1
        )
        changed = True
        print("[OK] added depenetration.h include")
    else:
        print("[SKIP] depenetration.h include already present")

    # 2. Insert depenetration call after position integration loop
    # Use regex for robust whitespace matching
    if MARKER_CALL not in content:
        pattern = re.compile(
            r'(\s*rb_integrate_position\(&world->bodies\[i\], dt\);\s*\n'
            r'\s*rigidbody_sanitize\(&world->bodies\[i\]\);\s*\n'
            r'\s*\})\s*\n'
            r'\}',
            re.MULTILINE
        )
        match = pattern.search(content)
        if not match:
            print("[FAIL] position integration loop not found in physics_world.c")
            print("       Try: grep -n 'rb_integrate_position' core/physics_world.c")
            sys.exit(1)

        replacement = (
            match.group(1) + '\n'
            '    /* MPE_PHASE1_DEPENETRATION_CALL */\n'
            '    physics_world_depenetration_pass(world, world_pairs, &pair_count, false);\n'
            '}'
        )
        content = content[:match.start()] + replacement + content[match.end():]
        changed = True
        print("[OK] inserted depenetration call into physics_world_step")
    else:
        print("[SKIP] depenetration call already present")

    if changed and not DRY_RUN:
        path.write_text(content)


def main():
    print("=" * 60)
    print("MPE Phase 1.2-prep #2: Depenetration pass")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN **\n")

    patch_depenetration_c()
    patch_depenetration_h()
    patch_physics_world()

    if not DRY_RUN:
        print("\n[RUN] building to verify compilation")
        rc = subprocess.call(["make", "-j4"], cwd=str(SRC))
        if rc != 0:
            print("[FAIL] build failed")
            sys.exit(1)
        print("[PASS] build succeeded")

        print("\n[RUN] running unit tests to verify no regressions")
        rc = subprocess.call(["make", "unit_tests"], cwd=str(SRC))
        if rc != 0:
            print("[FAIL] unit tests failed")
            sys.exit(1)

        print("\n[RUN] running kernel stability test")
        rc = subprocess.call(["make", "test_kernel_stability"], cwd=str(SRC))
        if rc != 0:
            print("[FAIL] kernel stability test failed")
            sys.exit(1)

        print("\n[PASS] depenetration pass verified")


if __name__ == "__main__":
    main()

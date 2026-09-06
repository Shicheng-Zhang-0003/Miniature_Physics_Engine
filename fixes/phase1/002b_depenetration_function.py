#!/usr/bin/env python3
"""
MPE Phase 1.2-prep #2b: Add physics_world_depenetration_pass
=============================================================
Adds a physics_world-aware depenetration function to depenetration.c
that doesn't depend on global obj_per_scene/object_count.
"""
import sys, re
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
SRC = ROOT / "v15R3" / "src"

MARKER = "MPE_PHASE1_DEPENETRATION_PASS"

NEW_FUNCTION = '''
/* MPE_PHASE1_DEPENETRATION_PASS: physics_world-aware depenetration.
* Standalone extraction that doesn't depend on global obj_per_scene.
* Resolves residual penetration between body pairs after the impulse solve. */
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

HEADER_DECL = '''
/* MPE_PHASE1_DEPENETRATION_PASS */
struct physics_world;
void physics_world_depenetration_pass(struct physics_world *world, broadphase_pair *pair_buffer,
                                       int *pair_count_pointer, bool rebuild_broadphase);
'''

def patch_depenetration_c():
    path = SRC / "physics" / "depenetration.c"
    content = path.read_text()
    if MARKER in content:
        print("[SKIP] function already present")
        return
    # Append at end of file
    content += NEW_FUNCTION
    if not DRY_RUN:
        path.write_text(content)
    print("[OK] added physics_world_depenetration_pass to depenetration.c")

def patch_depenetration_h():
    path = SRC / "physics" / "depenetration.h"
    content = path.read_text()
    if MARKER in content:
        print("[SKIP] declaration already present")
        return
    # Insert before #endif
    anchor = "#endif"
    if anchor not in content:
        print("[FAIL] #endif not found")
        return
    content = content.replace(anchor, HEADER_DECL + "\n" + anchor, 1)
    if not DRY_RUN:
        path.write_text(content)
    print("[OK] added declaration to depenetration.h")

def patch_physics_world_call():
    path = SRC / "core" / "physics_world.c"
    content = path.read_text()
    call_marker = "MPE_PHASE1_DEPENETRATION_CALL"
    if call_marker in content:
        print("[SKIP] call already present")
        return
    # Find the last rigidbody_sanitize call in the position integration loop
    # and insert the depenetration call after the loop closes
    pattern = re.compile(
        r'(for\s*\(\s*int\s+i\s*=\s*0\s*;\s*i\s*<\s*world->body_count\s*;\s*i\+\+\s*\)\s*\{\s*'
        r'rb_integrate_position\s*\(\s*&world->bodies\[i\]\s*,\s*dt\s*\)\s*;\s*'
        r'rigidbody_sanitize\s*\(\s*&world->bodies\[i\]\s*\)\s*;\s*\})',
        re.DOTALL
    )
    match = pattern.search(content)
    if not match:
        print("[FAIL] position integration loop not found")
        return
    insert_pos = match.end()
    call_code = '\n    /* MPE_PHASE1_DEPENETRATION_CALL */\n    physics_world_depenetration_pass(world, world_pairs, &pair_count, false);\n'
    content = content[:insert_pos] + call_code + content[insert_pos:]
    if not DRY_RUN:
        path.write_text(content)
    print("[OK] added depenetration call to physics_world_step")

def main():
    print("=" * 60)
    print("MPE Phase 1.2-prep #2b: physics_world depenetration")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN **\n")
    patch_depenetration_c()
    patch_depenetration_h()
    patch_physics_world_call()
    if not DRY_RUN:
        print("\n[RUN] rebuilding test_kernel_stability")
        import subprocess
        rc = subprocess.call(["make", "test_kernel_stability"], cwd=str(SRC))
        if rc != 0:
            print("[FAIL] build failed")
            sys.exit(1)
        print("[PASS] build succeeded")

if __name__ == "__main__":
    main()

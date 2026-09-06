#!/usr/bin/env python3
"""
MPE Phase 1.2-prep #2: Brand new physics_world-compatible depenetration pass.
Creates physics/depenetration_world.h and physics/depenetration_world.c.
Wires it into physics_world_step() and updates the Makefile test targets.
Leaves the old depenetration.c completely untouched.
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
PHYSICS_DIR = SRC / "physics"

HEADER_CODE = """#ifndef mpe_depenetration_world_h
#define mpe_depenetration_world_h

#include "../core/physics_world.h"
#include "broadphase.h"

/* MPE_PHASE1_DEPENETRATION_WORLD:
 * Pure physics_world depenetration pass.
 * Operates entirely on the provided physics_world and broadphase pairs.
 * Eliminates the hardcoded y=0 floor plane pass (the floor is just a static body).
 */
void physics_world_depenetration_pass(physics_world *world, broadphase_pair *pairs, int pair_count);

#endif /* mpe_depenetration_world_h */
"""

SOURCE_CODE = r"""/* MPE_PHASE1_DEPENETRATION_WORLD:
 * Pure physics_world depenetration pass.
 * Replaces the legacy a3_positional_depenetration_pass which relied on
 * global obj_per_scene and a hardcoded y=0 floor plane.
 */
#include "depenetration_world.h"
#include "collision_mechanics.h"
#include "../config/mpe_config.h"
#include <math.h>
#include <stdbool.h>

/* ------------------------------------------------------------------
 * Narrowphase dispatch for depenetration
 * ------------------------------------------------------------------ */
static bool depenetration_dispatch(rigidbody *a, rigidbody *b, collision_data *out) {
    if (a->type == object_sphere && b->type == object_sphere) {
        return collision_dual_sphere(a, b, out);
    }
    if (a->type == object_sphere && b->type == object_cube) {
        return collision_sphere_cube(a, b, out);
    }
    if (a->type == object_cube && b->type == object_sphere) {
        bool hit = collision_sphere_cube(b, a, out);
        if (hit) {
            out->normal_vector = vector3_scaling(out->normal_vector, -1.0f);
            out->object_a = a;
            out->object_b = b;
        }
        return hit;
    }
    if (a->type == object_cube && b->type == object_cube) {
        return collision_dual_cube(a, b, out);
    }
    /* Cylinder variants */
    if (a->type == object_cylinder && b->type == object_sphere) {
        return collision_cylinder_sphere(a, b, out);
    }
    if (a->type == object_sphere && b->type == object_cylinder) {
        bool hit = collision_cylinder_sphere(b, a, out);
        if (hit) {
            out->normal_vector = vector3_scaling(out->normal_vector, -1.0f);
            out->object_a = a;
            out->object_b = b;
        }
        return hit;
    }
    if (a->type == object_cylinder && b->type == object_cube) {
        return collision_cylinder_cube(a, b, out);
    }
    if (a->type == object_cube && b->type == object_cylinder) {
        bool hit = collision_cylinder_cube(b, a, out);
        if (hit) {
            out->normal_vector = vector3_scaling(out->normal_vector, -1.0f);
            out->object_a = a;
            out->object_b = b;
        }
        return hit;
    }
    if (a->type == object_cylinder && b->type == object_cylinder) {
        return collision_cylinder_cylinder(a, b, out);
    }
    return false;
}

/* ------------------------------------------------------------------
 * Manifold resolution
 * ------------------------------------------------------------------ */
static void resolve_manifold(collision_data *manifold) {
    if (!manifold || manifold->contact_count <= 0) return;

    rigidbody *a = manifold->object_a;
    rigidbody *b = manifold->object_b;
    if (!a || !b) return;

    float normal_len_sq = vector3_length_squared(manifold->normal_vector);
    if (!isfinite(normal_len_sq) || normal_len_sq < 0.000001f) return;

    float max_depth = 0.0f;
    float depth_sum = 0.0f;
    int depth_count = 0;
    float slop = g_cfg.depenetration.penetration_slop;

    for (int i = 0; i < manifold->contact_count; i++) {
        float depth = manifold->contacts[i].penetration;
        if (depth > max_depth) max_depth = depth;
        if (depth > slop) {
            depth_sum += depth;
            depth_count++;
        }
    }

    if (max_depth <= 0.0005f) return;

    bool a_sleep = a->is_sleeping && !a->static_state;
    bool b_sleep = b->is_sleeping && !b->static_state;
    float wake_thresh = g_cfg.depenetration.wake_depth_thresh;

    /* Wake sleeping bodies if penetration is significant */
    if (a_sleep && b_sleep && max_depth > wake_thresh) {
        rigidbody_wake(a);
        rigidbody_wake(b);
        a_sleep = false;
        b_sleep = false;
    }
    if (a_sleep && b->static_state && max_depth > wake_thresh) {
        rigidbody_wake(a);
        a_sleep = false;
    }
    if (b_sleep && a->static_state && max_depth > wake_thresh) {
        rigidbody_wake(b);
        b_sleep = false;
    }

    float inv_mass_a = (a->static_state || a_sleep) ? 0.0f : a->inverse_mass;
    float inv_mass_b = (b->static_state || b_sleep) ? 0.0f : b->inverse_mass;
    float inv_mass_sum = inv_mass_a + inv_mass_b;

    if (inv_mass_sum <= 0.0f) return;

    if (depth_count == 0) {
        depth_sum = max_depth;
        depth_count = 1;
    }

    float avg_depth = depth_sum / (float)depth_count;
    float correction_mag = (avg_depth - slop) * g_cfg.depenetration.correction_factor / inv_mass_sum;

    if (correction_mag <= 0.0f) return;
    if (correction_mag > g_cfg.depenetration.max_correction) {
        correction_mag = g_cfg.depenetration.max_correction;
    }

    vector3 correction = vector3_scaling(manifold->normal_vector, correction_mag);

    if (inv_mass_a > 0.0f) {
        a->position = vector3_subtraction(a->position, vector3_scaling(correction, inv_mass_a));
        if (correction_mag > 0.01f) rigidbody_wake(a);
    }
    if (inv_mass_b > 0.0f) {
        b->position = vector3_addition(b->position, vector3_scaling(correction, inv_mass_b));
        if (correction_mag > 0.01f) rigidbody_wake(b);
    }
}

/* ------------------------------------------------------------------
 * Main pass
 * ------------------------------------------------------------------ */
void physics_world_depenetration_pass(physics_world *world, broadphase_pair *pairs, int pair_count) {
    if (!world || !world->bodies || world->body_count < 2 || !pairs || pair_count <= 0) return;

    int iterations = g_cfg.depenetration.rebuild_iterations;
    if (iterations < 1) iterations = 1;

    for (int iter = 0; iter < iterations; iter++) {
        for (int i = 0; i < pair_count; i++) {
            int idx_a = pairs[i].object_index_a;
            int idx_b = pairs[i].object_index_b;

            if (idx_a < 0 || idx_a >= world->body_count) continue;
            if (idx_b < 0 || idx_b >= world->body_count) continue;

            rigidbody *a = &world->bodies[idx_a];
            rigidbody *b = &world->bodies[idx_b];

            /* Skip if both are static or sleeping */
            if ((a->static_state || a->is_sleeping) && (b->static_state || b->is_sleeping)) continue;

            collision_data manifold = {0};
            if (depenetration_dispatch(a, b, &manifold)) {
                resolve_manifold(&manifold);
            }
        }
    }
}
"""

def write_file(path: Path, content: str):
    if DRY_RUN:
        print(f"[DRY] Would write {path.relative_to(ROOT)} ({len(content)} bytes)")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    print(f"[WRITE] {path.relative_to(ROOT)}")

def patch_physics_world_c():
    pw = SRC / "core" / "physics_world.c"
    content = pw.read_text()
    changed = False

    if "depenetration_world.h" not in content:
        anchor = '#include "../config/mpe_constants.h"'
        if anchor in content:
            content = content.replace(anchor, anchor + '\n#include "../physics/depenetration_world.h" /* MPE_PHASE1_DEPENETRATION_WORLD */', 1)
            changed = True
            print("[OK] Added depenetration_world.h include to physics_world.c")

    marker = "MPE_PHASE1_DEPENETRATION_CALL"
    if marker not in content:
        pattern = r"(for\s*\(\s*int\s+i\s*=\s*0;\s*i\s*<\s*world->body_count;\s*i\+\+\s*\)\s*\{\s*rb_integrate_position\(&world->bodies\[i\],\s*dt\);\s*rigidbody_sanitize\(&world->bodies\[i\]\);\s*\})"
        match = re.search(pattern, content)
        if match:
            insert_pos = match.end()
            call_code = f"\n    /* {marker} */\n    physics_world_depenetration_pass(world, world_pairs, pair_count);\n"
            content = content[:insert_pos] + call_code + content[insert_pos:]
            changed = True
            print("[OK] Wired physics_world_depenetration_pass into physics_world_step")

    if changed and not DRY_RUN:
        pw.write_text(content)

def patch_makefile():
    mk = SRC / "makefile"
    content = mk.read_text()
    changed = False

    if "physics/depenetration_world.c" not in content:
        pattern = r"(MPE_PHASE0_KERNEL_STABILITY_SOURCES\s*:=.*?config/mpe_config_schema\.c)"
        match = re.search(pattern, content, re.DOTALL)
        if match:
            old_str = match.group(1)
            new_str = old_str + " physics/depenetration_world.c"
            content = content.replace(old_str, new_str, 1)
            changed = True
            print("[OK] Added depenetration_world.c to KERNEL_STABILITY_SOURCES")

        pattern2 = r"(WAKE_CONTACT_SOURCES\s*:=.*?config/mpe_config_schema\.c)"
        match2 = re.search(pattern2, content, re.DOTALL)
        if match2:
            old_str2 = match2.group(1)
            new_str2 = old_str2 + " physics/depenetration_world.c"
            content = content.replace(old_str2, new_str2, 1)
            changed = True
            print("[OK] Added depenetration_world.c to WAKE_CONTACT_SOURCES")

    if changed and not DRY_RUN:
        mk.write_text(content)

def main():
    print("=" * 60)
    print("MPE Phase 1.2-prep #2: physics_world depenetration pass")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN **\n")

    write_file(PHYSICS_DIR / "depenetration_world.h", HEADER_CODE)
    write_file(PHYSICS_DIR / "depenetration_world.c", SOURCE_CODE)
    patch_physics_world_c()
    patch_makefile()

    if not DRY_RUN:
        print("\n[RUN] Building and testing...")
        rc = subprocess.call(["make", "test_kernel_stability"], cwd=str(SRC))
        if rc != 0:
            print("[FAIL] test_kernel_stability failed")
            return 1
        print("[PASS] test_kernel_stability passed")

    print("\n" + "=" * 60)
    print("Phase 1.2-prep #2 complete.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())

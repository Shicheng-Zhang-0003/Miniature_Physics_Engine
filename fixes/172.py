#!/usr/bin/env python3
"""
MFS 172: Cylinder-vs-object narrowphase functions
==================================================
Adds three new collision functions to collision_mechanics.c:
  1. collision_cylinder_sphere  – axle-segment vs sphere
  2. collision_cylinder_cube    – axle-segment vs OBB (sampled)
  3. collision_cylinder_cylinder – segment-segment closest points

Also adds declarations to collision_mechanics.h.

These are pure math functions. Dispatch wiring is script 173.

Usage:
    cd <project_root>
    python3 fixes/172_cylinder_narrowphase.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [172] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.relative_to(SRC)}")

# ── the three functions to append to collision_mechanics.c ──

CYLINDER_FUNCTIONS = r'''
/* ================================================================
 * MFS_172: Cylinder-vs-object narrowphase
 * ================================================================ */

/* Cylinder vs Sphere.
 * The cylinder is modelled as its axle segment [E1,E2] with radius r_c.
 * Find the closest point on the segment to the sphere centre, then
 * do a sphere-sphere test at that point. */
bool collision_cylinder_sphere(rigidbody *cyl, rigidbody *sph,
                               collision_data *out) {
    if ((cyl->type != object_cylinder) || (sph->type != object_sphere)) {
        return false;
    }
    vector3 axis = cyl->cached_axes[0];
    float r_c = cyl->radius;
    float h   = cyl->cylinder_half_length;
    float r_s = sph->radius;

    vector3 e1 = vector3_subtraction(cyl->position, vector3_scaling(axis, h));
    vector3 e2 = vector3_addition(cyl->position, vector3_scaling(axis, h));

    /* closest point on segment [e1,e2] to sphere centre */
    vector3 seg = vector3_subtraction(e2, e1);
    float seg_len_sq = vector3_length_squared(seg);
    float t = 0.0f;
    if (seg_len_sq > 0.000001f) {
        t = vector3_dot(vector3_subtraction(sph->position, e1), seg) / seg_len_sq;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
    }
    vector3 closest = vector3_addition(e1, vector3_scaling(seg, t));

    float dist = vector3_length(vector3_subtraction(sph->position, closest));
    float min_dist = r_c + r_s;
    if (dist >= min_dist) return false;

    out->object_a = cyl;
    out->object_b = sph;
    out->contact_count = 1;

    if (dist > 0.0001f) {
        out->normal_vector = vector3_scaling(
            vector3_subtraction(sph->position, closest), 1.0f / dist);
    } else {
        out->normal_vector = (vector3){0.0f, 1.0f, 0.0f};
    }
    contact_point_data *cp = &out->contacts[0];
    cp->penetration = min_dist - dist;
    cp->position = vector3_addition(closest,
        vector3_scaling(out->normal_vector, r_c));
    return true;
}

/* Cylinder vs Cube (OBB).
 * Sample N points along the axle, find the one closest to the OBB
 * surface, then do a sphere-OBB test at that point with the
 * cylinder radius. 5 samples is enough for short axles (wheels). */
bool collision_cylinder_cube(rigidbody *cyl, rigidbody *cube,
                             collision_data *out) {
    if ((cyl->type != object_cylinder) || (cube->type != object_cube)) {
        return false;
    }
    vector3 axis = cyl->cached_axes[0];
    float r = cyl->radius;
    float h = cyl->cylinder_half_length;

    vector3 e1 = vector3_subtraction(cyl->position, vector3_scaling(axis, h));
    vector3 e2 = vector3_addition(cyl->position, vector3_scaling(axis, h));

    const int SAMPLES = 5;
    float best_dist = 1e30f;
    vector3 best_on_obb = cube->position;

    for (int s = 0; s <= SAMPLES; s++) {
        float t = (float)s / (float)SAMPLES;
        vector3 pt = vector3_addition(e1,
            vector3_scaling(vector3_subtraction(e2, e1), t));

        /* project into OBB local space */
        vector3 rel = vector3_subtraction(pt, cube->position);
        vector3 *axes = cube->cached_axes;
        vector3 local = {
            vector3_dot(rel, axes[0]),
            vector3_dot(rel, axes[1]),
            vector3_dot(rel, axes[2])
        };
        vector3 clamped = {
            fmaxf(-cube->half_extensions.x, fminf(cube->half_extensions.x, local.x)),
            fmaxf(-cube->half_extensions.y, fminf(cube->half_extensions.y, local.y)),
            fmaxf(-cube->half_extensions.z, fminf(cube->half_extensions.z, local.z))
        };
        vector3 on_obb = cube->position;
        on_obb = vector3_addition(on_obb, vector3_scaling(axes[0], clamped.x));
        on_obb = vector3_addition(on_obb, vector3_scaling(axes[1], clamped.y));
        on_obb = vector3_addition(on_obb, vector3_scaling(axes[2], clamped.z));

        float d = vector3_length(vector3_subtraction(pt, on_obb));
        if (d < best_dist) {
            best_dist = d;
            best_on_obb = on_obb;
        }
    }

    if (best_dist >= r) return false;

    out->object_a = cyl;
    out->object_b = cube;
    out->contact_count = 1;

    if (best_dist > 0.0001f) {
        out->normal_vector = vector3_scaling(
            vector3_subtraction(cyl->position, best_on_obb),
            1.0f / vector3_length(vector3_subtraction(cyl->position, best_on_obb)));
    } else {
        out->normal_vector = (vector3){0.0f, 1.0f, 0.0f};
    }
    contact_point_data *cp = &out->contacts[0];
    cp->penetration = r - best_dist;
    cp->position = best_on_obb;
    return true;
}

/* Cylinder vs Cylinder.
 * Segment-segment closest points, then sphere-sphere at those
 * points with respective radii. */
bool collision_cylinder_cylinder(rigidbody *cyl_a, rigidbody *cyl_b,
                                 collision_data *out) {
    if ((cyl_a->type != object_cylinder) || (cyl_b->type != object_cylinder)) {
        return false;
    }
    vector3 ax = cyl_a->cached_axes[0];
    vector3 bx = cyl_b->cached_axes[0];
    float ha = cyl_a->cylinder_half_length;
    float hb = cyl_b->cylinder_half_length;

    vector3 a1 = vector3_subtraction(cyl_a->position, vector3_scaling(ax, ha));
    vector3 a2 = vector3_addition(cyl_a->position, vector3_scaling(ax, ha));
    vector3 b1 = vector3_subtraction(cyl_b->position, vector3_scaling(bx, hb));
    vector3 b2 = vector3_addition(cyl_b->position, vector3_scaling(bx, hb));

    /* segment-segment closest points (Ericson, Real-Time Collision Detection) */
    vector3 d1 = vector3_subtraction(a2, a1);
    vector3 d2 = vector3_subtraction(b2, b1);
    vector3 r  = vector3_subtraction(a1, b1);
    float a = vector3_dot(d1, d1);
    float e = vector3_dot(d2, d2);
    float f = vector3_dot(d2, r);
    float s, t;

    if ((a <= 0.000001f) && (e <= 0.000001f)) {
        s = t = 0.0f;
    } else if (a <= 0.000001f) {
        s = 0.0f;
        t = f / e;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
    } else {
        float c = vector3_dot(d1, r);
        if (e <= 0.000001f) {
            t = 0.0f;
            s = -c / a;
            if (s < 0.0f) s = 0.0f;
            if (s > 1.0f) s = 1.0f;
        } else {
            float b = vector3_dot(d1, d2);
            float denom = a * e - b * b;
            s = (denom > 0.000001f) ? (b * f - c * e) / denom : 0.0f;
            if (s < 0.0f) s = 0.0f;
            if (s > 1.0f) s = 1.0f;
            t = (b * s + f) / e;
            if (t < 0.0f) { t = 0.0f; s = -c / a; }
            if (t > 1.0f) { t = 1.0f; s = (b - c) / a; }
            if (s < 0.0f) s = 0.0f;
            if (s > 1.0f) s = 1.0f;
        }
    }

    vector3 pa = vector3_addition(a1, vector3_scaling(d1, s));
    vector3 pb = vector3_addition(b1, vector3_scaling(d2, t));
    float dist = vector3_length(vector3_subtraction(pa, pb));
    float min_dist = cyl_a->radius + cyl_b->radius;
    if (dist >= min_dist) return false;

    out->object_a = cyl_a;
    out->object_b = cyl_b;
    out->contact_count = 1;
    if (dist > 0.0001f) {
        out->normal_vector = vector3_scaling(
            vector3_subtraction(pb, pa), 1.0f / dist);
    } else {
        out->normal_vector = (vector3){0.0f, 1.0f, 0.0f};
    }
    contact_point_data *cp = &out->contacts[0];
    cp->penetration = min_dist - dist;
    cp->position = vector3_scaling(vector3_addition(pa, pb), 0.5f);
    return true;
}
'''

# ── header declarations ──

HEADER_DECLS = r'''/* MFS_172: cylinder-vs-object narrowphase */
bool collision_cylinder_sphere(rigidbody *cyl, rigidbody *sph,
                               collision_data *out);
bool collision_cylinder_cube(rigidbody *cyl, rigidbody *cube,
                             collision_data *out);
bool collision_cylinder_cylinder(rigidbody *cyl_a, rigidbody *cyl_b,
                                 collision_data *out);
'''

# ── steps ──

def step_add_functions():
    log("Step 1: Appending cylinder narrowphase functions to collision_mechanics.c")
    p = SRC / "physics" / "collision_mechanics.c"
    content = p.read_text()
    if "MFS_172" in content:
        log("  [SKIP] already present")
        return True
    content += CYLINDER_FUNCTIONS
    write(p, content)
    return True

def step_add_declarations():
    log("Step 2: Adding declarations to collision_mechanics.h")
    p = SRC / "physics" / "collision_mechanics.h"
    content = p.read_text()
    if "collision_cylinder_sphere" in content:
        log("  [SKIP] already present")
        return True
    # insert before the #endif
    anchor = "#endif"
    idx = content.rfind(anchor)
    if idx < 0:
        log("  [FAIL] #endif not found")
        return False
    content = content[:idx] + HEADER_DECLS + "\n" + content[idx:]
    write(p, content)
    return True

def step_build():
    log("Step 3: Build check")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=180)
    if r.returncode != 0:
        print(r.stdout[-2000:] if r.stdout else "")
        print(r.stderr[-2000:] if r.stderr else "")
        log("[FAIL] build failed")
        return False
    log("[PASS] build clean")
    return True

def main():
    print("=" * 60)
    print("MFS 172: Cylinder-vs-object narrowphase functions")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")
    steps = [step_add_functions, step_add_declarations]
    for fn in steps:
        try:
            if not fn(): return 1
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__}: {e}")
            import traceback; traceback.print_exc()
            return 1
    if not DRY_RUN:
        if not step_build(): return 1
    print("=" * 60)
    print("  172 complete. Three cylinder narrowphase functions added.")
    print("  Next: run 173 to wire them into the dispatch.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())

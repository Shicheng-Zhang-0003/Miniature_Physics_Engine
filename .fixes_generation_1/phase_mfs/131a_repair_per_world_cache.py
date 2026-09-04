#!/usr/bin/env python3
"""
MFS 131a — Repair: per-world contact cache done properly
=========================================================
Repairs the 130a broadphase corruption and reworks 131 correctly:

  1. broadphase.c      — rewrite broadphase_bounding_radius cleanly
  2. physics_world.h   — cached_contact typedef moved here; cache is a
                         heap pointer (not a 3 MB inline array)
  3. collision_mechanics.c — remove private typedef; rewrite
                         contact_cache_save / contact_cache_clear with
                         real per-world routing + NULL global fallback
  4. physics_world.c   — init allocates cache, cleanup frees it,
                         clear resets it
  5. simulation_physics_loop.c — restore legacy warm-start save via NULL
  6. gui_robot_registry.c — drop bogus contact_cache_save(world, NULL, 0)
  7. all *.c           — contact_cache_clear() -> contact_cache_clear(NULL)
  8. FTC_PLAN_MANIFEST — mark item 101 done (only after green build+tests)

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/131a_repair_per_world_cache.py [--dry-run]
"""
import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
DRY_RUN = "--dry-run" in sys.argv


def log(msg):
    print(f"  [131a] {msg}")


def read(p):
    return p.read_text()


def write(p, t):
    if not DRY_RUN:
        p.write_text(t)


def lines_of(p):
    return read(p).splitlines(keepends=True)


def replace_function(lines, sig_marker, new_block):
    """Replace a whole function (sig line .. matching closing brace)."""
    start = None
    for i, ln in enumerate(lines):
        if sig_marker in ln:
            start = i
            break
    if start is None:
        return None
    depth = 0
    opened = False
    for i in range(start, len(lines)):
        depth += lines[i].count("{") - lines[i].count("}")
        if "{" in lines[i]:
            opened = True
        if opened and depth <= 0:
            return lines[:start] + new_block + lines[i + 1:]
    return None


def to_lines(block):
    return [l + "\n" for l in block.split("\n")]


# ---------------------------------------------------------------- 1. broadphase
def step_broadphase():
    log("Step 1: Rewriting broadphase_bounding_radius")
    path = SRC / "physics" / "broadphase.c"
    lines = lines_of(path)
    new_fn = to_lines(
        "static inline float broadphase_bounding_radius(rigidbody *rb) {\n"
        "    if (rb->type == object_sphere) {\n"
        "        return rb->radius;\n"
        "    }\n"
        "    if (rb->type == object_cylinder) { /* MPE_FTC_091 */\n"
        "        return sqrtf(rb->radius * rb->radius +\n"
        "                     rb->cylinder_half_length * rb->cylinder_half_length);\n"
        "    }\n"
        "    return sqrtf(rb->half_extensions.x * rb->half_extensions.x +\n"
        "                 rb->half_extensions.y * rb->half_extensions.y +\n"
        "                 rb->half_extensions.z * rb->half_extensions.z);\n"
        "}"
    )
    result = replace_function(lines, "broadphase_bounding_radius", new_fn)
    if result is None:
        log("  [FAIL] function not found")
        return False
    write(path, "".join(result))
    log("  [OK] function rewritten cleanly")
    return True


# ---------------------------------------------------------------- 2. physics_world.h
def step_physics_world_h():
    log("Step 2: physics_world.h — typedef + heap pointer member")
    path = SRC / "core" / "physics_world.h"
    content = read(path)

    if "} cached_contact;" not in content:
        typedef = (
            "/* MFS_131A: warm-start contact cache entry. Moved here from\n"
            " * collision_mechanics.c so physics_world can own a per-world cache\n"
            " * (Milestone 3, item 101). */\n"
            "typedef struct {\n"
            "    uint32_t object_id_a;\n"
            "    uint32_t object_id_b;\n"
            "    vector3 local_position_a;\n"
            "    vector3 local_position_b;\n"
            "    float accumulated_normal_impulse;\n"
            "    float accumulated_tangent_impulse;\n"
            "    uint32_t property_stamp_a;\n"
            "    uint32_t property_stamp_b;\n"
            "} cached_contact;\n\n"
        )
        anchor = "typedef struct {\n    rigidbody *bodies;"
        if anchor not in content:
            log("  [FAIL] physics_world struct anchor not found")
            return False
        content = content.replace(anchor, typedef + anchor, 1)
        log("  [OK] cached_contact typedef inserted")
    else:
        log("  [SKIP] typedef already present")

    if "cached_contact world_contact_cache[max_cached_contacts];" in content:
        content = content.replace(
            "    /* MFS_131: per-world warm-start contact cache */\n"
            "    cached_contact world_contact_cache[max_cached_contacts];\n",
            "    /* MFS_131A: per-world warm-start cache. Heap-allocated in\n"
            "     * physics_world_init (an inline array would be ~3 MB and would\n"
            "     * overflow the stack of tests that declare worlds locally).\n"
            "     * NULL-world callers fall back to the global cache. */\n"
            "    cached_contact *world_contact_cache;\n",
            1,
        )
        log("  [OK] inline array replaced with heap pointer")
    elif "cached_contact *world_contact_cache;" in content:
        log("  [SKIP] pointer member already present")
    else:
        log("  [FAIL] cache member not found in expected form")
        return False

    write(path, content)
    return True


# ---------------------------------------------------------------- 3. collision_mechanics.c
def step_collision_mechanics():
    log("Step 3: collision_mechanics.c — typedef removal + cache function rewrites")
    path = SRC / "physics" / "collision_mechanics.c"
    lines = lines_of(path)

    # 3a. Remove the private typedef (keep everything else)
    end_idx = None
    for i, ln in enumerate(lines):
        if ln.strip() == "} cached_contact;":
            end_idx = i
            break
    if end_idx is not None:
        start_idx = None
        for j in range(end_idx - 1, -1, -1):
            if lines[j].strip().startswith("typedef struct {"):
                start_idx = j
                break
        if start_idx is not None:
            del lines[start_idx:end_idx + 1]
            log("  [OK] private typedef removed")
        else:
            log("  [WARN] typedef start not found")
    else:
        log("  [SKIP] private typedef already gone")

    # 3b. Rewrite contact_cache_save
    new_save = to_lines(
        "void contact_cache_save(struct physics_world *world, collision_data *manifolds, int count) {\n"
        "    /* MFS_131A: per-world warm-start cache.\n"
        "     * world == NULL (legacy GUI path) falls back to the global cache. */\n"
        "    int *cache_count;\n"
        "    cached_contact *cache_array;\n"
        "    if ((world) && (world->world_contact_cache)) {\n"
        "        cache_count = &world->world_contact_cache_count;\n"
        "        cache_array = world->world_contact_cache;\n"
        "    } else {\n"
        "        cache_count = &contact_impulse_cache_count;\n"
        "        cache_array = contact_impulse_cache;\n"
        "    }\n"
        "    *cache_count = 0;\n"
        "    for (int m = 0; m < count; m++) {\n"
        "        collision_data *manifold = &manifolds[m];\n"
        "        for (int i = 0; i < manifold->contact_count; i++) {\n"
        "            if (*cache_count >= max_cached_contacts) {\n"
        "                return;\n"
        "            }\n"
        "            contact_point_data *cp = &manifold->contacts[i];\n"
        "            cached_contact *cc = &cache_array[(*cache_count)++];\n"
        "            cc->object_id_a = (manifold->object_a) ? manifold->object_a->object_id : 0;\n"
        "            cc->object_id_b = (manifold->object_b) ? manifold->object_b->object_id : 0;\n"
        "            /* MPE_TASK_05_CACHE_SAVE_STAMP_BEGIN */\n"
        "            cc->property_stamp_a = a3_task05_body_property_stamp(manifold->object_a);\n"
        "            cc->property_stamp_b = a3_task05_body_property_stamp(manifold->object_b);\n"
        "            /* MPE_TASK_05_CACHE_SAVE_STAMP_END */\n"
        "            cc->local_position_a = cp->local_position_a;\n"
        "            cc->local_position_b = cp->local_position_b;\n"
        "            cc->accumulated_normal_impulse = cp->accumulated_normal_impulse;\n"
        "            cc->accumulated_tangent_impulse = cp->accumulated_tangent_impulse;\n"
        "        }\n"
        "    }\n"
        "}"
    )
    result = replace_function(lines, "void contact_cache_save(", new_save)
    if result is None:
        log("  [FAIL] contact_cache_save not found")
        return False
    lines = result
    log("  [OK] contact_cache_save rewritten")

    # 3c. Rewrite contact_cache_clear
    new_clear = to_lines(
        "void contact_cache_clear(struct physics_world *world) {\n"
        "    /* MFS_131A: NULL world = legacy global cache. */\n"
        "    if ((world) && (world->world_contact_cache)) {\n"
        "        world->world_contact_cache_count = 0;\n"
        "    } else {\n"
        "        contact_impulse_cache_count = 0;\n"
        "    }\n"
        "}"
    )
    result = replace_function(lines, "void contact_cache_clear(", new_clear)
    if result is None:
        log("  [FAIL] contact_cache_clear not found")
        return False
    lines = result
    log("  [OK] contact_cache_clear rewritten")

    write(path, "".join(lines))
    return True


# ---------------------------------------------------------------- 4. physics_world.c
def step_physics_world_c():
    log("Step 4: physics_world.c — alloc/free/reset world cache")
    path = SRC / "core" / "physics_world.c"
    lines = lines_of(path)
    content_text = "".join(lines)

    # Update the stale header NOTE if present
    old_note = (
        "* NOTE: the contact warm-start cache is still engine-global. Worlds\n"
        "* must seed non-overlapping object_id ranges (see tests/two_world_test.c).\n"
        "* A per-world cache is tracked as future work.\n"
    )
    new_note = (
        "* NOTE (MFS_131A): each world now owns its warm-start contact cache\n"
        "* (world_contact_cache, heap-allocated in physics_world_init). Legacy\n"
        "* global-array callers pass a NULL world and share the global fallback.\n"
    )
    if old_note in content_text:
        content_text = content_text.replace(old_note, new_note, 1)
        log("  [OK] file header note updated")

    lines = content_text.splitlines(keepends=True)

    new_init = to_lines(
        "void physics_world_init(physics_world *world) {\n"
        "    if (!world) {\n"
        "        return;\n"
        "    }\n"
        "    memset(world, 0, sizeof(physics_world)); /* MPE_FTC_076a */\n"
        "    if (!world->bodies) {\n"
        "        world->bodies = (rigidbody *) malloc((size_t) mpe_max_bodies * sizeof(rigidbody));\n"
        "        world->body_capacity = mpe_max_bodies;\n"
        "    }\n"
        "    if (!world->world_contact_cache) {\n"
        "        world->world_contact_cache =\n"
        "            (cached_contact *) malloc((size_t) max_cached_contacts * sizeof(cached_contact)); /* MFS_131A */\n"
        "    }\n"
        "    world->body_count = 0;\n"
        "    if (world->next_object_id == 0) {\n"
        "        world->next_object_id = 1;\n"
        "    }\n"
        "}"
    )
    result = replace_function(lines, "void physics_world_init(", new_init)
    if result is None:
        log("  [FAIL] physics_world_init not found")
        return False
    lines = result
    log("  [OK] physics_world_init allocates cache")

    new_cleanup = to_lines(
        "void physics_world_cleanup(physics_world *world) {\n"
        "    if (!world) {\n"
        "        return;\n"
        "    }\n"
        "    if (world->bodies) {\n"
        "        free(world->bodies);\n"
        "        world->bodies = NULL;\n"
        "    }\n"
        "    if (world->world_contact_cache) {\n"
        "        free(world->world_contact_cache); /* MFS_131A */\n"
        "        world->world_contact_cache = NULL;\n"
        "    }\n"
        "    world->world_contact_cache_count = 0;\n"
        "    world->body_count = 0;\n"
        "    world->body_capacity = 0;\n"
        "}"
    )
    result = replace_function(lines, "void physics_world_cleanup(", new_cleanup)
    if result is None:
        log("  [FAIL] physics_world_cleanup not found")
        return False
    lines = result
    log("  [OK] physics_world_cleanup frees cache")

    new_clear = to_lines(
        "void physics_world_clear(physics_world *world) {\n"
        "    if (!world) {\n"
        "        return;\n"
        "    }\n"
        "    world->body_count = 0;\n"
        "    world->world_contact_cache_count = 0; /* MFS_131A */\n"
        "}"
    )
    result = replace_function(lines, "void physics_world_clear(", new_clear)
    if result is None:
        log("  [FAIL] physics_world_clear not found")
        return False
    lines = result
    log("  [OK] physics_world_clear resets cache count")

    write(path, "".join(lines))
    return True


# ---------------------------------------------------------------- 5. legacy loop
def step_legacy_loop():
    log("Step 5: simulation_physics_loop.c — restore legacy warm-start save")
    path = SRC / "core" / "simulation_physics_loop.c"
    lines = lines_of(path)
    fixed = False
    for i, ln in enumerate(lines):
        if "MFS_131: contact cache save removed" in ln:
            indent = ln[:len(ln) - len(ln.lstrip())]
            lines[i] = (indent + "contact_cache_save(NULL, active_manifold, manifold_count); "
                        "/* MFS_131A: legacy global fallback cache */\n")
            fixed = True
            break
    if not fixed:
        # Already restored?
        for ln in lines:
            if "contact_cache_save(NULL, active_manifold, manifold_count);" in ln:
                log("  [SKIP] already restored")
                return True
        log("  [FAIL] neither the comment nor the restored call found")
        return False
    write(path, "".join(lines))
    log("  [OK] legacy cache save restored")
    return True


# ---------------------------------------------------------------- 6. robot registry
def step_robot_registry():
    log("Step 6: gui_robot_registry.c — drop bogus cache call")
    path = SRC / "robotics" / "gui_robot_registry.c"
    lines = lines_of(path)
    kept = []
    removed = 0
    for ln in lines:
        if "contact_cache_save(mfs_gui_robot_world, NULL, 0);" in ln:
            removed += 1
            continue
        if ln.strip() == '#include "../physics/collision_mechanics.h" /* MFS_131 */':
            removed += 1
            continue
        kept.append(ln)
    if removed == 0:
        log("  [SKIP] nothing to remove")
        return True
    write(path, "".join(kept))
    log(f"  [OK] removed {removed} line(s)")
    return True


# ---------------------------------------------------------------- 7. clear callers
def step_clear_callers():
    log("Step 7: contact_cache_clear() -> contact_cache_clear(NULL) across tree")
    total = 0
    for c_file in sorted(SRC.rglob("*.c")):
        content = read(c_file)
        n1 = content.count("contact_cache_clear();")
        n2 = content.count("contact_cache_clear ();")
        if n1 or n2:
            content = content.replace("contact_cache_clear();", "contact_cache_clear(NULL);")
            content = content.replace("contact_cache_clear ();", "contact_cache_clear(NULL);")
            write(c_file, content)
            log(f"  [OK] {c_file.relative_to(SRC)}: {n1 + n2} call site(s)")
            total += n1 + n2
    if total == 0:
        log("  [SKIP] no zero-arg callers remain")
    return True


# ---------------------------------------------------------------- 8. manifest
def step_manifest():
    log("Step 8: FTC_PLAN_MANIFEST — mark item 101 done")
    path = ROOT / "fixes" / "FTC_PLAN_MANIFEST.md"
    content = read(path)
    old = "- [ ] **101: Per-World Contact Cache**"
    new = "- [x] **101: Per-World Contact Cache**"
    if old in content:
        write(path, content.replace(old, new, 1))
        log("  [OK] item 101 checked")
    elif new in content:
        log("  [SKIP] already checked")
    else:
        log("  [WARN] item 101 line not found")
    return True


# ---------------------------------------------------------------- main
def main():
    print("=" * 60)
    print("MFS 131a: Repair Per-World Contact Cache")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN **\n")

    if not SRC.exists():
        print(f"FATAL: Source directory not found: {SRC}")
        return 1

    steps = [
        step_broadphase,
        step_physics_world_h,
        step_collision_mechanics,
        step_physics_world_c,
        step_legacy_loop,
        step_robot_registry,
        step_clear_callers,
    ]
    for fn in steps:
        try:
            if not fn():
                print(f"\n[FAIL] {fn.__name__} failed. Aborting before build.")
                return 1
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__} raised: {e}")
            import traceback
            traceback.print_exc()
            return 1

    if DRY_RUN:
        print("\n  [DRY RUN] Skipping build/test/manifest.")
        return 0

    log("Build check...")
    r = subprocess.run([sys.executable, str(TOOLS / "build_check.py"), "--quick"],
                       cwd=str(ROOT), capture_output=True, text=True, timeout=180)
    print(r.stdout[-2500:] if r.stdout else "")
    if r.returncode != 0:
        print(r.stderr[-2500:] if r.stderr else "")
        log("[FAIL] Build failed")
        return 1
    log("[PASS] Build clean")

    log("Running headless tests...")
    r = subprocess.run([sys.executable, str(TOOLS / "test_runner.py")],
                       cwd=str(ROOT), capture_output=True, text=True, timeout=300)
    print(r.stdout[-2500:] if r.stdout else "")
    if r.returncode != 0:
        log("[FAIL] Tests failed — manifest NOT updated")
        return 1
    log("[PASS] All 8 tests green")

    step_manifest()

    print("=" * 60)
    print("  131a complete. Per-world contact cache is now real:")
    print("    - heap-allocated per world (no stack blowup)")
    print("    - NULL-world callers use the legacy global fallback")
    print("    - legacy GUI warm-start restored")
    print("    - broadphase bounding-radius function repaired")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())

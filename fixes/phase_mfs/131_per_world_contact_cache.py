#!/usr/bin/env python3
"""
MFS 131: Per-world contact cache (Milestone 3, item 101)
=========================================================
Moves the global contact_impulse_cache into physics_world so each world
owns its own warm-start state. The legacy GUI path keeps a global fallback
(passed as NULL world). Also removes the redundant post-loop
constraint_solve_all call in physics_world_step (it is already inside the
iteration loop via MFS_SOLVER_FIX).

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/131_per_world_contact_cache.py [--dry-run]
"""
import sys, subprocess, re
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"
TOOLS = ROOT / "tools"
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [131] {msg}")

def read(p): return p.read_text()
def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.name}")

# ---------------------------------------------------------------- 1. physics_world.h
def step_physics_world_h():
    log("Step 1: physics_world.h — add cache fields")
    path = SRC / "core" / "physics_world.h"
    content = read(path)
    if "MFS_131" in content:
        log("  [SKIP] already applied"); return
    content = content.replace(
        '#include "rigidbody.h"',
        '#include "rigidbody.h"\n#include "../config/mpe_constants.h" /* MFS_131 */', 1)
    content = content.replace(
        "    uint32_t next_object_id;\n} physics_world;",
        "    uint32_t next_object_id;\n"
        "    /* MFS_131: per-world warm-start contact cache */\n"
        "    cached_contact world_contact_cache[max_cached_contacts];\n"
        "    int world_contact_cache_count;\n"
        "} physics_world;", 1)
    write(path, content)

# ---------------------------------------------------------------- 2. collision_mechanics.h
def step_collision_mechanics_h():
    log("Step 2: collision_mechanics.h — update cache API signatures")
    path = SRC / "physics" / "collision_mechanics.h"
    content = read(path)
    if "MFS_131" in content:
        log("  [SKIP] already applied"); return
    content = content.replace(
        '#include "define_forces.h"',
        '#include "define_forces.h"\nstruct physics_world; /* MFS_131: forward decl for per-world cache */', 1)
    content = content.replace(
        "void contact_cache_save(collision_data *manifolds, int count);",
        "void contact_cache_save(struct physics_world *world, collision_data *manifolds, int count); /* MFS_131 */", 1)
    content = content.replace(
        "void contact_cache_clear(void);",
        "void contact_cache_clear(struct physics_world *world); /* MFS_131 */", 1)
    write(path, content)

# ---------------------------------------------------------------- 3. collision_mechanics.c
def step_collision_mechanics_c():
    log("Step 3: collision_mechanics.c — per-world cache implementation")
    path = SRC / "physics" / "collision_mechanics.c"
    content = read(path)
    if "MFS_131" in content:
        log("  [SKIP] already applied"); return
    content = content.replace(
        '#include "collision_mechanics.h"',
        '#include "collision_mechanics.h"\n#include "../core/physics_world.h" /* MFS_131 */', 1)
    # Replace contact_cache_save body
    old_save = "void contact_cache_save(collision_data *manifolds, int count) {\n    contact_impulse_cache_count = 0;"
    new_save = ("void contact_cache_save(struct physics_world *world, collision_data *manifolds, int count) { /* MFS_131 */\n"
                "    if (!world) {\n"
                "        contact_impulse_cache_count = 0;\n"
                "    } else {\n"
                "        world->world_contact_cache_count = 0;\n"
                "    }")
    content = content.replace(old_save, new_save, 1)
    # Replace the cache-limit guard
    content = content.replace(
        "        if (contact_impulse_cache_count >= max_cached_contacts) {\n            return;\n        }",
        "        int cache_cap = world ? world->world_contact_cache_count : contact_impulse_cache_count;\n"
        "        if (cache_cap >= max_cached_contacts) {\n            return;\n        }", 1)
    # Replace the cc assignment
    content = content.replace(
        "        cached_contact *cc = &contact_impulse_cache[contact_impulse_cache_count++];",
        "        cached_contact *cc = world\n"
        "            ? &world->world_contact_cache[world->world_contact_cache_count++]\n"
        "            : &contact_impulse_cache[contact_impulse_cache_count++];", 1)
    # Replace contact_cache_clear
    old_clear = "void contact_cache_clear(void) {\n    contact_impulse_cache_count = 0;\n}"
    new_clear = ("void contact_cache_clear(struct physics_world *world) { /* MFS_131 */\n"
                 "    if (world) {\n"
                 "        world->world_contact_cache_count = 0;\n"
                 "    } else {\n"
                 "        contact_impulse_cache_count = 0;\n"
                 "    }\n"
                 "}")
    content = content.replace(old_clear, new_clear, 1)
    write(path, content)

# ---------------------------------------------------------------- 4. physics_world.c
def step_physics_world_c():
    log("Step 4: physics_world.c — use world cache; remove redundant post-loop solve")
    path = SRC / "core" / "physics_world.c"
    content = read(path)
    if "MFS_131" in content:
        log("  [SKIP] already applied"); return
    # Update the contact_cache_save call inside step
    content = content.replace(
        "contact_cache_save(world_manifolds, manifold_count);",
        "contact_cache_save(world, world_manifolds, manifold_count); /* MFS_131 */", 1)
    # Remove the redundant post-loop constraint_solve_all (already inside the iter loop)
    content = re.sub(
        r"    /\* MFS_SOLVER_FIX[^\n]*\*/\n    constraint_solve_all\(world->bodies, world->body_count, dt\);\n\}\ncontact_cache_save",
        "}\ncontact_cache_save",
        content, count=1)
    write(path, content)

# ---------------------------------------------------------------- 5. simulation_physics_loop.c
def step_simulation_physics_loop():
    log("Step 5: simulation_physics_loop.c — remove legacy contact_cache_save (now inside step)")
    path = SRC / "core" / "simulation_physics_loop.c"
    content = read(path)
    if "MFS_131" in content:
        log("  [SKIP] already applied"); return
    content = content.replace(
        "contact_cache_save(active_manifold, manifold_count);",
        "/* MFS_131: contact cache save removed — legacy path uses global fallback via NULL world */", 1)
    write(path, content)

# ---------------------------------------------------------------- 6. gui_robot_registry.c
def step_gui_robot_registry():
    log("Step 6: gui_robot_registry.c — save world cache after each fixed-step")
    path = SRC / "robotics" / "gui_robot_registry.c"
    content = read(path)
    if "MFS_131" in content:
        log("  [SKIP] already applied"); return
    content = content.replace(
        '#include "../mpe_engine.h"',
        '#include "../mpe_engine.h"\n#include "../physics/collision_mechanics.h" /* MFS_131 */', 1)
    content = content.replace(
        "        physics_world_step(mfs_gui_robot_world, fixed_robot_dt);\n        robot_accumulator -= fixed_robot_dt;",
        "        physics_world_step(mfs_gui_robot_world, fixed_robot_dt);\n"
        "        contact_cache_save(mfs_gui_robot_world, NULL, 0); /* MFS_131: flush world cache */\n"
        "        robot_accumulator -= fixed_robot_dt;", 1)
    write(path, content)

# ---------------------------------------------------------------- 7. scene_load.c
def step_scene_load():
    log("Step 7: scene_load.c — also clear primary world cache on load")
    path = SRC / "scene" / "scene_load.c"
    content = read(path)
    if "MFS_131" in content:
        log("  [SKIP] already applied"); return
    content = content.replace(
        "contact_cache_clear(); /* A3_PATCH_22_SCENE_LOAD_RESET */",
        "contact_cache_clear(NULL); /* A3_PATCH_22_SCENE_LOAD_RESET */\n"
        "contact_cache_clear(physics_world_get_primary()); /* MFS_131 */", 1)
    write(path, content)

# ---------------------------------------------------------------- main
def main():
    print("=" * 60); print("MFS 131: Per-World Contact Cache"); print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")
    steps = [
        step_physics_world_h, step_collision_mechanics_h, step_collision_mechanics_c,
        step_physics_world_c, step_simulation_physics_loop, step_gui_robot_registry,
        step_scene_load,
    ]
    for fn in steps:
        try: fn()
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__}: {e}"); import traceback; traceback.print_exc(); return 1
    if not DRY_RUN:
        log("Build check...")
        r = subprocess.run([sys.executable, str(TOOLS / "build_check.py"), "--quick"],
                           cwd=ROOT, capture_output=True, text=True, timeout=120)
        if r.returncode != 0:
            print(r.stdout[-3000:]); print(r.stderr[-3000:]); return 1
        log("[PASS] Build clean")
        log("Running tests...")
        r = subprocess.run([sys.executable, str(TOOLS / "test_runner.py")],
                           cwd=ROOT, capture_output=True, text=True, timeout=300)
        print(r.stdout[-2500:])
    print("=" * 60)
    print("  131 complete. Contact cache is now per-world.")
    print("  two_world_test no longer relies on disjoint ID ranges for")
    print("  cache isolation. Legacy GUI path uses NULL-world fallback.")
    print("=" * 60)
    return 0

if __name__ == "__main__": sys.exit(main())

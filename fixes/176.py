#!/usr/bin/env python3
"""
MFS 176: FTC field builder
===========================
Creates robotics/field.h and robotics/field.c with:
  - ftc_field_config struct (12x12ft, wall height, thickness)
  - FTC_FIELD_STANDARD preset
  - ftc_field_build() – spawns 4 static wall cubes
  - ftc_get_spawn_position() – 4 alliance station positions

Usage:
    cd <project_root>
    python3 fixes/176_ftc_field.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [176] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.relative_to(SRC)}")

FIELD_H = r'''/* MFS_176: FTC field builder */
#ifndef field_h
#define field_h
#include "../core/physics_world.h"

typedef struct {
    float half_width;
    float half_depth;
    float wall_height;
    float wall_thickness;
} ftc_field_config;

extern const ftc_field_config FTC_FIELD_STANDARD;

int ftc_field_build(physics_world *world, const ftc_field_config *field);

typedef enum {
    FTC_SPAWN_RED_1, FTC_SPAWN_RED_2,
    FTC_SPAWN_BLUE_1, FTC_SPAWN_BLUE_2,
} ftc_spawn_position;

vector3 ftc_get_spawn_position(ftc_spawn_position pos, float rest_height);
#endif
'''

FIELD_C = r'''/* MFS_176: FTC field builder implementation */
#include "field.h"

const ftc_field_config FTC_FIELD_STANDARD = {
    .half_width     = 1.8288f,   /* 6 ft */
    .half_depth     = 1.8288f,
    .wall_height    = 0.3048f,   /* 12 in */
    .wall_thickness = 0.0254f,   /* 1 in  */
};

int ftc_field_build(physics_world *world, const ftc_field_config *f) {
    if (!world) return -1;

    /* north wall */
    int n = physics_world_add_cube(world,
        (vector3){0.0f, f->wall_height * 0.5f, f->half_depth},
        (vector3){f->half_width, f->wall_height * 0.5f, f->wall_thickness * 0.5f},
        0.0f);
    /* south wall */
    int s = physics_world_add_cube(world,
        (vector3){0.0f, f->wall_height * 0.5f, -f->half_depth},
        (vector3){f->half_width, f->wall_height * 0.5f, f->wall_thickness * 0.5f},
        0.0f);
    /* east wall */
    int e = physics_world_add_cube(world,
        (vector3){f->half_width, f->wall_height * 0.5f, 0.0f},
        (vector3){f->wall_thickness * 0.5f, f->wall_height * 0.5f, f->half_depth},
        0.0f);
    /* west wall */
    int w = physics_world_add_cube(world,
        (vector3){-f->half_width, f->wall_height * 0.5f, 0.0f},
        (vector3){f->wall_thickness * 0.5f, f->wall_height * 0.5f, f->half_depth},
        0.0f);

    if ((n < 0) || (s < 0) || (e < 0) || (w < 0)) return -1;
    return 0;
}

vector3 ftc_get_spawn_position(ftc_spawn_position pos, float rest_height) {
    switch (pos) {
        case FTC_SPAWN_RED_1:  return (vector3){-1.2f, rest_height, -1.2f};
        case FTC_SPAWN_RED_2:  return (vector3){ 1.2f, rest_height, -1.2f};
        case FTC_SPAWN_BLUE_1: return (vector3){-1.2f, rest_height,  1.2f};
        case FTC_SPAWN_BLUE_2: return (vector3){ 1.2f, rest_height,  1.2f};
    }
    return (vector3){0.0f, rest_height, 0.0f};
}
'''

def step_write():
    log("Step 1: Writing field.h and field.c")
    write(SRC / "robotics" / "field.h", FIELD_H)
    write(SRC / "robotics" / "field.c", FIELD_C)
    return True

def step_build():
    log("Step 2: Build check")
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
    print("MFS 176: FTC field builder")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")
    if not step_write(): return 1
    if not DRY_RUN:
        if not step_build(): return 1
    print("=" * 60)
    print("  176 complete. FTC field builder ready.")
    print("  ftc_field_build() spawns 4 walls at 12x12ft.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
MFS 149: Show physics_world_step internals + rigidbody struct (READ-ONLY)
==========================================================================
We need the exact internal order of physics_world_step (velocity integrate
-> contact solve -> position integrate) to place a post-contact wheel-lock,
and the rigidbody field layout to add an axle-lock flag.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/149_show_step_internals.py
"""
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"


def extract_function(text, name):
    idx = text.find(name)
    while idx != -1:
        b = text.find("{", idx)
        if b == -1:
            break
        if ")" in text[idx:b]:
            depth = 0
            for j in range(b, len(text)):
                if text[j] == "{":
                    depth += 1
                elif text[j] == "}":
                    depth -= 1
                    if depth == 0:
                        return text[idx:j + 1]
        idx = text.find(name, idx + 1)
    return None


def main():
    print("=" * 60)
    print("MFS 149: physics_world_step internals (read-only)")
    print("=" * 60)

    pw_c = (SRC / "core" / "physics_world.c").read_text()
    fn = extract_function(pw_c, "physics_world_step")
    if fn:
        print("\n----- physics_world_step (physics_world.c) -----")
        print(fn)
    else:
        print("[WARN] physics_world_step not found in physics_world.c")

    print("\n----- rigidbody.h (first 4000 chars) -----")
    rb_h = (SRC / "core" / "rigidbody.h").read_text()
    print(rb_h[:4000])

    print("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

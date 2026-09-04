#!/usr/bin/env python3
"""
MFS 130a: Robust residual sweep (whitespace-agnostic)
======================================================
Re-checks the three residual patterns from 130 using line-stripped matching.
Prints a diagnostic for each file, fixes anything still present.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/130a_robust_residual_sweep.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [130a] {msg}")

# ---------------------------------------------------------------- broadphase
def sweep_broadphase():
    path = SRC / "physics" / "broadphase.c"
    lines = path.read_text().splitlines(keepends=True)
    in_func = False; depth = 0; seen_generic = False
    out = []; removed = 0
    for line in lines:
        s = line.strip()
        if not in_func and "broadphase_bounding_radius" in s:
            in_func = True
            depth = s.count("{") - s.count("}")
            out.append(line); continue
        if not in_func:
            out.append(line); continue
        depth += s.count("{") - s.count("}")
        if depth <= 0:
            in_func = False; out.append(line); continue
        if s.startswith("return sqrtf(rb->half_extensions"):
            seen_generic = True; out.append(line); continue
        if seen_generic and s.startswith("if (rb->type == object_cylinder)"):
            log(f"  [FIX] broadphase.c: removing dead cylinder branch after generic return")
            skip_depth = s.count("{")
            removed += 1
            while skip_depth > 0 and (lines.index(line) + 1) < len(lines):
                idx = lines.index(line) + 1
                s2 = lines[idx].strip()
                skip_depth += s2.count("{") - s2.count("}")
                line = lines[idx]
            continue
        out.append(line)
    if removed and not DRY_RUN:
        path.write_text("".join(out))
    log(f"  broadphase.c: {'clean' if removed == 0 else f'{removed} dead block(s) removed'}")

# ---------------------------------------------------------------- rigidbody
def sweep_rigidbody():
    path = SRC / "core" / "rigidbody.c"
    lines = path.read_text().splitlines(keepends=True)
    in_fn = False; fn_depth = 0; in_block = False; seen_first = False
    out = []; removed = 0
    for line in lines:
        s = line.strip()
        if not in_fn and "void rigidbody_set_static" in s:
            in_fn = True; fn_depth = s.count("{") - s.count("}")
            out.append(line); continue
        if not in_fn:
            out.append(line); continue
        fn_depth += s.count("{") - s.count("}")
        if fn_depth <= 0:
            in_fn = False; out.append(line); continue
        if not in_block and "if (rigid_body->type == object_sphere)" in s:
            in_block = True; seen_first = False; out.append(line); continue
        if in_block:
            if "object_cylinder" in s:
                if not seen_first:
                    seen_first = True; out.append(line)
                else:
                    log("  [FIX] rigidbody.c: removing duplicate cylinder dispatch")
                    removed += 1
                continue
            if s.startswith("} else {"):
                in_block = False
            out.append(line); continue
        out.append(line)
    if removed and not DRY_RUN:
        path.write_text("".join(out))
    log(f"  rigidbody.c: {'clean' if removed == 0 else f'{removed} duplicate(s) removed'}")

# ---------------------------------------------------------------- input_control
def sweep_input_control():
    path = SRC / "ui_input" / "input_control.c"
    lines = path.read_text().splitlines(keepends=True)
    seen_g_release = False; out = []; removed = 0
    for line in lines:
        s = line.strip()
        if s == "if (event -> keyval == GDK_KEY_g) {input_state -> g_key_pressed = false;}":
            if seen_g_release:
                log("  [FIX] input_control.c: removing duplicate g_key_pressed release")
                removed += 1; continue
            seen_g_release = True
        out.append(line)
    if removed and not DRY_RUN:
        path.write_text("".join(out))
    log(f"  input_control.c: {'clean' if removed == 0 else f'{removed} duplicate(s) removed'}")

# ---------------------------------------------------------------- main
def main():
    print("=" * 60); print("MFS 130a: Robust Residual Sweep"); print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")
    sweep_broadphase(); sweep_rigidbody(); sweep_input_control()
    if not DRY_RUN:
        log("Build check...")
        r = subprocess.run([sys.executable, str(TOOLS / "build_check.py"), "--quick"],
                           cwd=ROOT, capture_output=True, text=True, timeout=120)
        if r.returncode != 0:
            print(r.stdout[-2000:]); print(r.stderr[-2000:]); return 1
        log("[PASS] Build clean")
        r = subprocess.run([sys.executable, str(TOOLS / "test_runner.py")],
                           cwd=ROOT, capture_output=True, text=True, timeout=300)
        print(r.stdout[-1500:])
    print("=" * 60); return 0

if __name__ == "__main__": sys.exit(main())

#!/usr/bin/env python3
"""
MFS 173a: Repair broken cylinder dispatch from 173
===================================================
173 inserted DISPATCH_BLOCK with an extra closing brace, producing:
    }
    } else if ((body_a->type == object_cylinder) ...
which is a syntax error. The fix removes the extra '}' so the chain reads:
    } else if ((body_a->type == object_cylinder) ...

Also verifies the same fix in simulation_physics_loop.c.

Usage:
    cd <project_root>
    python3 fixes/173a.py [--dry-run]
"""
import sys, subprocess, re
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [173a] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.relative_to(SRC)}")

def fix_physics_world():
    log("Step 1: Fixing dispatch in physics_world.c")
    p = SRC / "core" / "physics_world.c"
    c = p.read_text()

    if "MFS_173A_REPAIRED" in c:
        log("  [SKIP] already repaired"); return True

    if "collision_cylinder_sphere" not in c:
        log("  [SKIP] cylinder dispatch not present, nothing to repair"); return True

    # The broken pattern: a lone '}' on its own line, immediately followed by
    # '} else if ((body_a->type == object_cylinder)'
    # We need to remove that extra '}'.
    #
    # Pattern: "}\n} else if ((body_a->type == object_cylinder)"
    # Fix:     "} else if ((body_a->type == object_cylinder)"

    broken = "}\n} else if ((body_a->type == object_cylinder)"
    fixed  = "} else if ((body_a->type == object_cylinder) /* MFS_173A_REPAIRED */"

    if broken in c:
        c = c.replace(broken, fixed, 1)
        write(p, c)
        log("  [OK] removed extra closing brace before cylinder dispatch")
        return True

    # Try with \r\n just in case
    broken_crlf = "}\r\n} else if ((body_a->type == object_cylinder)"
    fixed_crlf  = "} else if ((body_a->type == object_cylinder) /* MFS_173A_REPAIRED */"
    if broken_crlf in c:
        c = c.replace(broken_crlf, fixed_crlf, 1)
        write(p, c)
        log("  [OK] removed extra closing brace (CRLF variant)")
        return True

    # Maybe the brace is on the same line with spaces
    broken_spaced = re.compile(r'\}\s*\n\} else if \(\(body_a->type == object_cylinder\)')
    m = broken_spaced.search(c)
    if m:
        c = c[:m.start()] + "} else if ((body_a->type == object_cylinder) /* MFS_173A_REPAIRED */" + c[m.end():]
        write(p, c)
        log("  [OK] removed extra closing brace (regex variant)")
        return True

    log("  [WARN] broken pattern not found, checking if already correct")
    if "} else if ((body_a->type == object_cylinder)" in c:
        log("  [OK] dispatch already looks correct")
        return True

    log("  [FAIL] cannot find cylinder dispatch at all"); return False

def fix_sim_loop():
    log("Step 2: Fixing dispatch in simulation_physics_loop.c")
    p = SRC / "core" / "simulation_physics_loop.c"
    c = p.read_text()

    if "MFS_173A_REPAIRED_SIM" in c:
        log("  [SKIP] already repaired"); return True

    if "collision_cylinder_sphere" not in c:
        log("  [SKIP] cylinder dispatch not present, nothing to repair"); return True

    broken = "}\n} else if ((rigid_body_a->type == object_cylinder)"
    fixed  = "} else if ((rigid_body_a->type == object_cylinder) /* MFS_173A_REPAIRED_SIM */"

    if broken in c:
        c = c.replace(broken, fixed, 1)
        write(p, c)
        log("  [OK] removed extra closing brace before cylinder dispatch")
        return True

    broken_crlf = "}\r\n} else if ((rigid_body_a->type == object_cylinder)"
    fixed_crlf  = "} else if ((rigid_body_a->type == object_cylinder) /* MFS_173A_REPAIRED_SIM */"
    if broken_crlf in c:
        c = c.replace(broken_crlf, fixed_crlf, 1)
        write(p, c)
        log("  [OK] removed extra closing brace (CRLF variant)")
        return True

    broken_spaced = re.compile(r'\}\s*\n\} else if \(\(rigid_body_a->type == object_cylinder\)')
    m = broken_spaced.search(c)
    if m:
        c = c[:m.start()] + "} else if ((rigid_body_a->type == object_cylinder) /* MFS_173A_REPAIRED_SIM */" + c[m.end():]
        write(p, c)
        log("  [OK] removed extra closing brace (regex variant)")
        return True

    if "} else if ((rigid_body_a->type == object_cylinder)" in c:
        log("  [OK] dispatch already looks correct")
        return True

    log("  [FAIL] cannot find cylinder dispatch at all"); return False

def build():
    log("Step 3: Build check")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=180)
    print(r.stdout[-2500:] if r.stdout else "")
    if r.returncode != 0:
        print(r.stderr[-2000:] if r.stderr else "")
        log("[FAIL] build still broken")
        return False
    log("[PASS] build clean")
    return True

def tests():
    log("Step 4: Running headless tests")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "test_runner.py")],
        cwd=str(ROOT), capture_output=True, text=True, timeout=300)
    print(r.stdout[-2500:] if r.stdout else "")
    if r.returncode != 0:
        log("[WARN] some tests failed")
        return False
    log("[PASS] all tests pass")
    return True

def main():
    print("=" * 60)
    print("MFS 173a: Repair cylinder dispatch brace error")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")

    if not fix_physics_world(): return 1
    if not fix_sim_loop(): return 1

    if not DRY_RUN:
        if not build(): return 1
        tests()

    print("=" * 60)
    print("  173a complete. Cylinder dispatch should now compile.")
    print("  If build still fails, paste the errors back.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())

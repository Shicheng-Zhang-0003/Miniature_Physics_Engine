#!/usr/bin/env python3
"""
R3-001  Cylinder integrity  (Layer 1 of the R3 cylinder perfection pass)
=========================================================================
Two correctness fixes in <version>/src/core/rigidbody.c.

FIX 1 - rigidbody_sanitize() has NO cylinder branch.
    The type dispatch today is:
        if (sphere) { validate radius }
        else        { validate half_extensions.x/y/z }   <- cylinders fall here
    Cylinders don't use half_extensions, so a cylinder's half_extensions get
    spuriously clamped to 0.01 while radius and cylinder_half_length are NEVER
    validated. A NaN / negative cylinder_half_length sails through uncorrected.
    -> Add a proper `else if (cylinder)` branch validating radius AND
       cylinder_half_length.

FIX 2 - rigidbody_set_static() double-calls the cylinder inertia update.
    The cylinder branch calls rigidbody_update_inertia_cylinder() twice
    (leftover from the old 093d awk repair). Remove the duplicate.

Idempotent: safe to run repeatedly. Indentation-aware (derives tabs/spaces
from the surrounding code, so it works regardless of format state).

Usage:
    cd <project_root>
    python3 fixes/r3_cylinders/001_cylinder_integrity.py [--dry-run]
"""
import re, sys, subprocess
from pathlib import Path

DRY_RUN = "--dry-run" in sys.argv


def find_root():
    p = Path(__file__).resolve().parent
    while p != p.parent:
        if (p / "v15R3" / "src").exists() or (p / "v15R2" / "src").exists():
            return p
        p = p.parent
    return None


ROOT = find_root()
if ROOT is None:
    print("FATAL: could not locate project root (no v15R3/src or v15R2/src)")
    sys.exit(1)

SRC = ROOT / "v15R3" / "src" if (ROOT / "v15R3" / "src").exists() else ROOT / "v15R2" / "src"
RB = SRC / "core" / "rigidbody.c"


def log(m):
    print(f"  [R3-001] {m}")


def main():
    print("=" * 64)
    print("R3-001: Cylinder integrity (sanitize branch + set_static dedupe)")
    print("=" * 64)
    if DRY_RUN:
        print("  ** DRY RUN - no files written **\n")
    if not RB.exists():
        log(f"FATAL: {RB} not found")
        return 1

    content = RB.read_text()
    original = content
    changed = []

    # ---- FIX 1: sanitize cylinder branch -------------------------------
    if "cylinder_half_length <= 0.0f" in content:
        log("FIX 1 [SKIP] sanitize cylinder branch already present")
    else:
        m = re.search(
            r'(?P<i1>[ \t]*)\} else \{\n'
            r'(?P<i2>[ \t]*)if \(!isfinite\(rigid_body->half_extensions\.x\) \|\| '
            r'\(rigid_body->half_extensions\.x <= 0\.0f\)\) \{\n'
            r'(?P<i3>[ \t]*)rigid_body->half_extensions\.x = 0\.01f;',
            content)
        if not m:
            log("FIX 1 [FAIL] sanitize else-block anchor not found")
            return 1
        i1, i2, i3 = m.group("i1"), m.group("i2"), m.group("i3")
        replacement = (
            f"{i1}}} else if (rigid_body->type == object_cylinder) {{ /* R3-001 */\n"
            f"{i2}if (!isfinite(rigid_body->radius) || (rigid_body->radius <= 0.0f)) {{\n"
            f"{i3}rigid_body->radius = 0.01f;\n"
            f"{i3}needs_inertia_recalc = true;\n"
            f"{i2}}}\n"
            f"{i2}if (!isfinite(rigid_body->cylinder_half_length) || (rigid_body->cylinder_half_length <= 0.0f)) {{\n"
            f"{i3}rigid_body->cylinder_half_length = 0.01f;\n"
            f"{i3}needs_inertia_recalc = true;\n"
            f"{i2}}}\n"
            f"{i1}}} else {{\n"
            f"{i2}if (!isfinite(rigid_body->half_extensions.x) || (rigid_body->half_extensions.x <= 0.0f)) {{\n"
            f"{i3}rigid_body->half_extensions.x = 0.01f;"
        )
        content = content.replace(m.group(0), replacement, 1)
        changed.append("sanitize cylinder branch")
        log("FIX 1 [OK] added cylinder branch to rigidbody_sanitize")

    # ---- FIX 2: set_static dedupe --------------------------------------
    dedupe = re.compile(
        r'([ \t]*)rigidbody_update_inertia_cylinder\(rigid_body\);\n'
        r'\1rigidbody_update_inertia_cylinder\(rigid_body\);')
    if dedupe.search(content):
        content = dedupe.sub(
            r'\1rigidbody_update_inertia_cylinder(rigid_body); /* R3-001 dedupe */',
            content, count=1)
        changed.append("set_static dedupe")
        log("FIX 2 [OK] removed duplicate inertia update in rigidbody_set_static")
    else:
        log("FIX 2 [SKIP] no consecutive duplicate found")

    if not changed:
        log("Nothing to do - already clean.")
        return 0

    if not DRY_RUN:
        backup = RB.with_suffix(RB.suffix + ".pre_r3_001")
        if not backup.exists():
            backup.write_text(original)
            log(f"backup -> {backup.name}")
        RB.write_text(content)
        log(f"wrote {RB.name}")

        # syntax-only compile check
        try:
            cflags = subprocess.run(
                ["pkg-config", "--cflags", "gtk+-3.0", "epoxy"],
                capture_output=True, text=True).stdout.split()
        except Exception:
            cflags = []
        r = subprocess.run(
            ["gcc", "-fsyntax-only"] + cflags + ["core/rigidbody.c"],
            cwd=str(SRC), capture_output=True, text=True)
        if r.returncode != 0:
            log("[FAIL] syntax check failed:")
            print(r.stderr[-2500:])
            return 1
        log("[PASS] rigidbody.c syntax check clean")
    else:
        log("[DRY RUN] skipping write + compile")

    print("=" * 64)
    print("Dry run complete." if DRY_RUN else "Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
Cleanup: Remove stale partial additions from the failed 002_depenetration_pass.py
that left a conflicting declaration in depenetration.h and a partial
implementation in depenetration.c.
"""
import sys
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
    print("FATAL: cannot locate project root")
    sys.exit(1)

SRC = ROOT / "v15R3" / "src"
STALE_MARKER = "MPE_PHASE1_DEPENETRATION_PASS"

def clean_depenetration_h():
    path = SRC / "physics" / "depenetration.h"
    if not path.exists():
        print("[SKIP] depenetration.h not found")
        return
    content = path.read_text()
    if STALE_MARKER not in content:
        print("[SKIP] depenetration.h already clean")
        return
    # Remove the stale declaration block
    lines = content.split('\n')
    clean_lines = []
    skip_block = False
    for line in lines:
        if STALE_MARKER in line:
            skip_block = True
            continue
        if skip_block:
            # Skip continuation lines until we hit a blank line or non-declaration
            stripped = line.strip()
            if stripped == '' or (not stripped.startswith('void') and not stripped.startswith('struct') and not stripped.startswith('int')):
                skip_block = False
            continue
        clean_lines.append(line)
    if not DRY_RUN:
        path.write_text('\n'.join(clean_lines))
    print("[OK] Removed stale declaration from depenetration.h")

def clean_depenetration_c():
    path = SRC / "physics" / "depenetration.c"
    if not path.exists():
        print("[SKIP] depenetration.c not found")
        return
    content = path.read_text()
    if STALE_MARKER not in content:
        print("[SKIP] depenetration.c already clean")
        return
    # Find and remove the appended stale block
    marker_comment = "/* MPE_PHASE1_DEPENETRATION_PASS:"
    idx = content.find(marker_comment)
    if idx >= 0:
        content = content[:idx].rstrip() + '\n'
        if not DRY_RUN:
            path.write_text(content)
        print("[OK] Removed stale implementation from depenetration.c")
    else:
        # Marker might be in a different form
        lines = content.split('\n')
        clean_lines = []
        skip = False
        for line in lines:
            if STALE_MARKER in line:
                skip = True
                continue
            if skip:
                stripped = line.strip()
                if stripped == '' or (stripped.startswith('/*') and STALE_MARKER not in stripped):
                    skip = False
                else:
                    continue
            clean_lines.append(line)
        if not DRY_RUN:
            path.write_text('\n'.join(clean_lines))
        print("[OK] Removed stale implementation from depenetration.c (line-by-line)")

def main():
    print("=" * 60)
    print("Cleanup: Remove stale 002 additions causing type conflict")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN **\n")
    clean_depenetration_h()
    clean_depenetration_c()
    print("\nDone. Now re-run: python3 fixes/phase1/002_depenetration_world.py")

if __name__ == "__main__":
    main()

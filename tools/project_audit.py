#!/usr/bin/env python3
"""
MPE / FTC Simulator Project Audit
=================================

Reports current architectural health:
- largest C/H files
- bash fix script presence
- test files
- source module counts
- likely global state usage
- simulation.c size
- robotics module presence

This is intentionally read-only.
"""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "v15R2" / "src"


def line_count(path: Path) -> int:
    try:
        return len(path.read_text(errors="ignore").splitlines())
    except Exception:
        return 0


def find_files(patterns):
    files = []
    for p in patterns:
        files.extend(ROOT.glob(p))
    return sorted(set(files))


def grep_count(path: Path, pattern: str) -> int:
    try:
        text = path.read_text(errors="ignore")
    except Exception:
        return 0
    return len(re.findall(pattern, text))


def main():
    print("MPE / FTC Simulator Project Audit")
    print("=" * 72)
    print(f"Root: {ROOT}")
    print(f"Source: {SRC}")
    print()

    if not SRC.exists():
        print("FATAL: v15R2/src not found")
        raise SystemExit(1)

    c_h_files = find_files(["v15R2/src/**/*.c", "v15R2/src/**/*.h"])
    c_files = [p for p in c_h_files if p.suffix == ".c"]
    h_files = [p for p in c_h_files if p.suffix == ".h"]

    print("Source inventory")
    print("-" * 72)
    print(f"C files:      {len(c_files)}")
    print(f"Header files: {len(h_files)}")
    print(f"C/H total:    {len(c_h_files)}")
    print()

    print("Largest C/H files")
    print("-" * 72)
    largest = sorted(c_h_files, key=line_count, reverse=True)[:15]
    for p in largest:
        rel = p.relative_to(ROOT)
        print(f"{line_count(p):5d} lines  {rel}")
    print()

    sim = SRC / "simulation.c"
    if sim.exists():
        print("simulation.c")
        print("-" * 72)
        print(f"Lines: {line_count(sim)}")
        print(f"object_count refs: {grep_count(sim, r'\\bobject_count\\b')}")
        print(f"obj_per_scene refs: {grep_count(sim, r'\\bobj_per_scene\\b')}")
        print(f"main_inputs refs: {grep_count(sim, r'\\bmain_inputs\\b')}")
        print()

    print("Robotics modules")
    print("-" * 72)
    robotics = SRC / "robotics"
    if robotics.exists():
        for p in sorted(robotics.glob("*.[ch]")):
            print(f"{line_count(p):5d} lines  {p.relative_to(ROOT)}")
    else:
        print("No robotics directory found")
    print()

    print("Tests")
    print("-" * 72)
    tests = sorted((SRC / "tests").glob("*.c")) if (SRC / "tests").exists() else []
    for p in tests:
        print(f"{line_count(p):5d} lines  {p.relative_to(ROOT)}")
    print()

    print("Legacy shell fix infrastructure")
    print("-" * 72)
    sh_files = find_files(["fixes/**/*.sh", "*.sh"])
    if not sh_files:
        print("No shell scripts found")
    else:
        print(f"Shell scripts found: {len(sh_files)}")
        for p in sh_files[:30]:
            print(f"  {p.relative_to(ROOT)}")
        if len(sh_files) > 30:
            print(f"  ... and {len(sh_files) - 30} more")
    print()

    print("Potential global-state hotspots")
    print("-" * 72)
    hot_patterns = [
        r"\\bobject_count\\b",
        r"\\bobj_per_scene\\b",
        r"\\bselected_object\\b",
        r"\\bmain_inputs\\b",
        r"\\bmain_camera_fov\\b",
    ]
    hotspots = []
    for p in c_h_files:
        score = sum(grep_count(p, pat) for pat in hot_patterns)
        if score > 0:
            hotspots.append((score, p))
    for score, p in sorted(hotspots, reverse=True)[:20]:
        print(f"{score:5d} refs  {p.relative_to(ROOT)}")
    print()

    print("Recommended next architectural target")
    print("-" * 72)
    print("1. Keep mecanum_drive as XFAIL until anisotropic friction is implemented.")
    print("2. Quarantine old bash fix scripts; do not run them as mutation tools.")
    print("3. Split simulation.c in small extractions:")
    print("   a. validation/reporting helpers")
    print("   b. GUI/editor input dispatch")
    print("   c. physics tick wrapper")
    print("   d. render/world synchronization")
    print("4. Build FTC HAL only after the physics_world path owns runtime state.")


if __name__ == "__main__":
    main()

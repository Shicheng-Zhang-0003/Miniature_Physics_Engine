#!/usr/bin/env python3
"""
MFS 152: VALIDATION-ONLY STATE AUDIT — NO source modifications.
================================================================
Runs the existing test infrastructure and reports current state:
  1. Clean build check          (tools/build_check.py)
  2. Standard headless suite    (tools/test_runner.py)
  3. Physics truth suite        (make test_physics_truth)
  4. Odometry diagnostic        (make test_odometry_diag)

This script compiles test binaries but does NOT edit any source file.
Safe to run repeatedly.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/152_validate_state.py
"""
import subprocess
import sys
import re
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"


def run(cmd, cwd, timeout=300):
    try:
        r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout, r.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "TIMEOUT"
    except Exception as e:
        return -1, "", str(e)


def grab(out, pattern):
    m = re.search(pattern, out)
    return m.group(0).strip() if m else None


def main():
    print("=" * 64)
    print("MFS 152: VALIDATION-ONLY STATE AUDIT  (no modifications)")
    print("=" * 64)
    report = []

    # ---- 1. Clean build ----
    print("\n[1/4] Clean build check")
    print("-" * 48)
    rc, out, err = run([sys.executable, str(TOOLS / "build_check.py"), "--quick"], ROOT)
    print((out or err)[-900:])
    report.append(("Clean build", rc == 0))

    # ---- 2. Standard headless suite ----
    print("\n[2/4] Standard headless suite (test_runner.py)")
    print("-" * 48)
    rc, out, err = run([sys.executable, str(TOOLS / "test_runner.py")], ROOT)
    print((out or err)[-1200:])
    report.append(("Standard suite", rc == 0))

    # ---- 3. Physics truth suite ----
    print("\n[3/4] Physics truth suite (24 tests)")
    print("-" * 48)
    rc, out, err = run(["make", "test_physics_truth"], SRC)
    print((out or err)[-1300:])
    summary_line = grab(out, r"Total:\s*\d+\s*\|\s*Passed:\s*\d+\s*\|\s*Failed:\s*\d+")
    report.append(("Physics truth suite", "Failed: 0" in out))

    # ---- 4. Odometry diagnostic ----
    print("\n[4/4] Odometry diagnostic (read-only)")
    print("-" * 48)
    rc, out, err = run(["make", "test_odometry_diag"], SRC)
    print((out or err)[-900:])
    report.append(("Odometry diagnostic ran", rc == 0))

    # ---- Summary ----
    print("\n" + "=" * 64)
    print("VALIDATION SUMMARY")
    print("=" * 64)
    for name, ok in report:
        print(f"  {'[PASS]' if ok else '[FAIL]'}  {name}")
    print("=" * 64)
    print("This audit made NO changes to the source tree.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

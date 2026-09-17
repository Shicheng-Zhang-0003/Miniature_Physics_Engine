#!/usr/bin/env python3
"""
MPE Test Runner (MPE-only run)
===============================
Discovers, builds, and runs all headless tests. Generates a summary report.

Robotics (MFS) tests are parked with the code in v15R3/robotics_backup.

Usage:
    python tools/test_runner.py              # Run all tests
    python tools/test_runner.py --list       # List available tests
    python tools/test_runner.py driven_wheel # Run specific test
"""

import subprocess
import sys
import time
from pathlib import Path
from typing import Optional


# v15R3 is the frozen GTK3 truth engine; v15S is the active GTK4 port.
# Prefer the active tree if present, else fall back to v15R3.
_src_v15s = Path(__file__).resolve().parent.parent / "v15S" / "src"
_src_v15r3 = Path(__file__).resolve().parent.parent / "v15R3" / "src"
SRC_DIR = _src_v15s if _src_v15s.exists() else _src_v15r3

KNOWN_TESTS = [
    "two_world",
    "revolute",
    "cylinder_drop",
    "driven_wheel",
    "math3_inverse",
    "floor_collision_diag",
    "cylinder_sphere",
    "cylinder_cube",
    "cylinder_cylinder",
    "list4_cylinder_floor",
    "scene_roundtrip",
    "static_hold",
    "rolling_decay",
    "ccd_sweep",
    "kinematic",
    "determinism",
    "momentum",
    "angmom",
    "spring",
    "projectile",
    "incline_accel",
    "pendulum",
    "bounce_series",
    "friction_stop",
    "stack",
    "f10_long_run",
    "sleep_contact_wake",
    "f11_torture",
    "frustum",
]

# Tests that encode desired future behavior but are currently expected
# to fail because the corresponding model is not implemented yet.
# MPE-only run: empty. (The old mecanum_drive XFAIL moved to
# v15R3/robotics_backup with the rest of MFS.)
EXPECTED_FAILURES = {
}


class TestResult:
    def __init__(self, name: str):
        self.name = name
        self.build_success: bool = False
        self.run_success: bool = False
        self.exit_code: int = -1
        self.stdout: str = ""
        self.stderr: str = ""
        self.duration: float = 0.0

    @property
    def passed(self) -> bool:
        return self.build_success and self.run_success and self.exit_code == 0

    @property
    def expected_failure(self) -> bool:
        return self.name in EXPECTED_FAILURES

    @property
    def xfailed(self) -> bool:
        return self.expected_failure and not self.passed

    @property
    def unexpected_pass(self) -> bool:
        return self.expected_failure and self.passed

    @property
    def blocking_failure(self) -> bool:
        return (not self.passed) and (not self.expected_failure)

    def summary_line(self) -> str:
        if self.unexpected_pass:
            status = "XPASS"
        elif self.xfailed:
            status = "XFAIL"
        elif self.passed:
            status = "PASS"
        else:
            status = "FAIL"
        dur = f"{self.duration:.1f}s"
        suffix = ""
        if self.expected_failure:
            suffix = f" — {EXPECTED_FAILURES[self.name]}"
        return f"  [{status}] {self.name:<25s} ({dur}){suffix}"


def build_test(name: str) -> tuple:
    # Build-only target: `test_<name>` also runs the test, which would
    # conflate test failures with build failures. `build_<name>` only
    # compiles; run_test() executes the binary separately.
    target = f"build_{name}"
    try:
        proc = subprocess.run(
            ["make", "-j4", target],
            cwd=str(SRC_DIR),
            capture_output=True, text=True, timeout=60,
        )
        output = proc.stdout + "\n" + proc.stderr
        return proc.returncode == 0, output
    except subprocess.TimeoutExpired:
        return False, "Build timed out"


def run_test(name: str, timeout: int = 60) -> TestResult:
    result = TestResult(name)
    start = time.monotonic()

    build_ok, build_output = build_test(name)
    result.build_success = build_ok
    if not build_ok:
        result.stderr = build_output
        result.duration = time.monotonic() - start
        return result

    binary = SRC_DIR / f"test_{name}"
    if not binary.exists():
        result.stderr = f"Binary not found: {binary}"
        result.duration = time.monotonic() - start
        return result

    try:
        proc = subprocess.run(
            [str(binary)], cwd=str(SRC_DIR),
            capture_output=True, text=True, timeout=timeout,
        )
        result.stdout = proc.stdout
        result.stderr = proc.stderr
        result.exit_code = proc.returncode
        result.run_success = proc.returncode == 0
    except subprocess.TimeoutExpired:
        result.stderr = f"Test timed out after {timeout}s"
        result.run_success = False

    result.duration = time.monotonic() - start
    return result


def run_all(test_filter: Optional[str] = None) -> list:
    tests = KNOWN_TESTS
    if test_filter:
        tests = [t for t in tests if test_filter.lower() in t.lower()]
        if not tests:
            print(f"No tests matching '{test_filter}'")
            return []

    results = []
    for name in tests:
        print(f"  Running {name}...", end=" ", flush=True)
        r = run_test(name)
        status = "PASS" if r.passed else "FAIL"
        print(f"{status} ({r.duration:.1f}s)")
        results.append(r)
    return results


def print_report(results: list):
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    passed = sum(1 for r in results if r.passed and not r.expected_failure)
    xfailed = sum(1 for r in results if r.xfailed)
    xpassed = sum(1 for r in results if r.unexpected_pass)
    failed = sum(1 for r in results if r.blocking_failure)

    for r in results:
        print(r.summary_line())

    print(
        f"\nTotal: {len(results)} | "
        f"Passed: {passed} | "
        f"Expected failures: {xfailed} | "
        f"Unexpected passes: {xpassed} | "
        f"Blocking failures: {failed}"
    )

    if failed > 0 or xfailed > 0 or xpassed > 0:
        print("\nNon-green test output:")
        for r in results:
            if not r.passed or r.unexpected_pass:
                print(f"\n--- {r.name} ---")
                if r.expected_failure:
                    print(f"EXPECTED FAILURE: {EXPECTED_FAILURES[r.name]}")
                if r.unexpected_pass:
                    print("UNEXPECTED PASS: consider removing this test from EXPECTED_FAILURES.")
                if not r.build_success:
                    print("BUILD FAILED:")
                    print(r.stderr[:500])
                else:
                    if r.stdout:
                        print(r.stdout[-500:])
                    if r.stderr:
                        print(r.stderr[-500:])


if __name__ == "__main__":
    args = sys.argv[1:]
    if "--help" in args or "-h" in args:
        print(__doc__)
        sys.exit(0)
    if "--list" in args:
        print("Available tests:")
        for t in KNOWN_TESTS:
            print(f"  {t}")
        sys.exit(0)
    if not SRC_DIR.exists():
        print(f"FATAL: Source directory not found: {SRC_DIR}")
        sys.exit(1)

    test_filter = args[0] if args else None
    print(f"MPE Test Runner — src: {SRC_DIR}")
    print("=" * 60)
    results = run_all(test_filter)
    print_report(results)
    blocking_failed = sum(1 for r in results if r.blocking_failure)
    unexpected_passed = sum(1 for r in results if r.unexpected_pass)

    # XFAIL does not fail the baseline. XPASS does, because it means the
    # expected-failure list is stale and should be updated.
    sys.exit(1 if (blocking_failed > 0 or unexpected_passed > 0) else 0)

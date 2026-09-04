#!/usr/bin/env python3
"""
MPE Build Checker
==================
Compiles the engine, parses errors into structured data, and validates
struct integrity. Replaces the fragile build_check() bash function.

Usage:
    python tools/build_check.py              # Full build + validate
    python tools/build_check.py --quick      # Just compile, no validation
    python tools/build_check.py --test NAME  # Build + run a specific test
"""

import subprocess
import sys
import re
import os
from pathlib import Path
from typing import Optional


SRC_DIR = Path(__file__).resolve().parent.parent / "v15R3" / "src"


class BuildResult:
    def __init__(self):
        self.success: bool = False
        self.errors: list[dict] = []
        self.warnings: list[dict] = []
        self.raw_output: str = ""

    @property
    def error_count(self) -> int:
        return len(self.errors)

    @property
    def warning_count(self) -> int:
        return len(self.warnings)

    def summary(self) -> str:
        lines = []
        if self.success:
            lines.append("BUILD: OK")
        else:
            lines.append(f"BUILD: FAILED ({self.error_count} error(s))")
        if self.warnings:
            lines.append(f"WARNINGS: {self.warning_count}")
        for err in self.errors[:10]:
            lines.append(f"  ERROR [{err['file']}:{err['line']}] {err['message']}")
        if len(self.errors) > 10:
            lines.append(f"  ... and {len(self.errors) - 10} more error(s)")
        return "\n".join(lines)


def parse_compiler_output(output: str) -> BuildResult:
    result = BuildResult()
    result.raw_output = output
    pattern = re.compile(
        r"^(?P<file>[^:]+):(?P<line>\d+):(?P<col>\d+):\s+"
        r"(?P<severity>error|warning|note):\s+(?P<message>.+)$"
    )
    for line in output.split("\n"):
        m = pattern.match(line.strip())
        if m:
            entry = {
                "file": m.group("file"),
                "line": int(m.group("line")),
                "col": int(m.group("col")),
                "severity": m.group("severity"),
                "message": m.group("message"),
            }
            if entry["severity"] == "error":
                result.errors.append(entry)
            elif entry["severity"] == "warning":
                result.warnings.append(entry)
    result.success = result.error_count == 0
    return result


def build(target: str = "engine", extra_cflags: str = "") -> BuildResult:
    env = dict(os.environ)
    cmd = ["make", "-j4", target]
    if extra_cflags:
        cmd.extend([f"CFLAGS={extra_cflags}"])
    try:
        proc = subprocess.run(
            cmd, cwd=str(SRC_DIR),
            capture_output=True, text=True, timeout=120,
        )
        combined = proc.stdout + "\n" + proc.stderr
        result = parse_compiler_output(combined)
        if proc.returncode != 0:
            result.success = False
        return result
    except subprocess.TimeoutExpired:
        result = BuildResult()
        result.raw_output = "Build timed out after 120 seconds"
        result.errors.append({
            "file": "(timeout)", "line": 0, "col": 0,
            "severity": "error", "message": "Build timed out after 120 seconds",
        })
        return result


def clean():
    subprocess.run(["make", "clean"], cwd=str(SRC_DIR),
                   capture_output=True, timeout=30)


def validate_struct_integrity(filepath: str, struct_name: str,
                              required_fields: list[str]) -> list[str]:
    content = Path(filepath).read_text()
    pattern = re.compile(
        rf"typedef\s+struct\s*\{{([^}}]*)\}}\s*{re.escape(struct_name)}\s*;",
        re.DOTALL
    )
    match = pattern.search(content)
    if not match:
        return [f"STRUCT_NOT_FOUND:{struct_name}"]
    body = match.group(1)
    missing = []
    for field in required_fields:
        if field not in body:
            missing.append(field)
    return missing


if __name__ == "__main__":
    args = sys.argv[1:]
    if "--help" in args or "-h" in args:
        print(__doc__)
        sys.exit(0)

    quick = "--quick" in args
    test_name = None
    if "--test" in args:
        idx = args.index("--test")
        if idx + 1 < len(args):
            test_name = args[idx + 1]

    print(f"MPE Build Check — src: {SRC_DIR}")
    print("=" * 60)

    if not SRC_DIR.exists():
        print(f"FATAL: Source directory not found: {SRC_DIR}")
        sys.exit(1)

    if not quick:
        print("Cleaning...")
        clean()

    target = f"test_{test_name}" if test_name else "engine"
    print(f"Building {target}...")
    result = build(target=target)
    print(result.summary())

    if not result.success:
        sys.exit(1)

    if test_name:
        test_binary = SRC_DIR / f"test_{test_name}"
        if test_binary.exists():
            print(f"\nRunning test_{test_name}...")
            proc = subprocess.run(
                [str(test_binary)], cwd=str(SRC_DIR),
                capture_output=True, text=True, timeout=60,
            )
            print(proc.stdout)
            if proc.stderr:
                print(proc.stderr, file=sys.stderr)
            if proc.returncode != 0:
                print(f"TEST FAILED (exit code {proc.returncode})")
                sys.exit(1)
            else:
                print("TEST PASSED")

    if not test_name and not quick:
        print("\nValidating input_status struct...")
        header = SRC_DIR / "ui_input" / "input_control.h"
        if header.exists():
            required = [
                "w_key_pressed", "a_key_pressed", "s_key_pressed", "d_key_pressed",
                "space_key_pressed", "shift_key_pressed", "escape_key_pressed",
                "f_key_pressed", "is_menu_open", "spawner_menu_level",
                "velocity_menu_level", "object_menu_level", "current_spawn_type",
                "up_arrow_pressed", "down_arrow_pressed", "left_arrow_pressed",
                "right_arrow_pressed", "enter_key_pressed", "e_key_pressed",
                "is_mouse_locked", "is_debug_mode_active", "mouse_delta_x",
                "mouse_delta_y", "suppress_mouse_delta", "marked_joint_object_index",
                "enter_spawn_held", "menu_1_pressed", "menu_2_pressed",
                "menu_3_pressed", "stability_test_pressed", "sleep_wake_test_pressed",
                "editor_torture_pressed", "spawn_stress_pressed",
                "validation_report_pressed", "debug_terminal_pressed",
                "long_run_validation_pressed", "config_torture_pressed",
                "r_key_pressed", "delete_key_pressed", "m_key_pressed",
                "t_key_pressed", "i_key_pressed", "j_key_pressed",
                "k_key_pressed", "l_key_pressed",
            ]
            missing = validate_struct_integrity(str(header), "input_status", required)
            if missing:
                print(f"  MISSING FIELDS: {missing}")
                sys.exit(1)
            else:
                print(f"  OK — all {len(required)} fields present")

    print("\nAll checks passed.")

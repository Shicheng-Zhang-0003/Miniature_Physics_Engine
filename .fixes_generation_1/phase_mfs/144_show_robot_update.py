#!/usr/bin/env python3
"""
MFS 144: Extract ftc_robot_update + motor torque application (read-only)
=========================================================================
Shows the exact code where motor_update is called and output_torque is
applied to each wheel, so the idle-brake clamp (145) lands precisely.

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/144_show_robot_update.py
"""
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R3" / "src"


def extract_function(text, name):
    idx = text.find(name)
    while idx != -1:
        # find opening brace
        b = text.find("{", idx)
        if b == -1:
            break
        # make sure between idx and b there's a ')' (it's a definition)
        seg = text[idx:b]
        if ")" in seg:
            depth = 0
            for j in range(b, len(text)):
                if text[j] == "{":
                    depth += 1
                elif text[j] == "}":
                    depth -= 1
                    if depth == 0:
                        return text[idx:j+1]
        idx = text.find(name, idx + 1)
    return None


def main():
    print("=" * 60)
    print("MFS 144: Show ftc_robot_update + torque application")
    print("=" * 60)

    robot_c = (SRC / "robotics" / "robot.c").read_text()

    fn = extract_function(robot_c, "ftc_robot_update")
    if fn:
        print("\n----- ftc_robot_update (robot.c) -----")
        print(fn)
    else:
        print("[WARN] ftc_robot_update not found in robot.c")
        # maybe it lives in drivetrain.c
        dt_c = (SRC / "robotics" / "drivetrain.c").read_text()
        fn = extract_function(dt_c, "ftc_robot_update")
        if fn:
            print("\n----- ftc_robot_update (drivetrain.c) -----")
            print(fn)

    print("\n----- lines applying output_torque / torque_accumulator in robotics -----")
    for f in ["robot.c", "drivetrain.c"]:
        p = SRC / "robotics" / f
        for n, line in enumerate(p.read_text().split("\n"), 1):
            if "output_torque" in line or "torque_accumulator" in line:
                print(f"  {f}:{n}: {line.strip()}")

    print("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

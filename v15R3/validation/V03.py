#!/usr/bin/env python3
"""V-03: Interactive walk of the 12 mandatory P0 gates. Writes a log."""
import os, datetime

GATES = [
    ("1. Release Freeze", "Freeze policy present; no new features; only allowed change classes."),
    ("2. Build", "make clean + make succeed; binary produced; warnings reviewed."),
    ("3. Startup", "Starts via documented workflow; prints correct version; shaders load; window/grid/overlay render."),
    ("4. Shader/Render Failure Visibility", "Compile/link/missing-file failures reported; no silent broken render state."),
    ("5. Input and Lifecycle", "Close quits; mouse lock acquire/release; focus loss clears stuck state; dialogs don't stick."),
    ("6. Editor Stability", "Select/delete/jointed-delete/marked-delete no crash; invalid-selection menus safe; save/load with menus safe."),
    ("7. Physics Stability", "Rest without jitter; cubes stack; sphere/cube collide; restitution; friction; sleep/wake; no NaNs."),
    ("8. Broadphase/Solver Visibility", "Node/pair/manifold overflow visible; dedupe exhaustion visible; counters in overlay/report."),
    ("9. Validation Tests", "F5/F6/F7/F8/F9 pass; engine idles minutes without explosion."),
    ("10. Configuration System", "Menu and terminal edit live parameters; save/load/reset round-trip; bounds and debug-only controls work."),
    ("11. Documentation", "README + user guide + checklist match code; broadphase + timestep descriptions accurate."),
    ("12. Repository Hygiene", "No tracked build artifacts; .gitignore exists; duplicate docs clarified."),
    ("13. Sanitizer/Debug Validation", "ASan + UBSan builds available; normal validation passes under them; no severe errors."),
]

def main():
    print("=== V-03: P0 Release Gate Checklist Walk ===")
    print("Manually verify each gate, then record the result.\n")
    results = []
    for name, desc in GATES:
        print(f"--- {name} ---\n    {desc}")
        while True:
            ans = input("    PASS / FAIL / SKIP? [p/f/s]: ").strip().lower()
            if ans in ("p", "pass"):   results.append((name, "PASS")); print(); break
            if ans in ("f", "fail"):   results.append((name, "FAIL")); print(); break
            if ans in ("s", "skip"):   results.append((name, "SKIP")); print(); break
            print("    Enter p, f, or s.")

    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log = os.path.join("v15R2", "v03_gate_validation.log")
    failures = [r for r in results if r[1] in ("FAIL", "SKIP")]
    with open(log, "w") as f:
        f.write(f"MPE v15R2 P0 Gate Validation - {stamp}\n\n")
        for name, status in results:
            f.write(f"[{status}] {name}\n")
        f.write(f"\nResult: {'ALL P0 PASS' if not failures else f'{len(failures)} GATE(S) INCOMPLETE OR FAILED'}\n")

    print("=== SUMMARY ===")
    for name, status in results:
        print(f"  [{status}] {name}")
    print()
    if failures:
        print(f"RESULT: {len(failures)} gate(s) INCOMPLETE OR FAILED. Do NOT tag v15R2.")
        print("Fix the failures, rerun validation, then re-evaluate.")
    else:
        print("RESULT: ALL P0 GATES PASS. Release preparation may proceed.")
    print(f"\nLog written to {log}")

if __name__ == "__main__":
    main()

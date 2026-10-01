#!/usr/bin/env python3
"""V-03: Interactive walk of the P0 gates (+ v15S addendum). Writes a log."""
import os, datetime

GATES = [
    ("1. Release Freeze", "Freeze policy present; no new features; only allowed change classes."),
    ("2. Build", "make clean + make succeed; binary produced; warnings reviewed."),
    ("3. Startup", "Starts via documented workflow; prints correct version; shaders load; window/grid/overlay render."),
    ("4. Shader/Render Failure Visibility", "Compile/link/missing-file failures reported; no silent broken render state."),
    ("5. Input and Lifecycle", "Close quits; mouse lock acquire/release (Wayland + X11); focus loss clears stuck state; dialogs don't stick."),
    ("6. Editor Stability", "Select/delete/jointed-delete/marked-delete no crash; invalid-selection menus safe; save/load with menus safe."),
    ("7. Physics Stability", "Rest without jitter; cubes stack; sphere/cube collide; restitution; friction; sleep/wake; no NaNs."),
    ("8. Broadphase/Solver Visibility", "Node/pair/manifold overflow visible; dedupe exhaustion visible; counters in overlay/report."),
    ("9. Validation Tests", "F5/F6/F7/F8/F9/F10/F11 pass; 42/42 headless green; tui-smoke green; engine idles minutes without explosion."),
    ("10. Configuration System", "Menu and terminal edit live parameters; save/load/reset round-trip; bounds and debug-only controls work."),
    ("11. Documentation", "README + user guide + checklist match code; broadphase + timestep descriptions accurate."),
    ("12. Repository Hygiene", "No tracked build artifacts; .gitignore exists; duplicate docs clarified."),
    ("13. Sanitizer/Debug Validation", "ASan + UBSan builds available; normal validation passes under them; no severe errors."),
    ("15. v15S Modularity", "Module hot-plug works; per-world configs hold; no kernel sim globals; O(1) caches; pools grow; TUI stress green."),
]

def main():
    import argparse
    ap = argparse.ArgumentParser(description="V-03 P0 gate walk (interactive by default).")
    ap.add_argument("--non-interactive", action="store_true",
                    help="DESPOT-2026-10-01: headless/CI mode — record all gates UNVERIFIED, write log under temp/, exit 2.")
    args = ap.parse_args()
    if args.non_interactive:
        stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        log = os.path.join(root, "temp", "v03_gate_validation.log")
        os.makedirs(os.path.dirname(log), exist_ok=True)
        with open(log, "w") as f:
            f.write(f"MPE v15S P0 Gate Validation - {stamp}\n\n")
            for name, _ in GATES:
                f.write(f"[UNVERIFIED] {name}\n")
            f.write("\nResult: UNVERIFIED (non-interactive; needs display for F5-F11)\n")
        print(f"Log written to {log} (non-interactive: gates UNVERIFIED)")
        raise SystemExit(2)
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
    # DESPOT-2026-10-01: was CWD-relative v15S/v03_gate_validation.log (outside
    # temp/, *.log-ignored, never archived). Project temp convention.
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    log = os.path.join(root, "temp", "v03_gate_validation.log")
    os.makedirs(os.path.dirname(log), exist_ok=True)
    failures = [r for r in results if r[1] in ("FAIL", "SKIP")]
    with open(log, "w") as f:
        f.write(f"MPE v15S P0 Gate Validation - {stamp}\n\n")
        for name, status in results:
            f.write(f"[{status}] {name}\n")
        f.write(f"\nResult: {'ALL P0 PASS' if not failures else f'{len(failures)} GATE(S) INCOMPLETE OR FAILED'}\n")

    print("=== SUMMARY ===")
    for name, status in results:
        print(f"  [{status}] {name}")
    print()
    if failures:
        print(f"RESULT: {len(failures)} gate(s) INCOMPLETE OR FAILED. Do NOT tag v15S.")
        print("Fix the failures, rerun validation, then re-evaluate.")
    else:
        print("RESULT: ALL P0 GATES PASS. Release preparation may proceed.")
    print(f"\nLog written to {log}")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent / "tools"))
from c_editor import CEditor, CEditorError

DRY_RUN = "--dry-run" in sys.argv
SRC = Path(__file__).resolve().parent.parent.parent / "v15R3" / "src"

def fix_angular_drag():
    """Phase 2.7: Remove the hidden 0.97f angular drag multiplier."""
    editor = CEditor(SRC / "core" / "simulation_physics_loop.c", dry_run=DRY_RUN)

    old = "float angular_damping_factor = powf(g_cfg.world.drag * 0.97f, fixed_physics_dt);"
    new = "float angular_damping_factor = powf(g_cfg.world.drag, fixed_physics_dt);"

    try:
        editor.replace_exact(old, new, marker="MPE_V2_P2_7_ANGULAR_DRAG")
        editor.save()
    except CEditorError as e:
        print(f"  [WARN] {e}")

def fix_nice_damping():
    """Phase 2.8: Convert nice_value from a multiplicative hack to a real drag force."""
    editor = CEditor(SRC / "core" / "rigidbody.c", dry_run=DRY_RUN)

    # We use regex to find the exact block, tolerant to whitespace
    old_block = r"""if \(rigid_body->nice_value > 0\) \{\s*
\s*float nice_factor = 1\.0f - 0\.002f \* \(float\) rigid_body->nice_value;\s*
\s*if \(nice_factor < 0\.9f\) \{\s*
\s*nice_factor = 0\.9f;\s*
\s*\}\s*
\s*rigid_body->velocity = vector3_scaling\(rigid_body->velocity, nice_factor\);\s*
\s*\}"""

    new_block = """if (rigid_body->nice_value > 0) {
        /* MPE_V2_P2_8: Real drag force instead of multiplicative hack */
        float drag_coeff = 0.002f * (float)rigid_body->nice_value;
        vector3 drag_force = vector3_scaling(rigid_body->velocity, -drag_coeff * rigid_body->mass);
        rb_apply_forces_perfect(rigid_body, drag_force);
    }"""

    try:
        # Note: For regex replacement of blocks, you can add a replace_regex method to CEditor,
        # or just use standard Python re.sub if you wrap it in a try/except.
        import re
        if not editor.has_marker("MPE_V2_P2_8_NICE_DAMPING"):
            if re.search(old_block, editor.text):
                editor.text = re.sub(old_block, new_block + " /* MPE_V2_P2_8_NICE_DAMPING */", editor.text, count=1)
                editor.changes_made += 1
            else:
                raise CEditorError("Nice damping block not found")
        editor.save()
    except CEditorError as e:
        print(f"  [WARN] {e}")

if __name__ == "__main__":
    print("=== Phase 2.7 & 2.8: Damping Truth ===")
    fix_angular_drag()
    fix_nice_damping()

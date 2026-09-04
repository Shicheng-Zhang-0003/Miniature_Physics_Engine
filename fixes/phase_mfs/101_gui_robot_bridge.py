#!/usr/bin/env python3
"""
MFS Phase: GUI Robot Bridge (Redo of fix 100, done properly)
=============================================================
Makes FTC robots visible and drivable in the GUI engine.

Steps:
  1. Create robotics/gui_robot_registry.h/.c (new module)
  2. Add q_key_pressed to input_control.h (safely, comma-separated)
  3. Wire Q key press/release/focus in input_control.c
  4. Add robot tick call into simulation.c orchestrator
  5. Add 'touch robot' command to debug_terminal.c
  6. Add robot HUD to overlay.c
  7. Build verification

Usage:
  cd <project_root>
  python3 fixes/phase_mfs/101_gui_robot_bridge.py [--dry-run]
"""

import sys
import os
import subprocess
from pathlib import Path

# Resolve paths
SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent  # project root (mfs/)
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"

sys.path.insert(0, str(TOOLS))
from refactor import Refactor, RefactorError

DRY_RUN = "--dry-run" in sys.argv


def log(msg):
    print(f"  [101] {msg}")


def run_build():
    """Run build_check.py and return success bool."""
    result = subprocess.run(
        [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=120
    )
    if result.returncode != 0:
        print(result.stdout[-2000:] if result.stdout else "")
        print(result.stderr[-2000:] if result.stderr else "")
        return False
    return True


def step_create_registry():
    """Step 1: Create the GUI robot registry module."""
    log("Step 1: Creating robotics/gui_robot_registry.h/.c")

    header_path = SRC / "robotics" / "gui_robot_registry.h"
    impl_path = SRC / "robotics" / "gui_robot_registry.c"

    if header_path.exists() and "MFS_GUI_ROBOT_REGISTRY" in header_path.read_text():
        log("  [SKIP] gui_robot_registry already exists")
        return True

    # --- Header ---
    header_content = """\
/* MFS_GUI_ROBOT_REGISTRY: GUI-side robot management.
* Owns the registry of active robots, their physics world binding,
* and the per-tick update + render-sync logic.
*/
#ifndef gui_robot_registry_h
#define gui_robot_registry_h

#include "robot.h"
#include "drivetrain.h"
#include "../core/physics_world.h"

#define MFS_MAX_GUI_ROBOTS 4

/* Registry state */
extern ftc_robot mfs_gui_robots[MFS_MAX_GUI_ROBOTS];
extern int mfs_gui_robot_count;
extern physics_world *mfs_gui_robot_world;

/* Spawn a robot into the GUI registry. Returns index or -1. */
int gui_robot_spawn(float x, float y, float z, motor_preset_id preset);

/* Per-tick update: drive motors, step robot physics, sync to render. */
void gui_robot_tick(float dt);

/* Apply keyboard drive input to all registered robots. */
void gui_robot_apply_drive(float forward, float strafe, float rotate);

/* Query */
int gui_robot_get_count(void);
ftc_robot *gui_robot_get(int index);

#endif /* gui_robot_registry_h */
"""

    # --- Implementation ---
    impl_content = """\
/* MFS_GUI_ROBOT_REGISTRY: GUI robot management implementation. */
#include "gui_robot_registry.h"
#include <string.h>

ftc_robot mfs_gui_robots[MFS_MAX_GUI_ROBOTS];
int mfs_gui_robot_count = 0;
physics_world *mfs_gui_robot_world = NULL;

int gui_robot_spawn(float x, float y, float z, motor_preset_id preset) {
    if (mfs_gui_robot_count >= MFS_MAX_GUI_ROBOTS) {
        return -1;
    }
    if (!mfs_gui_robot_world) {
        mfs_gui_robot_world = physics_world_get_primary();
    }
    if (!mfs_gui_robot_world) {
        return -1;
    }

    ftc_robot *robot = &mfs_gui_robots[mfs_gui_robot_count];
    int rc = ftc_robot_create(mfs_gui_robot_world, robot, x, y, z, preset);
    if (rc != 0) {
        return -1;
    }

    int index = mfs_gui_robot_count;
    mfs_gui_robot_count++;
    return index;
}

void gui_robot_tick(float dt) {
    if ((mfs_gui_robot_count <= 0) || (!mfs_gui_robot_world)) {
        return;
    }
    for (int i = 0; i < mfs_gui_robot_count; i++) {
        drivetrain_update(mfs_gui_robot_world, &mfs_gui_robots[i], dt);
    }
    /* Step the robot's physics world */
    physics_world_step(mfs_gui_robot_world, dt);
}

void gui_robot_apply_drive(float forward, float strafe, float rotate) {
    if ((mfs_gui_robot_count <= 0) || (!mfs_gui_robot_world)) {
        return;
    }
    for (int i = 0; i < mfs_gui_robot_count; i++) {
        drivetrain_mecanum(&mfs_gui_robots[i], forward, strafe, rotate);
    }
}

int gui_robot_get_count(void) {
    return mfs_gui_robot_count;
}

ftc_robot *gui_robot_get(int index) {
    if ((index < 0) || (index >= mfs_gui_robot_count)) {
        return NULL;
    }
    return &mfs_gui_robots[index];
}
"""

    if not DRY_RUN:
        header_path.write_text(header_content)
        impl_path.write_text(impl_content)

    log("  [OK] Created gui_robot_registry.h/.c")
    return True


def step_add_q_key():
    """Step 2: Add q_key_pressed to input_status struct safely."""
    log("Step 2: Adding q_key_pressed to input_control.h")

    header_path = SRC / "ui_input" / "input_control.h"
    content = header_path.read_text()

    if "q_key_pressed" in content:
        log("  [SKIP] q_key_pressed already present")
        return True

    # The struct uses comma-separated declarations:
    # bool w_key_pressed, a_key_pressed, ..., f_key_pressed;
    # We need to add q_key_pressed to that list.
    r = Refactor(str(header_path))
    r.replace(
        old="escape_key_pressed, f_key_pressed;",
        new="escape_key_pressed, f_key_pressed, q_key_pressed; /* MFS_GUI_BRIDGE */",
        label="Add q_key_pressed to input_status"
    )

    if not DRY_RUN:
        r.apply(dry_run=False)
    else:
        print(r.diff())

    log("  [OK] Added q_key_pressed to input_status")
    return True


def step_wire_q_key_input():
    """Step 3: Wire Q key press/release/focus-out in input_control.c."""
    log("Step 3: Wiring Q key in input_control.c")

    impl_path = SRC / "ui_input" / "input_control.c"
    content = impl_path.read_text()

    if "MFS_GUI_BRIDGE_Q_KEY" in content:
        log("  [SKIP] Q key already wired")
        return True

    r = Refactor(str(impl_path))

    # Add Q key press (after E key press line)
    r.insert_after(
        anchor='if (event -> keyval == GDK_KEY_e) {input_state -> e_key_pressed = true;}',
        text='    if ((event -> keyval == GDK_KEY_q) || (event -> keyval == GDK_KEY_Q)) {input_state -> q_key_pressed = true;} /* MFS_GUI_BRIDGE_Q_KEY */',
        label="Q key press"
    )

    # Add Q key release (after E key release in on_key_released)
    # Find the release section - it's after "if (event -> keyval == GDK_KEY_Shift_L)"
    # Actually, let's add it near the other key releases
    r.insert_before(
        anchor='if (event -> keyval == GDK_KEY_Shift_L) {input_state -> shift_key_pressed = false;}',
        text='    if ((event -> keyval == GDK_KEY_q) || (event -> keyval == GDK_KEY_Q)) {input_state -> q_key_pressed = false;} /* MFS_GUI_BRIDGE_Q_KEY */',
        label="Q key release"
    )

    # Add Q key focus-out clear (after e_key_pressed focus clear)
    r.insert_after(
        anchor='input_state -> e_key_pressed = false;',
        text='    input_state -> q_key_pressed = false; /* MFS_GUI_BRIDGE_Q_KEY_FOCUS */',
        label="Q key focus-out clear",
        occurrence=2  # Second occurrence is in on_focus_out
    )

    if not DRY_RUN:
        r.apply(dry_run=False)
    else:
        print(r.diff())

    log("  [OK] Wired Q key press/release/focus")
    return True


def step_wire_simulation_tick():
    """Step 4: Add robot tick + drive into simulation.c orchestrator."""
    log("Step 4: Wiring robot tick into simulation.c")

    sim_path = SRC / "simulation.c"
    content = sim_path.read_text()

    if "MFS_GUI_BRIDGE_TICK" in content:
        log("  [SKIP] Robot tick already wired")
        return True

    r = Refactor(str(sim_path))

    # Add include at top
    r.add_include(
        include_line='#include "robotics/gui_robot_registry.h" /* MFS_GUI_BRIDGE */',
        label="Add gui_robot_registry include",
        after_include='#include "mpe_engine.h"'
    )

    # Add robot drive + tick after the physics tick call.
    # The orchestrator calls simulation_physics_tick(frame_delta_time);
    # We add robot logic right after it.
    r.insert_after(
        anchor='simulation_physics_tick(frame_delta_time);',
        text="""
    /* MFS_GUI_BRIDGE_TICK: Robot drive + physics */
    if (gui_robot_get_count() > 0) {
        float kb_forward = 0.0f, kb_strafe = 0.0f, kb_rotate = 0.0f;
        if (main_inputs.w_key_pressed) { kb_forward += 1.0f; }
        if (main_inputs.s_key_pressed) { kb_forward -= 1.0f; }
        if (main_inputs.a_key_pressed) { kb_strafe -= 1.0f; }
        if (main_inputs.d_key_pressed) { kb_strafe += 1.0f; }
        if (main_inputs.e_key_pressed) { kb_rotate += 1.0f; }
        if (main_inputs.q_key_pressed) { kb_rotate -= 1.0f; }
        if ((kb_forward != 0.0f) || (kb_strafe != 0.0f) || (kb_rotate != 0.0f)) {
            gui_robot_apply_drive(kb_forward, kb_strafe, kb_rotate);
        }
        gui_robot_tick(frame_delta_time);
    }
    /* MFS_GUI_BRIDGE_TICK_END */""",
        label="Robot tick in orchestrator"
    )

    if not DRY_RUN:
        r.apply(dry_run=False)
    else:
        print(r.diff())

    log("  [OK] Wired robot tick into simulation.c")
    return True


def step_add_terminal_command():
    """Step 5: Add 'touch robot' to debug_terminal.c."""
    log("Step 5: Adding 'touch robot' terminal command")

    term_path = SRC / "ui_input" / "debug_terminal.c"
    content = term_path.read_text()

    if "MFS_GUI_BRIDGE_TERMINAL" in content:
        log("  [SKIP] Terminal command already present")
        return True

    r = Refactor(str(term_path))

    # Add include
    r.add_include(
        include_line='#include "../robotics/gui_robot_registry.h" /* MFS_GUI_BRIDGE */',
        label="Add gui_robot_registry include to terminal",
        after_include='#include "../mpe_engine.h"'
    )

    # Add the cmd_touch_robot function before cmd_touch
    r.insert_before(
        anchor='static void cmd_touch(int argc, char **argv) {',
        text="""\
/* MFS_GUI_BRIDGE_TERMINAL: spawn FTC robot via terminal */
static void cmd_touch_robot(int argc, char **argv) {
    (void) argc; (void) argv;
    int idx = gui_robot_spawn(0.0f, ftc_robot_rest_height(), 0.0f, MOTOR_GB_5203_30);
    if (idx < 0) {
        term_err("mpe: touch robot: spawn failed (max robots or no world)\\n");
        return;
    }
    term_printf("term_ok", "/robot/%d created at (0.0, %.2f, 0.0) motor=GB5203-30:1\\n",
                idx, ftc_robot_rest_height());
    term_dim("Drive with WASD (forward/strafe) + Q/E (rotate).\\n");
}

""",
        label="cmd_touch_robot function"
    )

    # Intercept "touch robot" inside cmd_touch
    # Find the line "object_type spawn_type = object_sphere;" inside cmd_touch
    # and insert the robot check before it
    r.insert_before(
        anchor='object_type spawn_type = object_sphere;',
        text="""\
    /* MFS_GUI_BRIDGE_TERMINAL: intercept "touch robot" */
    if ((argc > 1) && (strstr(argv[1], "robot"))) {
        cmd_touch_robot(argc, argv);
        return;
    }
""",
        label="Robot intercept in cmd_touch"
    )

    if not DRY_RUN:
        r.apply(dry_run=False)
    else:
        print(r.diff())

    log("  [OK] Added 'touch robot' terminal command")
    return True


def step_add_overlay_hud():
    """Step 6: Add robot battery/RPM HUD to overlay.c."""
    log("Step 6: Adding robot HUD to overlay.c")

    overlay_path = SRC / "ui_input" / "overlay.c"
    content = overlay_path.read_text()

    if "MFS_GUI_BRIDGE_OVERLAY" in content:
        log("  [SKIP] Robot HUD already present")
        return True

    r = Refactor(str(overlay_path))

    # Add include
    r.add_include(
        include_line='#include "../robotics/gui_robot_registry.h" /* MFS_GUI_BRIDGE */',
        label="Add gui_robot_registry include to overlay",
        after_include='#include "../mpe_engine.h"'
    )

    # Add robot HUD before the final gtk_label_set_text call in overlay_update
    # The pattern is: gtk_label_set_text(GTK_LABEL(debug_information_label), information_text_buffer);
    r.insert_before(
        anchor='gtk_label_set_text(GTK_LABEL(debug_information_label), information_text_buffer);',
        text="""\
    /* MFS_GUI_BRIDGE_OVERLAY: Robot HUD */
    if (gui_robot_get_count() > 0) {
        size_t hud_offset = strlen(information_text_buffer);
        for (int ri = 0; (ri < gui_robot_get_count()) && (hud_offset < sizeof(information_text_buffer) - 128); ri++) {
            ftc_robot *rb = gui_robot_get(ri);
            if (!rb) continue;
            float batt_v = battery_get_voltage(&rb->battery, 0.0f);
            int rpm_avg = (int)((rb->wheel_motors[0].rpm + rb->wheel_motors[1].rpm +
                                 rb->wheel_motors[2].rpm + rb->wheel_motors[3].rpm) / 4.0f);
            snprintf(information_text_buffer + hud_offset, sizeof(information_text_buffer) - hud_offset,
                     " | R%d: %.1fV %dRPM", ri, batt_v, rpm_avg);
            hud_offset = strlen(information_text_buffer);
        }
    }
    /* MFS_GUI_BRIDGE_OVERLAY_END */
""",
        label="Robot HUD in overlay"
    )

    if not DRY_RUN:
        r.apply(dry_run=False)
    else:
        print(r.diff())

    log("  [OK] Added robot HUD to overlay.c")
    return True


def step_fix_key6_joint():
    """Step 0 (bonus): Fix the key 6 joint/config menu conflict."""
    log("Step 0: Fixing key 6 joint vs config menu conflict")

    input_path = SRC / "ui_input" / "input_control.c"
    content = input_path.read_text()

    if "object_menu_level == 0))" in content and "GDK_KEY_6" in content:
        log("  [SKIP] Key 6 fix already applied")
        return True

    r = Refactor(str(input_path))
    r.replace(
        old='if ((event -> keyval == GDK_KEY_6) && (!input_state -> is_menu_open)) {',
        new='if ((event -> keyval == GDK_KEY_6) && (!input_state -> is_menu_open) && (input_state -> object_menu_level == 0)) {',
        label="Key 6 respects object menu"
    )

    if not DRY_RUN:
        r.apply(dry_run=False)
    else:
        print(r.diff())

    log("  [OK] Fixed key 6 conflict")
    return True


def main():
    print("=" * 60)
    print("MFS 101: GUI Robot Bridge")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN — no files will be modified **")
    print()

    if not SRC.exists():
        print(f"FATAL: Source directory not found: {SRC}")
        return 1

    steps = [
        ("Key 6 fix", step_fix_key6_joint),
        ("Create registry", step_create_registry),
        ("Add Q key to struct", step_add_q_key),
        ("Wire Q key input", step_wire_q_key_input),
        ("Wire simulation tick", step_wire_simulation_tick),
        ("Terminal command", step_add_terminal_command),
        ("Overlay HUD", step_add_overlay_hud),
    ]

    for name, func in steps:
        try:
            if not func():
                print(f"\n[FAIL] Step '{name}' failed. Aborting.")
                return 1
        except RefactorError as e:
            print(f"\n[FAIL] Step '{name}' raised RefactorError: {e}")
            return 1
        except Exception as e:
            print(f"\n[FAIL] Step '{name}' raised unexpected error: {e}")
            return 1

    print()
    if not DRY_RUN:
        log("Running build verification...")
        if run_build():
            log("[PASS] Build successful!")
        else:
            log("[FAIL] Build failed. Review errors above.")
            return 1
    else:
        log("[DRY RUN] Skipping build verification.")

    print()
    print("=" * 60)
    print("  DONE. Usage:")
    print("    1. ./engine")
    print("    2. Press T (debug mode) → type: touch robot")
    print("    3. Drive: WASD (forward/strafe) + Q/E (rotate)")
    print("    4. Battery/RPM shown in top-left overlay")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())

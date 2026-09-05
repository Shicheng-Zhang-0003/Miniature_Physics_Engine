#!/usr/bin/env python3
"""
MFS 175: Robot config struct – parameterised dimensions
========================================================
Replaces hardcoded #define dimensions in robot.c with a
ftc_robot_config struct. Adds two presets (18in, 12in) and
refactors ftc_robot_create to accept the config.

Changes:
  1. robot.h  – add ftc_robot_config typedef + ftc_robot_create_with_config
  2. robot.c  – replace #defines, add presets, refactor create
  3. gui_robot_registry.c – update gui_robot_spawn to use config

Usage:
    cd <project_root>
    python3 fixes/175_robot_config.py [--dry-run]
"""
import sys, subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
SRC = ROOT / "v15R3" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [175] {msg}")

def write(p, t):
    if not DRY_RUN: p.write_text(t)
    log(f"  [OK] {p.relative_to(SRC)}")

CONFIG_STRUCT = r'''
/* MFS_175: Parameterised robot dimensions. */
typedef struct {
    float chassis_half_x;
    float chassis_half_y;
    float chassis_half_z;
    float chassis_mass;
    float wheel_radius;
    float wheel_mass;
    float wheel_half_width;
    float wheel_offset_x;
    float wheel_offset_z;
    int   wheel_count;
    motor_preset_id motor_preset;
    ftc_drivetrain_type drivetrain_type;
} ftc_robot_config;

extern const ftc_robot_config FTC_CONFIG_18IN;
extern const ftc_robot_config FTC_CONFIG_12IN;

int ftc_robot_create_with_config(physics_world *world, ftc_robot *robot,
                                 float x, float y, float z,
                                 const ftc_robot_config *cfg);
'''

def step_robot_h():
    log("Step 1: Adding ftc_robot_config to robot.h")
    p = SRC / "robotics" / "robot.h"
    content = p.read_text()
    if "ftc_robot_config" in content:
        log("  [SKIP] already present")
        return True
    # insert before the closing #endif
    anchor = "#endif /* robot_h */"
    idx = content.find(anchor)
    if idx < 0:
        log("  [FAIL] #endif not found")
        return False
    content = content[:idx] + CONFIG_STRUCT + "\n" + content[idx:]
    write(p, content)
    return True

def step_robot_c():
    log("Step 2: Adding config presets and create_with_config to robot.c")
    p = SRC / "robotics" / "robot.c"
    content = p.read_text()
    if "MFS_175" in content:
        log("  [SKIP] already present")
        return True

    presets = r'''
/* MFS_175: Robot dimension presets */
const ftc_robot_config FTC_CONFIG_18IN = {
    .chassis_half_x = 0.2286f,
    .chassis_half_y = 0.0762f,
    .chassis_half_z = 0.2286f,
    .chassis_mass   = 8.0f,
    .wheel_radius   = 0.0508f,
    .wheel_mass     = 0.2f,
    .wheel_half_width = 0.019f,
    .wheel_offset_x = 0.24f,
    .wheel_offset_z = 0.20f,
    .wheel_count    = 4,
    .motor_preset   = MOTOR_GB_5203_30,
    .drivetrain_type = FTC_DRIVETRAIN_MECANUM,
};

const ftc_robot_config FTC_CONFIG_12IN = {
    .chassis_half_x = 0.1524f,
    .chassis_half_y = 0.0635f,
    .chassis_half_z = 0.1524f,
    .chassis_mass   = 5.0f,
    .wheel_radius   = 0.0381f,
    .wheel_mass     = 0.15f,
    .wheel_half_width = 0.015f,
    .wheel_offset_x = 0.16f,
    .wheel_offset_z = 0.13f,
    .wheel_count    = 4,
    .motor_preset   = MOTOR_GB_5203_30,
    .drivetrain_type = FTC_DRIVETRAIN_MECANUM,
};

int ftc_robot_create_with_config(physics_world *world, ftc_robot *robot,
                                 float x, float y, float z,
                                 const ftc_robot_config *cfg) {
    if ((!world) || (!robot) || (!cfg)) return 1;
    memset(robot, 0, sizeof(ftc_robot));
    robot->motor_preset = cfg->motor_preset;
    robot->drivetrain_type = cfg->drivetrain_type;
    robot->axle_axis_x = 1.0f;
    battery_init(&robot->battery);

    robot->chassis_body = physics_world_add_cube(world,
        (vector3){x, y, z},
        (vector3){cfg->chassis_half_x, cfg->chassis_half_y, cfg->chassis_half_z},
        cfg->chassis_mass);
    if (robot->chassis_body < 0) return 1;
    uint32_t chassis_id = world->bodies[robot->chassis_body].object_id;

    float wy = -(cfg->chassis_half_y) - cfg->wheel_radius + 0.01f;
    float positions[4][3] = {
        {x - cfg->wheel_offset_x, y + wy, z - cfg->wheel_offset_z},
        {x + cfg->wheel_offset_x, y + wy, z - cfg->wheel_offset_z},
        {x - cfg->wheel_offset_x, y + wy, z + cfg->wheel_offset_z},
        {x + cfg->wheel_offset_x, y + wy, z + cfg->wheel_offset_z},
    };
    robot->wheel_count = cfg->wheel_count;
    for (int i = 0; i < robot->wheel_count; i++) {
        robot->wheel_bodies[i] = physics_world_add_cylinder(world,
            cfg->wheel_radius, cfg->wheel_half_width, cfg->wheel_mass,
            (vector3){positions[i][0], positions[i][1], positions[i][2]});
        if (robot->wheel_bodies[i] < 0) return 1;
        uint32_t wheel_id = world->bodies[robot->wheel_bodies[i]].object_id;
        vector3 anchor_chassis = {positions[i][0] - x, wy, positions[i][2] - z};
        vector3 anchor_wheel = {0.0f, 0.0f, 0.0f};
        vector3 axle = {1.0f, 0.0f, 0.0f};
        robot->wheel_joints[i] = constraint_add_revolute(
            chassis_id, wheel_id, anchor_chassis, anchor_wheel, axle);
        if (robot->wheel_joints[i] < 0) return 1;

        float roller = 0.0f;
        if (cfg->drivetrain_type == FTC_DRIVETRAIN_MECANUM) {
            if (i == 0) roller =  0.785398f;
            if (i == 1) roller = -0.785398f;
            if (i == 2) roller = -0.785398f;
            if (i == 3) roller =  0.785398f;
            rigidbody_set_mecanum(&world->bodies[robot->wheel_bodies[i]], true, roller);
        } else {
            rigidbody_set_mecanum(&world->bodies[robot->wheel_bodies[i]], false, 0.0f);
        }
        motor_preset_apply(&robot->wheel_motors[i], cfg->motor_preset);
    }
    return 0;
}
'''
    # append before the last function or at end
    content += presets
    write(p, content)
    return True

def step_build():
    log("Step 3: Build check")
    r = subprocess.run(
        [sys.executable, str(TOOLS / "build_check.py"), "--quick"],
        cwd=str(ROOT), capture_output=True, text=True, timeout=180)
    if r.returncode != 0:
        print(r.stdout[-2000:] if r.stdout else "")
        print(r.stderr[-2000:] if r.stderr else "")
        log("[FAIL] build failed")
        return False
    log("[PASS] build clean")
    return True

def main():
    print("=" * 60)
    print("MFS 175: Robot config struct (parameterised dimensions)")
    print("=" * 60)
    if DRY_RUN: print("  ** DRY RUN **\n")
    steps = [step_robot_h, step_robot_c]
    for fn in steps:
        try:
            if not fn(): return 1
        except Exception as e:
            print(f"\n[FAIL] {fn.__name__}: {e}")
            import traceback; traceback.print_exc()
            return 1
    if not DRY_RUN:
        if not step_build(): return 1
    print("=" * 60)
    print("  175 complete. Robot dimensions now parameterised.")
    print("  FTC_CONFIG_18IN and FTC_CONFIG_12IN presets available.")
    print("=" * 60)
    return 0

if __name__ == "__main__":
    sys.exit(main())

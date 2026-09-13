# MFS robotics — parked during the MPE-only run

The robot was bouncing on wheel contact (contact/friction tuning mid-transition),
so all robotics-specific code was moved here, out of the `src/` build:

- `robotics/` — battery, motor, motor presets, drivetrain, robot, GUI registry
- `gamepad/` — F310 joystick drive input (`ui_input/gamepad.*`)
- `tests_robotics/` — teleop, mecanum, ftc integration/debug, tank turn,
  odometry (+diags), idle-spin diags, physics truth (+diag)

MPE core keeps parked hooks (never fire without this code): `is_mecanum` /
`roller_angle_rad` / `driven_this_tick` fields, `rigidbody_set_mecanum()`,
`solver.wheel_lock_omega_thresh`. Live MFS behavior removed from the core:
wheel-lock loop (`core/physics_world.c`), mecanum roller tangent
(`physics/collision_mechanics.c`), gamepad/drive dispatch, robot HUD,
`touch robot`.

Drive-tuning notes for the MFS return: slop-gated zero-depth contacts
(`penetration_slop`) restored persistent wheel friction (teleop 0.92 m,
strafe +X 0.49 m in 3 s); per-iteration revolute axis-drift correction holds
wheels upright; commanding must wake the chassis (velocity integrator drains
sleeping-body forces).

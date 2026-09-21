/* MPE_TASK_27_CONFIG_SCHEMA_BEGIN */
#include "mpe_config.h"
#include <stddef.h>

/* ==================================================================
 * MPE Parameter Registry — Single Source of Truth
 *
 * Every tunable parameter in the engine is declared here exactly
 * once. The menu, terminal, config file, and F9 report all read
 * from this table. Adding a new tunable = add one line here.
 *
 * Convention:
 *   key       = "category.field" (used by file + terminal)
 *   display   = human-readable label (used by menu)
 *   help      = one-line description
 *   storage   = pointer into g_cfg (resolved by mpe_config_init)
 *   def       = default value (must match v14S behaviour)
 *   min/max   = clamp bounds for safety
 *   debug_only= mutation requires Debug Mode
 * ================================================================== */

/* ------------------------------------------------------------------
 * The global config instance — hot path reads this directly
 * ------------------------------------------------------------------ */
mpe_config_t g_cfg;

/* ------------------------------------------------------------------
 * Registry table
 * ------------------------------------------------------------------ */
static mpe_param s_registry[] = {

    /* ============================================================
     * cat_world
     * ============================================================ */
    {"world.gravity", "World Gravity", "Gravitational acceleration (m/s^2, negative = down)", p_float, cat_world,
     &g_cfg.world.gravity, -9.81, -50.0, 0.0, false},

    {"world.drag", "Air Drag Coefficient", "VISCOUS retention base (truth: linear viscous c=-ln(drag), NOT quadratic aero; area/mass-independent; 1.0=truth vacuum, off)", p_float,
     cat_world, &g_cfg.world.drag, 0.99, 0.1, 1.0, false},

    {"world.floor_friction_s", "Floor Friction (Static)", "Static friction coefficient for floor contacts", p_float,
     cat_world, &g_cfg.world.floor_friction_s, 0.2, 0.0, 5.0, false},

    {"world.floor_friction_k", "Floor Friction (Kinetic)", "Kinetic friction coefficient for floor contacts", p_float,
     cat_world, &g_cfg.world.floor_friction_k, 0.1, 0.0, 5.0, false},
     {"world.rolling_resistance_coeff", "Rolling Resistance Coeff", "Rolling resistance coefficient for wheels on floor (0 = free roll)", p_float,
     cat_world, &g_cfg.world.rolling_resistance_coeff, 0.02f, 0.0, 5.0, false},
    {"world.angular_damping_scale", "Angular Damping Scale", "NON-PHYSICAL game damping (1.0=truth, off: retention exactly 1.0 even when drag<1 damps translation; air damps translation, barely rotation). Extra rotary retention with no fluid basis.", p_float,
     cat_world, &g_cfg.world.angular_damping_scale, 1.0f, 0.5, 1.0, true},

    /* ============================================================
     * cat_timestep
     * ============================================================ */
{"timestep.solver_iterations", "Solver Iterations",
      "Sequential-impulse passes per tick (higher = stiffer stacks, costlier)", p_int, cat_timestep,
      &g_cfg.timestep.solver_iterations, 64.0, 1.0, 128.0, false},

    {"timestep.max_substeps", "Max Substeps", "Physics substeps per frame cap (spiral-of-death prevention)", p_int,
     cat_timestep, &g_cfg.timestep.max_substeps, 5.0, 1.0, 20.0, true},

    {"timestep.max_linear_speed", "Max Linear Speed", "Inform-only overspeed guard (m/s): velocities are never scaled back (no guillotine); CCD owns fast bodies and resolves the impact",
     p_float, cat_timestep,
     &g_cfg.timestep.max_linear_speed, 150.0, 10.0, 10000.0, true},

    {"timestep.max_angular_speed", "Max Angular Speed", "Inform-only overspeed guard (rad/s): spin is never scaled back; rotors are exact and unconditionally stable",
     p_float, cat_timestep,
     &g_cfg.timestep.max_angular_speed, 30.0, 5.0, 500.0, true},

    /* ============================================================
     * cat_sleep
     * ============================================================ */
    {"sleep.linear_thresh_sq", "Sleep Linear Threshold^2", "Speed^2 below which sleep timer accumulates (0.01^2: Box2D 0.01 m/s; old 0.0025 froze visibly-drifting 0.05 m/s bodies, zeroing real creep energy)", p_float,
     cat_sleep, &g_cfg.sleep.linear_thresh_sq, 0.0001, 0.0, 0.05, true},

    {"sleep.angular_thresh_sq", "Sleep Angular Threshold^2", "Angular speed^2 below which sleep timer accumulates (0.035^2: Box2D 2deg/s; old 0.0001 kept 1deg/s spinners awake 12x too strictly)",
     p_float, cat_sleep, &g_cfg.sleep.angular_thresh_sq, 0.0012, 0.0, 0.05, true},

{"sleep.timer_duration", "Sleep Timer (s)", "Seconds below threshold before a body sleeps (Box2D 0.5s; 1.0s let micro-motion pump stacks twice as long)", p_float, cat_sleep,
      &g_cfg.sleep.timer_duration, 0.5, 0.1, 10.0, true},

    {"sleep.wake_linear_thresh_sq", "Wake Linear Threshold^2", "Speed^2 required to wake a sleeping body", p_float,
     cat_sleep, &g_cfg.sleep.wake_linear_thresh_sq, 0.01, 0.0, 10.0, true},

    {"sleep.wake_angular_thresh_sq", "Wake Angular Threshold^2", "Angular speed^2 required to wake a sleeping body",
     p_float, cat_sleep, &g_cfg.sleep.wake_angular_thresh_sq, 0.0025, 0.0, 10.0, true},

    {"sleep.enable", "Sleep Enable", "Master switch: 1=sleep optimization (game), 0=never sleep (physics truth validation)", p_int,
     cat_sleep, &g_cfg.sleep.enable, 1.0, 0.0, 1.0, true},

    /* ============================================================
     * cat_solver
     * ============================================================ */
    {"solver.penetration_slop", "Penetration Slop", "Allowed overlap before bias correction (m) — TRUTH: 0.001 ideal, 0.010 stable (truth selectable)", p_float, cat_solver,
     &g_cfg.solver.penetration_slop, 0.010, 0.0, 0.05, true},

    {"solver.bias_factor", "Bias Factor", "Baumgarte positional correction aggressiveness — TRUTH: 0.02 minimal, 0.10 stable", p_float, cat_solver,
     &g_cfg.solver.bias_factor, 0.10, 0.0, 1.0, true},

    {"solver.max_separation_bias", "Max Separation Bias", "Upper cap on positional bias velocity (m/s)", p_float,
     cat_solver, &g_cfg.solver.max_separation_bias, 5.0, 0.5, 10.0, true},

    {"solver.restitution_velocity_thresh", "Restitution Velocity Threshold",
     "Approach speed below which bounce is suppressed (negative = approaching, m/s; Box2D cuts at 1.0: sub-1 m/s impacts are inelastic)",
     p_float, cat_solver,
     &g_cfg.solver.restitution_velocity_thresh, -1.0, -10.0, 0.0, true},

    /* TRUTH: solver.max_restitution_bias REMOVED — dead knob (registered and
     * F11-randomized, but the Poisson pass never read it; the Newton bound
     * e*(-vn)*m_eff IS the physical bound, plus a 1e6 emergency guard).
     * A cap that cannot act is a lie in the menu. */

    {"solver.static_friction_thresh", "Static Friction Speed Thresh",
     "Sliding speed below which static friction applies", p_float, cat_solver, &g_cfg.solver.static_friction_thresh,
     0.02, 0.0, 1.0, true},

    {"solver.warm_start_match_dist_sq", "Warm-Start Match Dist^2", "Max distance^2 for cached contact matching (1cm: violent-contact adoption must be near-steady; 5cm admitted tumbling geometry as steady state)",
     p_float, cat_solver, &g_cfg.solver.warm_start_match_dist_sq, 0.0001, 0.0, 0.01, true},

    /* ============================================================
     * cat_depenetration
     * ============================================================ */
    {"depenetration.correction_factor", "Correction Factor", "Fraction of penetration corrected per pass", p_float,
     cat_depenetration, &g_cfg.depenetration.correction_factor, 0.35, 0.0, 1.0, true},

    {"depenetration.max_correction", "Max Correction", "Per-pass positional correction cap (m): 0.2m/teleport per pass was a tunneling-scale jump; 0.02m resolves deep overlap over ticks via split+passes without teleporting (CCD/boundary own tunneling)", p_float,
     cat_depenetration, &g_cfg.depenetration.max_correction, 0.02, 0.005, 0.1, true},
    /* TRUTH: depenetration.penetration_slop REMOVED from the registry — dead
     * since the single-slop unification (depenetration honors
     * solver.penetration_slop). The struct field remains for save-file
     * forward-compat but nothing reads it. Registry count unchanged (this
     * removal balances the sleep.enable addition at 77). */

    {"depenetration.wake_depth_thresh", "Wake Depth Threshold", "Overlap depth that wakes sleeping pairs (m). TRUTH: kept at 0.02, NOT unified with split wake 0.01: measured 0.01 re-admits the F10 runaway (runmax 13.07 m/s ejection, sleep churn on resting residual) while 0.02 holds runmax 0.00. Resting stacks carry ~0.01 residual; the wake gate must clear it.", p_float,
     cat_depenetration, &g_cfg.depenetration.wake_depth_thresh, 0.02, 0.0, 0.1, true},

    {"depenetration.rebuild_iterations", "Rebuild Iterations", "Depenetration iterations after boundary rebuild", p_int,
     cat_depenetration, &g_cfg.depenetration.rebuild_iterations, 3.0, 1.0, 10.0, true},

    /* ============================================================
     * cat_broadphase
     * ============================================================ */
    {"broadphase.cell_size_default", "Default Cell Size", "Grid cell size when scene is empty (m)", p_float,
     cat_broadphase, &g_cfg.broadphase.cell_size_default, 5.0, 0.5, 100.0, true},

    {"broadphase.cell_size_min", "Min Cell Size", "Smallest adaptive cell size (m)", p_float, cat_broadphase,
     &g_cfg.broadphase.cell_size_min, 1.0, 0.1, 100.0, true},

    {"broadphase.cell_size_max", "Max Cell Size", "Largest adaptive cell size (m)", p_float, cat_broadphase,
     &g_cfg.broadphase.cell_size_max, 50.0, 1.0, 500.0, true},

    {"broadphase.cell_size_multiplier", "Cell Size Multiplier", "Cell = multiplier * average bounding radius", p_float,
     cat_broadphase, &g_cfg.broadphase.cell_size_multiplier, 4.0, 1.0, 20.0, true},

    {"broadphase.max_cell_span_per_axis", "Max Cell Span", "Max grid cells a large object may occupy per axis", p_int,
     cat_broadphase, &g_cfg.broadphase.max_cell_span_per_axis, 8.0, 2.0, 64.0, true},

    /* ============================================================
     * cat_joints
     * ============================================================ */
    {"joints.max_acceleration", "Max Joint Acceleration", "Spring force limit expressed as acceleration (m/s^2)",
     p_float, cat_joints, &g_cfg.joints.max_acceleration, 200.0, 10.0, 10000.0, true},

    {"joints.default_spring_k", "Default Spring K", "Stiffness for normal joints (ln)", p_float, cat_joints,
     &g_cfg.joints.default_spring_k, 100.0, 1.0, 5000.0, false},

    {"joints.default_damping", "Default Damping", "Damping coefficient for normal joints", p_float, cat_joints,
     &g_cfg.joints.default_damping, 2.0, 0.0, 100.0, false},

    {"joints.soft_spring_k", "Soft Spring K", "Stiffness for soft joints (ln -s)", p_float, cat_joints,
     &g_cfg.joints.soft_spring_k, 20.0, 1.0, 5000.0, false},

    {"joints.soft_damping", "Soft Damping", "Damping coefficient for soft joints", p_float, cat_joints,
     &g_cfg.joints.soft_damping, 1.0, 0.0, 100.0, false},

    {"joints.revolute_beta", "Revolute Beta", "Baumgarte beta for revolute point-to-point", p_float, cat_joints,
     &g_cfg.joints.revolute_beta, 0.3, 0.0, 1.0, true},

    {"joints.revolute_max_bias", "Revolute Max Bias", "Cap on revolute anchor bias speed (m/s); bounds per-tick energy injection on large gaps", p_float,
      cat_joints, &g_cfg.joints.revolute_max_bias, 5.0, 0.5, 20.0, true},

    {"joints.revolute_motor_gain", "Revolute Motor Gain", "Proportional gain for revolute motor torque", p_float,
     cat_joints, &g_cfg.joints.revolute_motor_gain, 8.0, 0.0, 50.0, true},

    /* ============================================================
     * cat_boundary
     * ============================================================ */
    {"boundary.floor_emergency_slop", "Floor Emergency Slop", "Tolerance below floor before emergency clamp (m)",
      p_float, cat_boundary, &g_cfg.boundary.floor_emergency_slop, 0.05, 0.0, 1.0, true},

    /* ============================================================
     * cat_spawner
     * ============================================================ */
    {"spawner.mass", "Sphere Mass", "Default mass for spawned spheres (kg)", p_float, cat_spawner, &g_cfg.spawner.mass,
     1.0, 0.01, 10000.0, false},

    {"spawner.radius", "Sphere Radius", "Default radius for spawned spheres (m)", p_float, cat_spawner,
     &g_cfg.spawner.radius, 0.5, 0.01, 50.0, false},

    {"spawner.cube_mass", "Cube Mass", "Default mass for spawned cubes (kg)", p_float, cat_spawner,
     &g_cfg.spawner.cube_mass, 2.0, 0.01, 10000.0, false},

    {"spawner.cube_extent", "Cube Half-Extent", "Default half-extent for spawned cubes (m)", p_float, cat_spawner,
     &g_cfg.spawner.cube_extent, 0.5, 0.01, 50.0, false},

    {"spawner.cyl_mass", "Cylinder Mass", "Default mass for spawned cylinders (kg)", p_float, cat_spawner,
     &g_cfg.spawner.cyl_mass, 1.5, 0.01, 10000.0, false},

    {"spawner.cyl_radius", "Cylinder Radius", "Default radius for spawned cylinders (m)", p_float, cat_spawner,
     &g_cfg.spawner.cyl_radius, 0.4, 0.01, 50.0, false},

    {"spawner.cyl_half_length", "Cylinder Half-Length", "Default axle half-length for spawned cylinders (m)",
     p_float, cat_spawner, &g_cfg.spawner.cyl_half_length, 0.4, 0.01, 50.0, false},

    {"spawner.speed", "Launch Speed", "Launch velocity for spawned objects (m/s, uncapped — CCD owns fast bodies)",
     p_float, cat_spawner,
     &g_cfg.spawner.speed, 20.0, 0.0, 150.0, false},

    {"spawner.friction_s", "Spawn Friction (Static)", "Static friction applied to new objects", p_float, cat_spawner,
     &g_cfg.spawner.friction_s, 0.3, 0.0, 5.0, false},

    {"spawner.friction_k", "Spawn Friction (Kinetic)", "Kinetic friction applied to new objects", p_float, cat_spawner,
     &g_cfg.spawner.friction_k, 0.2, 0.0, 5.0, false},

    {"spawner.overlap_max_attempts", "Overlap Max Attempts", "Max separation passes for overlapping spawns", p_int,
     cat_spawner, &g_cfg.spawner.overlap_max_attempts, 24.0, 1.0, 200.0, true},

    {"spawner.overlap_thresh", "Overlap Threshold", "Ignored overlap depth for spawn separation (m)", p_float,
     cat_spawner, &g_cfg.spawner.overlap_thresh, 0.02, 0.0, 1.0, true},

    /* ============================================================
     * cat_body_defaults
     * ============================================================ */
    {"body_defaults.sphere_restitution", "Sphere Restitution", "Default bounce for new spheres (0=dead, 1=perfect)",
     p_float, cat_body_defaults, &g_cfg.body_defaults.sphere_restitution, 0.5, 0.0, 1.0, false},

    {"body_defaults.sphere_fric_s", "Sphere Static Friction", "Default static friction for new spheres", p_float,
     cat_body_defaults, &g_cfg.body_defaults.sphere_fric_s, 0.3, 0.0, 5.0, false},

    {"body_defaults.sphere_fric_k", "Sphere Kinetic Friction", "Default kinetic friction for new spheres", p_float,
     cat_body_defaults, &g_cfg.body_defaults.sphere_fric_k, 0.2, 0.0, 5.0, false},

    {"body_defaults.cube_restitution", "Cube Restitution", "Default bounce for new cubes (0=dead, 1=perfect)", p_float,
     cat_body_defaults, &g_cfg.body_defaults.cube_restitution, 0.5, 0.0, 1.0, false},

    {"body_defaults.cube_fric_s", "Cube Static Friction", "Default static friction for new cubes", p_float,
     cat_body_defaults, &g_cfg.body_defaults.cube_fric_s, 0.4, 0.0, 5.0, false},

    {"body_defaults.cube_fric_k", "Cube Kinetic Friction", "Default kinetic friction for new cubes", p_float,
     cat_body_defaults, &g_cfg.body_defaults.cube_fric_k, 0.3, 0.0, 5.0, false},
{"body_defaults.cylinder_restitution", "Cylinder Restitution", "Default bounce for new cylinders (wheels)", p_float,
cat_body_defaults, &g_cfg.body_defaults.cylinder_restitution, 0.3, 0.0, 1.0, false},
    {"body_defaults.cylinder_fric_s", "Cylinder Static Friction", "Default static friction for new cylinders", p_float,
     cat_body_defaults, &g_cfg.body_defaults.cylinder_fric_s, 0.4, 0.0, 5.0, false},
    {"body_defaults.cylinder_fric_k", "Cylinder Kinetic Friction", "Default kinetic friction for new cylinders", p_float,
     cat_body_defaults, &g_cfg.body_defaults.cylinder_fric_k, 0.3, 0.0, 5.0, false},

    /* ============================================================
     * cat_camera
     * ============================================================ */
    {"camera.move_speed", "Movement Speed", "Camera movement speed (m/s)", p_float, cat_camera,
     &g_cfg.camera.move_speed, 25.0, 1.0, 500.0, false},

    {"camera.mouse_sensitivity", "Mouse Sensitivity", "Mouse look sensitivity", p_float, cat_camera,
     &g_cfg.camera.mouse_sensitivity, 0.1, 0.01, 2.0, false},

    {"camera.steer_sensitivity", "Perspective Steer Sensitivity", "Mouse-look steering multiplier", p_float, cat_camera,
     &g_cfg.camera.steer_sensitivity, 0.12, 0.01, 2.0, false},

    {"camera.horizontal_friction", "Horizontal Friction", "Ground movement inertia bleed rate", p_float, cat_camera,
     &g_cfg.camera.horizontal_friction, 8.0, 0.0, 50.0, false},

    {"camera.jump_height", "Jump Height", "Jump apex height in Game Mode (m)", p_float, cat_camera,
     &g_cfg.camera.jump_height, 1.0, 0.1, 50.0, false},

    {"camera.ijkl_speed", "IJKL Steer Speed", "Keyboard steering speed (deg/s)", p_float, cat_camera,
     &g_cfg.camera.ijkl_speed, 35.0, 1.0, 200.0, false},

    /* ============================================================
     * cat_render
     * ============================================================ */
    {"render.light_x", "Light Position X", "Scene light X coordinate", p_float, cat_render, &g_cfg.render.light_x, 20.0,
     -500.0, 500.0, false},

    {"render.light_y", "Light Position Y", "Scene light Y coordinate", p_float, cat_render, &g_cfg.render.light_y, 40.0,
     -500.0, 500.0, false},

    {"render.light_z", "Light Position Z", "Scene light Z coordinate", p_float, cat_render, &g_cfg.render.light_z, 20.0,
     -500.0, 500.0, false},

    {"render.ambient_strength", "Ambient Strength", "Ambient light intensity (0=black, 1=full)", p_float, cat_render,
     &g_cfg.render.ambient_strength, 0.85, 0.0, 1.0, false},

    {"render.specular_coeff", "Specular Coefficient", "Specular highlight intensity", p_float, cat_render,
     &g_cfg.render.specular_coeff, 0.8, 0.0, 5.0, false},

    {"render.specular_exponent", "Specular Exponent", "Specular highlight sharpness (higher = tighter)", p_float,
     cat_render, &g_cfg.render.specular_exponent, 32.0, 1.0, 256.0, false},

    /* ============================================================
     * cat_ui
     * ============================================================ */
    {"ui.change_rate_game", "Change Rate (Game)", "Arrow-key step size in Game Mode menus", p_float, cat_ui,
     &g_cfg.ui.change_rate_game, 0.2, 0.01, 10.0, false},

    {"ui.change_rate_debug", "Change Rate (Debug)", "Arrow-key step size in Debug Mode menus", p_float, cat_ui,
     &g_cfg.ui.change_rate_debug, 0.01, 0.001, 10.0, false},

    {"ui.long_run_ticks", "Long-Run Ticks", "F10 validation duration in ticks (60 = 1 second)", p_int, cat_ui,
     &g_cfg.ui.long_run_ticks, 3600.0, 60.0, 36000.0, true},

    {"ui.enter_spawn_delay", "Enter Spawn Delay", "Seconds holding Enter before rapid-fire (s)", p_float, cat_ui,
     &g_cfg.ui.enter_spawn_delay, 0.3, 0.0, 5.0, false},

    {"ui.enter_spawn_interval", "Enter Spawn Interval", "Seconds between rapid-fire spawns (s)", p_float, cat_ui,
     &g_cfg.ui.enter_spawn_interval, 0.02, 0.001, 1.0, false},
};

/* ------------------------------------------------------------------
 * Registry count and public aliases
 * ------------------------------------------------------------------ */
const size_t g_registry_count = sizeof(s_registry) / sizeof(s_registry[0]);
const mpe_param *g_registry = s_registry;

/* MPE_TASK_27_CONFIG_SCHEMA_END */

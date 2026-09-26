/* MPE_FTC_073: FTC robot object */
#ifndef robot_h
#define robot_h

#include "motor.h"
#include "motor_presets.h"
#include "battery.h"
#include "core/physics_world.h"

#define FTC_MAX_WHEELS 8

/* MFS_DRIVETRAIN_TYPE: explicit wheel/traction model selection. */
typedef enum {
    FTC_DRIVETRAIN_MECANUM = 0,
    FTC_DRIVETRAIN_TANK = 1
} ftc_drivetrain_type;

typedef struct {
    /* Body indices in physics_world */
    int chassis_body;
    int wheel_bodies[FTC_MAX_WHEELS];
    int wheel_joints[FTC_MAX_WHEELS]; /* revolute joint indices */
    int wheel_count;

    /* Mecanum rollers: real bodies on real free revolute bearings (see
     * robot.c for geometry). They are what touches the floor and all CONTACT
     * FORCE comes from the engine's Coulomb solver (no chassis force is ever
     * injected). The bearing SPIN velocity is quasi-static (stiff DOF slaved,
     * DESPOT-2026-09-26): roller spin about its axle is prescribed to the
     * massless-roller equilibrium each tick with a small dissipative bearing
     * torque, because 60Hz explicit integration of the ~1ms roller mode is
     * unconditionally unstable. Off-axis wobble is preserved. */
/* FIX-AUDIT-DESPOT: was MFS_ROLLERS_PER_Wheel (lowercase 'heel' typo).
 * Renamed to MFS_ROLLERS_PER_WHEEL; the old spelling is kept as a
 * backward-compatible alias so out-of-tree code keeps compiling.
 * DESPOT-FIX: was hard-coded (8*2)=16 while the builder makes
 * MECANUM_ROLLERS_PER_ROW(8)*MECANUM_ROLLER_ROWS(1)=8 live rollers, wasting
 * half of every roller array (8 unused ints + joints per wheel). Now sized
 * to the true build count; static assert in robot.c locks them together. */
#define MFS_ROLLERS_PER_WHEEL 8
#define MFS_ROLLERS_PER_Wheel MFS_ROLLERS_PER_WHEEL /* compat alias */
    int roller_bodies[FTC_MAX_WHEELS][MFS_ROLLERS_PER_WHEEL];
    int roller_joints[FTC_MAX_WHEELS][MFS_ROLLERS_PER_WHEEL];
    int roller_count[FTC_MAX_WHEELS];
    /* Radius of the running surface: the roller pitch + roller radius for a
     * mecanum wheel, the tyre radius for a tank wheel. Everything that needs
     * the geometric lever arm (rolling resistance, odometry radius) must use
     * this, not the hub plate radius. */
    float wheel_effective_radius[FTC_MAX_WHEELS];

    /* Motor + electrical */
    motor wheel_motors[FTC_MAX_WHEELS];
    motor_preset_id motor_preset;
    battery battery;

    /* Axle direction in chassis-local space (for reading wheel speed) */
    float axle_axis_x, axle_axis_y, axle_axis_z;
    /* (removed: mecanum_chassis_force/torque dead fields, zero uses) */
    ftc_drivetrain_type drivetrain_type; /* MFS_DRIVETRAIN_TYPE */

    /* MFS_151_ODOMETRY: Wheel encoders and pose estimation */
    float wheel_radians[FTC_MAX_WHEELS]; /* MFS_163_BOUNDS_FIX: was [4], OOB if wheel_count > 4 */
    /* DESPOT-2026-09-26: encoder quantization state. wheel_radians above is
     * the TRUE continuous hub angle; the counts below are what a real
     * quadrature encoder reports (integer, PPR-limited). Odometry integrates
     * the QUANTIZED deltas (staircase, zero at creep speeds), not the true
     * omegas — previously PPR was plumbed but never consumed, so resolution
     * error was unmodeled. counts_per_rev is derived per robot from the
     * preset (base_ppr * gear_ratio); zero-size memset gives count 0. */
    int wheel_encoder_counts[FTC_MAX_WHEELS];
    float wheel_radians_quant[FTC_MAX_WHEELS];
    float odom_x, odom_z, odom_theta;
    /* odom_slip: slip REPORT flag (no fusion — odometry is never corrected).
     * DESPOT-2026-09-26: was hard-zeroed "ABI only". Now 1 when the
     * encoder-implied chassis motion disagrees with the true chassis motion
     * beyond (0.25 m/s planar or 0.35 rad/s yaw), else 0. Pure reporting:
     * odom_x/z/theta are still integrated from encoders only. */
    int odom_slip;
    /* clamp_events: counts ticks where the velocity safety monitor fired
     * (was a silent 3 m/s hard clamp; now telemetry only). */
    int clamp_events;
    /* MFS_TRACTION_CONTROL: per-wheel torque scale (1 = full). Cut when
     * slip is detected so a spun-up wheel re-grips instead of sliding
     * forever (kinetic friction alone can never re-capture a wheel whose
     * stall torque exceeds the kinetic cone). */
    float wheel_traction_scale[FTC_MAX_WHEELS];
    /* DESPOT-FIX (torque slew state): previously applied axle torque per
     * wheel (N.m). The driver feathers standing starts in reality (ESC
     * current-slew limits); a 0->stall step in one tick outruns the
     * contact (74 N-equiv vs 19 N cone) and peel-out locks symmetric
     * commands into chaos. Slew-limiting lets grip establish before full
     * torque lands. Zero-init via memset; memcpy-safe like the rest. */
    float wheel_applied_torque[FTC_MAX_WHEELS];
    /* MFS_PORT_V15S: roller geometry used to live on the rigidbody
     * (is_mecanum / roller_angle_rad, removed with the parked solver
     * hooks). It now lives here, owned by the robot that defines it. */
    float wheel_roller_angle[FTC_MAX_WHEELS]; /* radians, +45/-45 layout */
    bool wheel_is_mecanum[FTC_MAX_WHEELS];
} ftc_robot;

/* Create a 4-wheel robot at the given position. Returns 0 on success. */
/* MPE_FTC_095: chassis-centre height where wheels rest on the floor */
float ftc_robot_rest_height(void);
int ftc_robot_create_with_drive(physics_world *world, ftc_robot *robot, float x, float y, float z,
                                motor_preset_id preset, ftc_drivetrain_type drivetrain_type);

int ftc_robot_create(physics_world *world, ftc_robot *robot, float x, float y, float z, motor_preset_id preset);

/* Install the drive field: enable MPE's static ground plane and set its
 * friction. RETURNS 0 on success.
 *
 * This is REQUIRED before a drivetrain means anything. MPE ships
 * `static_plane_enabled = false` in a fresh world, so a world that never
 * calls this has gravity but no ground: the robot free-falls, its wheels
 * generate no contacts, and drivetrain behaviour (traction, rolling
 * resistance, odometry, mecanum lateral) is all unmeasurable. MPE's own
 * suite documents the same trap in tests/mpe_test.h, where three earlier
 * tests silently measured a floorless world.
 *
 * MFS deliberately does NOT enable the plane implicitly inside
 * ftc_robot_create*: it attaches to a host-owned world, and silently
 * turning on a collider in someone else's world would be a side effect
 * the host cannot see or override. Hosts call this explicitly.
 *
 * `mus`/`muk` are the static and kinetic friction coefficients of the
 * field surface (e.g. 0.6 / 0.4 for a typical FTC foam field). They are
 * written to the world config as well as the cached plane body, so the
 * value is honoured regardless of world initialisation order. */
int ftc_world_setup_field(physics_world *world, float mus, float muk);

/* Update all motors for one tick. Reads wheel angular velocity,
   computes electrical state, applies torque to wheel bodies. */
void ftc_robot_update(physics_world *world, ftc_robot *robot, float dt);

/* Set wheel motor commands (-1..1). */
void ftc_robot_set_wheel_commands(ftc_robot *robot, const float *commands, int count);

/* Get the chassis body's position (for validation). */
void ftc_robot_get_position(physics_world *world, ftc_robot *robot, float *px, float *py, float *pz);

#endif /* robot_h */

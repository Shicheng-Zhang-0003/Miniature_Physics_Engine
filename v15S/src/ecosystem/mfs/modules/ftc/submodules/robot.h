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
    float odom_x, odom_z, odom_theta;
    /* odom_slip: lateral roller thrust bypasses the wheels (direct chassis
     * force), so wheel encoders are structurally blind to strafe. When the
     * chassis-derived lateral velocity disagrees with the encoder FK, odom
     * lateral is fused from chassis motion (dead-wheel equivalent) and
     * this flag is set. 0 = pure encoders, 1 = fused. */
    int odom_slip;
    /* clamp_events: counts ticks where the velocity safety monitor fired
     * (was a silent 3 m/s hard clamp; now telemetry only). */
    int clamp_events;
    /* MFS_TRACTION_CONTROL: per-wheel torque scale (1 = full). Cut when
     * slip is detected so a spun-up wheel re-grips instead of sliding
     * forever (kinetic friction alone can never re-capture a wheel whose
     * stall torque exceeds the kinetic cone). */
    float wheel_traction_scale[FTC_MAX_WHEELS];
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

/* Update all motors for one tick. Reads wheel angular velocity,
   computes electrical state, applies torque to wheel bodies. */
void ftc_robot_update(physics_world *world, ftc_robot *robot, float dt);

/* Set wheel motor commands (-1..1). */
void ftc_robot_set_wheel_commands(ftc_robot *robot, const float *commands, int count);

/* Get the chassis body's position (for validation). */
void ftc_robot_get_position(physics_world *world, ftc_robot *robot, float *px, float *py, float *pz);

#endif /* robot_h */

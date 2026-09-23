/* MFS Module 1: True Simulator
 *
 * Core module for building, constructing, and running a robot from scratch
 * on a biobuzz field. Implements accurate drivetrain, intake, and shooter
 * physics for biobuzz-sized balls.
 *
 * This module follows the MPE Module Interface (MPI) v1 ABI:
 * - Never touches MPE kernel globals directly
 * - Uses mpe_world_* accessors and per-world config
 * - Per-world state via attach/detach
 * - Deterministic: uses det_math.h for transcendentals
 *
 * Architecture:
 * - mfs_module_1_attach(): allocates per-world MFS state, creates field,
 *   spawns robot with drivetrain + intake + shooter
 * - mfs_module_1_pre_step(): runs robot control loop, intake logic,
 *   shooter charging/firing, ball physics
 * - mfs_module_1_detach(): frees all MFS-created bodies/joints/state
 *
 * Robot Configuration (Module 1 - "Simple but Accurate"):
 * - 4-wheel mecanum drivetrain (goBILDA 5203 19.2:1)
 * - Roller intake (compliant contact, Hertz-based grip)
 * - Flywheel shooter (energy-based launch, Magnus effect)
 * - Biobuzz ball: 42mm diameter, 2.6g, COR 0.65
 *
 * Module 2 will add: detailed aerodynamics, ball spin decay,
 * intake roller slip modeling, shooter motor thermal limits.
 */

#ifndef mfs_module_1_h
#define mfs_module_1_h

#include <stdint.h>
#include <stdbool.h>
#include "core/math3d.h"
#include "core/rigidbody.h"
#include "core/physics_world.h"
#include "modules/ftc/submodules/robot.h"
#include "modules/ftc/submodules/motor_presets.h"
#include "modules/ftc/submodules/battery.h"
#include "modules/module_1/submodules/gamepad/gamepad.h"

#define MFS_MODULE_1_VERSION "1.0"
#define MFS_MODULE_1_NAME "mfs-simulator"

/* Biobuzz ball specifications */
#define MFS_BIOBUZZ_BALL_DIAMETER     0.042f  /* 42mm */
#define MFS_BIOBUZZ_BALL_RADIUS       0.021f
#define MFS_BIOBUZZ_BALL_MASS         0.0026f /* 2.6g */
#define MFS_BIOBUZZ_BALL_RESTITUTION  0.65f
#define MFS_BIOBUZZ_BALL_FRICTION_S   0.45f
#define MFS_BIOBUZZ_BALL_FRICTION_K   0.35f

/* Field dimensions (biobuzz standard) */
#define MFS_BIOBUZZ_FIELD_WIDTH       12.0f   /* 12m x 12m */
#define MFS_BIOBUZZ_FIELD_LENGTH      12.0f
#define MFS_BIOBUZZ_GOAL_HEIGHT       2.5f
#define MFS_BIOBUZZ_GOAL_WIDTH        1.5f

/* Robot configuration */
#define MFS_ROBOT_CHASSIS_WIDTH       0.45f
#define MFS_ROBOT_CHASSIS_LENGTH      0.45f
#define MFS_ROBOT_CHASSIS_HEIGHT      0.15f
#define MFS_ROBOT_CHASSIS_MASS        3.5f
#define MFS_ROBOT_WHEEL_PRESET        MOTOR_GB_5203_19_2  /* goBILDA 5203 19.2:1 */
#define MFS_ROBOT_MAX_BALLS           3

/* Intake configuration */
#define MFS_INTAKE_ROLLER_RADIUS      0.025f
#define MFS_INTAKE_ROLLER_LENGTH      0.20f
#define MFS_INTAKE_ROLLER_SPEED_RPM   600.0f
#define MFS_INTAKE_COMPLIANCE         0.005f  /* 5mm soft contact */

/* Shooter configuration */
#define MFS_SHOOTER_FLYWHEEL_RADIUS   0.050f
#define MFS_SHOOTER_FLYWHEEL_MASS     0.30f
#define MFS_SHOOTER_TARGET_RPM        4000.0f
#define MFS_SHOOTER_LAUNCH_ANGLE_DEG  35.0f
#define MFS_SHOOTER_SPINUP_TIME       2.0f    /* seconds to reach target RPM */

/* MFS per-world state */
typedef struct mfs_module_1_state {
    physics_world *world;
    
    /* Field elements */
    int field_floor_id;
    int goal_frame_id;
    int boundary_wall_ids[4];
    
    /* Robot */
    ftc_robot robot;
    bool robot_created;
    
    /* Ball management */
    int ball_body_ids[16];      /* biobuzz balls on field */
    int ball_count;
    int max_balls;
    float ball_spawn_timer;
    
    /* Intake state */
    int intake_roller_body;
    int intake_pivot_joint;
    bool intake_deployed;
    float intake_speed_rpm;
    float intake_power;         /* -1..1 */
    
    /* Shooter state */
    int shooter_flywheel_body;
    int shooter_pivot_joint;
    float shooter_rpm;
    float shooter_target_rpm;
    bool shooter_spinning_up;
    float shooter_spinup_timer;
    bool shooter_ready;
    
    /* Game state */
    int balls_fired;
    int balls_collected;
    float match_time;
    
    /* Robot control inputs */
    float drive_forward;
    float drive_strafe;
    float drive_rotate;
    bool intake_active;
    bool shooter_spinup_cmd;
    bool shooter_fire_cmd;
    
    /* Gamepad control */
    gamepad_state gamepad;
    bool gamepad_initialized;
    bool gamepad_control_enabled;  /* toggle with Start button */
    
    /* Button state tracking for edge detection */
    bool prev_button_start;
    bool prev_button_a;
    bool prev_button_b;
    bool prev_button_x;
    bool prev_button_y;
    bool prev_button_lb;
    bool prev_button_rb;
    bool prev_left_trigger;
    bool prev_right_trigger;
    
} mfs_module_1_state;

/* Module descriptor (exported symbol for dlopen) */
extern const mpe_module_desc_t mfs_module_1_desc;

/* Public API for host control */
void mfs_module_1_set_drive_commands(mfs_module_1_state *state,
                                     float forward, float strafe, float rotate);
void mfs_module_1_set_intake(mfs_module_1_state *state, bool active);
void mfs_module_1_set_shooter(mfs_module_1_state *state, bool spinup, bool fire);
void mfs_module_1_get_stats(const mfs_module_1_state *state,
                            int *balls_collected, int *balls_fired,
                            float *shooter_rpm, bool *shooter_ready);

/* Internal helpers (used by pre_step) */
void mfs_module_1_field_create(mfs_module_1_state *state);
void mfs_module_1_robot_create(mfs_module_1_state *state);
void mfs_module_1_intake_create(mfs_module_1_state *state);
void mfs_module_1_shooter_create(mfs_module_1_state *state);
void mfs_module_1_ball_spawn(mfs_module_1_state *state, vector3 pos);
void mfs_module_1_ball_physics_step(mfs_module_1_state *state, float dt);
void mfs_module_1_intake_step(mfs_module_1_state *state, float dt);
void mfs_module_1_shooter_step(mfs_module_1_state *state, float dt);
void mfs_module_1_robot_drive_step(mfs_module_1_state *state, float dt);
void mfs_module_1_gamepad_step(mfs_module_1_state *state, float dt);

/* Helper: get chassis body */
static inline rigidbody *mfs_get_chassis(const mfs_module_1_state *state) {
    if (!state || !state->robot_created || !state->world) return NULL;
    if (state->robot.chassis_body < 0 || state->robot.chassis_body >= state->world->body_count) return NULL;
    return &state->world->bodies[state->robot.chassis_body];
}

#endif /* mfs_module_1_h */
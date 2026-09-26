/* MFS Module 1: True Simulator Implementation
 *
 * Implements a complete robot simulation on a biobuzz field with
 * accurate drivetrain, intake, and shooter physics.
 *
 * This file is compiled into a static library (libmfs_module_1_core.a)
 * which is then linked with --whole-archive into the shared module.
 * All functions are marked with __attribute__((used)) to prevent
 * optimization removal since they're referenced via function pointers.
 */
#include "mfs_module_1.h"
#include "core/det_math.h"
#include "core/mpe_registry.h"
#include "physics/collision_mechanics.h"
#include "physics/broadphase.h"
#include "physics/constraint.h"
#include "modules/ftc/submodules/robot.h"
#include "modules/ftc/submodules/drivetrain.h"
#include "modules/ftc/submodules/motor.h"
#include "modules/ftc/submodules/battery.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* Forward declarations for module hooks (referenced in descriptor) */
__attribute__((used)) int mfs_module_1_attach(mpe_world_t *world, void **mod_state);
__attribute__((used)) void mfs_module_1_detach(mpe_world_t *world, void *mod_state);
__attribute__((used)) void mfs_module_1_pre_step(mpe_world_t *world, float dt, void *mod_state);
__attribute__((used)) void mfs_module_1_post_step(mpe_world_t *world, float dt, void *mod_state);

/* ================================================================
 * Module Descriptor (exported for dlopen)
 * ================================================================ */

__attribute__((used)) const mpe_module_desc_t mfs_module_1_desc = {
    .abi = MPE_MODULE_ABI,
    .name = MFS_MODULE_1_NAME,
    .version = MFS_MODULE_1_VERSION,
    .kind = "generic",
    /* PHYSICS-TRUTH: not bit-deterministic. Uses M_PI arithmetic, fabsf,
     * live gamepad_poll, and FTC motor/battery float paths; route trig
     * through det_math.h before claiming true. */
    .deterministic = false,
    .attach = mfs_module_1_attach,
    .detach = mfs_module_1_detach,
    .pre_step = mfs_module_1_pre_step,
    .post_step = mfs_module_1_post_step,
};

/* NOTE: no mpe_module_desc alias here on purpose. Both this TU and
 * ftc_module.c used to DEFINE it, breaking every link that combined
 * them (multiple-definition). The MFS-internal export is
 * mfs_module_1_desc above; MPI loading uses the ftc-fleet module. */

/* ================================================================
 * Module Lifecycle
 * ================================================================ */

__attribute__((used)) int mfs_module_1_attach(mpe_world_t *world, void **mod_state) {
    if (!world || !mod_state) return -1;
    
    mfs_module_1_state *state = calloc(1, sizeof(mfs_module_1_state));
    if (!state) return -1;
    
    state->world = world;
    state->max_balls = 16;
    state->shooter_target_rpm = MFS_SHOOTER_TARGET_RPM;
    state->intake_speed_rpm = MFS_INTAKE_ROLLER_SPEED_RPM;
    state->gamepad_control_enabled = true;

    /* Joint pools: the robot/intake/shooter are joint assemblies.
     * Without this, constraint_add_revolute fails and robot creation
     * silently aborts (the module test never ran — see Makefile D1). */
    constraint_pool_init(world);
    
    /* Initialize gamepad (singleton) */
    if (gamepad_init(&state->gamepad, NULL)) {
        state->gamepad_initialized = true;
        state->gamepad.deadzone = 0.15f;
    }
    
    /* Create field, robot, intake, shooter */
    mfs_module_1_field_create(state);
    mfs_module_1_robot_create(state);
    mfs_module_1_intake_create(state);
    mfs_module_1_shooter_create(state);
    
    /* Spawn initial balls */
    for (int i = 0; i < 5; i++) {
        vector3 pos = {
            -4.0f + (i % 3) * 0.5f,
            MFS_BIOBUZZ_BALL_RADIUS,
            -2.0f + (i / 3) * 0.5f
        };
        mfs_module_1_ball_spawn(state, pos);
    }
    
    *mod_state = state;
    return 0;
}

__attribute__((used)) void mfs_module_1_detach(mpe_world_t *world, void *mod_state) {
    (void)world;
    if (!mod_state) return;
    
    mfs_module_1_state *state = (mfs_module_1_state *)mod_state;
    
    if (state->gamepad_initialized) {
        gamepad_close(&state->gamepad);
    }
    
    /* Clean up balls - world cleanup handles bodies */
    for (int i = 0; i < state->ball_count; i++) {
        /* Body cleanup handled by world cleanup */
    }
    
    free(state);
}

/* ================================================================
 * Gamepad Control Step (F310 mapping)
 * ================================================================ */

__attribute__((used)) void mfs_module_1_gamepad_step(mfs_module_1_state *state, float dt) {
    (void)dt;
    if (!state->gamepad_initialized || !state->gamepad_control_enabled) return;
    
    gamepad_state *pad = &state->gamepad;
    
    /* Read axes (with deadzone applied) */
    float left_x = gamepad_get_axis(pad, gamepad_axis_left_x);
    float left_y = gamepad_get_axis(pad, gamepad_axis_left_y);
    float right_x = gamepad_get_axis(pad, gamepad_axis_right_x);
    /* (right_y unread: no pitch mapping exists) */
    /* DESPOT-FIX: triggers via gamepad_get_trigger() ([0,1] pressed amount,
     * rest-robust across drivers) — was gamepad_get_axis() ([-1,1]) which
     * misfired the 0.5 gate at rest on 0-rest drivers. */
    float lt = gamepad_get_trigger(pad, gamepad_axis_left_trigger);
    float rt = gamepad_get_trigger(pad, gamepad_axis_right_trigger);
    
    /* Read buttons */
    bool btn_a = gamepad_get_button(pad, gamepad_button_a);
    bool btn_b = gamepad_get_button(pad, gamepad_button_b);
    bool btn_x = gamepad_get_button(pad, gamepad_button_x);
    bool btn_y = gamepad_get_button(pad, gamepad_button_y);
    bool btn_lb = gamepad_get_button(pad, gamepad_button_lb);
    bool btn_rb = gamepad_get_button(pad, gamepad_button_rb);
    bool btn_start = gamepad_get_button(pad, gamepad_button_start);
    bool btn_back = gamepad_get_button(pad, gamepad_button_back);
    
    /* Edge detection for toggles */
    bool start_pressed = btn_start && !state->prev_button_start;
    bool a_pressed = btn_a && !state->prev_button_a;
    bool x_pressed = btn_x && !state->prev_button_x;
    bool y_pressed = btn_y && !state->prev_button_y;
    /* (removed unused edge vars b/lb/rb/lt/rt_pressed: B is level-read,
     * LB+RB combine in the e-stop below, triggers are level-read) */
    
    /* Update previous button states */
    state->prev_button_start = btn_start;
    state->prev_button_a = btn_a;
    state->prev_button_b = btn_b;
    state->prev_button_x = btn_x;
    state->prev_button_y = btn_y;
    state->prev_button_lb = btn_lb;
    state->prev_button_rb = btn_rb;
    state->prev_left_trigger = (lt > 0.5f);
    state->prev_right_trigger = (rt > 0.5f);
    
    /* Toggle gamepad control with Start button. Processed BEFORE the
     * enabled check: otherwise a disabled pad can never re-enable
     * (latch-dead — the toggle lived behind its own gate). */
    if (start_pressed) {
        state->gamepad_control_enabled = !state->gamepad_control_enabled;
    }

    if (!state->gamepad_control_enabled) return;
    
    /* Drive mapping (mecanum):
     * Left stick Y = forward/backward (inverted)
     * Left stick X = strafe left/right
     * Right stick X = rotate
     */
    state->drive_forward = -left_y;   /* forward = -Y (up on stick) */
    state->drive_strafe = left_x;     /* strafe right = +X */
    state->drive_rotate = right_x;    /* rotate right = +X */
    
    /* Intake: A toggles the mode, B momentarily reverses while held and
     * releases back to the mode power. (The old chain double-toggled on
     * A and latched reverse forever after any B-hold: the release path
     * was unreachable.) */
    if (a_pressed) {
        state->intake_active = !state->intake_active;
    }
    if (btn_b) {
        state->intake_power = -1.0f;  /* momentary reverse */
    } else {
        state->intake_power = state->intake_active ? 1.0f : 0.0f;
    }
    
    /* Shooter spin-up (X button toggle) */
    if (x_pressed) {
        state->shooter_spinup_cmd = !state->shooter_spinup_cmd;
    }
    
    /* Shooter fire (Y button) */
    if (y_pressed) {
        state->shooter_fire_cmd = true;
    }
    
    /* Quick fire with right trigger */
    if (rt > 0.5f && state->shooter_ready) {
        state->shooter_fire_cmd = true;
    }
    
    /* Intake speed control with left trigger */
    if (lt > 0.1f) {
        state->intake_speed_rpm = 600.0f + lt * 600.0f;  /* 600-1200 RPM */
    } else {
        state->intake_speed_rpm = 600.0f;
    }
    
    /* LB + RB = emergency stop (zero all commands) */
    if (btn_lb && btn_rb) {
        state->drive_forward = 0.0f;
        state->drive_strafe = 0.0f;
        state->drive_rotate = 0.0f;
        state->intake_active = false;
        state->shooter_spinup_cmd = false;
    }
    
    /* Back button = reset robot position (debug) */
    if (btn_back) {
        rigidbody *chassis = mfs_get_chassis(state);
        if (chassis) {
            physics_world *world = state->world;
            vector3 target = (vector3){0.0f, 0.2f, -3.0f};
            vector3 delta = vector3_subtraction(target, chassis->position);

            /* M8 FULL-ASSEMBLY RESET FIX: this used to move ONLY the chassis.
             * The four wheel bodies and their revolute joints stayed at the
             * old pose, so the next joint correction yanked every wheel back
             * with a large artificial impulse (and a matching spike in the
             * encoder deltas, because the wheels genuinely moved). It also
             * skipped odometry entirely. Now the whole assembly is
             * translated by one delta: orientation is unchanged, so the
             * revolute anchors and accumulated_angle stay valid, which is
             * what keeps the joints from having to correct anything.
             *
             * FIX-AUDIT-DESPOT: the old loop ALSO misused index_by_id on
             * wheel_bodies (which hold INDICES, not object ids — add_*
             * returns the slot), so it translated the wrong bodies and left
             * wheel 3 + every roller behind. Index-held fields (wheels,
             * rollers, chassis) are used as indices with bounds checks;
             * id-held fields (flywheel, intake) resolve via index_by_id —
             * uniformly through one reset step below. Owned extras (rollers,
             * flywheel, intake) move too, and every wheel motor observer is
             * reset: a warp is a discontinuous dw the disturbance observer
             * would otherwise read as a phantom stall spike. */
            int n = state->robot.wheel_count;
            if (n > FTC_MAX_WHEELS) n = FTC_MAX_WHEELS;
            for (int i = 0; i < n; i++) {
                int wi = state->robot.wheel_bodies[i];
                if (wi < 0 || wi >= world->body_count) continue;
                rigidbody *w = &world->bodies[wi];
                w->position = vector3_addition(w->position, delta);
                w->velocity = vector3_zero();
                w->angular_velocity = vector3_zero();
                w->is_sleeping = false;
                w->sleep_timer = 0.0f;
                for (int k = 0; k < state->robot.roller_count[i]; k++) {
                    int rb = state->robot.roller_bodies[i][k];
                    if (rb < 0 || rb >= world->body_count) continue;
                    rigidbody *ro = &world->bodies[rb];
                    ro->position = vector3_addition(ro->position, delta);
                    ro->velocity = vector3_zero();
                    ro->angular_velocity = vector3_zero();
                    ro->is_sleeping = false;
                    ro->sleep_timer = 0.0f;
                }
                motor_reset_observer(&state->robot.wheel_motors[i]);
            }
            {
                int fw = physics_world_index_by_id(
                    world, (uint32_t)state->shooter_flywheel_body);
                if (fw >= 0) {
                    rigidbody *f = &world->bodies[fw];
                    f->position = vector3_addition(f->position, delta);
                    f->velocity = vector3_zero();
                    f->angular_velocity = vector3_zero();
                    f->is_sleeping = false;
                    f->sleep_timer = 0.0f;
                }
                int ir = physics_world_index_by_id(
                    world, (uint32_t)state->intake_roller_body);
                if (ir >= 0) {
                    rigidbody *rr = &world->bodies[ir];
                    rr->position = vector3_addition(rr->position, delta);
                    rr->velocity = vector3_zero();
                    rr->angular_velocity = vector3_zero();
                    rr->is_sleeping = false;
                    rr->sleep_timer = 0.0f;
                }
            }
            chassis->position = target;
            chassis->velocity = vector3_zero();
            chassis->angular_velocity = vector3_zero();
            chassis->is_sleeping = false;
            chassis->sleep_timer = 0.0f;

            state->robot.odom_x = 0.0f;
            state->robot.odom_z = -3.0f;
            state->robot.odom_theta = 0.0f;
        }
    }
}

/* ================================================================
 * Pre-step: Main simulation tick
 * ================================================================ */

__attribute__((used)) void mfs_module_1_pre_step(mpe_world_t *world, float dt, void *mod_state) {
    mfs_module_1_state *state = (mfs_module_1_state *)mod_state;
    if (!state || !state->robot_created) return;
    
    /* Poll gamepad for input */
    if (state->gamepad_initialized) {
        gamepad_poll(&state->gamepad);
        mfs_module_1_gamepad_step(state, dt);
    }
    
    state->match_time += dt;
    
    /* Robot drive */
    mfs_module_1_robot_drive_step(state, dt);
    
    /* Intake logic */
    mfs_module_1_intake_step(state, dt);
    
    /* Shooter logic */
    mfs_module_1_shooter_step(state, dt);
    
    /* Ball physics (spin, drag, Magnus) */
    mfs_module_1_ball_physics_step(state, dt);
    
    /* Full drivetrain update (motor torque + traction + odometry +
     * damping). Calling ftc_robot_update directly skipped all of that:
     * no traction forces, no odometry, no chassis damping. mpe_world_t
     * IS physics_world (see core/mpe_module.h). */
    drivetrain_update((physics_world *)world, &state->robot, dt);
}

__attribute__((used)) void mfs_module_1_post_step(mpe_world_t *world, float dt, void *mod_state) {
    (void)world; (void)dt; (void)mod_state;
    /* No post-step work needed for Module 1 */
}

/* ================================================================
 * Field Creation
 * ================================================================ */

__attribute__((used)) void mfs_module_1_field_create(mfs_module_1_state *state) {
    physics_world *world = state->world;
    const float half_w = MFS_BIOBUZZ_FIELD_WIDTH * 0.5f;
    const float half_l = MFS_BIOBUZZ_FIELD_LENGTH * 0.5f;
    const float wall_h = 1.0f;
    const float wall_t = 0.1f;
    
    /* Floor - static plane at y=0 */
    int floor_idx = physics_world_add_cube(world,
        (vector3){0.0f, -0.05f, 0.0f},
        (vector3){half_w + 1.0f, 0.05f, half_l + 1.0f},
        0.0f);
    if (floor_idx >= 0) {
        state->field_floor_id = world->bodies[floor_idx].object_id;
        world->bodies[floor_idx].restitution = 0.0f;
        world->bodies[floor_idx].friction_static = 0.7f;
        world->bodies[floor_idx].friction_kinetic = 0.6f;
    }
    
    /* Boundary walls */
    physics_world_add_boundary_walls(world, half_w, half_l, wall_h, wall_t);
    
    /* Goal frame - two vertical posts + crossbar */
    const float goal_w = MFS_BIOBUZZ_GOAL_WIDTH;
    const float goal_h = MFS_BIOBUZZ_GOAL_HEIGHT;
    const float post_t = 0.08f;
    const float goal_z = half_l;
    
    /* Left post */
    physics_world_add_cube(world,
        (vector3){-goal_w*0.5f - post_t*0.5f, goal_h*0.5f, goal_z - post_t*0.5f},
        (vector3){post_t*0.5f, goal_h*0.5f, post_t*0.5f}, 0.0f);
    
    /* Right post */
    physics_world_add_cube(world,
        (vector3){goal_w*0.5f + post_t*0.5f, goal_h*0.5f, goal_z - post_t*0.5f},
        (vector3){post_t*0.5f, goal_h*0.5f, post_t*0.5f}, 0.0f);
    
    /* Crossbar */
    physics_world_add_cube(world,
        (vector3){0.0f, goal_h + post_t*0.5f, goal_z - post_t*0.5f},
        (vector3){goal_w*0.5f + post_t, post_t*0.5f, post_t*0.5f}, 0.0f);
}

/* ================================================================
 * Robot Creation
 * ================================================================ */

__attribute__((used)) void mfs_module_1_robot_create(mfs_module_1_state *state) {
    physics_world *world = state->world;
    
    /* Create FTC robot with mecanum drivetrain */
    int result = ftc_robot_create_with_drive(world, &state->robot,
        0.0f, 0.2f, -3.0f,  /* Start position */
        MFS_ROBOT_WHEEL_PRESET, FTC_DRIVETRAIN_MECANUM);
    
    if (result == 0) {
        state->robot_created = true;
        
        /* Configure robot battery */
        state->robot.battery.nominal_voltage = 12.8f;
        state->robot.battery.capacity_ah = 3.0f;
        /* FIX-AUDIT-DESPOT: 0.06 NiMH pack-level (was 0.015 LiPo-class);
         * keep in sync with battery_init. */
        state->robot.battery.internal_resistance = 0.06f;
        state->robot.battery.charge_fraction = 1.0f;
        
        /* M9 DEAD-FIELD FIX: this block re-initialized wheel_is_mecanum,
         * wheel_roller_angle and wheel_traction_scale AFTER ftc_robot_init
         * had already set all three authoritatively, and it wrote a
         * DIFFERENT roller pattern: `i%2` gives +,-,+,- (a Z arrangement,
         * which does not produce lateral motion) while robot.c sets the
         * correct mecanum X pattern +,-,-,+. Nothing read the field, so it
         * was a trap for whoever wires the roller model up. Deleted: the
         * single source of truth is ftc_robot_init / robot.c. */
        
        /* Initialize odometry */
        state->robot.odom_x = 0.0f;
        state->robot.odom_z = -3.0f;
        state->robot.odom_theta = 0.0f;
    }
}

/* ================================================================
 * Helper: get chassis body
 * ================================================================ */


/* ================================================================
 * Intake Creation (Roller-based compliant intake)
 * ================================================================ */

__attribute__((used)) void mfs_module_1_intake_create(mfs_module_1_state *state) {
    physics_world *world = state->world;
    rigidbody *chassis = mfs_get_chassis(state);
    if (!chassis) return;
    
    /* Roller positioned at front-lower of chassis */
    vector3 robot_pos = chassis->position;
    vector3 roller_pos = vector3_addition(robot_pos, 
        (vector3){0.0f, -MFS_ROBOT_CHASSIS_HEIGHT*0.3f, MFS_ROBOT_CHASSIS_LENGTH*0.5f + 0.03f});
    
    int roller_idx = physics_world_add_cylinder(world,
        MFS_INTAKE_ROLLER_RADIUS,
        MFS_INTAKE_ROLLER_LENGTH * 0.5f,
        0.05f,  /* Light roller mass */
        roller_pos);
    
    if (roller_idx >= 0) {
        state->intake_roller_body = world->bodies[roller_idx].object_id;
        rigidbody *roller = &world->bodies[roller_idx];
        roller->restitution = 0.0f;
        roller->friction_static = 0.8f;
        roller->friction_kinetic = 0.7f;
        roller->kinematic = false;  /* Driven by motor torque */
        
        /* Revolute joint to chassis for roller spin */
        int joint_idx = constraint_add_revolute(world,
            chassis->object_id,
            roller->object_id,
            (vector3){0.0f, -MFS_ROBOT_CHASSIS_HEIGHT*0.3f, MFS_ROBOT_CHASSIS_LENGTH*0.5f + 0.03f},  /* anchor on chassis */
            (vector3){0.0f, 0.0f, 0.0f},  /* anchor on roller (center) */
            (vector3){1.0f, 0.0f, 0.0f});  /* spin axis = X */
        if (joint_idx >= 0) {
            state->intake_pivot_joint = joint_idx;
            constraint_set_revolute_motor(world, joint_idx, true, 
                state->intake_speed_rpm * M_PI / 30.0f,  /* rad/s */
                0.5f);  /* max torque */
        }
    }
    
    state->intake_deployed = true;
}

/* ================================================================
 * Shooter Creation (Flywheel-based)
 * ================================================================ */

__attribute__((used)) void mfs_module_1_shooter_create(mfs_module_1_state *state) {
    physics_world *world = state->world;
    rigidbody *chassis = mfs_get_chassis(state);
    if (!chassis) return;
    
    /* Flywheel on a pylon clear of the chassis: at 0.8*H the tilted disc
     * grazed the chassis top inside the 10 mm slop band and slop friction
     * + joint fight killed spin by tick 3. 1.0*H clears slop with margin. */
    vector3 robot_pos = chassis->position;
    vector3 flywheel_pos = vector3_addition(robot_pos,
        (vector3){0.0f, MFS_ROBOT_CHASSIS_HEIGHT*1.0f, -MFS_ROBOT_CHASSIS_LENGTH*0.5f - 0.05f});
    
    int flywheel_idx = physics_world_add_cylinder(world,
        MFS_SHOOTER_FLYWHEEL_RADIUS,
        0.015f,  /* Thin flywheel */
        MFS_SHOOTER_FLYWHEEL_MASS,
        flywheel_pos);
    
    if (flywheel_idx >= 0) {
        state->shooter_flywheel_body = world->bodies[flywheel_idx].object_id;
        rigidbody *flywheel = &world->bodies[flywheel_idx];
        flywheel->restitution = 0.0f;
        flywheel->friction_static = 0.1f;
        flywheel->friction_kinetic = 0.05f;
        
        /* Revolute joint to chassis for flywheel spin */
        int joint_idx = constraint_add_revolute(world,
            chassis->object_id,
            flywheel->object_id,
            (vector3){0.0f, MFS_ROBOT_CHASSIS_HEIGHT*1.0f, -MFS_ROBOT_CHASSIS_LENGTH*0.5f - 0.05f},
            (vector3){0.0f, 0.0f, 0.0f},
            (vector3){0.0f, 1.0f, 0.0f});  /* spin axis = Y (horizontal) */
        if (joint_idx >= 0) {
            state->shooter_pivot_joint = joint_idx;
        }
        
        /* Angle the flywheel up by 35 degrees */
        float angle = MFS_SHOOTER_LAUNCH_ANGLE_DEG * M_PI / 180.0f;
        rigidbody *flywheel_body = &world->bodies[flywheel_idx];
        flywheel_body->orientation = vector4_from_axis_with_angle(
            (vector3){1.0f, 0.0f, 0.0f}, angle);
        rigidbody_update_axes(flywheel_body);
    }
    
    state->shooter_rpm = 0.0f;
    state->shooter_spinning_up = false;
    state->shooter_ready = false;
}

/* ================================================================
 * Ball Spawning & Physics
 * ================================================================ */

__attribute__((used)) void mfs_module_1_ball_spawn(mfs_module_1_state *state, vector3 pos) {
    if (state->ball_count >= state->max_balls) return;
    
    physics_world *world = state->world;
    int idx = physics_world_add_sphere(world,
        MFS_BIOBUZZ_BALL_RADIUS,
        MFS_BIOBUZZ_BALL_MASS,
        pos);
    
    if (idx >= 0) {
        rigidbody *ball = &world->bodies[idx];
        ball->restitution = MFS_BIOBUZZ_BALL_RESTITUTION;
        ball->friction_static = MFS_BIOBUZZ_BALL_FRICTION_S;
        ball->friction_kinetic = MFS_BIOBUZZ_BALL_FRICTION_K;
        ball->colour = (vector3){1.0f, 0.4f, 0.0f};  /* Orange */
        state->ball_body_ids[state->ball_count++] = ball->object_id;
    }
}

__attribute__((used)) void mfs_module_1_ball_physics_step(mfs_module_1_state *state, float dt) {
    (void)dt;
    physics_world *world = state->world;
    if (!world) return;
    
    const float air_density = 1.225f;
    const float drag_coeff = 0.47f;  /* Sphere */
    const float cross_section = M_PI * MFS_BIOBUZZ_BALL_RADIUS * MFS_BIOBUZZ_BALL_RADIUS;
    
    for (int i = 0; i < state->ball_count; i++) {
        int body_idx = physics_world_index_by_id(world, state->ball_body_ids[i]);
        if (body_idx < 0) continue;
        
        rigidbody *ball = &world->bodies[body_idx];
        
        /* Aerodynamic drag */
        float speed = vector3_length(ball->velocity);
        if (speed > 0.1f) {
            float drag_force = 0.5f * air_density * drag_coeff * cross_section * speed * speed;
            vector3 drag_dir = vector3_scaling(ball->velocity, -1.0f / speed);
            vector3 drag = vector3_scaling(drag_dir, drag_force);
            rb_apply_forces_perfect(ball, drag);
        }
        
        /* Magnus effect (lift from spin) */
        vector3 spin_axis = ball->angular_velocity;
        float spin_rate = vector3_length(spin_axis);
        if (spin_rate > 10.0f) {  /* Significant spin */
            vector3 spin_dir = vector3_scaling(spin_axis, 1.0f / spin_rate);
            vector3 vel_dir = (speed > 0.001f) ? vector3_scaling(ball->velocity, 1.0f / speed) : (vector3){0,0,0};
            
            /* Magnus force along spin x velocity (FIX-AUDIT-DESPOT direction
             * word: perpendicular-to-both is ambiguous about ORDER — the
             * model applies cross(spin_dir, vel_dir), i.e. spin-cross-
             * velocity, matching the code below).
             *
             * M6 MAGNUS SCALING FIX: the standard lift is
             *     F = 0.5*rho*A*v^2*Cl,  Cl = S*(omega*r)/v
             * which collapses to F = 0.5*rho*A*S*omega*r*v, i.e. LINEAR in
             * spin and velocity. The old expression had the right omega*v
             * shape but was missing the ball radius r, so the magnitude was
             * short by 1/r (~42x for a 24 mm ball). */
            vector3 magnus_dir = vector3_cross(spin_dir, vel_dir);
            float magnus_mag = 0.5f * air_density * cross_section *
                               spin_rate * MFS_BIOBUZZ_BALL_RADIUS * speed * 0.1f; /* S = 0.1 */
            vector3 magnus = vector3_scaling(magnus_dir, magnus_mag);
            rb_apply_forces_perfect(ball, magnus);
        }
    }
}

/* ================================================================
 * Intake Step
 * ================================================================ */

__attribute__((used)) void mfs_module_1_intake_step(mfs_module_1_state *state, float dt) {
    physics_world *world = state->world;
    /* FIX-AUDIT-DESPOT: was `intake_roller_body <= 0` + index_by_id — an
     * object-id checked with an index idiom (id 0/negative conflates
     * "unset" with "gone"). Resolve the id straight to a body pointer:
     * NULL means unset-or-gone, uniformly. */
    if (!state->intake_deployed) return;
    if (!world) return;
    rigidbody *roller =
        physics_world_body_by_id(world, (uint32_t)state->intake_roller_body);
    if (!roller) return;
    
    /* Control intake roller speed */
    float target_omega = state->intake_active ? 
        (state->intake_speed_rpm * M_PI / 30.0f) : 0.0f;
    
    /* M7 AXIAL PROJECTION FIX: the sign of the roller's spin was read from
     * the world X component of angular_velocity, while the drive torque below
     * is applied about the roller's own world axle (cached_axes[0]). When the
     * roller yaws with the chassis those two disagree, so the controller
     * compared the target against |omega| with the wrong sign and drove the
     * roller the wrong way (or oscillated). Project onto the axle instead:
     * the error is simply target minus actual axial spin. */
    float axial_omega = vector3_dot(roller->angular_velocity, roller->cached_axes[0]);
    float omega_error = target_omega - axial_omega;
    
    /* Simple P-control for intake motor */
    float torque = omega_error * 0.2f;  /* Proportional gain */
    if (torque > 0.5f) torque = 0.5f;
    if (torque < -0.5f) torque = -0.5f;

    /* Torque about the roller's world axle (cached_axes[0]), not raw X:
     * the roller yaws with the chassis. */
    roller->torque_accumulator =
        vector3_addition(roller->torque_accumulator,
                         vector3_scaling(roller->cached_axes[0], torque));
    
    /* Ball pickup detection: check contacts between intake and balls */
    if (state->intake_active) {
        for (int i = 0; i < state->ball_count; i++) {
            int ball_idx = physics_world_index_by_id(world, state->ball_body_ids[i]);
            if (ball_idx < 0) continue;
            
            rigidbody *ball = &world->bodies[ball_idx];
            
            /* Check if ball is near intake (simple distance check) */
            vector3 diff = vector3_subtraction(ball->position, roller->position);
            float dist = vector3_length(diff);
            float pickup_radius = MFS_INTAKE_ROLLER_RADIUS + MFS_BIOBUZZ_BALL_RADIUS + MFS_INTAKE_COMPLIANCE;
            
            if (dist < pickup_radius && ball->velocity.y < 0.5f) {
                /* FIX-AUDIT-DESPOT: balls_collected was never incremented
                 * (dead stat). Count each ball once, on first intake touch;
                 * the flag (not the distance edge) makes it tick-stable. */
                if (i >= 0 && i < 16 && !state->ball_counted[i]) {
                    state->ball_counted[i] = true;
                    state->balls_collected++;
                }
                /* M5 DIVIDE-BY-ZERO GUARD: at exact roller/ball coincidence
                 * dist is 0 and 1/dist is inf, which then propagates through
                 * the force accumulator as a NaN position and permanently
                 * poisons the body (rb_apply_forces_perfect rejects
                 * non-finite, but the entrain term below did not). Below the
                 * epsilon the direction is undefined, so use straight down:
                 * that is where a ball resting in the intake throat belongs,
                 * and the entrain term still applies either way. */
                vector3 to_roller = (dist > 1.0e-6f)
                    ? vector3_scaling(diff, -1.0f / dist)
                    : (vector3){0.0f, -1.0f, 0.0f};
                vector3 intake_force = vector3_scaling(to_roller, 2.0f);  /* 2N intake force */
                rb_apply_forces_perfect(ball, intake_force);
                
                /* Entrain the ball toward roller surface velocity: explicit
                 * rate 0.3/s times dt (dimensionless per-tick fraction).
                 * Compliant-contact stand-in, not a contact force. */
                vector3 roller_surf_vel = vector3_cross(roller->angular_velocity,
                    vector3_scaling(vector3_subtraction(ball->position, roller->position), 1.0f));
                ball->velocity = vector3_addition(ball->velocity,
                    vector3_scaling(roller_surf_vel, 0.3f * dt));
            }
        }
    }
}

/* ================================================================
 * Shooter Step
 * ================================================================ */

__attribute__((used)) void mfs_module_1_shooter_step(mfs_module_1_state *state, float dt) {
    physics_world *world = state->world;
    /* FIX-AUDIT-DESPOT: same id-vs-index cleanup as the intake step. */
    if (!world) return;
    rigidbody *flywheel =
        physics_world_body_by_id(world, (uint32_t)state->shooter_flywheel_body);
    if (!flywheel) return;
    /* Spin axis: the joint axis (0,1,0) tilted 35° about the chassis X at
     * creation. Reading/writing raw .y spun the wrong axis once tilted
     * (18% torque loss + rpm misread). Track the chassis frame so yaw
     * keeps the axis honest. */
    float tilt = MFS_SHOOTER_LAUNCH_ANGLE_DEG * (float)M_PI / 180.0f;
    vector3 sax = {0.0f, cosf(tilt), sinf(tilt)};
    {
        rigidbody *chassis = mfs_get_chassis(state);
        if (chassis) {
            sax = vector4_rotate_to_vector3(chassis->orientation, sax);
        }
    }
    float current_omega_y = vector3_dot(flywheel->angular_velocity, sax);
    state->shooter_rpm = fabsf(current_omega_y) * 30.0f / M_PI;

    /* Spin-up logic */
    if (state->shooter_spinup_cmd && !state->shooter_ready) {
        state->shooter_spinning_up = true;
        state->shooter_spinup_timer += dt;

        /* Apply spin-up torque */
        float target_omega = state->shooter_target_rpm * M_PI / 30.0f;
        float omega_error = target_omega - current_omega_y;
        float torque = omega_error * 0.05f;  /* Flywheel motor torque constant */
        if (torque > 0.3f) torque = 0.3f;
        flywheel->torque_accumulator =
            vector3_addition(flywheel->torque_accumulator, vector3_scaling(sax, torque));
        /* FIX-AUDIT-DESPOT shooter reaction couple: a flywheel motor is two
         * bodies acting on each other — mirror robot.c:704-713 and apply
         * -tau to the chassis about the same axis. Without this the
         * shooter spun up for free (angular momentum from nowhere) and the
         * chassis never felt the spin-up yaw/pitch kick a real mount does. */
        {
            rigidbody *chassis = mfs_get_chassis(state);
            if (chassis) {
                chassis->torque_accumulator =
                    vector3_subtraction(chassis->torque_accumulator,
                                        vector3_scaling(sax, torque));
            }
        }
        
        if (state->shooter_rpm >= state->shooter_target_rpm * 0.95f) {
            state->shooter_ready = true;
            state->shooter_spinning_up = false;
        }
    } else if (!state->shooter_spinup_cmd) {
        state->shooter_spinning_up = false;
        state->shooter_spinup_timer = 0.0f;
        state->shooter_ready = false;
    }
    
    /* Fire logic */
    if (state->shooter_fire_cmd && state->shooter_ready) {
        /* Find a ball in the shooter hopper (near flywheel) */
        for (int i = 0; i < state->ball_count; i++) {
            int ball_idx = physics_world_index_by_id(world, state->ball_body_ids[i]);
            if (ball_idx < 0) continue;
            
            rigidbody *ball = &world->bodies[ball_idx];
            vector3 diff = vector3_subtraction(ball->position, flywheel->position);
            float dist = vector3_length(diff);
            
            if (dist < 0.08f) {  /* Ball in shooting position */
                /* Launch ball: transfer flywheel surface velocity + angle */
                vector3 surface_vel = vector3_cross(flywheel->angular_velocity,
                    vector3_scaling(diff, 1.0f));
                float surf_speed = vector3_length(surface_vel);
                
                if (surf_speed > 5.0f) {
                    vector3 launch_dir = vector3_scaling(surface_vel, 1.0f / surf_speed);
                    /* C1 UNIT FIX: m*surf_speed is an IMPULSE (N.s), but
                     * rb_apply_forces_localised accumulates a FORCE (N) that
                     * the integrator turns into a = F/m. Passing the impulse
                     * straight through therefore delivered only
                     * 0.8*surf_speed*dt m/s of velocity - a factor of dt
                     * (60x at 1/60 s) too little, so fired balls barely moved.
                     * Convert the intended velocity change to the equivalent
                     * one-tick force: F = m*dv/dt. */
                    if (dt > 0.0f) {
                        float dv = surf_speed * 0.8f;  /* 80% transfer */
                        vector3 force = vector3_scaling(launch_dir,
                            MFS_BIOBUZZ_BALL_MASS * dv / dt);
                        rb_apply_forces_localised(ball, force, ball->position);
                    }

                    state->balls_fired++;  /* fired, not scored: no goal detection exists */
                    state->shooter_fire_cmd = false;  /* Consume fire command */
                    /* FIX-AUDIT-DESPOT: break lives INSIDE the success branch.
                     * The old break sat after the if, so a ball in position
                     * with low surface speed (or dt<=0) still ended the scan
                     * and the fire command starved behind an unshootable ball
                     * while a shootable one sat later in the list. */
                    break;  /* Only shoot one ball per fire command */
                }
            }
        }
    }
}

/* ================================================================
 * Robot Drive Step
 * ================================================================ */

__attribute__((used)) void mfs_module_1_robot_drive_step(mfs_module_1_state *state, float dt) {
    (void)dt;
    if (!state->robot_created) return;
    
    /* Canonical mecanum mixer (normalized, FTC rotate convention).
     * The hand mixer here was rotate-inverted vs drivetrain_mecanum and
     * clamped per-wheel without normalization, distorting combined
     * inputs. Single source of truth now. */
    drivetrain_mecanum(&state->robot, state->drive_forward, state->drive_strafe, state->drive_rotate);
}

/* ================================================================
 * Public API
 * ================================================================ */

__attribute__((used)) void mfs_module_1_set_drive_commands(mfs_module_1_state *state,
                                     float forward, float strafe, float rotate) {
    if (!state) return;
    state->drive_forward = forward;
    state->drive_strafe = strafe;
    state->drive_rotate = rotate;
}

__attribute__((used)) void mfs_module_1_set_intake(mfs_module_1_state *state, bool active) {
    if (!state) return;
    state->intake_active = active;
}

__attribute__((used)) void mfs_module_1_set_shooter(mfs_module_1_state *state, bool spinup, bool fire) {
    if (!state) return;
    state->shooter_spinup_cmd = spinup;
    state->shooter_fire_cmd = fire;
}

__attribute__((used)) void mfs_module_1_get_stats(const mfs_module_1_state *state,
                            int *balls_collected, int *balls_fired,
                            float *shooter_rpm, bool *shooter_ready) {
    if (!state) return;
    if (balls_collected) *balls_collected = state->balls_collected;
    if (balls_fired) *balls_fired = state->balls_fired;
    if (shooter_rpm) *shooter_rpm = state->shooter_rpm;
    if (shooter_ready) *shooter_ready = state->shooter_ready;
}

/* ================================================================
 * Helper: get chassis body
 * ================================================================ */


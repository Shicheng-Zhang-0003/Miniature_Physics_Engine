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
    float right_y = gamepad_get_axis(pad, gamepad_axis_right_y);
    float lt = gamepad_get_axis(pad, gamepad_axis_left_trigger);
    float rt = gamepad_get_axis(pad, gamepad_axis_right_trigger);
    
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
    bool b_pressed = btn_b && !state->prev_button_b;
    bool x_pressed = btn_x && !state->prev_button_x;
    bool y_pressed = btn_y && !state->prev_button_y;
    bool lb_pressed = btn_lb && !state->prev_button_lb;
    bool rb_pressed = btn_rb && !state->prev_button_rb;
    bool lt_pressed = (lt > 0.5f) && !state->prev_left_trigger;
    bool rt_pressed = (rt > 0.5f) && !state->prev_right_trigger;
    
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
    
    /* Toggle gamepad control with Start button */
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
    
    /* Intake toggle (A button) */
    if (a_pressed) {
        state->intake_active = !state->intake_active;
    }
    
    /* Intake reverse with B button (hold) */
    if (btn_b) {
        state->intake_active = true;
        state->intake_power = -1.0f;  /* reverse */
    } else if (a_pressed || !btn_b) {
        state->intake_power = 1.0f;
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
            chassis->position = (vector3){0.0f, 0.2f, -3.0f};
            chassis->velocity = vector3_zero();
            chassis->angular_velocity = vector3_zero();
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
    
    /* Update FTC robot motors (applies torque to wheel bodies) */
    ftc_robot_update(world, &state->robot, dt);
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
        state->robot.battery.internal_resistance = 0.015f;
        state->robot.battery.charge_fraction = 1.0f;
        
        /* Initialize wheel roller angles for mecanum (+45/-45 layout) */
        for (int i = 0; i < state->robot.wheel_count; i++) {
            state->robot.wheel_is_mecanum[i] = true;
            state->robot.wheel_roller_angle[i] = (i % 2 == 0) ? 0.785398f : -0.785398f; /* +45, -45 deg */
            state->robot.wheel_traction_scale[i] = 1.0f;
        }
        
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
    
    /* Flywheel positioned at top-rear of chassis, angled up */
    vector3 robot_pos = chassis->position;
    vector3 flywheel_pos = vector3_addition(robot_pos,
        (vector3){0.0f, MFS_ROBOT_CHASSIS_HEIGHT*0.8f, -MFS_ROBOT_CHASSIS_LENGTH*0.5f - 0.05f});
    
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
            (vector3){0.0f, MFS_ROBOT_CHASSIS_HEIGHT*0.8f, -MFS_ROBOT_CHASSIS_LENGTH*0.5f - 0.05f},
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
            
            /* Magnus force perpendicular to both spin and velocity */
            vector3 magnus_dir = vector3_cross(spin_dir, vel_dir);
            float magnus_mag = 0.5f * air_density * cross_section * spin_rate * speed * 0.1f;  /* S = 0.1 typical */
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
    if (!state->intake_deployed || state->intake_roller_body <= 0) return;
    
    int roller_idx = physics_world_index_by_id(world, state->intake_roller_body);
    if (roller_idx < 0) return;
    
    rigidbody *roller = &world->bodies[roller_idx];
    
    /* Control intake roller speed */
    float target_omega = state->intake_active ? 
        (state->intake_speed_rpm * M_PI / 30.0f) : 0.0f;
    
    float current_omega = vector3_length(roller->angular_velocity);
    float omega_error = target_omega - (roller->angular_velocity.x > 0 ? current_omega : -current_omega);
    
    /* Simple P-control for intake motor */
    float torque = omega_error * 0.2f;  /* Proportional gain */
    if (torque > 0.5f) torque = 0.5f;
    if (torque < -0.5f) torque = -0.5f;
    
    roller->torque_accumulator.x += torque;
    
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
                /* Apply intake force to pull ball in */
                vector3 to_roller = vector3_scaling(diff, -1.0f / dist);
                vector3 intake_force = vector3_scaling(to_roller, 2.0f);  /* 2N intake force */
                rb_apply_forces_perfect(ball, intake_force);
                
                /* Also apply roller surface velocity to ball (compliant contact) */
                vector3 roller_surf_vel = vector3_cross(roller->angular_velocity, 
                    vector3_scaling(vector3_subtraction(ball->position, roller->position), 1.0f));
                ball->velocity = vector3_addition(ball->velocity, 
                    vector3_scaling(roller_surf_vel, 0.3f * dt));  /* 30% velocity transfer */
            }
        }
    }
}

/* ================================================================
 * Shooter Step
 * ================================================================ */

__attribute__((used)) void mfs_module_1_shooter_step(mfs_module_1_state *state, float dt) {
    physics_world *world = state->world;
    if (state->shooter_flywheel_body <= 0) return;
    
    int flywheel_idx = physics_world_index_by_id(world, state->shooter_flywheel_body);
    if (flywheel_idx < 0) return;
    
    rigidbody *flywheel = &world->bodies[flywheel_idx];
    float current_omega_y = flywheel->angular_velocity.y;
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
        flywheel->torque_accumulator.y += torque;
        
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
                    /* Apply launch impulse */
                    vector3 impulse = vector3_scaling(launch_dir, 
                        MFS_BIOBUZZ_BALL_MASS * surf_speed * 0.8f);  /* 80% transfer */
                    rb_apply_forces_localised(ball, impulse, ball->position);
                    
                    state->balls_scored++;  /* Count as shot (scoring checked separately) */
                    state->shooter_fire_cmd = false;  /* Consume fire command */
                }
                break;  /* Only shoot one ball per fire command */
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
    
    /* Set wheel commands from drive inputs */
    float commands[4] = {
        state->drive_forward + state->drive_strafe + state->drive_rotate,  /* FL */
        state->drive_forward - state->drive_strafe - state->drive_rotate,  /* FR */
        state->drive_forward - state->drive_strafe + state->drive_rotate,  /* RL */
        state->drive_forward + state->drive_strafe - state->drive_rotate   /* RR */
    };
    
    /* Clamp to [-1, 1] */
    for (int i = 0; i < 4; i++) {
        if (commands[i] > 1.0f) commands[i] = 1.0f;
        if (commands[i] < -1.0f) commands[i] = -1.0f;
    }
    
    ftc_robot_set_wheel_commands(&state->robot, commands, 4);
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
                            int *balls_collected, int *balls_scored,
                            float *shooter_rpm, bool *shooter_ready) {
    if (!state) return;
    if (balls_collected) *balls_collected = state->balls_collected;
    if (balls_scored) *balls_scored = state->balls_scored;
    if (shooter_rpm) *shooter_rpm = state->shooter_rpm;
    if (shooter_ready) *shooter_ready = state->shooter_ready;
}

/* ================================================================
 * Helper: get chassis body
 * ================================================================ */


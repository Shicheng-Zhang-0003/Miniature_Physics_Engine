/* MFS Suite v2 — file C: hotload + module_1 (3 tests). */
#include <math.h>
#include <stdio.h>
#include <dlfcn.h>
#include <string.h>
#include "mfs_test.h"
#include "core/mpe_loader.h"
#include "ecosystem/mpe_ecosystem.h"
#include "modules/module_1/mfs_module_1.h"

#define DT (1.0f/60.0f)
#define FTC_ITERS 128

/* Forward declarations for physics_truth tests (defined in suite_b) */
extern int mfs_t_freefall(void);
extern int mfs_t_inertia(void);
extern int mfs_t_bounce(void);
extern int mfs_t_rolling(void);
extern int mfs_t_rolling_resistance(void);
extern int mfs_t_motor_free_speed(void);
extern int mfs_t_motor_stall(void);
extern int mfs_t_back_emf(void);
extern int mfs_t_static_friction(void);
extern int mfs_t_kinetic_friction(void);
extern int mfs_t_stability(void);
extern int mfs_t_coast_down(void);
extern int mfs_t_energy(void);
extern int mfs_t_cylinder_rest(void);
extern int mfs_t_revolute_anchor(void);

/* ftc_hotload: static vs dlopen bitwise */
int mfs_t_ftc_hotload(void) {
    mfs_test_t t; mfs_test_begin(&t, "ftc_hotload"); mfs_test_t *t_ptr = &t;
    physics_world w; mfs_test_world(&w);
    g_cfg.timestep.solver_iterations = FTC_ITERS;
    constraint_pool_init(&w);

    /* Static build path */
    ftc_robot robot_static;
    int rc = ftc_robot_create_with_drive(&w, &robot_static, 0.0f, ftc_robot_rest_height(), 0.0f,
                                         MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_TANK);
    if (rc != 0) { t_ptr->failures++; return t_ptr->failures; }
    const float dt = DT;
    for (int t_tick = 0; t_tick < 180; t_tick++) {
        drivetrain_tank(&robot_static, 1.0f, 1.0f);
        drivetrain_update(&w, &robot_static, dt);
        physics_world_step(&w, dt);
    }
    float pos_static[3];
    ftc_robot_get_position(&w, &robot_static, &pos_static[0], &pos_static[1], &pos_static[2]);
    physics_world_cleanup(&w);

    /* Dynamic path: dlopen */
    physics_world w2; mfs_test_world(&w2);
    g_cfg.timestep.solver_iterations = FTC_ITERS;
    constraint_pool_init(&w2);
    void *handle = dlopen("./plugins/mpe_ftc.so", RTLD_NOW);
    if (!handle) { t_ptr->failures++; return t_ptr->failures; }
    extern const mpe_module_desc_t mpe_module_desc;
    if (dlsym(handle, "mpe_module_desc") != &mpe_module_desc) { t_ptr->failures++; }
    void *state = NULL;
    int (*attach)(void*, void**) = dlsym(handle, "attach");
    int (*detach)(void*, void*) = dlsym(handle, "detach");
    void (*pre_step)(void*, float, void*) = dlsym(handle, "pre_step");
    void *spawn = dlsym(handle, "ftc_fleet_spawn");
    void *get_fn = dlsym(handle, "ftc_fleet_get");
    void *tank = dlsym(handle, "drivetrain_tank");
    if (!attach || !detach || !pre_step || !spawn || !get_fn || !tank) { t_ptr->failures++; }
    if (attach(&w2, &state) != 0) { t_ptr->failures++; dlclose(handle); return t_ptr->failures; }
    int (*spawn_fn)(void*, float, float, float, int, int) = spawn;
    int idx = spawn_fn(&w2, 0, ftc_robot_rest_height(), 0, MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_TANK);
    if (idx < 0) { t_ptr->failures++; }
    ftc_robot *r = ((ftc_robot* (*)(void*, int))get_fn)(&w2, idx);
    if (!r) { t_ptr->failures++; }
    for (int t_tick = 0; t_tick < 180; t_tick++) {
        ((void(*)(void*,float,float))tank)(r, 1.0f, 1.0f);
        drivetrain_update(&w2, r, dt);
        physics_world_step(&w2, dt);
    }
    float pos_dynamic[3];
    ftc_robot_get_position(&w2, r, &pos_dynamic[0], &pos_dynamic[1], &pos_dynamic[2]);
    int bitwise = (memcmp(pos_static, pos_dynamic, sizeof(pos_static)) == 0);
    if (!bitwise) t_ptr->failures++;
    detach(&w2, state); dlclose(handle);
    physics_world_cleanup(&w2);
    return t_ptr->failures;
}

/* module_1: BioBuzz attach/tick/detach */
int mfs_t_module_1(void) {
    mfs_test_t t; mfs_test_begin(&t, "module_1"); mfs_test_t *t_ptr = &t;
    physics_world w; mfs_test_world(&w);
    g_cfg.timestep.solver_iterations = FTC_ITERS;
    constraint_pool_init(&w);

    extern const mpe_module_desc_t mfs_module_1_desc;
    void *state = NULL;
    if (mfs_module_1_attach(&w, &state) != 0) { t_ptr->failures++; return t_ptr->failures; }
    const float dt = DT;
    int fail = 0;
    for (int t_tick = 0; t_tick < 100 && !fail; t_tick++) {
        mfs_module_1_pre_step(&w, dt, state);
        physics_world_step(&w, dt);
        mfs_module_1_post_step(&w, dt, state);
        if (!mfs_test_finite(&w)) { fail = 1; break; }
    }
    if (!fail) {
        int dist = 0;
        mfs_module_1_state *ms = (mfs_module_1_state*)state;
        for (int i = 0; i < 16; i++) {
            if (ms->ball_body_ids[i] >= 0 && ms->ball_body_ids[i] < w.body_count) {
                rigidbody *b = &w.bodies[ms->ball_body_ids[i]];
                float dx = b->position.x - (-4.0f + (i%3)*0.5f);
                float dz = b->position.z - (-2.0f + (i/3)*0.5f);
                float d = sqrtf(dx*dx + dz*dz);
                if (d > dist) dist = d;
            }
        }
        if (dist < 1) t_ptr->failures++;
        if (ms->balls_fired < 1) t_ptr->failures++;
    }
    mfs_module_1_detach(&w, state);
    physics_world_cleanup(&w);
    return t_ptr->failures;
}

/* physics_truth: re-exported from suite_b for unified run */
int mfs_t_physics_truth(void) {
    return mfs_t_freefall() + mfs_t_inertia() + mfs_t_bounce() +
           mfs_t_rolling() + mfs_t_rolling_resistance() +
           mfs_t_motor_free_speed() + mfs_t_motor_stall() +
           mfs_t_back_emf() + mfs_t_static_friction() +
           mfs_t_kinetic_friction() + mfs_t_stability() +
           mfs_t_coast_down() + mfs_t_energy() +
           mfs_t_cylinder_rest() + mfs_t_revolute_anchor();
}
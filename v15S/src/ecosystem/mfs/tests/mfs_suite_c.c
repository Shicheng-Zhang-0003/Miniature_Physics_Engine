/* MFS Suite v2 — file C: hotload + module_1 (3 tests). */
#include <math.h>
#include <stdio.h>
#include <dlfcn.h>
#include <string.h>
#include "mfs_test.h"
#include "core/mpe_loader.h"
#include "ecosystem/mpe_ecosystem.h"
#include "modules/ftc/ftc_fleet.h"
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

/* ftc_hotload: static vs dlopen bitwise + detach lifecycle.
 * DESPOT-2026-09-26: faithful port of tests/ftc_hotload_test.c. The prior
 * unified-suite version was a broken simplification: it dlsym'd standalone
 * attach/detach/pre_step symbols the .so never exports (they are struct
 * members) and called the NULL attach -> SIGSEGV whenever CWD resolved the
 * plugin; it also never registered the fleet with the world, so spawn
 * returned -1. This version uses the real contract: dlopen the descriptor,
 * physics_world_attach_module (which calls desc->attach AND registers the
 * fleet in tick_modules so spawn finds it), drive via pre_step inside
 * physics_world_step (no manual drivetrain_update — that would double-drive),
 * bitwise-compare pose+odometry, then detach/coast/re-attach lifecycle. */
int mfs_t_ftc_hotload(void) {
    mfs_test_t t; mfs_test_begin(&t, "ftc_hotload"); mfs_test_t *t_ptr = &t;
    extern const mpe_module_desc_t mpe_module_desc;
    const float dt = DT;

    /* Static path: linked-in desc, fleet spawn, pre_step-driven. */
    physics_world w1; mfs_test_world(&w1);
    g_cfg.timestep.solver_iterations = FTC_ITERS;
    constraint_pool_init(&w1);
    MFS_CHECK(t_ptr, physics_world_attach_module(&w1, &mpe_module_desc) >= 0);
    /* Refusal probe on a SEPARATE unattached world (spawning here would
     * consume fleet slot 0 and shift every later index). */
    {
        physics_world w0; mfs_test_world(&w0);
        constraint_pool_init(&w0);
        MFS_CHECK(t_ptr, ftc_fleet_spawn(&w0, 0, 0, 0, 0, 0) == -1);
        MFS_CHECK(t_ptr, ftc_fleet_count(&w0) == 0);
        MFS_CHECK(t_ptr, ftc_fleet_get(&w0, 0) == NULL);
        physics_world_cleanup(&w0);
    }
    int s0 = ftc_fleet_spawn(&w1, 0.0f, ftc_robot_rest_height(), 0.0f,
                             MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_MECANUM);
    MFS_CHECK(t_ptr, s0 == 0);
    ftc_robot *r1 = ftc_fleet_get(&w1, 0);
    MFS_CHECK(t_ptr, r1 != NULL);
    float sx = 0, sy = 0, sz = 0;
    if (r1) ftc_robot_get_position(&w1, r1, &sx, &sy, &sz);
    int ok1 = 1;
    for (int k = 0; k < 180 && ok1; k++) {
        if (r1) drivetrain_mecanum(r1, 1.0f, 0.0f, 0.0f);
        physics_world_step(&w1, dt); /* pre_step drives the fleet */
        if (!mfs_test_finite(&w1)) { ok1 = 0; }
    }
    MFS_CHECK(t_ptr, ok1);
    float ex = sx, ey = sy, ez = sz;
    if (r1) ftc_robot_get_position(&w1, r1, &ex, &ey, &ez);
    {
        float dx = ex - sx, dz = ez - sz;
        MFS_CHECK(t_ptr, sqrtf(dx * dx + dz * dz) >= 0.5f);
    }

    /* Dynamic path: identical script through the .so descriptor. */
    void *handle = dlopen("./plugins/mpe_ftc.so", RTLD_NOW);
    if (!handle) { t_ptr->failures++; physics_world_cleanup(&w1); return t_ptr->failures; }
    const mpe_module_desc_t *dyn_desc = dlsym(handle, "mpe_module_desc");
    typedef int (*spawn_fn_t)(struct physics_world *, float, float, float, int, int);
    typedef ftc_robot *(*get_fn_t)(struct physics_world *, int);
    typedef void (*mec_fn_t)(ftc_robot *, float, float, float);
    spawn_fn_t dyn_spawn = handle ? (spawn_fn_t)dlsym(handle, "ftc_fleet_spawn") : NULL;
    get_fn_t dyn_get = handle ? (get_fn_t)dlsym(handle, "ftc_fleet_get") : NULL;
    mec_fn_t dyn_mec = handle ? (mec_fn_t)dlsym(handle, "drivetrain_mecanum") : NULL;
    MFS_CHECK(t_ptr, dyn_desc != NULL && dyn_spawn && dyn_get && dyn_mec);
    MFS_CHECK(t_ptr, dyn_desc != &mpe_module_desc);
    physics_world w2; mfs_test_world(&w2);
    g_cfg.timestep.solver_iterations = FTC_ITERS;
    constraint_pool_init(&w2);
    if (dyn_desc) MFS_CHECK(t_ptr, physics_world_attach_module(&w2, dyn_desc) >= 0);
    int d0 = -1;
    if (dyn_spawn) d0 = dyn_spawn(&w2, 0.0f, ftc_robot_rest_height(), 0.0f,
                                  MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_MECANUM);
    MFS_CHECK(t_ptr, d0 == 0);
    ftc_robot *r2 = dyn_get ? dyn_get(&w2, 0) : NULL;
    MFS_CHECK(t_ptr, r2 != NULL);
    float tx = 0, ty = 0, tz = 0;
    if (r2) ftc_robot_get_position(&w2, r2, &tx, &ty, &tz);
    int ok2 = 1;
    for (int k = 0; k < 180 && ok2; k++) {
        if (r2 && dyn_mec) dyn_mec(r2, 1.0f, 0.0f, 0.0f);
        physics_world_step(&w2, dt);
        if (!mfs_test_finite(&w2)) { ok2 = 0; }
    }
    MFS_CHECK(t_ptr, ok2);
    float fx = tx, fy = ty, fz = tz;
    if (r2) ftc_robot_get_position(&w2, r2, &fx, &fy, &fz);
    {
        float dx = fx - tx, dz = fz - tz;
        MFS_CHECK(t_ptr, sqrtf(dx * dx + dz * dz) >= 0.5f);
    }

    /* Bitwise equivalence static vs dynamic (pose + odometry). */
    if (r1 && r2) {
        int same = (memcmp(&ex, &fx, 4) == 0) && (memcmp(&ey, &fy, 4) == 0) &&
                   (memcmp(&ez, &fz, 4) == 0) &&
                   (memcmp(&r1->odom_x, &r2->odom_x, 4) == 0) &&
                   (memcmp(&r1->odom_z, &r2->odom_z, 4) == 0) &&
                   (memcmp(&r1->odom_theta, &r2->odom_theta, 4) == 0);
        MFS_CHECK(t_ptr, same);
    } else {
        MFS_CHECK(t_ptr, 0);
    }

    /* Detach lifecycle on the dynamic world (r2 dangles after detach). */
    int w2chassis = (r2) ? r2->chassis_body : -1;
    if (dyn_desc) MFS_CHECK(t_ptr, physics_world_detach_module(&w2, "ftc-fleet") == 0);
    MFS_CHECK(t_ptr, ftc_fleet_count(&w1) == 1);
    int okc = 1;
    for (int k = 0; k < 60 && okc; k++) {
        physics_world_step(&w2, dt);
        if (!mfs_test_finite(&w2)) { okc = 0; }
    }
    MFS_CHECK(t_ptr, okc);
    if (w2chassis >= 0 && w2chassis < w2.body_count) {
        float gx = w2.bodies[w2chassis].position.x;
        float gz = w2.bodies[w2chassis].position.z;
        float ddx = gx - fx, ddz = gz - fz;
        MFS_CHECK(t_ptr, sqrtf(ddx * ddx + ddz * ddz) < 2.0f);
    }
    if (handle) dlclose(handle);
    physics_world_detach_module(&w1, "ftc-fleet");
    physics_world_cleanup(&w1);
    physics_world_cleanup(&w2);
    return t_ptr->failures;
}

/* module_1: BioBuzz attach/tick/detach.
 * DESPOT-2026-09-26: faithful port of modules/module_1/mfs_module_1_test.c.
 * The prior unified version never issued drive/shooter/fire commands and
 * never staged a ball at the flywheel, then gated on balls having moved and
 * fired — a guaranteed FAIL (it measured an idle world). This version keeps
 * the old command schedule (drive+intake at tick 30, shooter at 50, stage +
 * fire at 80) and the calibrated gates (drive >= 0.5 m, shooter >= 3000 rpm,
 * fired >= 1). */
int mfs_t_module_1(void) {
    mfs_test_t t; mfs_test_begin(&t, "module_1"); mfs_test_t *t_ptr = &t;
    physics_world w; mfs_test_world(&w);
    g_cfg.timestep.solver_iterations = FTC_ITERS;
    constraint_pool_init(&w);

    extern const mpe_module_desc_t mfs_module_1_desc;
    void *state = NULL;
    if (mfs_module_1_attach(&w, &state) != 0) { t_ptr->failures++; return t_ptr->failures; }
    mfs_module_1_state *ms = (mfs_module_1_state *)state;
    const float dt = DT;
    int fail = 0;
    for (int tick = 0; tick < 100 && !fail; tick++) {
        if (tick == 30) {
            mfs_module_1_set_drive_commands(ms, 1.0f, 0.0f, 0.0f);
            mfs_module_1_set_intake(ms, true);
        }
        if (tick == 50) {
            mfs_module_1_set_shooter(ms, true, false);
        }
        if (tick == 80 && ms->ball_count > 0) {
            int fw = physics_world_index_by_id(&w, ms->shooter_flywheel_body);
            int b0 = physics_world_index_by_id(&w, ms->ball_body_ids[0]);
            if (fw >= 0 && b0 >= 0) {
                w.bodies[b0].position = vector3_addition(
                    w.bodies[fw].position, (vector3){0.05f, 0.0f, 0.0f});
                w.bodies[b0].velocity = vector3_zero();
            }
            mfs_module_1_set_shooter(ms, true, true);
        }
        mfs_module_1_pre_step(&w, dt, state);
        physics_world_step(&w, dt);
        mfs_module_1_post_step(&w, dt, state);
        if (!mfs_test_finite(&w)) { fail = 1; break; }
    }
    if (fail) {
        t_ptr->failures++;
    } else {
        rigidbody *chassis = mfs_get_chassis(ms);
        MFS_CHECK(t_ptr, chassis != NULL);
        if (chassis) {
            float dist = sqrtf(chassis->position.x * chassis->position.x +
                               chassis->position.z * chassis->position.z);
            MFS_CHECK(t_ptr, dist >= 0.5f);
        }
        MFS_CHECK(t_ptr, ms->shooter_rpm >= 3000.0f);
        MFS_CHECK(t_ptr, ms->balls_fired >= 1);
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
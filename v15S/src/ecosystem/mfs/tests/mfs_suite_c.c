/* MFS Suite v2 — file C: hotload + module_1 (3 tests). */
#include <math.h>
#include <stdio.h>
#include "core/mpe_platform.h"
#include <string.h>
#include "mfs_test.h"
#include "core/mpe_loader.h"
#include "ecosystem/mpe_ecosystem.h"
#include "modules/ftc/ftc_fleet.h"
#include "modules/module_1/mfs_module_1.h"

/* Windows-aware plugin path: pick existing .so/.dll variant. */
static const char *mpe_pick_plugin(const char *so_path, char *buf, size_t n) {
#ifdef MPE_OS_WINDOWS
    /* so_path like "plugins/mpe_capsule.so": try as-is, then .dll variant. */
    if (access(so_path, R_OK) == 0) return so_path;
    size_t L = strlen(so_path);
    if (L > 3 && strcmp(so_path + L - 3, ".so") == 0) {
        snprintf(buf, n, "%.*s.dll", (int)(L - 3), so_path);
        if (access(buf, R_OK) == 0) return buf;
    } else if (L > 4 && _stricmp(so_path + L - 4, ".dll") == 0) {
        snprintf(buf, n, "%.*s.so", (int)(L - 4), so_path);
        if (access(buf, R_OK) == 0) return buf;
    }
    /* try MPE_PLUGIN_EXT variant of basename */
    return so_path;
#else
    (void)buf; (void)n;
    return so_path;
#endif
}


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

    /* Dynamic path: identical script through the plugin descriptor (.so/.dll). */
    char ftc_buf[1024]; const char *ftc_path = mpe_pick_plugin("./plugins/mpe_ftc.so", ftc_buf, sizeof(ftc_buf));
    /* fallback: also try without ./ prefix (MSYS2 vs native CWD) */
    if (access(ftc_path, R_OK) != 0) { ftc_path = mpe_pick_plugin("plugins/mpe_ftc.so", ftc_buf, sizeof(ftc_buf)); }
    void *handle = dlopen(ftc_path, RTLD_NOW);
    /* DESPOT-2026-09-28: early return leaked saved config (no end). */
    if (!handle) { t_ptr->failures++; physics_world_cleanup(&w1); mfs_test_end(t_ptr); return t_ptr->failures; }
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
    /* DESPOT-2026-09-28: begin without end leaked config (and never
     * checked det counters) — end restores + asserts zero fallbacks. */
    mfs_test_end(t_ptr);
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
    /* DESPOT-2026-09-28: attach-fail return leaked the world + config. */
    if (mfs_module_1_attach(&w, &state) != 0) {
        t_ptr->failures++;
        physics_world_cleanup(&w);
        mfs_test_end(t_ptr);
        return t_ptr->failures;
    }
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
    /* DESPOT-2026-09-28: begin without end (see hotload). */
    mfs_test_end(t_ptr);
    return t_ptr->failures;
}

/* physics_truth: re-exported from suite_b for unified run */
int mfs_t_physics_truth(void) {
    /* DESPOT-2026-09-28 (programming: suite_b raw tests never reset/check
     * det counters — only freefall uses begin/end). Aggregate gate over all
     * 15 subtests: any out-of-contract transcendental anywhere fails loud.
     * Config needs no save here: every raw subtest starts with
     * mpe_config_init() (init IS the isolation), and freefall's begin/end
     * restores around itself. */
    det_fallback_reset();
    int rc = mfs_t_freefall() + mfs_t_inertia() + mfs_t_bounce() +
           mfs_t_rolling() + mfs_t_rolling_resistance() +
           mfs_t_motor_free_speed() + mfs_t_motor_stall() +
           mfs_t_back_emf() + mfs_t_static_friction() +
           mfs_t_kinetic_friction() + mfs_t_stability() +
           mfs_t_coast_down() + mfs_t_energy() +
           mfs_t_cylinder_rest() + mfs_t_revolute_anchor();
    {
        unsigned long fp = det_fallback_pow_total();
        unsigned long ft = det_fallback_trig_total();
        if (fp != 0 || ft != 0) {
            printf("[FAIL] physics_truth: det fallbacks pow=%lu trig=%lu (want 0)\n", fp, ft);
            rc++;
        }
    }
    return rc;
}
/* mfs_t_intake_stop: GATED regression for MFS H5 (DESPOT-2026-09-29).
 *
 * The intake roller had two actuators: a revolute joint motor enabled once at
 * creation and never touched again, plus a P-control applying torque straight
 * to the roller body. The P-control targeted 0 when intake_active was false,
 * but the joint motor kept driving at the creation-time speed, so THE INTAKE
 * COULD NOT BE STOPPED. The 600-1200 RPM speed slider never reached the motor
 * and `intake_power` (momentary reverse) was written twice and read by
 * nothing.
 *
 * This is the test whose absence let that survive: nothing in the gated suite
 * ever switched the intake off and checked that it stopped.
 */
int mfs_t_intake_stop(void) {
    mfs_test_t t; mfs_test_begin(&t, "intake_stop"); mfs_test_t *t_ptr = &t;
    physics_world w; mfs_test_world(&w);
    g_cfg.timestep.solver_iterations = FTC_ITERS;
    constraint_pool_init(&w);

    extern const mpe_module_desc_t mfs_module_1_desc;
    void *state = NULL;
    if (mfs_module_1_attach(&w, &state) != 0) {
        t_ptr->failures++;
        physics_world_cleanup(&w);
        mfs_test_end(t_ptr);
        return t_ptr->failures;
    }
    mfs_module_1_state *ms = (mfs_module_1_state *)state;
    const float dt = DT;
    int fail = 0;

    /* Resolve the roller once, by id, the way the module does. */
    MFS_CHECK(t_ptr, ms->intake_roller_body >= 0);
    MFS_CHECK(t_ptr, ms->intake_pivot_joint >= 0);
    if (ms->intake_roller_body < 0 || ms->intake_pivot_joint < 0) {
        physics_world_cleanup(&w);
        mfs_test_end(t_ptr);
        return t_ptr->failures;
    }

    float omega_on = 0.0f;
    /* Phase 1: intake ON, let it spin up. */
    mfs_module_1_set_intake(ms, true);
    for (int tick = 0; tick < 120 && !fail; tick++) {
        mfs_module_1_pre_step(&w, dt, state);
            mfs_module_1_post_step(&w, dt, state);
        physics_world_step(&w, dt);
        if (!mfs_test_finite(&w)) fail = 1;
    }
    if (!fail) {
        rigidbody *roller = physics_world_body_by_id(&w, (uint32_t)ms->intake_roller_body);
        MFS_CHECK(t_ptr, roller != NULL);
        if (roller) {
            omega_on = vector3_dot(roller->angular_velocity, roller->cached_axes[0]);
        }
        MFS_INFO("intake ON: axial omega=%.3f rad/s", omega_on);
        /* It must actually be spinning, or "it stopped later" proves nothing. */
        MFS_CHECK(t_ptr, fabsf(omega_on) > 1.0f);
    }

    /* Phase 2: intake OFF. This is the H5 assertion. */
    if (!fail) {
        mfs_module_1_set_intake(ms, false);
        for (int tick = 0; tick < 180 && !fail; tick++) {
            mfs_module_1_pre_step(&w, dt, state);
            mfs_module_1_post_step(&w, dt, state);
            physics_world_step(&w, dt);
            if (!mfs_test_finite(&w)) fail = 1;
        }
        rigidbody *roller = physics_world_body_by_id(&w, (uint32_t)ms->intake_roller_body);
        MFS_CHECK(t_ptr, roller != NULL);
        if (roller) {
            float omega_off = vector3_dot(roller->angular_velocity, roller->cached_axes[0]);
            MFS_INFO("intake OFF: axial omega=%.3f rad/s (was %.3f)", omega_off, omega_on);
            /* Pre-fix the joint motor held the full creation-time speed here
             * forever, so this is the assertion that actually pins H5. */
            MFS_CHECK(t_ptr, fabsf(omega_off) < 0.25f * fabsf(omega_on));
            if (t_ptr->failures == 0) {
                printf("[PASS] intake stops when disabled\n");
            }
        }
    }

    /* Phase 3: momentary reverse must actually reverse (intake_power was
     * dead code -- written, never read). */
    if (!fail) {
        mfs_module_1_set_intake(ms, true);
        for (int tick = 0; tick < 120 && !fail; tick++) {
            mfs_module_1_pre_step(&w, dt, state);
            mfs_module_1_post_step(&w, dt, state);
            physics_world_step(&w, dt);
            if (!mfs_test_finite(&w)) fail = 1;
        }
        ms->intake_power = -1.0f;
        for (int tick = 0; tick < 180 && !fail; tick++) {
            mfs_module_1_pre_step(&w, dt, state);
            mfs_module_1_post_step(&w, dt, state);
            physics_world_step(&w, dt);
            if (!mfs_test_finite(&w)) fail = 1;
        }
        ms->intake_power = 0.0f;
        rigidbody *roller = physics_world_body_by_id(&w, (uint32_t)ms->intake_roller_body);
        MFS_CHECK(t_ptr, roller != NULL);
        if (roller) {
            float omega_rev = vector3_dot(roller->angular_velocity, roller->cached_axes[0]);
            MFS_INFO("intake REVERSE: axial omega=%.3f rad/s", omega_rev);
            MFS_CHECK(t_ptr, omega_rev < -0.5f);
            if (t_ptr->failures == 0) {
                printf("[PASS] intake reverses on intake_power < 0\n");
            }
        }
    }

    if (state) mfs_module_1_detach(&w, state);
    physics_world_cleanup(&w);
    mfs_test_end(t_ptr);
    return t_ptr->failures;
}

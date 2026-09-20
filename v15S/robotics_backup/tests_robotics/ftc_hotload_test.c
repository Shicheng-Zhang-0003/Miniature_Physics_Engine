/* MPE_FTC_HOTLOAD: the FTC fleet as a hot-pluggable kernel module.
 * Proves the import story end to end WITHOUT engine changes:
 *  1. static attach: &mpe_module_desc attached, fleet spawned, world
 *     stepped with no manual drivetrain_update() calls.
 *  2. dynamic import: dlopen(".../mpe_ftc.so") via mpe_loader_load (the
 *     same call `mod load` makes), dlsym the descriptor + fleet API,
 *     attach THAT desc to a second world, drive it.
 *  3. equivalence: same commands, same ticks -> bitwise-identical
 *     chassis pose + odometry across the static and dynamic copies.
 *  4. detach/re-attach lifecycle on the dynamic world.
 *
 * Build/run: robotics_backup/build_tests.sh (sets FTC_SO); or
 *   gcc ... -DMPE_FTC_HOTLOAD_TEST ... -ldl && FTC_SO=path ./a.out
 */
#ifdef MPE_FTC_HOTLOAD_TEST
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <dlfcn.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include "core/mpe_registry.h"
#include "core/mpe_loader.h"
#include "robotics_backup/robotics/robot.h"
#include "robotics_backup/robotics/drivetrain.h"
#include "robotics_backup/robotics/ftc_fleet.h"

extern const mpe_module_desc_t mpe_module_desc; /* static copy (ftc_module.c) */

static const float DT = 1.0f / 60.0f;
static int failures = 0;
#define CHECK(cond, ...) do { \
    if (cond) { printf("[PASS] " __VA_ARGS__); printf("\n"); } \
    else { printf("[FAIL] " __VA_ARGS__); printf("\n"); failures++; } \
} while (0)

static int finite_world(physics_world *w) {
    for (int i = 0; i < w->body_count; i++) {
        rigidbody *rb = &w->bodies[i];
        if (!isfinite(rb->position.x) || !isfinite(rb->position.y) || !isfinite(rb->position.z) ||
            !isfinite(rb->velocity.x) || !isfinite(rb->velocity.y) || !isfinite(rb->velocity.z)) {
            return 0;
        }
    }
    return 1;
}

static void setup_world(physics_world *w) {
    mpe_config_init();
    g_cfg.timestep.solver_iterations = 128;
    physics_world_init(w);
    constraint_pool_init(w);
}

/* dlsym'd dynamic API surface (full import: descriptor + fleet + drive) */
typedef int (*spawn_fn_t)(struct physics_world *, float, float, float,
                          motor_preset_id, ftc_drivetrain_type);
typedef ftc_robot *(*get_fn_t)(struct physics_world *, int);
typedef void (*tank_fn_t)(ftc_robot *, float, float);

static void print_bits(const char *tag, float a, float b) {
    uint32_t ua, ub;
    memcpy(&ua, &a, 4); memcpy(&ub, &b, 4);
    printf("[info] %s: %a vs %a (0x%08x vs 0x%08x)\n", tag, a, b, ua, ub);
}

int main(int argc, char **argv) {
    const char *so = getenv("FTC_SO");
    if (!so && argc > 1) so = argv[1];
    if (!so) so = "../robotics_backup/plugins/mpe_ftc.so";

    /* ---- 1. dynamic import through the kernel loader ---- */
    char err[512] = {0};
    CHECK(mpe_loader_load(so, err, sizeof(err)) == 0, "dlopen %s", so);
    if (err[0]) printf("[info] loader note: %s\n", err);
    const mpe_module_desc_t *dyn = mpe_find_module("ftc-fleet");
    CHECK(dyn != NULL, "registry lists ftc-fleet after load");
    if (!dyn) return 1;
    CHECK(dyn->abi == MPE_MODULE_ABI, "dynamic desc ABI match");
    void *h = dlopen(so, RTLD_NOW | RTLD_NOLOAD);
    CHECK(h != NULL, "handle re-acquired for dlsym");
    spawn_fn_t dyn_spawn = h ? (spawn_fn_t)dlsym(h, "ftc_fleet_spawn") : NULL;
    get_fn_t dyn_get = h ? (get_fn_t)dlsym(h, "ftc_fleet_get") : NULL;
    tank_fn_t dyn_tank = h ? (tank_fn_t)dlsym(h, "drivetrain_tank") : NULL;
    const mpe_module_desc_t *dyn_desc = h ? (const mpe_module_desc_t *)dlsym(h, "mpe_module_desc") : NULL;
    CHECK(dyn_spawn && dyn_get && dyn_tank && dyn_desc, "fleet API fully importable via dlsym");
    if (!dyn_spawn || !dyn_get || !dyn_tank || !dyn_desc) return 1;
    CHECK(dyn_desc != &mpe_module_desc, "dynamic desc is a distinct image copy");

    /* ---- 2a. static path: attach linked-in desc, spawn, drive ---- */
    physics_world w1;
    setup_world(&w1);
    CHECK(physics_world_attach_module(&w1, &mpe_module_desc) >= 0, "static attach");
    {
        physics_world w0;
        setup_world(&w0);
        CHECK(ftc_fleet_spawn(&w0, 0, 0, 0, 0, 0) == -1, "spawn refused when nothing attached");
        CHECK(ftc_fleet_count(&w0) == 0, "empty fleet count is 0");
        CHECK(ftc_fleet_get(&w0, 0) == NULL, "empty fleet get is NULL");
        physics_world_cleanup(&w0);
    }
    int s0 = ftc_fleet_spawn(&w1, 0.0f, ftc_robot_rest_height(), 0.0f,
                             MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_MECANUM);
    CHECK(s0 == 0, "static fleet spawn index 0");
    CHECK(ftc_fleet_count(&w1) == 1, "static fleet count 1");
    ftc_robot *r1 = ftc_fleet_get(&w1, 0);
    CHECK(r1 != NULL, "static fleet get");
    CHECK(ftc_fleet_get(&w1, 7) == NULL, "oob fleet get is NULL");
    float sx, sy, sz;
    ftc_robot_get_position(&w1, r1, &sx, &sy, &sz);
    for (int t = 0; t < 180; t++) {
        drivetrain_tank(r1, 1.0f, 1.0f);
        physics_world_step(&w1, DT); /* pre_step drives the fleet; no manual update */
        if (!finite_world(&w1)) { printf("[FAIL] static path NaN at tick %d\n", t); failures++; break; }
    }
    float ex, ey, ez;
    ftc_robot_get_position(&w1, r1, &ex, &ey, &ez);
    float dz1 = ex - sx, dz1z = ez - sz;
    float disp1 = sqrtf(dz1 * dz1 + dz1z * dz1z);
    CHECK(disp1 >= 0.5f, "static module path drives (%.4f m)", disp1);

    /* ---- 2b. dynamic path: identical script through the .so ---- */
    physics_world w2;
    setup_world(&w2);
    CHECK(physics_world_attach_module(&w2, dyn_desc) >= 0, "dynamic attach");
    int d0 = dyn_spawn(&w2, 0.0f, ftc_robot_rest_height(), 0.0f,
                       MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_MECANUM);
    CHECK(d0 == 0, "dynamic fleet spawn index 0");
    ftc_robot *r2 = dyn_get(&w2, 0);
    CHECK(r2 != NULL, "dynamic fleet get");
    float tx, ty, tz;
    ftc_robot_get_position(&w2, r2, &tx, &ty, &tz);
    for (int t = 0; t < 180; t++) {
        dyn_tank(r2, 1.0f, 1.0f);
        physics_world_step(&w2, DT);
        if (!finite_world(&w2)) { printf("[FAIL] dynamic path NaN at tick %d\n", t); failures++; break; }
    }
    float fx, fy, fz;
    ftc_robot_get_position(&w2, r2, &fx, &fy, &fz);
    float dx2 = fx - tx, dz2 = fz - tz;
    float disp2 = sqrtf(dx2 * dx2 + dz2 * dz2);
    CHECK(disp2 >= 0.5f, "dynamic module path drives (%.4f m)", disp2);

    /* ---- 3. bitwise equivalence static vs dynamic ---- */
    int same = (memcmp(&ex, &fx, 4) == 0) && (memcmp(&ey, &fy, 4) == 0) &&
               (memcmp(&ez, &fz, 4) == 0) && (memcmp(&r1->odom_x, &r2->odom_x, 4) == 0) &&
               (memcmp(&r1->odom_z, &r2->odom_z, 4) == 0) &&
               (memcmp(&r1->odom_theta, &r2->odom_theta, 4) == 0);
    if (!same) {
        print_bits("pos.x", ex, fx);
        print_bits("pos.y", ey, fy);
        print_bits("pos.z", ez, fz);
        print_bits("odom_x", r1->odom_x, r2->odom_x);
    }
    CHECK(same, "static vs dynamic bitwise-identical pose+odometry");

    /* ---- 4. detach lifecycle: torques stop, re-attach works ----
     * NOTE: r2 dangles after detach (the fleet array is freed — same
     * dangling-pointer rule as bodies after physics_world_cleanup), so
     * only the body INDEX (stable: detach never touches bodies) may be
     * reused afterwards. The detached coast below issues NO commands at
     * all: with the module gone nothing can drive. */
    int w2chassis = r2->chassis_body;
    CHECK(physics_world_detach_module(&w2, "ftc-fleet") == 0, "detach dynamic");
    CHECK(ftc_fleet_count(&w1) == 1, "static world unaffected by dynamic detach");
    for (int t = 0; t < 60; t++) {
        physics_world_step(&w2, DT);
        if (!finite_world(&w2)) { printf("[FAIL] post-detach NaN at tick %d\n", t); failures++; break; }
    }
    CHECK(finite_world(&w2), "detached world stays finite");
    {
        float gx = w2.bodies[w2chassis].position.x;
        float gz = w2.bodies[w2chassis].position.z;
        float coast = sqrtf((gx - fx) * (gx - fx) + (gz - fz) * (gz - fz));
        CHECK(coast < 2.0f, "detached robot only coasts (%.4f m, no drive)", coast);
    }
    CHECK(physics_world_attach_module(&w2, &mpe_module_desc) >= 0, "re-attach static desc");
    CHECK(ftc_fleet_count(&w2) == 0, "re-attached fleet starts empty");
    int s9 = ftc_fleet_spawn(&w2, 5.0f, ftc_robot_rest_height(), 5.0f,
                             MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_TANK);
    CHECK(s9 == 0, "spawn into re-attached fleet");
    CHECK(mpe_loader_unload(so) == 0, "unload .so");

    /* Balance every attach (module state is freed by detach, not by
     * world cleanup — same split as malloc/free). */
    CHECK(physics_world_detach_module(&w1, "ftc-fleet") == 0, "detach static");
    CHECK(physics_world_detach_module(&w2, "ftc-fleet") == 0, "detach re-attached");
    physics_world_cleanup(&w1);
    physics_world_cleanup(&w2);
    if (dlclose(h) != 0) { printf("[FAIL] dlclose\n"); failures++; }
    else printf("[PASS] dlclose\n");
    printf(failures ? "FTC HOTLOAD: %d FAILURES\n" : "FTC HOTLOAD: all green\n", failures);
    return failures ? 1 : 0;
}
#endif /* MPE_FTC_HOTLOAD_TEST */

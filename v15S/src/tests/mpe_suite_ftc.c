/* MPE Suite v2 — ftc ecosystem end-to-end (terminal-identical path).
 *
 * Loads ecosystem/mfs/mfs_ecosystem.so through the kernel loader,
 * attaches the bundle, and drives everything through its command()
 * surface — the exact calls `eco attach` + `eco command` make:
 * spawn -> drive tank -> 180 ticks -> telemetry/list -> detach/unload.
 * Pose is read via dlsym'd fleet accessors + robot.h layout (header-only;
 * same-tree build, like the terminal). Requires CWD=v15S/src and a
 * built bundle (build_suite depends on it).
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <dlfcn.h>
#include "mpe_test.h"
#include "core/mpe_registry.h"
#include "core/mpe_loader.h"
#include "ecosystem/mpe_ecosystem.h"
#include "ecosystem/mfs/modules/ftc/submodules/robot.h"
#include "ecosystem/mfs/modules/ftc/ftc_fleet.h"

typedef int (*spawn_fn_t)(struct physics_world *, float, float, float, motor_preset_id,
                          ftc_drivetrain_type);
typedef ftc_robot *(*get_fn_t)(struct physics_world *, int);
typedef void (*tank_fn_t)(ftc_robot *, float, float);

int mpe_t_ftc_ecosystem(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "ftc_ecosystem");
    mpe_config_init();
    g_cfg.timestep.solver_iterations = 128;

    if (access("ecosystem/mfs/mfs_ecosystem.so", R_OK) != 0) {
        printf("[SKIP] bundle not built (run from v15S/src after make)\n");
        mpe_test_end(&t);
        return t.failures;
    }
    char err[512] = {0};
    MPE_CHECK(&t, mpe_loader_load("ecosystem/mfs/mfs_ecosystem.so", err, sizeof(err)) == 0);
    MPE_CHECK(&t, mpe_ecosystem_find("mfs-simulator") != NULL);

    physics_world w;
    mpe_world_begin(&w);
    MPE_CHECK(&t, mpe_floor_slab(&w, 1.0f, 0.8f, 0.0f) >= 0);
    MPE_CHECK(&t, mpe_ecosystem_attach(&w, "mfs-simulator") == 0);
    void *st = mpe_ecosystem_state(&w, "mfs-simulator");
    MPE_CHECK(&t, st != NULL);
    const mpe_ecosystem_desc_t *ed = mpe_ecosystem_find("mfs-simulator");
    MPE_CHECK(&t, ed != NULL && ed->command != NULL);
    if (!st || !ed || !ed->command) {
        physics_world_cleanup(&w);
        mpe_test_end(&t);
        return t.failures + 1;
    }

    char *sp[] = {"spawn"};
    MPE_CHECK(&t, ed->command(st, 1, sp) == 0);
    char *dv[] = {"drive", "0", "tank", "1.0", "1.0"};
    MPE_CHECK(&t, ed->command(st, 5, dv) == 0);

    /* Pose reader resolves exactly like the terminal does. */
    spawn_fn_t p_spawn = NULL;
    get_fn_t p_get = NULL;
    {
        void *h = dlopen("ecosystem/mfs/mfs_ecosystem.so", RTLD_NOW | RTLD_LOCAL | RTLD_NOLOAD);
        if (h) {
            p_spawn = (spawn_fn_t)dlsym(h, "ftc_fleet_spawn");
            p_get = (get_fn_t)dlsym(h, "ftc_fleet_get");
        }
    }
    MPE_CHECK(&t, p_spawn != NULL && p_get != NULL);
    float x0 = 0, z0 = 0;
    ftc_robot *r0 = p_get ? p_get(&w, 0) : NULL;
    MPE_CHECK(&t, r0 != NULL);
    if (r0 && r0->chassis_body >= 0) {
        x0 = w.bodies[r0->chassis_body].position.x;
        z0 = w.bodies[r0->chassis_body].position.z;
    }

    const float dt = 1.0f / 60.0f;
    for (int k = 0; k < 180; k++) {
        mpe_ecosystem_pre_step(&w, dt);
        physics_world_step(&w, dt);
        mpe_ecosystem_post_step(&w, dt);
        if (!mpe_world_finite(&w)) {
            printf("[FAIL] non-finite at tick %d\n", k);
            t.failures++;
            break;
        }
    }
    char *tl[] = {"telemetry", "0"};
    MPE_CHECK(&t, ed->command(st, 2, tl) == 0);
    char *li[] = {"list"};
    MPE_CHECK(&t, ed->command(st, 1, li) == 0);

    ftc_robot *r1 = p_get ? p_get(&w, 0) : NULL;
    MPE_CHECK(&t, r1 != NULL);
    if (r1 && r1->chassis_body >= 0) {
        float dx = w.bodies[r1->chassis_body].position.x - x0;
        float dz = w.bodies[r1->chassis_body].position.z - z0;
        float disp = sqrtf(dx * dx + dz * dz);
        MPE_INFO("ecosystem drive displacement=%.4f m", disp);
        MPE_CHECK(&t, disp >= 0.5f);
        MPE_CHECK(&t, fabsf(w.bodies[r1->chassis_body].position.y - 0.18f) < 1.0f);
    }
    MPE_CHECK(&t, mpe_ecosystem_detach(&w, "mfs-simulator") == 0);
    MPE_CHECK(&t, mpe_loader_unload("ecosystem/mfs/mfs_ecosystem.so") == 0);
    MPE_CHECK(&t, mpe_ecosystem_find("mfs-simulator") == NULL);
    physics_world_cleanup(&w);
    if (t.failures == 0) {
        printf("[PASS] ftc ecosystem drive green\n");
    }
    mpe_test_end(&t);
    return t.failures;
}

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
#include "core/mpe_platform.h"
#ifndef MPE_OS_WINDOWS
#include <unistd.h>
#endif
#include "core/mpe_platform.h"
#include "mpe_test.h"
#include "core/mpe_registry.h"
#include "core/mpe_loader.h"
#include "ecosystem/mpe_ecosystem.h"
#include "ecosystem/mfs/modules/ftc/submodules/robot.h"
#include "ecosystem/mfs/modules/ftc/ftc_fleet.h"
/* Windows-aware plugin path: pick existing .so/.dll variant. */
static const char *mpe_pick_plugin (const char *so_path, char *buf, size_t n) {
#ifdef MPE_OS_WINDOWS
    /* so_path like "plugins/mpe_capsule.so": try as-is, then .dll variant. */
    if (access (so_path, R_OK) == 0)
        return so_path;
    size_t L = strlen (so_path);
    if (L > 3 && strcmp (so_path + L - 3, ".so") == 0) {
        snprintf (buf, n, "%.*s.dll", (int) (L - 3), so_path);
        if (access (buf, R_OK) == 0)
            return buf;
    } else if (L > 4 && _stricmp (so_path + L - 4, ".dll") == 0) {
        snprintf (buf, n, "%.*s.so", (int) (L - 4), so_path);
        if (access (buf, R_OK) == 0)
            return buf;
    }
    /* try MPE_PLUGIN_EXT variant of basename */
    return so_path;
#else
    (void) buf;
    (void) n;
    return so_path;
#endif
}
typedef int (*spawn_fn_t) (struct physics_world *, float, float, float, motor_preset_id, ftc_drivetrain_type);
typedef ftc_robot *(*get_fn_t) (struct physics_world *, int);
typedef void (*tank_fn_t) (ftc_robot *, float, float);
int mpe_t_ftc_ecosystem (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "ftc_ecosystem");
    g_cfg.timestep.solver_iterations = 128;
    char eco_buf[1024];
    const char *eco_path = mpe_pick_plugin ("ecosystem/mfs/mfs_ecosystem.so", eco_buf, sizeof (eco_buf));
    if (access (eco_path, R_OK) != 0) {
        /* DESPOT-2026-09-29: returned 0, reporting a green case that ran
         * nothing. See mpe_suite_loader.c. */
        printf ("[SKIP] bundle not built (run from v15S/src after make)\n");
        mpe_test_end (&t);
        return MPE_SKIPPED;
    }
    char err[512] = {0};
    MPE_CHECK (&t, mpe_loader_load (eco_path, err, sizeof (err)) == 0);
    MPE_CHECK (&t, mpe_ecosystem_find ("mfs-simulator") != NULL);
    physics_world w;
    mpe_world_begin (&w);
    MPE_CHECK (&t, mpe_floor_slab (&w, 1.0f, 0.8f, 0.0f) >= 0);
    MPE_CHECK (&t, mpe_ecosystem_attach (&w, "mfs-simulator") == 0);
    void *st = mpe_ecosystem_state (&w, "mfs-simulator");
    MPE_CHECK (&t, st != NULL);
    const mpe_ecosystem_desc_t *ed = mpe_ecosystem_find ("mfs-simulator");
    MPE_CHECK (&t, ed != NULL && ed->command != NULL);
    if (!st || !ed || !ed->command) {
        physics_world_cleanup (&w);
        mpe_test_end (&t);
        return t.failures + 1;
    }
    char *sp[] = {"spawn"};
    MPE_CHECK (&t, ed->command (st, 1, sp) == 0);
    char *dv[] = {"drive", "0", "tank", "1.0", "1.0"};
    MPE_CHECK (&t, ed->command (st, 5, dv) == 0);
    /* Pose reader resolves exactly like the terminal does. */
    spawn_fn_t p_spawn = NULL;
    get_fn_t p_get = NULL;
    {
        void *h = dlopen (eco_path, RTLD_NOW | RTLD_LOCAL | RTLD_NOLOAD);
        if (h) {
            p_spawn = (spawn_fn_t) dlsym (h, "ftc_fleet_spawn");
            p_get = (get_fn_t) dlsym (h, "ftc_fleet_get");
        }
    }
    MPE_CHECK (&t, p_spawn != NULL && p_get != NULL);
    float x0 = 0, z0 = 0;
    ftc_robot *r0 = p_get ? p_get (&w, 0) : NULL;
    MPE_CHECK (&t, r0 != NULL);
    if (r0 && r0->chassis_body >= 0) {
        x0 = w.bodies[r0->chassis_body].position.x;
        z0 = w.bodies[r0->chassis_body].position.z;
    }
    const float dt = 1.0f / 60.0f;
    for (int k = 0; k < 180; k++) {
        mpe_ecosystem_pre_step (&w, dt);
        physics_world_step (&w, dt);
        mpe_ecosystem_post_step (&w, dt);
        if (!mpe_world_finite (&w)) {
            printf ("[FAIL] non-finite at tick %d\n", k);
            t.failures++;
            break;
        }
    }
    char *tl[] = {"telemetry", "0"};
    MPE_CHECK (&t, ed->command (st, 2, tl) == 0);
    char *li[] = {"list"};
    MPE_CHECK (&t, ed->command (st, 1, li) == 0);
    ftc_robot *r1 = p_get ? p_get (&w, 0) : NULL;
    MPE_CHECK (&t, r1 != NULL);
    /* DESPOT-2026-09-29: both motion assertions were wrapped in
     * `if (r1 && r1->chassis_body >= 0)`. chassis_body == -1 -- which the MFS
     * harness explicitly defends against, mfs_test.h mfs_chassis_or_null --
     * made the whole block vanish while the case still reported PASS having
     * verified nothing about motion. An unconditional index here is also an
     * out-of-bounds read, so this is the MFS fix applied to the engine side. */
    MPE_CHECK (&t, r1 != NULL);
    MPE_CHECK (&t, r1 && r1->chassis_body >= 0);
    MPE_CHECK (&t, r1 && r1->chassis_body < w.body_count);
    if (r1 && r1->chassis_body >= 0 && r1->chassis_body < w.body_count) {
        float dx = w.bodies[r1->chassis_body].position.x - x0;
        float dz = w.bodies[r1->chassis_body].position.z - z0;
        float disp = sqrtf (dx * dx + dz * dz);
        MPE_INFO ("ecosystem drive displacement=%.4f m", disp);
        MPE_CHECK (&t, disp >= 0.5f);
        /* +/- 1 m on an 18 cm rest height was a +/-1 m band; tighten to a
         * value that would actually notice a robot at the wrong height. */
        MPE_CHECK (&t, fabsf (w.bodies[r1->chassis_body].position.y - 0.18f) < 0.25f);
    }
    MPE_CHECK (&t, mpe_ecosystem_detach (&w, "mfs-simulator") == 0);
    MPE_CHECK (&t, mpe_loader_unload (eco_path) == 0);
    MPE_CHECK (&t, mpe_ecosystem_find ("mfs-simulator") == NULL);
    physics_world_cleanup (&w);
    if (t.failures == 0) {
        printf ("[PASS] ftc ecosystem drive green\n");
    }
    mpe_test_end (&t);
    return t.failures;
}

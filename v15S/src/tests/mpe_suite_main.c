/* MPE Suite v2 — registry + main + link stubs.
 *
 * Single binary `test_mpe_suite`: exact-name dispatch, no substring fan-out.
 * Usage:
 *   ./test_mpe_suite --list      list tests
 *   ./test_mpe_suite <name>      run one test (exact match)
 *   ./test_mpe_suite             run all physics tests (diag excluded)
 *   ./test_mpe_suite --all       run all tests including diag-informational
 *
 * Exit code: number of failing tests (0 = green). Summary splits physics
 * vs diag exactly like tools/test_runner.py so diag never inflates green.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "core/physics_world.h"
#include "core/rigidbody.h"
#include "config/mpe_config.h"
#include "config/mpe_constants.h"
#include "ui_input/camera.h"
#include "mpe_test.h" /* MPE_SKIPPED and the shared harness contract */

/* ---- link stubs (same contract as v1 spring/scene tests) ---- */
camera main_camera_fov;
rigidbody *obj_per_scene = NULL;
int object_count = 0;
int object_capacity = 0;

int scene_find_object_index_by_id(uint32_t id) {
    (void)id;
    return -1;
}

rigidbody *scene_resolve_object_by_id(uint32_t id) {
    (void)id;
    return NULL;
}

uint32_t scene_allocate_object_id(void) {
    physics_world *w = physics_world_get_primary();
    if (w->next_object_id == 0) {
        w->next_object_id = 1;
    }
    return w->next_object_id++;
}

void scene_note_loaded_id(uint32_t id) {
    if ((id == 0) || (id == 0xFFFFFFFFu)) {
        return;
    }
    physics_world *w = physics_world_get_primary();
    if (id >= w->next_object_id) {
        w->next_object_id = id + 1;
    }
}

int scene_ensure_pool_capacity(int n) {
    (void)n;
    return 1;
}

void scene_clear(void) {
    physics_world *w = physics_world_get_primary();
    w->body_count = 0;
}

/* ---- test declarations (suite A/B/C) ---- */
int mpe_t_two_world(void);
int mpe_t_revolute(void);
int mpe_t_revolute_matrix(void);
int mpe_t_cylinder_drop(void);
int mpe_t_driven_wheel(void);
int mpe_t_math3_inverse(void);
int mpe_t_floor_collision_diag(void);
int mpe_t_cylinder_sphere(void);
int mpe_t_cylinder_sphere_inside(void);
int mpe_t_cylinder_cube(void);
int mpe_t_cylinder_platform(void);
int mpe_t_cylinder_cylinder(void);
int mpe_t_list4_cylinder_floor(void);
int mpe_t_scene_roundtrip(void);
int mpe_t_static_hold(void);
int mpe_t_rolling_decay(void);
int mpe_t_ccd_sweep(void);
int mpe_t_kinematic(void);
int mpe_t_determinism(void);
int mpe_t_momentum(void);
int mpe_t_angmom(void);
int mpe_t_spring(void);
int mpe_t_projectile(void);
int mpe_t_incline_accel(void);
int mpe_t_pendulum(void);
int mpe_t_bounce_series(void);
int mpe_t_friction_stop(void);
int mpe_t_stack(void);
int mpe_t_f10_long_run(void);
int mpe_t_sleep_contact_wake(void);
int mpe_t_f11_torture(void);
int mpe_t_frustum(void);
int mpe_t_frustum_culler(void);
int mpe_t_module(void);
int mpe_t_loader_lifecycle(void);
int mpe_t_ftc_ecosystem(void);

typedef struct {
    const char *name;
    int (*fn)(void);
    int diag; /* 1 = diag-informational, excluded from default run */
} mpe_entry_t;

/* DESPOT-2026-09-29: metamorphic suite. These carry NO golden numbers -- they
 * assert relations (rotation equivariance, solver convergence monotonicity,
 * sleep thresholds, config reachability) that any correct engine satisfies
 * regardless of what it computes. That is the one property every defect in
 * this audit shared: the code and the expected number were wrong the same
 * way, or the measurement could not express the thing it was checking. */
extern int mpe_t_reference_math(void);
extern int mpe_t_mass_properties(void);
extern int mpe_t_meta_rotation(void);
extern int mpe_t_meta_convergence(void);
extern int mpe_t_meta_config_wiring(void);
extern int mpe_t_sleep_settle(void);
extern int mpe_t_mouse_look_axes(void);
extern int mpe_t_body_materials_live(void);

static const mpe_entry_t mpe_registry[] = {
    {"meta_rotation", mpe_t_meta_rotation, 0},
    {"meta_convergence", mpe_t_meta_convergence, 0},
    {"meta_config_wiring", mpe_t_meta_config_wiring, 0},
    {"sleep_settle", mpe_t_sleep_settle, 0},
    {"mouse_look_axes", mpe_t_mouse_look_axes, 0},
    {"body_materials_live", mpe_t_body_materials_live, 0},
    {"mass_properties", mpe_t_mass_properties, 0},
    {"reference_math", mpe_t_reference_math, 0},
    {"two_world", mpe_t_two_world, 0},
    {"revolute", mpe_t_revolute, 0},
    {"revolute_matrix", mpe_t_revolute_matrix, 0},
    {"cylinder_drop", mpe_t_cylinder_drop, 0},
    {"driven_wheel", mpe_t_driven_wheel, 0},
    /* DESPOT-2026-09-29: was diag=1 (non-blocking). This is 256 fixed-seed SPD
     * matrices across scales 2^-24..2^24 plus singular-axis and non-finite
     * contracts -- the strongest test in the repo, filed next to two smoke
     * tests. Its failures must be blocking. */
    {"math3_inverse", mpe_t_math3_inverse, 0},
    /* Functionally a duplicate of cylinder_drop; kept informational. */
    {"floor_collision_diag", mpe_t_floor_collision_diag, 1},
    {"cylinder_sphere", mpe_t_cylinder_sphere, 0},
    {"cylinder_sphere_inside", mpe_t_cylinder_sphere_inside, 0},
    {"cylinder_cube", mpe_t_cylinder_cube, 0},
    {"cylinder_platform", mpe_t_cylinder_platform, 0},
    {"cylinder_cylinder", mpe_t_cylinder_cylinder, 0},
    {"list4_cylinder_floor", mpe_t_list4_cylinder_floor, 0},
    {"scene_roundtrip", mpe_t_scene_roundtrip, 0},
    {"static_hold", mpe_t_static_hold, 0},
    {"rolling_decay", mpe_t_rolling_decay, 0},
    {"ccd_sweep", mpe_t_ccd_sweep, 0},
    {"kinematic", mpe_t_kinematic, 0},
    {"determinism", mpe_t_determinism, 0},
    {"momentum", mpe_t_momentum, 0},
    {"angmom", mpe_t_angmom, 0},
    {"spring", mpe_t_spring, 0},
    {"projectile", mpe_t_projectile, 0},
    {"incline_accel", mpe_t_incline_accel, 0},
    {"pendulum", mpe_t_pendulum, 0},
    {"bounce_series", mpe_t_bounce_series, 0},
    {"friction_stop", mpe_t_friction_stop, 0},
    {"stack", mpe_t_stack, 0},
    {"f10_long_run", mpe_t_f10_long_run, 0},
    {"sleep_contact_wake", mpe_t_sleep_contact_wake, 0},
    {"f11_torture", mpe_t_f11_torture, 0},
    /* DESPOT-2026-09-29: canonical frustum projects ONE point and never runs
     * the shipped culler in render/new_render.c (frustum_test.c re-implements
     * plane extraction locally and links no engine objects), so it proves
     * nothing about the culler. The real legacy frustum_test.c gates
     * 4 poses x 20k samples. Kept informational for the canonical case;
     * the shipped culler remains untested -- tracked, not claimed green. */
    /* DESPOT-2026-09-29: mpe_t_frustum_culler exercises the SHIPPED culler
     * (extracted to math4_special.h, called by the renderer) and is BLOCKING.
     * The old canonical case projected one point and is kept informational. */
    {"frustum_culler", mpe_t_frustum_culler, 0},
    {"frustum", mpe_t_frustum, 1},
    {"module", mpe_t_module, 0},
    {"loader_lifecycle", mpe_t_loader_lifecycle, 0},
    {"ftc_ecosystem", mpe_t_ftc_ecosystem, 0},
};

#define MPE_NTESTS ((int)(sizeof(mpe_registry) / sizeof(mpe_registry[0])))

/* Returns 0 pass, 1 fail, 2 skipped.
 * DESPOT-2026-09-29: a skipped case used to be indistinguishable from a pass
 * here, so `make test_suite` could report "29/29 green" with two cases having
 * executed nothing (wrong CWD, or a bundle not yet built). Skips are now
 * counted and printed, and excluded from the green count. */
static int mpe_run_one(const mpe_entry_t *e) {
    printf("  Running %s...\n", e->name);
    int fails = e->fn();
    if (fails == MPE_SKIPPED) {
        printf("  [SKIP] %s (coverage did not run)\n", e->name);
        return 2;
    }
    printf("  [%s] %s (checks failed: %d)\n", fails == 0 ? "PASS" : "FAIL", e->name, fails);
    return fails == 0 ? 0 : 1;
}

int main(int argc, char **argv) {
    /* DESPOT-2026-09-29 THE BIG ONE. The suite never loaded the config.
     *
     * g_cfg is a plain global in mpe_config_schema.c, so the C runtime
     * zero-initialises it. mpe_test_begin() only *saves* g_cfg, and nothing
     * in this binary's startup path called mpe_config_init() — so every test
     * that relied on the shared harness ran against an all-zero config:
     *
     *   solver_iterations    0   (real 64)  -> the sequential-impulse solver
     *                                            ran ZERO iterations
     *   solver.bias_factor   0   (real 0.1) -> no Baumgarte positional bias,
     *                                            i.e. no penetration recovery
     *   solver.penetration_slop 0 (real 0.01)
     *   body_defaults restitution/friction all 0 (real 0.5 / 0.3)
     *   world.gravity        0   (real -9.81)
     *   timestep.max_substeps 0  (real 5)
     *
     * The shipped game is fine: root_gtk.c calls mpe_config_init() twice, as
     * do headless_main.c and tui/tui_main.c. The MFS harness calls it in
     * mfs_test_world(). This suite was the one path that did not — so the
     * engine's own comment in physics_world.c about "a world initialised
     * before mpe_config_init() runs sees a zeroed g_cfg" was describing this
     * suite, permanently, and nobody connected the two.
     *
     * Loaded here, once, before any test runs. Individual test files that
     * call mpe_config_init() themselves are unaffected (idempotent). */
    mpe_config_init();
    /* DESPOT-2026-09-29: print the regime up front so a CI log line records
     * which configuration produced the result. The suite is now expected to be
     * run once per regime (see tools/test_runner.py); a green result without
     * a regime tag is no longer a complete measurement. */
    {
        const char *regime = getenv("MPE_TEST_REGIME");
        printf("regime: %s\n", (regime && *regime) ? regime : "default");
    }
    printf("MPE Suite v2 — src: v15S (single binary, exact dispatch)\n");
    printf("============================================================\n");
    if (argc >= 2 && strcmp(argv[1], "--list") == 0) {
        printf("Available tests:\n");
        for (int i = 0; i < MPE_NTESTS; i++) {
            printf("  %s%s\n", mpe_registry[i].name, mpe_registry[i].diag ? " (diag)" : "");
        }
        return 0;
    }
    if (argc >= 2 && strcmp(argv[1], "--help") == 0) {
        printf("usage: test_mpe_suite [--list] [--all] [<exact-name>]\n");
        return 0;
    }
    int include_diag = (argc >= 2 && strcmp(argv[1], "--all") == 0);
    if (argc >= 3 && argv[1][0] != '-' && argv[2][0] != '-') {
        /* Multi-name sequence in one process (bisection/debugging). */
        int failed = 0;
        for (int a = 1; a < argc; a++) {
            int found = 0;
            for (int i = 0; i < MPE_NTESTS; i++) {
                if (strcmp(mpe_registry[i].name, argv[a]) == 0) {
                    failed += mpe_run_one(&mpe_registry[i]);
                    found = 1;
                    break;
                }
            }
            if (!found) {
                printf("No tests matching '%s'\n", argv[a]);
                failed++;
            }
        }
        return failed ? 1 : 0;
    }
    const char *only = NULL;
    if (argc >= 2 && !include_diag && argv[1][0] != '-') {
        only = argv[1];
    }
    if (only) {
        for (int i = 0; i < MPE_NTESTS; i++) {
            if (strcmp(mpe_registry[i].name, only) == 0) {
                return mpe_run_one(&mpe_registry[i]);
            }
        }
        printf("No tests matching '%s'\n", only);
        return 1;
    }
    int phys_pass = 0, phys_total = 0, diag_pass = 0, diag_total = 0, failed = 0;
    int skipped = 0;
    for (int i = 0; i < MPE_NTESTS; i++) {
        if (mpe_registry[i].diag && !include_diag) {
            continue;
        }
        int rc = mpe_run_one(&mpe_registry[i]);
        if (rc == 2) {
            /* Not a pass. Counted separately and reported, never green. */
            skipped++;
            continue;
        }
        if (mpe_registry[i].diag) {
            diag_total++;
            diag_pass += (rc == 0);
        } else {
            phys_total++;
            phys_pass += (rc == 0);
        }
        failed += rc;
    }
    printf("\n============================================================\n");
    printf("TEST SUMMARY\n");
    printf("============================================================\n");
    printf("Physics: %d/%d green | Diag/math: %d/%d (informational)\n", phys_pass, phys_total,
           diag_pass, diag_total);
    if (skipped) {
        printf("SKIPPED (coverage did not run, NOT counted green): %d\n", skipped);
    }
    printf("Total: %d | Blocking failures: %d\n", phys_total + diag_total, failed);
    return failed ? 1 : 0;
}

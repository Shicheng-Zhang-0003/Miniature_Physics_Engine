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
int mpe_t_cylinder_drop(void);
int mpe_t_driven_wheel(void);
int mpe_t_math3_inverse(void);
int mpe_t_floor_collision_diag(void);
int mpe_t_cylinder_sphere(void);
int mpe_t_cylinder_cube(void);
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
int mpe_t_module(void);
int mpe_t_loader_lifecycle(void);

typedef struct {
    const char *name;
    int (*fn)(void);
    int diag; /* 1 = diag-informational, excluded from default run */
} mpe_entry_t;

static const mpe_entry_t mpe_registry[] = {
    {"two_world", mpe_t_two_world, 0},
    {"revolute", mpe_t_revolute, 0},
    {"cylinder_drop", mpe_t_cylinder_drop, 0},
    {"driven_wheel", mpe_t_driven_wheel, 0},
    {"math3_inverse", mpe_t_math3_inverse, 1},
    {"floor_collision_diag", mpe_t_floor_collision_diag, 1},
    {"cylinder_sphere", mpe_t_cylinder_sphere, 0},
    {"cylinder_cube", mpe_t_cylinder_cube, 0},
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
    {"frustum", mpe_t_frustum, 1},
    {"module", mpe_t_module, 0},
    {"loader_lifecycle", mpe_t_loader_lifecycle, 0},
};

#define MPE_NTESTS ((int)(sizeof(mpe_registry) / sizeof(mpe_registry[0])))

static int mpe_run_one(const mpe_entry_t *e) {
    printf("  Running %s...\n", e->name);
    int fails = e->fn();
    printf("  [%s] %s (checks failed: %d)\n", fails == 0 ? "PASS" : "FAIL", e->name, fails);
    return fails == 0 ? 0 : 1;
}

int main(int argc, char **argv) {
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
    for (int i = 0; i < MPE_NTESTS; i++) {
        if (mpe_registry[i].diag && !include_diag) {
            continue;
        }
        int rc = mpe_run_one(&mpe_registry[i]);
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
    printf("Total: %d | Blocking failures: %d\n", phys_total + diag_total, failed);
    return failed ? 1 : 0;
}

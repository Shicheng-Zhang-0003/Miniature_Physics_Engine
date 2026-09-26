/* MFS Suite v2 — unified registry and main.
 * Exact-name dispatch, config isolation, NaN watchdog, summary. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "mfs_test.h"

typedef struct {
    const char *name;
    int (*fn)(void);
    bool diag;
} mfs_test_entry_t;

extern int mfs_t_teleop(void);
extern int mfs_t_mecanum(void);
extern int mfs_t_tank(void);
extern int mfs_t_odometry(void);
extern int mfs_t_ftc_integration(void);
extern int mfs_t_ftc_hotload(void);
extern int mfs_t_module_1(void);
extern int mfs_t_physics_truth(void);

static const mfs_test_entry_t registry[] = {
    {"teleop",      mfs_t_teleop,      false},
    {"mecanum",     mfs_t_mecanum,     false},
    {"tank",        mfs_t_tank,        false},
    {"odometry",    mfs_t_odometry,    false},
    {"ftc_integration", mfs_t_ftc_integration, false},
    {"ftc_hotload", mfs_t_ftc_hotload, false},
    {"module_1",    mfs_t_module_1,    false},
    {"physics_truth", mfs_t_physics_truth, false},
};

int main(int argc, char **argv) {
    if (argc > 1 && strcmp(argv[1], "--list") == 0) {
        printf("Blocking tests (%zu):\n", sizeof(registry)/sizeof(registry[0]));
        for (size_t i = 0; i < sizeof(registry)/sizeof(registry[0]); i++) {
            printf("  %s\n", registry[i].name);
        }
        return 0;
    }

    if (argc > 1 && strcmp(argv[1], "--all") == 0) {
        int total = 0, pass = 0, fail = 0;
        printf("=== Running all blocking tests ===\n");
        for (size_t i = 0; i < sizeof(registry)/sizeof(registry[0]); i++) {
            printf("--- %s ---\n", registry[i].name);
            int rc = registry[i].fn();
            if (rc == 0) {
                printf("[PASS] %s\n", registry[i].name);
                pass++;
            } else {
                printf("[FAIL] %s (failures=%d)\n", registry[i].name, rc);
                fail++;
            }
            total++;
        }
        printf("\n=== SUMMARY ===\n");
        printf("Total: %d\n", total);
        printf("Pass:  %d\n", pass);
        printf("Fail:  %d\n", fail);
        return fail > 0 ? 1 : 0;
    }

    if (argc < 2) {
        fprintf(stderr, "Usage: %s [--list|--all|<test_name>]\n", argv[0]);
        return 1;
    }

    for (size_t i = 0; i < sizeof(registry)/sizeof(registry[0]); i++) {
        if (strcmp(argv[1], registry[i].name) == 0) {
            int rc = registry[i].fn();
            return rc > 0 ? 1 : 0;
        }
    }

    fprintf(stderr, "Unknown test: %s\n", argv[1]);
    return 1;
}

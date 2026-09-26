/* FTC robot fleet implementation. */
#include "ftc_fleet.h"
#include "submodules/drivetrain.h"
#include <stdlib.h>
#include <string.h>

#define FTC_FLEET_INIT_CAP 4
/* FIX-AUDIT-DESPOT: hard cap 32 is an array bound, NOT a performance claim.
 * Practical limit is 2-4 robots: each mecanum robot adds 4 wheels + up to
 * 32 roller bodies + 36 joints to one Gauss-Seidel island, and solver cost
 * grows superlinearly with constraint count at 128 iterations. Past ~4 the
 * tick slows and contacts go unconverged; raise only with profiling. */
#define FTC_FLEET_MAX 32

/* Module registry name shared with ftc_module.c's descriptor. */
#define FTC_FLEET_MODULE_NAME "ftc-fleet"

typedef struct {
    ftc_robot *robots;
    int count;
    int cap;
} ftc_fleet_t;

void *ftc_fleet_create(void) {
    ftc_fleet_t *f = (ftc_fleet_t *)calloc(1, sizeof(ftc_fleet_t));
    if (!f) return NULL;
    f->robots = (ftc_robot *)calloc((size_t)FTC_FLEET_INIT_CAP, sizeof(ftc_robot));
    if (!f->robots) {
        free(f);
        return NULL;
    }
    f->cap = FTC_FLEET_INIT_CAP;
    f->count = 0;
    return f;
}

void ftc_fleet_destroy(void *fleet_state) {
    ftc_fleet_t *f = (ftc_fleet_t *)fleet_state;
    if (!f) return;
    free(f->robots);
    free(f);
}

/* Weak link into the bundle's internal registry (defined by
 * mfs_internal.c, ABSENT from standalone mpe_ftc.so builds). Lets fleet
 * lookup work through bundle attachments without linking bundle code:
 * when weak-unresolved the pointer is NULL and only the tick-table path
 * applies. */
__attribute__((weak)) void *mfs_internal_module_state_for(const void *world, const char *name);

/* Locate the fleet attached to a world by module name (no side table:
 * the state pointer lives in the world's own tick tables, so worlds
 * never share fleet state). Falls back to the bundle-internal attachment
 * when the world uses the mfs-simulator ecosystem instead of a direct
 * ftc-fleet tick attach. */
static ftc_fleet_t *fleet_of(struct physics_world *world) {
    if (!world) return NULL;
    for (int i = 0; i < world->tick_module_count; i++) {
        if (world->tick_modules[i] && world->tick_modules[i]->name &&
            strcmp(world->tick_modules[i]->name, FTC_FLEET_MODULE_NAME) == 0) {
            return (ftc_fleet_t *)world->tick_module_state[i];
        }
    }
    if (mfs_internal_module_state_for) {
        return (ftc_fleet_t *)mfs_internal_module_state_for((const void *)world,
                                                            FTC_FLEET_MODULE_NAME);
    }
    return NULL;
}

int ftc_fleet_spawn(struct physics_world *world, float x, float y, float z,
                    motor_preset_id preset, ftc_drivetrain_type drivetrain_type) {
    ftc_fleet_t *f = fleet_of(world);
    if (!f || !f->robots) return -1;
    if (f->count >= FTC_FLEET_MAX) return -1;
    if (f->count >= f->cap) {
        int ncap = f->cap * 2;
        /* DESPOT-FIX: old growth could realloc past FTC_FLEET_MAX (cap 32 ->
         * ncap 64) then keep spawning to 32 while holding 64 slots. Clamp so
         * capacity never exceeds the bound. */
        if (ncap > FTC_FLEET_MAX) ncap = FTC_FLEET_MAX;
        if (ncap <= f->cap) return -1;
        ftc_robot *nr = (ftc_robot *)realloc(f->robots, (size_t)ncap * sizeof(ftc_robot));
        if (!nr) return -1;
        f->robots = nr;
        f->cap = ncap;
    }
    if (ftc_robot_create_with_drive(world, &f->robots[f->count], x, y, z,
                                    preset, drivetrain_type) != 0) {
        return -1;
    }
    return f->count++;
}

int ftc_fleet_count(struct physics_world *world) {
    ftc_fleet_t *f = fleet_of(world);
    return f ? f->count : 0;
}

ftc_robot *ftc_fleet_get(struct physics_world *world, int index) {
    ftc_fleet_t *f = fleet_of(world);
    if (!f || !f->robots || index < 0 || index >= f->count) return NULL;
    return &f->robots[index];
}

void ftc_fleet_step_all(struct physics_world *world, void *fleet_state, float dt) {
    ftc_fleet_t *f = (ftc_fleet_t *)fleet_state;
    if (!world || !f || !f->robots || !(dt > 0.0f)) return;
    /* Spawn order = update order: deterministic across runs and
     * across static/dynamic copies of this code. */
    for (int i = 0; i < f->count; i++) {
        drivetrain_update(world, &f->robots[i], dt);
    }
}

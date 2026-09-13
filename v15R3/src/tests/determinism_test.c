/* MPE determinism truth test: two identically built worlds stepped through
 * the identical call sequence must agree BITWISE (exact == on every dynamic
 * field). Same binary + same platform: guaranteed by construction (fixed
 * iteration order, exact IEEE ops, deterministic transcendentals, fused
 * contraction disabled). Cross-platform: guaranteed while IEEE-754 holds. */
#ifdef MPE_DETERMINISM_TEST
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

static void build_scene(physics_world *world) {
    physics_world_init(world);
    constraint_pool_init(world);
    int a = physics_world_add_sphere(world, 0.5f, 2.0f, (vector3){-1.0f, 3.0f, 0.5f});
    world->bodies[a].velocity = (vector3){1.5f, -0.5f, 0.25f};
    world->bodies[a].angular_velocity = (vector3){3.0f, -1.0f, 2.0f};
    world->bodies[a].restitution = 0.4f;
    int b = physics_world_add_cube(world, (vector3){1.0f, 0.5f, -0.5f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    world->bodies[b].velocity = (vector3){-0.75f, 0.0f, 0.5f};
    world->bodies[b].angular_velocity = (vector3){0.0f, 2.0f, -1.5f};
    world->bodies[b].restitution = 0.3f;
    world->bodies[b].nice_value = 3;
    int c = physics_world_add_cylinder(world, 0.3f, 0.4f, 1.5f, (vector3){0.0f, 2.0f, 1.0f});
    world->bodies[c].velocity = (vector3){0.2f, -1.0f, -0.3f};
    world->bodies[c].angular_velocity = (vector3){-2.0f, 0.5f, 1.0f};
    world->bodies[c].restitution = 0.2f;
    int d = physics_world_add_cube(world, (vector3){0.0f, 1.6f, 0.0f}, (vector3){0.4f, 0.4f, 0.4f}, 1.0f);
    world->bodies[d].velocity = (vector3){0.0f, -0.2f, 0.0f};
    /* Static floor slab for contact coverage. */
    physics_world_add_cube(world, (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
}

static int bodies_equal(const rigidbody *a, const rigidbody *b) {
    const float *fa = &a->position.x;
    const float *fb = &b->position.x;
    /* Compare the 3+3+4+3+3 kinematic vectors + accumulators exactly. */
    for (int i = 0; i < 3 + 3 + 4 + 3 + 3 + 3 + 3; i++) {
        if (fa[i] != fb[i]) {
            return 0;
        }
        if (!isfinite(fa[i])) {
            return 0;
        }
    }
    return (a->is_sleeping == b->is_sleeping) && (a->sleep_timer == b->sleep_timer) &&
           (a->static_state == b->static_state) && (a->kinematic == b->kinematic);
}

int main(void) {
    mpe_config_init();
    physics_world w1, w2;
    build_scene(&w1);
    build_scene(&w2);
    const float dt = 1.0f / 60.0f;
    for (int t = 0; t < 600; t++) {
        physics_world_step(&w1, dt);
        physics_world_step(&w2, dt);
    }
    int fail = 0;
    if (w1.body_count != w2.body_count) {
        printf("[FAIL] body count diverged\n");
        return 1;
    }
    for (int i = 0; i < w1.body_count; i++) {
        if (!bodies_equal(&w1.bodies[i], &w2.bodies[i])) {
            printf("[FAIL] body %d diverged bitwise\n", i);
            fail = 1;
        }
    }
    if (w1.world_contact_cache_count != w2.world_contact_cache_count) {
        printf("[FAIL] warm cache diverged\n");
        fail = 1;
    }
    if (fail == 0) {
        printf("[PASS] determinism: 600 ticks bitwise identical across twin worlds\n");
    }
    physics_world_cleanup(&w1);
    physics_world_cleanup(&w2);
    return fail;
}
#endif /* MPE_DETERMINISM_TEST */

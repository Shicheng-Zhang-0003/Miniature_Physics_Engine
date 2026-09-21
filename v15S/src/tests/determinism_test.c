/* MPE determinism truth test: two identically built worlds stepped through
 * the identical call sequence must agree BITWISE (exact == on every dynamic
 * field). Same binary + same platform: guaranteed by construction (fixed
 * iteration order, exact IEEE ops, deterministic transcendentals, fused
 * contraction disabled). Cross-platform: guaranteed while IEEE-754 holds. */
#ifdef mpe_determinism_test
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
    /* TRUTH: nice=0 (no numerical damping). Old nice=3 baked game damping
     * into the determinism proof; determinism must hold for pure physics. */
    world->bodies[b].nice_value = 0;
    int c = physics_world_add_cylinder(world, 0.3f, 0.4f, 1.5f, (vector3){0.0f, 2.0f, 1.0f});
    world->bodies[c].velocity = (vector3){0.2f, -1.0f, -0.3f};
    world->bodies[c].angular_velocity = (vector3){-2.0f, 0.5f, 1.0f};
    world->bodies[c].restitution = 0.2f;
    int d = physics_world_add_cube(world, (vector3){0.0f, 1.6f, 0.0f}, (vector3){0.4f, 0.4f, 0.4f}, 1.0f);
    world->bodies[d].velocity = (vector3){0.0f, -0.2f, 0.0f};
    /* Static floor slab for contact coverage. */
    physics_world_add_cube(world, (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
}

static int vec3_eq(vector3 a, vector3 b) {
    return (a.x == b.x) && (a.y == b.y) && (a.z == b.z) && isfinite(a.x) && isfinite(a.y) && isfinite(a.z) &&
           isfinite(b.x) && isfinite(b.y) && isfinite(b.z);
}

static int vec4_eq(vector4 a, vector4 b) {
    return (a.w == b.w) && (a.x == b.x) && (a.y == b.y) && (a.z == b.z) && isfinite(a.w) && isfinite(a.x) &&
           isfinite(a.y) && isfinite(a.z) && isfinite(b.w) && isfinite(b.x) && isfinite(b.y) && isfinite(b.z);
}

static int bodies_equal(const rigidbody *a, const rigidbody *b) {
    /* TRUTH: compare NAMED fields explicitly. The old float-window walk
     * (&position.x, 22 floats) assumed struct layout (pos/vel/acc/orient/
     * angvel/angacc/force) with no padding, silently skipping friction,
     * inertia, accumulators, axes and IDs — and invoking UB across
     * bool/int/enum padding. Named compare covers the full dynamic state
     * with no layout assumptions (memcmp would trip on padding garbage). */
    if (!vec3_eq(a->position, b->position)) return 0;
    if (!vec3_eq(a->velocity, b->velocity)) return 0;
    if (!vec3_eq(a->acceleration, b->acceleration)) return 0;
    if (!vec4_eq(a->orientation, b->orientation)) return 0;
    if (!vec3_eq(a->angular_velocity, b->angular_velocity)) return 0;
    if (!vec3_eq(a->angular_acceleration, b->angular_acceleration)) return 0;
    if (!vec3_eq(a->force_accumulator, b->force_accumulator)) return 0;
    if (!vec3_eq(a->torque_accumulator, b->torque_accumulator)) return 0;
    if (a->mass != b->mass || a->inverse_mass != b->inverse_mass) return 0;
    if (a->friction_static != b->friction_static || a->friction_kinetic != b->friction_kinetic) return 0;
    if (a->restitution != b->restitution) return 0;
    if (a->radius != b->radius || a->cylinder_half_length != b->cylinder_half_length) return 0;
    if (!vec3_eq(a->half_extensions, b->half_extensions)) return 0;
    if (!vec3_eq(a->cached_axes[0], b->cached_axes[0])) return 0;
    if (!vec3_eq(a->cached_axes[1], b->cached_axes[1])) return 0;
    if (!vec3_eq(a->cached_axes[2], b->cached_axes[2])) return 0;
    if (a->object_id != b->object_id || a->object_generation != b->object_generation) return 0;
    if (a->type != b->type || a->custom_shape != b->custom_shape) return 0;
    return (a->is_sleeping == b->is_sleeping) && (a->sleep_timer == b->sleep_timer) &&
           (a->static_state == b->static_state) && (a->kinematic == b->kinematic);
}

static int caches_equal(const physics_world *a, const physics_world *b) {
    /* TRUTH: count-only compare passed with divergent contents. Field-wise
     * (no memcmp: padding garbage differs legitimately). */
    if (a->world_contact_cache_count != b->world_contact_cache_count) return 0;
    for (int i = 0; i < a->world_contact_cache_count; i++) {
        const cached_contact *ca = &a->world_contact_cache[i];
        const cached_contact *cb = &b->world_contact_cache[i];
        if (ca->object_id_a != cb->object_id_a || ca->object_id_b != cb->object_id_b) return 0;
        if (ca->property_stamp_a != cb->property_stamp_a || ca->property_stamp_b != cb->property_stamp_b) return 0;
        if (!vec3_eq(ca->local_position_a, cb->local_position_a)) return 0;
        if (!vec3_eq(ca->local_position_b, cb->local_position_b)) return 0;
        if (ca->accumulated_normal_impulse != cb->accumulated_normal_impulse) return 0;
        if (ca->accumulated_tangent_impulse != cb->accumulated_tangent_impulse) return 0;
        if (!vec3_eq(ca->tangent_dir, cb->tangent_dir)) return 0;
        if (ca->hash_next != cb->hash_next) return 0;
    }
    return 1;
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
    } else if (!caches_equal(&w1, &w2)) {
        printf("[FAIL] warm cache contents diverged\n");
        fail = 1;
    }
    if (fail == 0) {
        printf("[PASS] determinism: 600 ticks bitwise identical across twin worlds\n");
    }
    physics_world_cleanup(&w1);
    physics_world_cleanup(&w2);
    return fail;
}
#endif /* mpe_determinism_test */

/* MPE scene save/load round-trip test.
 * Guards the v200 format on the primary world: LE fields, stable IDs
 * (preserved, not remapped), nice_value, sleep state, springs, revolute
 * joints with motor params, and CRC32 tamper rejection across save ->
 * clear -> load. Spring pool ops are stubbed (spring_joint.c is GL-tainted
 * and not linked headless); they operate on the primary world's pool. */
#ifdef MPE_SCENE_ROUNDTRIP_TEST
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "core/rigidbody.h"
#include "core/physics_world.h"
#include "config/mpe_config.h"
#include "config/mpe_constants.h"
#include "scene/scene_load.h"
#include "scene/scene_saving.h"
#include "scene/scene_id_remap.h"
#include "physics/spring_joint.h"
#include "physics/constraint.h"

/* --- Stubs for TU-bound symbols (operate on the primary world) --- */

uint32_t scene_allocate_object_id(void) {
    physics_world *world = physics_world_get_primary();
    if (world->next_object_id == 0) {
        world->next_object_id = 1;
    }
    return world->next_object_id++;
}

void scene_note_loaded_id(uint32_t object_id) {
    if ((object_id == 0) || (object_id == 0xFFFFFFFFu)) {
        return;
    }
    physics_world *world = physics_world_get_primary();
    if (object_id >= world->next_object_id) {
        world->next_object_id = object_id + 1;
    }
}

int scene_ensure_pool_capacity(int required_capacity) {
    (void) required_capacity;
    return 1;
}

void scene_clear(void) {
    physics_world *world = physics_world_get_primary();
    world->body_count = 0;
    joint_init_pool(world);
}

void joint_init_pool(physics_world *world) {
    if (!world) {
        return;
    }
    for (int i = 0; i < mpe_max_joints; i++) {
        world->spring_joints[i].is_active = false;
    }
    world->spring_joint_count = 0;
}

int add_joint_by_ids(physics_world *world, uint32_t id_a, uint32_t id_b, float eq, float k, float c) {
    if (!world) {
        return -1;
    }
    if ((id_a == 0) || (id_b == 0) || (id_a == id_b)) {
        return -1;
    }
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!world->spring_joints[i].is_active) {
            world->spring_joints[i].object_id_a = id_a;
            world->spring_joints[i].object_id_b = id_b;
            world->spring_joints[i].equilibrium_length = eq;
            world->spring_joints[i].spring_constant = k;
            world->spring_joints[i].damping_coefficient = c;
            world->spring_joints[i].is_active = true;
            world->spring_joint_count++;
            return i;
        }
    }
    return -1;
}

static int failures = 0;
#define CHECK(cond, label) do { \
    if (cond) { printf("  [PASS] %s\n", label); } \
    else { printf("  [FAIL] %s\n", label); failures++; } \
} while (0)
#define CHECKF(a, b, eps, label) CHECK(fabsf((a) - (b)) <= (eps), label)

int main(void) {
    mpe_config_init();
    physics_world *world = physics_world_get_primary();
    physics_world_init(world);
    scene_id_remap_reset();

    /* Body 0: dynamic sphere with distinctive state. */
    rigidbody_initialisation_sphere(&world->bodies[0], 0.5f, 2.0f,
        (vector3){1.0f, 2.0f, 3.0f});
    world->bodies[0].velocity = (vector3){0.25f, -0.5f, 1.0f};
    world->bodies[0].colour = (vector3){0.1f, 0.2f, 0.3f};
    world->bodies[0].restitution = 0.4f;
    world->bodies[0].nice_value = 7;
    world->bodies[0].object_id = scene_allocate_object_id();
    /* Body 1: sleeping cube. */
    rigidbody_initialisation_cube(&world->bodies[1],
        (vector3){-1.0f, 0.5f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    world->bodies[1].is_sleeping = true;
    world->bodies[1].sleep_timer = 0.0f;
    world->bodies[1].object_id = scene_allocate_object_id();
    world->body_count = 2;
    joint_init_pool(world);
    constraint_pool_init(world);
    CHECK(add_joint_by_ids(world, 1, 2, 1.5f, 20.0f, 1.0f) >= 0, "joint created pre-save");
    int rev_idx = constraint_add_revolute(world, 1, 2, (vector3){0.0f, -0.3f, 0.0f}, (vector3){0.0f, 1.0f, 0.0f},
                                          (vector3){0.0f, 0.0f, 1.0f});
    CHECK(rev_idx >= 0, "revolute created pre-save");
    constraint_set_revolute_motor(world, rev_idx, true, 2.5f, 10.0f);

    const char *path = "/tmp/mpe_scene_roundtrip.dat";
    CHECK(save_scene(path) == 1, "save_scene succeeds");
    CHECK(world->spring_joint_count == 1, "one joint saved");

    scene_clear();
    world->next_object_id = 100; /* v200 preserves IDs: allocator drift must NOT leak into loaded IDs */
    CHECK(scene_loading(path) == 1, "scene_loading succeeds");
    CHECK(world->body_count == 2, "two bodies loaded");
    CHECK(world->bodies[0].object_id == 1, "v200 stable ID preserved (body 0)");
    CHECK(world->bodies[1].object_id == 2, "v200 stable ID preserved (body 1)");
    CHECK(scene_allocate_object_id() >= 3, "allocator advanced past loaded IDs");

    rigidbody *s = &world->bodies[0];
    CHECK(s->type == object_sphere, "body 0 type round-trips");
    CHECKF(s->mass, 2.0f, 1e-5f, "body 0 mass round-trips");
    CHECKF(s->radius, 0.5f, 1e-5f, "body 0 radius round-trips");
    CHECKF(s->position.x, 1.0f, 1e-5f, "body 0 position round-trips");
    CHECKF(s->velocity.y, -0.5f, 1e-5f, "body 0 velocity round-trips");
    CHECKF(s->colour.z, 0.3f, 1e-5f, "body 0 colour round-trips");
    CHECKF(s->restitution, 0.4f, 1e-5f, "body 0 restitution round-trips");
    CHECK(s->nice_value == 7, "body 0 nice_value persists");
    CHECK(!s->is_sleeping, "body 0 awake");

    rigidbody *c = &world->bodies[1];
    CHECK(c->type == object_cube, "body 1 type round-trips");
    CHECKF(c->mass, 1.0f, 1e-5f, "body 1 mass round-trips");
    CHECK(c->is_sleeping, "body 1 sleep state persists");

    CHECK(world->spring_joint_count == 1, "one joint restored");
    if (world->spring_joint_count == 1) {
        uint32_t ja = world->spring_joints[0].object_id_a;
        uint32_t jb = world->spring_joints[0].object_id_b;
        uint32_t i0 = world->bodies[0].object_id;
        uint32_t i1 = world->bodies[1].object_id;
        CHECK(((ja == i0) && (jb == i1)) || ((ja == i1) && (jb == i0)),
              "joint endpoints remapped to loaded bodies");
        CHECKF(world->spring_joints[0].spring_constant, 20.0f, 1e-5f, "joint params round-trip");
    }

    CHECK(constraint_get_count(world) == 1, "one revolute restored");
    if (constraint_get_count(world) == 1) {
        const constraint *rc = constraint_pool_at(world, 0);
        CHECK((rc != NULL) && (rc->type == CONSTRAINT_REVOLUTE), "restored joint is revolute");
        if (rc) {
            CHECK(((rc->body_id_a == 1) && (rc->body_id_b == 2)) ||
                  ((rc->body_id_a == 2) && (rc->body_id_b == 1)),
                  "revolute endpoints reference preserved IDs");
            CHECK(rc->p.revolute.motor_enabled, "revolute motor flag round-trips");
            CHECKF(rc->p.revolute.motor_target_speed, 2.5f, 1e-5f, "revolute motor target round-trips");
            CHECKF(rc->p.revolute.motor_max_torque, 10.0f, 1e-5f, "revolute motor torque round-trips");
        }
    }

    /* CRC32 tamper rejection: corrupt one payload byte (mass field), keep
     * the footer. Load must fail AND leave the live scene untouched. */
    {
        FILE *rf = fopen(path, "rb");
        fseek(rf, 0, SEEK_END);
        long fsize = ftell(rf);
        fseek(rf, 0, SEEK_SET);
        unsigned char *bytes = (unsigned char *) malloc((size_t) fsize);
        size_t got = fread(bytes, 1, (size_t) fsize, rf);
        fclose(rf);
        CHECK(got == (size_t) fsize, "scene file readable for tamper test");
        const char *tamper_path = "/tmp/mpe_scene_tampered.dat";
        bytes[20] ^= 0xFFu; /* inside body 0's mass: data corrupt, footer intact */
        FILE *wf = fopen(tamper_path, "wb");
        fwrite(bytes, 1, (size_t) fsize, wf);
        fclose(wf);
        free(bytes);
        int count_before = world->body_count;
        CHECK(scene_loading(tamper_path) == 0, "tampered scene rejected");
        CHECK(world->body_count == count_before, "live scene untouched by failed load");
        CHECK(world->bodies[0].object_id == 1, "live IDs untouched by failed load");
        remove(tamper_path);
    }

    remove(path);
    physics_world_cleanup(world);
    if (failures == 0) printf("[PASS] scene round-trip complete\n");
    else printf("[FAIL] scene round-trip: %d checks failed\n", failures);
    return failures ? 1 : 0;
}
#endif /* MPE_SCENE_ROUNDTRIP_TEST */

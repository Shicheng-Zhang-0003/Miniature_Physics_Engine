/* MPE scene save/load round-trip test.
 * Guards the v200 format: LE fields, stable IDs (preserved, not remapped),
 * nice_value, sleep state, springs, revolute joints with motor params, and
 * CRC32 tamper rejection across save -> clear -> load. */
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

/* Scene globals normally owned by simulation.c; the test owns them. */
rigidbody *obj_per_scene = NULL;
int object_count = 0;
int object_capacity = 0;

/* Joint pool normally owned by spring_joint.c (GL-tainted TU, not linked
 * headless); the test provides the storage and pool ops. */
spring_joint joint_pool[mpe_max_joints];
int current_joint_count = 0;

static uint32_t test_next_id = 1;

uint32_t scene_allocate_object_id(void) { return test_next_id++; }

/* v200 stable IDs: the loader preserves saved IDs and advances the
 * allocator past them (real implementation lives in scene_init.c). */
void scene_note_loaded_id(uint32_t object_id) {
    if ((object_id != 0) && (object_id != 0xFFFFFFFFu) && (object_id >= test_next_id)) {
        test_next_id = object_id + 1;
    }
}

int scene_ensure_pool_capacity(int required_capacity) {
    if (required_capacity <= 0) return 1;
    if (obj_per_scene && (required_capacity <= object_capacity)) return 1;
    rigidbody * grown = (rigidbody *)realloc(obj_per_scene,
        (size_t)mpe_max_bodies * sizeof(rigidbody));
    if (!grown) return 0;
    obj_per_scene = grown;
    object_capacity = mpe_max_bodies;
    return 1;
}

void scene_clear(void) {
    object_count = 0;
    joint_init_pool();
}

void joint_init_pool(void) {
    for (int i = 0; i < mpe_max_joints; i++) joint_pool[i].is_active = false;
    current_joint_count = 0;
}

int add_joint_by_ids(uint32_t id_a, uint32_t id_b, float eq, float k, float c) {
    if ((id_a == 0) || (id_b == 0) || (id_a == id_b)) return -1;
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!joint_pool[i].is_active) {
            joint_pool[i].object_id_a = id_a;
            joint_pool[i].object_id_b = id_b;
            joint_pool[i].equilibrium_length = eq;
            joint_pool[i].spring_constant = k;
            joint_pool[i].damping_coefficient = c;
            joint_pool[i].is_active = true;
            current_joint_count++;
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
    scene_id_remap_reset();
    if (!scene_ensure_pool_capacity(4)) { printf("[FAIL] pool alloc\n"); return 1; }

    /* Body 0: dynamic sphere with distinctive state. */
    rigidbody_initialisation_sphere(&obj_per_scene[0], 0.5f, 2.0f,
        (vector3){1.0f, 2.0f, 3.0f});
    obj_per_scene[0].velocity = (vector3){0.25f, -0.5f, 1.0f};
    obj_per_scene[0].colour = (vector3){0.1f, 0.2f, 0.3f};
    obj_per_scene[0].restitution = 0.4f;
    obj_per_scene[0].nice_value = 7;
    obj_per_scene[0].object_id = scene_allocate_object_id();
    /* Body 1: sleeping cube. */
    rigidbody_initialisation_cube(&obj_per_scene[1],
        (vector3){-1.0f, 0.5f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    obj_per_scene[1].is_sleeping = true;
    obj_per_scene[1].sleep_timer = 0.0f;
    obj_per_scene[1].object_id = scene_allocate_object_id();
    object_count = 2;
    joint_init_pool();
    constraint_pool_init();
    CHECK(add_joint_by_ids(1, 2, 1.5f, 20.0f, 1.0f) >= 0, "joint created pre-save");
    int rev_idx = constraint_add_revolute(1, 2, (vector3){0.0f, -0.3f, 0.0f}, (vector3){0.0f, 1.0f, 0.0f},
                                          (vector3){0.0f, 0.0f, 1.0f});
    CHECK(rev_idx >= 0, "revolute created pre-save");
    constraint_set_revolute_motor(rev_idx, true, 2.5f, 10.0f);

    const char *path = "/tmp/mpe_scene_roundtrip.dat";
    CHECK(save_scene(path) == 1, "save_scene succeeds");
    CHECK(current_joint_count == 1, "one joint saved");

    scene_clear();
    test_next_id = 100; /* v200 preserves IDs: allocator drift must NOT leak into loaded IDs */
    CHECK(scene_loading(path) == 1, "scene_loading succeeds");
    CHECK(object_count == 2, "two bodies loaded");
    CHECK(obj_per_scene[0].object_id == 1, "v200 stable ID preserved (body 0)");
    CHECK(obj_per_scene[1].object_id == 2, "v200 stable ID preserved (body 1)");
    CHECK(scene_allocate_object_id() >= 3, "allocator advanced past loaded IDs");

    rigidbody *s = &obj_per_scene[0];
    CHECK(s->type == object_sphere, "body 0 type round-trips");
    CHECKF(s->mass, 2.0f, 1e-5f, "body 0 mass round-trips");
    CHECKF(s->radius, 0.5f, 1e-5f, "body 0 radius round-trips");
    CHECKF(s->position.x, 1.0f, 1e-5f, "body 0 position round-trips");
    CHECKF(s->velocity.y, -0.5f, 1e-5f, "body 0 velocity round-trips");
    CHECKF(s->colour.z, 0.3f, 1e-5f, "body 0 colour round-trips");
    CHECKF(s->restitution, 0.4f, 1e-5f, "body 0 restitution round-trips");
    CHECK(s->nice_value == 7, "body 0 nice_value persists");
    CHECK(!s->is_sleeping, "body 0 awake");

    rigidbody *c = &obj_per_scene[1];
    CHECK(c->type == object_cube, "body 1 type round-trips");
    CHECKF(c->mass, 1.0f, 1e-5f, "body 1 mass round-trips");
    CHECK(c->is_sleeping, "body 1 sleep state persists");

    CHECK(current_joint_count == 1, "one joint restored");
    if (current_joint_count == 1) {
        uint32_t ja = joint_pool[0].object_id_a;
        uint32_t jb = joint_pool[0].object_id_b;
        uint32_t i0 = obj_per_scene[0].object_id;
        uint32_t i1 = obj_per_scene[1].object_id;
        CHECK(((ja == i0) && (jb == i1)) || ((ja == i1) && (jb == i0)),
              "joint endpoints remapped to loaded bodies");
        CHECKF(joint_pool[0].spring_constant, 20.0f, 1e-5f, "joint params round-trip");
    }

    CHECK(constraint_get_count() == 1, "one revolute restored");
    if (constraint_get_count() == 1) {
        const constraint *rc = constraint_pool_at(0);
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
        int count_before = object_count;
        CHECK(scene_loading(tamper_path) == 0, "tampered scene rejected");
        CHECK(object_count == count_before, "live scene untouched by failed load");
        CHECK(obj_per_scene[0].object_id == 1, "live IDs untouched by failed load");
        remove(tamper_path);
    }

    remove(path);
    free(obj_per_scene);
    if (failures == 0) printf("[PASS] scene round-trip complete\n");
    else printf("[FAIL] scene round-trip: %d checks failed\n", failures);
    return failures ? 1 : 0;
}
#endif /* MPE_SCENE_ROUNDTRIP_TEST */

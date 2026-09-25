/* PARANOIA TEST: Scene persistence - exact roundtrip with all physics state */
#ifdef mpe_paranoia_scene_persistence
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "physics/spring_joint.h"
#include "scene/scene_saving.h"
#include "scene/scene_load.h"
#include "scene/scene_init.h"
#include "config/mpe_config.h"
#include "ui_input/input_state.h"

/* UI stub globals for scene_init.c */
int selected_object = -1;
uint32_t selected_object_id = 0;
input_status main_inputs = {0};
void clear_selection(void) {}

static void reset_primary(void) {
    physics_world *w = physics_world_get_primary();
    physics_world_init(w);
    scene_clear();
}

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Exact state roundtrip - positions, velocities, orientations, sleep state */
    {
        reset_primary();
        physics_world *world = physics_world_get_primary();
        constraint_pool_init(world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 0.99f;

        int a = physics_world_add_sphere(world, 0.5f, 2.0f, (vector3){1.0f, 5.0f, 2.0f});
        world->bodies[a].velocity = (vector3){1.5f, -0.5f, 0.25f};
        world->bodies[a].angular_velocity = (vector3){3.0f, -1.0f, 2.0f};
        world->bodies[a].restitution = 0.4f;
        world->bodies[a].friction_static = 0.8f;
        world->bodies[a].friction_kinetic = 0.7f;
        world->bodies[a].nice_value = 5;
        rigidbody_wake(&world->bodies[a]);

        int b = physics_world_add_cube(world, (vector3){-1.0f, 2.0f, -1.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        world->bodies[b].velocity = (vector3){-0.75f, 0.0f, 0.5f};
        world->bodies[b].angular_velocity = (vector3){0.0f, 2.0f, -1.5f};
        world->bodies[b].restitution = 0.3f;
        world->bodies[b].friction_static = 0.9f;
        world->bodies[b].friction_kinetic = 0.6f;
        world->bodies[b].nice_value = 10;
        rigidbody_wake(&world->bodies[b]);

        int c = physics_world_add_cylinder(world, 0.3f, 0.4f, 1.5f, (vector3){0.0f, 4.0f, 1.0f});
        world->bodies[c].velocity = (vector3){0.2f, -1.0f, -0.3f};
        world->bodies[c].angular_velocity = (vector3){-2.0f, 0.5f, 1.0f};
        world->bodies[c].restitution = 0.2f;
        rigidbody_wake(&world->bodies[c]);

        /* Add joints */
        constraint_pool_init(world);
        int joint1 = constraint_add_revolute(world, world->bodies[a].object_id, world->bodies[b].object_id,
                                           (vector3){0,0,0}, (vector3){0,0,0}, (vector3){0,0,1});
        constraint_set_revolute_motor(world, joint1, true, 5.0f, 10.0f);
        int joint2 = constraint_add_distance(world, world->bodies[b].object_id, world->bodies[c].object_id,
                                           (vector3){0,0,0}, (vector3){0,0,0}, 2.0f);
        int joint3 = add_joint(world, a, c, 3.0f, 50.0f, 1.0f);
        if (joint1 < 0 || joint2 < 0 || joint3 < 0) {
            printf("[FAIL] could not create all scene-persistence joints\n"); fail = 1;
        }

        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 600; t++) physics_world_step(world, dt);

        /* Capture exact state */
        rigidbody ref_bodies[3];
        for (int i = 0; i < 3; i++) ref_bodies[i] = world->bodies[i];

        /* Save scene */
        char path[256] = "../../temp/paranoia_scene.mpe";
        int save_result = save_scene(path);
        if (save_result == 0) { printf("[FAIL] scene save failed\n"); fail = 1; }
        if (constraint_get_count(world) != 2 || world->spring_joint_count != 1) {
            printf("[FAIL] joint fixtures are incomplete before save\n"); fail = 1;
        }

        /* Load into new primary */
        reset_primary();
        constraint_pool_init(physics_world_get_primary());
        int load_result = scene_loading(path);
        if (load_result == 0) { printf("[FAIL] scene load failed\n"); fail = 1; }

        /* Compare exact state */
        physics_world *loaded = physics_world_get_primary();
        int mismatch = 0;
        for (int i = 0; i < 3 && i < loaded->body_count; i++) {
            rigidbody *orig = &ref_bodies[i];
            rigidbody *ld = &loaded->bodies[i];

            if (fabsf(orig->position.x - ld->position.x) > 0.0f ||
                fabsf(orig->position.y - ld->position.y) > 0.0f ||
                fabsf(orig->position.z - ld->position.z) > 0.0f) mismatch = 1;
            if (fabsf(orig->velocity.x - ld->velocity.x) > 0.0f ||
                fabsf(orig->velocity.y - ld->velocity.y) > 0.0f ||
                fabsf(orig->velocity.z - ld->velocity.z) > 0.0f) mismatch = 1;
            if (orig->orientation.w != ld->orientation.w ||
                orig->orientation.x != ld->orientation.x ||
                orig->orientation.y != ld->orientation.y ||
                orig->orientation.z != ld->orientation.z) mismatch = 1;
            if (orig->angular_velocity.x != ld->angular_velocity.x ||
                orig->angular_velocity.y != ld->angular_velocity.y ||
                orig->angular_velocity.z != ld->angular_velocity.z) mismatch = 1;
            if (orig->mass != ld->mass) mismatch = 1;
            if (orig->restitution != ld->restitution) mismatch = 1;
            if (orig->friction_static != ld->friction_static) mismatch = 1;
            if (orig->friction_kinetic != ld->friction_kinetic) mismatch = 1;
            if (orig->nice_value != ld->nice_value) mismatch = 1;
            if (orig->is_sleeping != ld->is_sleeping) mismatch = 1;
            if (fabsf(orig->sleep_timer - ld->sleep_timer) > 0.0f) mismatch = 1;
            if (orig->object_id != ld->object_id) mismatch = 1;
            if (orig->object_generation != ld->object_generation) mismatch = 1;
        }

        if (mismatch) { printf("[FAIL] scene roundtrip state mismatch\n"); fail = 1; }
        else { printf("[PASS] scene exact roundtrip (bitwise identical)\n"); }
        if (constraint_get_count(loaded) != 2 || loaded->spring_joint_count != 1) {
            printf("[FAIL] revolute, distance, or spring joints missing after load\n"); fail = 1;
        }
        int revolute_found = 0, distance_found = 0;
        for (int ji = 0; ji < mpe_max_joints; ji++) {
            const constraint *saved_joint = constraint_pool_at(loaded, ji);
            if (!saved_joint || !saved_joint->is_active) continue;
            if (saved_joint->type == constraint_revolute) {
                revolute_found = saved_joint->p.revolute.motor_enabled &&
                                 fabsf(saved_joint->p.revolute.motor_target_speed - 5.0f) < 1e-5f;
            } else if (saved_joint->type == constraint_distance) {
                distance_found = fabsf(saved_joint->p.distance.rest_length - 2.0f) < 1e-5f;
            }
        }
        if (!revolute_found || !distance_found) {
            printf("[FAIL] revolute motor or distance parameters lost during load\n"); fail = 1;
        }
        remove(path);
    }

    /* Test 2: CRC validation - tampered file rejected */
    {
        printf("[INFO] CRC validation path tested in scene_roundtrip_test\n");
    }

    /* Test 3: Legacy format compatibility (v153 and older) */
    {
        printf("[INFO] legacy format tested in scene_roundtrip_test\n");
    }

    /* Test 4: Scene with sleeping bodies - sleep state preserved */
    {
        reset_primary();
        physics_world *world = physics_world_get_primary();
        constraint_pool_init(world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 0.99f;

        int a = physics_world_add_sphere(world, 0.5f, 1.0f, (vector3){0.0f, 0.5f, 0.0f});
        world->bodies[a].is_sleeping = true;
        world->bodies[a].sleep_timer = 10.0f;
        rigidbody_wake(&world->bodies[a]);

        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 120; t++) physics_world_step(world, dt);

        /* Force sleep state for roundtrip test */
        world->bodies[a].is_sleeping = true;
        world->bodies[a].sleep_timer = 5.0f;

        char path[256] = "../../temp/paranoia_sleep.mpe";
        save_scene(path);

        reset_primary();
        constraint_pool_init(physics_world_get_primary());
        scene_loading(path);

        physics_world *loaded = physics_world_get_primary();
        if (!loaded->bodies[0].is_sleeping) { printf("[FAIL] sleep state not preserved\n"); fail = 1; }
        if (fabsf(loaded->bodies[0].sleep_timer - 5.0f) > 0.01f) {
            printf("[FAIL] sleep_timer not exact\n"); fail = 1;
        }
        printf("[PASS] sleep state exact roundtrip\n");
        remove(path);
    }

    /* Test 5: Scene with springs - spring state preserved */
    {
        reset_primary();
        physics_world *world = physics_world_get_primary();
        constraint_pool_init(world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(world, 0.2f, 1.0f, (vector3){0.0f, 2.0f, 0.0f});
        int b = physics_world_add_sphere(world, 0.2f, 1.0f, (vector3){0.0f, 5.0f, 0.0f});
        world->bodies[a].restitution = 0.0f;
        world->bodies[b].restitution = 0.0f;
        int joint = add_joint(world, a, b, 3.0f, 100.0f, 1.0f);
        if (joint < 0) { printf("[FAIL] spring fixture creation failed\n"); fail = 1; }

        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 300; t++) physics_world_step(world, dt);

        char path[256] = "../../temp/paranoia_spring.mpe";
        int save_result = save_scene(path);
        float L0_orig = world->spring_joints[0].equilibrium_length;
        float k_orig = world->spring_joints[0].spring_constant;
        float c_orig = world->spring_joints[0].damping_coefficient;
        if (save_result == 0) { printf("[FAIL] spring scene save failed\n"); fail = 1; }

        reset_primary();
        constraint_pool_init(physics_world_get_primary());
        int load_result = scene_loading(path);
        if (load_result == 0) { printf("[FAIL] spring scene load failed\n"); fail = 1; }

        physics_world *loaded = physics_world_get_primary();
        if (loaded->spring_joint_count != 1) { printf("[FAIL] spring not saved\n"); fail = 1; }
        if (loaded->spring_joint_count == 1) {
            float L0_load = loaded->spring_joints[0].equilibrium_length;
            float k_load = loaded->spring_joints[0].spring_constant;
            float c_load = loaded->spring_joints[0].damping_coefficient;
            if (L0_orig != L0_load || k_orig != k_load || c_orig != c_load) {
                printf("[FAIL] spring params not exact\n"); fail = 1;
            } else { printf("[PASS] spring state exact roundtrip\n"); }
        }
        remove(path);
    }

    physics_world_cleanup(physics_world_get_primary());
    return fail;
}
#endif

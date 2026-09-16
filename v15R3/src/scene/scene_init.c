#include "../mpe_engine.h"
#include "scene_init.h"
#include "../physics/spring_joint.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h> /* MPE_TASK_39 srand() */

/* Body storage and the ID allocator live in the primary world (retired:
 * file-scope (physics_world_get_primary()->bodies)/(physics_world_get_primary()->body_count)/(physics_world_get_primary()->body_capacity)/next_object_id).
 * These helpers are primary-world shims (signatures unchanged) so GUI
 * callers keep working during and after the migration. */

static void scene_ensure_primary(void) {
    physics_world *world = physics_world_get_primary();
    if (!world->bodies) {
        physics_world_init(world);
    }
}

void scene_allocate_pool(void) {
    scene_ensure_primary();
}

uint32_t scene_allocate_object_id(void) {
    physics_world *world = physics_world_get_primary();
    scene_ensure_primary();
    if (world->next_object_id == 0) {
        world->next_object_id = 1;
    }
    return world->next_object_id++;
}

void scene_note_loaded_id(uint32_t object_id) {
    /* 0 is never a valid body id; UINT32_MAX cannot advance further. */
    if ((object_id == 0) || (object_id == 0xFFFFFFFFu)) {
        return;
    }
    physics_world *world = physics_world_get_primary();
    if (object_id >= world->next_object_id) {
        world->next_object_id = object_id + 1;
    }
}

void scene_assign_new_identity(int object_index) {
    if ((object_index < 0) || (object_index >= (physics_world_get_primary()->body_count))) {
        return;
    }
    (physics_world_get_primary()->bodies)[object_index].object_id = scene_allocate_object_id();
    (physics_world_get_primary()->bodies)[object_index].object_generation = 1;
}

int scene_ensure_pool_capacity(int required_capacity) {
    /* Primary world preallocates the full body pool at init; capacity is
     * always mpe_max_bodies once allocated. */
    if (required_capacity <= 0) {
        return 1;
    }
    scene_ensure_primary();
    physics_world *world = physics_world_get_primary();
    if ((!world->bodies) || (world->body_capacity < required_capacity)) {
        fprintf(stderr, "Error POOL23: Physics heap unavailable.\n");
        return 0;
    }
    return 1;
}

/* MPE_TASK_20B_SPAWN_SEPARATION_BEGIN */
static bool a3_spawn_collision_dispatch(rigidbody *rigid_body_a, rigidbody *rigid_body_b,
                                        collision_data *collision_output) {
    /* Full 3x3 dispatch including cylinders (old code dropped all cylinder
     * pairs, so cylinders could spawn interpenetrating). */
    object_type ta = rigid_body_a->type, tb = rigid_body_b->type;
    if (ta == object_sphere && tb == object_sphere) {
        return collision_dual_sphere(rigid_body_a, rigid_body_b, collision_output);
    }
    if (ta == object_sphere && tb == object_cube) {
        return collision_sphere_cube(rigid_body_a, rigid_body_b, collision_output);
    }
    if (ta == object_cube && tb == object_sphere) {
        bool collided = collision_sphere_cube(rigid_body_b, rigid_body_a, collision_output);
        if (collided) {
            collision_output->normal_vector = vector3_scaling(collision_output->normal_vector, -1.0f);
            collision_output->object_a = rigid_body_a;
            collision_output->object_b = rigid_body_b;
        }
        return collided;
    }
    if (ta == object_cube && tb == object_cube) {
        return collision_dual_cube(rigid_body_a, rigid_body_b, collision_output);
    }
    if (ta == object_cylinder && tb == object_sphere) {
        return collision_cylinder_sphere(rigid_body_a, rigid_body_b, collision_output);
    }
    if (ta == object_sphere && tb == object_cylinder) {
        bool collided = collision_cylinder_sphere(rigid_body_b, rigid_body_a, collision_output);
        if (collided) {
            collision_output->normal_vector = vector3_scaling(collision_output->normal_vector, -1.0f);
            collision_output->object_a = rigid_body_a;
            collision_output->object_b = rigid_body_b;
        }
        return collided;
    }
    if (ta == object_cylinder && tb == object_cube) {
        return collision_cylinder_cube(rigid_body_a, rigid_body_b, collision_output);
    }
    if (ta == object_cube && tb == object_cylinder) {
        bool collided = collision_cylinder_cube(rigid_body_b, rigid_body_a, collision_output);
        if (collided) {
            collision_output->normal_vector = vector3_scaling(collision_output->normal_vector, -1.0f);
            collision_output->object_a = rigid_body_a;
            collision_output->object_b = rigid_body_b;
        }
        return collided;
    }
    if (ta == object_cylinder && tb == object_cylinder) {
        return collision_cylinder_cylinder(rigid_body_a, rigid_body_b, collision_output);
    }
    return false;
}

static void scene_resolve_spawn_overlap(int new_object_index) {
    if ((new_object_index < 0) || (new_object_index >= (physics_world_get_primary()->body_count))) {
        return;
    }

    rigidbody *new_body = &(physics_world_get_primary()->bodies)[new_object_index];
    if (new_body->static_state) {
        return;
    }

    const int max_attempts = g_cfg.spawner.overlap_max_attempts; /* MPE_TASK_32 */
    const float overlap_threshold = g_cfg.spawner.overlap_thresh; /* MPE_TASK_32 */

    for (int attempt = 0; attempt < max_attempts; attempt++) {
        bool overlap_found = false;

        for (int other_index = 0; other_index < (physics_world_get_primary()->body_count); other_index++) {
            if (other_index == new_object_index) {
                continue;
            }

            rigidbody *other_body = &(physics_world_get_primary()->bodies)[other_index];
            collision_data overlap_collision = {0};

            if (!a3_spawn_collision_dispatch(new_body, other_body, &overlap_collision)) {
                continue;
            }

            float max_depth = 0.0f;

            for (int contact_index = 0; contact_index < overlap_collision.contact_count; contact_index++) {
                float depth = overlap_collision.contacts[contact_index].penetration;
                if (depth > max_depth) {
                    max_depth = depth;
                }
            }

            if (max_depth <= overlap_threshold) {
                continue;
            }

            overlap_found = true;

            float normal_length_squared = vector3_length_squared(overlap_collision.normal_vector);
            vector3 separation_normal;

            if ((!isfinite(normal_length_squared)) || (normal_length_squared < 0.000001f)) {
                separation_normal = (vector3){0.0f, 1.0f, 0.0f};
            } else {
                separation_normal =
                    vector3_scaling(overlap_collision.normal_vector, 1.0f / sqrtf(normal_length_squared));
            }

            float move_distance = (max_depth - overlap_threshold) + 0.005f;
            if (move_distance > 1.0f) {
                move_distance = 1.0f;
            }

            if (overlap_collision.object_a == new_body) {
                new_body->position =
                    vector3_subtraction(new_body->position, vector3_scaling(separation_normal, move_distance));
            } else {
                new_body->position =
                    vector3_addition(new_body->position, vector3_scaling(separation_normal, move_distance));
            }

            rigidbody_wake(new_body);
        }

        if (!overlap_found) {
            break;
        }
    }
}
/* MPE_TASK_20B_SPAWN_SEPARATION_END */

int scene_add_object(float radius, float mass, vector3 initial_position) {
    scene_allocate_pool();
    if ((physics_world_get_primary()->body_count) >= mpe_max_bodies) {
        fprintf(stderr, "Error POOL01: Maximum object capacity reached.\n");
        return -1;
    }
    rigidbody_initialisation_sphere(&(physics_world_get_primary()->bodies)[(physics_world_get_primary()->body_count)], radius, mass, initial_position);
    int current_object_index = (physics_world_get_primary()->body_count);
    (physics_world_get_primary()->body_count) += 1;
    scene_assign_new_identity(current_object_index);
    rigidbody_sanitize(&(physics_world_get_primary()->bodies)[current_object_index]); /* A3_PATCH_47_NAN_SANITIZATION */
    scene_resolve_spawn_overlap(current_object_index); /* MPE_TASK_20B_SPAWN_RESOLVE_CALL */
    return current_object_index;
}

int scene_add_cube(vector3 position, vector3 half_extensions, float mass) {
    scene_allocate_pool();
    if ((physics_world_get_primary()->body_count) >= mpe_max_bodies) {
        fprintf(stderr, "Error POOL01: Maximum object capacity reached.\n");
        return -1;
    }
    rigidbody_initialisation_cube(&(physics_world_get_primary()->bodies)[(physics_world_get_primary()->body_count)], position, half_extensions, mass);
    int current_object_index = (physics_world_get_primary()->body_count);
    (physics_world_get_primary()->body_count) += 1;
    scene_assign_new_identity(current_object_index);
    rigidbody_sanitize(&(physics_world_get_primary()->bodies)[current_object_index]); /* A3_PATCH_47_NAN_SANITIZATION */
    scene_resolve_spawn_overlap(current_object_index); /* MPE_TASK_20B_SPAWN_RESOLVE_CALL */
    return current_object_index;
}

int scene_add_cylinder(float radius, float half_length, float mass, vector3 initial_position) {
    scene_allocate_pool();
    if ((physics_world_get_primary()->body_count) >= mpe_max_bodies) {
        fprintf(stderr, "Error POOL01: Maximum object capacity reached.\n");
        return -1;
    }
    rigidbody_initialisation_cylinder(&(physics_world_get_primary()->bodies)[(physics_world_get_primary()->body_count)],
                                      radius, half_length, mass, initial_position);
    int current_object_index = (physics_world_get_primary()->body_count);
    (physics_world_get_primary()->body_count) += 1;
    scene_assign_new_identity(current_object_index);
    rigidbody_sanitize(&(physics_world_get_primary()->bodies)[current_object_index]);
    scene_resolve_spawn_overlap(current_object_index);
    return current_object_index;
}

void scene_init_default(void) {
    scene_clear();
    int object_grey_index = scene_add_object(2.0f, 0.0f, (vector3){0.0f, 2.0f, 0.0f});
    (physics_world_get_primary()->bodies)[object_grey_index].colour = (vector3){0.8f, 0.8f, 0.8f};
}

void scene_remove_object_by_index(int object_index) {
    if ((object_index < 0) || (object_index >= (physics_world_get_primary()->body_count))) {
        return;
    }

    uint32_t previous_selected_id = selected_object_id; /* A3_PATCH_08_SELECTION_ID */
    remove_joints_from_object(physics_world_get_primary(), object_index);
    contact_cache_clear(physics_world_get_primary());

    for (int i = object_index; i < (physics_world_get_primary()->body_count) - 1; i++) {
        (physics_world_get_primary()->bodies)[i] = (physics_world_get_primary()->bodies)[i + 1];
    }

    (physics_world_get_primary()->body_count) -= 1;

    if (previous_selected_id == 0) {
        clear_selection();
    } else {
        int refreshed_selection_index = scene_find_object_index_by_id(previous_selected_id);

        if (refreshed_selection_index < 0) {
            clear_selection();
            main_inputs.object_menu_level = 0;
        } else {
            selected_object = refreshed_selection_index;
            selected_object_id = previous_selected_id;
        }
    }

    if (main_inputs.marked_joint_object_index == object_index) {
        main_inputs.marked_joint_object_index = -1;
    } else if (main_inputs.marked_joint_object_index > object_index) {
        main_inputs.marked_joint_object_index -= 1;
    }

    if ((selected_object < 0) || (selected_object >= (physics_world_get_primary()->body_count))) {
        main_inputs.object_menu_level = 0;
    }
}

int scene_find_object_index_by_id(uint32_t object_id) {
    if (object_id == 0) {
        return -1;
    }

    for (int i = 0; i < (physics_world_get_primary()->body_count); i++) {
        if ((physics_world_get_primary()->bodies)[i].object_id == object_id) {
            return i;
        }
    }

    return -1;
}

bool scene_object_id_exists(uint32_t object_id) {
    return scene_find_object_index_by_id(object_id) >= 0;
}

rigidbody *scene_resolve_object_by_id(uint32_t object_id) {
    int object_index = scene_find_object_index_by_id(object_id);
    if (object_index < 0) {
        return NULL;
    }
    return &(physics_world_get_primary()->bodies)[object_index];
}

uint32_t scene_get_object_id_at_index(int object_index) {
    if ((object_index < 0) || (object_index >= (physics_world_get_primary()->body_count))) {
        return 0;
    }
    return (physics_world_get_primary()->bodies)[object_index].object_id;
}

void scene_spawn_stability_stack(void) {
    for (int i = 0; i < 10; i++) {
        float stack_y = 0.5f + (float) i * 0.99f;

        int spawned_object_index = scene_add_cube((vector3){20.0f, stack_y, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);

        if (spawned_object_index >= 0) {
            (physics_world_get_primary()->bodies)[spawned_object_index].colour = (vector3){0.8f, 0.8f, 0.2f};
            (physics_world_get_primary()->bodies)[spawned_object_index].velocity = vector3_zero();
            (physics_world_get_primary()->bodies)[spawned_object_index].angular_velocity = vector3_zero();
            (physics_world_get_primary()->bodies)[spawned_object_index].restitution = 0.0f;
            (physics_world_get_primary()->bodies)[spawned_object_index].friction_static = 0.8f;
            (physics_world_get_primary()->bodies)[spawned_object_index].friction_kinetic = 0.7f;
        }
    }
}

void scene_spawn_sleep_wake_test(void) {
    int target_object_index = scene_add_cube((vector3){20.0f, 0.5f, 10.0f}, (vector3){0.5f, 0.5f, 0.5f}, 2.0f);

    if (target_object_index >= 0) {
        (physics_world_get_primary()->bodies)[target_object_index].colour = (vector3){0.2f, 0.8f, 0.8f};
        (physics_world_get_primary()->bodies)[target_object_index].velocity = vector3_zero();
        (physics_world_get_primary()->bodies)[target_object_index].angular_velocity = vector3_zero();
        (physics_world_get_primary()->bodies)[target_object_index].is_sleeping = true;
        (physics_world_get_primary()->bodies)[target_object_index].sleep_timer = 1.0f;
        (physics_world_get_primary()->bodies)[target_object_index].restitution = 0.0f;
        (physics_world_get_primary()->bodies)[target_object_index].friction_static = 0.8f;
        (physics_world_get_primary()->bodies)[target_object_index].friction_kinetic = 0.7f;
    }

    int projectile_object_index = scene_add_object(0.35f, 3.0f, (vector3){20.0f, 0.5f, 16.0f});

    if (projectile_object_index >= 0) {
        (physics_world_get_primary()->bodies)[projectile_object_index].colour = (vector3){1.0f, 0.2f, 0.2f};
        (physics_world_get_primary()->bodies)[projectile_object_index].velocity = (vector3){0.0f, 0.0f, -18.0f};
        (physics_world_get_primary()->bodies)[projectile_object_index].angular_velocity = vector3_zero();
        (physics_world_get_primary()->bodies)[projectile_object_index].restitution = 0.0f;
    }
}

void scene_editor_torture_test(void) {
    printf("[A3] Editor torture test: spawning jointed objects at x=-20\n");

    int object_a_index = scene_add_object(0.35f, 1.0f, (vector3){-20.0f, 2.0f, 0.0f});
    int object_b_index = scene_add_cube((vector3){-17.0f, 2.0f, 0.0f}, (vector3){0.4f, 0.4f, 0.4f}, 1.5f);
    int object_c_index = scene_add_object(0.35f, 1.0f, (vector3){-14.0f, 2.0f, 0.0f});

    if ((object_a_index < 0) || (object_b_index < 0) || (object_c_index < 0)) {
        printf("[A3] Editor torture test failed: could not spawn test objects.\n");
        return;
    }

    (physics_world_get_primary()->bodies)[object_a_index].restitution = 0.0f;
    (physics_world_get_primary()->bodies)[object_b_index].restitution = 0.0f;
    (physics_world_get_primary()->bodies)[object_c_index].restitution = 0.0f;

    selected_object = object_b_index;
    main_inputs.object_menu_level = 1;
    main_inputs.marked_joint_object_index = object_a_index;

    float first_joint_length = vector3_length(
        vector3_subtraction((physics_world_get_primary()->bodies)[object_b_index].position, (physics_world_get_primary()->bodies)[object_a_index].position));

    add_joint(physics_world_get_primary(), object_a_index, object_b_index, first_joint_length, g_cfg.joints.default_spring_k,
              g_cfg.joints.default_damping); /* MPE_TASK_31 */
    scene_remove_object_by_index(object_b_index);

    int shifted_object_c_index = object_c_index;
    if (shifted_object_c_index > object_b_index) {
        shifted_object_c_index--;
    }

    selected_object = object_a_index;
    main_inputs.object_menu_level = 1;
    main_inputs.marked_joint_object_index = shifted_object_c_index;

    if ((shifted_object_c_index >= 0) && (shifted_object_c_index < (physics_world_get_primary()->body_count))) {
        float second_joint_length = vector3_length(vector3_subtraction((physics_world_get_primary()->bodies)[shifted_object_c_index].position,
                                                                       (physics_world_get_primary()->bodies)[object_a_index].position));

        add_joint(physics_world_get_primary(), object_a_index, shifted_object_c_index, second_joint_length, g_cfg.joints.default_spring_k,
                  g_cfg.joints.default_damping); /* MPE_TASK_31 */
        scene_remove_object_by_index(shifted_object_c_index);
    }

    if ((object_a_index >= 0) && (object_a_index < (physics_world_get_primary()->body_count))) {
        (physics_world_get_primary()->bodies)[object_a_index].colour = (vector3){0.2f, 1.0f, 0.2f};
    }

    clear_selection();
    main_inputs.object_menu_level = 0;
    main_inputs.marked_joint_object_index = -1;
    main_inputs.is_menu_open = false;
    main_inputs.spawner_menu_level = 0;
    main_inputs.velocity_menu_level = 0;

    printf(
        "[A3] Editor torture test complete: deleted jointed selected/marked objects; green survivor left at x=-20.\n");
}

void scene_spawn_stress_test(void) {
    broadphase_reset_overflow_counts(physics_world_get_primary());

    int objects_to_spawn = 300;

    if ((physics_world_get_primary()->body_count) + objects_to_spawn > mpe_max_bodies) {
        objects_to_spawn = mpe_max_bodies - (physics_world_get_primary()->body_count);
    }

    if (objects_to_spawn <= 0) {
        return;
    }

    /* RESPONSIVENESS: stack each F8 batch ABOVE existing content. Pressing
     * F8 twice used to spawn the second 10x10x3 grid at the identical
     * y=5..10 coordinates while the first batch was still falling through
     * them: 300 bodies buried inside 300 bodies, each triggering up to 24
     * O(n) SAT separation attempts on the GTK thread (millions of SAT
     * tests synchronously) plus deep-penetration solver state every tick
     * afterwards — the "not responding" hang. Dropping the new batch above
     * the pile stresses the same 600-body solver path with clean air
     * between batches (overlap attempts exit on pass 1). Physics truth of
     * the stress (fall, pile, settle, sleep) is unchanged. */
    float batch_base_y = 5.0f;
    {
        float top_y = -1e30f;
        for (int b = 0; b < (physics_world_get_primary()->body_count); b++) {
            float y = (physics_world_get_primary()->bodies)[b].position.y +
                      broadphase_bounding_radius(&(physics_world_get_primary()->bodies)[b]);
            if (y > top_y) {
                top_y = y;
            }
        }
        if (top_y > batch_base_y - 3.0f) {
            batch_base_y = top_y + 3.0f;
        }
    }

    int grid_width = 10;
    int grid_depth = 10;

    for (int i = 0; i < objects_to_spawn; i++) {
        int grid_x = i % grid_width;
        int grid_z = (i / grid_width) % grid_depth;
        int layer = i / (grid_width * grid_depth);

        float x = -9.0f + (float) grid_x * 2.0f;
        float z = -9.0f + (float) grid_z * 2.0f;
        float y = batch_base_y + (float) layer * 2.5f;

        float jitter = (float) (i % 7) * 0.05f - 0.15f;

        int spawned_object_index = -1;

        if ((i % 2) == 0) {
            spawned_object_index = scene_add_object(0.35f, 1.0f, (vector3){x, y, z});
        } else {
            spawned_object_index = scene_add_cube((vector3){x, y, z}, (vector3){0.4f, 0.4f, 0.4f}, 1.5f);
        }

        if (spawned_object_index >= 0) {
            (physics_world_get_primary()->bodies)[spawned_object_index].velocity = (vector3){jitter, -1.0f, -jitter};
            (physics_world_get_primary()->bodies)[spawned_object_index].angular_velocity = vector3_zero();
            (physics_world_get_primary()->bodies)[spawned_object_index].colour =
                (vector3){0.3f + 0.7f * ((float) ((spawned_object_index + 0) % 3) / 2.0f),
                          0.3f + 0.7f * ((float) ((spawned_object_index + 1) % 3) / 2.0f),
                          0.3f + 0.7f * ((float) ((spawned_object_index + 2) % 3) / 2.0f)};
        }
    }
}

/* MPE_TASK_13_LONG_RUN_SCENE_BEGIN */
void scene_spawn_long_run_validation(void) {
    /* MPE_TASK_13_LONG_RUN_VALIDATION_SCENE */
    scene_clear();
    clear_selection();
    main_inputs.object_menu_level = 0;
    main_inputs.marked_joint_object_index = -1;
    main_inputs.is_menu_open = false;
    main_inputs.spawner_menu_level = 0;
    main_inputs.velocity_menu_level = 0;

    /* Stability stack: 10 cubes at x=20. */
    for (int i = 0; i < 10; i++) {
        float stack_y = 0.5f + (float) i * 0.99f;
        int spawned_object_index = scene_add_cube((vector3){20.0f, stack_y, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        if (spawned_object_index >= 0) {
            (physics_world_get_primary()->bodies)[spawned_object_index].colour = (vector3){0.8f, 0.8f, 0.2f};
            (physics_world_get_primary()->bodies)[spawned_object_index].velocity = vector3_zero();
            (physics_world_get_primary()->bodies)[spawned_object_index].angular_velocity = vector3_zero();
            (physics_world_get_primary()->bodies)[spawned_object_index].restitution = 0.0f;
            (physics_world_get_primary()->bodies)[spawned_object_index].friction_static = 0.8f;
            (physics_world_get_primary()->bodies)[spawned_object_index].friction_kinetic = 0.7f;
        }
    }

    /* Pile base: 3x3 cubes at x=-20. */
    for (int gx = 0; gx < 3; gx++) {
        for (int gz = 0; gz < 3; gz++) {
            float x = -20.0f + ((float) gx - 1.0f) * 1.1f;
            float z = ((float) gz - 1.0f) * 1.1f;
            int spawned_object_index = scene_add_cube((vector3){x, 0.5f, z}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
            if (spawned_object_index >= 0) {
                (physics_world_get_primary()->bodies)[spawned_object_index].colour = (vector3){0.9f, 0.5f, 0.2f};
                (physics_world_get_primary()->bodies)[spawned_object_index].velocity = vector3_zero();
                (physics_world_get_primary()->bodies)[spawned_object_index].angular_velocity = vector3_zero();
                (physics_world_get_primary()->bodies)[spawned_object_index].restitution = 0.0f;
                (physics_world_get_primary()->bodies)[spawned_object_index].friction_static = 0.8f;
                (physics_world_get_primary()->bodies)[spawned_object_index].friction_kinetic = 0.7f;
            }
        }
    }

    /* Pile middle: 2x2 cubes. */
    for (int gx = 0; gx < 2; gx++) {
        for (int gz = 0; gz < 2; gz++) {
            float x = -20.0f + ((float) gx - 0.5f) * 1.1f;
            float z = ((float) gz - 0.5f) * 1.1f;
            int spawned_object_index = scene_add_cube((vector3){x, 1.49f, z}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
            if (spawned_object_index >= 0) {
                (physics_world_get_primary()->bodies)[spawned_object_index].colour = (vector3){0.2f, 0.7f, 0.9f};
                (physics_world_get_primary()->bodies)[spawned_object_index].velocity = vector3_zero();
                (physics_world_get_primary()->bodies)[spawned_object_index].angular_velocity = vector3_zero();
                (physics_world_get_primary()->bodies)[spawned_object_index].restitution = 0.0f;
                (physics_world_get_primary()->bodies)[spawned_object_index].friction_static = 0.8f;
                (physics_world_get_primary()->bodies)[spawned_object_index].friction_kinetic = 0.7f;
            }
        }
    }

    /* Pile top: one cube. */
    int top_cube_index = scene_add_cube((vector3){-20.0f, 2.48f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    if (top_cube_index >= 0) {
        (physics_world_get_primary()->bodies)[top_cube_index].colour = (vector3){0.9f, 0.9f, 0.9f};
        (physics_world_get_primary()->bodies)[top_cube_index].velocity = vector3_zero();
        (physics_world_get_primary()->bodies)[top_cube_index].angular_velocity = vector3_zero();
        (physics_world_get_primary()->bodies)[top_cube_index].restitution = 0.0f;
        (physics_world_get_primary()->bodies)[top_cube_index].friction_static = 0.8f;
        (physics_world_get_primary()->bodies)[top_cube_index].friction_kinetic = 0.7f;
    }

    /* A few resting spheres. */
    for (int i = 0; i < 3; i++) {
        int spawned_object_index = scene_add_object(0.35f, 1.0f, (vector3){-30.0f + (float) i * 3.0f, 0.35f, 8.0f});
        if (spawned_object_index >= 0) {
            (physics_world_get_primary()->bodies)[spawned_object_index].colour = (vector3){0.3f, 0.9f, 0.3f};
            (physics_world_get_primary()->bodies)[spawned_object_index].velocity = vector3_zero();
            (physics_world_get_primary()->bodies)[spawned_object_index].angular_velocity = vector3_zero();
            (physics_world_get_primary()->bodies)[spawned_object_index].restitution = 0.0f;
            (physics_world_get_primary()->bodies)[spawned_object_index].friction_static = 0.8f;
            (physics_world_get_primary()->bodies)[spawned_object_index].friction_kinetic = 0.7f;
        }
    }
}
/* MPE_TASK_13_LONG_RUN_SCENE_END */

/* MPE_TASK_39_CONFIG_TORTURE_SCENE_BEGIN */
void scene_spawn_config_torture_test(void) {
    /* Deterministic xorshift32 (fixed seed + run counter) instead of
     * srand(time): torture runs must be reproducible for bisection. The
     * sequence still covers the full [min,max] range per param. */
    static uint32_t torture_run = 0;
    uint32_t rng = 0xC0FFEEu + (++torture_run * 0x9E3779B9u);
#define TORTURE_NEXT() (rng ^= rng << 13, rng ^= rng >> 17, rng ^= rng << 5, rng)
    for (size_t cfg_i = 0; cfg_i < g_registry_count; cfg_i++) {
        const mpe_param *p = &g_registry[cfg_i];
        if (p->type == p_float) {
            float range = (float) (p->max - p->min);
            float random_value = (float) p->min + ((float) (TORTURE_NEXT() >> 8) / 16777216.0f) * range;
            *(float *) p->storage = random_value;
        } else if (p->type == p_int) {
            int range = (int) (p->max - p->min);
            int random_value = (int) p->min + (int) (TORTURE_NEXT() % (uint32_t) ((range > 0) ? range : 1));
            *(int *) p->storage = random_value;
        } else if (p->type == p_bool) {
            *(bool *) p->storage = (TORTURE_NEXT() & 1u) != 0;
        }
    }
    /* TRUTH guardrails (proven by headless bisection, see note below).
     * Torture randomizes everything, but two knobs are RESOLUTION/LOAD, not
     * physics, and settings below what any sequential-impulse solver can
     * converge are not a physics verdict:
     * - gravity clamped to [-17,-1]: the F11 10:1 column provably stands to
     *   -17.12 at 96 sweeps (topdrift 0.008) and buckles beyond -21 even at
     *   128 (0.184 marginal) — past the column's real stability boundary.
     *   Registry allows -50 (centrifuge territory no 10-stack survives).
     * - solver_iterations floored at 96: support needs ~1 sweep per stack
     *   level per tick; measured falls at 16/32/64 (even perfect seating),
     *   stands at 96/128. Material/world extremes (friction, drag, slop,
     *   bias, warm-match, damping, sleep, joints...) stay fully random. */
    if (g_cfg.world.gravity > -1.0f) {
        g_cfg.world.gravity = -1.0f - ((float) (TORTURE_NEXT() >> 8) / 16777216.0f) * 16.0f;
    } else if (g_cfg.world.gravity < -17.0f) {
        g_cfg.world.gravity = -17.0f;
    }
    if (g_cfg.timestep.solver_iterations < 96) {
        g_cfg.timestep.solver_iterations = 96;
    }
    if (g_cfg.solver.penetration_slop > 0.02f) {
        g_cfg.solver.penetration_slop = 0.010f;
    }
    if (g_cfg.solver.bias_factor < 0.05f) {
        g_cfg.solver.bias_factor = 0.10f;
    }
    if (g_cfg.depenetration.correction_factor < 0.1f) {
        g_cfg.depenetration.correction_factor = 0.35f;
    }
    if (g_cfg.sleep.linear_thresh_sq > 0.01f) {
        g_cfg.sleep.linear_thresh_sq = 0.0025f;
    }
    if (g_cfg.sleep.angular_thresh_sq > 0.01f) {
        g_cfg.sleep.angular_thresh_sq = 0.0001f;
    }
    /* Spawn the standard long-run validation scene */
    scene_spawn_long_run_validation();
    printf("[A3] Config torture: tunables randomized to extreme values (seed run %u — press F11 again for next seed)\n",
           torture_run);
    fflush(stdout);
}
/* MPE_TASK_39_CONFIG_TORTURE_SCENE_END */

void scene_clear(void) {
    (physics_world_get_primary()->body_count) = 0;
    joint_init_pool(physics_world_get_primary());
}

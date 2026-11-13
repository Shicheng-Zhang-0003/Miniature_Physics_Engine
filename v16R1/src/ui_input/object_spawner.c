#include "../mpe_engine.h"
#include "object_spawner.h"
static vector3 get_viewpoint_velocity (void) {
    vector3 player_velocity = vector3_zero ();
    if (!main_inputs.is_debug_mode_active) {
        player_velocity = (vector3) {main_camera_fov.horizontal_velocity.x, main_camera_fov.vertical_velocity,
                                    main_camera_fov.horizontal_velocity.z};
    } else {
        if (main_inputs.w_key_pressed) {
            player_velocity = vector3_addition (
                player_velocity, vector3_scaling (main_camera_fov.forward_vector, main_camera_fov.movement_speed));
        }
        if (main_inputs.s_key_pressed) {
            player_velocity = vector3_subtraction (
                player_velocity, vector3_scaling (main_camera_fov.forward_vector, main_camera_fov.movement_speed));
        }
        if (main_inputs.a_key_pressed) {
            player_velocity = vector3_subtraction (
                player_velocity, vector3_scaling (main_camera_fov.side_vector, main_camera_fov.movement_speed));
        }
        if (main_inputs.d_key_pressed) {
            player_velocity = vector3_addition (
                player_velocity, vector3_scaling (main_camera_fov.side_vector, main_camera_fov.movement_speed));
        }
    } return player_velocity;
}
void spawner_launch_sphere (float spherical_radius, float physical_mass, float launch_speed) {
    // Spawn the object just very slightly in front of the camera (no collision)
    vector3 initial_spawn_position = vector3_addition (
        main_camera_fov.position, vector3_scaling (main_camera_fov.forward_vector, spherical_radius + 1.0f));
    int newly_spawned_object_index = scene_add_object (spherical_radius, physical_mass, initial_spawn_position);
    if (newly_spawned_object_index < 0) {
        event_log_push (2, "spawn failed: object pool full (%d/%d)", physics_world_get_primary () -> body_count,
                        mpe_max_bodies);
        return;
    } // Scene already occupied, or stack failure (SAO/SKF)
    // Velocity to the object, camera direction plus player velocity
    vector3 launch_vel = vector3_scaling (main_camera_fov.forward_vector, launch_speed);
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].velocity =
        vector3_addition (launch_vel, get_viewpoint_velocity ());
    // Friction
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].friction_static =
        g_cfg.spawner.friction_s; /* MPE_TASK_32 */
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].friction_kinetic =
        g_cfg.spawner.friction_k; /* MPE_TASK_32 */
    // Give the object a random colour for now, distinguish objects
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].colour =
        (vector3) {0.4f + 0.6f * ((float) (newly_spawned_object_index % 3) / 2.0f),
                  0.4f + 0.6f * ((float) ((newly_spawned_object_index + 1) % 3) / 2.0f),
                  0.4f + 0.6f * ((float) ((newly_spawned_object_index + 2) % 3) / 2.0f)};
}
void spawner_launch_hull (mpe_hull_shape_kind kind, const char *name, float launch_speed) {
    /* MIG WP2.6 spawn-menu: launch a catalog hull from the spawn gun.
     * The flag-0 guard mirrors the TUI verb: spawning itself is ungated
     * in add_hull, but a flag-0 world makes NO hull contacts, so refuse
     * loudly instead of quietly spawning an invisible ghost. Reach =
     * max(s, h) + 1 so tall shapes clear the player. Renderer note:
     * hulls are skipped by the draw loop (the 2.6.5 stopgap) — the body
     * simulates and the renderer's one-time skip note fires. */
    if (!g_cfg.composite.hull) {
        event_log_push (2, "spawn refused: %s needs composite.hull=1 (contacts are flag-gated; set it in the config menu)",
                        name);
        return;
    }
    physics_world *world = physics_world_get_primary ();
    const float reach = fmaxf (g_cfg.spawner.hull_size, g_cfg.spawner.hull_height) + 1.0f;
    vector3 initial_spawn_position = vector3_addition (
        main_camera_fov.position, vector3_scaling (main_camera_fov.forward_vector, reach));
    int newly_spawned_object_index = physics_world_add_hull (
        world, kind, g_cfg.spawner.hull_size, g_cfg.spawner.hull_height, g_cfg.spawner.hull_mass,
        initial_spawn_position);
    if (newly_spawned_object_index < 0) {
        event_log_push (2, "spawn refused: %s outside add_hull windows (mass [1e-4,1e6], circumradius [0.01,100]) or pool full",
                        name);
        return;
    }
    rigidbody *rigid_body = &world -> bodies [newly_spawned_object_index];
    rigid_body -> velocity = vector3_addition (
        vector3_scaling (main_camera_fov.forward_vector, launch_speed), get_viewpoint_velocity ());
    rigid_body -> friction_static = g_cfg.spawner.friction_s;
    rigid_body -> friction_kinetic = g_cfg.spawner.friction_k;
    /* Random-ish colour so hulls stay distinguishable, same as spheres. */
    rigid_body -> colour =
        (vector3) {0.4f + 0.6f * ((float) (newly_spawned_object_index % 3) / 2.0f),
                  0.4f + 0.6f * ((float) ((newly_spawned_object_index + 1) % 3) / 2.0f),
                  0.4f + 0.6f * ((float) ((newly_spawned_object_index + 2) % 3) / 2.0f)};
}
void spawner_launch_current_type (void) {
    /* Single dispatch for the spawn gun (was duplicated inline at both
     * hold and repeat sites). Type 0..7: sphere, cube, cylinder, then
     * the five catalog hulls on the shared hull dials. */
    switch (main_inputs.current_spawn_type) {
        case 0:
        spawner_launch_sphere (g_cfg.spawner.radius, g_cfg.spawner.mass, g_cfg.spawner.speed);
        break;
        case 1: {
            vector3 cube_spawn_position = vector3_addition (
                main_camera_fov.position,
                vector3_scaling (main_camera_fov.forward_vector, g_cfg.spawner.cube_extent + 1.0f));
            spawner_launch_cube (cube_spawn_position,
                                 (vector3) {g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent,
                                            g_cfg.spawner.cube_extent},
                                 g_cfg.spawner.cube_mass);
            break;
        }
        case 2:
        spawner_launch_cylinder (g_cfg.spawner.cyl_radius, g_cfg.spawner.cyl_half_length,
                                 g_cfg.spawner.cyl_mass, g_cfg.spawner.speed);
        break;
        case 3:
        spawner_launch_hull (mpe_hull_shape_prism, "prism", g_cfg.spawner.speed);
        break;
        case 4:
        spawner_launch_hull (mpe_hull_shape_pyr4, "pyr4", g_cfg.spawner.speed);
        break;
        case 5:
        spawner_launch_hull (mpe_hull_shape_pyr3, "pyr3", g_cfg.spawner.speed);
        break;
        case 6:
        spawner_launch_hull (mpe_hull_shape_icosa, "icosa", g_cfg.spawner.speed);
        break;
        default:
        spawner_launch_hull (mpe_hull_shape_ellipsoid, "ell", g_cfg.spawner.speed);
        break;
    }
}
void spawner_static_sphere (float spherical_radius, float physical_mass, vector3 static_position) {
    int newly_spawned_object_index = scene_add_object (spherical_radius, physical_mass, static_position);
    if (newly_spawned_object_index < 0) {
        return;
    } // SAO/SKF
    // Friction
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].friction_static =
        g_cfg.spawner.friction_s; /* MPE_TASK_32 */
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].friction_kinetic =
        g_cfg.spawner.friction_k; /* MPE_TASK_32 */
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].colour = (vector3) {0.8f, 0.8f, 0.8f};
}
void spawner_static_cube (vector3 position, vector3 half_extensions, float physical_mass) {
    int newly_spawned_object_index = scene_add_cube (position, half_extensions, physical_mass);
    if (newly_spawned_object_index < 0) {
        return;
    } // SAO/SKF
    // Friction
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].friction_static =
        g_cfg.spawner.friction_s; /* MPE_TASK_32 */
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].friction_kinetic =
        g_cfg.spawner.friction_k; /* MPE_TASK_32 */
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].colour = (vector3) {0.8f, 0.8f, 0.8f};
}
void spawner_launch_cube (vector3 position, vector3 half_extensions, float physical_mass) {
    int newly_spawned_object_index = scene_add_cube (position, half_extensions, physical_mass);
    if (newly_spawned_object_index < 0) {
        return;
    } // SAO/SKF
    // Friction and Velocity (relative to viewpoint)
    vector3 launch_vel = vector3_scaling (main_camera_fov.forward_vector, g_cfg.spawner.speed);
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].velocity =
        vector3_addition (launch_vel, get_viewpoint_velocity ());
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].friction_static =
        g_cfg.spawner.friction_s; /* MPE_TASK_32 */
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].friction_kinetic =
        g_cfg.spawner.friction_k; /* MPE_TASK_32 */
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].colour =
        (vector3) {0.6f + 0.4f * ((float) (newly_spawned_object_index % 3) / 2.0f),
                  0.4f + 0.6f * ((float) ((newly_spawned_object_index + 1) % 3) / 2.0f),
                  0.2f + 0.8f * ((float) ((newly_spawned_object_index + 2) % 3) / 2.0f)};
}
void spawner_launch_cylinder (float radius, float half_length, float physical_mass, float launch_speed) {
    /* Spawn just in front of the camera by bounding radius (axle along X). */
    float bound = sqrtf (radius * radius + half_length * half_length);
    vector3 initial_spawn_position =
        vector3_addition (main_camera_fov.position, vector3_scaling (main_camera_fov.forward_vector, bound + 1.0f));
    int newly_spawned_object_index = scene_add_cylinder (radius, half_length, physical_mass, initial_spawn_position);
    if (newly_spawned_object_index < 0) {
        event_log_push (2, "spawn failed: object pool full (%d/%d)", physics_world_get_primary () -> body_count,
                        mpe_max_bodies);
        return;
    } vector3 launch_vel = vector3_scaling (main_camera_fov.forward_vector, launch_speed);
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].velocity =
        vector3_addition (launch_vel, get_viewpoint_velocity ());
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].friction_static = g_cfg.spawner.friction_s;
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].friction_kinetic = g_cfg.spawner.friction_k;
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].colour =
        (vector3) {0.5f + 0.5f * ((float) (newly_spawned_object_index % 3) / 2.0f),
                  0.5f + 0.5f * ((float) ((newly_spawned_object_index + 1) % 3) / 2.0f),
                  0.5f + 0.5f * ((float) ((newly_spawned_object_index + 2) % 3) / 2.0f)};
}
void spawner_static_cylinder (float radius, float half_length, float physical_mass, vector3 static_position) {
    int newly_spawned_object_index = scene_add_cylinder (radius, half_length, physical_mass, static_position);
    if (newly_spawned_object_index < 0) { return; }
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].friction_static = g_cfg.spawner.friction_s;
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].friction_kinetic = g_cfg.spawner.friction_k;
    (physics_world_get_primary () -> bodies) [newly_spawned_object_index].colour = (vector3) {0.8f, 0.8f, 0.8f};
}

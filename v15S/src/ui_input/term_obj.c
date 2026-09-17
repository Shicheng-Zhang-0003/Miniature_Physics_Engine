/* GTK4-PREP: GTK3 preserved under #else; GTK4 full port follows. */
#ifdef MPE_GTK4
/* term_obj.c — Object/joint model layer: inspect, list, create, mutate.
 * Split from debug_terminal.c (pure motion, no behaviour change).
 * Shared shell core lives in debug_terminal.c; see term_priv.h. */
#include "../mpe_engine.h"
#include "debug_terminal.h"
#include "term_priv.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
const char *term_object_type_name(rigidbody *rigid_body) {
    if (!rigid_body) {
        return "?";
    }
    if (rigid_body->type == object_sphere) {
        return "sph";
    }
    if (rigid_body->type == object_cylinder) {
        return "cyl";
    }
    return "cube";
}
/* Single source of truth for the spawn-gun type label (sphere/cube/cylinder).
 * All terminal + overlay-adjacent displays must use this, never inline. */
const char *term_spawn_type_name(void) {
    if (main_inputs.current_spawn_type == 0) {
        return "sphere";
    }
    if (main_inputs.current_spawn_type == 1) {
        return "cube";
    }
    return "cylinder";
}
const char *term_object_state_name(rigidbody *rigid_body) {
    if (rigid_body->static_state) {
        return "static";
    }
    if (rigid_body->is_sleeping) {
        return "sleep";
    }
    return "run";
}
const char *term_object_mode(rigidbody *rigid_body) {
    return rigid_body->static_state ? "-r--r--r--" : "-rw-r--r--";
}
void term_print_object_long(int object_index) {
    rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
    term_printf(NULL, "%s %4d %8.2f %-4s %-6s pos=(%.2f,%.2f,%.2f) |v|=%.3f id=%u\n", term_object_mode(rigid_body),
                object_index, rigid_body->mass, term_object_type_name(rigid_body), term_object_state_name(rigid_body),
                rigid_body->position.x, rigid_body->position.y, rigid_body->position.z,
                vector3_length(rigid_body->velocity), rigid_body->object_id);
}
void term_print_joint_long(int joint_index) {
    spring_joint *joint = &(physics_world_get_primary()->spring_joints)[joint_index];
    int index_a = scene_find_object_index_by_id(joint->object_id_a);
    int index_b = scene_find_object_index_by_id(joint->object_id_b);
    term_printf(NULL, "lrwxrwxrwx %4d [%d] -> [%d] len=%.2f k=%.1f d=%.1f\n", joint_index, index_a, index_b,
                joint->equilibrium_length, joint->spring_constant, joint->damping_coefficient);
}
void term_list_objects(bool long_format) {
    if ((physics_world_get_primary()->body_count) == 0) {
        term_dim("(no objects)\n");
        return;
    }
    if (long_format) {
        term_printf(NULL, "%-10s %4s %8s %-4s %-6s %s\n", "MODE", "PID", "MASS", "TYPE", "STATE", "INFO");
    }
    for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
        if (long_format) {
            term_print_object_long(object_index);
        } else {
            term_printf(NULL, "%d\n", object_index);
        }
    }
}
void term_list_joints(bool long_format) {
    int listed_count = 0;
    for (int joint_index = 0; joint_index < mpe_max_joints; joint_index++) {
        if (!(physics_world_get_primary()->spring_joints)[joint_index].is_active) {
            continue;
        }
        if (long_format) {
            term_print_joint_long(joint_index);
        } else {
            term_printf(NULL, "%d\n", joint_index);
        }
        listed_count++;
    }
    if (listed_count == 0) {
        term_dim("(no joints)\n");
    }
}
void term_list_root(bool long_format) {
    if (long_format) {
        term_out("drwxr-xr-x 2 root root 0 obj\n");
        term_out("drwxr-xr-x 2 root root 0 joint\n");
        term_out("-rw-r--r-- 1 root root 0 world\n");
        term_out("-rw-r--r-- 1 root root 0 camera\n");
        term_out("-rw-r--r-- 1 root root 0 spawner\n");
    } else {
        term_out("obj/\njoint/\nworld\ncamera\nspawner\n");
    }
}
void term_print_object_cat(int object_index) {
    rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
    term_printf("term_echo", "/obj/%d\n", object_index);
    term_printf(NULL, "  id:         %u\n", rigid_body->object_id);
    term_printf(NULL, "  type:       %s\n", term_object_type_name(rigid_body));
    term_printf(NULL, "  state:      %s\n", term_object_state_name(rigid_body));
    term_printf(NULL, "  mass:       %.4f\n", rigid_body->mass);
    term_printf(NULL, "  inv_mass:   %.4f\n", rigid_body->inverse_mass);
    if (rigid_body->type == object_sphere) {
        term_printf(NULL, "  radius:     %.4f\n", rigid_body->radius);
    } else {
        term_printf(NULL, "  half_ext:   (%.4f, %.4f, %.4f)\n", rigid_body->half_extensions.x,
                    rigid_body->half_extensions.y, rigid_body->half_extensions.z);
    }
    term_printf(NULL, "  position:   (%.4f, %.4f, %.4f)\n", rigid_body->position.x, rigid_body->position.y,
                rigid_body->position.z);
    term_printf(NULL, "  velocity:   (%.4f, %.4f, %.4f)\n", rigid_body->velocity.x, rigid_body->velocity.y,
                rigid_body->velocity.z);
    term_printf(NULL, "  angular_v:  (%.4f, %.4f, %.4f)\n", rigid_body->angular_velocity.x,
                rigid_body->angular_velocity.y, rigid_body->angular_velocity.z);
    term_printf(NULL, "  orient:     (%.4f, %.4f, %.4f, %.4f)\n", rigid_body->orientation.w, rigid_body->orientation.x,
                rigid_body->orientation.y, rigid_body->orientation.z);
    term_printf(NULL, "  friction:   s=%.3f k=%.3f\n", rigid_body->friction_static, rigid_body->friction_kinetic);
    term_printf(NULL, "  restitution:%.3f\n", rigid_body->restitution);
    term_printf(NULL, "  colour:     (%.2f, %.2f, %.2f)\n", rigid_body->colour.x, rigid_body->colour.y,
                rigid_body->colour.z);
    term_printf(NULL, "  sleep_time: %.2f\n", rigid_body->sleep_timer);
}
void term_print_joint_cat(int joint_index) {
    spring_joint *joint = &(physics_world_get_primary()->spring_joints)[joint_index];
    int index_a = scene_find_object_index_by_id(joint->object_id_a);
    int index_b = scene_find_object_index_by_id(joint->object_id_b);
    term_printf("term_echo", "/joint/%d\n", joint_index);
    term_printf(NULL, "  object_a:   %d (id=%u)\n", index_a, joint->object_id_a);
    term_printf(NULL, "  object_b:   %d (id=%u)\n", index_b, joint->object_id_b);
    term_printf(NULL, "  length:     %.4f\n", joint->equilibrium_length);
    term_printf(NULL, "  stiffness:  %.4f\n", joint->spring_constant);
    term_printf(NULL, "  damping:    %.4f\n", joint->damping_coefficient);
}
void term_print_world(void) {
    term_printf("term_echo", "/world\n");
    term_printf(NULL, "  version:            %s\n", a3_version_string);
    term_printf(NULL, "  mode:               %s\n", main_inputs.is_debug_mode_active ? "debug" : "game");
    term_printf(NULL, "  gravity:            %.4f\n", g_cfg.world.gravity);
    term_printf(NULL, "  drag:               %.4f\n", g_cfg.world.drag);
    term_printf(NULL, "  floor_friction_s:   %.4f\n", g_cfg.world.floor_friction_s);
    term_printf(NULL, "  floor_friction_k:   %.4f\n", g_cfg.world.floor_friction_k);
    term_printf(NULL, "  objects:            %d\n", (physics_world_get_primary()->body_count));
    term_printf(NULL, "  joints:             %d\n", (physics_world_get_primary()->spring_joint_count));
}
void term_print_camera(void) {
    term_printf("term_echo", "/camera\n");
    term_printf(NULL, "  position:   (%.4f, %.4f, %.4f)\n", main_camera_fov.position.x, main_camera_fov.position.y,
                main_camera_fov.position.z);
    term_printf(NULL, "  yaw:        %.4f\n", main_camera_fov.yaw);
    term_printf(NULL, "  pitch:      %.4f\n", main_camera_fov.pitch);
    term_printf(NULL, "  speed:      %.4f\n", main_camera_fov.movement_speed);
    term_printf(NULL, "  jump:       %.4f\n", g_cfg.camera.jump_height);
}
void term_print_spawner(void) {
    term_printf("term_echo", "/spawner\n");
    term_printf(NULL, "  type:        %s\n", term_spawn_type_name());
    term_printf(NULL, "  mass:        %.4f\n", g_cfg.spawner.mass);
    term_printf(NULL, "  radius:      %.4f\n", g_cfg.spawner.radius);
    term_printf(NULL, "  cube_mass:   %.4f\n", g_cfg.spawner.cube_mass);
    term_printf(NULL, "  cube_extent: %.4f\n", g_cfg.spawner.cube_extent);
    term_printf(NULL, "  cyl_mass:    %.4f\n", g_cfg.spawner.cyl_mass);
    term_printf(NULL, "  cyl_radius:  %.4f\n", g_cfg.spawner.cyl_radius);
    term_printf(NULL, "  cyl_half:    %.4f\n", g_cfg.spawner.cyl_half_length);
    term_printf(NULL, "  speed:       %.4f\n", g_cfg.spawner.speed);
    term_printf(NULL, "  friction_s:  %.4f\n", g_cfg.spawner.friction_s);
    term_printf(NULL, "  friction_k:  %.4f\n", g_cfg.spawner.friction_k);
}
/* ------------------------------------------------------------------ */
/* Scene mutation helpers                                              */
/* ------------------------------------------------------------------ */
int term_create_object(object_type spawn_type) {
    int created_index = -1;
    if (spawn_type == object_sphere) {
        vector3 spawn_position = vector3_addition(
            main_camera_fov.position, vector3_scaling(main_camera_fov.forward_vector, g_cfg.spawner.radius + 1.0f));
        created_index = scene_add_object(g_cfg.spawner.radius, g_cfg.spawner.mass, spawn_position);
    } else if (spawn_type == object_cylinder) {
        float bound = sqrtf(g_cfg.spawner.cyl_radius * g_cfg.spawner.cyl_radius +
                            g_cfg.spawner.cyl_half_length * g_cfg.spawner.cyl_half_length);
        vector3 spawn_position = vector3_addition(main_camera_fov.position,
                                                  vector3_scaling(main_camera_fov.forward_vector, bound + 1.0f));
        created_index = scene_add_cylinder(g_cfg.spawner.cyl_radius, g_cfg.spawner.cyl_half_length,
                                           g_cfg.spawner.cyl_mass, spawn_position);
    } else {
        vector3 spawn_position =
            vector3_addition(main_camera_fov.position,
                             vector3_scaling(main_camera_fov.forward_vector, g_cfg.spawner.cube_extent + 1.0f));
        created_index = scene_add_cube(
            spawn_position, (vector3){g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent},
            g_cfg.spawner.cube_mass);
    }
    if (created_index < 0) {
        term_err("mpe: touch: cannot create object (scene full?)\n");
        return -1;
    }
    (physics_world_get_primary()->bodies)[created_index].friction_static = g_cfg.spawner.friction_s;
    (physics_world_get_primary()->bodies)[created_index].friction_kinetic = g_cfg.spawner.friction_k;
    (physics_world_get_primary()->bodies)[created_index].velocity = vector3_zero();
    (physics_world_get_primary()->bodies)[created_index].angular_velocity = vector3_zero();
    (physics_world_get_primary()->bodies)[created_index].colour = (vector3){0.35f + 0.65f * ((float) ((created_index + 0) % 3) / 2.0f),
                                                    0.35f + 0.65f * ((float) ((created_index + 1) % 3) / 2.0f),
                                                    0.35f + 0.65f * ((float) ((created_index + 2) % 3) / 2.0f)};
    return created_index;
}
int term_duplicate_object(int source_index) {
    if ((source_index < 0) || (source_index >= (physics_world_get_primary()->body_count))) {
        return -1;
    }
    rigidbody snapshot = (physics_world_get_primary()->bodies)[source_index];
    vector3 copy_position = vector3_addition(snapshot.position, (vector3){1.0f, 0.0f, 0.0f});
    int created_index = -1;
    if (snapshot.type == object_sphere) {
        created_index = scene_add_object(snapshot.radius, snapshot.mass, copy_position);
    } else if (snapshot.type == object_cylinder) {
        /* Cylinders duplicate as cylinders (old code fell through to cube,
         * spawning the wrong shape with axle dims as box extents). */
        created_index =
            scene_add_cylinder(snapshot.radius, snapshot.cylinder_half_length, snapshot.mass, copy_position);
    } else {
        created_index = scene_add_cube(copy_position, snapshot.half_extensions, snapshot.mass);
    }
    if (created_index < 0) {
        term_err("mpe: cp: cannot duplicate object (scene full?)\n");
        return -1;
    }
    rigidbody *created_body = &(physics_world_get_primary()->bodies)[created_index];
    created_body->colour = snapshot.colour;
    created_body->restitution = snapshot.restitution;
    created_body->friction_static = snapshot.friction_static;
    created_body->friction_kinetic = snapshot.friction_kinetic;
    created_body->velocity = snapshot.velocity;
    created_body->angular_velocity = snapshot.angular_velocity;
    if (snapshot.static_state) {
        rigidbody_set_static(created_body, true);
    }
    return created_index;
}
void term_set_object_mass(int object_index, float new_mass) {
    if ((object_index < 0) || (object_index >= (physics_world_get_primary()->body_count))) {
        return;
    }
    rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
    if (new_mass < 0.0f) {
        new_mass = 0.0f;
    }
    if (new_mass <= 0.0f) {
        rigidbody_set_static(rigid_body, true);
    } else {
        if (rigid_body->static_state) {
            rigidbody_set_static(rigid_body, false);
        }
        rigid_body->mass = new_mass;
        rigid_body->inverse_mass = 1.0f / new_mass;
        if (rigid_body->type == object_sphere) {
            rigidbody_update_inertia_sphere(rigid_body);
        } else {
            rigidbody_update_inertia_cube(rigid_body);
        }
        rigidbody_wake(rigid_body);
    }
    contact_cache_clear(physics_world_get_primary());
}
void term_set_object_static(int object_index, bool make_static) {
    if ((object_index < 0) || (object_index >= (physics_world_get_primary()->body_count))) {
        return;
    }
    rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
    rigidbody_set_static(rigid_body, make_static);
    contact_cache_clear(physics_world_get_primary());
}
bool term_mode_is_static(const char *mode_text) {
    if (term_str_eq(mode_text, "static")) {
        return true;
    }
    if (term_str_eq(mode_text, "dynamic")) {
        return false;
    }
    if (term_str_eq(mode_text, "0")) {
        return true;
    }
    if (term_str_eq(mode_text, "000")) {
        return true;
    }
    if (term_str_eq(mode_text, "-x")) {
        return true;
    }
    if (term_str_eq(mode_text, "+x")) {
        return false;
    }
    char *endptr = NULL;
    long mode_bits = strtol(mode_text, &endptr, 8);
    if ((endptr != mode_text) && (*endptr == '\0')) {
        if (mode_bits == 0) {
            return true;
        }
        return false;
    }
    return false;
}
#else
/* term_obj.c — Object/joint model layer: inspect, list, create, mutate.
 * Split from debug_terminal.c (pure motion, no behaviour change).
 * Shared shell core lives in debug_terminal.c; see term_priv.h. */
#include "../mpe_engine.h"
#include "debug_terminal.h"
#include "term_priv.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
const char *term_object_type_name(rigidbody *rigid_body) {
    if (!rigid_body) {
        return "?";
    }
    if (rigid_body->type == object_sphere) {
        return "sph";
    }
    if (rigid_body->type == object_cylinder) {
        return "cyl";
    }
    return "cube";
}
/* Single source of truth for the spawn-gun type label (sphere/cube/cylinder).
 * All terminal + overlay-adjacent displays must use this, never inline. */
const char *term_spawn_type_name(void) {
    if (main_inputs.current_spawn_type == 0) {
        return "sphere";
    }
    if (main_inputs.current_spawn_type == 1) {
        return "cube";
    }
    return "cylinder";
}
const char *term_object_state_name(rigidbody *rigid_body) {
    if (rigid_body->static_state) {
        return "static";
    }
    if (rigid_body->is_sleeping) {
        return "sleep";
    }
    return "run";
}
const char *term_object_mode(rigidbody *rigid_body) {
    return rigid_body->static_state ? "-r--r--r--" : "-rw-r--r--";
}
void term_print_object_long(int object_index) {
    rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
    term_printf(NULL, "%s %4d %8.2f %-4s %-6s pos=(%.2f,%.2f,%.2f) |v|=%.3f id=%u\n", term_object_mode(rigid_body),
                object_index, rigid_body->mass, term_object_type_name(rigid_body), term_object_state_name(rigid_body),
                rigid_body->position.x, rigid_body->position.y, rigid_body->position.z,
                vector3_length(rigid_body->velocity), rigid_body->object_id);
}
void term_print_joint_long(int joint_index) {
    spring_joint *joint = &(physics_world_get_primary()->spring_joints)[joint_index];
    int index_a = scene_find_object_index_by_id(joint->object_id_a);
    int index_b = scene_find_object_index_by_id(joint->object_id_b);
    term_printf(NULL, "lrwxrwxrwx %4d [%d] -> [%d] len=%.2f k=%.1f d=%.1f\n", joint_index, index_a, index_b,
                joint->equilibrium_length, joint->spring_constant, joint->damping_coefficient);
}
void term_list_objects(bool long_format) {
    if ((physics_world_get_primary()->body_count) == 0) {
        term_dim("(no objects)\n");
        return;
    }
    if (long_format) {
        term_printf(NULL, "%-10s %4s %8s %-4s %-6s %s\n", "MODE", "PID", "MASS", "TYPE", "STATE", "INFO");
    }
    for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
        if (long_format) {
            term_print_object_long(object_index);
        } else {
            term_printf(NULL, "%d\n", object_index);
        }
    }
}
void term_list_joints(bool long_format) {
    int listed_count = 0;
    for (int joint_index = 0; joint_index < mpe_max_joints; joint_index++) {
        if (!(physics_world_get_primary()->spring_joints)[joint_index].is_active) {
            continue;
        }
        if (long_format) {
            term_print_joint_long(joint_index);
        } else {
            term_printf(NULL, "%d\n", joint_index);
        }
        listed_count++;
    }
    if (listed_count == 0) {
        term_dim("(no joints)\n");
    }
}
void term_list_root(bool long_format) {
    if (long_format) {
        term_out("drwxr-xr-x 2 root root 0 obj\n");
        term_out("drwxr-xr-x 2 root root 0 joint\n");
        term_out("-rw-r--r-- 1 root root 0 world\n");
        term_out("-rw-r--r-- 1 root root 0 camera\n");
        term_out("-rw-r--r-- 1 root root 0 spawner\n");
    } else {
        term_out("obj/\njoint/\nworld\ncamera\nspawner\n");
    }
}
void term_print_object_cat(int object_index) {
    rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
    term_printf("term_echo", "/obj/%d\n", object_index);
    term_printf(NULL, "  id:         %u\n", rigid_body->object_id);
    term_printf(NULL, "  type:       %s\n", term_object_type_name(rigid_body));
    term_printf(NULL, "  state:      %s\n", term_object_state_name(rigid_body));
    term_printf(NULL, "  mass:       %.4f\n", rigid_body->mass);
    term_printf(NULL, "  inv_mass:   %.4f\n", rigid_body->inverse_mass);
    if (rigid_body->type == object_sphere) {
        term_printf(NULL, "  radius:     %.4f\n", rigid_body->radius);
    } else {
        term_printf(NULL, "  half_ext:   (%.4f, %.4f, %.4f)\n", rigid_body->half_extensions.x,
                    rigid_body->half_extensions.y, rigid_body->half_extensions.z);
    }
    term_printf(NULL, "  position:   (%.4f, %.4f, %.4f)\n", rigid_body->position.x, rigid_body->position.y,
                rigid_body->position.z);
    term_printf(NULL, "  velocity:   (%.4f, %.4f, %.4f)\n", rigid_body->velocity.x, rigid_body->velocity.y,
                rigid_body->velocity.z);
    term_printf(NULL, "  angular_v:  (%.4f, %.4f, %.4f)\n", rigid_body->angular_velocity.x,
                rigid_body->angular_velocity.y, rigid_body->angular_velocity.z);
    term_printf(NULL, "  orient:     (%.4f, %.4f, %.4f, %.4f)\n", rigid_body->orientation.w, rigid_body->orientation.x,
                rigid_body->orientation.y, rigid_body->orientation.z);
    term_printf(NULL, "  friction:   s=%.3f k=%.3f\n", rigid_body->friction_static, rigid_body->friction_kinetic);
    term_printf(NULL, "  restitution:%.3f\n", rigid_body->restitution);
    term_printf(NULL, "  colour:     (%.2f, %.2f, %.2f)\n", rigid_body->colour.x, rigid_body->colour.y,
                rigid_body->colour.z);
    term_printf(NULL, "  sleep_time: %.2f\n", rigid_body->sleep_timer);
}
void term_print_joint_cat(int joint_index) {
    spring_joint *joint = &(physics_world_get_primary()->spring_joints)[joint_index];
    int index_a = scene_find_object_index_by_id(joint->object_id_a);
    int index_b = scene_find_object_index_by_id(joint->object_id_b);
    term_printf("term_echo", "/joint/%d\n", joint_index);
    term_printf(NULL, "  object_a:   %d (id=%u)\n", index_a, joint->object_id_a);
    term_printf(NULL, "  object_b:   %d (id=%u)\n", index_b, joint->object_id_b);
    term_printf(NULL, "  length:     %.4f\n", joint->equilibrium_length);
    term_printf(NULL, "  stiffness:  %.4f\n", joint->spring_constant);
    term_printf(NULL, "  damping:    %.4f\n", joint->damping_coefficient);
}
void term_print_world(void) {
    term_printf("term_echo", "/world\n");
    term_printf(NULL, "  version:            %s\n", a3_version_string);
    term_printf(NULL, "  mode:               %s\n", main_inputs.is_debug_mode_active ? "debug" : "game");
    term_printf(NULL, "  gravity:            %.4f\n", g_cfg.world.gravity);
    term_printf(NULL, "  drag:               %.4f\n", g_cfg.world.drag);
    term_printf(NULL, "  floor_friction_s:   %.4f\n", g_cfg.world.floor_friction_s);
    term_printf(NULL, "  floor_friction_k:   %.4f\n", g_cfg.world.floor_friction_k);
    term_printf(NULL, "  objects:            %d\n", (physics_world_get_primary()->body_count));
    term_printf(NULL, "  joints:             %d\n", (physics_world_get_primary()->spring_joint_count));
}
void term_print_camera(void) {
    term_printf("term_echo", "/camera\n");
    term_printf(NULL, "  position:   (%.4f, %.4f, %.4f)\n", main_camera_fov.position.x, main_camera_fov.position.y,
                main_camera_fov.position.z);
    term_printf(NULL, "  yaw:        %.4f\n", main_camera_fov.yaw);
    term_printf(NULL, "  pitch:      %.4f\n", main_camera_fov.pitch);
    term_printf(NULL, "  speed:      %.4f\n", main_camera_fov.movement_speed);
    term_printf(NULL, "  jump:       %.4f\n", g_cfg.camera.jump_height);
}
void term_print_spawner(void) {
    term_printf("term_echo", "/spawner\n");
    term_printf(NULL, "  type:        %s\n", term_spawn_type_name());
    term_printf(NULL, "  mass:        %.4f\n", g_cfg.spawner.mass);
    term_printf(NULL, "  radius:      %.4f\n", g_cfg.spawner.radius);
    term_printf(NULL, "  cube_mass:   %.4f\n", g_cfg.spawner.cube_mass);
    term_printf(NULL, "  cube_extent: %.4f\n", g_cfg.spawner.cube_extent);
    term_printf(NULL, "  cyl_mass:    %.4f\n", g_cfg.spawner.cyl_mass);
    term_printf(NULL, "  cyl_radius:  %.4f\n", g_cfg.spawner.cyl_radius);
    term_printf(NULL, "  cyl_half:    %.4f\n", g_cfg.spawner.cyl_half_length);
    term_printf(NULL, "  speed:       %.4f\n", g_cfg.spawner.speed);
    term_printf(NULL, "  friction_s:  %.4f\n", g_cfg.spawner.friction_s);
    term_printf(NULL, "  friction_k:  %.4f\n", g_cfg.spawner.friction_k);
}
/* ------------------------------------------------------------------ */
/* Scene mutation helpers                                              */
/* ------------------------------------------------------------------ */
int term_create_object(object_type spawn_type) {
    int created_index = -1;
    if (spawn_type == object_sphere) {
        vector3 spawn_position = vector3_addition(
            main_camera_fov.position, vector3_scaling(main_camera_fov.forward_vector, g_cfg.spawner.radius + 1.0f));
        created_index = scene_add_object(g_cfg.spawner.radius, g_cfg.spawner.mass, spawn_position);
    } else if (spawn_type == object_cylinder) {
        float bound = sqrtf(g_cfg.spawner.cyl_radius * g_cfg.spawner.cyl_radius +
                            g_cfg.spawner.cyl_half_length * g_cfg.spawner.cyl_half_length);
        vector3 spawn_position = vector3_addition(main_camera_fov.position,
                                                  vector3_scaling(main_camera_fov.forward_vector, bound + 1.0f));
        created_index = scene_add_cylinder(g_cfg.spawner.cyl_radius, g_cfg.spawner.cyl_half_length,
                                           g_cfg.spawner.cyl_mass, spawn_position);
    } else {
        vector3 spawn_position =
            vector3_addition(main_camera_fov.position,
                             vector3_scaling(main_camera_fov.forward_vector, g_cfg.spawner.cube_extent + 1.0f));
        created_index = scene_add_cube(
            spawn_position, (vector3){g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent},
            g_cfg.spawner.cube_mass);
    }
    if (created_index < 0) {
        term_err("mpe: touch: cannot create object (scene full?)\n");
        return -1;
    }
    (physics_world_get_primary()->bodies)[created_index].friction_static = g_cfg.spawner.friction_s;
    (physics_world_get_primary()->bodies)[created_index].friction_kinetic = g_cfg.spawner.friction_k;
    (physics_world_get_primary()->bodies)[created_index].velocity = vector3_zero();
    (physics_world_get_primary()->bodies)[created_index].angular_velocity = vector3_zero();
    (physics_world_get_primary()->bodies)[created_index].colour = (vector3){0.35f + 0.65f * ((float) ((created_index + 0) % 3) / 2.0f),
                                                    0.35f + 0.65f * ((float) ((created_index + 1) % 3) / 2.0f),
                                                    0.35f + 0.65f * ((float) ((created_index + 2) % 3) / 2.0f)};
    return created_index;
}
int term_duplicate_object(int source_index) {
    if ((source_index < 0) || (source_index >= (physics_world_get_primary()->body_count))) {
        return -1;
    }
    rigidbody snapshot = (physics_world_get_primary()->bodies)[source_index];
    vector3 copy_position = vector3_addition(snapshot.position, (vector3){1.0f, 0.0f, 0.0f});
    int created_index = -1;
    if (snapshot.type == object_sphere) {
        created_index = scene_add_object(snapshot.radius, snapshot.mass, copy_position);
    } else if (snapshot.type == object_cylinder) {
        /* Cylinders duplicate as cylinders (old code fell through to cube,
         * spawning the wrong shape with axle dims as box extents). */
        created_index =
            scene_add_cylinder(snapshot.radius, snapshot.cylinder_half_length, snapshot.mass, copy_position);
    } else {
        created_index = scene_add_cube(copy_position, snapshot.half_extensions, snapshot.mass);
    }
    if (created_index < 0) {
        term_err("mpe: cp: cannot duplicate object (scene full?)\n");
        return -1;
    }
    rigidbody *created_body = &(physics_world_get_primary()->bodies)[created_index];
    created_body->colour = snapshot.colour;
    created_body->restitution = snapshot.restitution;
    created_body->friction_static = snapshot.friction_static;
    created_body->friction_kinetic = snapshot.friction_kinetic;
    created_body->velocity = snapshot.velocity;
    created_body->angular_velocity = snapshot.angular_velocity;
    if (snapshot.static_state) {
        rigidbody_set_static(created_body, true);
    }
    return created_index;
}
void term_set_object_mass(int object_index, float new_mass) {
    if ((object_index < 0) || (object_index >= (physics_world_get_primary()->body_count))) {
        return;
    }
    rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
    if (new_mass < 0.0f) {
        new_mass = 0.0f;
    }
    if (new_mass <= 0.0f) {
        rigidbody_set_static(rigid_body, true);
    } else {
        if (rigid_body->static_state) {
            rigidbody_set_static(rigid_body, false);
        }
        rigid_body->mass = new_mass;
        rigid_body->inverse_mass = 1.0f / new_mass;
        if (rigid_body->type == object_sphere) {
            rigidbody_update_inertia_sphere(rigid_body);
        } else {
            rigidbody_update_inertia_cube(rigid_body);
        }
        rigidbody_wake(rigid_body);
    }
    contact_cache_clear(physics_world_get_primary());
}
void term_set_object_static(int object_index, bool make_static) {
    if ((object_index < 0) || (object_index >= (physics_world_get_primary()->body_count))) {
        return;
    }
    rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
    rigidbody_set_static(rigid_body, make_static);
    contact_cache_clear(physics_world_get_primary());
}
bool term_mode_is_static(const char *mode_text) {
    if (term_str_eq(mode_text, "static")) {
        return true;
    }
    if (term_str_eq(mode_text, "dynamic")) {
        return false;
    }
    if (term_str_eq(mode_text, "0")) {
        return true;
    }
    if (term_str_eq(mode_text, "000")) {
        return true;
    }
    if (term_str_eq(mode_text, "-x")) {
        return true;
    }
    if (term_str_eq(mode_text, "+x")) {
        return false;
    }
    char *endptr = NULL;
    long mode_bits = strtol(mode_text, &endptr, 8);
    if ((endptr != mode_text) && (*endptr == '\0')) {
        if (mode_bits == 0) {
            return true;
        }
        return false;
    }
    return false;
}
#endif /* MPE_GTK4 */

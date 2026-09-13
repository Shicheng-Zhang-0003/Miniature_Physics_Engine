/* term_fs.c — Filesystem commands: help..kill.
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
static uint32_t term_id_buffer[mpe_max_bodies];
void cmd_help(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_dim("POSIX-style MPE debug shell. Mutating commands require debug mode.\n");
    for (size_t command_index = 0; command_index < terminal_command_count; command_index++) {
        term_printf(NULL, "  %-8s %-44s %s\n", terminal_commands[command_index].name,
                    terminal_commands[command_index].usage, terminal_commands[command_index].description);
    }
}
void cmd_man(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: man <command>\n");
        return;
    }
    for (size_t command_index = 0; command_index < terminal_command_count; command_index++) {
        if (term_str_eq(argv[1], terminal_commands[command_index].name)) {
            term_printf("term_echo", "NAME\n");
            term_printf(NULL, "    %s - %s\n", terminal_commands[command_index].name,
                        terminal_commands[command_index].description);
            term_printf("term_echo", "SYNOPSIS\n");
            term_printf(NULL, "    %s\n", terminal_commands[command_index].usage);
            return;
        }
    }
    term_printf("term_err", "mpe: no manual entry for %s\n", argv[1]);
}
void cmd_clear(int argc, char **argv) {
    (void) argc;
    (void) argv;
    if (terminal_output_buffer) {
        gtk_text_buffer_set_text(terminal_output_buffer, "", -1);
    }
}
void cmd_history(int argc, char **argv) {
    (void) argc;
    (void) argv;
    if (term_history_count == 0) {
        term_dim("(no history)\n");
        return;
    }
    for (int history_index = term_history_count - 1; history_index >= 0; history_index--) {
        term_printf(NULL, "%4d %s\n", term_history_count - history_index, term_history[history_index]);
    }
}
void cmd_pwd(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_printf(NULL, "%s\n", term_cwd);
}
void cmd_cd(int argc, char **argv) {
    const char *target = (argc > 1) ? argv[1] : "/";
    if (term_str_eq(target, "~") || term_str_eq(target, "/") || term_str_eq(target, "..")) {
        snprintf(term_cwd, sizeof(term_cwd), "/");
    } else if (strstr(target, "joint")) {
        snprintf(term_cwd, sizeof(term_cwd), "/joint");
    } else if (strstr(target, "obj")) {
        snprintf(term_cwd, sizeof(term_cwd), "/obj");
    } else {
        term_printf("term_err", "mpe: cd: %s: No such directory\n", target);
        return;
    }
    term_update_prompt();
}
void term_ls_internal(bool long_format, int argc, char **argv) {
    const char *path = term_cwd;
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        if (argv[argument_index][0] != '-') {
            path = argv[argument_index];
            break;
        }
    }
    if (term_str_eq(path, "/")) {
        term_list_root(long_format);
    } else if (strstr(path, "joint")) {
        term_list_joints(long_format);
    } else if (strstr(path, "obj")) {
        term_list_objects(long_format);
    } else if (strstr(term_cwd, "joint")) {
        term_list_joints(long_format);
    } else if (strstr(term_cwd, "obj")) {
        term_list_objects(long_format);
    } else {
        term_list_root(long_format);
    }
}
void cmd_ls(int argc, char **argv) {
    bool long_format = false;
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        if ((argv[argument_index][0] == '-') && (strstr(argv[argument_index], "l"))) {
            long_format = true;
        }
    }
    term_ls_internal(long_format, argc, argv);
}
void cmd_ll(int argc, char **argv) {
    term_ls_internal(true, argc, argv);
}
void cmd_cat(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: cat <path...>\n");
        return;
    }
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        const char *target = argv[argument_index];
        if (strstr(target, "world")) {
            term_print_world();
        } else if (strstr(target, "camera")) {
            term_print_camera();
        } else if (strstr(target, "spawner")) {
            term_print_spawner();
        } else if (term_classify_token(target) == term_target_joint) {
            int joint_index = term_joint_from_token(target);
            if (joint_index >= 0) {
                term_print_joint_cat(joint_index);
            } else {
                term_printf("term_err", "mpe: %s: No such joint\n", target);
            }
        } else {
            int object_index = term_object_from_token(target);
            if (object_index >= 0) {
                term_print_object_cat(object_index);
            } else {
                term_printf("term_err", "mpe: %s: No such object\n", target);
            }
        }
    }
}
void cmd_touch(int argc, char **argv) {
    if (argc < 2) {
        int created_index = term_create_object(object_sphere);
        if (created_index >= 0) {
            term_printf("term_ok", "/obj/%d\n", created_index);
        }
        return;
    }
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        if (argv[argument_index][0] == '-') {
            continue;
        }
    if ((argc > 1) && (strstr(argv[1], "robot"))) {
        term_err("mpe: touch: unknown type 'robot' (types: sph, cube)\n");
        return;
    }

        object_type spawn_type = object_sphere;
        if (strstr(argv[argument_index], "cube")) {
            spawn_type = object_cube;
        }
        int created_index = term_create_object(spawn_type);
        if (created_index >= 0) {
            term_printf("term_ok", "/obj/%d\n", created_index);
        }
    }
}
void cmd_cp(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: cp <object> [dest]\n");
        return;
    }
    int source_index = term_require_object(argv[1]);
    if (source_index < 0) {
        return;
    }
    int created_index = term_duplicate_object(source_index);
    if (created_index >= 0) {
        term_printf("term_ok", "/obj/%d\n", created_index);
    }
}
void cmd_rm(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: rm [-rf] <path...>\n");
        return;
    }
    int delete_count = 0;
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        if (argv[argument_index][0] == '-') {
            continue;
        }
        const char *target = argv[argument_index];
        bool all_targets = term_is_all_token(target);
        term_target_kind kind = term_classify_token(target);
        if (all_targets) {
            if (kind == term_target_joint) {
                int removed_count = 0;
                for (int joint_index = 0; joint_index < mpe_max_joints; joint_index++) {
                    if ((physics_world_get_primary()->spring_joints)[joint_index].is_active) {
                        remove_joint(physics_world_get_primary(), joint_index);
                        removed_count++;
                    }
                }
                term_printf("term_ok", "removed %d joint(s)\n", removed_count);
            } else {
                scene_clear();
                clear_selection();
                contact_cache_clear(physics_world_get_primary());
                main_inputs.object_menu_level = 0;
                main_inputs.marked_joint_object_index = -1;
                main_inputs.is_menu_open = false;
                main_inputs.spawner_menu_level = 0;
                main_inputs.velocity_menu_level = 0;
                delete_count = 0;
                term_ok("removed all objects\n");
            }
            continue;
        }
        if (kind == term_target_joint) {
            int joint_index = term_joint_from_token(target);
            if (joint_index >= 0) {
                remove_joint(physics_world_get_primary(), joint_index);
                term_printf("term_ok", "removed /joint/%d\n", joint_index);
            } else {
                term_printf("term_err", "mpe: %s: No such joint\n", target);
            }
        } else {
            int object_index = term_object_from_token(target);
            if (object_index >= 0) {
                if (delete_count < mpe_max_bodies) {
                    term_id_buffer[delete_count++] = (physics_world_get_primary()->bodies)[object_index].object_id;
                }
            } else {
                term_printf("term_err", "mpe: %s: No such object\n", target);
            }
        }
    }
    if (delete_count > 0) {
        for (int delete_index = 0; delete_index < delete_count; delete_index++) {
            int object_index = scene_find_object_index_by_id(term_id_buffer[delete_index]);
            if (object_index >= 0) {
                scene_remove_object_by_index(object_index);
            }
        }
        term_printf("term_ok", "removed %d object(s)\n", delete_count);
    }
}
void cmd_mv(int argc, char **argv) {
    if (argc < 3) {
        term_err("usage: mv <object> /pos/x/y/z | /vel/dx/dy/dz\n");
        return;
    }
    int object_index = term_require_object(argv[1]);
    if (object_index < 0) {
        return;
    }
    rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
    float x = 0.0f, y = 0.0f, z = 0.0f;
    int movement_kind = term_parse_movement_destination(argv[2], &x, &y, &z);
    if (movement_kind == 1) {
        rigid_body->position = (vector3){x, y, z};
        rigidbody_wake(rigid_body);
        term_printf("term_ok", "/obj/%d moved to (%.2f, %.2f, %.2f)\n", object_index, x, y, z);
    } else if (movement_kind == 2) {
        rigid_body->velocity = vector3_addition(rigid_body->velocity, (vector3){x, y, z});
        rigidbody_wake(rigid_body);
        term_printf("term_ok", "/obj/%d impulse (%.2f, %.2f, %.2f)\n", object_index, x, y, z);
    } else {
        term_err("usage: mv <object> /pos/x/y/z | /vel/dx/dy/dz\n");
    }
}
void cmd_ln(int argc, char **argv) {
    bool soft_joint = false;
    int argument_index = 1;
    if ((argc > 1) && (term_str_eq(argv[1], "-s"))) {
        soft_joint = true;
        argument_index = 2;
    }
    if (argc < argument_index + 2) {
        term_err("usage: ln [-s] <object> <object>\n");
        return;
    }
    int index_a = term_require_object(argv[argument_index]);
    if (index_a < 0) {
        return;
    }
    int index_b = term_require_object(argv[argument_index + 1]);
    if (index_b < 0) {
        return;
    }
    if (index_a == index_b) {
        term_err("mpe: ln: cannot link an object to itself\n");
        return;
    }
    float rest_length =
        vector3_length(vector3_subtraction((physics_world_get_primary()->bodies)[index_b].position, (physics_world_get_primary()->bodies)[index_a].position));
    float spring_constant = soft_joint ? g_cfg.joints.soft_spring_k : g_cfg.joints.default_spring_k;
    float damping_coefficient = soft_joint ? g_cfg.joints.soft_damping : g_cfg.joints.default_damping;
    int joint_index = add_joint(physics_world_get_primary(), index_a, index_b, rest_length, spring_constant, damping_coefficient);
    if (joint_index < 0) {
        term_err("mpe: ln: cannot create joint\n");
        return;
    }
    term_printf("term_ok", "/joint/%d -> /obj/%d -> /obj/%d\n", joint_index, index_a, index_b);
}
void cmd_unlink(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: unlink <path>\n");
        return;
    }
    const char *target = argv[1];
    if (term_classify_token(target) == term_target_joint) {
        int joint_index = term_require_joint(target);
        if (joint_index >= 0) {
            remove_joint(physics_world_get_primary(), joint_index);
            term_printf("term_ok", "removed /joint/%d\n", joint_index);
        }
    } else {
        int object_index = term_require_object(target);
        if (object_index >= 0) {
            scene_remove_object_by_index(object_index);
            term_printf("term_ok", "removed /obj/%d\n", object_index);
        }
    }
}
void cmd_chmod(int argc, char **argv) {
    if (argc < 3) {
        term_err("usage: chmod static|dynamic|mode <object...>\n");
        return;
    }
    bool make_static = term_mode_is_static(argv[1]);
    for (int argument_index = 2; argument_index < argc; argument_index++) {
        int object_index = term_require_object(argv[argument_index]);
        if (object_index < 0) {
            continue;
        }
        term_set_object_static(object_index, make_static);
        term_printf("term_ok", "/obj/%d -> %s\n", object_index, make_static ? "static" : "dynamic");
    }
}
void cmd_chown(int argc, char **argv) {
    if (argc < 3) {
        term_err("usage: chown <mass> <object...>\n");
        return;
    }
    float new_mass = 0.0f;
    if (!term_parse_float(argv[1], &new_mass)) {
        term_printf("term_err", "mpe: chown: invalid mass '%s'\n", argv[1]);
        return;
    }
    for (int argument_index = 2; argument_index < argc; argument_index++) {
        int object_index = term_require_object(argv[argument_index]);
        if (object_index < 0) {
            continue;
        }
        term_set_object_mass(object_index, new_mass);
        term_printf("term_ok", "/obj/%d mass=%.3f\n", object_index, new_mass);
    }
}
void cmd_kill(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: kill [-STOP|-CONT|-9|-TERM] <object...>\n");
        return;
    }
    enum { kill_term, kill_stop, kill_cont };
    int kill_action = kill_term;
    int argument_index = 1;
    if ((argc > 1) && ((argv[1][0] == '-') || (g_str_has_prefix(argv[1], "SIG")))) {
        const char *signal_text = argv[1];
        if (signal_text[0] == '-') {
            signal_text++;
        }
        if (g_str_has_prefix(signal_text, "SIG")) {
            signal_text += 3;
        }
        if (term_str_eq(signal_text, "STOP") || term_str_eq(signal_text, "19")) {
            kill_action = kill_stop;
        } else if (term_str_eq(signal_text, "CONT") || term_str_eq(signal_text, "18")) {
            kill_action = kill_cont;
        } else {
            kill_action = kill_term;
        }
        argument_index = 2;
    }
    int delete_count = 0;
    for (; argument_index < argc; argument_index++) {
        const char *target = argv[argument_index];
        if (term_is_all_token(target)) {
            if (kill_action == kill_stop) {
                for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
                    (physics_world_get_primary()->bodies)[object_index].velocity = vector3_zero();
                    (physics_world_get_primary()->bodies)[object_index].angular_velocity = vector3_zero();
                    (physics_world_get_primary()->bodies)[object_index].is_sleeping = true;
                    (physics_world_get_primary()->bodies)[object_index].sleep_timer = 2.0f;
                }
                term_ok("stopped all objects\n");
            } else if (kill_action == kill_cont) {
                for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
                    rigidbody_wake(&(physics_world_get_primary()->bodies)[object_index]);
                }
                term_ok("continued all objects\n");
            } else {
                scene_clear();
                clear_selection();
                contact_cache_clear(physics_world_get_primary());
                main_inputs.object_menu_level = 0;
                main_inputs.marked_joint_object_index = -1;
                main_inputs.is_menu_open = false;
                main_inputs.spawner_menu_level = 0;
                main_inputs.velocity_menu_level = 0;
                delete_count = 0;
                term_ok("killed all objects\n");
            }
            continue;
        }
        int object_index = term_object_from_token(target);
        if (object_index < 0) {
            term_printf("term_err", "mpe: %s: No such object\n", target);
            continue;
        }
        rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
        if (kill_action == kill_stop) {
            rigid_body->velocity = vector3_zero();
            rigid_body->angular_velocity = vector3_zero();
            rigid_body->is_sleeping = true;
            rigid_body->sleep_timer = 2.0f;
            term_printf("term_ok", "stopped /obj/%d\n", object_index);
        } else if (kill_action == kill_cont) {
            rigidbody_wake(rigid_body);
            term_printf("term_ok", "continued /obj/%d\n", object_index);
        } else {
            if (delete_count < mpe_max_bodies) {
                term_id_buffer[delete_count++] = rigid_body->object_id;
            }
        }
    }
    if (delete_count > 0) {
        for (int delete_index = 0; delete_index < delete_count; delete_index++) {
            int object_index = scene_find_object_index_by_id(term_id_buffer[delete_index]);
            if (object_index >= 0) {
                scene_remove_object_by_index(object_index);
            }
        }
        term_printf("term_ok", "killed %d object(s)\n", delete_count);
    }
}
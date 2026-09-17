/* GTK4-PREP: GTK3 preserved under #else; GTK4 full port follows. */
#ifdef MPE_GTK4
/* term_sys.c — System/info commands: ps..time.
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
void cmd_ps(int argc, char **argv) {
    bool detailed = false;
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        if (strstr(argv[argument_index], "aux") || strstr(argv[argument_index], "-a")) {
            detailed = true;
        }
    }
    if ((physics_world_get_primary()->body_count) == 0) {
        term_dim("(no objects)\n");
        return;
    }
    if (detailed) {
        term_printf(NULL, "%4s %6s %-4s %-6s %8s %8s %s\n", "PID", "ID", "TYPE", "STATE", "MASS", "SPEED", "POSITION");
        for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
            rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
            term_printf(NULL, "%4d %6u %-4s %-6s %8.2f %8.3f (%.2f,%.2f,%.2f)\n", object_index, rigid_body->object_id,
                        term_object_type_name(rigid_body), term_object_state_name(rigid_body), rigid_body->mass,
                        vector3_length(rigid_body->velocity), rigid_body->position.x, rigid_body->position.y,
                        rigid_body->position.z);
        }
    } else {
        term_printf(NULL, "%4s %6s %-4s %-6s %8s\n", "PID", "ID", "TYPE", "STATE", "MASS");
        for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
            rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
            term_printf(NULL, "%4d %6u %-4s %-6s %8.2f\n", object_index, rigid_body->object_id,
                        term_object_type_name(rigid_body), term_object_state_name(rigid_body), rigid_body->mass);
        }
    }
}
void cmd_top(int argc, char **argv) {
    int limit = 10;
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        if (term_str_eq(argv[argument_index], "-n") && (argument_index + 1 < argc)) {
            float parsed_limit = 0.0f;
            if (term_parse_float(argv[argument_index + 1], &parsed_limit)) {
                limit = (int) parsed_limit;
            }
        }
    }
    if (limit < 1) {
        limit = 1;
    }
    if (limit > 16) {
        limit = 16;
    }
    if ((physics_world_get_primary()->body_count) == 0) {
        term_dim("(no objects)\n");
        return;
    }
    int top_indices[16];
    float top_speeds[16];
    for (int slot_index = 0; slot_index < limit; slot_index++) {
        top_indices[slot_index] = -1;
        top_speeds[slot_index] = -1.0f;
    }
    for (int slot_index = 0; slot_index < limit; slot_index++) {
        int best_index = -1;
        float best_speed = -1.0f;
        for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
            bool already_listed = false;
            for (int previous_slot = 0; previous_slot < slot_index; previous_slot++) {
                if (top_indices[previous_slot] == object_index) {
                    already_listed = true;
                    break;
                }
            }
            if (already_listed) {
                continue;
            }
            float object_speed = vector3_length((physics_world_get_primary()->bodies)[object_index].velocity);
            if (object_speed > best_speed) {
                best_speed = object_speed;
                best_index = object_index;
            }
        }
        if (best_index < 0) {
            break;
        }
        top_indices[slot_index] = best_index;
        top_speeds[slot_index] = best_speed;
    }
    term_printf(NULL, "%4s %-4s %-6s %8s %s\n", "PID", "TYPE", "STATE", "SPEED", "POSITION");
    for (int slot_index = 0; slot_index < limit; slot_index++) {
        if (top_indices[slot_index] < 0) {
            break;
        }
        rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[top_indices[slot_index]];
        term_printf(NULL, "%4d %-4s %-6s %8.3f (%.2f,%.2f,%.2f)\n", top_indices[slot_index],
                    term_object_type_name(rigid_body), term_object_state_name(rigid_body), top_speeds[slot_index],
                    rigid_body->position.x, rigid_body->position.y, rigid_body->position.z);
    }
}
void cmd_df(int argc, char **argv) {
    (void) argc;
    (void) argv;
    int object_capacity_value = ((physics_world_get_primary()->body_capacity) > 0) ? (physics_world_get_primary()->body_capacity) : mpe_max_bodies;
    int joint_capacity_value = mpe_max_joints;
    int object_percent = (object_capacity_value > 0) ? ((physics_world_get_primary()->body_count) * 100 / object_capacity_value) : 0;
    int joint_percent = (joint_capacity_value > 0) ? ((physics_world_get_primary()->spring_joint_count) * 100 / joint_capacity_value) : 0;
    term_printf(NULL, "Filesystem     Size   Used  Avail Use%% Mounted on\n");
    term_printf(NULL, "objects       %6d %6d %6d %3d%% /obj\n", object_capacity_value, (physics_world_get_primary()->body_count),
                object_capacity_value - (physics_world_get_primary()->body_count), object_percent);
    term_printf(NULL, "joints        %6d %6d %6d %3d%% /joint\n", joint_capacity_value, (physics_world_get_primary()->spring_joint_count),
                joint_capacity_value - (physics_world_get_primary()->spring_joint_count), joint_percent);
}
void cmd_du(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: du <path...>\n");
        return;
    }
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        const char *target = argv[argument_index];
        if (term_classify_token(target) == term_target_joint) {
            int joint_index = term_joint_from_token(target);
            if (joint_index >= 0) {
                spring_joint *joint = &(physics_world_get_primary()->spring_joints)[joint_index];
                term_printf(NULL, "/joint/%d len=%.2f k=%.1f d=%.1f\n", joint_index, joint->equilibrium_length,
                            joint->spring_constant, joint->damping_coefficient);
            } else {
                term_printf("term_err", "mpe: %s: No such joint\n", target);
            }
        } else {
            int object_index = term_object_from_token(target);
            if (object_index >= 0) {
                rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
                float size_value = (rigid_body->type == object_sphere) ? rigid_body->radius
                                                                       : vector3_length(rigid_body->half_extensions);
                term_printf(NULL, "/obj/%d mass=%.2f size=%.2f\n", object_index, rigid_body->mass, size_value);
            } else {
                term_printf("term_err", "mpe: %s: No such object\n", target);
            }
        }
    }
}
void cmd_uname(int argc, char **argv) {
    /* MPE_TASK_V15R2_UNAME_EXPANDED */
    bool print_all = false, print_sys = false, print_rel = false;
    bool print_mach = false, print_os = false;
    for (int i = 1; i < argc; i++) {
        if (term_str_eq(argv[i], "-a")) {
            print_all = true;
        } else if (term_str_eq(argv[i], "-s")) {
            print_sys = true;
        } else if (term_str_eq(argv[i], "-r")) {
            print_rel = true;
        } else if (term_str_eq(argv[i], "-m")) {
            print_mach = true;
        } else if (term_str_eq(argv[i], "-o")) {
            print_os = true;
        } else if (argv[i][0] == '-') {
            print_all = true;
        }
    }
    if (print_all) {
        term_printf(NULL, "MPE %s mpe-engine x86_64 POSIX-like/GTK3/OpenGL3.3 MPE\n", a3_version_string);
    } else if (print_sys) {
        term_out("MPE\n");
    } else if (print_rel) {
        term_printf(NULL, "%s\n", a3_version_string);
    } else if (print_mach) {
        term_out("x86_64\n");
    } else if (print_os) {
        term_out("POSIX-like/GTK3/OpenGL3.3\n");
    } else {
        term_printf(NULL, "MPE %s\n", a3_version_string);
    }
}
void cmd_whoami(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_out("root\n");
}
void cmd_date(int argc, char **argv) {
    (void) argc;
    (void) argv;
    time_t current_time = time(NULL);
    struct tm *local_time = localtime(&current_time);
    char time_buffer[128];
    strftime(time_buffer, sizeof(time_buffer), "%a %Y-%m-%d %H:%M:%S %Z", local_time);
    term_printf(NULL, "%s\n", time_buffer);
}
void cmd_echo(int argc, char **argv) {
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        term_out(argv[argument_index]);
        if (argument_index + 1 < argc) {
            term_out(" ");
        }
    }
    term_out("\n");
}
/* MPE_TASK_38_REGISTRY_ENV_BEGIN */
void cmd_env(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_printf(NULL, "CAMERA_SPEED=%.4f\n", main_camera_fov.movement_speed);
    int current_category = -1;
    for (size_t i = 0; i < g_registry_count; i++) {
        if ((int) g_registry[i].category != current_category) {
            current_category = (int) g_registry[i].category;
            term_printf("term_echo", "[%s]\n", mpe_config_category_name((param_category) current_category));
        }
        if (g_registry[i].type == p_int) {
            term_printf(NULL, "  %s = %d\n", g_registry[i].key, *(int *) g_registry[i].storage);
        } else if (g_registry[i].type == p_bool) {
            term_printf(NULL, "  %s = %s\n", g_registry[i].key, (*(bool *) g_registry[i].storage) ? "true" : "false");
        } else {
            term_printf(NULL, "  %s = %.4f\n", g_registry[i].key, *(float *) g_registry[i].storage);
        }
    }
}
/* MPE_TASK_38_REGISTRY_ENV_END */
/* MPE_TASK_38_REGISTRY_EXPORT_BEGIN */
void cmd_export(int argc, char **argv) {
    if (argc < 2) {
        cmd_env(argc, argv);
        return;
    }
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        char **parts = g_strsplit(argv[argument_index], "=", 2);
        if ((!parts[0]) || (!parts[1])) {
            term_printf("term_err", "mpe: export: usage: export KEY=value\n");
            g_strfreev(parts);
            continue;
        }
        const char *variable_name = parts[0];
        float variable_value = 0.0f;
        if (!term_parse_float(parts[1], &variable_value)) {
            term_printf("term_err", "mpe: export: invalid value '%s'\n", parts[1]);
            g_strfreev(parts);
            continue;
        }
        if (term_str_eq(variable_name, "CAMERA_SPEED")) {
            main_camera_fov.movement_speed = variable_value;
            term_printf("term_ok", "CAMERA_SPEED=%.4f\n", variable_value);
        } else {
            const mpe_param *param = mpe_config_find(variable_name);
            if (param) {
                bool clamped = !mpe_config_set_float(variable_name, variable_value);
                if (param->type == p_int) {
                    term_printf("term_ok", "%s = %d%s\n", variable_name, *(int *) param->storage,
                                clamped ? " (clamped)" : "");
                } else if (param->type == p_bool) {
                    term_printf("term_ok", "%s = %s%s\n", variable_name, (*(bool *) param->storage) ? "true" : "false",
                                clamped ? " (clamped)" : "");
                } else {
                    term_printf("term_ok", "%s = %.4f%s\n", variable_name, *(float *) param->storage,
                                clamped ? " (clamped)" : "");
                }
            } else {
                term_printf("term_err", "mpe: export: %s: unknown key\n", variable_name);
            }
        }
        g_strfreev(parts);
    }
}
/* MPE_TASK_38_REGISTRY_EXPORT_END */
/* MPE_TASK_38_CONFIG_COMMAND_BEGIN */
void cmd_config(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: config save|load|reset\n");
        return;
    }
    if (term_str_eq(argv[1], "save")) {
        if (mpe_config_save("status/engine.cfg")) {
            term_ok("config saved to status/engine.cfg\n");
        } else {
            term_err("mpe: config: save failed\n");
        }
    } else if (term_str_eq(argv[1], "load")) {
        if (mpe_config_load("status/engine.cfg")) {
            contact_cache_clear(physics_world_get_primary());
            term_ok("config loaded from status/engine.cfg\n");
        } else {
            term_err("mpe: config: load failed (file missing?)\n");
        }
    } else if (term_str_eq(argv[1], "reset")) {
        mpe_config_reset_defaults();
        contact_cache_clear(physics_world_get_primary());
        term_ok("config reset to defaults\n");
    } else {
        term_err("mpe: config: unknown subcommand. Use save|load|reset\n");
    }
}
/* MPE_TASK_38_CONFIG_COMMAND_END */
/* ------------------------------------------------------------------ */
/* Execution                                                           */
/* ------------------------------------------------------------------ */
/* MPE_TASK_V15R2_PHASE2_IMPL_BEGIN */
void cmd_exit(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_dim("logout\n");
    if (terminal_window) {
        gtk_window_destroy(GTK_WINDOW(terminal_window));
    }
}
void cmd_logout(int argc, char **argv) {
    cmd_exit(argc, argv);
}
void cmd_quit(int argc, char **argv) {
    cmd_exit(argc, argv);
}
void cmd_poweroff(int argc, char **argv) {
    (void) argc;
    (void) argv;
    mpe_config_save("status/engine.cfg");
    term_ok("System halted.\n");
    event_log_push(log_info, "Engine shutdown via terminal poweroff");
    GApplication *app = g_application_get_default();
    if (app) {
        g_application_quit(app);
    }
}
void cmd_shutdown(int argc, char **argv) {
    cmd_poweroff(argc, argv);
}
void cmd_reboot(int argc, char **argv) {
    (void) argc;
    (void) argv;
    scene_clear();
    clear_selection();
    contact_cache_clear(physics_world_get_primary());
    editor_reset();
    scene_init_default();
    term_ok("System rebooted.\n");
    event_log_push(log_info, "Scene rebooted via terminal");
}
void cmd_halt(int argc, char **argv) {
    (void) argc;
    (void) argv;
    if (physics_is_halted()) {
        physics_halt_set(false);
        term_ok("System resumed.\n");
    } else {
        physics_halt_set(true);
        term_ok("System halted. Type 'halt' again to resume.\n");
    }
}
void cmd_sleep(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: sleep <seconds>\n");
        return;
    }
    float seconds = 0.0f;
    if ((!term_parse_float(argv[1], &seconds)) || (seconds <= 0.0f)) {
        term_err("mpe: sleep: invalid duration\n");
        return;
    }
    int ticks = (int) (seconds * 60.0f);
    physics_halt_for_ticks(ticks);
    term_printf("term_ok", "Sleeping for %.1f seconds (%d ticks)...\n", seconds, ticks);
}
void cmd_sync(int argc, char **argv) {
    (void) argc;
    (void) argv;
    bool cfg_ok = mpe_config_save("status/engine.cfg");
    int scene_ok = save_scene("status/scene.dat");
    if (cfg_ok && scene_ok) {
        term_ok("sync: config + scene flushed to disk\n");
    } else {
        term_printf("term_err", "mpe: sync: config=%s scene=%s\n", cfg_ok ? "ok" : "FAILED",
                    scene_ok ? "ok" : "FAILED");
    }
}
void cmd_uptime(int argc, char **argv) {
    (void) argc;
    (void) argv;
    gint64 now = g_get_monotonic_time();
    double elapsed = (double) (now - term_engine_start_time) / 1000000.0;
    int hours = (int) (elapsed / 3600.0);
    int minutes = (int) (fmod(elapsed, 3600.0) / 60.0);
    int seconds = (int) fmod(elapsed, 60.0);
    term_printf(NULL, " up %02d:%02d:%02d, %d objects, %d joints, sleeping=%d\n", hours, minutes, seconds, (physics_world_get_primary()->body_count),
                (physics_world_get_primary()->spring_joint_count), debug_last_sleeping_object_count);
}
void cmd_free(int argc, char **argv) {
    (void) argc;
    (void) argv;
    int obj_cap = ((physics_world_get_primary()->body_capacity) > 0) ? (physics_world_get_primary()->body_capacity) : mpe_max_bodies;
    term_printf(NULL, "             total      used      free  use%%\n");
    term_printf(NULL, "objects:    %6d   %6d   %6d   %3d%%\n", obj_cap, (physics_world_get_primary()->body_count), obj_cap - (physics_world_get_primary()->body_count),
                (obj_cap > 0) ? ((physics_world_get_primary()->body_count) * 100 / obj_cap) : 0);
    term_printf(NULL, "joints:     %6d   %6d   %6d   %3d%%\n", mpe_max_joints, (physics_world_get_primary()->spring_joint_count),
                mpe_max_joints - (physics_world_get_primary()->spring_joint_count),
                (mpe_max_joints > 0) ? ((physics_world_get_primary()->spring_joint_count) * 100 / mpe_max_joints) : 0);
    term_printf(NULL, "bp pairs:   %6d   %6d\n", mpe_max_broadphase_pairs, debug_last_broadphase_pair_count);
    term_printf(NULL, "manifolds:  %6d   %6d\n", a3_max_manifolds, debug_last_manifold_count);
    term_printf(NULL, "cache:      hits=%d misses=%d\n", contact_cache_get_hits(physics_world_get_primary()), contact_cache_get_misses(physics_world_get_primary()));
}
void cmd_w(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_printf("term_echo", "camera0:  flags=<%s>\n", main_inputs.is_mouse_locked ? "LOCKED" : "FREE");
    term_printf(NULL, "  position  (%.2f, %.2f, %.2f)\n", main_camera_fov.position.x, main_camera_fov.position.y,
                main_camera_fov.position.z);
    term_printf(NULL, "  yaw=%.2f  pitch=%.2f  speed=%.2f m/s\n", main_camera_fov.yaw, main_camera_fov.pitch,
                main_camera_fov.movement_speed);
    term_printf(NULL, "  mode=%s  spawn=%s  sel=%d\n", main_inputs.is_debug_mode_active ? "DEBUG" : "GAME",
                term_spawn_type_name(), selected_object);
}
void cmd_hostname(int argc, char **argv) {
    if ((argc > 1) && (term_str_eq(argv[1], "-f"))) {
        term_out("mpe-engine.local\n");
    } else {
        term_out("mpe-engine\n");
    }
}
void cmd_id(int argc, char **argv) {
    (void) argc;
    (void) argv;
    if (main_inputs.is_debug_mode_active) {
        term_out("uid=0(root) gid=0(root) groups=0(root),1(debug)\n");
    } else {
        term_out("uid=1000(observer) gid=1000(observer) groups=1000(observer)\n");
    }
}
void cmd_which(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: which <command>\n");
        return;
    }
    for (size_t i = 0; i < terminal_command_count; i++) {
        if (term_str_eq(argv[1], terminal_commands[i].name)) {
            term_printf(NULL, "%s: shell builtin\n", terminal_commands[i].name);
            return;
        }
    }
    term_printf("term_err", "mpe: which: no %s in (/usr/bin)\n", argv[1]);
}
void cmd_true(int argc, char **argv) {
    (void) argc;
    (void) argv;
}
void cmd_false(int argc, char **argv) {
    (void) argc;
    (void) argv;
}
void cmd_time(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: time <command...>\n");
        return;
    }
    char cmd_buf[2048];
    cmd_buf[0] = '\0';
    size_t offset = 0;
    for (int i = 1; i < argc; i++) {
        if (i > 1) {
            cmd_buf[offset++] = ' ';
        }
        size_t len = strlen(argv[i]);
        if (offset + len < sizeof(cmd_buf) - 1) {
            memcpy(cmd_buf + offset, argv[i], len);
            offset += len;
            cmd_buf[offset] = '\0';
        }
    }
    gint64 start_time = g_get_monotonic_time();
    term_execute(cmd_buf);
    gint64 end_time = g_get_monotonic_time();
    double elapsed = (double) (end_time - start_time) / 1000000.0;
    term_printf("term_dim", "\nreal\t%dm%.3fs\n", (int) (elapsed / 60.0), fmod(elapsed, 60.0));
}
/* MPE_TASK_V15R2_PHASE2_IMPL_END */
/* MPE_TASK_V15R2_PHASE3_IMPL_BEGIN */
#else
/* term_sys.c — System/info commands: ps..time.
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
void cmd_ps(int argc, char **argv) {
    bool detailed = false;
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        if (strstr(argv[argument_index], "aux") || strstr(argv[argument_index], "-a")) {
            detailed = true;
        }
    }
    if ((physics_world_get_primary()->body_count) == 0) {
        term_dim("(no objects)\n");
        return;
    }
    if (detailed) {
        term_printf(NULL, "%4s %6s %-4s %-6s %8s %8s %s\n", "PID", "ID", "TYPE", "STATE", "MASS", "SPEED", "POSITION");
        for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
            rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
            term_printf(NULL, "%4d %6u %-4s %-6s %8.2f %8.3f (%.2f,%.2f,%.2f)\n", object_index, rigid_body->object_id,
                        term_object_type_name(rigid_body), term_object_state_name(rigid_body), rigid_body->mass,
                        vector3_length(rigid_body->velocity), rigid_body->position.x, rigid_body->position.y,
                        rigid_body->position.z);
        }
    } else {
        term_printf(NULL, "%4s %6s %-4s %-6s %8s\n", "PID", "ID", "TYPE", "STATE", "MASS");
        for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
            rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
            term_printf(NULL, "%4d %6u %-4s %-6s %8.2f\n", object_index, rigid_body->object_id,
                        term_object_type_name(rigid_body), term_object_state_name(rigid_body), rigid_body->mass);
        }
    }
}
void cmd_top(int argc, char **argv) {
    int limit = 10;
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        if (term_str_eq(argv[argument_index], "-n") && (argument_index + 1 < argc)) {
            float parsed_limit = 0.0f;
            if (term_parse_float(argv[argument_index + 1], &parsed_limit)) {
                limit = (int) parsed_limit;
            }
        }
    }
    if (limit < 1) {
        limit = 1;
    }
    if (limit > 16) {
        limit = 16;
    }
    if ((physics_world_get_primary()->body_count) == 0) {
        term_dim("(no objects)\n");
        return;
    }
    int top_indices[16];
    float top_speeds[16];
    for (int slot_index = 0; slot_index < limit; slot_index++) {
        top_indices[slot_index] = -1;
        top_speeds[slot_index] = -1.0f;
    }
    for (int slot_index = 0; slot_index < limit; slot_index++) {
        int best_index = -1;
        float best_speed = -1.0f;
        for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
            bool already_listed = false;
            for (int previous_slot = 0; previous_slot < slot_index; previous_slot++) {
                if (top_indices[previous_slot] == object_index) {
                    already_listed = true;
                    break;
                }
            }
            if (already_listed) {
                continue;
            }
            float object_speed = vector3_length((physics_world_get_primary()->bodies)[object_index].velocity);
            if (object_speed > best_speed) {
                best_speed = object_speed;
                best_index = object_index;
            }
        }
        if (best_index < 0) {
            break;
        }
        top_indices[slot_index] = best_index;
        top_speeds[slot_index] = best_speed;
    }
    term_printf(NULL, "%4s %-4s %-6s %8s %s\n", "PID", "TYPE", "STATE", "SPEED", "POSITION");
    for (int slot_index = 0; slot_index < limit; slot_index++) {
        if (top_indices[slot_index] < 0) {
            break;
        }
        rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[top_indices[slot_index]];
        term_printf(NULL, "%4d %-4s %-6s %8.3f (%.2f,%.2f,%.2f)\n", top_indices[slot_index],
                    term_object_type_name(rigid_body), term_object_state_name(rigid_body), top_speeds[slot_index],
                    rigid_body->position.x, rigid_body->position.y, rigid_body->position.z);
    }
}
void cmd_df(int argc, char **argv) {
    (void) argc;
    (void) argv;
    int object_capacity_value = ((physics_world_get_primary()->body_capacity) > 0) ? (physics_world_get_primary()->body_capacity) : mpe_max_bodies;
    int joint_capacity_value = mpe_max_joints;
    int object_percent = (object_capacity_value > 0) ? ((physics_world_get_primary()->body_count) * 100 / object_capacity_value) : 0;
    int joint_percent = (joint_capacity_value > 0) ? ((physics_world_get_primary()->spring_joint_count) * 100 / joint_capacity_value) : 0;
    term_printf(NULL, "Filesystem     Size   Used  Avail Use%% Mounted on\n");
    term_printf(NULL, "objects       %6d %6d %6d %3d%% /obj\n", object_capacity_value, (physics_world_get_primary()->body_count),
                object_capacity_value - (physics_world_get_primary()->body_count), object_percent);
    term_printf(NULL, "joints        %6d %6d %6d %3d%% /joint\n", joint_capacity_value, (physics_world_get_primary()->spring_joint_count),
                joint_capacity_value - (physics_world_get_primary()->spring_joint_count), joint_percent);
}
void cmd_du(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: du <path...>\n");
        return;
    }
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        const char *target = argv[argument_index];
        if (term_classify_token(target) == term_target_joint) {
            int joint_index = term_joint_from_token(target);
            if (joint_index >= 0) {
                spring_joint *joint = &(physics_world_get_primary()->spring_joints)[joint_index];
                term_printf(NULL, "/joint/%d len=%.2f k=%.1f d=%.1f\n", joint_index, joint->equilibrium_length,
                            joint->spring_constant, joint->damping_coefficient);
            } else {
                term_printf("term_err", "mpe: %s: No such joint\n", target);
            }
        } else {
            int object_index = term_object_from_token(target);
            if (object_index >= 0) {
                rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
                float size_value = (rigid_body->type == object_sphere) ? rigid_body->radius
                                                                       : vector3_length(rigid_body->half_extensions);
                term_printf(NULL, "/obj/%d mass=%.2f size=%.2f\n", object_index, rigid_body->mass, size_value);
            } else {
                term_printf("term_err", "mpe: %s: No such object\n", target);
            }
        }
    }
}
void cmd_uname(int argc, char **argv) {
    /* MPE_TASK_V15R2_UNAME_EXPANDED */
    bool print_all = false, print_sys = false, print_rel = false;
    bool print_mach = false, print_os = false;
    for (int i = 1; i < argc; i++) {
        if (term_str_eq(argv[i], "-a")) {
            print_all = true;
        } else if (term_str_eq(argv[i], "-s")) {
            print_sys = true;
        } else if (term_str_eq(argv[i], "-r")) {
            print_rel = true;
        } else if (term_str_eq(argv[i], "-m")) {
            print_mach = true;
        } else if (term_str_eq(argv[i], "-o")) {
            print_os = true;
        } else if (argv[i][0] == '-') {
            print_all = true;
        }
    }
    if (print_all) {
        term_printf(NULL, "MPE %s mpe-engine x86_64 POSIX-like/GTK3/OpenGL3.3 MPE\n", a3_version_string);
    } else if (print_sys) {
        term_out("MPE\n");
    } else if (print_rel) {
        term_printf(NULL, "%s\n", a3_version_string);
    } else if (print_mach) {
        term_out("x86_64\n");
    } else if (print_os) {
        term_out("POSIX-like/GTK3/OpenGL3.3\n");
    } else {
        term_printf(NULL, "MPE %s\n", a3_version_string);
    }
}
void cmd_whoami(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_out("root\n");
}
void cmd_date(int argc, char **argv) {
    (void) argc;
    (void) argv;
    time_t current_time = time(NULL);
    struct tm *local_time = localtime(&current_time);
    char time_buffer[128];
    strftime(time_buffer, sizeof(time_buffer), "%a %Y-%m-%d %H:%M:%S %Z", local_time);
    term_printf(NULL, "%s\n", time_buffer);
}
void cmd_echo(int argc, char **argv) {
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        term_out(argv[argument_index]);
        if (argument_index + 1 < argc) {
            term_out(" ");
        }
    }
    term_out("\n");
}
/* MPE_TASK_38_REGISTRY_ENV_BEGIN */
void cmd_env(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_printf(NULL, "CAMERA_SPEED=%.4f\n", main_camera_fov.movement_speed);
    int current_category = -1;
    for (size_t i = 0; i < g_registry_count; i++) {
        if ((int) g_registry[i].category != current_category) {
            current_category = (int) g_registry[i].category;
            term_printf("term_echo", "[%s]\n", mpe_config_category_name((param_category) current_category));
        }
        if (g_registry[i].type == p_int) {
            term_printf(NULL, "  %s = %d\n", g_registry[i].key, *(int *) g_registry[i].storage);
        } else if (g_registry[i].type == p_bool) {
            term_printf(NULL, "  %s = %s\n", g_registry[i].key, (*(bool *) g_registry[i].storage) ? "true" : "false");
        } else {
            term_printf(NULL, "  %s = %.4f\n", g_registry[i].key, *(float *) g_registry[i].storage);
        }
    }
}
/* MPE_TASK_38_REGISTRY_ENV_END */
/* MPE_TASK_38_REGISTRY_EXPORT_BEGIN */
void cmd_export(int argc, char **argv) {
    if (argc < 2) {
        cmd_env(argc, argv);
        return;
    }
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        char **parts = g_strsplit(argv[argument_index], "=", 2);
        if ((!parts[0]) || (!parts[1])) {
            term_printf("term_err", "mpe: export: usage: export KEY=value\n");
            g_strfreev(parts);
            continue;
        }
        const char *variable_name = parts[0];
        float variable_value = 0.0f;
        if (!term_parse_float(parts[1], &variable_value)) {
            term_printf("term_err", "mpe: export: invalid value '%s'\n", parts[1]);
            g_strfreev(parts);
            continue;
        }
        if (term_str_eq(variable_name, "CAMERA_SPEED")) {
            main_camera_fov.movement_speed = variable_value;
            term_printf("term_ok", "CAMERA_SPEED=%.4f\n", variable_value);
        } else {
            const mpe_param *param = mpe_config_find(variable_name);
            if (param) {
                bool clamped = !mpe_config_set_float(variable_name, variable_value);
                if (param->type == p_int) {
                    term_printf("term_ok", "%s = %d%s\n", variable_name, *(int *) param->storage,
                                clamped ? " (clamped)" : "");
                } else if (param->type == p_bool) {
                    term_printf("term_ok", "%s = %s%s\n", variable_name, (*(bool *) param->storage) ? "true" : "false",
                                clamped ? " (clamped)" : "");
                } else {
                    term_printf("term_ok", "%s = %.4f%s\n", variable_name, *(float *) param->storage,
                                clamped ? " (clamped)" : "");
                }
            } else {
                term_printf("term_err", "mpe: export: %s: unknown key\n", variable_name);
            }
        }
        g_strfreev(parts);
    }
}
/* MPE_TASK_38_REGISTRY_EXPORT_END */
/* MPE_TASK_38_CONFIG_COMMAND_BEGIN */
void cmd_config(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: config save|load|reset\n");
        return;
    }
    if (term_str_eq(argv[1], "save")) {
        if (mpe_config_save("status/engine.cfg")) {
            term_ok("config saved to status/engine.cfg\n");
        } else {
            term_err("mpe: config: save failed\n");
        }
    } else if (term_str_eq(argv[1], "load")) {
        if (mpe_config_load("status/engine.cfg")) {
            contact_cache_clear(physics_world_get_primary());
            term_ok("config loaded from status/engine.cfg\n");
        } else {
            term_err("mpe: config: load failed (file missing?)\n");
        }
    } else if (term_str_eq(argv[1], "reset")) {
        mpe_config_reset_defaults();
        contact_cache_clear(physics_world_get_primary());
        term_ok("config reset to defaults\n");
    } else {
        term_err("mpe: config: unknown subcommand. Use save|load|reset\n");
    }
}
/* MPE_TASK_38_CONFIG_COMMAND_END */
/* ------------------------------------------------------------------ */
/* Execution                                                           */
/* ------------------------------------------------------------------ */
/* MPE_TASK_V15R2_PHASE2_IMPL_BEGIN */
void cmd_exit(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_dim("logout\n");
    if (terminal_window) {
        gtk_widget_destroy(terminal_window);
    }
}
void cmd_logout(int argc, char **argv) {
    cmd_exit(argc, argv);
}
void cmd_quit(int argc, char **argv) {
    cmd_exit(argc, argv);
}
void cmd_poweroff(int argc, char **argv) {
    (void) argc;
    (void) argv;
    mpe_config_save("status/engine.cfg");
    term_ok("System halted.\n");
    event_log_push(log_info, "Engine shutdown via terminal poweroff");
    gtk_main_quit();
}
void cmd_shutdown(int argc, char **argv) {
    cmd_poweroff(argc, argv);
}
void cmd_reboot(int argc, char **argv) {
    (void) argc;
    (void) argv;
    scene_clear();
    clear_selection();
    contact_cache_clear(physics_world_get_primary());
    editor_reset();
    scene_init_default();
    term_ok("System rebooted.\n");
    event_log_push(log_info, "Scene rebooted via terminal");
}
void cmd_halt(int argc, char **argv) {
    (void) argc;
    (void) argv;
    if (physics_is_halted()) {
        physics_halt_set(false);
        term_ok("System resumed.\n");
    } else {
        physics_halt_set(true);
        term_ok("System halted. Type 'halt' again to resume.\n");
    }
}
void cmd_sleep(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: sleep <seconds>\n");
        return;
    }
    float seconds = 0.0f;
    if ((!term_parse_float(argv[1], &seconds)) || (seconds <= 0.0f)) {
        term_err("mpe: sleep: invalid duration\n");
        return;
    }
    int ticks = (int) (seconds * 60.0f);
    physics_halt_for_ticks(ticks);
    term_printf("term_ok", "Sleeping for %.1f seconds (%d ticks)...\n", seconds, ticks);
}
void cmd_sync(int argc, char **argv) {
    (void) argc;
    (void) argv;
    bool cfg_ok = mpe_config_save("status/engine.cfg");
    int scene_ok = save_scene("status/scene.dat");
    if (cfg_ok && scene_ok) {
        term_ok("sync: config + scene flushed to disk\n");
    } else {
        term_printf("term_err", "mpe: sync: config=%s scene=%s\n", cfg_ok ? "ok" : "FAILED",
                    scene_ok ? "ok" : "FAILED");
    }
}
void cmd_uptime(int argc, char **argv) {
    (void) argc;
    (void) argv;
    gint64 now = g_get_monotonic_time();
    double elapsed = (double) (now - term_engine_start_time) / 1000000.0;
    int hours = (int) (elapsed / 3600.0);
    int minutes = (int) (fmod(elapsed, 3600.0) / 60.0);
    int seconds = (int) fmod(elapsed, 60.0);
    term_printf(NULL, " up %02d:%02d:%02d, %d objects, %d joints, sleeping=%d\n", hours, minutes, seconds, (physics_world_get_primary()->body_count),
                (physics_world_get_primary()->spring_joint_count), debug_last_sleeping_object_count);
}
void cmd_free(int argc, char **argv) {
    (void) argc;
    (void) argv;
    int obj_cap = ((physics_world_get_primary()->body_capacity) > 0) ? (physics_world_get_primary()->body_capacity) : mpe_max_bodies;
    term_printf(NULL, "             total      used      free  use%%\n");
    term_printf(NULL, "objects:    %6d   %6d   %6d   %3d%%\n", obj_cap, (physics_world_get_primary()->body_count), obj_cap - (physics_world_get_primary()->body_count),
                (obj_cap > 0) ? ((physics_world_get_primary()->body_count) * 100 / obj_cap) : 0);
    term_printf(NULL, "joints:     %6d   %6d   %6d   %3d%%\n", mpe_max_joints, (physics_world_get_primary()->spring_joint_count),
                mpe_max_joints - (physics_world_get_primary()->spring_joint_count),
                (mpe_max_joints > 0) ? ((physics_world_get_primary()->spring_joint_count) * 100 / mpe_max_joints) : 0);
    term_printf(NULL, "bp pairs:   %6d   %6d\n", mpe_max_broadphase_pairs, debug_last_broadphase_pair_count);
    term_printf(NULL, "manifolds:  %6d   %6d\n", a3_max_manifolds, debug_last_manifold_count);
    term_printf(NULL, "cache:      hits=%d misses=%d\n", contact_cache_get_hits(physics_world_get_primary()), contact_cache_get_misses(physics_world_get_primary()));
}
void cmd_w(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_printf("term_echo", "camera0:  flags=<%s>\n", main_inputs.is_mouse_locked ? "LOCKED" : "FREE");
    term_printf(NULL, "  position  (%.2f, %.2f, %.2f)\n", main_camera_fov.position.x, main_camera_fov.position.y,
                main_camera_fov.position.z);
    term_printf(NULL, "  yaw=%.2f  pitch=%.2f  speed=%.2f m/s\n", main_camera_fov.yaw, main_camera_fov.pitch,
                main_camera_fov.movement_speed);
    term_printf(NULL, "  mode=%s  spawn=%s  sel=%d\n", main_inputs.is_debug_mode_active ? "DEBUG" : "GAME",
                term_spawn_type_name(), selected_object);
}
void cmd_hostname(int argc, char **argv) {
    if ((argc > 1) && (term_str_eq(argv[1], "-f"))) {
        term_out("mpe-engine.local\n");
    } else {
        term_out("mpe-engine\n");
    }
}
void cmd_id(int argc, char **argv) {
    (void) argc;
    (void) argv;
    if (main_inputs.is_debug_mode_active) {
        term_out("uid=0(root) gid=0(root) groups=0(root),1(debug)\n");
    } else {
        term_out("uid=1000(observer) gid=1000(observer) groups=1000(observer)\n");
    }
}
void cmd_which(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: which <command>\n");
        return;
    }
    for (size_t i = 0; i < terminal_command_count; i++) {
        if (term_str_eq(argv[1], terminal_commands[i].name)) {
            term_printf(NULL, "%s: shell builtin\n", terminal_commands[i].name);
            return;
        }
    }
    term_printf("term_err", "mpe: which: no %s in (/usr/bin)\n", argv[1]);
}
void cmd_true(int argc, char **argv) {
    (void) argc;
    (void) argv;
}
void cmd_false(int argc, char **argv) {
    (void) argc;
    (void) argv;
}
void cmd_time(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: time <command...>\n");
        return;
    }
    char cmd_buf[2048];
    cmd_buf[0] = '\0';
    size_t offset = 0;
    for (int i = 1; i < argc; i++) {
        if (i > 1) {
            cmd_buf[offset++] = ' ';
        }
        size_t len = strlen(argv[i]);
        if (offset + len < sizeof(cmd_buf) - 1) {
            memcpy(cmd_buf + offset, argv[i], len);
            offset += len;
            cmd_buf[offset] = '\0';
        }
    }
    gint64 start_time = g_get_monotonic_time();
    term_execute(cmd_buf);
    gint64 end_time = g_get_monotonic_time();
    double elapsed = (double) (end_time - start_time) / 1000000.0;
    term_printf("term_dim", "\nreal\t%dm%.3fs\n", (int) (elapsed / 60.0), fmod(elapsed, 60.0));
}
/* MPE_TASK_V15R2_PHASE2_IMPL_END */
/* MPE_TASK_V15R2_PHASE3_IMPL_BEGIN */
#endif /* MPE_GTK4 */

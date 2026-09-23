/* GTK4-PREP: GTK3 preserved under #else; GTK4 full port follows. */
#ifdef MPE_GTK4
/* term_admin.c — Admin/batch/scene/shell commands: sed..dmesg + vi.
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
#include <strings.h>

static gint64 posix_monotonic_time(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (gint64)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
}
bool all_targets_matched_any(int argc, char **argv) {
    for (int i = 2; i < argc; i++) {
        if (term_is_all_token(argv[i])) {
            return true;
        }
    }
    return false;
}
void cmd_sed(int argc, char **argv) {
    if (argc < 3) {
        term_err("usage: sed s/field/value/ <target...>\n");
        return;
    }
    const char *expression = argv[1];
    if ((expression[0] != 's') || (expression[1] != '/')) {
        term_err("mpe: sed: expression must start with s/\n");
        return;
    }
    const char *field_start = expression + 2;
    const char *field_end = strchr(field_start, '/');
    if (!field_end) {
        term_err("mpe: sed: malformed expression (missing second /)\n");
        return;
    }
    char field_name[64];
    int field_len = (int) (field_end - field_start);
    if (field_len >= 64) {
        field_len = 63;
    }
    strncpy(field_name, field_start, field_len);
    field_name[field_len] = '\0';
    const char *value_start = field_end + 1;
    const char *value_end = strchr(value_start, '/');
    char value_str[64];
    if (value_end) {
        int value_len = (int) (value_end - value_start);
        if (value_len >= 64) {
            value_len = 63;
        }
        strncpy(value_str, value_start, value_len);
        value_str[value_len] = '\0';
    } else {
        strncpy(value_str, value_start, 63);
        value_str[63] = '\0';
    }
    float new_value = 0.0f;
    bool is_numeric = term_parse_float(value_str, &new_value);
    bool make_static = term_str_eq(value_str, "static") || term_str_eq(value_str, "1");
    bool make_dynamic = term_str_eq(value_str, "dynamic") || term_str_eq(value_str, "0");
    int modified_count = 0;
    for (int argument_index = 2; argument_index < argc; argument_index++) {
        const char *target = argv[argument_index];
        bool all_targets = term_is_all_token(target);
        if (all_targets) {
            for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
                rigidbody *rb = &(physics_world_get_primary()->bodies)[object_index];
                if (term_str_eq(field_name, "mass")) {
                    if (!is_numeric) {
                        continue;
                    }
                    term_set_object_mass(object_index, new_value);
                } else if (term_str_eq(field_name, "radius")) {
                    if ((!is_numeric) || (rb->type != object_sphere)) {
                        continue;
                    }
                    rb->radius = new_value;
                    rigidbody_update_inertia_sphere(rb);
                    rigidbody_wake(rb);
                } else if (term_str_eq(field_name, "friction_static") || term_str_eq(field_name, "fs")) {
                    if (!is_numeric) {
                        continue;
                    }
                    rb->friction_static = new_value;
                } else if (term_str_eq(field_name, "friction_kinetic") || term_str_eq(field_name, "fk")) {
                    if (!is_numeric) {
                        continue;
                    }
                    rb->friction_kinetic = new_value;
                } else if (term_str_eq(field_name, "restitution") || term_str_eq(field_name, "rest")) {
                    if (!is_numeric) {
                        continue;
                    }
                    rb->restitution = new_value;
                    if (rb->restitution < 0.0f) {
                        rb->restitution = 0.0f;
                    }
                    if (rb->restitution > 1.0f) {
                        rb->restitution = 1.0f;
                    }
                } else if (term_str_eq(field_name, "static")) {
                    term_set_object_static(object_index, make_static || (!make_dynamic));
                } else if (term_str_eq(field_name, "dynamic")) {
                    term_set_object_static(object_index, !make_dynamic);
                } else if (term_str_eq(field_name, "nice")) {
                    if (!is_numeric) {
                        continue;
                    }
                    int nice_val = (int) new_value;
                    if (nice_val < -20) {
                        nice_val = -20;
                    }
                    if (nice_val > 19) {
                        nice_val = 19;
                    }
                    rb->nice_value = nice_val;
                }
                modified_count++;
            }
            contact_cache_clear(physics_world_get_primary());
            continue;
        }
        int object_index = term_object_from_token(target);
        if (object_index < 0) {
            term_printf("term_err", "mpe: sed: %s: No such object\n", target);
            continue;
        }
        rigidbody *rb = &(physics_world_get_primary()->bodies)[object_index];
        if (term_str_eq(field_name, "mass")) {
            if (!is_numeric) {
                term_err("mpe: sed: mass requires numeric value\n");
                continue;
            }
            term_set_object_mass(object_index, new_value);
        } else if (term_str_eq(field_name, "radius")) {
            if (!is_numeric) {
                term_err("mpe: sed: radius requires numeric value\n");
                continue;
            }
            if (rb->type != object_sphere) {
                term_err("mpe: sed: radius only applies to spheres\n");
                continue;
            }
            rb->radius = new_value;
            rigidbody_update_inertia_sphere(rb);
            rigidbody_wake(rb);
        } else if (term_str_eq(field_name, "friction_static") || term_str_eq(field_name, "fs")) {
            if (!is_numeric) {
                term_err("mpe: sed: friction requires numeric value\n");
                continue;
            }
            rb->friction_static = new_value;
        } else if (term_str_eq(field_name, "friction_kinetic") || term_str_eq(field_name, "fk")) {
            if (!is_numeric) {
                term_err("mpe: sed: friction requires numeric value\n");
                continue;
            }
            rb->friction_kinetic = new_value;
        } else if (term_str_eq(field_name, "restitution") || term_str_eq(field_name, "rest")) {
            if (!is_numeric) {
                term_err("mpe: sed: restitution requires numeric value\n");
                continue;
            }
            rb->restitution = new_value;
            if (rb->restitution < 0.0f) {
                rb->restitution = 0.0f;
            }
            if (rb->restitution > 1.0f) {
                rb->restitution = 1.0f;
            }
        } else if (term_str_eq(field_name, "static")) {
            term_set_object_static(object_index, true);
        } else if (term_str_eq(field_name, "dynamic")) {
            term_set_object_static(object_index, false);
        } else if (term_str_eq(field_name, "nice")) {
            if (!is_numeric) {
                term_err("mpe: sed: nice requires numeric value\n");
                continue;
            }
            int nice_val = (int) new_value;
            if (nice_val < -20) {
                nice_val = -20;
            }
            if (nice_val > 19) {
                nice_val = 19;
            }
            rb->nice_value = nice_val;
        } else {
            term_printf("term_err", "mpe: sed: unknown field '%s'\n", field_name);
            continue;
        }
        modified_count++;
        contact_cache_clear(physics_world_get_primary());
        term_printf("term_ok", "/obj/%d: %s=%s\n", object_index, field_name, value_str);
    }
    if (all_targets_matched_any(argc, argv)) {
        term_printf("term_ok", "sed: modified %d object(s)\n", modified_count);
    }
}

void cmd_nice(int argc, char **argv) {
    if (argc < 3) {
        term_err("usage: nice <priority> <object...>\n");
        return;
    }
    float priority_float = 0.0f;
    if (!term_parse_float(argv[1], &priority_float)) {
        term_printf("term_err", "mpe: nice: invalid priority '%s'\n", argv[1]);
        return;
    }
    int priority = (int) priority_float;
    if (priority < -20) {
        priority = -20;
    }
    if (priority > 19) {
        priority = 19;
    }
    for (int argument_index = 2; argument_index < argc; argument_index++) {
        int object_index = term_require_object(argv[argument_index]);
        if (object_index < 0) {
            continue;
        }
        (physics_world_get_primary()->bodies)[object_index].nice_value = priority;
        rigidbody_wake(&(physics_world_get_primary()->bodies)[object_index]);
        term_printf("term_ok", "/obj/%d nice=%d\n", object_index, priority);
    }
}
void cmd_renice(int argc, char **argv) {
    cmd_nice(argc, argv);
}
void cmd_ping(int argc, char **argv) {
    int ping_count = 1;
    int argument_index = 1;
    if ((argc > 1) && (term_str_eq(argv[1], "-c")) && (argc > 2)) {
        float v = 0.0f;
        if (term_parse_float(argv[2], &v)) {
            ping_count = (int) v;
        }
        if (ping_count < 1) {
            ping_count = 1;
        }
        if (ping_count > 10) {
            ping_count = 10;
        }
        argument_index = 3;
    }
    if (argument_index >= argc) {
        term_err("usage: ping [-c count] <object>\n");
        return;
    }
    int object_index = term_require_object(argv[argument_index]);
    if (object_index < 0) {
        return;
    }
    rigidbody *rb = &(physics_world_get_primary()->bodies)[object_index];
    if (rb->static_state) {
        term_printf("term_dim", "PING /obj/%d: no response (static)\n", object_index);
        return;
    }
    if (rb->is_sleeping) {
        term_printf("term_dim", "PING /obj/%d: no response (sleeping)\n", object_index);
        return;
    }
    for (int ping_index = 0; ping_index < ping_count; ping_index++) {
        vector3 micro_impulse = vector3_scaling(rb->velocity, 0.0f);
        micro_impulse = (vector3){0.001f, 0.001f, 0.001f};
        rb->velocity = vector3_addition(rb->velocity, micro_impulse);
    }
    float speed_delta = 0.001f * sqrtf(3.0f) * (float) ping_count;
    term_printf("term_ok", "PING /obj/%d: %d ping(s), velocity delta %.6f m/s, state=%s\n", object_index, ping_count,
                speed_delta, rb->is_sleeping ? "sleep" : "run");
}
void cmd_mount(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: mount <path>\n");
        return;
    }
    const char *scene_path = argv[1];
    if (scene_loading(scene_path)) {
        editor_reset();
        contact_cache_clear(physics_world_get_primary());
        term_printf("term_ok", "mounted %s: %d objects loaded\n", scene_path, (physics_world_get_primary()->body_count));
        event_log_push(log_info, "Scene mounted via terminal: %s", scene_path);
    } else {
        term_printf("term_err", "mpe: mount: %s: failed to load\n", scene_path);
    }
}
void cmd_umount(int argc, char **argv) {
    (void) argc;
    (void) argv;
    int save_result = save_scene("status/scene.dat");
    if (save_result) {
        term_ok("umount: scene saved to status/scene.dat\n");
    } else {
        term_err("mpe: umount: save failed\n");
    }
    scene_clear();
    clear_selection();
    contact_cache_clear(physics_world_get_primary());
    editor_reset();
    term_ok("umount: scene cleared\n");
    event_log_push(log_info, "Scene unmounted via terminal");
}
void cmd_mkfs(int argc, char **argv) {
    (void) argc;
    (void) argv;
    scene_clear();
    clear_selection();
    contact_cache_clear(physics_world_get_primary());
    editor_reset();
    term_printf("term_ok", "Scene formatted. 0 objects, 0 joints.\n");
    event_log_push(log_info, "Scene formatted via terminal (mkfs)");
}
void cmd_fsck(int argc, char **argv) {
    bool auto_fix = false;
    for (int i = 1; i < argc; i++) {
        if (term_str_eq(argv[i], "-y")) {
            auto_fix = true;
        }
    }
    int error_count = 0;
    int warning_count = 0;
    term_printf("term_echo", "fsck: checking %d objects...\n", (physics_world_get_primary()->body_count));
    for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
        rigidbody *rb = &(physics_world_get_primary()->bodies)[object_index];
        bool has_error = false;
        if ((!isfinite(rb->position.x)) || (!isfinite(rb->position.y)) || (!isfinite(rb->position.z))) {
            term_printf("term_err", "  /obj/%d: position NaN/Inf\n", object_index);
            has_error = true;
        }
        if ((!isfinite(rb->velocity.x)) || (!isfinite(rb->velocity.y)) || (!isfinite(rb->velocity.z))) {
            term_printf("term_err", "  /obj/%d: velocity NaN/Inf\n", object_index);
            has_error = true;
        }
        if ((!isfinite(rb->angular_velocity.x)) || (!isfinite(rb->angular_velocity.y)) ||
            (!isfinite(rb->angular_velocity.z))) {
            term_printf("term_err", "  /obj/%d: angular_velocity NaN/Inf\n", object_index);
            has_error = true;
        }
        if ((!isfinite(rb->orientation.w)) || (!isfinite(rb->orientation.x)) || (!isfinite(rb->orientation.y)) ||
            (!isfinite(rb->orientation.z))) {
            term_printf("term_err", "  /obj/%d: orientation NaN/Inf\n", object_index);
            has_error = true;
        }
        if ((!rb->static_state) && ((rb->mass <= 0.0f) || (!isfinite(rb->mass)))) {
            term_printf("term_err", "  /obj/%d: invalid mass %.4f (dynamic)\n", object_index, rb->mass);
            has_error = true;
        }
        if ((!rb->static_state) && (rb->inverse_mass <= 0.0f)) {
            term_printf("term_dim", "  /obj/%d: warning: inverse_mass=%.4f (dynamic)\n", object_index,
                        rb->inverse_mass);
            warning_count++;
        }
        float orient_len_sq = rb->orientation.w * rb->orientation.w + rb->orientation.x * rb->orientation.x +
                              rb->orientation.y * rb->orientation.y + rb->orientation.z * rb->orientation.z;
        if ((orient_len_sq < 0.9f) || (orient_len_sq > 1.1f)) {
            term_printf("term_dim", "  /obj/%d: warning: orientation not normalized (|q|^2=%.4f)\n", object_index,
                        orient_len_sq);
            warning_count++;
        }
        if (has_error) {
            error_count++;
            if (auto_fix) {
                rigidbody_sanitize(rb);
                rigidbody_wake(rb); /* FIX_032 */
                term_printf("term_ok", "  /obj/%d: sanitized\n", object_index);
            }
        }
    }
    term_printf("term_echo", "fsck: checking %d joint slots...\n", mpe_max_joints);
    for (int joint_index = 0; joint_index < mpe_max_joints; joint_index++) {
        if (!(physics_world_get_primary()->spring_joints)[joint_index].is_active) {
            continue;
        }
        spring_joint *j = &(physics_world_get_primary()->spring_joints)[joint_index];
        int index_a = scene_find_object_index_by_id(j->object_id_a);
        int index_b = scene_find_object_index_by_id(j->object_id_b);
        if (index_a < 0) {
            term_printf("term_err", "  /joint/%d: object_a (id=%u) not found\n", joint_index, j->object_id_a);
            error_count++;
            if (auto_fix) {
                remove_joint(physics_world_get_primary(), joint_index);
                term_printf("term_ok", "  /joint/%d: removed\n", joint_index);
            }
            continue;
        }
        if (index_b < 0) {
            term_printf("term_err", "  /joint/%d: object_b (id=%u) not found\n", joint_index, j->object_id_b);
            error_count++;
            if (auto_fix) {
                remove_joint(physics_world_get_primary(), joint_index);
                term_printf("term_ok", "  /joint/%d: removed\n", joint_index);
            }
            continue;
        }
        if ((j->equilibrium_length < 0.0f) || (!isfinite(j->equilibrium_length))) {
            term_printf("term_err", "  /joint/%d: invalid rest length %.4f\n", joint_index, j->equilibrium_length);
            error_count++;
        }
        if ((j->spring_constant <= 0.0f) || (!isfinite(j->spring_constant))) {
            term_printf("term_err", "  /joint/%d: invalid spring constant %.4f\n", joint_index, j->spring_constant);
            error_count++;
        }
    }
    if (auto_fix) {
        contact_cache_clear(physics_world_get_primary());
    }
    if (error_count == 0) {
        term_printf("term_ok", "fsck: PASS — %d objects, %d joints, %d warning(s), 0 errors\n", (physics_world_get_primary()->body_count),
                    (physics_world_get_primary()->spring_joint_count), warning_count);
    } else {
        term_printf("term_err", "fsck: FAIL — %d error(s), %d warning(s)%s\n", error_count, warning_count,
                    auto_fix ? " (auto-fixed)" : " (run fsck -y to fix)");
    }
    event_log_push(error_count == 0 ? log_info : log_warn, "fsck: %d errors, %d warnings%s", error_count, warning_count,
                   auto_fix ? " (fixed)" : "");
}
/* MPE_TASK_V15R2_PHASE5_IMPL_END */
/* MPE_TASK_V15R2_PHASE6_IMPL_BEGIN */
void cmd_netstat(int argc, char **argv) {
    bool show_all = false;
    for (int i = 1; i < argc; i++) {
        if (term_str_eq(argv[i], "-a")) {
            show_all = true;
        }
    }
    term_printf("term_echo", "Active Joints (spring connections)\n");
    term_printf(NULL, "Proto  Local        Foreign      State         K        D      Len\n");
    int listed = 0;
    for (int ji = 0; ji < mpe_max_joints; ji++) {
        if ((!(physics_world_get_primary()->spring_joints)[ji].is_active) && (!show_all)) {
            continue;
        }
        int ia = scene_find_object_index_by_id((physics_world_get_primary()->spring_joints)[ji].object_id_a);
        int ib = scene_find_object_index_by_id((physics_world_get_primary()->spring_joints)[ji].object_id_b);
        const char *state_text = (physics_world_get_primary()->spring_joints)[ji].is_active ? "ESTABLISHED" : "CLOSED";
        term_printf(NULL, "spring /obj/%-6d /obj/%-6d %-12s %7.1f %7.1f %7.2f\n", ia, ib, state_text,
                    (physics_world_get_primary()->spring_joints)[ji].spring_constant, (physics_world_get_primary()->spring_joints)[ji].damping_coefficient,
                    (physics_world_get_primary()->spring_joints)[ji].equilibrium_length);
        listed++;
    }
    if (listed == 0) {
        term_dim("(no connections)\n");
    } else {
        term_printf("term_dim", "%d connection(s) active\n", listed);
    }
}
void cmd_ifconfig(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_printf("term_echo", "camera0:  flags=<%s>  mode %s\n", main_inputs.is_mouse_locked ? "LOCKED" : "FREE",
                main_inputs.is_debug_mode_active ? "DEBUG" : "GAME");
    term_printf(NULL, "    position (%.2f, %.2f, %.2f)\n", main_camera_fov.position.x, main_camera_fov.position.y,
                main_camera_fov.position.z);
    term_printf(NULL, "    yaw %.2f  pitch %.2f  speed %.2f m/s\n", main_camera_fov.yaw, main_camera_fov.pitch,
                main_camera_fov.movement_speed);
    term_printf(NULL, "    sensitivity %.3f  steer %.3f\n", g_cfg.camera.mouse_sensitivity,
                g_cfg.camera.steer_sensitivity);
    term_out("\n");
    term_printf("term_echo", "render0:  flags=<ACTIVE>\n");
    term_printf(NULL, "    light (%.1f, %.1f, %.1f)\n", g_cfg.render.light_x, g_cfg.render.light_y,
                g_cfg.render.light_z);
    term_printf(NULL, "    ambient %.2f  specular %.2f  exponent %.1f\n", g_cfg.render.ambient_strength,
                g_cfg.render.specular_coeff, g_cfg.render.specular_exponent);
    term_out("\n");
    term_printf("term_echo", "input0:  flags=<%s>\n", main_inputs.is_mouse_locked ? "GRABBED" : "RELEASED");
    term_printf(NULL, "    spawn=%s  selected=%d  marked_joint=%d\n",
                term_spawn_type_name(), selected_object,
                main_inputs.marked_joint_object_index);
    term_printf(NULL, "    objects=%d  joints=%d  sleeping=%d\n", (physics_world_get_primary()->body_count), (physics_world_get_primary()->spring_joint_count),
                debug_last_sleeping_object_count);
}
void cmd_lsmod(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_printf(NULL, "%-24s %6s  %s\n", "Module", "Size", "Used by");
    term_printf(NULL, "%-24s %6s  %s\n", "instanced_shader", "1", "sphere_mesh, cube_mesh");
    term_printf(NULL, "%-24s %6s  %s\n", "utility_shader", "1", "grid, wireframe, joints");
    term_printf(NULL, "%-24s %6s  %s\n", "sphere_mesh", "1", "instanced_shader");
    term_printf(NULL, "%-24s %6s  %s\n", "cube_mesh", "1", "instanced_shader");
    term_printf(NULL, "%-24s %6s  %s\n", "grid_mesh", "1", "utility_shader");
    term_printf(NULL, "%-24s %6s  %s\n", "wireframe_renderer", "1", "utility_shader");
    term_printf(NULL, "%-24s %6s  %s\n", "joint_renderer", "1", "utility_shader");
    term_printf(NULL, "%-24s %6s  %s\n", "spatial_hash_bp", "1", "collision_pipeline");
    term_printf(NULL, "%-24s %6s  %s\n", "contact_cache", "1", "impulse_solver");
    term_printf(NULL, "%-24s %6s  %s\n", "sequential_solver", "1", "physics_step");
    term_printf(NULL, "%-24s %6s  %s\n", "depenetration_pass", "1", "physics_step");
    term_printf(NULL, "%-24s %6s  %s\n", "sleep_system", "1", "physics_step, broadphase");
    term_printf(NULL, "%-24s %6s  %s\n", "config_registry", "1", "config_menu, terminal, F9");
    term_printf(NULL, "%-24s %6s  %s\n", "event_log", "1", "dmesg (pending)");
    term_printf(NULL, "%-24s %6s  %s\n", "debug_terminal", "1", "input_control");
}
/* MPE_TASK_V15R2_PHASE6_IMPL_END */
/* MPE_TASK_V15R2_PHASE7_IMPL_BEGIN */
void cmd_alias(int argc, char **argv) {
    if (argc < 2) {
        if (term_alias_count == 0) {
            term_dim("(no aliases defined)\n");
            return;
        }
        for (int i = 0; i < term_alias_count; i++) {
            term_printf(NULL, "alias %s='%s'\n", term_alias_names[i], term_alias_values[i]);
        }
        return;
    }
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        char *equals_pos = strchr(argv[argument_index], '=');
        if (!equals_pos) {
            bool found = false;
            for (int i = 0; i < term_alias_count; i++) {
                if (term_str_eq(argv[argument_index], term_alias_names[i])) {
                    term_printf(NULL, "alias %s='%s'\n", term_alias_names[i], term_alias_values[i]);
                    found = true;
                    break;
                }
            }
            if (!found) {
                term_printf("term_err", "mpe: alias: %s: not found\n", argv[argument_index]);
            }
            continue;
        }
        *equals_pos = '\0';
        const char *alias_name = argv[argument_index];
        const char *alias_value = equals_pos + 1;
        bool updated = false;
        for (int i = 0; i < term_alias_count; i++) {
            if (term_str_eq(alias_name, term_alias_names[i])) {
                strncpy(term_alias_values[i], alias_value, term_alias_value_len - 1);
                term_alias_values[i][term_alias_value_len - 1] = '\0';
                updated = true;
                break;
            }
        }
        if (!updated) {
            if (term_alias_count >= term_alias_max) {
                term_err("mpe: alias: alias table full\n");
                continue;
            }
            strncpy(term_alias_names[term_alias_count], alias_name, term_alias_name_len - 1);
            term_alias_names[term_alias_count][term_alias_name_len - 1] = '\0';
            strncpy(term_alias_values[term_alias_count], alias_value, term_alias_value_len - 1);
            term_alias_values[term_alias_count][term_alias_value_len - 1] = '\0';
            term_alias_count++;
        }
        term_printf("term_ok", "alias %s='%s'\n", alias_name, alias_value);
    }
}
void cmd_unalias(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: unalias <name>\n");
        return;
    }
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        bool found = false;
        for (int i = 0; i < term_alias_count; i++) {
            if (term_str_eq(argv[argument_index], term_alias_names[i])) {
                int entries_to_move = term_alias_count - i - 1;
                if (entries_to_move > 0) {
                    memmove(term_alias_names[i], term_alias_names[i + 1],
                            (size_t) entries_to_move * sizeof term_alias_names[0]);
                    memmove(term_alias_values[i], term_alias_values[i + 1],
                            (size_t) entries_to_move * sizeof term_alias_values[0]);
                }
                term_alias_names[term_alias_count - 1][0] = '\0';
                term_alias_values[term_alias_count - 1][0] = '\0';
                term_alias_count--;
                found = true;
                term_printf("term_ok", "unalias %s\n", argv[argument_index]);
                break;
            }
        }
        if (!found) {
            term_printf("term_err", "mpe: unalias: %s: not found\n", argv[argument_index]);
        }
    }
}
void cmd_jobs(int argc, char **argv) {
    (void) argc;
    (void) argv;
    int job_count = 0;
    if (long_run_validation_active) {
        int seconds_remaining = long_run_validation_ticks_remaining / 60;
        term_printf(NULL, "[%d]+ Running    long-run validation (%ds remaining / %d total)\n", ++job_count,
                    seconds_remaining, long_run_validation_total_ticks / 60);
    }
    if (!long_run_validation_active) {
        term_dim("(no active jobs)\n");
    }
}
void cmd_lsof(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_printf("term_echo", "COMMAND     TYPE     NAME\n");
    term_printf(NULL, "%-11s %-8s %s\n", "terminal", "win", debug_terminal_is_open() ? "open" : "closed");
    term_printf(NULL, "%-11s %-8s %s\n", "mouse", "lock", main_inputs.is_mouse_locked ? "grabbed" : "released");
    term_printf(NULL, "%-11s %-8s %s\n", "mode", "state", main_inputs.is_debug_mode_active ? "debug" : "game");
    if ((selected_object >= 0) && (selected_object < (physics_world_get_primary()->body_count))) {
        term_printf(NULL, "%-11s %-8s /obj/%d\n", "selection", "obj", selected_object);
    } else {
        term_printf(NULL, "%-11s %-8s %s\n", "selection", "obj", "(none)");
    }
    if (main_inputs.marked_joint_object_index >= 0) {
        term_printf(NULL, "%-11s %-8s /obj/%d\n", "joint_mark", "obj", main_inputs.marked_joint_object_index);
    }
    if (main_inputs.is_menu_open) {
        term_printf(NULL, "%-11s %-8s scene_menu\n", "menu", "open");
    }
    if (main_inputs.spawner_menu_level > 0) {
        term_printf(NULL, "%-11s %-8s spawner_menu (level %d)\n", "menu", "open", main_inputs.spawner_menu_level);
    }
    if (main_inputs.velocity_menu_level > 0) {
        term_printf(NULL, "%-11s %-8s velocity_menu (level %d)\n", "menu", "open", main_inputs.velocity_menu_level);
    }
    if (main_inputs.object_menu_level > 0) {
        term_printf(NULL, "%-11s %-8s object_menu (level %d)\n", "menu", "open", main_inputs.object_menu_level);
    }
    if (config_menu_is_open()) {
        term_printf(NULL, "%-11s %-8s config_menu\n", "menu", "open");
    }
    if (physics_is_halted()) {
        term_printf(NULL, "%-11s %-8s HALTED\n", "physics", "state");
    }
}
void cmd_seq(int argc, char **argv) {
    int first = 1, last = 1;
    if (argc == 2) {
        float v = 0.0f;
        if (term_parse_float(argv[1], &v)) {
            last = (int) v;
        }
    } else if (argc >= 3) {
        float v1 = 0.0f, v2 = 0.0f;
        if (term_parse_float(argv[1], &v1)) {
            first = (int) v1;
        }
        if (term_parse_float(argv[2], &v2)) {
            last = (int) v2;
        }
    }
    if (last < first) {
        int temp = first;
        first = last;
        last = temp;
    }
    int limit = last - first + 1;
    if (limit > 1000) {
        limit = 1000;
        last = first + 999;
    }
    for (int i = first; i <= last; i++) {
        term_printf(NULL, "%d\n", i);
    }
}
void cmd_tee(int argc, char **argv) {
    if (argc < 3) {
        term_err("usage: tee <filename> <command...>\n");
        return;
    }
    const char *output_filename = argv[1];
    /* Jail: only allow writes inside status/ or /tmp/mpe-*. Absolute paths
     * elsewhere or ".." traversal could overwrite engine sources/shaders. */
    bool allowed = false;
    if (strncmp(output_filename, "status/", 7) == 0) {
        allowed = true;
    } else if (strncmp(output_filename, "/tmp/mpe-", 9) == 0) {
        allowed = true;
    } else if (strchr(output_filename, '/') == NULL) {
        /* Bare filename -> sandbox into status/. */
        allowed = true;
    }
    if (strstr(output_filename, "..") != NULL) {
        allowed = false;
    }
    if (!allowed) {
        term_err("mpe: tee: path jailed (use status/<file> or /tmp/mpe-<file>)\n");
        return;
    }
    char sub_command[2048];
    sub_command[0] = '\0';
    size_t offset = 0;
    for (int i = 2; i < argc; i++) {
        if (i > 2) {
            sub_command[offset++] = ' ';
        }
        size_t len = strlen(argv[i]);
        if (offset + len < sizeof(sub_command) - 1) {
            memcpy(sub_command + offset, argv[i], len);
            offset += len;
            sub_command[offset] = '\0';
        }
    }
    term_capture_begin();
    term_execute(sub_command);
    term_capture_end();
    char *captured = term_capture_get();
    if (captured && captured[0] != '\0') {
        char resolved[512];
        if (strchr(output_filename, '/') == NULL) {
            snprintf(resolved, sizeof(resolved), "status/%s", output_filename);
        } else {
            snprintf(resolved, sizeof(resolved), "%s", output_filename);
        }
        FILE *output_file = fopen(resolved, "w");
        if (output_file) {
            fputs(captured, output_file);
            fclose(output_file);
            term_printf("term_ok", "tee: wrote %zu bytes to %s\n", strlen(captured), resolved);
        } else {
            term_printf("term_err", "mpe: tee: %s: cannot open for writing\n", resolved);
        }
    }
    term_capture_reset();
}
void cmd_watch(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: watch <command...>\n");
        return;
    }
    char sub_command[2048];
    sub_command[0] = '\0';
    size_t offset = 0;
    for (int i = 1; i < argc; i++) {
        if (i > 1) {
            sub_command[offset++] = ' ';
        }
        size_t len = strlen(argv[i]);
        if (offset + len < sizeof(sub_command) - 1) {
            memcpy(sub_command + offset, argv[i], len);
            offset += len;
            sub_command[offset] = '\0';
        }
    }
    term_dim("-- watch: single execution (periodic mode deferred) --\n");
    term_execute(sub_command);
}
void cmd_sudo(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: sudo <command...>\n");
        return;
    }
    if (main_inputs.is_debug_mode_active) {
        char sub_command[2048];
        sub_command[0] = '\0';
        size_t offset = 0;
        for (int i = 1; i < argc; i++) {
            if (i > 1) {
                sub_command[offset++] = ' ';
            }
            size_t len = strlen(argv[i]);
            if (offset + len < sizeof(sub_command) - 1) {
                memcpy(sub_command + offset, argv[i], len);
                offset += len;
                sub_command[offset] = '\0';
            }
        }
        term_execute(sub_command);
        return;
    }
    term_dim("[sudo] bypassing game-mode restriction\n");
    char sub_command[2048];
    sub_command[0] = '\0';
    size_t offset = 0;
    for (int i = 1; i < argc; i++) {
        if (i > 1) {
            sub_command[offset++] = ' ';
        }
        size_t len = strlen(argv[i]);
        if (offset + len < sizeof(sub_command) - 1) {
            memcpy(sub_command + offset, argv[i], len);
            offset += len;
            sub_command[offset] = '\0';
        }
    }
    term_sudo_active = true;
    term_execute(sub_command);
    term_sudo_active = false;
}
void cmd_su(int argc, char **argv) {
    (void) argc;
    (void) argv;
    main_inputs.is_debug_mode_active = !main_inputs.is_debug_mode_active;
    debug_terminal_sync_mode();
    if (term_engine_start_time == 0) {
        term_engine_start_time = posix_monotonic_time();
    } /* FIX_029 */
    if (main_inputs.is_debug_mode_active) {
        term_ok("Switched to debug mode.\n");
    } else {
        term_ok("Switched to game mode.\n");
    }
}
void cmd_dmesg(int argc, char **argv) {
    int max_events = 32;
    int filter_level = -1;
    for (int i = 1; i < argc; i++) {
        if (term_str_eq(argv[i], "-n") && (i + 1 < argc)) {
            float v = 0.0f;
            if (term_parse_float(argv[i + 1], &v)) {
                max_events = (int) v;
            }
            i++;
        } else if (term_str_eq(argv[i], "-l") && (i + 1 < argc)) {
            i++;
            if (term_str_eq(argv[i], "info")) {
                filter_level = 0;
            } else if (term_str_eq(argv[i], "warn")) {
                filter_level = 1;
            } else if (term_str_eq(argv[i], "error")) {
                filter_level = 2;
            }
        }
    }
    if (max_events < 1) {
        max_events = 1;
    }
    if (max_events > 256) {
        max_events = 256;
    }
    int total_events = event_log_get_count();
    if (total_events == 0) {
        term_dim("(event log empty)\n");
        return;
    }
    int start_index = total_events - max_events;
    if (start_index < 0) {
        start_index = 0;
    }
    int printed = 0;
    for (int i = start_index; i < total_events; i++) {
        log_level level;
        time_t timestamp;
        const char *message = event_log_get_message(i, &level, &timestamp);
        if (!message) {
            continue;
        }
        if ((filter_level >= 0) && ((int) level < filter_level)) {
            continue;
        }
        struct tm *local_time = localtime(&timestamp);
        char time_buffer[32];
        strftime(time_buffer, sizeof(time_buffer), "%H:%M:%S", local_time);
        const char *level_text = (level == log_info) ? "INFO" : (level == log_warn) ? "WARN" : "ERR ";
        const char *tag_name = (level == log_error) ? "term_err" : (level == log_warn) ? "term_echo" : NULL;
        term_printf(tag_name, "[%s] %s: %s\n", time_buffer, level_text, message);
        printed++;
    }
    if (printed == 0) {
        term_dim("(no matching events)\n");
    } else {
        term_printf("term_dim", "%d event(s) shown\n", printed);
    }
}
/* MPE_TASK_V15R2_PHASE7_IMPL_END */
/* MPE_TASK_V15R2_PHASE8_IMPL_BEGIN */
/* MPE_TASK_V15R2_MICROVIM_FILE_WHITELIST_BEGIN */
typedef struct {
    const char *path;
    const char *description;
} mv_editable_file;
static const mv_editable_file mv_known_files[] = {{"status/engine.cfg", "Main engine configuration (78 tunables)"},
                                                  {"status/engine.cfg.backup", "F10/F11 validation config backup"},
                                                  {"status/engine.cfg.bak", "MicroVim auto-backup (last :w)"},
                                                  {"status/scene.dat", "Scene save file (binary)"},
                                                  /* Project files below resolve relative to the engine working
                                                   * directory (v15R3/src): ../ enters v15R3/, ../../ the repo
                                                   * root. Listed wrongly as bare names before (they opened as
                                                   * [New] buffers and would save strays into src/). */
                                                  {"../../readme.md", "Project README"},
                                                  {"../evolution.txt", "Version lineage (stages to v15R3 dev)"},
                                                  {"../how_to_use.md", "User guide / controls reference"},
                                                  {"../RELEASE_POLICY.md", "Release cycle rules"},
                                                  {"../RELEASE_GATES.md", "P0/P1/P2 gate checklist"},
                                                  {"../../scope.md", "Defect & debt audit"},
                                                  {"../../LICENSE", "GPL-3.0 license text"},
                                                  {"../../.gitignore", "Git ignore rules"},
                                                  {"../validation/V01.sh", "Sanitizer build script"},
                                                  {"../validation/V02.sh", "Clean build + warning review"},
                                                  {"../validation/V03.py", "P0 gate interactive walk"},
                                                  {"../validation/V04.sh", "F10 long-run validation guide"},
                                                  {"makefile", "Build system makefile"},
                                                  {"compile", "Compile script"},
                                                  {"config/mpe_constants.h", "Compile-time constants manifest"},
                                                  {"config/mpe_config.h", "Config API header"},
                                                  {"config/mpe_config_schema.c", "Config parameter registry"},
                                                  {"config/mpe_config.c", "Config implementation"},
                                                  {"ui_input/microvim.h", "MicroVim header"},
                                                  {"ui_input/microvim.c", "MicroVim implementation"},
                                                  {"ui_input/debug_terminal.h", "Terminal header"},
                                                  {"ui_input/debug_terminal.c", "Terminal implementation"},
                                                  {"render/shaders/vertex_shader.glsl", "Vertex shader"},
                                                  {"render/shaders/fragment_shader.glsl", "Fragment shader"},
                                                  {"render/shaders/utility_vertex.glsl", "Utility vertex shader"},
                                                  {"render/shaders/utility_fragment.glsl", "Utility fragment shader"},
                                                  {"render/shaders/axis_vertex.glsl", "Axis vertex shader"},
                                                  {"render/shaders/axis_fragment.glsl", "Axis fragment shader"},
                                                  {NULL, NULL}};
static const char *mv_allowed_extensions[] = {".cfg", ".ini", ".conf", ".txt", ".md", ".glsl",
                                              ".sh",  ".py",  ".h",    ".c",   NULL};
static const char *mv_blocked_extensions[] = {".dat", ".o",   ".so",  ".a",     ".bin",
                                              ".exe", ".obj", ".dll", ".dylib", NULL};
bool mv_file_is_allowed(const char *filepath) {
    if (!filepath || filepath[0] == '\0') {
        return false;
    }
    /* Reject absolute paths */
    if (filepath[0] == '/') {
        return false;
    }
    /* Reject paths with null bytes (defensive) */
    /* Check blocked extensions first */
    const char *dot = strrchr(filepath, '.');
    if (dot) {
        for (int i = 0; mv_blocked_extensions[i]; i++) {
            if (strcasecmp(dot, mv_blocked_extensions[i]) == 0) {
                return false;
            }
        }
    }
    /* Check if it's in the known files list */
    for (int i = 0; mv_known_files[i].path; i++) {
        if (strcmp(filepath, mv_known_files[i].path) == 0) {
            return true;
        }
    }
    /* Check if it has an allowed extension */
    if (dot) {
        for (int i = 0; mv_allowed_extensions[i]; i++) {
            if (strcasecmp(dot, mv_allowed_extensions[i]) == 0) {
                return true;
            }
        }
    }
    return false;
}
/* MPE_TASK_V15R2_MICROVIM_FILE_WHITELIST_END */
void cmd_vi(int argc, char **argv) {
    /* Handle --list / -l flag */
    if ((argc > 1) && (term_str_eq(argv[1], "--list") || term_str_eq(argv[1], "-l"))) {
        term_printf("term_echo", "MicroVim editable files:\n");
        term_out("\n");
        for (int i = 0; mv_known_files[i].path; i++) {
            term_printf(NULL, "  %-40s %s\n", mv_known_files[i].path, mv_known_files[i].description);
        }
        term_out("\n");
        term_dim("Also allowed: any file with extensions: .cfg .ini .conf .txt .md .glsl .sh .py .h .c\n");
        term_dim("Blocked: .dat .o .so .a .bin .exe and other binary formats\n");
        term_out("\n");
        term_dim("Usage: vi <path>  |  vi --list  |  vi --help\n");
        return;
    }
    /* Handle --help / -h flag */
    if ((argc > 1) && (term_str_eq(argv[1], "--help") || term_str_eq(argv[1], "-h"))) {
        term_printf("term_echo", "MicroVim — minimal modal editor\n");
        term_out("\n");
        term_out("  vi [path]       Open file (default: status/engine.cfg)\n");
        term_out("  vi --list       Show editable files\n");
        term_out("  vi --help       This help\n");
        term_out("\n");
        term_out("  Modes: Normal (default), Insert (i/a/o), Command (:)\n");
        term_out("  Nav:   h j k l  w b e  0 $  gg G  Ctrl+F/B  { }\n");
        term_out("  Edit:  x dd dw d$  yy p P  u Ctrl+R  J  ~  cc cw C S\n");
        term_out("  Cmd:   :w :q :q! :wq :x :e <file> :N :set nu :s/o/n/g\n");
        term_out("  Exit:  :q or :wq or double-Escape in Normal mode\n");
        return;
    }
    const char *target_file = "status/engine.cfg";
    if (argc > 1) {
        target_file = argv[1];
    }
    /* Validate file against whitelist */
    if (!mv_file_is_allowed(target_file)) {
        term_printf("term_err", "mpe: vi: %s: not an editable file\n", target_file);
        term_dim("Use 'vi --list' to see editable files.\n");
        return;
    }
    /* Prevent opening while microvim is already active */
    if (microvim_is_active()) {
        term_err("mpe: vi: editor already open (close it first with :q or Esc Esc)\n");
        return;
    }
    microvim_open(target_file);
    if (terminal_entry) {
        gtk_widget_set_visible(terminal_entry, FALSE);
    }
    if (terminal_prompt_label) {
        gtk_widget_set_visible(terminal_prompt_label, FALSE);
    }
    term_printf("term_echo", "MicroVim opened: %s\n", target_file);
    term_dim("Modes: Normal/Insert/Command. Esc=Normal, i=Insert, :=Command.\n");
    term_dim("Save: :w  Quit: :q  Save+Quit: :wq  Force quit: :q!  Exit: Esc Esc\n");
    if (terminal_output_buffer) {
        microvim_render(terminal_output_buffer);
    }
}
/* MPE_TASK_V15R2_PHASE8_IMPL_END */
#else
/* term_admin.c — Admin/batch/scene/shell commands: sed..dmesg + vi.
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
#include <strings.h>

static gint64 posix_monotonic_time(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (gint64)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
}
bool all_targets_matched_any(int argc, char **argv) {
    for (int i = 2; i < argc; i++) {
        if (term_is_all_token(argv[i])) {
            return true;
        }
    }
    return false;
}
void cmd_sed(int argc, char **argv) {
    if (argc < 3) {
        term_err("usage: sed s/field/value/ <target...>\n");
        return;
    }
    const char *expression = argv[1];
    if ((expression[0] != 's') || (expression[1] != '/')) {
        term_err("mpe: sed: expression must start with s/\n");
        return;
    }
    const char *field_start = expression + 2;
    const char *field_end = strchr(field_start, '/');
    if (!field_end) {
        term_err("mpe: sed: malformed expression (missing second /)\n");
        return;
    }
    char field_name[64];
    int field_len = (int) (field_end - field_start);
    if (field_len >= 64) {
        field_len = 63;
    }
    strncpy(field_name, field_start, field_len);
    field_name[field_len] = '\0';
    const char *value_start = field_end + 1;
    const char *value_end = strchr(value_start, '/');
    char value_str[64];
    if (value_end) {
        int value_len = (int) (value_end - value_start);
        if (value_len >= 64) {
            value_len = 63;
        }
        strncpy(value_str, value_start, value_len);
        value_str[value_len] = '\0';
    } else {
        strncpy(value_str, value_start, 63);
        value_str[63] = '\0';
    }
    float new_value = 0.0f;
    bool is_numeric = term_parse_float(value_str, &new_value);
    bool make_static = term_str_eq(value_str, "static") || term_str_eq(value_str, "1");
    bool make_dynamic = term_str_eq(value_str, "dynamic") || term_str_eq(value_str, "0");
    int modified_count = 0;
    for (int argument_index = 2; argument_index < argc; argument_index++) {
        const char *target = argv[argument_index];
        bool all_targets = term_is_all_token(target);
        if (all_targets) {
            for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
                rigidbody *rb = &(physics_world_get_primary()->bodies)[object_index];
                if (term_str_eq(field_name, "mass")) {
                    if (!is_numeric) {
                        continue;
                    }
                    term_set_object_mass(object_index, new_value);
                } else if (term_str_eq(field_name, "radius")) {
                    if ((!is_numeric) || (rb->type != object_sphere)) {
                        continue;
                    }
                    rb->radius = new_value;
                    rigidbody_update_inertia_sphere(rb);
                    rigidbody_wake(rb);
                } else if (term_str_eq(field_name, "friction_static") || term_str_eq(field_name, "fs")) {
                    if (!is_numeric) {
                        continue;
                    }
                    rb->friction_static = new_value;
                } else if (term_str_eq(field_name, "friction_kinetic") || term_str_eq(field_name, "fk")) {
                    if (!is_numeric) {
                        continue;
                    }
                    rb->friction_kinetic = new_value;
                } else if (term_str_eq(field_name, "restitution") || term_str_eq(field_name, "rest")) {
                    if (!is_numeric) {
                        continue;
                    }
                    rb->restitution = new_value;
                    if (rb->restitution < 0.0f) {
                        rb->restitution = 0.0f;
                    }
                    if (rb->restitution > 1.0f) {
                        rb->restitution = 1.0f;
                    }
                } else if (term_str_eq(field_name, "static")) {
                    term_set_object_static(object_index, make_static || (!make_dynamic));
                } else if (term_str_eq(field_name, "dynamic")) {
                    term_set_object_static(object_index, !make_dynamic);
                } else if (term_str_eq(field_name, "nice")) {
                    if (!is_numeric) {
                        continue;
                    }
                    int nice_val = (int) new_value;
                    if (nice_val < -20) {
                        nice_val = -20;
                    }
                    if (nice_val > 19) {
                        nice_val = 19;
                    }
                    rb->nice_value = nice_val;
                }
                modified_count++;
            }
            contact_cache_clear(physics_world_get_primary());
            continue;
        }
        int object_index = term_object_from_token(target);
        if (object_index < 0) {
            term_printf("term_err", "mpe: sed: %s: No such object\n", target);
            continue;
        }
        rigidbody *rb = &(physics_world_get_primary()->bodies)[object_index];
        if (term_str_eq(field_name, "mass")) {
            if (!is_numeric) {
                term_err("mpe: sed: mass requires numeric value\n");
                continue;
            }
            term_set_object_mass(object_index, new_value);
        } else if (term_str_eq(field_name, "radius")) {
            if (!is_numeric) {
                term_err("mpe: sed: radius requires numeric value\n");
                continue;
            }
            if (rb->type != object_sphere) {
                term_err("mpe: sed: radius only applies to spheres\n");
                continue;
            }
            rb->radius = new_value;
            rigidbody_update_inertia_sphere(rb);
            rigidbody_wake(rb);
        } else if (term_str_eq(field_name, "friction_static") || term_str_eq(field_name, "fs")) {
            if (!is_numeric) {
                term_err("mpe: sed: friction requires numeric value\n");
                continue;
            }
            rb->friction_static = new_value;
        } else if (term_str_eq(field_name, "friction_kinetic") || term_str_eq(field_name, "fk")) {
            if (!is_numeric) {
                term_err("mpe: sed: friction requires numeric value\n");
                continue;
            }
            rb->friction_kinetic = new_value;
        } else if (term_str_eq(field_name, "restitution") || term_str_eq(field_name, "rest")) {
            if (!is_numeric) {
                term_err("mpe: sed: restitution requires numeric value\n");
                continue;
            }
            rb->restitution = new_value;
            if (rb->restitution < 0.0f) {
                rb->restitution = 0.0f;
            }
            if (rb->restitution > 1.0f) {
                rb->restitution = 1.0f;
            }
        } else if (term_str_eq(field_name, "static")) {
            term_set_object_static(object_index, true);
        } else if (term_str_eq(field_name, "dynamic")) {
            term_set_object_static(object_index, false);
        } else if (term_str_eq(field_name, "nice")) {
            if (!is_numeric) {
                term_err("mpe: sed: nice requires numeric value\n");
                continue;
            }
            int nice_val = (int) new_value;
            if (nice_val < -20) {
                nice_val = -20;
            }
            if (nice_val > 19) {
                nice_val = 19;
            }
            rb->nice_value = nice_val;
        } else {
            term_printf("term_err", "mpe: sed: unknown field '%s'\n", field_name);
            continue;
        }
        modified_count++;
        contact_cache_clear(physics_world_get_primary());
        term_printf("term_ok", "/obj/%d: %s=%s\n", object_index, field_name, value_str);
    }
    if (all_targets_matched_any(argc, argv)) {
        term_printf("term_ok", "sed: modified %d object(s)\n", modified_count);
    }
}

void cmd_nice(int argc, char **argv) {
    if (argc < 3) {
        term_err("usage: nice <priority> <object...>\n");
        return;
    }
    float priority_float = 0.0f;
    if (!term_parse_float(argv[1], &priority_float)) {
        term_printf("term_err", "mpe: nice: invalid priority '%s'\n", argv[1]);
        return;
    }
    int priority = (int) priority_float;
    if (priority < -20) {
        priority = -20;
    }
    if (priority > 19) {
        priority = 19;
    }
    for (int argument_index = 2; argument_index < argc; argument_index++) {
        int object_index = term_require_object(argv[argument_index]);
        if (object_index < 0) {
            continue;
        }
        (physics_world_get_primary()->bodies)[object_index].nice_value = priority;
        rigidbody_wake(&(physics_world_get_primary()->bodies)[object_index]);
        term_printf("term_ok", "/obj/%d nice=%d\n", object_index, priority);
    }
}
void cmd_renice(int argc, char **argv) {
    cmd_nice(argc, argv);
}
void cmd_ping(int argc, char **argv) {
    int ping_count = 1;
    int argument_index = 1;
    if ((argc > 1) && (term_str_eq(argv[1], "-c")) && (argc > 2)) {
        float v = 0.0f;
        if (term_parse_float(argv[2], &v)) {
            ping_count = (int) v;
        }
        if (ping_count < 1) {
            ping_count = 1;
        }
        if (ping_count > 10) {
            ping_count = 10;
        }
        argument_index = 3;
    }
    if (argument_index >= argc) {
        term_err("usage: ping [-c count] <object>\n");
        return;
    }
    int object_index = term_require_object(argv[argument_index]);
    if (object_index < 0) {
        return;
    }
    rigidbody *rb = &(physics_world_get_primary()->bodies)[object_index];
    if (rb->static_state) {
        term_printf("term_dim", "PING /obj/%d: no response (static)\n", object_index);
        return;
    }
    if (rb->is_sleeping) {
        term_printf("term_dim", "PING /obj/%d: no response (sleeping)\n", object_index);
        return;
    }
    for (int ping_index = 0; ping_index < ping_count; ping_index++) {
        vector3 micro_impulse = vector3_scaling(rb->velocity, 0.0f);
        micro_impulse = (vector3){0.001f, 0.001f, 0.001f};
        rb->velocity = vector3_addition(rb->velocity, micro_impulse);
    }
    float speed_delta = 0.001f * sqrtf(3.0f) * (float) ping_count;
    term_printf("term_ok", "PING /obj/%d: %d ping(s), velocity delta %.6f m/s, state=%s\n", object_index, ping_count,
                speed_delta, rb->is_sleeping ? "sleep" : "run");
}
void cmd_mount(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: mount <path>\n");
        return;
    }
    const char *scene_path = argv[1];
    if (scene_loading(scene_path)) {
        editor_reset();
        contact_cache_clear(physics_world_get_primary());
        term_printf("term_ok", "mounted %s: %d objects loaded\n", scene_path, (physics_world_get_primary()->body_count));
        event_log_push(log_info, "Scene mounted via terminal: %s", scene_path);
    } else {
        term_printf("term_err", "mpe: mount: %s: failed to load\n", scene_path);
    }
}
void cmd_umount(int argc, char **argv) {
    (void) argc;
    (void) argv;
    int save_result = save_scene("status/scene.dat");
    if (save_result) {
        term_ok("umount: scene saved to status/scene.dat\n");
    } else {
        term_err("mpe: umount: save failed\n");
    }
    scene_clear();
    clear_selection();
    contact_cache_clear(physics_world_get_primary());
    editor_reset();
    term_ok("umount: scene cleared\n");
    event_log_push(log_info, "Scene unmounted via terminal");
}
void cmd_mkfs(int argc, char **argv) {
    (void) argc;
    (void) argv;
    scene_clear();
    clear_selection();
    contact_cache_clear(physics_world_get_primary());
    editor_reset();
    term_printf("term_ok", "Scene formatted. 0 objects, 0 joints.\n");
    event_log_push(log_info, "Scene formatted via terminal (mkfs)");
}
void cmd_fsck(int argc, char **argv) {
    bool auto_fix = false;
    for (int i = 1; i < argc; i++) {
        if (term_str_eq(argv[i], "-y")) {
            auto_fix = true;
        }
    }
    int error_count = 0;
    int warning_count = 0;
    term_printf("term_echo", "fsck: checking %d objects...\n", (physics_world_get_primary()->body_count));
    for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
        rigidbody *rb = &(physics_world_get_primary()->bodies)[object_index];
        bool has_error = false;
        if ((!isfinite(rb->position.x)) || (!isfinite(rb->position.y)) || (!isfinite(rb->position.z))) {
            term_printf("term_err", "  /obj/%d: position NaN/Inf\n", object_index);
            has_error = true;
        }
        if ((!isfinite(rb->velocity.x)) || (!isfinite(rb->velocity.y)) || (!isfinite(rb->velocity.z))) {
            term_printf("term_err", "  /obj/%d: velocity NaN/Inf\n", object_index);
            has_error = true;
        }
        if ((!isfinite(rb->angular_velocity.x)) || (!isfinite(rb->angular_velocity.y)) ||
            (!isfinite(rb->angular_velocity.z))) {
            term_printf("term_err", "  /obj/%d: angular_velocity NaN/Inf\n", object_index);
            has_error = true;
        }
        if ((!isfinite(rb->orientation.w)) || (!isfinite(rb->orientation.x)) || (!isfinite(rb->orientation.y)) ||
            (!isfinite(rb->orientation.z))) {
            term_printf("term_err", "  /obj/%d: orientation NaN/Inf\n", object_index);
            has_error = true;
        }
        if ((!rb->static_state) && ((rb->mass <= 0.0f) || (!isfinite(rb->mass)))) {
            term_printf("term_err", "  /obj/%d: invalid mass %.4f (dynamic)\n", object_index, rb->mass);
            has_error = true;
        }
        if ((!rb->static_state) && (rb->inverse_mass <= 0.0f)) {
            term_printf("term_dim", "  /obj/%d: warning: inverse_mass=%.4f (dynamic)\n", object_index,
                        rb->inverse_mass);
            warning_count++;
        }
        float orient_len_sq = rb->orientation.w * rb->orientation.w + rb->orientation.x * rb->orientation.x +
                              rb->orientation.y * rb->orientation.y + rb->orientation.z * rb->orientation.z;
        if ((orient_len_sq < 0.9f) || (orient_len_sq > 1.1f)) {
            term_printf("term_dim", "  /obj/%d: warning: orientation not normalized (|q|^2=%.4f)\n", object_index,
                        orient_len_sq);
            warning_count++;
        }
        if (has_error) {
            error_count++;
            if (auto_fix) {
                rigidbody_sanitize(rb);
                rigidbody_wake(rb); /* FIX_032 */
                term_printf("term_ok", "  /obj/%d: sanitized\n", object_index);
            }
        }
    }
    term_printf("term_echo", "fsck: checking %d joint slots...\n", mpe_max_joints);
    for (int joint_index = 0; joint_index < mpe_max_joints; joint_index++) {
        if (!(physics_world_get_primary()->spring_joints)[joint_index].is_active) {
            continue;
        }
        spring_joint *j = &(physics_world_get_primary()->spring_joints)[joint_index];
        int index_a = scene_find_object_index_by_id(j->object_id_a);
        int index_b = scene_find_object_index_by_id(j->object_id_b);
        if (index_a < 0) {
            term_printf("term_err", "  /joint/%d: object_a (id=%u) not found\n", joint_index, j->object_id_a);
            error_count++;
            if (auto_fix) {
                remove_joint(physics_world_get_primary(), joint_index);
                term_printf("term_ok", "  /joint/%d: removed\n", joint_index);
            }
            continue;
        }
        if (index_b < 0) {
            term_printf("term_err", "  /joint/%d: object_b (id=%u) not found\n", joint_index, j->object_id_b);
            error_count++;
            if (auto_fix) {
                remove_joint(physics_world_get_primary(), joint_index);
                term_printf("term_ok", "  /joint/%d: removed\n", joint_index);
            }
            continue;
        }
        if ((j->equilibrium_length < 0.0f) || (!isfinite(j->equilibrium_length))) {
            term_printf("term_err", "  /joint/%d: invalid rest length %.4f\n", joint_index, j->equilibrium_length);
            error_count++;
        }
        if ((j->spring_constant <= 0.0f) || (!isfinite(j->spring_constant))) {
            term_printf("term_err", "  /joint/%d: invalid spring constant %.4f\n", joint_index, j->spring_constant);
            error_count++;
        }
    }
    if (auto_fix) {
        contact_cache_clear(physics_world_get_primary());
    }
    if (error_count == 0) {
        term_printf("term_ok", "fsck: PASS — %d objects, %d joints, %d warning(s), 0 errors\n", (physics_world_get_primary()->body_count),
                    (physics_world_get_primary()->spring_joint_count), warning_count);
    } else {
        term_printf("term_err", "fsck: FAIL — %d error(s), %d warning(s)%s\n", error_count, warning_count,
                    auto_fix ? " (auto-fixed)" : " (run fsck -y to fix)");
    }
    event_log_push(error_count == 0 ? log_info : log_warn, "fsck: %d errors, %d warnings%s", error_count, warning_count,
                   auto_fix ? " (fixed)" : "");
}
/* MPE_TASK_V15R2_PHASE5_IMPL_END */
/* MPE_TASK_V15R2_PHASE6_IMPL_BEGIN */
void cmd_netstat(int argc, char **argv) {
    bool show_all = false;
    for (int i = 1; i < argc; i++) {
        if (term_str_eq(argv[i], "-a")) {
            show_all = true;
        }
    }
    term_printf("term_echo", "Active Joints (spring connections)\n");
    term_printf(NULL, "Proto  Local        Foreign      State         K        D      Len\n");
    int listed = 0;
    for (int ji = 0; ji < mpe_max_joints; ji++) {
        if ((!(physics_world_get_primary()->spring_joints)[ji].is_active) && (!show_all)) {
            continue;
        }
        int ia = scene_find_object_index_by_id((physics_world_get_primary()->spring_joints)[ji].object_id_a);
        int ib = scene_find_object_index_by_id((physics_world_get_primary()->spring_joints)[ji].object_id_b);
        const char *state_text = (physics_world_get_primary()->spring_joints)[ji].is_active ? "ESTABLISHED" : "CLOSED";
        term_printf(NULL, "spring /obj/%-6d /obj/%-6d %-12s %7.1f %7.1f %7.2f\n", ia, ib, state_text,
                    (physics_world_get_primary()->spring_joints)[ji].spring_constant, (physics_world_get_primary()->spring_joints)[ji].damping_coefficient,
                    (physics_world_get_primary()->spring_joints)[ji].equilibrium_length);
        listed++;
    }
    if (listed == 0) {
        term_dim("(no connections)\n");
    } else {
        term_printf("term_dim", "%d connection(s) active\n", listed);
    }
}
void cmd_ifconfig(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_printf("term_echo", "camera0:  flags=<%s>  mode %s\n", main_inputs.is_mouse_locked ? "LOCKED" : "FREE",
                main_inputs.is_debug_mode_active ? "DEBUG" : "GAME");
    term_printf(NULL, "    position (%.2f, %.2f, %.2f)\n", main_camera_fov.position.x, main_camera_fov.position.y,
                main_camera_fov.position.z);
    term_printf(NULL, "    yaw %.2f  pitch %.2f  speed %.2f m/s\n", main_camera_fov.yaw, main_camera_fov.pitch,
                main_camera_fov.movement_speed);
    term_printf(NULL, "    sensitivity %.3f  steer %.3f\n", g_cfg.camera.mouse_sensitivity,
                g_cfg.camera.steer_sensitivity);
    term_out("\n");
    term_printf("term_echo", "render0:  flags=<ACTIVE>\n");
    term_printf(NULL, "    light (%.1f, %.1f, %.1f)\n", g_cfg.render.light_x, g_cfg.render.light_y,
                g_cfg.render.light_z);
    term_printf(NULL, "    ambient %.2f  specular %.2f  exponent %.1f\n", g_cfg.render.ambient_strength,
                g_cfg.render.specular_coeff, g_cfg.render.specular_exponent);
    term_out("\n");
    term_printf("term_echo", "input0:  flags=<%s>\n", main_inputs.is_mouse_locked ? "GRABBED" : "RELEASED");
    term_printf(NULL, "    spawn=%s  selected=%d  marked_joint=%d\n",
                term_spawn_type_name(), selected_object,
                main_inputs.marked_joint_object_index);
    term_printf(NULL, "    objects=%d  joints=%d  sleeping=%d\n", (physics_world_get_primary()->body_count), (physics_world_get_primary()->spring_joint_count),
                debug_last_sleeping_object_count);
}
void cmd_lsmod(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_printf(NULL, "%-24s %6s  %s\n", "Module", "Size", "Used by");
    term_printf(NULL, "%-24s %6s  %s\n", "instanced_shader", "1", "sphere_mesh, cube_mesh");
    term_printf(NULL, "%-24s %6s  %s\n", "utility_shader", "1", "grid, wireframe, joints");
    term_printf(NULL, "%-24s %6s  %s\n", "sphere_mesh", "1", "instanced_shader");
    term_printf(NULL, "%-24s %6s  %s\n", "cube_mesh", "1", "instanced_shader");
    term_printf(NULL, "%-24s %6s  %s\n", "grid_mesh", "1", "utility_shader");
    term_printf(NULL, "%-24s %6s  %s\n", "wireframe_renderer", "1", "utility_shader");
    term_printf(NULL, "%-24s %6s  %s\n", "joint_renderer", "1", "utility_shader");
    term_printf(NULL, "%-24s %6s  %s\n", "spatial_hash_bp", "1", "collision_pipeline");
    term_printf(NULL, "%-24s %6s  %s\n", "contact_cache", "1", "impulse_solver");
    term_printf(NULL, "%-24s %6s  %s\n", "sequential_solver", "1", "physics_step");
    term_printf(NULL, "%-24s %6s  %s\n", "depenetration_pass", "1", "physics_step");
    term_printf(NULL, "%-24s %6s  %s\n", "sleep_system", "1", "physics_step, broadphase");
    term_printf(NULL, "%-24s %6s  %s\n", "config_registry", "1", "config_menu, terminal, F9");
    term_printf(NULL, "%-24s %6s  %s\n", "event_log", "1", "dmesg (pending)");
    term_printf(NULL, "%-24s %6s  %s\n", "debug_terminal", "1", "input_control");
}
/* MPE_TASK_V15R2_PHASE6_IMPL_END */
/* MPE_TASK_V15R2_PHASE7_IMPL_BEGIN */
void cmd_alias(int argc, char **argv) {
    if (argc < 2) {
        if (term_alias_count == 0) {
            term_dim("(no aliases defined)\n");
            return;
        }
        for (int i = 0; i < term_alias_count; i++) {
            term_printf(NULL, "alias %s='%s'\n", term_alias_names[i], term_alias_values[i]);
        }
        return;
    }
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        char *equals_pos = strchr(argv[argument_index], '=');
        if (!equals_pos) {
            bool found = false;
            for (int i = 0; i < term_alias_count; i++) {
                if (term_str_eq(argv[argument_index], term_alias_names[i])) {
                    term_printf(NULL, "alias %s='%s'\n", term_alias_names[i], term_alias_values[i]);
                    found = true;
                    break;
                }
            }
            if (!found) {
                term_printf("term_err", "mpe: alias: %s: not found\n", argv[argument_index]);
            }
            continue;
        }
        *equals_pos = '\0';
        const char *alias_name = argv[argument_index];
        const char *alias_value = equals_pos + 1;
        bool updated = false;
        for (int i = 0; i < term_alias_count; i++) {
            if (term_str_eq(alias_name, term_alias_names[i])) {
                strncpy(term_alias_values[i], alias_value, term_alias_value_len - 1);
                term_alias_values[i][term_alias_value_len - 1] = '\0';
                updated = true;
                break;
            }
        }
        if (!updated) {
            if (term_alias_count >= term_alias_max) {
                term_err("mpe: alias: alias table full\n");
                continue;
            }
            strncpy(term_alias_names[term_alias_count], alias_name, term_alias_name_len - 1);
            term_alias_names[term_alias_count][term_alias_name_len - 1] = '\0';
            strncpy(term_alias_values[term_alias_count], alias_value, term_alias_value_len - 1);
            term_alias_values[term_alias_count][term_alias_value_len - 1] = '\0';
            term_alias_count++;
        }
        term_printf("term_ok", "alias %s='%s'\n", alias_name, alias_value);
    }
}
void cmd_unalias(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: unalias <name>\n");
        return;
    }
    for (int argument_index = 1; argument_index < argc; argument_index++) {
        bool found = false;
        for (int i = 0; i < term_alias_count; i++) {
            if (term_str_eq(argv[argument_index], term_alias_names[i])) {
                int entries_to_move = term_alias_count - i - 1;
                if (entries_to_move > 0) {
                    memmove(term_alias_names[i], term_alias_names[i + 1],
                            (size_t) entries_to_move * sizeof term_alias_names[0]);
                    memmove(term_alias_values[i], term_alias_values[i + 1],
                            (size_t) entries_to_move * sizeof term_alias_values[0]);
                }
                term_alias_names[term_alias_count - 1][0] = '\0';
                term_alias_values[term_alias_count - 1][0] = '\0';
                term_alias_count--;
                found = true;
                term_printf("term_ok", "unalias %s\n", argv[argument_index]);
                break;
            }
        }
        if (!found) {
            term_printf("term_err", "mpe: unalias: %s: not found\n", argv[argument_index]);
        }
    }
}
void cmd_jobs(int argc, char **argv) {
    (void) argc;
    (void) argv;
    int job_count = 0;
    if (long_run_validation_active) {
        int seconds_remaining = long_run_validation_ticks_remaining / 60;
        term_printf(NULL, "[%d]+ Running    long-run validation (%ds remaining / %d total)\n", ++job_count,
                    seconds_remaining, long_run_validation_total_ticks / 60);
    }
    if (!long_run_validation_active) {
        term_dim("(no active jobs)\n");
    }
}
void cmd_lsof(int argc, char **argv) {
    (void) argc;
    (void) argv;
    term_printf("term_echo", "COMMAND     TYPE     NAME\n");
    term_printf(NULL, "%-11s %-8s %s\n", "terminal", "win", debug_terminal_is_open() ? "open" : "closed");
    term_printf(NULL, "%-11s %-8s %s\n", "mouse", "lock", main_inputs.is_mouse_locked ? "grabbed" : "released");
    term_printf(NULL, "%-11s %-8s %s\n", "mode", "state", main_inputs.is_debug_mode_active ? "debug" : "game");
    if ((selected_object >= 0) && (selected_object < (physics_world_get_primary()->body_count))) {
        term_printf(NULL, "%-11s %-8s /obj/%d\n", "selection", "obj", selected_object);
    } else {
        term_printf(NULL, "%-11s %-8s %s\n", "selection", "obj", "(none)");
    }
    if (main_inputs.marked_joint_object_index >= 0) {
        term_printf(NULL, "%-11s %-8s /obj/%d\n", "joint_mark", "obj", main_inputs.marked_joint_object_index);
    }
    if (main_inputs.is_menu_open) {
        term_printf(NULL, "%-11s %-8s scene_menu\n", "menu", "open");
    }
    if (main_inputs.spawner_menu_level > 0) {
        term_printf(NULL, "%-11s %-8s spawner_menu (level %d)\n", "menu", "open", main_inputs.spawner_menu_level);
    }
    if (main_inputs.velocity_menu_level > 0) {
        term_printf(NULL, "%-11s %-8s velocity_menu (level %d)\n", "menu", "open", main_inputs.velocity_menu_level);
    }
    if (main_inputs.object_menu_level > 0) {
        term_printf(NULL, "%-11s %-8s object_menu (level %d)\n", "menu", "open", main_inputs.object_menu_level);
    }
    if (config_menu_is_open()) {
        term_printf(NULL, "%-11s %-8s config_menu\n", "menu", "open");
    }
    if (physics_is_halted()) {
        term_printf(NULL, "%-11s %-8s HALTED\n", "physics", "state");
    }
}
void cmd_seq(int argc, char **argv) {
    int first = 1, last = 1;
    if (argc == 2) {
        float v = 0.0f;
        if (term_parse_float(argv[1], &v)) {
            last = (int) v;
        }
    } else if (argc >= 3) {
        float v1 = 0.0f, v2 = 0.0f;
        if (term_parse_float(argv[1], &v1)) {
            first = (int) v1;
        }
        if (term_parse_float(argv[2], &v2)) {
            last = (int) v2;
        }
    }
    if (last < first) {
        int temp = first;
        first = last;
        last = temp;
    }
    int limit = last - first + 1;
    if (limit > 1000) {
        limit = 1000;
        last = first + 999;
    }
    for (int i = first; i <= last; i++) {
        term_printf(NULL, "%d\n", i);
    }
}
void cmd_tee(int argc, char **argv) {
    if (argc < 3) {
        term_err("usage: tee <filename> <command...>\n");
        return;
    }
    const char *output_filename = argv[1];
    /* Jail: only allow writes inside status/ or /tmp/mpe-*. Absolute paths
     * elsewhere or ".." traversal could overwrite engine sources/shaders. */
    bool allowed = false;
    if (strncmp(output_filename, "status/", 7) == 0) {
        allowed = true;
    } else if (strncmp(output_filename, "/tmp/mpe-", 9) == 0) {
        allowed = true;
    } else if (strchr(output_filename, '/') == NULL) {
        /* Bare filename -> sandbox into status/. */
        allowed = true;
    }
    if (strstr(output_filename, "..") != NULL) {
        allowed = false;
    }
    if (!allowed) {
        term_err("mpe: tee: path jailed (use status/<file> or /tmp/mpe-<file>)\n");
        return;
    }
    char sub_command[2048];
    sub_command[0] = '\0';
    size_t offset = 0;
    for (int i = 2; i < argc; i++) {
        if (i > 2) {
            sub_command[offset++] = ' ';
        }
        size_t len = strlen(argv[i]);
        if (offset + len < sizeof(sub_command) - 1) {
            memcpy(sub_command + offset, argv[i], len);
            offset += len;
            sub_command[offset] = '\0';
        }
    }
    term_capture_begin();
    term_execute(sub_command);
    term_capture_end();
    char *captured = term_capture_get();
    if (captured && captured[0] != '\0') {
        char resolved[512];
        if (strchr(output_filename, '/') == NULL) {
            snprintf(resolved, sizeof(resolved), "status/%s", output_filename);
        } else {
            snprintf(resolved, sizeof(resolved), "%s", output_filename);
        }
        FILE *output_file = fopen(resolved, "w");
        if (output_file) {
            fputs(captured, output_file);
            fclose(output_file);
            term_printf("term_ok", "tee: wrote %zu bytes to %s\n", strlen(captured), resolved);
        } else {
            term_printf("term_err", "mpe: tee: %s: cannot open for writing\n", resolved);
        }
    }
    term_capture_reset();
}
void cmd_watch(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: watch <command...>\n");
        return;
    }
    char sub_command[2048];
    sub_command[0] = '\0';
    size_t offset = 0;
    for (int i = 1; i < argc; i++) {
        if (i > 1) {
            sub_command[offset++] = ' ';
        }
        size_t len = strlen(argv[i]);
        if (offset + len < sizeof(sub_command) - 1) {
            memcpy(sub_command + offset, argv[i], len);
            offset += len;
            sub_command[offset] = '\0';
        }
    }
    term_dim("-- watch: single execution (periodic mode deferred) --\n");
    term_execute(sub_command);
}
void cmd_sudo(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: sudo <command...>\n");
        return;
    }
    if (main_inputs.is_debug_mode_active) {
        char sub_command[2048];
        sub_command[0] = '\0';
        size_t offset = 0;
        for (int i = 1; i < argc; i++) {
            if (i > 1) {
                sub_command[offset++] = ' ';
            }
            size_t len = strlen(argv[i]);
            if (offset + len < sizeof(sub_command) - 1) {
                memcpy(sub_command + offset, argv[i], len);
                offset += len;
                sub_command[offset] = '\0';
            }
        }
        term_execute(sub_command);
        return;
    }
    term_dim("[sudo] bypassing game-mode restriction\n");
    char sub_command[2048];
    sub_command[0] = '\0';
    size_t offset = 0;
    for (int i = 1; i < argc; i++) {
        if (i > 1) {
            sub_command[offset++] = ' ';
        }
        size_t len = strlen(argv[i]);
        if (offset + len < sizeof(sub_command) - 1) {
            memcpy(sub_command + offset, argv[i], len);
            offset += len;
            sub_command[offset] = '\0';
        }
    }
    term_sudo_active = true;
    term_execute(sub_command);
    term_sudo_active = false;
}
void cmd_su(int argc, char **argv) {
    (void) argc;
    (void) argv;
    main_inputs.is_debug_mode_active = !main_inputs.is_debug_mode_active;
    debug_terminal_sync_mode();
    if (term_engine_start_time == 0) {
        term_engine_start_time = posix_monotonic_time();
    } /* FIX_029 */
    if (main_inputs.is_debug_mode_active) {
        term_ok("Switched to debug mode.\n");
    } else {
        term_ok("Switched to game mode.\n");
    }
}
void cmd_dmesg(int argc, char **argv) {
    int max_events = 32;
    int filter_level = -1;
    for (int i = 1; i < argc; i++) {
        if (term_str_eq(argv[i], "-n") && (i + 1 < argc)) {
            float v = 0.0f;
            if (term_parse_float(argv[i + 1], &v)) {
                max_events = (int) v;
            }
            i++;
        } else if (term_str_eq(argv[i], "-l") && (i + 1 < argc)) {
            i++;
            if (term_str_eq(argv[i], "info")) {
                filter_level = 0;
            } else if (term_str_eq(argv[i], "warn")) {
                filter_level = 1;
            } else if (term_str_eq(argv[i], "error")) {
                filter_level = 2;
            }
        }
    }
    if (max_events < 1) {
        max_events = 1;
    }
    if (max_events > 256) {
        max_events = 256;
    }
    int total_events = event_log_get_count();
    if (total_events == 0) {
        term_dim("(event log empty)\n");
        return;
    }
    int start_index = total_events - max_events;
    if (start_index < 0) {
        start_index = 0;
    }
    int printed = 0;
    for (int i = start_index; i < total_events; i++) {
        log_level level;
        time_t timestamp;
        const char *message = event_log_get_message(i, &level, &timestamp);
        if (!message) {
            continue;
        }
        if ((filter_level >= 0) && ((int) level < filter_level)) {
            continue;
        }
        struct tm *local_time = localtime(&timestamp);
        char time_buffer[32];
        strftime(time_buffer, sizeof(time_buffer), "%H:%M:%S", local_time);
        const char *level_text = (level == log_info) ? "INFO" : (level == log_warn) ? "WARN" : "ERR ";
        const char *tag_name = (level == log_error) ? "term_err" : (level == log_warn) ? "term_echo" : NULL;
        term_printf(tag_name, "[%s] %s: %s\n", time_buffer, level_text, message);
        printed++;
    }
    if (printed == 0) {
        term_dim("(no matching events)\n");
    } else {
        term_printf("term_dim", "%d event(s) shown\n", printed);
    }
}
/* MPE_TASK_V15R2_PHASE7_IMPL_END */
/* MPE_TASK_V15R2_PHASE8_IMPL_BEGIN */
/* MPE_TASK_V15R2_MICROVIM_FILE_WHITELIST_BEGIN */
typedef struct {
    const char *path;
    const char *description;
} mv_editable_file;
static const mv_editable_file mv_known_files[] = {{"status/engine.cfg", "Main engine configuration (78 tunables)"},
                                                  {"status/engine.cfg.backup", "F10/F11 validation config backup"},
                                                  {"status/engine.cfg.bak", "MicroVim auto-backup (last :w)"},
                                                  {"status/scene.dat", "Scene save file (binary)"},
                                                  /* Project files below resolve relative to the engine working
                                                   * directory (v15R3/src): ../ enters v15R3/, ../../ the repo
                                                   * root. Listed wrongly as bare names before (they opened as
                                                   * [New] buffers and would save strays into src/). */
                                                  {"../../readme.md", "Project README"},
                                                  {"../evolution.txt", "Version lineage (stages to v15R3 dev)"},
                                                  {"../how_to_use.md", "User guide / controls reference"},
                                                  {"../RELEASE_POLICY.md", "Release cycle rules"},
                                                  {"../RELEASE_GATES.md", "P0/P1/P2 gate checklist"},
                                                  {"../../scope.md", "Defect & debt audit"},
                                                  {"../../LICENSE", "GPL-3.0 license text"},
                                                  {"../../.gitignore", "Git ignore rules"},
                                                  {"../validation/V01.sh", "Sanitizer build script"},
                                                  {"../validation/V02.sh", "Clean build + warning review"},
                                                  {"../validation/V03.py", "P0 gate interactive walk"},
                                                  {"../validation/V04.sh", "F10 long-run validation guide"},
                                                  {"makefile", "Build system makefile"},
                                                  {"compile", "Compile script"},
                                                  {"config/mpe_constants.h", "Compile-time constants manifest"},
                                                  {"config/mpe_config.h", "Config API header"},
                                                  {"config/mpe_config_schema.c", "Config parameter registry"},
                                                  {"config/mpe_config.c", "Config implementation"},
                                                  {"ui_input/microvim.h", "MicroVim header"},
                                                  {"ui_input/microvim.c", "MicroVim implementation"},
                                                  {"ui_input/debug_terminal.h", "Terminal header"},
                                                  {"ui_input/debug_terminal.c", "Terminal implementation"},
                                                  {"render/shaders/vertex_shader.glsl", "Vertex shader"},
                                                  {"render/shaders/fragment_shader.glsl", "Fragment shader"},
                                                  {"render/shaders/utility_vertex.glsl", "Utility vertex shader"},
                                                  {"render/shaders/utility_fragment.glsl", "Utility fragment shader"},
                                                  {"render/shaders/axis_vertex.glsl", "Axis vertex shader"},
                                                  {"render/shaders/axis_fragment.glsl", "Axis fragment shader"},
                                                  {NULL, NULL}};
static const char *mv_allowed_extensions[] = {".cfg", ".ini", ".conf", ".txt", ".md", ".glsl",
                                              ".sh",  ".py",  ".h",    ".c",   NULL};
static const char *mv_blocked_extensions[] = {".dat", ".o",   ".so",  ".a",     ".bin",
                                              ".exe", ".obj", ".dll", ".dylib", NULL};
bool mv_file_is_allowed(const char *filepath) {
    if (!filepath || filepath[0] == '\0') {
        return false;
    }
    /* Reject absolute paths */
    if (filepath[0] == '/') {
        return false;
    }
    /* Reject paths with null bytes (defensive) */
    /* Check blocked extensions first */
    const char *dot = strrchr(filepath, '.');
    if (dot) {
        for (int i = 0; mv_blocked_extensions[i]; i++) {
            if (strcasecmp(dot, mv_blocked_extensions[i]) == 0) {
                return false;
            }
        }
    }
    /* Check if it's in the known files list */
    for (int i = 0; mv_known_files[i].path; i++) {
        if (strcmp(filepath, mv_known_files[i].path) == 0) {
            return true;
        }
    }
    /* Check if it has an allowed extension */
    if (dot) {
        for (int i = 0; mv_allowed_extensions[i]; i++) {
            if (strcasecmp(dot, mv_allowed_extensions[i]) == 0) {
                return true;
            }
        }
    }
    return false;
}
/* MPE_TASK_V15R2_MICROVIM_FILE_WHITELIST_END */
void cmd_vi(int argc, char **argv) {
    /* Handle --list / -l flag */
    if ((argc > 1) && (term_str_eq(argv[1], "--list") || term_str_eq(argv[1], "-l"))) {
        term_printf("term_echo", "MicroVim editable files:\n");
        term_out("\n");
        for (int i = 0; mv_known_files[i].path; i++) {
            term_printf(NULL, "  %-40s %s\n", mv_known_files[i].path, mv_known_files[i].description);
        }
        term_out("\n");
        term_dim("Also allowed: any file with extensions: .cfg .ini .conf .txt .md .glsl .sh .py .h .c\n");
        term_dim("Blocked: .dat .o .so .a .bin .exe and other binary formats\n");
        term_out("\n");
        term_dim("Usage: vi <path>  |  vi --list  |  vi --help\n");
        return;
    }
    /* Handle --help / -h flag */
    if ((argc > 1) && (term_str_eq(argv[1], "--help") || term_str_eq(argv[1], "-h"))) {
        term_printf("term_echo", "MicroVim — minimal modal editor\n");
        term_out("\n");
        term_out("  vi [path]       Open file (default: status/engine.cfg)\n");
        term_out("  vi --list       Show editable files\n");
        term_out("  vi --help       This help\n");
        term_out("\n");
        term_out("  Modes: Normal (default), Insert (i/a/o), Command (:)\n");
        term_out("  Nav:   h j k l  w b e  0 $  gg G  Ctrl+F/B  { }\n");
        term_out("  Edit:  x dd dw d$  yy p P  u Ctrl+R  J  ~  cc cw C S\n");
        term_out("  Cmd:   :w :q :q! :wq :x :e <file> :N :set nu :s/o/n/g\n");
        term_out("  Exit:  :q or :wq or double-Escape in Normal mode\n");
        return;
    }
    const char *target_file = "status/engine.cfg";
    if (argc > 1) {
        target_file = argv[1];
    }
    /* Validate file against whitelist */
    if (!mv_file_is_allowed(target_file)) {
        term_printf("term_err", "mpe: vi: %s: not an editable file\n", target_file);
        term_dim("Use 'vi --list' to see editable files.\n");
        return;
    }
    /* Prevent opening while microvim is already active */
    if (microvim_is_active()) {
        term_err("mpe: vi: editor already open (close it first with :q or Esc Esc)\n");
        return;
    }
    microvim_open(target_file);
    if (terminal_entry) {
        gtk_widget_hide(terminal_entry);
    }
    if (terminal_prompt_label) {
        gtk_widget_hide(terminal_prompt_label);
    }
    term_printf("term_echo", "MicroVim opened: %s\n", target_file);
    term_dim("Modes: Normal/Insert/Command. Esc=Normal, i=Insert, :=Command.\n");
    term_dim("Save: :w  Quit: :q  Save+Quit: :wq  Force quit: :q!  Exit: Esc Esc\n");
    if (terminal_output_buffer) {
        microvim_render(terminal_output_buffer);
    }
}
/* MPE_TASK_V15R2_PHASE8_IMPL_END */
#endif /* MPE_GTK4 */

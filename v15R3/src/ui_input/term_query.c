/* term_query.c — Query/text commands: stat..less.
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
void cmd_stat(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: stat <path>\n");
        return;
    }
    const char *target = argv[1];
    if (strstr(target, "world")) {
        term_printf("term_echo", "  File: /world\n");
        term_printf(NULL, "  Size: %zu params    Blocks: 13    IO Block: config\n", g_registry_count);
        term_printf(NULL, "  Mode: (0644/-rw-r--r--)  Uid: 0  Gid: 0\n");
        term_printf(NULL, "  Gravity: %.4f  Drag: %.4f\n", g_cfg.world.gravity, g_cfg.world.drag);
        term_printf(NULL, "  Objects: %d  Joints: %d  Mode: %s\n", (physics_world_get_primary()->body_count), (physics_world_get_primary()->spring_joint_count),
                    main_inputs.is_debug_mode_active ? "debug" : "game");
        return;
    }
    if (strstr(target, "camera")) {
        term_printf("term_echo", "  File: /camera\n");
        term_printf(NULL, "  Position: (%.4f, %.4f, %.4f)\n", main_camera_fov.position.x, main_camera_fov.position.y,
                    main_camera_fov.position.z);
        term_printf(NULL, "  Yaw: %.4f  Pitch: %.4f  Speed: %.4f\n", main_camera_fov.yaw, main_camera_fov.pitch,
                    main_camera_fov.movement_speed);
        term_printf(NULL, "  Mouse: %s  Mode: %s\n", main_inputs.is_mouse_locked ? "locked" : "free",
                    main_inputs.is_debug_mode_active ? "debug" : "game");
        return;
    }
    if (strstr(target, "spawner")) {
        term_printf("term_echo", "  File: /spawner\n");
        term_printf(NULL, "  Type: %s  Mass: %.4f  Radius: %.4f\n",
                    (main_inputs.current_spawn_type == 0) ? "sphere" : "cube", g_cfg.spawner.mass,
                    g_cfg.spawner.radius);
        term_printf(NULL, "  Speed: %.4f  Friction: s=%.3f k=%.3f\n", g_cfg.spawner.speed, g_cfg.spawner.friction_s,
                    g_cfg.spawner.friction_k);
        return;
    }
    int object_index = term_object_from_token(target);
    if (object_index < 0) {
        term_printf("term_err", "mpe: stat: %s: No such object\n", target);
        return;
    }
    rigidbody *rb = &(physics_world_get_primary()->bodies)[object_index];
    int joint_count_for_obj = 0;
    for (int ji = 0; ji < mpe_max_joints; ji++) {
        if (!(physics_world_get_primary()->spring_joints)[ji].is_active) {
            continue;
        }
        int ia = scene_find_object_index_by_id((physics_world_get_primary()->spring_joints)[ji].object_id_a);
        int ib = scene_find_object_index_by_id((physics_world_get_primary()->spring_joints)[ji].object_id_b);
        if ((ia == object_index) || (ib == object_index)) {
            joint_count_for_obj++;
        }
    }
    term_printf("term_echo", "  File: /obj/%d\n", object_index);
    term_printf(NULL, "  Size: %.4f kg    Links: %d    Inode: %u\n", rb->mass, joint_count_for_obj, rb->object_id);
    term_printf(NULL, "  Access: %s/%s  Mode: %s\n", rb->static_state ? "static" : "dynamic",
                rb->is_sleeping ? "sleeping" : "awake", term_object_mode(rb));
    term_printf(NULL, "  Type: %s\n", term_object_type_name(rb));
    if (rb->type == object_sphere) {
        term_printf(NULL, "  Radius: %.4f\n", rb->radius);
    } else {
        term_printf(NULL, "  HalfExt: (%.4f, %.4f, %.4f)\n", rb->half_extensions.x, rb->half_extensions.y,
                    rb->half_extensions.z);
    }
    term_printf(NULL, "  Position: (%.4f, %.4f, %.4f)\n", rb->position.x, rb->position.y, rb->position.z);
    term_printf(NULL, "  Velocity: (%.4f, %.4f, %.4f)  |v|=%.4f\n", rb->velocity.x, rb->velocity.y, rb->velocity.z,
                vector3_length(rb->velocity));
    term_printf(NULL, "  AngVel: (%.4f, %.4f, %.4f)  |w|=%.4f\n", rb->angular_velocity.x, rb->angular_velocity.y,
                rb->angular_velocity.z, vector3_length(rb->angular_velocity));
    term_printf(NULL, "  Orient: (%.4f, %.4f, %.4f, %.4f)\n", rb->orientation.w, rb->orientation.x, rb->orientation.y,
                rb->orientation.z);
    term_printf(NULL, "  Friction: s=%.3f k=%.3f  Restitution: %.3f\n", rb->friction_static, rb->friction_kinetic,
                rb->restitution);
    term_printf(NULL, "  Colour: (%.2f, %.2f, %.2f)\n", rb->colour.x, rb->colour.y, rb->colour.z);
    term_printf(NULL, "  SleepTimer: %.2f  Nice: %d\n", rb->sleep_timer, rb->nice_value);
}
void cmd_find(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: find /obj [-type t] [-mass v] [-sleeping] [-awake] [-static] [-dynamic]\n");
        return;
    }
    int filter_type = -1;
    float mass_exact = -1.0f;
    float mass_greater = -1.0f;
    float mass_less = -1.0f;
    bool filter_sleeping = false;
    bool filter_awake = false;
    bool filter_static = false;
    bool filter_dynamic = false;
    for (int i = 1; i < argc; i++) {
        if (term_str_eq(argv[i], "-type") && (i + 1 < argc)) {
            i++;
            if (term_str_eq(argv[i], "sphere") || term_str_eq(argv[i], "sph")) {
                filter_type = 0;
            } else if (term_str_eq(argv[i], "cube")) {
                filter_type = 1;
            }
        } else if (term_str_eq(argv[i], "-mass") && (i + 1 < argc)) {
            i++;
            const char *val = argv[i];
            if (val[0] == '+') {
                term_parse_float(val + 1, &mass_greater);
            } else if (val[0] == '-') {
                term_parse_float(val + 1, &mass_less);
            } else {
                term_parse_float(val, &mass_exact);
            }
        } else if (term_str_eq(argv[i], "-sleeping")) {
            filter_sleeping = true;
        } else if (term_str_eq(argv[i], "-awake")) {
            filter_awake = true;
        } else if (term_str_eq(argv[i], "-static")) {
            filter_static = true;
        } else if (term_str_eq(argv[i], "-dynamic")) {
            filter_dynamic = true;
        }
    }
    int match_count = 0;
    for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
        rigidbody *rb = &(physics_world_get_primary()->bodies)[object_index];
        if ((filter_type == 0) && (rb->type != object_sphere)) {
            continue;
        }
        if ((filter_type == 1) && (rb->type != object_cube)) {
            continue;
        }
        if ((mass_exact >= 0.0f) && (fabsf(rb->mass - mass_exact) > 0.001f)) {
            continue;
        }
        if ((mass_greater >= 0.0f) && (rb->mass <= mass_greater)) {
            continue;
        }
        if ((mass_less >= 0.0f) && (rb->mass >= mass_less)) {
            continue;
        }
        if (filter_sleeping && (!rb->is_sleeping)) {
            continue;
        }
        if (filter_awake && (rb->is_sleeping)) {
            continue;
        }
        if (filter_static && (!rb->static_state)) {
            continue;
        }
        if (filter_dynamic && (rb->static_state)) {
            continue;
        }
        term_printf(NULL, "/obj/%d\n", object_index);
        match_count++;
    }
    if (match_count == 0) {
        term_dim("(no matches)\n");
    } else {
        term_printf("term_dim", "%d match(es)\n", match_count);
    }
}
void cmd_wc(int argc, char **argv) {
    if (argc < 2) {
        term_printf(NULL, "%d objects, %d joints\n", (physics_world_get_primary()->body_count), (physics_world_get_primary()->spring_joint_count));
        return;
    }
    for (int i = 1; i < argc; i++) {
        if (argv[i][0] == '-') {
            continue;
        }
        if (strstr(argv[i], "joint")) {
            int active_joints = 0;
            for (int ji = 0; ji < mpe_max_joints; ji++) {
                if ((physics_world_get_primary()->spring_joints)[ji].is_active) {
                    active_joints++;
                }
            }
            term_printf(NULL, "%d /joint\n", active_joints);
        } else if (strstr(argv[i], "obj")) {
            term_printf(NULL, "%d /obj\n", (physics_world_get_primary()->body_count));
        } else {
            term_printf(NULL, "%d objects, %d joints\n", (physics_world_get_primary()->body_count), (physics_world_get_primary()->spring_joint_count));
        }
    }
}
void cmd_file(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: file <path>\n");
        return;
    }
    const char *target = argv[1];
    if (strstr(target, "world")) {
        term_printf(NULL, "/world: physics configuration, %zu parameters\n", g_registry_count);
        return;
    }
    if (strstr(target, "camera")) {
        term_printf(NULL, "/camera: viewport state, %s mode\n", main_inputs.is_debug_mode_active ? "debug" : "game");
        return;
    }
    if (strstr(target, "spawner")) {
        term_printf(NULL, "/spawner: object factory, type=%s\n",
                    (main_inputs.current_spawn_type == 0) ? "sphere" : "cube");
        return;
    }
    if (term_classify_token(target) == term_target_joint) {
        int joint_index = term_joint_from_token(target);
        if (joint_index >= 0) {
            spring_joint *j = &(physics_world_get_primary()->spring_joints)[joint_index];
            term_printf(NULL, "/joint/%d: spring joint, k=%.1f d=%.1f len=%.2f\n", joint_index, j->spring_constant,
                        j->damping_coefficient, j->equilibrium_length);
        } else {
            term_printf("term_err", "mpe: file: %s: No such joint\n", target);
        }
        return;
    }
    int object_index = term_object_from_token(target);
    if (object_index < 0) {
        term_printf("term_err", "mpe: file: %s: No such object\n", target);
        return;
    }
    rigidbody *rb = &(physics_world_get_primary()->bodies)[object_index];
    term_printf(NULL, "/obj/%d: rigid body, %s, %.2f kg, %s%s\n", object_index, term_object_type_name(rb), rb->mass,
                rb->static_state ? "static" : "dynamic", rb->is_sleeping ? ", sleeping" : "");
}
void cmd_diff(int argc, char **argv) {
    if (argc < 3) {
        term_err("usage: diff <object_a> <object_b>\n");
        return;
    }
    int index_a = term_require_object(argv[1]);
    if (index_a < 0) {
        return;
    }
    int index_b = term_require_object(argv[2]);
    if (index_b < 0) {
        return;
    }
    if (index_a == index_b) {
        term_ok("Objects are identical (same object)\n");
        return;
    }
    rigidbody *a = &(physics_world_get_primary()->bodies)[index_a];
    rigidbody *b = &(physics_world_get_primary()->bodies)[index_b];
    int diff_count = 0;
    term_printf("term_echo", "--- /obj/%d\n", index_a);
    term_printf("term_echo", "+++ /obj/%d\n", index_b);
    if (a->type != b->type) {
        term_printf("term_err", "  type:       %s -> %s\n", term_object_type_name(a), term_object_type_name(b));
        diff_count++;
    }
    if (fabsf(a->mass - b->mass) > 0.001f) {
        term_printf("term_err", "  mass:       %.4f -> %.4f\n", a->mass, b->mass);
        diff_count++;
    }
    if ((a->type == object_sphere) && (b->type == object_sphere)) {
        if (fabsf(a->radius - b->radius) > 0.001f) {
            term_printf("term_err", "  radius:     %.4f -> %.4f\n", a->radius, b->radius);
            diff_count++;
        }
    }
    if (vector3_length_squared(vector3_subtraction(a->position, b->position)) > 0.001f) {
        term_printf(NULL, "  position:   (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)\n", a->position.x, a->position.y,
                    a->position.z, b->position.x, b->position.y, b->position.z);
        diff_count++;
    }
    if (vector3_length_squared(vector3_subtraction(a->velocity, b->velocity)) > 0.001f) {
        term_printf(NULL, "  velocity:   |%.3f| -> |%.3f|\n", vector3_length(a->velocity), vector3_length(b->velocity));
        diff_count++;
    }
    if (a->static_state != b->static_state) {
        term_printf("term_err", "  static:     %s -> %s\n", a->static_state ? "yes" : "no",
                    b->static_state ? "yes" : "no");
        diff_count++;
    }
    if (a->is_sleeping != b->is_sleeping) {
        term_printf(NULL, "  sleeping:   %s -> %s\n", a->is_sleeping ? "yes" : "no", b->is_sleeping ? "yes" : "no");
        diff_count++;
    }
    if ((fabsf(a->friction_static - b->friction_static) > 0.001f) ||
        (fabsf(a->friction_kinetic - b->friction_kinetic) > 0.001f)) {
        term_printf(NULL, "  friction:   s=%.3f k=%.3f -> s=%.3f k=%.3f\n", a->friction_static, a->friction_kinetic,
                    b->friction_static, b->friction_kinetic);
        diff_count++;
    }
    if (fabsf(a->restitution - b->restitution) > 0.001f) {
        term_printf(NULL, "  restitution:%.3f -> %.3f\n", a->restitution, b->restitution);
        diff_count++;
    }
    if (a->nice_value != b->nice_value) {
        term_printf(NULL, "  nice:       %d -> %d\n", a->nice_value, b->nice_value);
        diff_count++;
    }
    if (diff_count == 0) {
        term_ok("Objects are identical\n");
    } else {
        term_printf("term_dim", "%d difference(s)\n", diff_count);
    }
}
void cmd_xxd(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: xxd <object> [-l len] [-s offset]\n");
        return;
    }
    int object_index = term_require_object(argv[1]);
    if (object_index < 0) {
        return;
    }
    int dump_length = (int) sizeof(rigidbody);
    int dump_offset = 0;
    for (int i = 2; i < argc; i++) {
        if (term_str_eq(argv[i], "-l") && (i + 1 < argc)) {
            float v = 0.0f;
            if (term_parse_float(argv[i + 1], &v)) {
                dump_length = (int) v;
            }
            i++;
        } else if (term_str_eq(argv[i], "-s") && (i + 1 < argc)) {
            float v = 0.0f;
            if (term_parse_float(argv[i + 1], &v)) {
                dump_offset = (int) v;
            }
            i++;
        }
    }
    int struct_size = (int) sizeof(rigidbody);
    if (dump_offset >= struct_size) {
        term_err("mpe: xxd: offset beyond struct size\n");
        return;
    }
    if (dump_offset + dump_length > struct_size) {
        dump_length = struct_size - dump_offset;
    }
    if (dump_length <= 0) {
        dump_length = struct_size - dump_offset;
    }
    const unsigned char *raw = (const unsigned char *) &(physics_world_get_primary()->bodies)[object_index];
    term_printf("term_echo", "xxd /obj/%d  (%d bytes at offset %d of %d)\n", object_index, dump_length, dump_offset,
                struct_size);
    for (int row = 0; row < dump_length; row += 16) {
        int row_len = dump_length - row;
        if (row_len > 16) {
            row_len = 16;
        }
        char hex_part[64];
        char ascii_part[20];
        int hex_offset = 0;
        for (int col = 0; col < 16; col++) {
            if (col < row_len) {
                unsigned char byte = raw[dump_offset + row + col];
                hex_offset += snprintf(hex_part + hex_offset, sizeof(hex_part) - hex_offset, "%02x ", byte);
                ascii_part[col] = ((byte >= 32) && (byte < 127)) ? (char) byte : '.';
            } else {
                hex_offset += snprintf(hex_part + hex_offset, sizeof(hex_part) - hex_offset, "   ");
                ascii_part[col] = ' ';
            }
        }
        ascii_part[row_len] = '\0';
        term_printf(NULL, "%08x: %-48s  |%s|\n", dump_offset + row, hex_part, ascii_part);
    }
}
/* MPE_TASK_V15R2_PHASE3_IMPL_END */
/* MPE_TASK_V15R2_PHASE4_IMPL_BEGIN */
/* --- sort helpers --- */
static int a3_sort_key = 0; /* 0=index 1=mass 2=speed 3=type 4=pos.y */
static bool a3_sort_reverse = false;
int a3_sort_compare(const void *pa, const void *pb) {
    int ia = *(const int *) pa;
    int ib = *(const int *) pb;
    rigidbody *ra = &(physics_world_get_primary()->bodies)[ia];
    rigidbody *rb = &(physics_world_get_primary()->bodies)[ib];
    float va = 0.0f, vb = 0.0f;
    switch (a3_sort_key) {
    case 1:
        va = ra->mass;
        vb = rb->mass;
        break;
    case 2:
        va = vector3_length(ra->velocity);
        vb = vector3_length(rb->velocity);
        break;
    case 3:
        va = (float) ra->type;
        vb = (float) rb->type;
        break;
    case 4:
        va = ra->position.y;
        vb = rb->position.y;
        break;
    default:
        va = (float) ia;
        vb = (float) ib;
        break;
    }
    int result = (va < vb) ? -1 : ((va > vb) ? 1 : 0);
    return a3_sort_reverse ? -result : result;
}
void cmd_sort(int argc, char **argv) {
    a3_sort_key = 0;
    a3_sort_reverse = false;
    bool list_joints = false;
    for (int i = 1; i < argc; i++) {
        if (term_str_eq(argv[i], "-k") && (i + 1 < argc)) {
            i++;
            if (term_str_eq(argv[i], "mass")) {
                a3_sort_key = 1;
            } else if (term_str_eq(argv[i], "speed")) {
                a3_sort_key = 2;
            } else if (term_str_eq(argv[i], "type")) {
                a3_sort_key = 3;
            } else if (term_str_eq(argv[i], "pos.y")) {
                a3_sort_key = 4;
            }
        } else if (term_str_eq(argv[i], "-r")) {
            a3_sort_reverse = true;
        } else if (strstr(argv[i], "joint")) {
            list_joints = true;
        }
    }
    if (list_joints) {
        term_list_joints(true);
        return;
    }
    if ((physics_world_get_primary()->body_count) == 0) {
        term_dim("(no objects)\n");
        return;
    }
    static int sort_indices[mpe_max_bodies];
    for (int i = 0; i < (physics_world_get_primary()->body_count); i++) {
        sort_indices[i] = i;
    }
    qsort(sort_indices, (size_t) (physics_world_get_primary()->body_count), sizeof(int), a3_sort_compare);
    term_printf(NULL, "%-10s %4s %8s %-4s %-6s %s\n", "MODE", "PID", "MASS", "TYPE", "STATE", "INFO");
    for (int i = 0; i < (physics_world_get_primary()->body_count); i++) {
        term_print_object_long(sort_indices[i]);
    }
}
void cmd_grep(int argc, char **argv) {
    if (argc < 2) {
        term_err("usage: grep <pattern> [path]\n");
        return;
    }
    const char *pattern = argv[1];
    term_capture_begin();
    if ((argc > 2) && (strstr(argv[2], "joint"))) {
        term_list_joints(true);
    } else {
        term_list_objects(true);
    }
    term_capture_end();
    char *captured = term_capture_get();
    if ((!captured) || (captured[0] == '\0')) {
        term_capture_reset();
        term_dim("(no output)\n");
        return;
    }
    int match_count = 0;
    char *line_start = captured;
    char *newline_pos;
    while ((newline_pos = strchr(line_start, '\n')) != NULL) {
        *newline_pos = '\0';
        if (g_ascii_strncasecmp(line_start, pattern, strlen(pattern)) == 0 || strstr(line_start, pattern) != NULL) {
            term_printf(NULL, "%s\n", line_start);
            match_count++;
        }
        line_start = newline_pos + 1;
    }
    if (line_start[0] != '\0') {
        if (strstr(line_start, pattern) != NULL) {
            term_printf(NULL, "%s\n", line_start);
            match_count++;
        }
    }
    term_capture_reset();
    if (match_count == 0) {
        term_dim("(no matches)\n");
    } else {
        term_printf("term_dim", "%d match(es)\n", match_count);
    }
}
void cmd_head(int argc, char **argv) {
    int line_count = 10;
    bool list_joints = false;
    for (int i = 1; i < argc; i++) {
        if (term_str_eq(argv[i], "-n") && (i + 1 < argc)) {
            float v = 0.0f;
            if (term_parse_float(argv[i + 1], &v)) {
                line_count = (int) v;
            }
            i++;
        } else if (strstr(argv[i], "joint")) {
            list_joints = true;
        }
    }
    if (line_count < 1) {
        line_count = 1;
    }
    term_capture_begin();
    if (list_joints) {
        term_list_joints(true);
    } else {
        term_list_objects(true);
    }
    term_capture_end();
    char *captured = term_capture_get();
    if ((!captured) || (captured[0] == '\0')) {
        term_capture_reset();
        term_dim("(no output)\n");
        return;
    }
    int printed = 0;
    char *line_start = captured;
    char *newline_pos;
    while (((newline_pos = strchr(line_start, '\n')) != NULL) && (printed < line_count)) {
        *newline_pos = '\0';
        term_printf(NULL, "%s\n", line_start);
        printed++;
        line_start = newline_pos + 1;
    }
    if ((printed < line_count) && (line_start[0] != '\0')) {
        term_printf(NULL, "%s\n", line_start);
    }
    term_capture_reset();
}
void cmd_tail(int argc, char **argv) {
    int line_count = 10;
    bool list_joints = false;
    for (int i = 1; i < argc; i++) {
        if (term_str_eq(argv[i], "-n") && (i + 1 < argc)) {
            float v = 0.0f;
            if (term_parse_float(argv[i + 1], &v)) {
                line_count = (int) v;
            }
            i++;
        } else if (strstr(argv[i], "joint")) {
            list_joints = true;
        }
    }
    if (line_count < 1) {
        line_count = 1;
    }
    term_capture_begin();
    if (list_joints) {
        term_list_joints(true);
    } else {
        term_list_objects(true);
    }
    term_capture_end();
    char *captured = term_capture_get();
    if ((!captured) || (captured[0] == '\0')) {
        term_capture_reset();
        term_dim("(no output)\n");
        return;
    }
    /* Count total lines */
    int total_lines = 0;
    char *scan = captured;
    while (*scan) {
        if (*scan == '\n') {
            total_lines++;
        }
        scan++;
    }
    if (captured[strlen(captured) - 1] != '\n') {
        total_lines++;
    }
    /* Skip to the start of the last N lines */
    int skip = total_lines - line_count;
    if (skip < 0) {
        skip = 0;
    }
    char *line_start = captured;
    for (int i = 0; i < skip; i++) {
        char *nl = strchr(line_start, '\n');
        if (!nl) {
            break;
        }
        line_start = nl + 1;
    }
    /* Print remaining */
    char *newline_pos;
    while ((newline_pos = strchr(line_start, '\n')) != NULL) {
        *newline_pos = '\0';
        term_printf(NULL, "%s\n", line_start);
        line_start = newline_pos + 1;
    }
    if (line_start[0] != '\0') {
        term_printf(NULL, "%s\n", line_start);
    }
    term_capture_reset();
}
void cmd_less(int argc, char **argv) {
    int page_size = 40;
    bool list_joints = false;
    for (int i = 1; i < argc; i++) {
        if (strstr(argv[i], "joint")) {
            list_joints = true;
        }
    }
    term_capture_begin();
    if (list_joints) {
        term_list_joints(true);
    } else {
        term_list_objects(true);
    }
    term_capture_end();
    char *captured = term_capture_get();
    if ((!captured) || (captured[0] == '\0')) {
        term_capture_reset();
        term_dim("(no output)\n");
        return;
    }
    int total_lines = 0;
    char *scan = captured;
    while (*scan) {
        if (*scan == '\n') {
            total_lines++;
        }
        scan++;
    }
    if (captured[strlen(captured) - 1] != '\n') {
        total_lines++;
    }
    term_printf("term_dim", "-- %d lines total, showing first %d --\n", total_lines, page_size);
    int printed = 0;
    char *line_start = captured;
    char *newline_pos;
    while (((newline_pos = strchr(line_start, '\n')) != NULL) && (printed < page_size)) {
        *newline_pos = '\0';
        term_printf(NULL, "%s\n", line_start);
        printed++;
        line_start = newline_pos + 1;
    }
    if ((printed < page_size) && (line_start[0] != '\0')) {
        term_printf(NULL, "%s\n", line_start);
        printed++;
    }
    if (total_lines > page_size) {
        term_printf("term_dim", "-- %d more lines (use head/tail/grep to filter) --\n", total_lines - printed);
    }
    term_capture_reset();
}
/* MPE_TASK_V15R2_PHASE4_IMPL_END */
/* MPE_TASK_V15R2_PHASE5_IMPL_BEGIN */
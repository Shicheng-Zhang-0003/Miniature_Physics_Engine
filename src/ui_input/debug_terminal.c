#include "../mpe_engine.h"
#include "debug_terminal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <gdk/gdkkeysyms.h>

/* MPE_TASK_23_POSIX_DEBUG_TERMINAL */

static GtkWidget *terminal_window = NULL;
static GtkWidget *terminal_output_view = NULL;
static GtkTextBuffer *terminal_output_buffer = NULL;
static GtkWidget *terminal_entry = NULL;
static GtkWidget *terminal_prompt_label = NULL;

static char term_cwd [256] = "/";

#define TERM_HISTORY_SIZE 64
#define TERM_HISTORY_LENGTH 511

static char term_history [TERM_HISTORY_SIZE][TERM_HISTORY_LENGTH + 1];
static int term_history_count = 0;
static int term_history_cursor = -1;

static uint32_t term_id_buffer [MPE_MAX_BODIES];

/* ------------------------------------------------------------------ */
/* Output helpers                                                      */
/* ------------------------------------------------------------------ */

static void term_scroll_to_bottom (void) {
if (!terminal_output_buffer) {return;}
GtkTextIter end_iter;
gtk_text_buffer_get_end_iter (terminal_output_buffer, &end_iter);
gtk_text_view_scroll_to_iter (GTK_TEXT_VIEW (terminal_output_view), &end_iter, 0.0, FALSE, 0.0, 0.0);
}

static void term_append_with_tag (const char *tag_name, const char *text) {
if (!terminal_output_buffer) {return;}
GtkTextIter end_iter;
gtk_text_buffer_get_end_iter (terminal_output_buffer, &end_iter);
if (tag_name) {
gtk_text_buffer_insert_with_tags_by_name (terminal_output_buffer, &end_iter, text, -1, tag_name, NULL);
} else {
gtk_text_buffer_insert (terminal_output_buffer, &end_iter, text, -1);
}
term_scroll_to_bottom ();
}

static void term_out (const char *text) {term_append_with_tag (NULL, text);}
static void term_ok (const char *text) {term_append_with_tag ("term_ok", text);}
static void term_err (const char *text) {term_append_with_tag ("term_err", text);}
static void term_echo (const char *text) {term_append_with_tag ("term_echo", text);}
static void term_dim (const char *text) {term_append_with_tag ("term_dim", text);}

static void term_printf (const char *tag_name, const char *format, ...) {
char line_buffer [2048];
va_list argument_list;
va_start (argument_list, format);
vsnprintf (line_buffer, sizeof (line_buffer), format, argument_list);
va_end (argument_list);
term_append_with_tag (tag_name, line_buffer);
}

static void term_update_prompt (void) {
if (!terminal_prompt_label) {return;}
char prompt_buffer [320];
snprintf (prompt_buffer, sizeof (prompt_buffer), "mpe:%s>", term_cwd);
gtk_label_set_text (GTK_LABEL (terminal_prompt_label), prompt_buffer);
}

/* ------------------------------------------------------------------ */
/* History                                                             */
/* ------------------------------------------------------------------ */

static void term_history_push (const char *command_text) {
if (command_text [0] == '\0') {return;}
if ((term_history_count > 0) && (strcmp (term_history [0], command_text) == 0)) {return;}
if (term_history_count < TERM_HISTORY_SIZE) {term_history_count++;}
for (int history_index = term_history_count - 1; history_index > 0; history_index--) {
strncpy (term_history [history_index], term_history [history_index - 1], TERM_HISTORY_LENGTH);
term_history [history_index][TERM_HISTORY_LENGTH] = '\0';
}
strncpy (term_history [0], command_text, TERM_HISTORY_LENGTH);
term_history [0][TERM_HISTORY_LENGTH] = '\0';
}

/* ------------------------------------------------------------------ */
/* String/path helpers                                                 */
/* ------------------------------------------------------------------ */

static bool term_str_eq (const char *string_a, const char *string_b) {
if ((!string_a) || (!string_b)) {return false;}
return g_ascii_strcasecmp (string_a, string_b) == 0;
}

static const char *term_last_path_component (const char *token) {
if (!token) {return "";}
const char *slash = strrchr (token, '/');
return slash ? (slash + 1) : token;
}

static bool term_is_all_token (const char *token) {
return term_str_eq (term_last_path_component (token), "all");
}

static bool term_parse_float (const char *token, float *output_value) {
if (!token) {return false;}
char *endptr = NULL;
float parsed_value = strtof (token, &endptr);
if ((endptr == token) || (*endptr != '\0') || (!isfinite (parsed_value))) {return false;}
*output_value = parsed_value;
return true;
}

static int term_object_from_token (const char *token) {
if (!token) {return -1;}
if (term_str_eq (token, "sel") || term_str_eq (term_last_path_component (token), "sel")) {
if ((selected_object >= 0) && (selected_object < object_count)) {return selected_object;}
return -1;
}
const char *component = term_last_path_component (token);
char *endptr = NULL;
long parsed_index = strtol (component, &endptr, 10);
if ((endptr == component) || (*endptr != '\0')) {return -1;}
if ((parsed_index < 0) || (parsed_index >= object_count)) {return -1;}
return (int) parsed_index;
}

static int term_joint_from_token (const char *token) {
if (!token) {return -1;}
const char *component = term_last_path_component (token);
char *endptr = NULL;
long parsed_index = strtol (component, &endptr, 10);
if ((endptr == component) || (*endptr != '\0')) {return -1;}
if ((parsed_index < 0) || (parsed_index >= MPE_MAX_JOINTS)) {return -1;}
if (!joint_pool [parsed_index].is_active) {return -1;}
return (int) parsed_index;
}

typedef enum {
TERM_TARGET_OBJECT,
TERM_TARGET_JOINT
} term_target_kind;

static term_target_kind term_classify_token (const char *token) {
if (!token) {return TERM_TARGET_OBJECT;}
if (strstr (token, "joint")) {return TERM_TARGET_JOINT;}
if (strstr (token, "obj")) {return TERM_TARGET_OBJECT;}
if (strstr (term_cwd, "joint")) {return TERM_TARGET_JOINT;}
return TERM_TARGET_OBJECT;
}

static int term_require_object (const char *token) {
int object_index = term_object_from_token (token);
if (object_index < 0) {
term_printf ("term_err", "mpe: %s: No such object\n", token ? token : "(null)");
}
return object_index;
}

static int term_require_joint (const char *token) {
int joint_index = term_joint_from_token (token);
if (joint_index < 0) {
term_printf ("term_err", "mpe: %s: No such joint\n", token ? token : "(null)");
}
return joint_index;
}

static int term_parse_movement_destination (const char *token, float *x, float *y, float *z) {
if (!token) {return 0;}
char **parts = g_strsplit (token, "/", -1);
int movement_kind = 0;
for (int part_index = 0; parts [part_index]; part_index++) {
if (term_str_eq (parts [part_index], "pos") || term_str_eq (parts [part_index], "vel")) {
if ((!parts [part_index + 1]) || (!parts [part_index + 2]) || (!parts [part_index + 3])) {break;}
float parsed_x, parsed_y, parsed_z;
if (!term_parse_float (parts [part_index + 1], &parsed_x)) {break;}
if (!term_parse_float (parts [part_index + 2], &parsed_y)) {break;}
if (!term_parse_float (parts [part_index + 3], &parsed_z)) {break;}
*x = parsed_x;
*y = parsed_y;
*z = parsed_z;
movement_kind = term_str_eq (parts [part_index], "pos") ? 1 : 2;
break;
}
}
g_strfreev (parts);
return movement_kind;
}

/* ------------------------------------------------------------------ */
/* Command declarations                                                */
/* ------------------------------------------------------------------ */

static void cmd_help (int argc, char **argv);
static void cmd_man (int argc, char **argv);
static void cmd_clear (int argc, char **argv);
static void cmd_history (int argc, char **argv);
static void cmd_pwd (int argc, char **argv);
static void cmd_cd (int argc, char **argv);
static void cmd_ls (int argc, char **argv);
static void cmd_ll (int argc, char **argv);
static void cmd_cat (int argc, char **argv);
static void cmd_touch (int argc, char **argv);
static void cmd_cp (int argc, char **argv);
static void cmd_rm (int argc, char **argv);
static void cmd_mv (int argc, char **argv);
static void cmd_ln (int argc, char **argv);
static void cmd_unlink (int argc, char **argv);
static void cmd_chmod (int argc, char **argv);
static void cmd_chown (int argc, char **argv);
static void cmd_kill (int argc, char **argv);
static void cmd_ps (int argc, char **argv);
static void cmd_top (int argc, char **argv);
static void cmd_df (int argc, char **argv);
static void cmd_du (int argc, char **argv);
static void cmd_uname (int argc, char **argv);
static void cmd_whoami (int argc, char **argv);
static void cmd_date (int argc, char **argv);
static void cmd_echo (int argc, char **argv);
static void cmd_env (int argc, char **argv);
static void cmd_export (int argc, char **argv);

typedef struct {
const char *name;
bool mutates;
void (*handler) (int argc, char **argv);
const char *usage;
const char *description;
} terminal_command;

static const terminal_command terminal_commands [] = {
{"help",    false, cmd_help,    "help [command]",                     "show help"},
{"man",     false, cmd_man,     "man <command>",                      "manual page"},
{"clear",   false, cmd_clear,   "clear",                              "clear terminal"},
{"history", false, cmd_history, "history",                            "command history"},
{"pwd",     false, cmd_pwd,     "pwd",                                "print working directory"},
{"cd",      false, cmd_cd,      "cd [/|/obj|/joint|..]",              "change directory"},
{"ls",      false, cmd_ls,      "ls [-l] [path]",                     "list objects/joints"},
{"ll",      false, cmd_ll,      "ll [path]",                          "long listing"},
{"cat",     false, cmd_cat,     "cat <path...>",                      "inspect object/joint/world/camera/spawner"},
{"touch",   true,  cmd_touch,   "touch [new.sph|new.cube...]",        "create object"},
{"cp",      true,  cmd_cp,      "cp <object> [dest]",                 "duplicate object"},
{"rm",      true,  cmd_rm,      "rm [-rf] <path...>",                 "remove object/joint/all"},
{"mv",      true,  cmd_mv,      "mv <object> /pos/x/y/z|/vel/x/y/z",  "move or impulse object"},
{"ln",      true,  cmd_ln,      "ln [-s] <object> <object>",          "create spring joint"},
{"unlink",  true,  cmd_unlink,  "unlink <path>",                      "remove joint/object"},
{"chmod",   true,  cmd_chmod,   "chmod static|dynamic <object...>",   "change static state"},
{"chown",   true,  cmd_chown,   "chown <mass> <object...>",           "change mass"},
{"kill",    true,  cmd_kill,    "kill [-STOP|-CONT|-9] <object...>",  "sleep/wake/delete object"},
{"ps",      false, cmd_ps,      "ps [aux]",                           "object process table"},
{"top",     false, cmd_top,     "top [-n count]",                     "fastest objects"},
{"df",      false, cmd_df,      "df [-h]",                            "capacity usage"},
{"du",      false, cmd_du,      "du <path...>",                       "object/joint usage"},
{"uname",   false, cmd_uname,   "uname [-a]",                         "engine version"},
{"whoami",  false, cmd_whoami,  "whoami",                             "print user"},
{"date",    false, cmd_date,    "date",                               "print date/time"},
{"echo",    false, cmd_echo,    "echo [args...]",                     "print arguments"},
{"env",     false, cmd_env,     "env",                                "print world variables"},
{"export",  true,  cmd_export,  "export VAR=value",                   "set world variable"},
};

#define TERMINAL_COMMAND_COUNT (sizeof (terminal_commands) / sizeof (terminal_commands [0]))

/* ------------------------------------------------------------------ */
/* Listing/print helpers                                               */
/* ------------------------------------------------------------------ */

static const char *term_object_type_name (rigidbody *rigid_body) {
return (rigid_body -> type == object_sphere) ? "sph" : "cube";
}

static const char *term_object_state_name (rigidbody *rigid_body) {
if (rigid_body -> static_state) {return "static";}
if (rigid_body -> is_sleeping) {return "sleep";}
return "run";
}

static const char *term_object_mode (rigidbody *rigid_body) {
return rigid_body -> static_state ? "-r--r--r--" : "-rw-r--r--";
}

static void term_print_object_long (int object_index) {
rigidbody *rigid_body = &obj_per_scene [object_index];
term_printf (NULL, "%s %4d %8.2f %-4s %-6s pos=(%.2f,%.2f,%.2f) |v|=%.3f id=%u\n",
term_object_mode (rigid_body),
object_index,
rigid_body -> mass,
term_object_type_name (rigid_body),
term_object_state_name (rigid_body),
rigid_body -> position.x, rigid_body -> position.y, rigid_body -> position.z,
vector3_length (rigid_body -> velocity),
rigid_body -> object_id);
}

static void term_print_joint_long (int joint_index) {
spring_joint *joint = &joint_pool [joint_index];
int index_a = scene_find_object_index_by_id (joint -> object_id_a);
int index_b = scene_find_object_index_by_id (joint -> object_id_b);
term_printf (NULL, "lrwxrwxrwx %4d [%d] -> [%d] len=%.2f k=%.1f d=%.1f\n",
joint_index, index_a, index_b,
joint -> equilibrium_length, joint -> spring_constant, joint -> damping_coefficient);
}

static void term_list_objects (bool long_format) {
if (object_count == 0) {
term_dim ("(no objects)\n");
return;
}
if (long_format) {
term_printf (NULL, "%-10s %4s %8s %-4s %-6s %s\n", "MODE", "PID", "MASS", "TYPE", "STATE", "INFO");
}
for (int object_index = 0; object_index < object_count; object_index++) {
if (long_format) {term_print_object_long (object_index);}
else {term_printf (NULL, "%d\n", object_index);}
}
}

static void term_list_joints (bool long_format) {
int listed_count = 0;
for (int joint_index = 0; joint_index < MPE_MAX_JOINTS; joint_index++) {
if (!joint_pool [joint_index].is_active) {continue;}
if (long_format) {term_print_joint_long (joint_index);}
else {term_printf (NULL, "%d\n", joint_index);}
listed_count++;
}
if (listed_count == 0) {term_dim ("(no joints)\n");}
}

static void term_list_root (bool long_format) {
if (long_format) {
term_out ("drwxr-xr-x 2 root root 0 obj\n");
term_out ("drwxr-xr-x 2 root root 0 joint\n");
term_out ("-rw-r--r-- 1 root root 0 world\n");
term_out ("-rw-r--r-- 1 root root 0 camera\n");
term_out ("-rw-r--r-- 1 root root 0 spawner\n");
} else {
term_out ("obj/\njoint/\nworld\ncamera\nspawner\n");
}
}

static void term_print_object_cat (int object_index) {
rigidbody *rigid_body = &obj_per_scene [object_index];
term_printf ("term_echo", "/obj/%d\n", object_index);
term_printf (NULL, "  id:         %u\n", rigid_body -> object_id);
term_printf (NULL, "  type:       %s\n", term_object_type_name (rigid_body));
term_printf (NULL, "  state:      %s\n", term_object_state_name (rigid_body));
term_printf (NULL, "  mass:       %.4f\n", rigid_body -> mass);
term_printf (NULL, "  inv_mass:   %.4f\n", rigid_body -> inverse_mass);
if (rigid_body -> type == object_sphere) {
term_printf (NULL, "  radius:     %.4f\n", rigid_body -> radius);
} else {
term_printf (NULL, "  half_ext:   (%.4f, %.4f, %.4f)\n",
rigid_body -> half_extensions.x, rigid_body -> half_extensions.y, rigid_body -> half_extensions.z);
}
term_printf (NULL, "  position:   (%.4f, %.4f, %.4f)\n",
rigid_body -> position.x, rigid_body -> position.y, rigid_body -> position.z);
term_printf (NULL, "  velocity:   (%.4f, %.4f, %.4f)\n",
rigid_body -> velocity.x, rigid_body -> velocity.y, rigid_body -> velocity.z);
term_printf (NULL, "  angular_v:  (%.4f, %.4f, %.4f)\n",
rigid_body -> angular_velocity.x, rigid_body -> angular_velocity.y, rigid_body -> angular_velocity.z);
term_printf (NULL, "  orient:     (%.4f, %.4f, %.4f, %.4f)\n",
rigid_body -> orientation.w, rigid_body -> orientation.x,
rigid_body -> orientation.y, rigid_body -> orientation.z);
term_printf (NULL, "  friction:   s=%.3f k=%.3f\n", rigid_body -> friction_static, rigid_body -> friction_kinetic);
term_printf (NULL, "  restitution:%.3f\n", rigid_body -> restitution);
term_printf (NULL, "  colour:     (%.2f, %.2f, %.2f)\n",
rigid_body -> colour.x, rigid_body -> colour.y, rigid_body -> colour.z);
term_printf (NULL, "  sleep_time: %.2f\n", rigid_body -> sleep_timer);
}

static void term_print_joint_cat (int joint_index) {
spring_joint *joint = &joint_pool [joint_index];
int index_a = scene_find_object_index_by_id (joint -> object_id_a);
int index_b = scene_find_object_index_by_id (joint -> object_id_b);
term_printf ("term_echo", "/joint/%d\n", joint_index);
term_printf (NULL, "  object_a:   %d (id=%u)\n", index_a, joint -> object_id_a);
term_printf (NULL, "  object_b:   %d (id=%u)\n", index_b, joint -> object_id_b);
term_printf (NULL, "  length:     %.4f\n", joint -> equilibrium_length);
term_printf (NULL, "  stiffness:  %.4f\n", joint -> spring_constant);
term_printf (NULL, "  damping:    %.4f\n", joint -> damping_coefficient);
}

static void term_print_world (void) {
term_printf ("term_echo", "/world\n");
term_printf (NULL, "  version:            %s\n", A3_VERSION_STRING);
term_printf (NULL, "  mode:               %s\n", main_inputs.is_debug_mode_active ? "debug" : "game");
term_printf (NULL, "  gravity:            %.4f\n", world_gravity_y);
term_printf (NULL, "  drag:               %.4f\n", world_drag_coefficient);
term_printf (NULL, "  floor_friction_s:   %.4f\n", world_surface_friction_static);
term_printf (NULL, "  floor_friction_k:   %.4f\n", world_surface_friction_kinetic);
term_printf (NULL, "  objects:            %d\n", object_count);
term_printf (NULL, "  joints:             %d\n", current_joint_count);
}

static void term_print_camera (void) {
term_printf ("term_echo", "/camera\n");
term_printf (NULL, "  position:   (%.4f, %.4f, %.4f)\n",
main_camera_fov.position.x, main_camera_fov.position.y, main_camera_fov.position.z);
term_printf (NULL, "  yaw:        %.4f\n", main_camera_fov.yaw);
term_printf (NULL, "  pitch:      %.4f\n", main_camera_fov.pitch);
term_printf (NULL, "  speed:      %.4f\n", main_camera_fov.movement_speed);
term_printf (NULL, "  jump:       %.4f\n", jump_height);
}

static void term_print_spawner (void) {
term_printf ("term_echo", "/spawner\n");
term_printf (NULL, "  type:        %s\n", (main_inputs.current_spawn_type == 0) ? "sphere" : "cube");
term_printf (NULL, "  mass:        %.4f\n", spawn_mass);
term_printf (NULL, "  radius:      %.4f\n", spawn_radius);
term_printf (NULL, "  cube_mass:   %.4f\n", spawn_cube_mass);
term_printf (NULL, "  cube_extent: %.4f\n", spawn_cube_extent);
term_printf (NULL, "  speed:       %.4f\n", spawn_speed);
term_printf (NULL, "  friction_s:  %.4f\n", friction_static);
term_printf (NULL, "  friction_k:  %.4f\n", friction_kinetic);
}

/* ------------------------------------------------------------------ */
/* Scene mutation helpers                                              */
/* ------------------------------------------------------------------ */

static int term_create_object (object_type spawn_type) {
int created_index = -1;
if (spawn_type == object_sphere) {
vector3 spawn_position = vector3_addition (
main_camera_fov.position,
vector3_scaling (main_camera_fov.forward_vector, spawn_radius + 1.0f)
);
created_index = scene_add_object (spawn_radius, spawn_mass, spawn_position);
} else {
vector3 spawn_position = vector3_addition (
main_camera_fov.position,
vector3_scaling (main_camera_fov.forward_vector, spawn_cube_extent + 1.0f)
);
created_index = scene_add_cube (
spawn_position,
(vector3) {spawn_cube_extent, spawn_cube_extent, spawn_cube_extent},
spawn_cube_mass
);
}
if (created_index < 0) {
term_err ("mpe: touch: cannot create object (scene full?)\n");
return -1;
}
obj_per_scene [created_index].friction_static = friction_static;
obj_per_scene [created_index].friction_kinetic = friction_kinetic;
obj_per_scene [created_index].velocity = vector3_zero ();
obj_per_scene [created_index].angular_velocity = vector3_zero ();
obj_per_scene [created_index].colour = (vector3) {
0.35f + 0.65f * ((float) ((created_index + 0) % 3) / 2.0f),
0.35f + 0.65f * ((float) ((created_index + 1) % 3) / 2.0f),
0.35f + 0.65f * ((float) ((created_index + 2) % 3) / 2.0f)
};
return created_index;
}

static int term_duplicate_object (int source_index) {
if ((source_index < 0) || (source_index >= object_count)) {return -1;}
rigidbody snapshot = obj_per_scene [source_index];
vector3 copy_position = vector3_addition (snapshot.position, (vector3) {1.0f, 0.0f, 0.0f});
int created_index = -1;
if (snapshot.type == object_sphere) {
created_index = scene_add_object (snapshot.radius, snapshot.mass, copy_position);
} else {
created_index = scene_add_cube (copy_position, snapshot.half_extensions, snapshot.mass);
}
if (created_index < 0) {
term_err ("mpe: cp: cannot duplicate object (scene full?)\n");
return -1;
}
rigidbody *created_body = &obj_per_scene [created_index];
created_body -> colour = snapshot.colour;
created_body -> restitution = snapshot.restitution;
created_body -> friction_static = snapshot.friction_static;
created_body -> friction_kinetic = snapshot.friction_kinetic;
created_body -> velocity = snapshot.velocity;
created_body -> angular_velocity = snapshot.angular_velocity;
if (snapshot.static_state) {rigidbody_set_static (created_body, true);}
return created_index;
}

static void term_set_object_mass (int object_index, float new_mass) {
if ((object_index < 0) || (object_index >= object_count)) {return;}
rigidbody *rigid_body = &obj_per_scene [object_index];
if (new_mass < 0.0f) {new_mass = 0.0f;}
if (new_mass <= 0.0f) {
rigidbody_set_static (rigid_body, true);
} else {
if (rigid_body -> static_state) {rigidbody_set_static (rigid_body, false);}
rigid_body -> mass = new_mass;
rigid_body -> inverse_mass = 1.0f / new_mass;
if (rigid_body -> type == object_sphere) {rigidbody_update_inertia_sphere (rigid_body);}
else {rigidbody_update_inertia_cube (rigid_body);}
rigidbody_wake (rigid_body);
}
contact_cache_clear ();
}

static void term_set_object_static (int object_index, bool make_static) {
if ((object_index < 0) || (object_index >= object_count)) {return;}
rigidbody *rigid_body = &obj_per_scene [object_index];
rigidbody_set_static (rigid_body, make_static);
contact_cache_clear ();
}

static bool term_mode_is_static (const char *mode_text) {
if (term_str_eq (mode_text, "static")) {return true;}
if (term_str_eq (mode_text, "dynamic")) {return false;}
if (term_str_eq (mode_text, "0")) {return true;}
if (term_str_eq (mode_text, "000")) {return true;}
if (term_str_eq (mode_text, "-x")) {return true;}
if (term_str_eq (mode_text, "+x")) {return false;}
char *endptr = NULL;
long mode_bits = strtol (mode_text, &endptr, 8);
if ((endptr != mode_text) && (*endptr == '\0')) {
if (mode_bits == 0) {return true;}
return false;
}
return false;
}

/* ------------------------------------------------------------------ */
/* Command implementations                                             */
/* ------------------------------------------------------------------ */

static void cmd_help (int argc, char **argv) {
(void) argc; (void) argv;
term_dim ("POSIX-style MPE debug shell. Mutating commands require debug mode.\n");
for (size_t command_index = 0; command_index < TERMINAL_COMMAND_COUNT; command_index++) {
term_printf (NULL, "  %-8s %-44s %s\n",
terminal_commands [command_index].name,
terminal_commands [command_index].usage,
terminal_commands [command_index].description);
}
}

static void cmd_man (int argc, char **argv) {
if (argc < 2) {
term_err ("usage: man <command>\n");
return;
}
for (size_t command_index = 0; command_index < TERMINAL_COMMAND_COUNT; command_index++) {
if (term_str_eq (argv [1], terminal_commands [command_index].name)) {
term_printf ("term_echo", "NAME\n");
term_printf (NULL, "    %s - %s\n\n", terminal_commands [command_index].name, terminal_commands [command_index].description);
term_printf ("term_echo", "SYNOPSIS\n");
term_printf (NULL, "    %s\n", terminal_commands [command_index].usage);
return;
}
}
term_printf ("term_err", "mpe: no manual entry for %s\n", argv [1]);
}

static void cmd_clear (int argc, char **argv) {
(void) argc; (void) argv;
if (terminal_output_buffer) {gtk_text_buffer_set_text (terminal_output_buffer, "", -1);}
}

static void cmd_history (int argc, char **argv) {
(void) argc; (void) argv;
if (term_history_count == 0) {
term_dim ("(no history)\n");
return;
}
for (int history_index = term_history_count - 1; history_index >= 0; history_index--) {
term_printf (NULL, "%4d %s\n", term_history_count - history_index, term_history [history_index]);
}
}

static void cmd_pwd (int argc, char **argv) {
(void) argc; (void) argv;
term_printf (NULL, "%s\n", term_cwd);
}

static void cmd_cd (int argc, char **argv) {
const char *target = (argc > 1) ? argv [1] : "/";
if (term_str_eq (target, "~") || term_str_eq (target, "/") || term_str_eq (target, "..")) {
snprintf (term_cwd, sizeof (term_cwd), "/");
} else if (strstr (target, "joint")) {
snprintf (term_cwd, sizeof (term_cwd), "/joint");
} else if (strstr (target, "obj")) {
snprintf (term_cwd, sizeof (term_cwd), "/obj");
} else {
term_printf ("term_err", "mpe: cd: %s: No such directory\n", target);
return;
}
term_update_prompt ();
}

static void term_ls_internal (bool long_format, int argc, char **argv) {
const char *path = term_cwd;
for (int argument_index = 1; argument_index < argc; argument_index++) {
if (argv [argument_index][0] != '-') {
path = argv [argument_index];
break;
}
}
if (term_str_eq (path, "/")) {term_list_root (long_format);}
else if (strstr (path, "joint")) {term_list_joints (long_format);}
else if (strstr (path, "obj")) {term_list_objects (long_format);}
else if (strstr (term_cwd, "joint")) {term_list_joints (long_format);}
else if (strstr (term_cwd, "obj")) {term_list_objects (long_format);}
else {term_list_root (long_format);}
}

static void cmd_ls (int argc, char **argv) {
bool long_format = false;
for (int argument_index = 1; argument_index < argc; argument_index++) {
if ((argv [argument_index][0] == '-') && (strstr (argv [argument_index], "l"))) {long_format = true;}
}
term_ls_internal (long_format, argc, argv);
}

static void cmd_ll (int argc, char **argv) {
term_ls_internal (true, argc, argv);
}

static void cmd_cat (int argc, char **argv) {
if (argc < 2) {
term_err ("usage: cat <path...>\n");
return;
}
for (int argument_index = 1; argument_index < argc; argument_index++) {
const char *target = argv [argument_index];
if (strstr (target, "world")) {term_print_world ();}
else if (strstr (target, "camera")) {term_print_camera ();}
else if (strstr (target, "spawner")) {term_print_spawner ();}
else if (term_classify_token (target) == TERM_TARGET_JOINT) {
int joint_index = term_joint_from_token (target);
if (joint_index >= 0) {term_print_joint_cat (joint_index);}
else {term_printf ("term_err", "mpe: %s: No such joint\n", target);}
} else {
int object_index = term_object_from_token (target);
if (object_index >= 0) {term_print_object_cat (object_index);}
else {term_printf ("term_err", "mpe: %s: No such object\n", target);}
}
}
}

static void cmd_touch (int argc, char **argv) {
if (argc < 2) {
int created_index = term_create_object (object_sphere);
if (created_index >= 0) {term_printf ("term_ok", "/obj/%d\n", created_index);}
return;
}
for (int argument_index = 1; argument_index < argc; argument_index++) {
if (argv [argument_index][0] == '-') {continue;}
object_type spawn_type = object_sphere;
if (strstr (argv [argument_index], "cube")) {spawn_type = object_cube;}
int created_index = term_create_object (spawn_type);
if (created_index >= 0) {term_printf ("term_ok", "/obj/%d\n", created_index);}
}
}

static void cmd_cp (int argc, char **argv) {
if (argc < 2) {
term_err ("usage: cp <object> [dest]\n");
return;
}
int source_index = term_require_object (argv [1]);
if (source_index < 0) {return;}
int created_index = term_duplicate_object (source_index);
if (created_index >= 0) {term_printf ("term_ok", "/obj/%d\n", created_index);}
}

static void cmd_rm (int argc, char **argv) {
if (argc < 2) {
term_err ("usage: rm [-rf] <path...>\n");
return;
}
int delete_count = 0;
for (int argument_index = 1; argument_index < argc; argument_index++) {
if (argv [argument_index][0] == '-') {continue;}
const char *target = argv [argument_index];
bool all_targets = term_is_all_token (target);
term_target_kind kind = term_classify_token (target);
if (all_targets) {
if (kind == TERM_TARGET_JOINT) {
int removed_count = 0;
for (int joint_index = 0; joint_index < MPE_MAX_JOINTS; joint_index++) {
if (joint_pool [joint_index].is_active) {remove_joint (joint_index); removed_count++;}
}
term_printf ("term_ok", "removed %d joint(s)\n", removed_count);
} else {
scene_clear ();
clear_selection ();
contact_cache_clear ();
main_inputs.object_menu_level = 0;
main_inputs.marked_joint_object_index = -1;
main_inputs.is_menu_open = false;
main_inputs.spawner_menu_level = 0;
main_inputs.velocity_menu_level = 0;
delete_count = 0;
term_ok ("removed all objects\n");
}
continue;
}
if (kind == TERM_TARGET_JOINT) {
int joint_index = term_joint_from_token (target);
if (joint_index >= 0) {
remove_joint (joint_index);
term_printf ("term_ok", "removed /joint/%d\n", joint_index);
} else {
term_printf ("term_err", "mpe: %s: No such joint\n", target);
}
} else {
int object_index = term_object_from_token (target);
if (object_index >= 0) {
if (delete_count < MPE_MAX_BODIES) {
term_id_buffer [delete_count++] = obj_per_scene [object_index].object_id;
}
} else {
term_printf ("term_err", "mpe: %s: No such object\n", target);
}
}
}
if (delete_count > 0) {
for (int delete_index = 0; delete_index < delete_count; delete_index++) {
int object_index = scene_find_object_index_by_id (term_id_buffer [delete_index]);
if (object_index >= 0) {scene_remove_object_by_index (object_index);}
}
term_printf ("term_ok", "removed %d object(s)\n", delete_count);
}
}

static void cmd_mv (int argc, char **argv) {
if (argc < 3) {
term_err ("usage: mv <object> /pos/x/y/z | /vel/dx/dy/dz\n");
return;
}
int object_index = term_require_object (argv [1]);
if (object_index < 0) {return;}
rigidbody *rigid_body = &obj_per_scene [object_index];
float x = 0.0f, y = 0.0f, z = 0.0f;
int movement_kind = term_parse_movement_destination (argv [2], &x, &y, &z);
if (movement_kind == 1) {
rigid_body -> position = (vector3) {x, y, z};
rigidbody_wake (rigid_body);
term_printf ("term_ok", "/obj/%d moved to (%.2f, %.2f, %.2f)\n", object_index, x, y, z);
} else if (movement_kind == 2) {
rigid_body -> velocity = vector3_addition (rigid_body -> velocity, (vector3) {x, y, z});
rigidbody_wake (rigid_body);
term_printf ("term_ok", "/obj/%d impulse (%.2f, %.2f, %.2f)\n", object_index, x, y, z);
} else {
term_err ("usage: mv <object> /pos/x/y/z | /vel/dx/dy/dz\n");
}
}

static void cmd_ln (int argc, char **argv) {
bool soft_joint = false;
int argument_index = 1;
if ((argc > 1) && (term_str_eq (argv [1], "-s"))) {
soft_joint = true;
argument_index = 2;
}
if (argc < argument_index + 2) {
term_err ("usage: ln [-s] <object> <object>\n");
return;
}
int index_a = term_require_object (argv [argument_index]);
if (index_a < 0) {return;}
int index_b = term_require_object (argv [argument_index + 1]);
if (index_b < 0) {return;}
if (index_a == index_b) {
term_err ("mpe: ln: cannot link an object to itself\n");
return;
}
float rest_length = vector3_length (vector3_subtraction (
obj_per_scene [index_b].position,
obj_per_scene [index_a].position
));
float spring_constant = soft_joint ? 20.0f : 100.0f;
float damping_coefficient = soft_joint ? 1.0f : 2.0f;
int joint_index = add_joint (index_a, index_b, rest_length, spring_constant, damping_coefficient);
if (joint_index < 0) {
term_err ("mpe: ln: cannot create joint\n");
return;
}
term_printf ("term_ok", "/joint/%d -> /obj/%d -> /obj/%d\n", joint_index, index_a, index_b);
}

static void cmd_unlink (int argc, char **argv) {
if (argc < 2) {
term_err ("usage: unlink <path>\n");
return;
}
const char *target = argv [1];
if (term_classify_token (target) == TERM_TARGET_JOINT) {
int joint_index = term_require_joint (target);
if (joint_index >= 0) {
remove_joint (joint_index);
term_printf ("term_ok", "removed /joint/%d\n", joint_index);
}
} else {
int object_index = term_require_object (target);
if (object_index >= 0) {
scene_remove_object_by_index (object_index);
term_printf ("term_ok", "removed /obj/%d\n", object_index);
}
}
}

static void cmd_chmod (int argc, char **argv) {
if (argc < 3) {
term_err ("usage: chmod static|dynamic|mode <object...>\n");
return;
}
bool make_static = term_mode_is_static (argv [1]);
for (int argument_index = 2; argument_index < argc; argument_index++) {
int object_index = term_require_object (argv [argument_index]);
if (object_index < 0) {continue;}
term_set_object_static (object_index, make_static);
term_printf ("term_ok", "/obj/%d -> %s\n", object_index, make_static ? "static" : "dynamic");
}
}

static void cmd_chown (int argc, char **argv) {
if (argc < 3) {
term_err ("usage: chown <mass> <object...>\n");
return;
}
float new_mass = 0.0f;
if (!term_parse_float (argv [1], &new_mass)) {
term_printf ("term_err", "mpe: chown: invalid mass '%s'\n", argv [1]);
return;
}
for (int argument_index = 2; argument_index < argc; argument_index++) {
int object_index = term_require_object (argv [argument_index]);
if (object_index < 0) {continue;}
term_set_object_mass (object_index, new_mass);
term_printf ("term_ok", "/obj/%d mass=%.3f\n", object_index, new_mass);
}
}

static void cmd_kill (int argc, char **argv) {
if (argc < 2) {
term_err ("usage: kill [-STOP|-CONT|-9|-TERM] <object...>\n");
return;
}
enum {KILL_TERM, KILL_STOP, KILL_CONT};
int kill_action = KILL_TERM;
int argument_index = 1;
if ((argc > 1) && ((argv [1][0] == '-') || (g_str_has_prefix (argv [1], "SIG")))) {
const char *signal_text = argv [1];
if (signal_text [0] == '-') {signal_text++;}
if (g_str_has_prefix (signal_text, "SIG")) {signal_text += 3;}
if (term_str_eq (signal_text, "STOP") || term_str_eq (signal_text, "19")) {kill_action = KILL_STOP;}
else if (term_str_eq (signal_text, "CONT") || term_str_eq (signal_text, "18")) {kill_action = KILL_CONT;}
else {kill_action = KILL_TERM;}
argument_index = 2;
}
int delete_count = 0;
for (; argument_index < argc; argument_index++) {
const char *target = argv [argument_index];
if (term_is_all_token (target)) {
if (kill_action == KILL_STOP) {
for (int object_index = 0; object_index < object_count; object_index++) {
obj_per_scene [object_index].velocity = vector3_zero ();
obj_per_scene [object_index].angular_velocity = vector3_zero ();
obj_per_scene [object_index].is_sleeping = true;
obj_per_scene [object_index].sleep_timer = 2.0f;
}
term_ok ("stopped all objects\n");
} else if (kill_action == KILL_CONT) {
for (int object_index = 0; object_index < object_count; object_index++) {
rigidbody_wake (&obj_per_scene [object_index]);
}
term_ok ("continued all objects\n");
} else {
scene_clear ();
clear_selection ();
contact_cache_clear ();
main_inputs.object_menu_level = 0;
main_inputs.marked_joint_object_index = -1;
main_inputs.is_menu_open = false;
main_inputs.spawner_menu_level = 0;
main_inputs.velocity_menu_level = 0;
delete_count = 0;
term_ok ("killed all objects\n");
}
continue;
}
int object_index = term_object_from_token (target);
if (object_index < 0) {
term_printf ("term_err", "mpe: %s: No such object\n", target);
continue;
}
rigidbody *rigid_body = &obj_per_scene [object_index];
if (kill_action == KILL_STOP) {
rigid_body -> velocity = vector3_zero ();
rigid_body -> angular_velocity = vector3_zero ();
rigid_body -> is_sleeping = true;
rigid_body -> sleep_timer = 2.0f;
term_printf ("term_ok", "stopped /obj/%d\n", object_index);
} else if (kill_action == KILL_CONT) {
rigidbody_wake (rigid_body);
term_printf ("term_ok", "continued /obj/%d\n", object_index);
} else {
if (delete_count < MPE_MAX_BODIES) {
term_id_buffer [delete_count++] = rigid_body -> object_id;
}
}
}
if (delete_count > 0) {
for (int delete_index = 0; delete_index < delete_count; delete_index++) {
int object_index = scene_find_object_index_by_id (term_id_buffer [delete_index]);
if (object_index >= 0) {scene_remove_object_by_index (object_index);}
}
term_printf ("term_ok", "killed %d object(s)\n", delete_count);
}
}

static void cmd_ps (int argc, char **argv) {
bool detailed = false;
for (int argument_index = 1; argument_index < argc; argument_index++) {
if (strstr (argv [argument_index], "aux") || strstr (argv [argument_index], "-a")) {detailed = true;}
}
if (object_count == 0) {
term_dim ("(no objects)\n");
return;
}
if (detailed) {
term_printf (NULL, "%4s %6s %-4s %-6s %8s %8s %s\n", "PID", "ID", "TYPE", "STATE", "MASS", "SPEED", "POSITION");
for (int object_index = 0; object_index < object_count; object_index++) {
rigidbody *rigid_body = &obj_per_scene [object_index];
term_printf (NULL, "%4d %6u %-4s %-6s %8.2f %8.3f (%.2f,%.2f,%.2f)\n",
object_index,
rigid_body -> object_id,
term_object_type_name (rigid_body),
term_object_state_name (rigid_body),
rigid_body -> mass,
vector3_length (rigid_body -> velocity),
rigid_body -> position.x, rigid_body -> position.y, rigid_body -> position.z);
}
} else {
term_printf (NULL, "%4s %6s %-4s %-6s %8s\n", "PID", "ID", "TYPE", "STATE", "MASS");
for (int object_index = 0; object_index < object_count; object_index++) {
rigidbody *rigid_body = &obj_per_scene [object_index];
term_printf (NULL, "%4d %6u %-4s %-6s %8.2f\n",
object_index,
rigid_body -> object_id,
term_object_type_name (rigid_body),
term_object_state_name (rigid_body),
rigid_body -> mass);
}
}
}

static void cmd_top (int argc, char **argv) {
int limit = 10;
for (int argument_index = 1; argument_index < argc; argument_index++) {
if (term_str_eq (argv [argument_index], "-n") && (argument_index + 1 < argc)) {
float parsed_limit = 0.0f;
if (term_parse_float (argv [argument_index + 1], &parsed_limit)) {limit = (int) parsed_limit;}
}
}
if (limit < 1) {limit = 1;}
if (limit > 16) {limit = 16;}
if (object_count == 0) {
term_dim ("(no objects)\n");
return;
}
int top_indices [16];
float top_speeds [16];
for (int slot_index = 0; slot_index < limit; slot_index++) {
top_indices [slot_index] = -1;
top_speeds [slot_index] = -1.0f;
}
for (int slot_index = 0; slot_index < limit; slot_index++) {
int best_index = -1;
float best_speed = -1.0f;
for (int object_index = 0; object_index < object_count; object_index++) {
bool already_listed = false;
for (int previous_slot = 0; previous_slot < slot_index; previous_slot++) {
if (top_indices [previous_slot] == object_index) {already_listed = true; break;}
}
if (already_listed) {continue;}
float object_speed = vector3_length (obj_per_scene [object_index].velocity);
if (object_speed > best_speed) {
best_speed = object_speed;
best_index = object_index;
}
}
if (best_index < 0) {break;}
top_indices [slot_index] = best_index;
top_speeds [slot_index] = best_speed;
}
term_printf (NULL, "%4s %-4s %-6s %8s %s\n", "PID", "TYPE", "STATE", "SPEED", "POSITION");
for (int slot_index = 0; slot_index < limit; slot_index++) {
if (top_indices [slot_index] < 0) {break;}
rigidbody *rigid_body = &obj_per_scene [top_indices [slot_index]];
term_printf (NULL, "%4d %-4s %-6s %8.3f (%.2f,%.2f,%.2f)\n",
top_indices [slot_index],
term_object_type_name (rigid_body),
term_object_state_name (rigid_body),
top_speeds [slot_index],
rigid_body -> position.x, rigid_body -> position.y, rigid_body -> position.z);
}
}

static void cmd_df (int argc, char **argv) {
(void) argc; (void) argv;
int object_capacity_value = (object_capacity > 0) ? object_capacity : MPE_MAX_BODIES;
int joint_capacity_value = MPE_MAX_JOINTS;
int object_percent = (object_capacity_value > 0) ? (object_count * 100 / object_capacity_value) : 0;
int joint_percent = (joint_capacity_value > 0) ? (current_joint_count * 100 / joint_capacity_value) : 0;
term_printf (NULL, "Filesystem     Size   Used  Avail Use%% Mounted on\n");
term_printf (NULL, "objects       %6d %6d %6d %3d%% /obj\n",
object_capacity_value, object_count, object_capacity_value - object_count, object_percent);
term_printf (NULL, "joints        %6d %6d %6d %3d%% /joint\n",
joint_capacity_value, current_joint_count, joint_capacity_value - current_joint_count, joint_percent);
}

static void cmd_du (int argc, char **argv) {
if (argc < 2) {
term_err ("usage: du <path...>\n");
return;
}
for (int argument_index = 1; argument_index < argc; argument_index++) {
const char *target = argv [argument_index];
if (term_classify_token (target) == TERM_TARGET_JOINT) {
int joint_index = term_joint_from_token (target);
if (joint_index >= 0) {
spring_joint *joint = &joint_pool [joint_index];
term_printf (NULL, "/joint/%d len=%.2f k=%.1f d=%.1f\n",
joint_index, joint -> equilibrium_length, joint -> spring_constant, joint -> damping_coefficient);
} else {
term_printf ("term_err", "mpe: %s: No such joint\n", target);
}
} else {
int object_index = term_object_from_token (target);
if (object_index >= 0) {
rigidbody *rigid_body = &obj_per_scene [object_index];
float size_value = (rigid_body -> type == object_sphere) ? rigid_body -> radius : vector3_length (rigid_body -> half_extensions);
term_printf (NULL, "/obj/%d mass=%.2f size=%.2f\n", object_index, rigid_body -> mass, size_value);
} else {
term_printf ("term_err", "mpe: %s: No such object\n", target);
}
}
}
}

static void cmd_uname (int argc, char **argv) {
/* MPE_TASK_23_FIX_UNAME_WARNING_BEGIN */
(void) argv;
/* MPE_TASK_23_FIX_UNAME_WARNING_END */
if (argc > 1) {
term_printf (NULL, "MPE %s physics-shell x86_64 POSIX-like GTK3 OpenGL\n", A3_VERSION_STRING);
} else {
term_printf (NULL, "MPE %s\n", A3_VERSION_STRING);
}
}

static void cmd_whoami (int argc, char **argv) {
(void) argc; (void) argv;
term_out ("root\n");
}

static void cmd_date (int argc, char **argv) {
(void) argc; (void) argv;
time_t current_time = time (NULL);
struct tm *local_time = localtime (&current_time);
char time_buffer [128];
strftime (time_buffer, sizeof (time_buffer), "%a %Y-%m-%d %H:%M:%S %Z", local_time);
term_printf (NULL, "%s\n", time_buffer);
}

static void cmd_echo (int argc, char **argv) {
for (int argument_index = 1; argument_index < argc; argument_index++) {
term_out (argv [argument_index]);
if (argument_index + 1 < argc) {term_out (" ");}
}
term_out ("\n");
}

static void cmd_env (int argc, char **argv) {
(void) argc; (void) argv;
term_printf (NULL, "GRAVITY=%.4f\n", world_gravity_y);
term_printf (NULL, "DRAG=%.4f\n", world_drag_coefficient);
term_printf (NULL, "FRICTION_STATIC=%.4f\n", world_surface_friction_static);
term_printf (NULL, "FRICTION_KINETIC=%.4f\n", world_surface_friction_kinetic);
term_printf (NULL, "SPAWN_MASS=%.4f\n", spawn_mass);
term_printf (NULL, "SPAWN_RADIUS=%.4f\n", spawn_radius);
term_printf (NULL, "SPAWN_CUBE_MASS=%.4f\n", spawn_cube_mass);
term_printf (NULL, "SPAWN_CUBE_EXTENT=%.4f\n", spawn_cube_extent);
term_printf (NULL, "SPAWN_SPEED=%.4f\n", spawn_speed);
term_printf (NULL, "CAMERA_SPEED=%.4f\n", main_camera_fov.movement_speed);
term_printf (NULL, "JUMP_HEIGHT=%.4f\n", jump_height);
term_printf (NULL, "CHANGE_RATE=%.4f\n", variable_change_rate);
}

static void cmd_export (int argc, char **argv) {
if (argc < 2) {
cmd_env (argc, argv);
return;
}
for (int argument_index = 1; argument_index < argc; argument_index++) {
char **parts = g_strsplit (argv [argument_index], "=", 2);
if ((!parts [0]) || (!parts [1])) {
term_printf ("term_err", "mpe: export: usage: export VAR=value\n");
g_strfreev (parts);
continue;
}
const char *variable_name = parts [0];
float variable_value = 0.0f;
if (!term_parse_float (parts [1], &variable_value)) {
term_printf ("term_err", "mpe: export: invalid value '%s'\n", parts [1]);
g_strfreev (parts);
continue;
}
bool variable_found = true;
if (term_str_eq (variable_name, "GRAVITY")) {world_gravity_y = variable_value;}
else if (term_str_eq (variable_name, "DRAG")) {
if (variable_value < 0.1f) {variable_value = 0.1f;}
if (variable_value > 1.0f) {variable_value = 1.0f;}
world_drag_coefficient = variable_value;
}
else if (term_str_eq (variable_name, "FRICTION_STATIC")) {world_surface_friction_static = variable_value;}
else if (term_str_eq (variable_name, "FRICTION_KINETIC")) {world_surface_friction_kinetic = variable_value;}
else if (term_str_eq (variable_name, "SPAWN_MASS")) {spawn_mass = variable_value;}
else if (term_str_eq (variable_name, "SPAWN_RADIUS")) {spawn_radius = variable_value;}
else if (term_str_eq (variable_name, "SPAWN_CUBE_MASS")) {spawn_cube_mass = variable_value;}
else if (term_str_eq (variable_name, "SPAWN_CUBE_EXTENT")) {spawn_cube_extent = variable_value;}
else if (term_str_eq (variable_name, "SPAWN_SPEED")) {spawn_speed = variable_value;}
else if (term_str_eq (variable_name, "CAMERA_SPEED")) {main_camera_fov.movement_speed = variable_value;}
else if (term_str_eq (variable_name, "JUMP_HEIGHT")) {jump_height = variable_value;}
else if (term_str_eq (variable_name, "CHANGE_RATE")) {variable_change_rate = variable_value;}
else {variable_found = false;}
if (variable_found) {
term_printf ("term_ok", "%s=%.4f\n", variable_name, variable_value);
} else {
term_printf ("term_err", "mpe: export: %s: unknown variable\n", variable_name);
}
g_strfreev (parts);
}
}

/* ------------------------------------------------------------------ */
/* Execution                                                           */
/* ------------------------------------------------------------------ */

static void term_execute (char *command_line) {
while (*command_line == ' ') {command_line++;}
if (*command_line == '\0') {return;}
char prompt_buffer [320];
snprintf (prompt_buffer, sizeof (prompt_buffer), "mpe:%s> ", term_cwd);
term_echo (prompt_buffer);
term_out (command_line);
term_out ("\n");

int argument_count = 0;
char **argument_vector = NULL;
GError *parse_error = NULL;

if (!g_shell_parse_argv (command_line, &argument_count, &argument_vector, &parse_error)) {
if (parse_error) {
term_printf ("term_err", "mpe: %s\n", parse_error -> message);
g_clear_error (&parse_error);
} else {
term_err ("mpe: parse error\n");
}
return;
}

if (argument_count <= 0) {
if (argument_vector) {g_strfreev (argument_vector);}
return;
}

const terminal_command *found_command = NULL;
for (size_t command_index = 0; command_index < TERMINAL_COMMAND_COUNT; command_index++) {
if (term_str_eq (argument_vector [0], terminal_commands [command_index].name)) {
found_command = &terminal_commands [command_index];
break;
}
}

if (!found_command) {
term_printf ("term_err", "mpe: %s: command not found\n", argument_vector [0]);
g_strfreev (argument_vector);
return;
}

if ((found_command -> mutates) && (!main_inputs.is_debug_mode_active)) {
term_printf ("term_err", "mpe: %s: Permission denied (switch to debug mode with 0)\n", found_command -> name);
g_strfreev (argument_vector);
return;
}

found_command -> handler (argument_count, argument_vector);
g_strfreev (argument_vector);
}

/* ------------------------------------------------------------------ */
/* GTK signals                                                         */
/* ------------------------------------------------------------------ */

static void on_terminal_entry_activate (GtkEntry *entry) {
const gchar *entry_text = gtk_entry_get_text (entry);
char command_copy [TERM_HISTORY_LENGTH + 1];
strncpy (command_copy, entry_text, TERM_HISTORY_LENGTH);
command_copy [TERM_HISTORY_LENGTH] = '\0';
term_history_push (command_copy);
term_history_cursor = -1;
term_execute (command_copy);
gtk_entry_set_text (entry, "");
}

static gboolean on_terminal_entry_keypress (GtkWidget *widget, GdkEventKey *event) {
if (event -> keyval == GDK_KEY_Up) {
if (term_history_count > 0) {
if (term_history_cursor < term_history_count - 1) {term_history_cursor++;}
gtk_entry_set_text (GTK_ENTRY (widget), term_history [term_history_cursor]);
gtk_editable_set_position (GTK_EDITABLE (widget), -1);
}
return TRUE;
}
if (event -> keyval == GDK_KEY_Down) {
if (term_history_cursor > 0) {
term_history_cursor--;
gtk_entry_set_text (GTK_ENTRY (widget), term_history [term_history_cursor]);
} else {
term_history_cursor = -1;
gtk_entry_set_text (GTK_ENTRY (widget), "");
}
gtk_editable_set_position (GTK_EDITABLE (widget), -1);
return TRUE;
}
if ((event -> state & GDK_CONTROL_MASK) && ((event -> keyval == GDK_KEY_l) || (event -> keyval == GDK_KEY_L))) {
if (terminal_output_buffer) {gtk_text_buffer_set_text (terminal_output_buffer, "", -1);}
return TRUE;
}
return FALSE;
}

static gboolean on_terminal_window_keypress (GtkWidget *widget, GdkEventKey *event) {
if (event -> keyval == GDK_KEY_Escape) {
gtk_widget_destroy (widget);
return TRUE;
}
return FALSE;
}

static void on_terminal_window_destroy (GtkWidget *widget) {
(void) widget;
terminal_window = NULL;
terminal_output_view = NULL;
terminal_output_buffer = NULL;
terminal_entry = NULL;
terminal_prompt_label = NULL;
term_history_cursor = -1;
}

/* ------------------------------------------------------------------ */
/* Public interface                                                    */
/* ------------------------------------------------------------------ */

bool debug_terminal_is_open (void) {
return terminal_window != NULL;
}

void debug_terminal_focus_entry (void) {
if ((!terminal_window) || (!terminal_entry)) {return;}
gtk_window_present (GTK_WINDOW (terminal_window));
gtk_widget_grab_focus (terminal_entry);
}

void debug_terminal_sync_mode (void) {
if (!terminal_window) {return;}
if (main_inputs.is_debug_mode_active) {
gtk_window_set_title (GTK_WINDOW (terminal_window), "MPE POSIX Debug Terminal - debug mode");
term_ok ("[terminal unlocked] debug mode active\n");
} else {
gtk_window_set_title (GTK_WINDOW (terminal_window), "MPE POSIX Debug Terminal - LOCKED (game mode)");
term_err ("[terminal locked] game mode — read-only commands only\n");
}
}

void debug_terminal_open (GtkWidget *parent_window) {
if (!main_inputs.is_debug_mode_active) {return;}
if (terminal_window) {
debug_terminal_focus_entry ();
return;
}

terminal_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
gtk_widget_set_name (terminal_window, "mpe-debug-terminal");
gtk_window_set_default_size (GTK_WINDOW (terminal_window), 820, 560);
if ((parent_window) && (GTK_IS_WIDGET (parent_window))) {
gtk_window_set_transient_for (GTK_WINDOW (terminal_window), GTK_WINDOW (parent_window));
}
g_signal_connect (terminal_window, "destroy", G_CALLBACK (on_terminal_window_destroy), NULL);
g_signal_connect (terminal_window, "key-press-event", G_CALLBACK (on_terminal_window_keypress), NULL);

static bool terminal_css_installed = false;
if (!terminal_css_installed) {
GtkCssProvider *css_provider = gtk_css_provider_new ();
gtk_css_provider_load_from_data (css_provider,
"#mpe-debug-terminal { background: #0b111a; }\n"
"#mpe-debug-terminal textview { background: #0b111a; color: #cdd6e4; font-family: monospace; font-size: 13px; }\n"
"#mpe-debug-terminal textview text { background: #0b111a; }\n"
"#mpe-debug-terminal entry { background: #101826; color: #ffcf87; caret-color: #ffcf87; font-family: monospace; font-size: 13px; border: none; padding: 6px 8px; }\n"
"#mpe-debug-terminal label { color: #ffcf87; font-family: monospace; font-weight: bold; }\n",
-1, NULL);
gtk_style_context_add_provider_for_screen (gdk_screen_get_default (),
GTK_STYLE_PROVIDER (css_provider),
GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
g_object_unref (css_provider);
terminal_css_installed = true;
}

GtkWidget *root_box = gtk_box_new (GTK_ORIENTATION_VERTICAL, 0);
gtk_container_add (GTK_CONTAINER (terminal_window), root_box);

GtkWidget *scrolled_window = gtk_scrolled_window_new (NULL, NULL);
gtk_box_pack_start (GTK_BOX (root_box), scrolled_window, TRUE, TRUE, 0);

terminal_output_view = gtk_text_view_new ();
gtk_text_view_set_editable (GTK_TEXT_VIEW (terminal_output_view), FALSE);
gtk_text_view_set_cursor_visible (GTK_TEXT_VIEW (terminal_output_view), FALSE);
gtk_text_view_set_wrap_mode (GTK_TEXT_VIEW (terminal_output_view), GTK_WRAP_WORD_CHAR);
gtk_text_view_set_left_margin (GTK_TEXT_VIEW (terminal_output_view), 10);
gtk_text_view_set_right_margin (GTK_TEXT_VIEW (terminal_output_view), 10);
gtk_text_view_set_top_margin (GTK_TEXT_VIEW (terminal_output_view), 10);
terminal_output_buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (terminal_output_view));
gtk_text_buffer_create_tag (terminal_output_buffer, "term_echo",
"foreground", "#ffcf87", "weight", PANGO_WEIGHT_BOLD, NULL);
gtk_text_buffer_create_tag (terminal_output_buffer, "term_ok",
"foreground", "#8be28b", NULL);
gtk_text_buffer_create_tag (terminal_output_buffer, "term_err",
"foreground", "#ff7b72", NULL);
gtk_text_buffer_create_tag (terminal_output_buffer, "term_dim",
"foreground", "#5f7387", NULL);
gtk_container_add (GTK_CONTAINER (scrolled_window), terminal_output_view);

GtkWidget *input_box = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 6);
gtk_box_pack_start (GTK_BOX (root_box), input_box, FALSE, FALSE, 0);

terminal_prompt_label = gtk_label_new ("mpe:/>");
gtk_box_pack_start (GTK_BOX (input_box), terminal_prompt_label, FALSE, FALSE, 10);

terminal_entry = gtk_entry_new ();
gtk_entry_set_placeholder_text (GTK_ENTRY (terminal_entry), "type help and press Enter");
gtk_box_pack_start (GTK_BOX (input_box), terminal_entry, TRUE, TRUE, 10);
g_signal_connect (terminal_entry, "activate", G_CALLBACK (on_terminal_entry_activate), NULL);
g_signal_connect (terminal_entry, "key-press-event", G_CALLBACK (on_terminal_entry_keypress), NULL);

if (main_inputs.is_mouse_locked) {
if ((parent_window) && (GTK_IS_WIDGET (parent_window))) {mouse_lock_disable (parent_window);}
main_inputs.is_mouse_locked = false;
}

gtk_widget_show_all (terminal_window);
term_update_prompt ();
debug_terminal_sync_mode ();

term_printf ("term_echo", "MPE POSIX Debug Terminal %s\n", A3_VERSION_STRING);
term_out ("Virtual root: /obj /joint /world /camera /spawner\n");
term_dim ("Type 'help' or 'man <command>'. Ctrl+L clears. Esc closes.\n");
}

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
/* MPE_TASK_V15R2_OUTPUT_CAPTURE_BEGIN */
static char *term_capture_buffer = NULL;
static size_t term_capture_length = 0;
static size_t term_capture_capacity = 0;
static bool term_capturing = false;

static void term_capture_begin(void) {
term_capturing = true;
term_capture_length = 0;
if (!term_capture_buffer) {
term_capture_capacity = 8192;
term_capture_buffer = malloc(term_capture_capacity);
}
if (term_capture_buffer) { term_capture_buffer[0] = '\0'; }
}

static void term_capture_end(void) {
term_capturing = false;
}

static char *term_capture_get(void) {
return term_capture_buffer ? term_capture_buffer : "";
}

static void term_capture_reset(void) {
if (term_capture_buffer) { free(term_capture_buffer); term_capture_buffer = NULL; }
term_capture_length = 0;
term_capture_capacity = 0;
term_capturing = false;
}
/* MPE_TASK_V15R2_OUTPUT_CAPTURE_END */
/* MPE_TASK_V15R2_PHASE7_ALIAS_STORAGE_BEGIN */
#define TERM_ALIAS_MAX 32
#define TERM_ALIAS_NAME_LEN 64
#define TERM_ALIAS_VALUE_LEN 256
static char term_alias_names [TERM_ALIAS_MAX][TERM_ALIAS_NAME_LEN];
static char term_alias_values [TERM_ALIAS_MAX][TERM_ALIAS_VALUE_LEN];
static int term_alias_count = 0;
static bool term_sudo_active = false;
/* MPE_TASK_V15R2_PHASE7_ALIAS_STORAGE_END */

static void term_append_with_tag (const char *tag_name, const char *text) {
/* MPE_TASK_V15R2_OUTPUT_CAPTURE_INTERCEPT */
if (term_capturing) {
size_t text_len = strlen(text);
while (term_capture_length + text_len + 1 > term_capture_capacity) {
term_capture_capacity *= 2;
term_capture_buffer = realloc(term_capture_buffer, term_capture_capacity);
if (!term_capture_buffer) { term_capturing = false; return; }
}
memcpy(term_capture_buffer + term_capture_length, text, text_len + 1);
term_capture_length += text_len;
return;
}
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
static void cmd_config (int argc, char **argv); /* MPE_TASK_38 */
/* MPE_TASK_V15R2_PHASE2_DECLS_BEGIN */
static void cmd_exit (int argc, char **argv);
static void cmd_logout (int argc, char **argv);
static void cmd_quit (int argc, char **argv);
static void cmd_poweroff (int argc, char **argv);
static void cmd_shutdown (int argc, char **argv);
static void cmd_reboot (int argc, char **argv);
static void cmd_halt (int argc, char **argv);
static void cmd_sleep (int argc, char **argv);
static void cmd_sync (int argc, char **argv);
static void cmd_uptime (int argc, char **argv);
static void cmd_free (int argc, char **argv);
static void cmd_w (int argc, char **argv);
static void cmd_hostname (int argc, char **argv);
static void cmd_id (int argc, char **argv);
static void cmd_which (int argc, char **argv);
static void cmd_true (int argc, char **argv);
static void cmd_false (int argc, char **argv);
static void cmd_time (int argc, char **argv);
static void term_execute (char *command_line);
/* MPE_TASK_V15R2_PHASE2_DECLS_END */
/* MPE_TASK_V15R2_PHASE3_DECLS_BEGIN */
static void cmd_stat (int argc, char **argv);
static void cmd_find (int argc, char **argv);
static void cmd_wc (int argc, char **argv);
static void cmd_file (int argc, char **argv);
static void cmd_diff (int argc, char **argv);
static void cmd_xxd (int argc, char **argv);
/* MPE_TASK_V15R2_PHASE3_DECLS_END */
/* MPE_TASK_V15R2_PHASE4_DECLS_BEGIN */
static void cmd_sort (int argc, char **argv);
static void cmd_grep (int argc, char **argv);
static void cmd_head (int argc, char **argv);
static void cmd_tail (int argc, char **argv);
static void cmd_less (int argc, char **argv);
/* MPE_TASK_V15R2_PHASE4_DECLS_END */
/* MPE_TASK_V15R2_PHASE5_DECLS_BEGIN */
static void cmd_sed (int argc, char **argv);
static void cmd_nice (int argc, char **argv);
static void cmd_renice (int argc, char **argv);
static void cmd_ping (int argc, char **argv);
static void cmd_mount (int argc, char **argv);
static void cmd_umount (int argc, char **argv);
static void cmd_mkfs (int argc, char **argv);
static void cmd_fsck (int argc, char **argv);
/* MPE_TASK_V15R2_PHASE5_DECLS_END */
/* MPE_TASK_V15R2_PHASE6_DECLS_BEGIN */
static void cmd_netstat (int argc, char **argv);
static void cmd_ifconfig (int argc, char **argv);
static void cmd_lsmod (int argc, char **argv);
/* MPE_TASK_V15R2_PHASE6_DECLS_END */
/* MPE_TASK_V15R2_PHASE7_DECLS_BEGIN */
static void cmd_alias (int argc, char **argv);
static void cmd_unalias (int argc, char **argv);
static void cmd_jobs (int argc, char **argv);
static void cmd_lsof (int argc, char **argv);
static void cmd_seq (int argc, char **argv);
static void cmd_tee (int argc, char **argv);
static void cmd_watch (int argc, char **argv);
static void cmd_sudo (int argc, char **argv);
static void cmd_su (int argc, char **argv);
static void cmd_dmesg (int argc, char **argv);
/* MPE_TASK_V15R2_PHASE7_DECLS_END */
/* MPE_TASK_V15R2_PHASE8_DECLS_BEGIN */
static void cmd_vi (int argc, char **argv);
/* MPE_TASK_V15R2_PHASE8_DECLS_END */
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
{"uname",   false, cmd_uname,   "uname [-a|-s|-r|-m|-o]",              "engine version"},
{"whoami",  false, cmd_whoami,  "whoami",                             "print user"},
{"date",    false, cmd_date,    "date",                               "print date/time"},
{"echo",    false, cmd_echo,    "echo [args...]",                     "print arguments"},
{"env",     false, cmd_env,     "env",                                "print all config parameters"},
{"export",  true,  cmd_export,  "export KEY=value",                   "set config parameter"},
{"config",  true,  cmd_config,  "config save|load|reset",             "manage config file"},
/* MPE_TASK_V15R2_PHASE2_TABLE_BEGIN */
{"exit",     false, cmd_exit,     "exit",                              "close terminal"},
{"logout",   false, cmd_logout,   "logout",                            "close terminal"},
{"quit",     false, cmd_quit,     "quit",                              "close terminal"},
{"poweroff", true,  cmd_poweroff, "poweroff",                          "save config and exit engine"},
{"shutdown", true,  cmd_shutdown, "shutdown",                          "save config and exit engine"},
{"reboot",   true,  cmd_reboot,   "reboot",                            "reset scene to default"},
{"halt",     true,  cmd_halt,     "halt",                              "pause/resume physics"},
{"sleep",    true,  cmd_sleep,    "sleep <seconds>",                   "pause physics for N seconds"},
{"sync",     true,  cmd_sync,     "sync",                              "force-save config and scene"},
{"uptime",   false, cmd_uptime,   "uptime",                            "engine runtime stats"},
{"free",     false, cmd_free,     "free",                              "capacity usage report"},
{"w",        false, cmd_w,        "w",                                 "camera and input state"},
{"hostname", false, cmd_hostname, "hostname [-f]",                     "engine identity"},
{"id",       false, cmd_id,       "id",                                "permission context"},
{"which",    false, cmd_which,    "which <command>",                   "locate command"},
{"true",     false, cmd_true,     "true",                              "no-op success"},
{"false",    false, cmd_false,    "false",                             "no-op failure"},
{"time",     false, cmd_time,     "time <command...>",                 "measure command duration"},
/* MPE_TASK_V15R2_PHASE2_TABLE_END */
/* MPE_TASK_V15R2_PHASE3_TABLE_BEGIN */
{"stat",     false, cmd_stat,     "stat <path>",                       "detailed object metadata"},
{"find",     false, cmd_find,     "find /obj [filters]",               "search objects by property"},
{"wc",       false, cmd_wc,       "wc [path]",                         "count objects/joints"},
{"file",     false, cmd_file,     "file <path>",                       "identify object type"},
{"diff",     false, cmd_diff,     "diff <obj_a> <obj_b>",              "compare two objects"},
{"xxd",      false, cmd_xxd,      "xxd <object> [-l len] [-s off]",    "hex dump object struct"},
/* MPE_TASK_V15R2_PHASE3_TABLE_END */
/* MPE_TASK_V15R2_PHASE4_TABLE_BEGIN */
{"sort",     false, cmd_sort,     "sort /obj [-k field] [-r]",         "sort objects by property"},
{"grep",     false, cmd_grep,     "grep <pattern> [path]",             "filter listing by pattern"},
{"head",     false, cmd_head,     "head [-n count] [path]",            "first N lines of listing"},
{"tail",     false, cmd_tail,     "tail [-n count] [path]",            "last N lines of listing"},
{"less",     false, cmd_less,     "less [path]",                       "paginated listing viewer"},
/* MPE_TASK_V15R2_PHASE4_TABLE_END */
/* MPE_TASK_V15R2_PHASE5_TABLE_BEGIN */
{"sed",      true,  cmd_sed,      "sed s/field/value/ <target>",       "batch property editor"},
{"nice",     true,  cmd_nice,     "nice <priority> <object>",          "set per-object damping"},
{"renice",   true,  cmd_renice,   "renice <priority> <object>",        "change object damping"},
{"ping",     true,  cmd_ping,     "ping [-c count] <object>",          "test object responsiveness"},
{"mount",    true,  cmd_mount,    "mount <path>",                      "load scene file"},
{"umount",   true,  cmd_umount,   "umount",                            "save and clear scene"},
{"mkfs",     true,  cmd_mkfs,     "mkfs",                              "format scene (clear all)"},
{"fsck",     false, cmd_fsck,     "fsck [-y]",                         "scene integrity check"},
/* MPE_TASK_V15R2_PHASE5_TABLE_END */
/* MPE_TASK_V15R2_PHASE6_TABLE_BEGIN */
{"netstat",  false, cmd_netstat,  "netstat [-a] [-t]",                "joint network topology"},
{"ifconfig", false, cmd_ifconfig, "ifconfig",                          "camera/render/input state"},
{"lsmod",    false, cmd_lsmod,    "lsmod",                             "loaded engine modules"},
/* MPE_TASK_V15R2_PHASE6_TABLE_END */
/* MPE_TASK_V15R2_PHASE7_TABLE_BEGIN */
{"alias",    true,  cmd_alias,    "alias [name=value]",                "define/list command aliases"},
{"unalias",  true,  cmd_unalias,  "unalias <name>",                    "remove command alias"},
{"jobs",     false, cmd_jobs,     "jobs",                              "active validation tests"},
{"lsof",     false, cmd_lsof,     "lsof",                              "open editor/input state"},
{"seq",      false, cmd_seq,      "seq [first] <last>",                "print number sequence"},
{"tee",      true,  cmd_tee,      "tee <file> <command...>",           "capture output to file"},
{"watch",    false, cmd_watch,    "watch <command...>",                "execute command (one-shot)"},
{"sudo",     true,  cmd_sudo,     "sudo <command...>",                 "bypass game-mode restriction"},
{"su",       true,  cmd_su,       "su",                                "toggle game/debug mode"},
{"dmesg",    false, cmd_dmesg,    "dmesg [-n count] [-l level]",       "engine event log"},
/* MPE_TASK_V15R2_PHASE7_TABLE_END */
/* MPE_TASK_V15R2_PHASE8_TABLE_BEGIN */
{"vi",       true,  cmd_vi,       "vi [file|--list|--help]",           "open microvim editor"},
{"vim",      true,  cmd_vi,       "vim [filename]",                    "open microvim editor"},
{"microvim", true,  cmd_vi,       "microvim [filename]",               "open microvim editor"},
/* MPE_TASK_V15R2_PHASE8_TABLE_END */
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
term_printf (NULL, "  gravity:            %.4f\n", g_cfg.world.gravity);
term_printf (NULL, "  drag:               %.4f\n", g_cfg.world.drag);
term_printf (NULL, "  floor_friction_s:   %.4f\n", g_cfg.world.floor_friction_s);
term_printf (NULL, "  floor_friction_k:   %.4f\n", g_cfg.world.floor_friction_k);
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
term_printf (NULL, "  jump:       %.4f\n", g_cfg.camera.jump_height);
}
static void term_print_spawner (void) {
term_printf ("term_echo", "/spawner\n");
term_printf (NULL, "  type:        %s\n", (main_inputs.current_spawn_type == 0) ? "sphere" : "cube");
term_printf (NULL, "  mass:        %.4f\n", g_cfg.spawner.mass);
term_printf (NULL, "  radius:      %.4f\n", g_cfg.spawner.radius);
term_printf (NULL, "  cube_mass:   %.4f\n", g_cfg.spawner.cube_mass);
term_printf (NULL, "  cube_extent: %.4f\n", g_cfg.spawner.cube_extent);
term_printf (NULL, "  speed:       %.4f\n", g_cfg.spawner.speed);
term_printf (NULL, "  friction_s:  %.4f\n", g_cfg.spawner.friction_s);
term_printf (NULL, "  friction_k:  %.4f\n", g_cfg.spawner.friction_k);
}
/* ------------------------------------------------------------------ */
/* Scene mutation helpers                                              */
/* ------------------------------------------------------------------ */
static int term_create_object (object_type spawn_type) {
int created_index = -1;
if (spawn_type == object_sphere) {
vector3 spawn_position = vector3_addition (
main_camera_fov.position,
vector3_scaling (main_camera_fov.forward_vector, g_cfg.spawner.radius + 1.0f)
);
created_index = scene_add_object (g_cfg.spawner.radius, g_cfg.spawner.mass, spawn_position);
} else {
vector3 spawn_position = vector3_addition (
main_camera_fov.position,
vector3_scaling (main_camera_fov.forward_vector, g_cfg.spawner.cube_extent + 1.0f)
);
created_index = scene_add_cube (
spawn_position,
(vector3) {g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent, g_cfg.spawner.cube_extent},
g_cfg.spawner.cube_mass
);
}
if (created_index < 0) {
term_err ("mpe: touch: cannot create object (scene full?)\n");
return -1;
}
obj_per_scene [created_index].friction_static = g_cfg.spawner.friction_s;
obj_per_scene [created_index].friction_kinetic = g_cfg.spawner.friction_k;
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
term_printf (NULL, "    %s - %s\n", terminal_commands [command_index].name, terminal_commands [command_index].description);
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
float spring_constant = soft_joint ? g_cfg.joints.soft_spring_k : g_cfg.joints.default_spring_k;
float damping_coefficient = soft_joint ? g_cfg.joints.soft_damping : g_cfg.joints.default_damping;
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
/* MPE_TASK_V15R2_UNAME_EXPANDED */
bool print_all = false, print_sys = false, print_rel = false;
bool print_mach = false, print_os = false;
for (int i = 1; i < argc; i++) {
if (term_str_eq (argv [i], "-a")) {print_all = true;}
else if (term_str_eq (argv [i], "-s")) {print_sys = true;}
else if (term_str_eq (argv [i], "-r")) {print_rel = true;}
else if (term_str_eq (argv [i], "-m")) {print_mach = true;}
else if (term_str_eq (argv [i], "-o")) {print_os = true;}
else if (argv [i][0] == '-') {print_all = true;}
}
if (print_all) {
term_printf (NULL, "MPE %s mpe-engine x86_64 POSIX-like/GTK3/OpenGL3.3 MPE\n", A3_VERSION_STRING);
} else if (print_sys) {
term_out ("MPE\n");
} else if (print_rel) {
term_printf (NULL, "%s\n", A3_VERSION_STRING);
} else if (print_mach) {
term_out ("x86_64\n");
} else if (print_os) {
term_out ("POSIX-like/GTK3/OpenGL3.3\n");
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
/* MPE_TASK_38_REGISTRY_ENV_BEGIN */
static void cmd_env (int argc, char **argv) {
(void) argc; (void) argv;
term_printf (NULL, "CAMERA_SPEED=%.4f\n", main_camera_fov.movement_speed);
int current_category = -1;
for (size_t i = 0; i < g_registry_count; i++) {
if ((int) g_registry [i].category != current_category) {
current_category = (int) g_registry [i].category;
term_printf ("term_echo", "[%s]\n", mpe_config_category_name ((param_category) current_category));
}
if (g_registry [i].type == P_INT) {
term_printf (NULL, "  %s = %d\n", g_registry [i].key, *(int *) g_registry [i].storage);
} else if (g_registry [i].type == P_BOOL) {
term_printf (NULL, "  %s = %s\n", g_registry [i].key, (*(bool *) g_registry [i].storage) ? "true" : "false");
} else {
term_printf (NULL, "  %s = %.4f\n", g_registry [i].key, *(float *) g_registry [i].storage);
}
}
}
/* MPE_TASK_38_REGISTRY_ENV_END */
/* MPE_TASK_38_REGISTRY_EXPORT_BEGIN */
static void cmd_export (int argc, char **argv) {
if (argc < 2) {
cmd_env (argc, argv);
return;
}
for (int argument_index = 1; argument_index < argc; argument_index++) {
char **parts = g_strsplit (argv [argument_index], "=", 2);
if ((!parts [0]) || (!parts [1])) {
term_printf ("term_err", "mpe: export: usage: export KEY=value\n");
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
if (term_str_eq (variable_name, "CAMERA_SPEED")) {
main_camera_fov.movement_speed = variable_value;
term_printf ("term_ok", "CAMERA_SPEED=%.4f\n", variable_value);
} else {
const mpe_param *param = mpe_config_find (variable_name);
if (param) {
bool clamped = !mpe_config_set_float (variable_name, variable_value);
if (param -> type == P_INT) {
term_printf ("term_ok", "%s = %d%s\n", variable_name, *(int *) param -> storage, clamped ? " (clamped)" : "");
} else if (param -> type == P_BOOL) {
term_printf ("term_ok", "%s = %s%s\n", variable_name, (*(bool *) param -> storage) ? "true" : "false", clamped ? " (clamped)" : "");
} else {
term_printf ("term_ok", "%s = %.4f%s\n", variable_name, *(float *) param -> storage, clamped ? " (clamped)" : "");
}
} else {
term_printf ("term_err", "mpe: export: %s: unknown key\n", variable_name);
}
}
g_strfreev (parts);
}
}
/* MPE_TASK_38_REGISTRY_EXPORT_END */
/* MPE_TASK_38_CONFIG_COMMAND_BEGIN */
static void cmd_config (int argc, char **argv) {
if (argc < 2) {
term_err ("usage: config save|load|reset\n");
return;
}
if (term_str_eq (argv [1], "save")) {
if (mpe_config_save ("status/engine.cfg")) {
term_ok ("config saved to status/engine.cfg\n");
} else {
term_err ("mpe: config: save failed\n");
}
} else if (term_str_eq (argv [1], "load")) {
if (mpe_config_load ("status/engine.cfg")) {
contact_cache_clear ();
term_ok ("config loaded from status/engine.cfg\n");
} else {
term_err ("mpe: config: load failed (file missing?)\n");
}
} else if (term_str_eq (argv [1], "reset")) {
mpe_config_reset_defaults ();
contact_cache_clear ();
term_ok ("config reset to defaults\n");
} else {
term_err ("mpe: config: unknown subcommand. Use save|load|reset\n");
}
}
/* MPE_TASK_38_CONFIG_COMMAND_END */
/* ------------------------------------------------------------------ */
/* Execution                                                           */
/* ------------------------------------------------------------------ */
/* MPE_TASK_V15R2_PHASE2_IMPL_BEGIN */
static void cmd_exit (int argc, char **argv) {
(void) argc; (void) argv;
term_dim ("logout\n");
if (terminal_window) {gtk_widget_destroy (terminal_window);}
}
static void cmd_logout (int argc, char **argv) {cmd_exit (argc, argv);}
static void cmd_quit (int argc, char **argv) {cmd_exit (argc, argv);}
static void cmd_poweroff (int argc, char **argv) {
(void) argc; (void) argv;
mpe_config_save ("status/engine.cfg");
term_ok ("System halted.\n");
event_log_push (LOG_INFO, "Engine shutdown via terminal poweroff");
gtk_main_quit ();
}
static void cmd_shutdown (int argc, char **argv) {cmd_poweroff (argc, argv);}
static void cmd_reboot (int argc, char **argv) {
(void) argc; (void) argv;
scene_clear ();
clear_selection ();
contact_cache_clear ();
editor_reset ();
scene_init_default ();
term_ok ("System rebooted.\n");
event_log_push (LOG_INFO, "Scene rebooted via terminal");
}
static void cmd_halt (int argc, char **argv) {
(void) argc; (void) argv;
if (physics_is_halted ()) {
physics_halt_set (false);
term_ok ("System resumed.\n");
} else {
physics_halt_set (true);
term_ok ("System halted. Type 'halt' again to resume.\n");
}
}
static void cmd_sleep (int argc, char **argv) {
if (argc < 2) {term_err ("usage: sleep <seconds>\n"); return;}
float seconds = 0.0f;
if ((!term_parse_float (argv [1], &seconds)) || (seconds <= 0.0f)) {
term_err ("mpe: sleep: invalid duration\n");
return;
}
int ticks = (int) (seconds * 60.0f);
physics_halt_for_ticks (ticks);
term_printf ("term_ok", "Sleeping for %.1f seconds (%d ticks)...\n", seconds, ticks);
}
static void cmd_sync (int argc, char **argv) {
(void) argc; (void) argv;
bool cfg_ok = mpe_config_save ("status/engine.cfg");
int scene_ok = save_scene ("status/scene.dat");
if (cfg_ok && scene_ok) {
term_ok ("sync: config + scene flushed to disk\n");
} else {
term_printf ("term_err", "mpe: sync: config=%s scene=%s\n",
cfg_ok ? "ok" : "FAILED", scene_ok ? "ok" : "FAILED");
}
}
static void cmd_uptime (int argc, char **argv) {
(void) argc; (void) argv;
static gint64 a3_term_start_time = 0;
if (a3_term_start_time == 0) {a3_term_start_time = g_get_monotonic_time ();}
gint64 now = g_get_monotonic_time ();
double elapsed = (double) (now - a3_term_start_time) / 1000000.0;
int hours = (int) (elapsed / 3600.0);
int minutes = (int) (fmod (elapsed, 3600.0) / 60.0);
int seconds = (int) fmod (elapsed, 60.0);
term_printf (NULL, " up %02d:%02d:%02d, %d objects, %d joints, sleeping=%d\n",
hours, minutes, seconds,
object_count, current_joint_count, debug_last_sleeping_object_count);
}
static void cmd_free (int argc, char **argv) {
(void) argc; (void) argv;
int obj_cap = (object_capacity > 0) ? object_capacity : MPE_MAX_BODIES;
term_printf (NULL, "             total      used      free  use%%\n");
term_printf (NULL, "objects:    %6d   %6d   %6d   %3d%%\n",
obj_cap, object_count, obj_cap - object_count,
(obj_cap > 0) ? (object_count * 100 / obj_cap) : 0);
term_printf (NULL, "joints:     %6d   %6d   %6d   %3d%%\n",
MPE_MAX_JOINTS, current_joint_count, MPE_MAX_JOINTS - current_joint_count,
(MPE_MAX_JOINTS > 0) ? (current_joint_count * 100 / MPE_MAX_JOINTS) : 0);
term_printf (NULL, "bp pairs:   %6d   %6d\n",
MPE_MAX_BROADPHASE_PAIRS, debug_last_broadphase_pair_count);
term_printf (NULL, "manifolds:  %6d   %6d\n",
A3_MAX_MANIFOLDS, debug_last_manifold_count);
term_printf (NULL, "cache:      hits=%d misses=%d\n",
contact_cache_get_hits (), contact_cache_get_misses ());
}
static void cmd_w (int argc, char **argv) {
(void) argc; (void) argv;
term_printf ("term_echo", "camera0:  flags=<%s>\n",
main_inputs.is_mouse_locked ? "LOCKED" : "FREE");
term_printf (NULL, "  position  (%.2f, %.2f, %.2f)\n",
main_camera_fov.position.x, main_camera_fov.position.y, main_camera_fov.position.z);
term_printf (NULL, "  yaw=%.2f  pitch=%.2f  speed=%.2f m/s\n",
main_camera_fov.yaw, main_camera_fov.pitch, main_camera_fov.movement_speed);
term_printf (NULL, "  mode=%s  spawn=%s  sel=%d\n",
main_inputs.is_debug_mode_active ? "DEBUG" : "GAME",
(main_inputs.current_spawn_type == 0) ? "sphere" : "cube",
selected_object);
}
static void cmd_hostname (int argc, char **argv) {
if ((argc > 1) && (term_str_eq (argv [1], "-f"))) {
term_out ("mpe-engine.local\n");
} else {
term_out ("mpe-engine\n");
}
}
static void cmd_id (int argc, char **argv) {
(void) argc; (void) argv;
if (main_inputs.is_debug_mode_active) {
term_out ("uid=0(root) gid=0(root) groups=0(root),1(debug)\n");
} else {
term_out ("uid=1000(observer) gid=1000(observer) groups=1000(observer)\n");
}
}
static void cmd_which (int argc, char **argv) {
if (argc < 2) {term_err ("usage: which <command>\n"); return;}
for (size_t i = 0; i < TERMINAL_COMMAND_COUNT; i++) {
if (term_str_eq (argv [1], terminal_commands [i].name)) {
term_printf (NULL, "%s: shell builtin\n", terminal_commands [i].name);
return;
}
}
term_printf ("term_err", "mpe: which: no %s in (/usr/bin)\n", argv [1]);
}
static void cmd_true (int argc, char **argv) {(void) argc; (void) argv;}
static void cmd_false (int argc, char **argv) {(void) argc; (void) argv;}
static void cmd_time (int argc, char **argv) {
if (argc < 2) {term_err ("usage: time <command...>\n"); return;}
char cmd_buf [2048];
cmd_buf [0] = '\0';
size_t offset = 0;
for (int i = 1; i < argc; i++) {
if (i > 1) {cmd_buf [offset++] = ' ';}
size_t len = strlen (argv [i]);
if (offset + len < sizeof (cmd_buf) - 1) {
memcpy (cmd_buf + offset, argv [i], len);
offset += len;
cmd_buf [offset] = '\0';
}
}
gint64 start_time = g_get_monotonic_time ();
term_execute (cmd_buf);
gint64 end_time = g_get_monotonic_time ();
double elapsed = (double) (end_time - start_time) / 1000000.0;
term_printf ("term_dim", "\nreal\t%dm%.3fs\n", (int) (elapsed / 60.0), fmod (elapsed, 60.0));
}
/* MPE_TASK_V15R2_PHASE2_IMPL_END */
/* MPE_TASK_V15R2_PHASE3_IMPL_BEGIN */
static void cmd_stat (int argc, char **argv) {
if (argc < 2) {
term_err ("usage: stat <path>\n");
return;
}
const char *target = argv [1];
if (strstr (target, "world")) {
term_printf ("term_echo", "  File: /world\n");
term_printf (NULL, "  Size: %zu params    Blocks: 13    IO Block: config\n", g_registry_count);
term_printf (NULL, "  Mode: (0644/-rw-r--r--)  Uid: 0  Gid: 0\n");
term_printf (NULL, "  Gravity: %.4f  Drag: %.4f\n", g_cfg.world.gravity, g_cfg.world.drag);
term_printf (NULL, "  Objects: %d  Joints: %d  Mode: %s\n",
object_count, current_joint_count,
main_inputs.is_debug_mode_active ? "debug" : "game");
return;
}
if (strstr (target, "camera")) {
term_printf ("term_echo", "  File: /camera\n");
term_printf (NULL, "  Position: (%.4f, %.4f, %.4f)\n",
main_camera_fov.position.x, main_camera_fov.position.y, main_camera_fov.position.z);
term_printf (NULL, "  Yaw: %.4f  Pitch: %.4f  Speed: %.4f\n",
main_camera_fov.yaw, main_camera_fov.pitch, main_camera_fov.movement_speed);
term_printf (NULL, "  Mouse: %s  Mode: %s\n",
main_inputs.is_mouse_locked ? "locked" : "free",
main_inputs.is_debug_mode_active ? "debug" : "game");
return;
}
if (strstr (target, "spawner")) {
term_printf ("term_echo", "  File: /spawner\n");
term_printf (NULL, "  Type: %s  Mass: %.4f  Radius: %.4f\n",
(main_inputs.current_spawn_type == 0) ? "sphere" : "cube",
g_cfg.spawner.mass, g_cfg.spawner.radius);
term_printf (NULL, "  Speed: %.4f  Friction: s=%.3f k=%.3f\n",
g_cfg.spawner.speed, g_cfg.spawner.friction_s, g_cfg.spawner.friction_k);
return;
}
int object_index = term_object_from_token (target);
if (object_index < 0) {
term_printf ("term_err", "mpe: stat: %s: No such object\n", target);
return;
}
rigidbody *rb = &obj_per_scene [object_index];
int joint_count_for_obj = 0;
for (int ji = 0; ji < MPE_MAX_JOINTS; ji++) {
if (!joint_pool [ji].is_active) {continue;}
int ia = scene_find_object_index_by_id (joint_pool [ji].object_id_a);
int ib = scene_find_object_index_by_id (joint_pool [ji].object_id_b);
if ((ia == object_index) || (ib == object_index)) {joint_count_for_obj++;}
}
term_printf ("term_echo", "  File: /obj/%d\n", object_index);
term_printf (NULL, "  Size: %.4f kg    Links: %d    Inode: %u\n",
rb -> mass, joint_count_for_obj, rb -> object_id);
term_printf (NULL, "  Access: %s/%s  Mode: %s\n",
rb -> static_state ? "static" : "dynamic",
rb -> is_sleeping ? "sleeping" : "awake",
term_object_mode (rb));
term_printf (NULL, "  Type: %s\n", term_object_type_name (rb));
if (rb -> type == object_sphere) {
term_printf (NULL, "  Radius: %.4f\n", rb -> radius);
} else {
term_printf (NULL, "  HalfExt: (%.4f, %.4f, %.4f)\n",
rb -> half_extensions.x, rb -> half_extensions.y, rb -> half_extensions.z);
}
term_printf (NULL, "  Position: (%.4f, %.4f, %.4f)\n",
rb -> position.x, rb -> position.y, rb -> position.z);
term_printf (NULL, "  Velocity: (%.4f, %.4f, %.4f)  |v|=%.4f\n",
rb -> velocity.x, rb -> velocity.y, rb -> velocity.z,
vector3_length (rb -> velocity));
term_printf (NULL, "  AngVel: (%.4f, %.4f, %.4f)  |w|=%.4f\n",
rb -> angular_velocity.x, rb -> angular_velocity.y, rb -> angular_velocity.z,
vector3_length (rb -> angular_velocity));
term_printf (NULL, "  Orient: (%.4f, %.4f, %.4f, %.4f)\n",
rb -> orientation.w, rb -> orientation.x, rb -> orientation.y, rb -> orientation.z);
term_printf (NULL, "  Friction: s=%.3f k=%.3f  Restitution: %.3f\n",
rb -> friction_static, rb -> friction_kinetic, rb -> restitution);
term_printf (NULL, "  Colour: (%.2f, %.2f, %.2f)\n",
rb -> colour.x, rb -> colour.y, rb -> colour.z);
term_printf (NULL, "  SleepTimer: %.2f  Nice: %d\n",
rb -> sleep_timer, rb -> nice_value);
}
static void cmd_find (int argc, char **argv) {
if (argc < 2) {
term_err ("usage: find /obj [-type t] [-mass v] [-sleeping] [-awake] [-static] [-dynamic]\n");
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
if (term_str_eq (argv [i], "-type") && (i + 1 < argc)) {
i++;
if (term_str_eq (argv [i], "sphere") || term_str_eq (argv [i], "sph")) {filter_type = 0;}
else if (term_str_eq (argv [i], "cube")) {filter_type = 1;}
} else if (term_str_eq (argv [i], "-mass") && (i + 1 < argc)) {
i++;
const char *val = argv [i];
if (val [0] == '+') {term_parse_float (val + 1, &mass_greater);}
else if (val [0] == '-') {term_parse_float (val + 1, &mass_less);}
else {term_parse_float (val, &mass_exact);}
} else if (term_str_eq (argv [i], "-sleeping")) {filter_sleeping = true;}
else if (term_str_eq (argv [i], "-awake")) {filter_awake = true;}
else if (term_str_eq (argv [i], "-static")) {filter_static = true;}
else if (term_str_eq (argv [i], "-dynamic")) {filter_dynamic = true;}
}
int match_count = 0;
for (int object_index = 0; object_index < object_count; object_index++) {
rigidbody *rb = &obj_per_scene [object_index];
if ((filter_type == 0) && (rb -> type != object_sphere)) {continue;}
if ((filter_type == 1) && (rb -> type != object_cube)) {continue;}
if ((mass_exact >= 0.0f) && (fabsf (rb -> mass - mass_exact) > 0.001f)) {continue;}
if ((mass_greater >= 0.0f) && (rb -> mass <= mass_greater)) {continue;}
if ((mass_less >= 0.0f) && (rb -> mass >= mass_less)) {continue;}
if (filter_sleeping && (!rb -> is_sleeping)) {continue;}
if (filter_awake && (rb -> is_sleeping)) {continue;}
if (filter_static && (!rb -> static_state)) {continue;}
if (filter_dynamic && (rb -> static_state)) {continue;}
term_printf (NULL, "/obj/%d\n", object_index);
match_count++;
}
if (match_count == 0) {term_dim ("(no matches)\n");}
else {term_printf ("term_dim", "%d match(es)\n", match_count);}
}
static void cmd_wc (int argc, char **argv) {
if (argc < 2) {
term_printf (NULL, "%d objects, %d joints\n", object_count, current_joint_count);
return;
}
for (int i = 1; i < argc; i++) {
if (argv [i][0] == '-') {continue;}
if (strstr (argv [i], "joint")) {
int active_joints = 0;
for (int ji = 0; ji < MPE_MAX_JOINTS; ji++) {
if (joint_pool [ji].is_active) {active_joints++;}
}
term_printf (NULL, "%d /joint\n", active_joints);
} else if (strstr (argv [i], "obj")) {
term_printf (NULL, "%d /obj\n", object_count);
} else {
term_printf (NULL, "%d objects, %d joints\n", object_count, current_joint_count);
}
}
}
static void cmd_file (int argc, char **argv) {
if (argc < 2) {
term_err ("usage: file <path>\n");
return;
}
const char *target = argv [1];
if (strstr (target, "world")) {
term_printf (NULL, "/world: physics configuration, %zu parameters\n", g_registry_count);
return;
}
if (strstr (target, "camera")) {
term_printf (NULL, "/camera: viewport state, %s mode\n",
main_inputs.is_debug_mode_active ? "debug" : "game");
return;
}
if (strstr (target, "spawner")) {
term_printf (NULL, "/spawner: object factory, type=%s\n",
(main_inputs.current_spawn_type == 0) ? "sphere" : "cube");
return;
}
if (term_classify_token (target) == TERM_TARGET_JOINT) {
int joint_index = term_joint_from_token (target);
if (joint_index >= 0) {
spring_joint *j = &joint_pool [joint_index];
term_printf (NULL, "/joint/%d: spring joint, k=%.1f d=%.1f len=%.2f\n",
joint_index, j -> spring_constant, j -> damping_coefficient, j -> equilibrium_length);
} else {
term_printf ("term_err", "mpe: file: %s: No such joint\n", target);
}
return;
}
int object_index = term_object_from_token (target);
if (object_index < 0) {
term_printf ("term_err", "mpe: file: %s: No such object\n", target);
return;
}
rigidbody *rb = &obj_per_scene [object_index];
term_printf (NULL, "/obj/%d: rigid body, %s, %.2f kg, %s%s\n",
object_index,
term_object_type_name (rb),
rb -> mass,
rb -> static_state ? "static" : "dynamic",
rb -> is_sleeping ? ", sleeping" : "");
}
static void cmd_diff (int argc, char **argv) {
if (argc < 3) {
term_err ("usage: diff <object_a> <object_b>\n");
return;
}
int index_a = term_require_object (argv [1]);
if (index_a < 0) {return;}
int index_b = term_require_object (argv [2]);
if (index_b < 0) {return;}
if (index_a == index_b) {
term_ok ("Objects are identical (same object)\n");
return;
}
rigidbody *a = &obj_per_scene [index_a];
rigidbody *b = &obj_per_scene [index_b];
int diff_count = 0;
term_printf ("term_echo", "--- /obj/%d\n", index_a);
term_printf ("term_echo", "+++ /obj/%d\n", index_b);
if (a -> type != b -> type) {
term_printf ("term_err", "  type:       %s -> %s\n", term_object_type_name (a), term_object_type_name (b));
diff_count++;
}
if (fabsf (a -> mass - b -> mass) > 0.001f) {
term_printf ("term_err", "  mass:       %.4f -> %.4f\n", a -> mass, b -> mass);
diff_count++;
}
if ((a -> type == object_sphere) && (b -> type == object_sphere)) {
if (fabsf (a -> radius - b -> radius) > 0.001f) {
term_printf ("term_err", "  radius:     %.4f -> %.4f\n", a -> radius, b -> radius);
diff_count++;
}
}
if (vector3_length_squared (vector3_subtraction (a -> position, b -> position)) > 0.001f) {
term_printf (NULL, "  position:   (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)\n",
a -> position.x, a -> position.y, a -> position.z,
b -> position.x, b -> position.y, b -> position.z);
diff_count++;
}
if (vector3_length_squared (vector3_subtraction (a -> velocity, b -> velocity)) > 0.001f) {
term_printf (NULL, "  velocity:   |%.3f| -> |%.3f|\n",
vector3_length (a -> velocity), vector3_length (b -> velocity));
diff_count++;
}
if (a -> static_state != b -> static_state) {
term_printf ("term_err", "  static:     %s -> %s\n",
a -> static_state ? "yes" : "no", b -> static_state ? "yes" : "no");
diff_count++;
}
if (a -> is_sleeping != b -> is_sleeping) {
term_printf (NULL, "  sleeping:   %s -> %s\n",
a -> is_sleeping ? "yes" : "no", b -> is_sleeping ? "yes" : "no");
diff_count++;
}
if ((fabsf (a -> friction_static - b -> friction_static) > 0.001f) ||
(fabsf (a -> friction_kinetic - b -> friction_kinetic) > 0.001f)) {
term_printf (NULL, "  friction:   s=%.3f k=%.3f -> s=%.3f k=%.3f\n",
a -> friction_static, a -> friction_kinetic,
b -> friction_static, b -> friction_kinetic);
diff_count++;
}
if (fabsf (a -> restitution - b -> restitution) > 0.001f) {
term_printf (NULL, "  restitution:%.3f -> %.3f\n", a -> restitution, b -> restitution);
diff_count++;
}
if (a -> nice_value != b -> nice_value) {
term_printf (NULL, "  nice:       %d -> %d\n", a -> nice_value, b -> nice_value);
diff_count++;
}
if (diff_count == 0) {
term_ok ("Objects are identical\n");
} else {
term_printf ("term_dim", "%d difference(s)\n", diff_count);
}
}
static void cmd_xxd (int argc, char **argv) {
if (argc < 2) {
term_err ("usage: xxd <object> [-l len] [-s offset]\n");
return;
}
int object_index = term_require_object (argv [1]);
if (object_index < 0) {return;}
int dump_length = (int) sizeof (rigidbody);
int dump_offset = 0;
for (int i = 2; i < argc; i++) {
if (term_str_eq (argv [i], "-l") && (i + 1 < argc)) {
float v = 0.0f;
if (term_parse_float (argv [i + 1], &v)) {dump_length = (int) v;}
i++;
} else if (term_str_eq (argv [i], "-s") && (i + 1 < argc)) {
float v = 0.0f;
if (term_parse_float (argv [i + 1], &v)) {dump_offset = (int) v;}
i++;
}
}
int struct_size = (int) sizeof (rigidbody);
if (dump_offset >= struct_size) {
term_err ("mpe: xxd: offset beyond struct size\n");
return;
}
if (dump_offset + dump_length > struct_size) {dump_length = struct_size - dump_offset;}
if (dump_length <= 0) {dump_length = struct_size - dump_offset;}
const unsigned char *raw = (const unsigned char *) &obj_per_scene [object_index];
term_printf ("term_echo", "xxd /obj/%d  (%d bytes at offset %d of %d)\n",
object_index, dump_length, dump_offset, struct_size);
for (int row = 0; row < dump_length; row += 16) {
int row_len = dump_length - row;
if (row_len > 16) {row_len = 16;}
char hex_part [64];
char ascii_part [20];
int hex_offset = 0;
for (int col = 0; col < 16; col++) {
if (col < row_len) {
unsigned char byte = raw [dump_offset + row + col];
hex_offset += snprintf (hex_part + hex_offset, sizeof (hex_part) - hex_offset, "%02x ", byte);
ascii_part [col] = ((byte >= 32) && (byte < 127)) ? (char) byte : '.';
} else {
hex_offset += snprintf (hex_part + hex_offset, sizeof (hex_part) - hex_offset, "   ");
ascii_part [col] = ' ';
}
}
ascii_part [row_len] = '\0';
term_printf (NULL, "%08x: %-48s  |%s|\n", dump_offset + row, hex_part, ascii_part);
}
}
/* MPE_TASK_V15R2_PHASE3_IMPL_END */
/* MPE_TASK_V15R2_PHASE4_IMPL_BEGIN */
/* --- sort helpers --- */
static int a3_sort_key = 0; /* 0=index 1=mass 2=speed 3=type 4=pos.y */
static bool a3_sort_reverse = false;
static int a3_sort_compare (const void *pa, const void *pb) {
int ia = *(const int *) pa;
int ib = *(const int *) pb;
rigidbody *ra = &obj_per_scene [ia];
rigidbody *rb = &obj_per_scene [ib];
float va = 0.0f, vb = 0.0f;
switch (a3_sort_key) {
case 1: va = ra -> mass; vb = rb -> mass; break;
case 2: va = vector3_length (ra -> velocity); vb = vector3_length (rb -> velocity); break;
case 3: va = (float) ra -> type; vb = (float) rb -> type; break;
case 4: va = ra -> position.y; vb = rb -> position.y; break;
default: va = (float) ia; vb = (float) ib; break;
}
int result = (va < vb) ? -1 : ((va > vb) ? 1 : 0);
return a3_sort_reverse ? -result : result;
}
static void cmd_sort (int argc, char **argv) {
a3_sort_key = 0;
a3_sort_reverse = false;
bool list_joints = false;
for (int i = 1; i < argc; i++) {
if (term_str_eq (argv [i], "-k") && (i + 1 < argc)) {
i++;
if (term_str_eq (argv [i], "mass")) {a3_sort_key = 1;}
else if (term_str_eq (argv [i], "speed")) {a3_sort_key = 2;}
else if (term_str_eq (argv [i], "type")) {a3_sort_key = 3;}
else if (term_str_eq (argv [i], "pos.y")) {a3_sort_key = 4;}
} else if (term_str_eq (argv [i], "-r")) {a3_sort_reverse = true;}
else if (strstr (argv [i], "joint")) {list_joints = true;}
}
if (list_joints) {
term_list_joints (true);
return;
}
if (object_count == 0) {term_dim ("(no objects)\n"); return;}
static int sort_indices [MPE_MAX_BODIES];
for (int i = 0; i < object_count; i++) {sort_indices [i] = i;}
qsort (sort_indices, (size_t) object_count, sizeof (int), a3_sort_compare);
term_printf (NULL, "%-10s %4s %8s %-4s %-6s %s\n", "MODE", "PID", "MASS", "TYPE", "STATE", "INFO");
for (int i = 0; i < object_count; i++) {
term_print_object_long (sort_indices [i]);
}
}
static void cmd_grep (int argc, char **argv) {
if (argc < 2) {
term_err ("usage: grep <pattern> [path]\n");
return;
}
const char *pattern = argv [1];
term_capture_begin ();
if ((argc > 2) && (strstr (argv [2], "joint"))) {term_list_joints (true);}
else {term_list_objects (true);}
term_capture_end ();
char *captured = term_capture_get ();
if ((!captured) || (captured [0] == '\0')) {
term_capture_reset ();
term_dim ("(no output)\n");
return;
}
int match_count = 0;
char *line_start = captured;
char *newline_pos;
while ((newline_pos = strchr (line_start, '\n')) != NULL) {
*newline_pos = '\0';
if (g_ascii_strncasecmp (line_start, pattern, strlen (pattern)) == 0 ||
strstr (line_start, pattern) != NULL) {
term_printf (NULL, "%s\n", line_start);
match_count++;
}
line_start = newline_pos + 1;
}
if (line_start [0] != '\0') {
if (strstr (line_start, pattern) != NULL) {
term_printf (NULL, "%s\n", line_start);
match_count++;
}
}
term_capture_reset ();
if (match_count == 0) {term_dim ("(no matches)\n");}
else {term_printf ("term_dim", "%d match(es)\n", match_count);}
}
static void cmd_head (int argc, char **argv) {
int line_count = 10;
bool list_joints = false;
for (int i = 1; i < argc; i++) {
if (term_str_eq (argv [i], "-n") && (i + 1 < argc)) {
float v = 0.0f;
if (term_parse_float (argv [i + 1], &v)) {line_count = (int) v;}
i++;
} else if (strstr (argv [i], "joint")) {list_joints = true;}
}
if (line_count < 1) {line_count = 1;}
term_capture_begin ();
if (list_joints) {term_list_joints (true);}
else {term_list_objects (true);}
term_capture_end ();
char *captured = term_capture_get ();
if ((!captured) || (captured [0] == '\0')) {
term_capture_reset ();
term_dim ("(no output)\n");
return;
}
int printed = 0;
char *line_start = captured;
char *newline_pos;
while (((newline_pos = strchr (line_start, '\n')) != NULL) && (printed < line_count)) {
*newline_pos = '\0';
term_printf (NULL, "%s\n", line_start);
printed++;
line_start = newline_pos + 1;
}
if ((printed < line_count) && (line_start [0] != '\0')) {
term_printf (NULL, "%s\n", line_start);
}
term_capture_reset ();
}
static void cmd_tail (int argc, char **argv) {
int line_count = 10;
bool list_joints = false;
for (int i = 1; i < argc; i++) {
if (term_str_eq (argv [i], "-n") && (i + 1 < argc)) {
float v = 0.0f;
if (term_parse_float (argv [i + 1], &v)) {line_count = (int) v;}
i++;
} else if (strstr (argv [i], "joint")) {list_joints = true;}
}
if (line_count < 1) {line_count = 1;}
term_capture_begin ();
if (list_joints) {term_list_joints (true);}
else {term_list_objects (true);}
term_capture_end ();
char *captured = term_capture_get ();
if ((!captured) || (captured [0] == '\0')) {
term_capture_reset ();
term_dim ("(no output)\n");
return;
}
/* Count total lines */
int total_lines = 0;
char *scan = captured;
while (*scan) {if (*scan == '\n') {total_lines++;} scan++;}
if (captured [strlen (captured) - 1] != '\n') {total_lines++;}
/* Skip to the start of the last N lines */
int skip = total_lines - line_count;
if (skip < 0) {skip = 0;}
char *line_start = captured;
for (int i = 0; i < skip; i++) {
char *nl = strchr (line_start, '\n');
if (!nl) {break;}
line_start = nl + 1;
}
/* Print remaining */
char *newline_pos;
while ((newline_pos = strchr (line_start, '\n')) != NULL) {
*newline_pos = '\0';
term_printf (NULL, "%s\n", line_start);
line_start = newline_pos + 1;
}
if (line_start [0] != '\0') {term_printf (NULL, "%s\n", line_start);}
term_capture_reset ();
}
static void cmd_less (int argc, char **argv) {
int page_size = 40;
bool list_joints = false;
for (int i = 1; i < argc; i++) {
if (strstr (argv [i], "joint")) {list_joints = true;}
}
term_capture_begin ();
if (list_joints) {term_list_joints (true);}
else {term_list_objects (true);}
term_capture_end ();
char *captured = term_capture_get ();
if ((!captured) || (captured [0] == '\0')) {
term_capture_reset ();
term_dim ("(no output)\n");
return;
}
int total_lines = 0;
char *scan = captured;
while (*scan) {if (*scan == '\n') {total_lines++;} scan++;}
if (captured [strlen (captured) - 1] != '\n') {total_lines++;}
term_printf ("term_dim", "-- %d lines total, showing first %d --\n", total_lines, page_size);
int printed = 0;
char *line_start = captured;
char *newline_pos;
while (((newline_pos = strchr (line_start, '\n')) != NULL) && (printed < page_size)) {
*newline_pos = '\0';
term_printf (NULL, "%s\n", line_start);
printed++;
line_start = newline_pos + 1;
}
if ((printed < page_size) && (line_start [0] != '\0')) {
term_printf (NULL, "%s\n", line_start);
printed++;
}
if (total_lines > page_size) {
term_printf ("term_dim", "-- %d more lines (use head/tail/grep to filter) --\n", total_lines - printed);
}
term_capture_reset ();
}
/* MPE_TASK_V15R2_PHASE4_IMPL_END */
/* MPE_TASK_V15R2_PHASE5_IMPL_BEGIN */
static bool all_targets_matched_any (int argc, char **argv) {
for (int i = 2; i < argc; i++) {
if (term_is_all_token (argv [i])) {return true;}
}
return false;
}
static void cmd_sed (int argc, char **argv) {
if (argc < 3) {
term_err ("usage: sed s/field/value/ <target...>\n");
return;
}
const char *expression = argv [1];
if ((expression [0] != 's') || (expression [1] != '/')) {
term_err ("mpe: sed: expression must start with s/\n");
return;
}
const char *field_start = expression + 2;
const char *field_end = strchr (field_start, '/');
if (!field_end) {
term_err ("mpe: sed: malformed expression (missing second /)\n");
return;
}
char field_name [64];
int field_len = (int) (field_end - field_start);
if (field_len >= 64) {field_len = 63;}
strncpy (field_name, field_start, field_len);
field_name [field_len] = '\0';
const char *value_start = field_end + 1;
const char *value_end = strchr (value_start, '/');
char value_str [64];
if (value_end) {
int value_len = (int) (value_end - value_start);
if (value_len >= 64) {value_len = 63;}
strncpy (value_str, value_start, value_len);
value_str [value_len] = '\0';
} else {
strncpy (value_str, value_start, 63);
value_str [63] = '\0';
}
float new_value = 0.0f;
bool is_numeric = term_parse_float (value_str, &new_value);
bool make_static = term_str_eq (value_str, "static") || term_str_eq (value_str, "1");
bool make_dynamic = term_str_eq (value_str, "dynamic") || term_str_eq (value_str, "0");
int modified_count = 0;
for (int argument_index = 2; argument_index < argc; argument_index++) {
const char *target = argv [argument_index];
bool all_targets = term_is_all_token (target);
if (all_targets) {
for (int object_index = 0; object_index < object_count; object_index++) {
rigidbody *rb = &obj_per_scene [object_index];
if (term_str_eq (field_name, "mass")) {
if (!is_numeric) {continue;}
term_set_object_mass (object_index, new_value);
} else if (term_str_eq (field_name, "radius")) {
if ((!is_numeric) || (rb -> type != object_sphere)) {continue;}
rb -> radius = new_value;
rigidbody_update_inertia_sphere (rb);
rigidbody_wake (rb);
} else if (term_str_eq (field_name, "friction_static") || term_str_eq (field_name, "fs")) {
if (!is_numeric) {continue;}
rb -> friction_static = new_value;
} else if (term_str_eq (field_name, "friction_kinetic") || term_str_eq (field_name, "fk")) {
if (!is_numeric) {continue;}
rb -> friction_kinetic = new_value;
} else if (term_str_eq (field_name, "restitution") || term_str_eq (field_name, "rest")) {
if (!is_numeric) {continue;}
rb -> restitution = new_value;
if (rb -> restitution < 0.0f) {rb -> restitution = 0.0f;}
if (rb -> restitution > 1.0f) {rb -> restitution = 1.0f;}
} else if (term_str_eq (field_name, "static")) {
term_set_object_static (object_index, make_static || (!make_dynamic));
} else if (term_str_eq (field_name, "dynamic")) {
term_set_object_static (object_index, !make_dynamic);
} else if (term_str_eq (field_name, "nice")) {
if (!is_numeric) {continue;}
int nice_val = (int) new_value;
if (nice_val < -20) {nice_val = -20;}
if (nice_val > 19) {nice_val = 19;}
rb -> nice_value = nice_val;
}
modified_count++;
}
contact_cache_clear ();
continue;
}
int object_index = term_object_from_token (target);
if (object_index < 0) {
term_printf ("term_err", "mpe: sed: %s: No such object\n", target);
continue;
}
rigidbody *rb = &obj_per_scene [object_index];
if (term_str_eq (field_name, "mass")) {
if (!is_numeric) {term_err ("mpe: sed: mass requires numeric value\n"); continue;}
term_set_object_mass (object_index, new_value);
} else if (term_str_eq (field_name, "radius")) {
if (!is_numeric) {term_err ("mpe: sed: radius requires numeric value\n"); continue;}
if (rb -> type != object_sphere) {term_err ("mpe: sed: radius only applies to spheres\n"); continue;}
rb -> radius = new_value;
rigidbody_update_inertia_sphere (rb);
rigidbody_wake (rb);
} else if (term_str_eq (field_name, "friction_static") || term_str_eq (field_name, "fs")) {
if (!is_numeric) {term_err ("mpe: sed: friction requires numeric value\n"); continue;}
rb -> friction_static = new_value;
} else if (term_str_eq (field_name, "friction_kinetic") || term_str_eq (field_name, "fk")) {
if (!is_numeric) {term_err ("mpe: sed: friction requires numeric value\n"); continue;}
rb -> friction_kinetic = new_value;
} else if (term_str_eq (field_name, "restitution") || term_str_eq (field_name, "rest")) {
if (!is_numeric) {term_err ("mpe: sed: restitution requires numeric value\n"); continue;}
rb -> restitution = new_value;
if (rb -> restitution < 0.0f) {rb -> restitution = 0.0f;}
if (rb -> restitution > 1.0f) {rb -> restitution = 1.0f;}
} else if (term_str_eq (field_name, "static")) {
term_set_object_static (object_index, true);
} else if (term_str_eq (field_name, "dynamic")) {
term_set_object_static (object_index, false);
} else if (term_str_eq (field_name, "nice")) {
if (!is_numeric) {term_err ("mpe: sed: nice requires numeric value\n"); continue;}
int nice_val = (int) new_value;
if (nice_val < -20) {nice_val = -20;}
if (nice_val > 19) {nice_val = 19;}
rb -> nice_value = nice_val;
} else {
term_printf ("term_err", "mpe: sed: unknown field '%s'\n", field_name);
continue;
}
modified_count++;
contact_cache_clear ();
term_printf ("term_ok", "/obj/%d: %s=%s\n", object_index, field_name, value_str);
}
if (all_targets_matched_any (argc, argv)) {
term_printf ("term_ok", "sed: modified %d object(s)\n", modified_count);
}
}

static void cmd_nice (int argc, char **argv) {
if (argc < 3) {
term_err ("usage: nice <priority> <object...>\n");
return;
}
float priority_float = 0.0f;
if (!term_parse_float (argv [1], &priority_float)) {
term_printf ("term_err", "mpe: nice: invalid priority '%s'\n", argv [1]);
return;
}
int priority = (int) priority_float;
if (priority < -20) {priority = -20;}
if (priority > 19) {priority = 19;}
for (int argument_index = 2; argument_index < argc; argument_index++) {
int object_index = term_require_object (argv [argument_index]);
if (object_index < 0) {continue;}
obj_per_scene [object_index].nice_value = priority;
rigidbody_wake (&obj_per_scene [object_index]);
term_printf ("term_ok", "/obj/%d nice=%d\n", object_index, priority);
}
}
static void cmd_renice (int argc, char **argv) {
cmd_nice (argc, argv);
}
static void cmd_ping (int argc, char **argv) {
int ping_count = 1;
int argument_index = 1;
if ((argc > 1) && (term_str_eq (argv [1], "-c")) && (argc > 2)) {
float v = 0.0f;
if (term_parse_float (argv [2], &v)) {ping_count = (int) v;}
if (ping_count < 1) {ping_count = 1;}
if (ping_count > 10) {ping_count = 10;}
argument_index = 3;
}
if (argument_index >= argc) {
term_err ("usage: ping [-c count] <object>\n");
return;
}
int object_index = term_require_object (argv [argument_index]);
if (object_index < 0) {return;}
rigidbody *rb = &obj_per_scene [object_index];
if (rb -> static_state) {
term_printf ("term_dim", "PING /obj/%d: no response (static)\n", object_index);
return;
}
if (rb -> is_sleeping) {
term_printf ("term_dim", "PING /obj/%d: no response (sleeping)\n", object_index);
return;
}
for (int ping_index = 0; ping_index < ping_count; ping_index++) {
vector3 micro_impulse = vector3_scaling (rb -> velocity, 0.0f);
micro_impulse = (vector3) {0.001f, 0.001f, 0.001f};
rb -> velocity = vector3_addition (rb -> velocity, micro_impulse);
}
float speed_delta = 0.001f * sqrtf (3.0f) * (float) ping_count;
term_printf ("term_ok", "PING /obj/%d: %d ping(s), velocity delta %.6f m/s, state=%s\n",
object_index, ping_count, speed_delta,
rb -> is_sleeping ? "sleep" : "run");
}
static void cmd_mount (int argc, char **argv) {
if (argc < 2) {
term_err ("usage: mount <path>\n");
return;
}
const char *scene_path = argv [1];
if (scene_loading (scene_path)) {
editor_reset ();
contact_cache_clear ();
term_printf ("term_ok", "mounted %s: %d objects loaded\n", scene_path, object_count);
event_log_push (LOG_INFO, "Scene mounted via terminal: %s", scene_path);
} else {
term_printf ("term_err", "mpe: mount: %s: failed to load\n", scene_path);
}
}
static void cmd_umount (int argc, char **argv) {
(void) argc; (void) argv;
int save_result = save_scene ("status/scene.dat");
if (save_result) {
term_ok ("umount: scene saved to status/scene.dat\n");
} else {
term_err ("mpe: umount: save failed\n");
}
scene_clear ();
clear_selection ();
contact_cache_clear ();
editor_reset ();
term_ok ("umount: scene cleared\n");
event_log_push (LOG_INFO, "Scene unmounted via terminal");
}
static void cmd_mkfs (int argc, char **argv) {
(void) argc; (void) argv;
scene_clear ();
clear_selection ();
contact_cache_clear ();
editor_reset ();
term_printf ("term_ok", "Scene formatted. 0 objects, 0 joints.\n");
event_log_push (LOG_INFO, "Scene formatted via terminal (mkfs)");
}
static void cmd_fsck (int argc, char **argv) {
bool auto_fix = false;
for (int i = 1; i < argc; i++) {
if (term_str_eq (argv [i], "-y")) {auto_fix = true;}
}
int error_count = 0;
int warning_count = 0;
term_printf ("term_echo", "fsck: checking %d objects...\n", object_count);
for (int object_index = 0; object_index < object_count; object_index++) {
rigidbody *rb = &obj_per_scene [object_index];
bool has_error = false;
if ((!isfinite (rb -> position.x)) || (!isfinite (rb -> position.y)) || (!isfinite (rb -> position.z))) {
term_printf ("term_err", "  /obj/%d: position NaN/Inf\n", object_index);
has_error = true;
}
if ((!isfinite (rb -> velocity.x)) || (!isfinite (rb -> velocity.y)) || (!isfinite (rb -> velocity.z))) {
term_printf ("term_err", "  /obj/%d: velocity NaN/Inf\n", object_index);
has_error = true;
}
if ((!isfinite (rb -> angular_velocity.x)) || (!isfinite (rb -> angular_velocity.y)) || (!isfinite (rb -> angular_velocity.z))) {
term_printf ("term_err", "  /obj/%d: angular_velocity NaN/Inf\n", object_index);
has_error = true;
}
if ((!isfinite (rb -> orientation.w)) || (!isfinite (rb -> orientation.x)) ||
(!isfinite (rb -> orientation.y)) || (!isfinite (rb -> orientation.z))) {
term_printf ("term_err", "  /obj/%d: orientation NaN/Inf\n", object_index);
has_error = true;
}
if ((!rb -> static_state) && ((rb -> mass <= 0.0f) || (!isfinite (rb -> mass)))) {
term_printf ("term_err", "  /obj/%d: invalid mass %.4f (dynamic)\n", object_index, rb -> mass);
has_error = true;
}
if ((!rb -> static_state) && (rb -> inverse_mass <= 0.0f)) {
term_printf ("term_dim", "  /obj/%d: warning: inverse_mass=%.4f (dynamic)\n", object_index, rb -> inverse_mass);
warning_count++;
}
float orient_len_sq = rb -> orientation.w * rb -> orientation.w +
rb -> orientation.x * rb -> orientation.x +
rb -> orientation.y * rb -> orientation.y +
rb -> orientation.z * rb -> orientation.z;
if ((orient_len_sq < 0.9f) || (orient_len_sq > 1.1f)) {
term_printf ("term_dim", "  /obj/%d: warning: orientation not normalized (|q|^2=%.4f)\n", object_index, orient_len_sq);
warning_count++;
}
if (has_error) {
error_count++;
if (auto_fix) {
rigidbody_sanitize (rb);
term_printf ("term_ok", "  /obj/%d: sanitized\n", object_index);
}
}
}
term_printf ("term_echo", "fsck: checking %d joint slots...\n", MPE_MAX_JOINTS);
for (int joint_index = 0; joint_index < MPE_MAX_JOINTS; joint_index++) {
if (!joint_pool [joint_index].is_active) {continue;}
spring_joint *j = &joint_pool [joint_index];
int index_a = scene_find_object_index_by_id (j -> object_id_a);
int index_b = scene_find_object_index_by_id (j -> object_id_b);
if (index_a < 0) {
term_printf ("term_err", "  /joint/%d: object_a (id=%u) not found\n", joint_index, j -> object_id_a);
error_count++;
if (auto_fix) {remove_joint (joint_index); term_printf ("term_ok", "  /joint/%d: removed\n", joint_index);}
continue;
}
if (index_b < 0) {
term_printf ("term_err", "  /joint/%d: object_b (id=%u) not found\n", joint_index, j -> object_id_b);
error_count++;
if (auto_fix) {remove_joint (joint_index); term_printf ("term_ok", "  /joint/%d: removed\n", joint_index);}
continue;
}
if ((j -> equilibrium_length < 0.0f) || (!isfinite (j -> equilibrium_length))) {
term_printf ("term_err", "  /joint/%d: invalid rest length %.4f\n", joint_index, j -> equilibrium_length);
error_count++;
}
if ((j -> spring_constant <= 0.0f) || (!isfinite (j -> spring_constant))) {
term_printf ("term_err", "  /joint/%d: invalid spring constant %.4f\n", joint_index, j -> spring_constant);
error_count++;
}
}
if (auto_fix) {contact_cache_clear ();}
if (error_count == 0) {
term_printf ("term_ok", "fsck: PASS — %d objects, %d joints, %d warning(s), 0 errors\n",
object_count, current_joint_count, warning_count);
} else {
term_printf ("term_err", "fsck: FAIL — %d error(s), %d warning(s)%s\n",
error_count, warning_count, auto_fix ? " (auto-fixed)" : " (run fsck -y to fix)");
}
event_log_push (error_count == 0 ? LOG_INFO : LOG_WARN,
"fsck: %d errors, %d warnings%s", error_count, warning_count, auto_fix ? " (fixed)" : "");
}
/* MPE_TASK_V15R2_PHASE5_IMPL_END */
/* MPE_TASK_V15R2_PHASE6_IMPL_BEGIN */
static void cmd_netstat (int argc, char **argv) {
bool show_all = false;
for (int i = 1; i < argc; i++) {
if (term_str_eq (argv [i], "-a")) {show_all = true;}
}
term_printf ("term_echo", "Active Joints (spring connections)\n");
term_printf (NULL, "Proto  Local        Foreign      State         K        D      Len\n");
int listed = 0;
for (int ji = 0; ji < MPE_MAX_JOINTS; ji++) {
if ((!joint_pool [ji].is_active) && (!show_all)) {continue;}
int ia = scene_find_object_index_by_id (joint_pool [ji].object_id_a);
int ib = scene_find_object_index_by_id (joint_pool [ji].object_id_b);
const char *state_text = joint_pool [ji].is_active ? "ESTABLISHED" : "CLOSED";
term_printf (NULL, "spring /obj/%-6d /obj/%-6d %-12s %7.1f %7.1f %7.2f\n",
ia, ib, state_text,
joint_pool [ji].spring_constant,
joint_pool [ji].damping_coefficient,
joint_pool [ji].equilibrium_length);
listed++;
}
if (listed == 0) {term_dim ("(no connections)\n");}
else {term_printf ("term_dim", "%d connection(s) active\n", listed);}
}
static void cmd_ifconfig (int argc, char **argv) {
(void) argc; (void) argv;
term_printf ("term_echo", "camera0:  flags=<%s>  mode %s\n",
main_inputs.is_mouse_locked ? "LOCKED" : "FREE",
main_inputs.is_debug_mode_active ? "DEBUG" : "GAME");
term_printf (NULL, "    position (%.2f, %.2f, %.2f)\n",
main_camera_fov.position.x, main_camera_fov.position.y, main_camera_fov.position.z);
term_printf (NULL, "    yaw %.2f  pitch %.2f  speed %.2f m/s\n",
main_camera_fov.yaw, main_camera_fov.pitch, main_camera_fov.movement_speed);
term_printf (NULL, "    sensitivity %.3f  steer %.3f\n",
g_cfg.camera.mouse_sensitivity, g_cfg.camera.steer_sensitivity);
term_out ("\n");
term_printf ("term_echo", "render0:  flags=<ACTIVE>\n");
term_printf (NULL, "    light (%.1f, %.1f, %.1f)\n",
g_cfg.render.light_x, g_cfg.render.light_y, g_cfg.render.light_z);
term_printf (NULL, "    ambient %.2f  specular %.2f  exponent %.1f\n",
g_cfg.render.ambient_strength, g_cfg.render.specular_coeff, g_cfg.render.specular_exponent);
term_out ("\n");
term_printf ("term_echo", "input0:  flags=<%s>\n",
main_inputs.is_mouse_locked ? "GRABBED" : "RELEASED");
term_printf (NULL, "    spawn=%s  selected=%d  marked_joint=%d\n",
(main_inputs.current_spawn_type == 0) ? "sphere" : "cube",
selected_object, main_inputs.marked_joint_object_index);
term_printf (NULL, "    objects=%d  joints=%d  sleeping=%d\n",
object_count, current_joint_count, debug_last_sleeping_object_count);
}
static void cmd_lsmod (int argc, char **argv) {
(void) argc; (void) argv;
term_printf (NULL, "%-24s %6s  %s\n", "Module", "Size", "Used by");
term_printf (NULL, "%-24s %6s  %s\n", "instanced_shader", "1", "sphere_mesh, cube_mesh");
term_printf (NULL, "%-24s %6s  %s\n", "utility_shader", "1", "grid, wireframe, joints");
term_printf (NULL, "%-24s %6s  %s\n", "sphere_mesh", "1", "instanced_shader");
term_printf (NULL, "%-24s %6s  %s\n", "cube_mesh", "1", "instanced_shader");
term_printf (NULL, "%-24s %6s  %s\n", "grid_mesh", "1", "utility_shader");
term_printf (NULL, "%-24s %6s  %s\n", "wireframe_renderer", "1", "utility_shader");
term_printf (NULL, "%-24s %6s  %s\n", "joint_renderer", "1", "utility_shader");
term_printf (NULL, "%-24s %6s  %s\n", "spatial_hash_bp", "1", "collision_pipeline");
term_printf (NULL, "%-24s %6s  %s\n", "contact_cache", "1", "impulse_solver");
term_printf (NULL, "%-24s %6s  %s\n", "sequential_solver", "1", "physics_step");
term_printf (NULL, "%-24s %6s  %s\n", "depenetration_pass", "1", "physics_step");
term_printf (NULL, "%-24s %6s  %s\n", "sleep_system", "1", "physics_step, broadphase");
term_printf (NULL, "%-24s %6s  %s\n", "config_registry", "1", "config_menu, terminal, F9");
term_printf (NULL, "%-24s %6s  %s\n", "event_log", "1", "dmesg (pending)");
term_printf (NULL, "%-24s %6s  %s\n", "debug_terminal", "1", "input_control");
}
/* MPE_TASK_V15R2_PHASE6_IMPL_END */
/* MPE_TASK_V15R2_PHASE7_IMPL_BEGIN */
static void cmd_alias (int argc, char **argv) {
if (argc < 2) {
if (term_alias_count == 0) {term_dim ("(no aliases defined)\n"); return;}
for (int i = 0; i < term_alias_count; i++) {
term_printf (NULL, "alias %s='%s'\n", term_alias_names [i], term_alias_values [i]);
}
return;
}
for (int argument_index = 1; argument_index < argc; argument_index++) {
char *equals_pos = strchr (argv [argument_index], '=');
if (!equals_pos) {
bool found = false;
for (int i = 0; i < term_alias_count; i++) {
if (term_str_eq (argv [argument_index], term_alias_names [i])) {
term_printf (NULL, "alias %s='%s'\n", term_alias_names [i], term_alias_values [i]);
found = true;
break;
}
}
if (!found) {term_printf ("term_err", "mpe: alias: %s: not found\n", argv [argument_index]);}
continue;
}
*equals_pos = '\0';
const char *alias_name = argv [argument_index];
const char *alias_value = equals_pos + 1;
bool updated = false;
for (int i = 0; i < term_alias_count; i++) {
if (term_str_eq (alias_name, term_alias_names [i])) {
strncpy (term_alias_values [i], alias_value, TERM_ALIAS_VALUE_LEN - 1);
term_alias_values [i][TERM_ALIAS_VALUE_LEN - 1] = '\0';
updated = true;
break;
}
}
if (!updated) {
if (term_alias_count >= TERM_ALIAS_MAX) {
term_err ("mpe: alias: alias table full\n");
continue;
}
strncpy (term_alias_names [term_alias_count], alias_name, TERM_ALIAS_NAME_LEN - 1);
term_alias_names [term_alias_count][TERM_ALIAS_NAME_LEN - 1] = '\0';
strncpy (term_alias_values [term_alias_count], alias_value, TERM_ALIAS_VALUE_LEN - 1);
term_alias_values [term_alias_count][TERM_ALIAS_VALUE_LEN - 1] = '\0';
term_alias_count++;
}
term_printf ("term_ok", "alias %s='%s'\n", alias_name, alias_value);
}
}
static void cmd_unalias (int argc, char **argv) {
if (argc < 2) {term_err ("usage: unalias <name>\n"); return;}
for (int argument_index = 1; argument_index < argc; argument_index++) {
bool found = false;
for (int i = 0; i < term_alias_count; i++) {
if (term_str_eq (argv [argument_index], term_alias_names [i])) {
for (int j = i; j < term_alias_count - 1; j++) {
strncpy (term_alias_names [j], term_alias_names [j + 1], TERM_ALIAS_NAME_LEN);
strncpy (term_alias_values [j], term_alias_values [j + 1], TERM_ALIAS_VALUE_LEN);
}
term_alias_count--;
found = true;
term_printf ("term_ok", "unalias %s\n", argv [argument_index]);
break;
}
}
if (!found) {term_printf ("term_err", "mpe: unalias: %s: not found\n", argv [argument_index]);}
}
}
static void cmd_jobs (int argc, char **argv) {
(void) argc; (void) argv;
int job_count = 0;
if (long_run_validation_active) {
int seconds_remaining = long_run_validation_ticks_remaining / 60;
term_printf (NULL, "[%d]+ Running    long-run validation (%ds remaining / %d total)\n",
++job_count, seconds_remaining, long_run_validation_total_ticks / 60);
}
if (!long_run_validation_active) {
term_dim ("(no active jobs)\n");
}
}
static void cmd_lsof (int argc, char **argv) {
(void) argc; (void) argv;
term_printf ("term_echo", "COMMAND     TYPE     NAME\n");
term_printf (NULL, "%-11s %-8s %s\n", "terminal", "win", debug_terminal_is_open () ? "open" : "closed");
term_printf (NULL, "%-11s %-8s %s\n", "mouse", "lock", main_inputs.is_mouse_locked ? "grabbed" : "released");
term_printf (NULL, "%-11s %-8s %s\n", "mode", "state", main_inputs.is_debug_mode_active ? "debug" : "game");
if ((selected_object >= 0) && (selected_object < object_count)) {
term_printf (NULL, "%-11s %-8s /obj/%d\n", "selection", "obj", selected_object);
} else {
term_printf (NULL, "%-11s %-8s %s\n", "selection", "obj", "(none)");
}
if (main_inputs.marked_joint_object_index >= 0) {
term_printf (NULL, "%-11s %-8s /obj/%d\n", "joint_mark", "obj", main_inputs.marked_joint_object_index);
}
if (main_inputs.is_menu_open) {term_printf (NULL, "%-11s %-8s scene_menu\n", "menu", "open");}
if (main_inputs.spawner_menu_level > 0) {term_printf (NULL, "%-11s %-8s spawner_menu (level %d)\n", "menu", "open", main_inputs.spawner_menu_level);}
if (main_inputs.velocity_menu_level > 0) {term_printf (NULL, "%-11s %-8s velocity_menu (level %d)\n", "menu", "open", main_inputs.velocity_menu_level);}
if (main_inputs.object_menu_level > 0) {term_printf (NULL, "%-11s %-8s object_menu (level %d)\n", "menu", "open", main_inputs.object_menu_level);}
if (config_menu_is_open ()) {term_printf (NULL, "%-11s %-8s config_menu\n", "menu", "open");}
if (physics_is_halted ()) {term_printf (NULL, "%-11s %-8s HALTED\n", "physics", "state");}
}
static void cmd_seq (int argc, char **argv) {
int first = 1, last = 1;
if (argc == 2) {
float v = 0.0f;
if (term_parse_float (argv [1], &v)) {last = (int) v;}
} else if (argc >= 3) {
float v1 = 0.0f, v2 = 0.0f;
if (term_parse_float (argv [1], &v1)) {first = (int) v1;}
if (term_parse_float (argv [2], &v2)) {last = (int) v2;}
}
if (last < first) {int temp = first; first = last; last = temp;}
int limit = last - first + 1;
if (limit > 1000) {limit = 1000; last = first + 999;}
for (int i = first; i <= last; i++) {
term_printf (NULL, "%d\n", i);
}
}
static void cmd_tee (int argc, char **argv) {
if (argc < 3) {term_err ("usage: tee <filename> <command...>\n"); return;}
const char *output_filename = argv [1];
char sub_command [2048];
sub_command [0] = '\0';
size_t offset = 0;
for (int i = 2; i < argc; i++) {
if (i > 2) {sub_command [offset++] = ' ';}
size_t len = strlen (argv [i]);
if (offset + len < sizeof (sub_command) - 1) {
memcpy (sub_command + offset, argv [i], len);
offset += len;
sub_command [offset] = '\0';
}
}
term_capture_begin ();
term_execute (sub_command);
term_capture_end ();
char *captured = term_capture_get ();
if (captured && captured [0] != '\0') {
FILE *output_file = fopen (output_filename, "w");
if (output_file) {
fputs (captured, output_file);
fclose (output_file);
term_printf ("term_ok", "tee: wrote %zu bytes to %s\n", strlen (captured), output_filename);
} else {
term_printf ("term_err", "mpe: tee: %s: cannot open for writing\n", output_filename);
}
}
term_capture_reset ();
}
static void cmd_watch (int argc, char **argv) {
if (argc < 2) {term_err ("usage: watch <command...>\n"); return;}
char sub_command [2048];
sub_command [0] = '\0';
size_t offset = 0;
for (int i = 1; i < argc; i++) {
if (i > 1) {sub_command [offset++] = ' ';}
size_t len = strlen (argv [i]);
if (offset + len < sizeof (sub_command) - 1) {
memcpy (sub_command + offset, argv [i], len);
offset += len;
sub_command [offset] = '\0';
}
}
term_dim ("-- watch: single execution (periodic mode deferred) --\n");
term_execute (sub_command);
}
static void cmd_sudo (int argc, char **argv) {
if (argc < 2) {term_err ("usage: sudo <command...>\n"); return;}
if (main_inputs.is_debug_mode_active) {
char sub_command [2048];
sub_command [0] = '\0';
size_t offset = 0;
for (int i = 1; i < argc; i++) {
if (i > 1) {sub_command [offset++] = ' ';}
size_t len = strlen (argv [i]);
if (offset + len < sizeof (sub_command) - 1) {
memcpy (sub_command + offset, argv [i], len);
offset += len;
sub_command [offset] = '\0';
}
}
term_execute (sub_command);
return;
}
term_dim ("[sudo] bypassing game-mode restriction\n");
char sub_command [2048];
sub_command [0] = '\0';
size_t offset = 0;
for (int i = 1; i < argc; i++) {
if (i > 1) {sub_command [offset++] = ' ';}
size_t len = strlen (argv [i]);
if (offset + len < sizeof (sub_command) - 1) {
memcpy (sub_command + offset, argv [i], len);
offset += len;
sub_command [offset] = '\0';
}
}
term_sudo_active = true;
term_execute (sub_command);
term_sudo_active = false;
}
static void cmd_su (int argc, char **argv) {
(void) argc; (void) argv;
main_inputs.is_debug_mode_active = !main_inputs.is_debug_mode_active;
debug_terminal_sync_mode ();
if (main_inputs.is_debug_mode_active) {
term_ok ("Switched to debug mode.\n");
} else {
term_ok ("Switched to game mode.\n");
}
}
static void cmd_dmesg (int argc, char **argv) {
int max_events = 32;
int filter_level = -1;
for (int i = 1; i < argc; i++) {
if (term_str_eq (argv [i], "-n") && (i + 1 < argc)) {
float v = 0.0f;
if (term_parse_float (argv [i + 1], &v)) {max_events = (int) v;}
i++;
} else if (term_str_eq (argv [i], "-l") && (i + 1 < argc)) {
i++;
if (term_str_eq (argv [i], "info")) {filter_level = 0;}
else if (term_str_eq (argv [i], "warn")) {filter_level = 1;}
else if (term_str_eq (argv [i], "error")) {filter_level = 2;}
}
}
if (max_events < 1) {max_events = 1;}
if (max_events > 256) {max_events = 256;}
int total_events = event_log_get_count ();
if (total_events == 0) {
term_dim ("(event log empty)\n");
return;
}
int start_index = total_events - max_events;
if (start_index < 0) {start_index = 0;}
int printed = 0;
for (int i = start_index; i < total_events; i++) {
log_level level;
time_t timestamp;
const char *message = event_log_get_message (i, &level, &timestamp);
if (!message) {continue;}
if ((filter_level >= 0) && ((int) level < filter_level)) {continue;}
struct tm *local_time = localtime (&timestamp);
char time_buffer [32];
strftime (time_buffer, sizeof (time_buffer), "%H:%M:%S", local_time);
const char *level_text = (level == LOG_INFO) ? "INFO" : (level == LOG_WARN) ? "WARN" : "ERR ";
const char *tag_name = (level == LOG_ERROR) ? "term_err" : (level == LOG_WARN) ? "term_echo" : NULL;
term_printf (tag_name, "[%s] %s: %s\n", time_buffer, level_text, message);
printed++;
}
if (printed == 0) {term_dim ("(no matching events)\n");}
else {term_printf ("term_dim", "%d event(s) shown\n", printed);}
}
/* MPE_TASK_V15R2_PHASE7_IMPL_END */
/* MPE_TASK_V15R2_PHASE8_IMPL_BEGIN */
/* MPE_TASK_V15R2_MICROVIM_FILE_WHITELIST_BEGIN */
typedef struct {
const char *path;
const char *description;
} mv_editable_file;
static const mv_editable_file mv_known_files [] = {
{"status/engine.cfg",          "Main engine configuration (57 tunables)"},
{"status/engine.cfg.backup",   "F10/F11 validation config backup"},
{"status/engine.cfg.bak",      "MicroVim auto-backup (last :w)"},
{"../readme.md",               "Project README"},
{"../evolution.txt",           "Version lineage (stage 0 → v15R2)"},
{"../how_to_use.md",           "User guide / controls reference"},
{"../RELEASE_POLICY.md",       "Release cycle rules"},
{"../RELEASE_GATES.md",        "P0/P1/P2 gate checklist"},
{"../release_notes_v15R1.md",  "v15R1 release notes"},
{"../LICENSE",                 "GPL-3.0 license text"},
{"../.gitignore",              "Git ignore rules"},
{"../validation/V01.sh",       "Sanitizer build script"},
{"../validation/V02.sh",       "Clean build + warning review"},
{"../validation/V03.py",       "P0 gate interactive walk"},
{"../validation/V04.sh",       "F10 long-run validation guide"},
{NULL, NULL}
};
static const char *mv_allowed_extensions [] = {
".cfg", ".ini", ".conf", ".txt", ".md", ".glsl", ".sh", ".py", ".h", ".c",
NULL
};
static const char *mv_blocked_extensions [] = {
".dat", ".o", ".so", ".a", ".bin", ".exe", ".obj", ".dll", ".dylib",
NULL
};
static bool mv_file_is_allowed (const char *filepath) {
if (!filepath || filepath [0] == '\0') {return false;}
/* Reject absolute paths */
if (filepath [0] == '/') {return false;}
/* Reject paths with null bytes (defensive) */
/* Check blocked extensions first */
const char *dot = strrchr (filepath, '.');
if (dot) {
for (int i = 0; mv_blocked_extensions [i]; i++) {
if (g_ascii_strcasecmp (dot, mv_blocked_extensions [i]) == 0) {return false;}
}
}
/* Check if it's in the known files list */
for (int i = 0; mv_known_files [i].path; i++) {
if (strcmp (filepath, mv_known_files [i].path) == 0) {return true;}
}
/* Check if it has an allowed extension */
if (dot) {
for (int i = 0; mv_allowed_extensions [i]; i++) {
if (g_ascii_strcasecmp (dot, mv_allowed_extensions [i]) == 0) {return true;}
}
}
return false;
}
/* MPE_TASK_V15R2_MICROVIM_FILE_WHITELIST_END */
static void cmd_vi (int argc, char **argv) {
/* Handle --list / -l flag */
if ((argc > 1) && (term_str_eq (argv [1], "--list") || term_str_eq (argv [1], "-l"))) {
term_printf ("term_echo", "MicroVim editable files:\n");
for (int i = 0; mv_known_files [i].path; i++) {
term_printf (NULL, "  %-32s %s\n", mv_known_files [i].path, mv_known_files [i].description);
}
term_dim ("\nAlso allowed: any file with extensions: .cfg .ini .conf .txt .md .glsl .sh .py .h .c\n");
term_dim ("Blocked: .dat .o .so .a .bin .exe and other binary formats\n");
term_dim ("Usage: vi <path>  |  vi --list\n");
return;
}
/* Handle --help / -h flag */
if ((argc > 1) && (term_str_eq (argv [1], "--help") || term_str_eq (argv [1], "-h"))) {
term_printf ("term_echo", "MicroVim — minimal modal editor\n");
term_out ("\n");
term_out ("  vi [path]       Open file (default: status/engine.cfg)\n");
term_out ("  vi --list       Show editable files\n");
term_out ("  vi --help       This help\n");
term_out ("\n");
term_out ("  Modes: Normal (default), Insert (i/a/o), Command (:)\n");
term_out ("  Nav:   h j k l  w b e  0 $  gg G  Ctrl+F/B  { }\n");
term_out ("  Edit:  x dd dw d$  yy p P  u Ctrl+R  J  ~  cc cw C S\n");
term_out ("  Cmd:   :w :q :q! :wq :x :e <file> :N :set nu :s/o/n/g\n");
term_out ("  Exit:  :q or :wq or double-Escape in Normal mode\n");
return;
}
const char *target_file = "status/engine.cfg";
if (argc > 1) {target_file = argv [1];}
/* Validate file against whitelist */
if (!mv_file_is_allowed (target_file)) {
term_printf ("term_err", "mpe: vi: %s: not an editable file\n", target_file);
term_dim ("Use 'vi --list' to see editable files.\n");
return;
}
/* Prevent opening while microvim is already active */
if (microvim_is_active ()) {
term_err ("mpe: vi: editor already open (close it first with :q or Esc Esc)\n");
return;
}
microvim_open (target_file);
if (terminal_entry) {gtk_widget_hide (terminal_entry);}
if (terminal_prompt_label) {gtk_widget_hide (terminal_prompt_label);}
term_printf ("term_echo", "MicroVim opened: %s\n", target_file);
term_dim ("Modes: Normal/Insert/Command. Esc=Normal, i=Insert, :=Command.\n");
term_dim ("Save: :w  Quit: :q  Save+Quit: :wq  Force quit: :q!  Exit: Esc Esc\n");
if (terminal_output_buffer) {microvim_render (terminal_output_buffer);}
}
/* MPE_TASK_V15R2_PHASE8_IMPL_END */
static void term_execute (char *command_line) {
while (*command_line == ' ') {command_line++;}
if (*command_line == '\0') {return;}
/* MPE_TASK_V15R2_PHASE7_ALIAS_EXPANSION_BEGIN */
{
char first_word [TERM_ALIAS_NAME_LEN];
int word_index = 0;
const char *scan = command_line;
while ((*scan) && (*scan != ' ') && (word_index < TERM_ALIAS_NAME_LEN - 1)) {
first_word [word_index++] = *scan++;
}
first_word [word_index] = '\0';
for (int alias_index = 0; alias_index < term_alias_count; alias_index++) {
if (g_ascii_strcasecmp (first_word, term_alias_names [alias_index]) == 0) {
static char expanded_command [2048];
snprintf (expanded_command, sizeof (expanded_command), "%s%s",
term_alias_values [alias_index], scan);
command_line = expanded_command;
break;
}
}
}
/* MPE_TASK_V15R2_PHASE7_ALIAS_EXPANSION_END */
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
if ((found_command -> mutates) && (!main_inputs.is_debug_mode_active) && (!term_sudo_active)) {
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
/* MPE_TASK_V15R2_MICROVIM_ENTRY_BLOCK */
if (microvim_is_active ()) {return TRUE;}
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
/* MPE_TASK_V15R2_MICROVIM_KEY_ROUTING_BEGIN */
if (microvim_is_active ()) {
if ((event -> keyval == GDK_KEY_Escape) && (microvim_get_mode () == MV_NORMAL)) {
/* Double-escape in normal mode exits editor */
static gint64 last_escape_time = 0;
gint64 now = g_get_monotonic_time ();
if ((now - last_escape_time) < 500000) {
microvim_close ();
if (terminal_entry) {gtk_widget_show (terminal_entry);}
if (terminal_prompt_label) {gtk_widget_show (terminal_prompt_label);}
term_ok ("MicroVim exited.\n");
gtk_widget_grab_focus (terminal_entry);
last_escape_time = 0;
return TRUE;
}
last_escape_time = now;
}
microvim_handle_key (event);
/* MPE_TASK_V15R2_MICROVIM_EXIT_FIX_BEGIN */
if (microvim_is_active ()) {
if (terminal_output_buffer) {microvim_render (terminal_output_buffer);}
} else {
/* MicroVim closed itself via :q, :wq, :q!, or :x */
if (terminal_entry) {gtk_widget_show (terminal_entry);}
if (terminal_prompt_label) {gtk_widget_show (terminal_prompt_label);}
term_ok ("MicroVim exited.\n");
gtk_widget_grab_focus (terminal_entry);
}
/* MPE_TASK_V15R2_MICROVIM_EXIT_FIX_END */
return TRUE;
}
/* MPE_TASK_V15R2_MICROVIM_KEY_ROUTING_END */
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

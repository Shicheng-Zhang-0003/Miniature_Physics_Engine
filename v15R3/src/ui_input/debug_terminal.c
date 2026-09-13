#include "../mpe_engine.h"
#include "debug_terminal.h"
#include "term_priv.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <gdk/gdkkeysyms.h>
/* MPE_TASK_23_POSIX_DEBUG_TERMINAL */
GtkWidget *terminal_window = NULL;
GtkWidget *terminal_output_view = NULL;
GtkTextBuffer *terminal_output_buffer = NULL;
GtkWidget *terminal_entry = NULL;
GtkWidget *terminal_prompt_label = NULL;
char term_cwd[256] = "/";
char term_history[term_history_size][term_history_length + 1];
int term_history_count = 0;
int term_history_cursor = -1;

/* ------------------------------------------------------------------ */
/* Output helpers                                                      */
/* ------------------------------------------------------------------ */
static void term_scroll_to_bottom(void) {
    if (!terminal_output_buffer) {
        return;
    }
    GtkTextIter end_iter;
    gtk_text_buffer_get_end_iter(terminal_output_buffer, &end_iter);
    gtk_text_view_scroll_to_iter(GTK_TEXT_VIEW(terminal_output_view), &end_iter, 0.0, FALSE, 0.0, 0.0);
}
/* MPE_TASK_V15R2_OUTPUT_CAPTURE_BEGIN */
static char *term_capture_buffer = NULL;
static size_t term_capture_length = 0;
static size_t term_capture_capacity = 0;
static bool term_capturing = false;

void term_capture_begin(void) {
    term_capturing = true;
    term_capture_length = 0;
    if (!term_capture_buffer) {
        term_capture_capacity = 8192;
        term_capture_buffer = malloc(term_capture_capacity);
    }
    if (term_capture_buffer) {
        term_capture_buffer[0] = '\0';
    }
}

void term_capture_end(void) {
    term_capturing = false;
}

char *term_capture_get(void) {
    return term_capture_buffer ? term_capture_buffer : "";
}

void term_capture_reset(void) {
    if (term_capture_buffer) {
        free(term_capture_buffer);
        term_capture_buffer = NULL;
    }
    term_capture_length = 0;
    term_capture_capacity = 0;
    term_capturing = false;
}
/* MPE_TASK_V15R2_OUTPUT_CAPTURE_END */
/* MPE_TASK_V15R2_PHASE7_ALIAS_STORAGE_BEGIN */
char term_alias_names[term_alias_max][term_alias_name_len];
char term_alias_values[term_alias_max][term_alias_value_len];
int term_alias_count = 0;

bool term_sudo_active = false;
gint64 term_engine_start_time = 0; /* FIX_029 */
/* MPE_TASK_V15R2_PHASE7_ALIAS_STORAGE_END */

static void term_append_with_tag(const char *tag_name, const char *text) {
    /* MPE_TASK_V15R2_OUTPUT_CAPTURE_INTERCEPT */
    if (term_capturing) {
        size_t text_len = strlen(text);
        while (term_capture_length + text_len + 1 > term_capture_capacity) {
            term_capture_capacity *= 2;
            term_capture_buffer = realloc(term_capture_buffer, term_capture_capacity);
            if (!term_capture_buffer) {
                term_capturing = false;
                return;
            }
        }
        memcpy(term_capture_buffer + term_capture_length, text, text_len + 1);
        term_capture_length += text_len;
        return;
    }
    if (!terminal_output_buffer) {
        return;
    }
    GtkTextIter end_iter;
    gtk_text_buffer_get_end_iter(terminal_output_buffer, &end_iter);
    if (tag_name) {
        gtk_text_buffer_insert_with_tags_by_name(terminal_output_buffer, &end_iter, text, -1, tag_name, NULL);
    } else {
        gtk_text_buffer_insert(terminal_output_buffer, &end_iter, text, -1);
    }
    term_scroll_to_bottom();
}
void term_out(const char *text) {
    term_append_with_tag(NULL, text);
}
void term_ok(const char *text) {
    term_append_with_tag("term_ok", text);
}
void term_err(const char *text) {
    term_append_with_tag("term_err", text);
}
void term_echo(const char *text) {
    term_append_with_tag("term_echo", text);
}
void term_dim(const char *text) {
    term_append_with_tag("term_dim", text);
}
void term_printf(const char *tag_name, const char *format, ...) {
    char line_buffer[2048];
    va_list argument_list;
    va_start(argument_list, format);
    vsnprintf(line_buffer, sizeof(line_buffer), format, argument_list);
    va_end(argument_list);
    term_append_with_tag(tag_name, line_buffer);
}
void term_update_prompt(void) {
    if (!terminal_prompt_label) {
        return;
    }
    char prompt_buffer[320];
    snprintf(prompt_buffer, sizeof(prompt_buffer), "mpe:%s>", term_cwd);
    gtk_label_set_text(GTK_LABEL(terminal_prompt_label), prompt_buffer);
}
/* ------------------------------------------------------------------ */
/* History                                                             */
/* ------------------------------------------------------------------ */
static void term_history_push(const char *command_text) {
    if (command_text[0] == '\0') {
        return;
    }
    if ((term_history_count > 0) && (strcmp(term_history[0], command_text) == 0)) {
        return;
    }
    if (term_history_count < term_history_size) {
        term_history_count++;
    }
    for (int history_index = term_history_count - 1; history_index > 0; history_index--) {
        strncpy(term_history[history_index], term_history[history_index - 1], term_history_length);
        term_history[history_index][term_history_length] = '\0';
    }
    strncpy(term_history[0], command_text, term_history_length);
    term_history[0][term_history_length] = '\0';
}
/* ------------------------------------------------------------------ */
/* String/path helpers                                                 */
/* ------------------------------------------------------------------ */
bool term_str_eq(const char *string_a, const char *string_b) {
    if ((!string_a) || (!string_b)) {
        return false;
    }
    return g_ascii_strcasecmp(string_a, string_b) == 0;
}
const char *term_last_path_component(const char *token) {
    if (!token) {
        return "";
    }
    const char *slash = strrchr(token, '/');
    return slash ? (slash + 1) : token;
}
bool term_is_all_token(const char *token) {
    return term_str_eq(term_last_path_component(token), "all");
}
bool term_parse_float(const char *token, float *output_value) {
    if (!token) {
        return false;
    }
    char *endptr = NULL;
    float parsed_value = strtof(token, &endptr);
    if ((endptr == token) || (*endptr != '\0') || (!isfinite(parsed_value))) {
        return false;
    }
    *output_value = parsed_value;
    return true;
}
int term_object_from_token(const char *token) {
    if (!token) {
        return -1;
    }
    if (term_str_eq(token, "sel") || term_str_eq(term_last_path_component(token), "sel")) {
        if ((selected_object >= 0) && (selected_object < object_count)) {
            return selected_object;
        }
        return -1;
    }
    const char *component = term_last_path_component(token);
    char *endptr = NULL;
    long parsed_index = strtol(component, &endptr, 10);
    if ((endptr == component) || (*endptr != '\0')) {
        return -1;
    }
    if ((parsed_index < 0) || (parsed_index >= object_count)) {
        return -1;
    }
    return (int) parsed_index;
}
int term_joint_from_token(const char *token) {
    if (!token) {
        return -1;
    }
    const char *component = term_last_path_component(token);
    char *endptr = NULL;
    long parsed_index = strtol(component, &endptr, 10);
    if ((endptr == component) || (*endptr != '\0')) {
        return -1;
    }
    if ((parsed_index < 0) || (parsed_index >= mpe_max_joints)) {
        return -1;
    }
    if (!joint_pool[parsed_index].is_active) {
        return -1;
    }
    return (int) parsed_index;
}
term_target_kind term_classify_token(const char *token) {
    if (!token) {
        return term_target_object;
    }
    if (strstr(token, "joint")) {
        return term_target_joint;
    }
    if (strstr(token, "obj")) {
        return term_target_object;
    }
    if (strstr(term_cwd, "joint")) {
        return term_target_joint;
    }
    return term_target_object;
}
int term_require_object(const char *token) {
    int object_index = term_object_from_token(token);
    if (object_index < 0) {
        term_printf("term_err", "mpe: %s: No such object\n", token ? token : "(null)");
    }
    return object_index;
}
int term_require_joint(const char *token) {
    int joint_index = term_joint_from_token(token);
    if (joint_index < 0) {
        term_printf("term_err", "mpe: %s: No such joint\n", token ? token : "(null)");
    }
    return joint_index;
}
int term_parse_movement_destination(const char *token, float *x, float *y, float *z) {
    if (!token) {
        return 0;
    }
    char **parts = g_strsplit(token, "/", -1);
    int movement_kind = 0;
    for (int part_index = 0; parts[part_index]; part_index++) {
        if (term_str_eq(parts[part_index], "pos") || term_str_eq(parts[part_index], "vel")) {
            if ((!parts[part_index + 1]) || (!parts[part_index + 2]) || (!parts[part_index + 3])) {
                break;
            }
            float parsed_x, parsed_y, parsed_z;
            if (!term_parse_float(parts[part_index + 1], &parsed_x)) {
                break;
            }
            if (!term_parse_float(parts[part_index + 2], &parsed_y)) {
                break;
            }
            if (!term_parse_float(parts[part_index + 3], &parsed_z)) {
                break;
            }
            *x = parsed_x;
            *y = parsed_y;
            *z = parsed_z;
            movement_kind = term_str_eq(parts[part_index], "pos") ? 1 : 2;
            break;
        }
    }
    g_strfreev(parts);
    return movement_kind;
}
/* Command declarations moved to term_priv.h. */
const terminal_command terminal_commands[] = {
    {"help", false, cmd_help, "help [command]", "show help"},
    {"man", false, cmd_man, "man <command>", "manual page"},
    {"clear", false, cmd_clear, "clear", "clear terminal"},
    {"history", false, cmd_history, "history", "command history"},
    {"pwd", false, cmd_pwd, "pwd", "print working directory"},
    {"cd", false, cmd_cd, "cd [/|/obj|/joint|..]", "change directory"},
    {"ls", false, cmd_ls, "ls [-l] [path]", "list objects/joints"},
    {"ll", false, cmd_ll, "ll [path]", "long listing"},
    {"cat", false, cmd_cat, "cat <path...>", "inspect object/joint/world/camera/spawner"},
    {"touch", true, cmd_touch, "touch [new.sph|new.cube...]", "create object"},
    {"cp", true, cmd_cp, "cp <object> [dest]", "duplicate object"},
    {"rm", true, cmd_rm, "rm [-rf] <path...>", "remove object/joint/all"},
    {"mv", true, cmd_mv, "mv <object> /pos/x/y/z|/vel/x/y/z", "move or impulse object"},
    {"ln", true, cmd_ln, "ln [-s] <object> <object>", "create spring joint"},
    {"unlink", true, cmd_unlink, "unlink <path>", "remove joint/object"},
    {"chmod", true, cmd_chmod, "chmod static|dynamic <object...>", "change static state"},
    {"chown", true, cmd_chown, "chown <mass> <object...>", "change mass"},
    {"kill", true, cmd_kill, "kill [-STOP|-CONT|-9] <object...>", "sleep/wake/delete object"},
    {"ps", false, cmd_ps, "ps [aux]", "object process table"},
    {"top", false, cmd_top, "top [-n count]", "fastest objects"},
    {"df", false, cmd_df, "df [-h]", "capacity usage"},
    {"du", false, cmd_du, "du <path...>", "object/joint usage"},
    {"uname", false, cmd_uname, "uname [-a|-s|-r|-m|-o]", "engine version"},
    {"whoami", false, cmd_whoami, "whoami", "print user"},
    {"date", false, cmd_date, "date", "print date/time"},
    {"echo", false, cmd_echo, "echo [args...]", "print arguments"},
    {"env", false, cmd_env, "env", "print all config parameters"},
    {"export", true, cmd_export, "export KEY=value", "set config parameter"},
    {"config", true, cmd_config, "config save|load|reset", "manage config file"},
    /* MPE_TASK_V15R2_PHASE2_TABLE_BEGIN */
    {"exit", false, cmd_exit, "exit", "close terminal"},
    {"logout", false, cmd_logout, "logout", "close terminal"},
    {"quit", false, cmd_quit, "quit", "close terminal"},
    {"poweroff", true, cmd_poweroff, "poweroff", "save config and exit engine"},
    {"shutdown", true, cmd_shutdown, "shutdown", "save config and exit engine"},
    {"reboot", true, cmd_reboot, "reboot", "reset scene to default"},
    {"halt", true, cmd_halt, "halt", "pause/resume physics"},
    {"sleep", true, cmd_sleep, "sleep <seconds>", "pause physics for N seconds"},
    {"sync", true, cmd_sync, "sync", "force-save config and scene"},
    {"uptime", false, cmd_uptime, "uptime", "engine runtime stats"},
    {"free", false, cmd_free, "free", "capacity usage report"},
    {"w", false, cmd_w, "w", "camera and input state"},
    {"hostname", false, cmd_hostname, "hostname [-f]", "engine identity"},
    {"id", false, cmd_id, "id", "permission context"},
    {"which", false, cmd_which, "which <command>", "locate command"},
    {"true", false, cmd_true, "true", "no-op success"},
    {"false", false, cmd_false, "false", "no-op failure"},
    {"time", false, cmd_time, "time <command...>", "measure command duration"},
    /* MPE_TASK_V15R2_PHASE2_TABLE_END */
    /* MPE_TASK_V15R2_PHASE3_TABLE_BEGIN */
    {"stat", false, cmd_stat, "stat <path>", "detailed object metadata"},
    {"find", false, cmd_find, "find /obj [filters]", "search objects by property"},
    {"wc", false, cmd_wc, "wc [path]", "count objects/joints"},
    {"file", false, cmd_file, "file <path>", "identify object type"},
    {"diff", false, cmd_diff, "diff <obj_a> <obj_b>", "compare two objects"},
    {"xxd", false, cmd_xxd, "xxd <object> [-l len] [-s off]", "hex dump object struct"},
    /* MPE_TASK_V15R2_PHASE3_TABLE_END */
    /* MPE_TASK_V15R2_PHASE4_TABLE_BEGIN */
    {"sort", false, cmd_sort, "sort /obj [-k field] [-r]", "sort objects by property"},
    {"grep", false, cmd_grep, "grep <pattern> [path]", "filter listing by pattern"},
    {"head", false, cmd_head, "head [-n count] [path]", "first N lines of listing"},
    {"tail", false, cmd_tail, "tail [-n count] [path]", "last N lines of listing"},
    {"less", false, cmd_less, "less [path]", "paginated listing viewer"},
    /* MPE_TASK_V15R2_PHASE4_TABLE_END */
    /* MPE_TASK_V15R2_PHASE5_TABLE_BEGIN */
    {"sed", true, cmd_sed, "sed s/field/value/ <target>", "batch property editor"},
    {"nice", true, cmd_nice, "nice <priority> <object>", "set per-object damping"},
    {"renice", true, cmd_renice, "renice <priority> <object>", "change object damping"},
    {"ping", true, cmd_ping, "ping [-c count] <object>", "test object responsiveness"},
    {"mount", true, cmd_mount, "mount <path>", "load scene file"},
    {"umount", true, cmd_umount, "umount", "save and clear scene"},
    {"mkfs", true, cmd_mkfs, "mkfs", "format scene (clear all)"},
    {"fsck", false, cmd_fsck, "fsck [-y]", "scene integrity check"},
    /* MPE_TASK_V15R2_PHASE5_TABLE_END */
    /* MPE_TASK_V15R2_PHASE6_TABLE_BEGIN */
    {"netstat", false, cmd_netstat, "netstat [-a] [-t]", "joint network topology"},
    {"ifconfig", false, cmd_ifconfig, "ifconfig", "camera/render/input state"},
    {"lsmod", false, cmd_lsmod, "lsmod", "loaded engine modules"},
    /* MPE_TASK_V15R2_PHASE6_TABLE_END */
    /* MPE_TASK_V15R2_PHASE7_TABLE_BEGIN */
    {"alias", true, cmd_alias, "alias [name=value]", "define/list command aliases"},
    {"unalias", true, cmd_unalias, "unalias <name>", "remove command alias"},
    {"jobs", false, cmd_jobs, "jobs", "active validation tests"},
    {"lsof", false, cmd_lsof, "lsof", "open editor/input state"},
    {"seq", false, cmd_seq, "seq [first] <last>", "print number sequence"},
    {"tee", true, cmd_tee, "tee <file> <command...>", "capture output to file"},
    {"watch", false, cmd_watch, "watch <command...>", "execute command (one-shot)"},
    {"sudo", true, cmd_sudo, "sudo <command...>", "bypass game-mode restriction"},
    {"su", true, cmd_su, "su", "toggle game/debug mode"},
    {"dmesg", false, cmd_dmesg, "dmesg [-n count] [-l level]", "engine event log"},
    /* MPE_TASK_V15R2_PHASE7_TABLE_END */
    /* MPE_TASK_V15R2_PHASE8_TABLE_BEGIN */
    {"vi", true, cmd_vi, "vi [file|--list|--help]", "open microvim editor"},
    {"vim", true, cmd_vi, "vim [filename]", "open microvim editor"},
    {"microvim", true, cmd_vi, "microvim [filename]", "open microvim editor"},
    /* MPE_TASK_V15R2_PHASE8_TABLE_END */
};
const size_t terminal_command_count = sizeof(terminal_commands) / sizeof(terminal_commands[0]);

/* ------------------------------------------------------------------ */
/* Listing/print helpers                                               */
/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/* Command implementations                                             */
/* ------------------------------------------------------------------ */
void term_execute(char *command_line) {
    while (*command_line == ' ') {
        command_line++;
    }
    if (*command_line == '\0') {
        return;
    }
    /* MPE_TASK_V15R2_PHASE7_ALIAS_EXPANSION_BEGIN */
    {
        char first_word[term_alias_name_len];
        int word_index = 0;
        const char *scan = command_line;
        while ((*scan) && (*scan != ' ') && (word_index < term_alias_name_len - 1)) {
            first_word[word_index++] = *scan++;
        }
        first_word[word_index] = '\0';
        for (int alias_index = 0; alias_index < term_alias_count; alias_index++) {
            if (g_ascii_strcasecmp(first_word, term_alias_names[alias_index]) == 0) {
                static char expanded_command[2048];
                snprintf(expanded_command, sizeof(expanded_command), "%s%s", term_alias_values[alias_index], scan);
                command_line = expanded_command;
                break;
            }
        }
    }
    /* MPE_TASK_V15R2_PHASE7_ALIAS_EXPANSION_END */
    char prompt_buffer[320];
    snprintf(prompt_buffer, sizeof(prompt_buffer), "mpe:%s> ", term_cwd);
    term_echo(prompt_buffer);
    term_out(command_line);
    term_out("\n");
    int argument_count = 0;
    char **argument_vector = NULL;
    GError *parse_error = NULL;
    if (!g_shell_parse_argv(command_line, &argument_count, &argument_vector, &parse_error)) {
        if (parse_error) {
            term_printf("term_err", "mpe: %s\n", parse_error->message);
            g_clear_error(&parse_error);
        } else {
            term_err("mpe: parse error\n");
        }
        return;
    }
    if (argument_count <= 0) {
        if (argument_vector) {
            g_strfreev(argument_vector);
        }
        return;
    }
    const terminal_command *found_command = NULL;
    for (size_t command_index = 0; command_index < terminal_command_count; command_index++) {
        if (term_str_eq(argument_vector[0], terminal_commands[command_index].name)) {
            found_command = &terminal_commands[command_index];
            break;
        }
    }
    if (!found_command) {
        term_printf("term_err", "mpe: %s: command not found\n", argument_vector[0]);
        g_strfreev(argument_vector);
        return;
    }
    if ((found_command->mutates) && (!main_inputs.is_debug_mode_active) && (!term_sudo_active)) {
        term_printf("term_err", "mpe: %s: Permission denied (switch to debug mode with 0)\n", found_command->name);
        g_strfreev(argument_vector);
        return;
    }
    found_command->handler(argument_count, argument_vector);
    g_strfreev(argument_vector);
}
/* ------------------------------------------------------------------ */
/* GTK signals                                                         */
/* ------------------------------------------------------------------ */
static void on_terminal_entry_activate(GtkEntry *entry) {
    const gchar *entry_text = gtk_entry_get_text(entry);
    char command_copy[term_history_length + 1];
    strncpy(command_copy, entry_text, term_history_length);
    command_copy[term_history_length] = '\0';
    term_history_push(command_copy);
    term_history_cursor = -1;
    term_execute(command_copy);
    gtk_entry_set_text(entry, "");
}
static gboolean on_terminal_entry_keypress(GtkWidget *widget, GdkEventKey *event) {
    /* MPE_TASK_V15R2_MICROVIM_ENTRY_BLOCK */
    if (microvim_is_active()) {
        return TRUE;
    }
    if (event->keyval == GDK_KEY_Up) {
        if (term_history_count > 0) {
            if (term_history_cursor < term_history_count - 1) {
                term_history_cursor++;
            }
            gtk_entry_set_text(GTK_ENTRY(widget), term_history[term_history_cursor]);
            gtk_editable_set_position(GTK_EDITABLE(widget), -1);
        }
        return TRUE;
    }
    if (event->keyval == GDK_KEY_Down) {
        if (term_history_cursor > 0) {
            term_history_cursor--;
            gtk_entry_set_text(GTK_ENTRY(widget), term_history[term_history_cursor]);
        } else {
            term_history_cursor = -1;
            gtk_entry_set_text(GTK_ENTRY(widget), "");
        }
        gtk_editable_set_position(GTK_EDITABLE(widget), -1);
        return TRUE;
    }
    if ((event->state & GDK_CONTROL_MASK) && ((event->keyval == GDK_KEY_l) || (event->keyval == GDK_KEY_L))) {
        if (terminal_output_buffer) {
            gtk_text_buffer_set_text(terminal_output_buffer, "", -1);
        }
        return TRUE;
    }
    return FALSE;
}
static gboolean on_terminal_window_keypress(GtkWidget *widget, GdkEventKey *event) {
    /* MPE_TASK_V15R2_MICROVIM_KEY_ROUTING_BEGIN */
    if (microvim_is_active()) {
        if ((event->keyval == GDK_KEY_Escape) && (microvim_get_mode() == mv_normal)) {
            /* Double-escape in normal mode exits editor */
            static gint64 last_escape_time = 0;
            gint64 now = g_get_monotonic_time();
            if ((now - last_escape_time) < 500000) {
                microvim_close();
                if (terminal_entry) {
                    gtk_widget_show(terminal_entry);
                }
                if (terminal_prompt_label) {
                    gtk_widget_show(terminal_prompt_label);
                }
                term_ok("MicroVim exited.\n");
                gtk_widget_grab_focus(terminal_entry);
                last_escape_time = 0;
                return TRUE;
            }
            last_escape_time = now;
        }
        microvim_handle_key(event);
        /* MPE_TASK_V15R2_MICROVIM_EXIT_FIX_BEGIN */
        if (microvim_is_active()) {
            if (terminal_output_buffer) {
                microvim_render(terminal_output_buffer);
            }
        } else {
            /* MicroVim closed itself via :q, :wq, :q!, or :x */
            if (terminal_output_buffer) {
                gtk_text_buffer_set_text(terminal_output_buffer, "", -1);
            }
            if (terminal_entry) {
                gtk_widget_show(terminal_entry);
            }
            if (terminal_prompt_label) {
                gtk_widget_show(terminal_prompt_label);
            }
            term_ok("MicroVim exited.\n");
            gtk_widget_grab_focus(terminal_entry);
        }
        /* MPE_TASK_V15R2_MICROVIM_EXIT_FIX_END */
        return TRUE;
    }
    /* MPE_TASK_V15R2_MICROVIM_KEY_ROUTING_END */
    if (event->keyval == GDK_KEY_Escape) {
        gtk_widget_destroy(widget);
        return TRUE;
    }
    return FALSE;
}
static void on_terminal_window_destroy(GtkWidget *widget) {
    (void) widget;
    terminal_window = NULL;
    terminal_output_view = NULL;
    terminal_output_buffer = NULL;
    terminal_entry = NULL;
    terminal_prompt_label = NULL;
    term_history_cursor = -1;
    term_capture_reset(); /* FIX_030: free capture buffer on close */
}
/* ------------------------------------------------------------------ */
/* Public interface                                                    */
/* ------------------------------------------------------------------ */
bool debug_terminal_is_open(void) {
    return terminal_window != NULL;
}
void debug_terminal_focus_entry(void) {
    if ((!terminal_window) || (!terminal_entry)) {
        return;
    }
    gtk_window_present(GTK_WINDOW(terminal_window));
    gtk_widget_grab_focus(terminal_entry);
}
void debug_terminal_sync_mode(void) {
    if (!terminal_window) {
        return;
    }
    if (main_inputs.is_debug_mode_active) {
        gtk_window_set_title(GTK_WINDOW(terminal_window), "MPE POSIX Debug Terminal - debug mode");
        term_ok("[terminal unlocked] debug mode active\n");
    } else {
        gtk_window_set_title(GTK_WINDOW(terminal_window), "MPE POSIX Debug Terminal - LOCKED (game mode)");
        term_err("[terminal locked] game mode — read-only commands only\n");
    }
}
void debug_terminal_open(GtkWidget *parent_window) {
    if (!main_inputs.is_debug_mode_active) {
        return;
    }
    if (terminal_window) {
        debug_terminal_focus_entry();
        return;
    }
    terminal_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_widget_set_name(terminal_window, "mpe-debug-terminal");
    gtk_window_set_default_size(GTK_WINDOW(terminal_window), 820, 560);
    if ((parent_window) && (GTK_IS_WIDGET(parent_window))) {
        gtk_window_set_transient_for(GTK_WINDOW(terminal_window), GTK_WINDOW(parent_window));
    }
    g_signal_connect(terminal_window, "destroy", G_CALLBACK(on_terminal_window_destroy), NULL);
    g_signal_connect(terminal_window, "key-press-event", G_CALLBACK(on_terminal_window_keypress), NULL);
    static bool terminal_css_installed = false;
    if (!terminal_css_installed) {
        GtkCssProvider *css_provider = gtk_css_provider_new();
        gtk_css_provider_load_from_data(
            css_provider,
            "#mpe-debug-terminal { background: #0b111a; }\n"
            "#mpe-debug-terminal textview { background: #0b111a; color: #cdd6e4; font-family: monospace; font-size: "
            "13px; }\n"
            "#mpe-debug-terminal textview text { background: #0b111a; }\n"
            "#mpe-debug-terminal entry { background: #101826; color: #ffcf87; caret-color: #ffcf87; font-family: "
            "monospace; font-size: 13px; border: none; padding: 6px 8px; }\n"
            "#mpe-debug-terminal label { color: #ffcf87; font-family: monospace; font-weight: bold; }\n",
            -1, NULL);
        gtk_style_context_add_provider_for_screen(gdk_screen_get_default(), GTK_STYLE_PROVIDER(css_provider),
                                                  GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
        g_object_unref(css_provider);
        terminal_css_installed = true;
    }
    GtkWidget *root_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_add(GTK_CONTAINER(terminal_window), root_box);
    GtkWidget *scrolled_window = gtk_scrolled_window_new(NULL, NULL);
    gtk_box_pack_start(GTK_BOX(root_box), scrolled_window, TRUE, TRUE, 0);
    terminal_output_view = gtk_text_view_new();
    gtk_text_view_set_editable(GTK_TEXT_VIEW(terminal_output_view), FALSE);
    gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(terminal_output_view), FALSE);
    gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(terminal_output_view), GTK_WRAP_WORD_CHAR);
    gtk_text_view_set_left_margin(GTK_TEXT_VIEW(terminal_output_view), 10);
    gtk_text_view_set_right_margin(GTK_TEXT_VIEW(terminal_output_view), 10);
    gtk_text_view_set_top_margin(GTK_TEXT_VIEW(terminal_output_view), 10);
    terminal_output_buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(terminal_output_view));
    gtk_text_buffer_create_tag(terminal_output_buffer, "term_echo", "foreground", "#ffcf87", "weight",
                               PANGO_WEIGHT_BOLD, NULL);
    gtk_text_buffer_create_tag(terminal_output_buffer, "term_ok", "foreground", "#8be28b", NULL);
    gtk_text_buffer_create_tag(terminal_output_buffer, "term_err", "foreground", "#ff7b72", NULL);
    gtk_text_buffer_create_tag(terminal_output_buffer, "term_dim", "foreground", "#5f7387", NULL);
    gtk_container_add(GTK_CONTAINER(scrolled_window), terminal_output_view);
    GtkWidget *input_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_box_pack_start(GTK_BOX(root_box), input_box, FALSE, FALSE, 0);
    terminal_prompt_label = gtk_label_new("mpe:/>");
    gtk_box_pack_start(GTK_BOX(input_box), terminal_prompt_label, FALSE, FALSE, 10);
    terminal_entry = gtk_entry_new();
    gtk_entry_set_placeholder_text(GTK_ENTRY(terminal_entry), "type help and press Enter");
    gtk_box_pack_start(GTK_BOX(input_box), terminal_entry, TRUE, TRUE, 10);
    g_signal_connect(terminal_entry, "activate", G_CALLBACK(on_terminal_entry_activate), NULL);
    g_signal_connect(terminal_entry, "key-press-event", G_CALLBACK(on_terminal_entry_keypress), NULL);
    if (main_inputs.is_mouse_locked) {
        if ((parent_window) && (GTK_IS_WIDGET(parent_window))) {
            mouse_lock_disable(parent_window);
        }
        main_inputs.is_mouse_locked = false;
    }
    gtk_widget_show_all(terminal_window);
    term_update_prompt();
    debug_terminal_sync_mode();
    if (term_engine_start_time == 0) {
        term_engine_start_time = g_get_monotonic_time();
    } /* FIX_029 */
    term_printf("term_echo", "MPE POSIX Debug Terminal %s\n", a3_version_string);
    term_out("Virtual root: /obj /joint /world /camera /spawner\n");
    term_dim("Type 'help' or 'man <command>'. Ctrl+L clears. Esc closes.\n");
}

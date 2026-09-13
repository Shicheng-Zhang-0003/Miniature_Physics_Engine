/* Internal shared declarations for the debug-terminal command files.
 * NOT public API (see debug_terminal.h). All cross-file terminal
 * symbols live here so term_*.c can share the shell core in
 * debug_terminal.c without circular includes. */
#ifndef term_priv_h
#define term_priv_h

#include <gtk/gtk.h>
#include <stdbool.h>
#include <stdint.h>
#include "../core/rigidbody.h"
#include "../config/mpe_constants.h"

/* alias table dimensions (storage lives in debug_terminal.c) */
#define term_alias_max 32
#define term_alias_name_len 64
#define term_alias_value_len 256

typedef enum { term_target_object, term_target_joint } term_target_kind;

typedef struct {
    const char *name;
    bool mutates;
    void (*handler)(int argc, char **argv);
    const char *usage;
    const char *description;
} terminal_command;

/* shell state (defined in debug_terminal.c) */
extern GtkWidget *terminal_window;
extern GtkWidget *terminal_output_view;
extern GtkTextBuffer *terminal_output_buffer;
extern GtkWidget *terminal_entry;
extern GtkWidget *terminal_prompt_label;
extern char term_cwd[256];
extern char term_history[term_history_size][term_history_length + 1];
extern int term_history_count;
extern int term_history_cursor;
extern char term_alias_names[term_alias_max][term_alias_name_len];
extern char term_alias_values[term_alias_max][term_alias_value_len];
extern int term_alias_count;
extern bool term_sudo_active;
extern gint64 term_engine_start_time;

/* output + capture + prompt (defined in debug_terminal.c) */
void term_out(const char *text);
void term_ok(const char *text);
void term_err(const char *text);
void term_echo(const char *text);
void term_dim(const char *text);
void term_update_prompt(void);
void term_capture_begin(void);
void term_capture_end(void);
void term_capture_reset(void);
char *term_capture_get(void);
void term_printf(const char *tag_name, const char *format, ...);

/* token/target helpers (defined in debug_terminal.c) */
bool term_str_eq(const char *string_a, const char *string_b);
const char *term_last_path_component(const char *token);
bool term_is_all_token(const char *token);
bool term_parse_float(const char *token, float *output_value);
int term_object_from_token(const char *token);
int term_joint_from_token(const char *token);
term_target_kind term_classify_token(const char *token);
int term_require_object(const char *token);
int term_require_joint(const char *token);
int term_parse_movement_destination(const char *token, float *x, float *y, float *z);
void term_execute(char *command_line);

/* object/joint model layer (defined in term_obj.c) */
const char *term_object_type_name(rigidbody *rigid_body);
const char *term_object_state_name(rigidbody *rigid_body);
const char *term_object_mode(rigidbody *rigid_body);
void term_print_object_long(int object_index);
void term_print_joint_long(int joint_index);
void term_list_objects(bool long_format);
void term_list_joints(bool long_format);
void term_list_root(bool long_format);
void term_print_object_cat(int object_index);
void term_print_joint_cat(int joint_index);
void term_print_world(void);
void term_print_camera(void);
void term_print_spawner(void);
int term_create_object(object_type spawn_type);
int term_duplicate_object(int source_index);
void term_set_object_mass(int object_index, float new_mass);
void term_set_object_static(int object_index, bool make_static);
bool term_mode_is_static(const char *mode_text);

/* dispatch table (defined in debug_terminal.c; read by help/man) */
extern const terminal_command terminal_commands[];
extern const size_t terminal_command_count;

/* command handlers (defined in term_*.c) */
void cmd_help(int argc, char **argv);
void cmd_man(int argc, char **argv);
void cmd_clear(int argc, char **argv);
void cmd_history(int argc, char **argv);
void cmd_pwd(int argc, char **argv);
void cmd_cd(int argc, char **argv);
void cmd_ls(int argc, char **argv);
void cmd_ll(int argc, char **argv);
void cmd_cat(int argc, char **argv);
void cmd_touch(int argc, char **argv);
void cmd_cp(int argc, char **argv);
void cmd_rm(int argc, char **argv);
void cmd_mv(int argc, char **argv);
void cmd_ln(int argc, char **argv);
void cmd_unlink(int argc, char **argv);
void cmd_chmod(int argc, char **argv);
void cmd_chown(int argc, char **argv);
void cmd_kill(int argc, char **argv);
void cmd_ps(int argc, char **argv);
void cmd_top(int argc, char **argv);
void cmd_df(int argc, char **argv);
void cmd_du(int argc, char **argv);
void cmd_uname(int argc, char **argv);
void cmd_whoami(int argc, char **argv);
void cmd_date(int argc, char **argv);
void cmd_echo(int argc, char **argv);
void cmd_env(int argc, char **argv);
void cmd_export(int argc, char **argv);
void cmd_config(int argc, char **argv);
void cmd_exit(int argc, char **argv);
void cmd_logout(int argc, char **argv);
void cmd_quit(int argc, char **argv);
void cmd_poweroff(int argc, char **argv);
void cmd_shutdown(int argc, char **argv);
void cmd_reboot(int argc, char **argv);
void cmd_halt(int argc, char **argv);
void cmd_sleep(int argc, char **argv);
void cmd_sync(int argc, char **argv);
void cmd_uptime(int argc, char **argv);
void cmd_free(int argc, char **argv);
void cmd_w(int argc, char **argv);
void cmd_hostname(int argc, char **argv);
void cmd_id(int argc, char **argv);
void cmd_which(int argc, char **argv);
void cmd_true(int argc, char **argv);
void cmd_false(int argc, char **argv);
void cmd_time(int argc, char **argv);
void cmd_stat(int argc, char **argv);
void cmd_find(int argc, char **argv);
void cmd_wc(int argc, char **argv);
void cmd_file(int argc, char **argv);
void cmd_diff(int argc, char **argv);
void cmd_xxd(int argc, char **argv);
void cmd_sort(int argc, char **argv);
void cmd_grep(int argc, char **argv);
void cmd_head(int argc, char **argv);
void cmd_tail(int argc, char **argv);
void cmd_less(int argc, char **argv);
void cmd_sed(int argc, char **argv);
void cmd_nice(int argc, char **argv);
void cmd_renice(int argc, char **argv);
void cmd_ping(int argc, char **argv);
void cmd_mount(int argc, char **argv);
void cmd_umount(int argc, char **argv);
void cmd_mkfs(int argc, char **argv);
void cmd_fsck(int argc, char **argv);
void cmd_netstat(int argc, char **argv);
void cmd_ifconfig(int argc, char **argv);
void cmd_lsmod(int argc, char **argv);
void cmd_alias(int argc, char **argv);
void cmd_unalias(int argc, char **argv);
void cmd_jobs(int argc, char **argv);
void cmd_lsof(int argc, char **argv);
void cmd_seq(int argc, char **argv);
void cmd_tee(int argc, char **argv);
void cmd_watch(int argc, char **argv);
void cmd_sudo(int argc, char **argv);
void cmd_su(int argc, char **argv);
void cmd_dmesg(int argc, char **argv);
void cmd_vi(int argc, char **argv);

#endif /* term_priv_h */

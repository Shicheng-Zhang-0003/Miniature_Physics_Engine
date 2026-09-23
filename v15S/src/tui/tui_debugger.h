#ifndef tui_debugger_h
#define tui_debugger_h

#include "../core/physics_world.h"
#include "../core/rigidbody.h"
#include <stdio.h>
/* Forward-declare ncurses WINDOW so non-curses consumers (tui_dump,
 * snapshot tests) don't inherit <ncurses.h>. */
typedef struct _win_st WINDOW;

typedef enum {
    TUI_MODE_OVERVIEW,
    TUI_MODE_OBJECT_DETAIL,
    TUI_MODE_JOINT_DETAIL,
    TUI_MODE_SCENE_GRAPH,
    TUI_MODE_HELP
} tui_mode_t;

typedef struct {
    physics_world *world;
    tui_mode_t mode;
    int selected_object;
    int selected_joint;
    int scroll_offset;
    int detail_scroll;
    bool paused;
    float time_scale;
    uint64_t frame_count;
    double last_frame_time;
    WINDOW *main_win;
    WINDOW *sidebar_win;
    WINDOW *detail_win;
    WINDOW *status_win;
    int term_width;
    int term_height;
    char filter_text[64];
    int filter_len;
    bool filter_active;
} tui_debugger_t;

void tui_debugger_init(tui_debugger_t *dbg, physics_world *world);
void tui_debugger_cleanup(tui_debugger_t *dbg);
void tui_debugger_run(tui_debugger_t *dbg);
void tui_debugger_step(tui_debugger_t *dbg, float dt);
void tui_debugger_render(tui_debugger_t *dbg);
void tui_debugger_handle_input(tui_debugger_t *dbg, int ch);

void tui_render_overview(tui_debugger_t *dbg);
void tui_render_object_detail(tui_debugger_t *dbg);
void tui_render_joint_detail(tui_debugger_t *dbg);
void tui_render_scene_graph(tui_debugger_t *dbg);
void tui_render_help(tui_debugger_t *dbg);
void tui_render_sidebar(tui_debugger_t *dbg);
void tui_render_status(tui_debugger_t *dbg);

void tui_format_vector3(char *buf, size_t sz, vector3 v, const char *label);
void tui_format_vector4(char *buf, size_t sz, vector4 q, const char *label);
void tui_format_euler(char *buf, size_t sz, vector4 q, const char *label);
void tui_format_matrix3(char *buf, size_t sz, math3 m, const char *label);
float tui_quat_to_yaw(vector4 q);
float tui_quat_to_pitch(vector4 q);
float tui_quat_to_roll(vector4 q);

/* Plain-text state dump (tui_dump.c). No ncurses: scriptable, diffable,
 * pipeable — this is the terminal debug-output suite. Prints the full
 * engine/body/joint/graph state for the given tick. Returns 0 when every
 * body position/velocity is finite, 1 otherwise. */
int tui_dump_snapshot(FILE *out, physics_world *world, unsigned long tick, float dt);

/* One-line body summary shared by the UI table and the dump. */
void tui_describe_body(physics_world *world, int idx, char *buf, size_t sz);

#endif
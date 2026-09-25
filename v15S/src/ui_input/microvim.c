/* GTK4-PREP: GTK3 preserved under #else; GTK4 full port follows. */
#ifdef MPE_GTK4

/* MPE_TASK_V15R2_MICROVIM_IMPL_BEGIN */
#include "../mpe_engine.h"
#include "microvim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <stdint.h>
#include <ctype.h>
#include <gdk/gdkkeysyms.h>

#define mv_view_height 40
#define mv_undo_depth 64
#define mv_max_line_len 4096

/* ------------------------------------------------------------------ */
/* State                                                                */
/* ------------------------------------------------------------------ */
typedef struct {
    char **lines;
    int line_count;
    int line_capacity;
} mv_snapshot;

static struct {
    char **lines;
    int line_count;
    int line_capacity;
    int cursor_row;
    int cursor_col;
    int preferred_col;
    mv_mode mode;
    char pending_op;
    char command_buf[256];
    int command_len;
    char search_buf[256];
    int search_len;
    bool search_forward;
    bool modified;
    char filename[512];
    bool show_line_numbers;
    char *yank_text;
    bool yank_is_linewise;
    int scroll_offset;
    bool active;
    bool quit_requested;
    bool file_exists; /* MPE_TASK_V15R2_FILE_EXISTS_FLAG */
    mv_snapshot undo_stack[mv_undo_depth];
    int undo_top;
    int redo_top;
} mv;

/* ------------------------------------------------------------------ */
/* Line buffer helpers                                                  */
/* ------------------------------------------------------------------ */
static void mv_clear_lines(void) {
    for (int i = 0; i < mv.line_count; i++) {
        if (mv.lines[i]) {
            free(mv.lines[i]);
            mv.lines[i] = NULL;
        }
    }
    mv.line_count = 0;
}

static bool mv_ensure_capacity(int needed) {
    if (needed < 0 || (size_t)needed > SIZE_MAX / sizeof(*mv.lines)) return false;
    if (needed <= mv.line_capacity) {
        return true;
    }
    int new_cap = mv.line_capacity < 64 ? 64
                  : mv.line_capacity <= INT_MAX / 2 ? mv.line_capacity * 2
                  : needed;
    if (new_cap < needed) {
        new_cap = needed;
    }
    if ((size_t)new_cap > SIZE_MAX / sizeof(*mv.lines)) return false;
    char **new_lines = (char **)realloc(mv.lines, (size_t)new_cap * sizeof(*mv.lines));
    if (!new_lines) return false;
    for (int i = mv.line_capacity; i < new_cap; i++) {
        new_lines[i] = NULL;
    }
    mv.lines = new_lines;
    mv.line_capacity = new_cap;
    return true;
}

static bool mv_set_line(int index, const char *text) {
    if (index < 0) return false;
    char *replacement = text && strlen(text) >= (size_t)mv_max_line_len
                            ? term_strndup(text, mv_max_line_len - 1)
                            : term_strdup(text ? text : "");
    if (!replacement || !mv_ensure_capacity(index + 1)) {
        free(replacement);
        return false;
    }
    if (mv.lines[index]) {
        free(mv.lines[index]);
    }
    mv.lines[index] = replacement;
    return true;
}

static bool mv_insert_line(int index, const char *text) {
    if (index < 0 || index > mv.line_count || mv.line_count == INT_MAX) return false;
    char *replacement = text && strlen(text) >= (size_t)mv_max_line_len
                            ? term_strndup(text, mv_max_line_len - 1)
                            : term_strdup(text ? text : "");
    if (!replacement || !mv_ensure_capacity(mv.line_count + 1)) {
        free(replacement);
        return false;
    }
    for (int i = mv.line_count; i > index; i--) {
        mv.lines[i] = mv.lines[i - 1];
    }
    mv.lines[index] = replacement;
    mv.line_count++;
    return true;
}

static void mv_delete_line(int index) {
    if ((index < 0) || (index >= mv.line_count)) {
        return;
    }
    free(mv.lines[index]);
    for (int i = index; i < mv.line_count - 1; i++) {
        mv.lines[i] = mv.lines[i + 1];
    }
    mv.lines[mv.line_count - 1] = NULL;
    mv.line_count--;
    if (mv.line_count == 0) {
        if (!mv_insert_line(0, "")) microvim_close();
    }
}

static int mv_line_len(int row) {
    if ((row < 0) || (row >= mv.line_count) || !mv.lines || !mv.lines[row]) {
        return 0;
    }
    return (int) strlen(mv.lines[row]);
}

static void mv_clamp_cursor(void) {
    if (mv.line_count <= 0) {
        mv.cursor_row = 0;
        mv.cursor_col = 0;
        return;
    }
    if (mv.cursor_row < 0) {
        mv.cursor_row = 0;
    }
    if (mv.cursor_row >= mv.line_count) {
        mv.cursor_row = mv.line_count - 1;
    }
    int len = mv_line_len(mv.cursor_row);
    if (mv.cursor_col > len) {
        mv.cursor_col = len;
    }
    if (mv.cursor_col < 0) {
        mv.cursor_col = 0;
    }
}

/* ------------------------------------------------------------------ */
/* Undo                                                                 */
/* ------------------------------------------------------------------ */
static void mv_snapshot_clear(mv_snapshot *snap) {
    if (!snap) return;
    if (snap->lines) {
        for (int i = 0; i < snap->line_count; i++) free(snap->lines[i]);
        free(snap->lines);
    }
    *snap = (mv_snapshot){0};
}

static void mv_clear_undo_history(void) {
    for (int i = 0; i < mv_undo_depth; i++) mv_snapshot_clear(&mv.undo_stack[i]);
    mv.undo_top = 0;
    mv.redo_top = 0;
}

static bool mv_undo_push(void) {
    if (mv.line_count <= 0 || !mv.lines) return false;
    int snapshot_count = mv.line_count;
    char **snapshot_lines = (char **)calloc((size_t)snapshot_count, sizeof(*snapshot_lines));
    if (!snapshot_lines) return false;
    for (int i = 0; i < snapshot_count; i++) {
        if (!mv.lines[i] || !(snapshot_lines[i] = term_strdup(mv.lines[i]))) {
            for (int j = 0; j < i; j++) free(snapshot_lines[j]);
            free(snapshot_lines);
            return false;
        }
    }
    for (int i = mv.undo_top; i < mv.redo_top && i < mv_undo_depth; i++)
        mv_snapshot_clear(&mv.undo_stack[i]);
    mv.redo_top = mv.undo_top;
    if (mv.undo_top >= mv_undo_depth) {
        mv_snapshot_clear(&mv.undo_stack[0]);
        for (int i = 1; i < mv_undo_depth; i++) mv.undo_stack[i - 1] = mv.undo_stack[i];
        mv.undo_stack[mv_undo_depth - 1] = (mv_snapshot){0};
        mv.undo_top = mv_undo_depth - 1;
        mv.redo_top = mv.undo_top;
    }
    mv.undo_stack[mv.undo_top].lines = snapshot_lines;
    mv.undo_stack[mv.undo_top].line_count = snapshot_count;
    mv.undo_stack[mv.undo_top].line_capacity = snapshot_count;
    mv.undo_top++;
    mv.redo_top = mv.undo_top;
    return true;
}

static bool mv_restore_snapshot(const mv_snapshot *snap) {
    if (!snap || snap->line_count <= 0 || !snap->lines) return false;
    char **restored = (char **)calloc((size_t)snap->line_count, sizeof(*restored));
    if (!restored) return false;
    for (int i = 0; i < snap->line_count; i++) {
        if (!snap->lines[i] || !(restored[i] = term_strdup(snap->lines[i]))) {
            for (int j = 0; j < i; j++) free(restored[j]);
            free(restored);
            return false;
        }
    }
    if (!mv_ensure_capacity(snap->line_count)) {
        for (int i = 0; i < snap->line_count; i++) free(restored[i]);
        free(restored);
        return false;
    }
    mv_clear_lines();
    memcpy(mv.lines, restored, (size_t)snap->line_count * sizeof(*restored));
    mv.line_count = snap->line_count;
    free(restored);
    return true;
}

static void mv_undo_perform(void) {
    if (mv.undo_top <= 0) {
        return;
    }
    mv_snapshot *snap = &mv.undo_stack[mv.undo_top - 1];
    if (!mv_restore_snapshot(snap)) return;
    mv.undo_top--;
    mv.modified = true;
    mv_clamp_cursor();
}

static void mv_redo_perform(void) {
    if (mv.undo_top >= mv.redo_top) {
        return;
    }
    mv_snapshot *snap = &mv.undo_stack[mv.undo_top];
    if (!mv_restore_snapshot(snap)) return;
    mv.undo_top++;
    mv.modified = true;
    mv_clamp_cursor();
}

/* ------------------------------------------------------------------ */
/* File I/O                                                             */
/* ------------------------------------------------------------------ */
static bool mv_load_file(const char *filename) {
    mv_clear_lines();
    if (!filename || !mv_ensure_capacity(64)) return false;
    FILE *f = fopen(filename, "r");
    if (!f) {
        if (!mv_insert_line(0, "")) return false;
        return false;
    }
    char line_buf[mv_max_line_len];
    while (fgets(line_buf, sizeof(line_buf), f)) {
        size_t len = strlen(line_buf);
        while ((len > 0) && ((line_buf[len - 1] == '\n') || (line_buf[len - 1] == '\r'))) {
            line_buf[--len] = '\0';
        }
        if (!mv_insert_line(mv.line_count, line_buf)) {
            fclose(f);
            mv_clear_lines();
            return false;
        }
    }
    fclose(f);
    if (mv.line_count == 0) {
        if (!mv_insert_line(0, "")) return false;
    }
    return true;
}

static bool mv_save_file(void) {
    char backup_path[560];
    snprintf(backup_path, sizeof(backup_path), "%s.bak", mv.filename);
    FILE *existing = fopen(mv.filename, "r");
    if (existing) {
        fclose(existing);
        FILE *src = fopen(mv.filename, "r");
        FILE *dst = fopen(backup_path, "w");
        if (src && dst) {
            char buf[4096];
            size_t n;
            while ((n = fread(buf, 1, sizeof(buf), src)) > 0) {
                fwrite(buf, 1, n, dst);
            }
        }
        if (src) {
            fclose(src);
        }
        if (dst) {
            fclose(dst);
        }
    }
    FILE *f = fopen(mv.filename, "w");
    if (!f) {
        return false;
    }
    for (int i = 0; i < mv.line_count; i++) {
        fprintf(f, "%s\n", mv.lines[i]);
    }
    fclose(f);
    mv.modified = false;
    return true;
}

/* ------------------------------------------------------------------ */
/* Word motion                                                          */
/* ------------------------------------------------------------------ */
static bool mv_is_word_char(char c) {
    return isalnum((unsigned char) c) || (c == '_');
}

static void mv_word_forward(void) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    int len = mv_line_len(row);
    while (col < len && mv_is_word_char(mv.lines[row][col])) {
        col++;
    }
    while (col < len && !mv_is_word_char(mv.lines[row][col]) && mv.lines[row][col] != ' ') {
        col++;
    }
    while (col < len && mv.lines[row][col] == ' ') {
        col++;
    }
    if (col >= len) {
        if (row + 1 < mv.line_count) {
            row++;
            col = 0;
        }
    }
    mv.cursor_row = row;
    mv.cursor_col = col;
    mv_clamp_cursor();
}

static void mv_word_backward(void) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    if (col > 0) {
        col--;
    }
    while (col > 0 && mv.lines[row][col] == ' ') {
        col--;
    }
    if (col > 0 && mv_is_word_char(mv.lines[row][col])) {
        while (col > 0 && mv_is_word_char(mv.lines[row][col - 1])) {
            col--;
        }
    } else if (col > 0) {
        while (col > 0 && !mv_is_word_char(mv.lines[row][col - 1]) && mv.lines[row][col - 1] != ' ') {
            col--;
        }
    }
    mv.cursor_row = row;
    mv.cursor_col = col;
    mv_clamp_cursor();
}

static void mv_word_end(void) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    int len = mv_line_len(row);
    if (col < len) {
        col++;
    }
    while (col < len && mv.lines[row][col] == ' ') {
        col++;
    }
    if (col < len && mv_is_word_char(mv.lines[row][col])) {
        while (col < len && mv_is_word_char(mv.lines[row][col])) {
            col++;
        }
    } else {
        while (col < len && !mv_is_word_char(mv.lines[row][col]) && mv.lines[row][col] != ' ') {
            col++;
        }
    }
    if (col > 0) {
        col--;
    }
    mv.cursor_row = row;
    mv.cursor_col = col;
    mv_clamp_cursor();
}

/* ------------------------------------------------------------------ */
/* Search                                                               */
/* ------------------------------------------------------------------ */
static void mv_search_execute(bool forward) {
    if (mv.search_len == 0 || mv.line_count <= 0) {
        return;
    }
    /* Lower pattern once instead of per-line per-attempt (old code allocated
     * two strings per line scanned). */
    char *pattern_lower = term_ascii_strdown(mv.search_buf);
    if (!pattern_lower) {
        return;
    }
    size_t pat_len = strlen(pattern_lower);
    int start_row = mv.cursor_row;
    for (int attempt = 0; attempt < mv.line_count * 2; attempt++) {
        int row = (forward ? (start_row + attempt) : (start_row - attempt + mv.line_count * 2)) % mv.line_count;
        const char *line = mv.lines[row];
        if (!line) {
            continue;
        }
        /* Case-insensitive substring search without per-line allocation. */
        const char *match = NULL;
        size_t line_len = strlen(line);
        if (pat_len <= line_len) {
            for (size_t i = 0; i + pat_len <= line_len; i++) {
                bool eq = true;
                for (size_t k = 0; k < pat_len; k++) {
                    if (term_ascii_tolower(line[i + k]) != pattern_lower[k]) {
                        eq = false;
                        break;
                    }
                }
                if (eq) {
                    match = line + i;
                    /* For forward/backward skip semantics we need the first
                     * valid match per row; for simplicity take the first and
                     * apply cursor gating below. A second scan finds later
                     * matches on the start row if the first is behind cursor. */
                    int col0 = (int) (match - line);
                    if ((row == start_row) && forward && (col0 <= mv.cursor_col)) {
                        /* Look for a later match on the same row. */
                        const char *later = NULL;
                        for (size_t j = (size_t)(mv.cursor_col + 1); j + pat_len <= line_len; j++) {
                            bool eq2 = true;
                            for (size_t k = 0; k < pat_len; k++) {
                                if (term_ascii_tolower(line[j + k]) != pattern_lower[k]) {
                                    eq2 = false;
                                    break;
                                }
                            }
                            if (eq2) {
                                later = line + j;
                                break;
                            }
                        }
                        if (later) {
                            match = later;
                        } else {
                            continue;
                        }
                    }
                    if ((row == start_row) && !forward && ((int) (match - line) >= mv.cursor_col)) {
                        continue;
                    }
                    mv.cursor_row = row;
                    mv.cursor_col = (int) (match - line);
                    free(pattern_lower);
                    return;
                }
            }
        }
    }
    free(pattern_lower);
}

/* ------------------------------------------------------------------ */
/* Editing operations                                                   */
/* ------------------------------------------------------------------ */
static void mv_delete_char_under_cursor(void) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    int len = mv_line_len(row);
    if (col >= len) {
        return;
    }
    if (!mv_undo_push()) return;
    char *line = mv.lines[row];
    memmove(line + col, line + col + 1, (size_t) (len - col));
    mv.modified = true;
    mv_clamp_cursor();
}

static void mv_delete_char_before_cursor(void) {
    int col = mv.cursor_col;
    if (col <= 0) {
        return;
    }
    if (!mv_undo_push()) return;
    char *line = mv.lines[mv.cursor_row];
    int len = mv_line_len(mv.cursor_row);
    memmove(line + col - 1, line + col, (size_t) (len - col + 1));
    mv.cursor_col--;
    mv.modified = true;
}

static void mv_delete_line_op(void) {
    if (!mv_undo_push()) return;
    if (mv.yank_text) {
        free(mv.yank_text);
    }
    mv.yank_text = term_strdup(mv.lines[mv.cursor_row]);
    mv.yank_is_linewise = true;
    mv_delete_line(mv.cursor_row);
    mv.modified = true;
    mv_clamp_cursor();
}

static void mv_delete_to_end(void) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    int len = mv_line_len(row);
    if (col >= len) {
        return;
    }
    if (!mv_undo_push()) return;
    if (mv.yank_text) {
        free(mv.yank_text);
    }
    mv.yank_text = term_strdup(mv.lines[row] + col);
    mv.yank_is_linewise = false;
    mv.lines[row][col] = '\0';
    mv.modified = true;
    mv_clamp_cursor();
}

static void mv_delete_to_start(void) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    if (col <= 0) {
        return;
    }
    if (!mv_undo_push()) return;
    if (mv.yank_text) {
        free(mv.yank_text);
    }
    mv.yank_text = term_strndup(mv.lines[row], col);
    mv.yank_is_linewise = false;
    char *new_line = term_strdup(mv.lines[row] + col);
    free(mv.lines[row]);
    mv.lines[row] = new_line;
    mv.cursor_col = 0;
    mv.modified = true;
}

static void mv_yank_line(void) {
    if (mv.yank_text) {
        free(mv.yank_text);
    }
    mv.yank_text = term_strdup(mv.lines[mv.cursor_row]);
    mv.yank_is_linewise = true;
}

static void mv_paste(bool after) {
    if (!mv.yank_text) {
        return;
    }
    if (!mv_undo_push()) return;
    if (mv.yank_is_linewise) {
        int insert_row = after ? mv.cursor_row + 1 : mv.cursor_row;
        if (!mv_insert_line(insert_row, mv.yank_text)) return;
        mv.cursor_row = insert_row;
        mv.cursor_col = 0;
    } else {
        int row = mv.cursor_row;
        int col = after ? mv.cursor_col + 1 : mv.cursor_col;
        int len = mv_line_len(row);
        if (col > len) {
            col = len;
        }
        int yank_len = (int) strlen(mv.yank_text);
        char *new_line = (char *) malloc((size_t) (len + yank_len + 1));
        memcpy(new_line, mv.lines[row], (size_t) col);
        memcpy(new_line + col, mv.yank_text, (size_t) yank_len);
        memcpy(new_line + col + yank_len, mv.lines[row] + col, (size_t) (len - col + 1));
        free(mv.lines[row]);
        mv.lines[row] = new_line;
        mv.cursor_col = col + yank_len - 1;
    }
    mv.modified = true;
    mv_clamp_cursor();
}

static void mv_join_lines(void) {
    if (mv.cursor_row >= mv.line_count - 1) {
        return;
    }
    if (!mv_undo_push()) return;
    int len_a = mv_line_len(mv.cursor_row);
    char *joined = term_format("%s %s", mv.lines[mv.cursor_row], mv.lines[mv.cursor_row + 1]);
    if (!joined || !mv_set_line(mv.cursor_row, joined)) {
        free(joined);
        return;
    }
    free(joined);
    mv_delete_line(mv.cursor_row + 1);
    mv.cursor_col = len_a;
    mv.modified = true;
    mv_clamp_cursor();
}

static void mv_insert_char(char c) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    int len = mv_line_len(row);
    if (len >= mv_max_line_len - 1) {
        return;
    } /* FIX_039: prevent overflow */
    char *line = mv.lines[row];
    char *new_line = (char *) malloc((size_t) (len + 2));
    if (!new_line) return;
    memcpy(new_line, line, (size_t) col);
    new_line[col] = c;
    memcpy(new_line + col + 1, line + col, (size_t) (len - col + 1));
    free(mv.lines[row]);
    mv.lines[row] = new_line;
    mv.cursor_col++;
    mv.modified = true;
}

static void mv_insert_newline(void) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    char *second_half = term_strdup(mv.lines[row] + col);
    if (!second_half || !mv_insert_line(row + 1, second_half)) {
        free(second_half);
        return;
    }
    mv.lines[row][col] = '\0';
    free(second_half);
    mv.cursor_row++;
    mv.cursor_col = 0;
    mv.modified = true;
}

static void mv_backspace(void) {
    if (mv.cursor_col > 0) {
        mv_delete_char_before_cursor();
    } else if (mv.cursor_row > 0) {
        if (!mv_undo_push()) return;
        int prev_len = mv_line_len(mv.cursor_row - 1);
        char *joined = term_format("%s%s", mv.lines[mv.cursor_row - 1], mv.lines[mv.cursor_row]);
        if (!joined || !mv_set_line(mv.cursor_row - 1, joined)) {
            free(joined);
            return;
        }
        free(joined);
        mv_delete_line(mv.cursor_row);
        mv.cursor_row--;
        mv.cursor_col = prev_len;
        mv.modified = true;
    }
}

/* ------------------------------------------------------------------ */
/* Command mode execution                                               */
/* ------------------------------------------------------------------ */
static void mv_execute_command(void) {
    char *cmd = mv.command_buf;
    if (cmd[0] == 'w' && cmd[1] == 'q') {
        mv_save_file();
        mv.file_exists = true; /* MPE_TASK_V15R2_WQ_SETS_EXISTS */
        mv.quit_requested = true;
    } else if (cmd[0] == 'w' && cmd[1] == '\0') {
        mv_save_file();
        mv.file_exists = true; /* MPE_TASK_V15R2_SAVE_SETS_EXISTS */
        /* Config integration: reload if editing engine.cfg */
        if (strstr(mv.filename, "engine.cfg")) {
            mpe_config_load("status/engine.cfg");
            contact_cache_clear(physics_world_get_primary());
        }
        mv.mode = mv_normal;
    } else if (cmd[0] == 'q' && cmd[1] == '!') {
        mv.quit_requested = true;
    } else if (cmd[0] == 'q' && cmd[1] == '\0') {
        if (mv.modified) {
            /* MPE_TASK_V15R2_QUIT_MODIFIED_ERROR_BEGIN */
            mv.mode = mv_normal;
            mv.command_len = 0;
            mv.command_buf[0] = '\0';
            /* Render an error message in the status area by temporarily
setting a flag that microvim_render will pick up */
            mv.command_len =
                snprintf(mv.command_buf, sizeof(mv.command_buf), "E37: No write since last change (use :q! or :wq)");
            mv.mode = mv_command; /* Show error in command line area */
            return;
            /* MPE_TASK_V15R2_QUIT_MODIFIED_ERROR_END */
        }
        mv.quit_requested = true;
    } else if (cmd[0] == 'x') {
        mv_save_file();
        mv.quit_requested = true;
    } else if (cmd[0] == 'e' && cmd[1] == ' ') {
        if (!mv.modified || (cmd[2] == '!')) {
            mv_load_file(cmd + 2);
            if (mv.line_count == 0) {
                microvim_close();
                return;
            }
            mv.cursor_row = 0;
            mv.cursor_col = 0;
            mv.modified = false;
            mv_clear_undo_history();
        }
        mv.mode = mv_normal;
    } else if (strncmp(cmd, "set number", 10) == 0 || strncmp(cmd, "set nu", 6) == 0) {
        mv.show_line_numbers = true;
        mv.mode = mv_normal;
    } else if (strncmp(cmd, "set nonumber", 12) == 0 || strncmp(cmd, "set nonu", 8) == 0) {
        mv.show_line_numbers = false;
        mv.mode = mv_normal;
    } else if (cmd[0] == 's' && cmd[1] == '/') {
        /* Substitute on current line: s/old/new/ or s/old/new/g */
        char *p1 = cmd + 2;
        char *p2 = strchr(p1, '/');
        if (p2) {
            *p2 = '\0';
            char *p3 = p2 + 1;
            char *p4 = strchr(p3, '/');
            bool global_replace = false;
            if (p4) {
                *p4 = '\0';
                if (*(p4 + 1) == 'g') {
                    global_replace = true;
                }
            }
            if (!p1[0]) {
                mv.command_len = snprintf(mv.command_buf, sizeof(mv.command_buf),
                                          "E486: empty substitute pattern");
                return;
            }
            char *line = mv.lines[mv.cursor_row];
            char *lower_line = term_ascii_strdown(line);
            char *lower_old = term_ascii_strdown(p1);
            if (!lower_line || !lower_old) {
                free(lower_line);
                free(lower_old);
                return;
            }
            char *match = strstr(lower_line, lower_old);
            if (match) {
                if (!mv_undo_push()) {
                    free(lower_line);
                    free(lower_old);
                    return;
                }
                int old_len = (int) strlen(p1);
                int new_len = (int) strlen(p3);
                int pos = (int) (match - lower_line);
                do {
                    int line_len = (int) strlen(mv.lines[mv.cursor_row]);
                    if (new_len > mv_max_line_len - 1 - (line_len - old_len)) break;
                    char *new_line = (char *) malloc((size_t) (line_len - old_len + new_len + 1));
                    if (!new_line) break;
                    memcpy(new_line, mv.lines[mv.cursor_row], (size_t) pos);
                    memcpy(new_line + pos, p3, (size_t) new_len);
                    memcpy(new_line + pos + new_len, mv.lines[mv.cursor_row] + pos + old_len,
                           (size_t) (line_len - pos - old_len + 1));
                    free(mv.lines[mv.cursor_row]);
                    mv.lines[mv.cursor_row] = new_line;
                    mv.modified = true;
                    if (!global_replace) {
                        break;
                    }
                    free(lower_line);
                    lower_line = term_ascii_strdown(mv.lines[mv.cursor_row]);
                    if (!lower_line) break;
                    pos += new_len;
                    match = strstr(lower_line + pos, lower_old);
                    if (match) {
                        pos = (int) (match - lower_line);
                    }
                } while (match);
            }
            free(lower_line);
            free(lower_old);
        }
        mv.mode = mv_normal;
    } else if (isdigit((unsigned char) cmd[0])) {
        int target_line = atoi(cmd) - 1;
        if (target_line < 0) {
            target_line = 0;
        }
        if (target_line >= mv.line_count) {
            target_line = mv.line_count - 1;
        }
        mv.cursor_row = target_line;
        mv.cursor_col = 0;
        mv.mode = mv_normal;
    } else {
        mv.mode = mv_normal;
    }
    mv.command_len = 0;
    mv.command_buf[0] = '\0';
}

/* ------------------------------------------------------------------ */
/* Key handling                                                         */
/* ------------------------------------------------------------------ */
static void mv_handle_normal_key(guint keyval, guint keycode, GdkModifierType state) {
    (void) keycode;
    unsigned key = keyval;
    bool ctrl = (state & GDK_CONTROL_MASK) != 0;

    if (ctrl) {
        if (key == GDK_KEY_f) {
            mv.cursor_row += mv_view_height;
            mv_clamp_cursor();
            return;
        }
        if (key == GDK_KEY_b) {
            mv.cursor_row -= mv_view_height;
            mv_clamp_cursor();
            return;
        }
        if (key == GDK_KEY_d) {
            mv.cursor_row += mv_view_height / 2;
            mv_clamp_cursor();
            return;
        }
        if (key == GDK_KEY_u) {
            mv.cursor_row -= mv_view_height / 2;
            mv_clamp_cursor();
            return;
        }
        if (key == GDK_KEY_r) {
            mv_redo_perform();
            return;
        }
        return;
    }

    if (mv.pending_op) {
        char op = mv.pending_op;
        mv.pending_op = 0;
        if (key == GDK_KEY_d && op == 'd') {
            mv_delete_line_op();
            return;
        }
        if (key == GDK_KEY_c && op == 'c') {
            mv_delete_line_op();
            mv.mode = mv_insert;
            return;
        }
        if (key == GDK_KEY_y && op == 'y') {
            mv_yank_line();
            return;
        }
        if (key == GDK_KEY_w) {
            if (op == 'd') {
                mv_delete_to_end();
                return;
            }
            if (op == 'y') {
                mv_yank_line();
                return;
            }
        }
        if (key == GDK_KEY_0 || key == GDK_KEY_Home) {
            if (op == 'd') {
                mv_delete_to_start();
                return;
            }
            if (op == 'c') {
                mv_delete_to_start();
                mv.mode = mv_insert;
                return;
            }
        }
        if (key == GDK_KEY_dollar || key == GDK_KEY_End) {
            if (op == 'd') {
                mv_delete_to_end();
                return;
            }
            if (op == 'c') {
                mv_delete_to_end();
                mv.mode = mv_insert;
                return;
            }
        }
        if (key == GDK_KEY_G) {
            if (op == 'd') {
                if (!mv_undo_push()) return;
                while (mv.line_count > mv.cursor_row + 1) {
                    mv_delete_line(mv.line_count - 1);
                }
                mv.modified = true;
            }
            return;
        }
        if (key == GDK_KEY_g && op == 'g') {
            mv.cursor_row = 0;
            mv.cursor_col = 0;
            return;
        }
        return;
    }

    switch (key) {
    case GDK_KEY_h:
    case GDK_KEY_Left:
        mv.cursor_col--;
        mv_clamp_cursor();
        break;
    case GDK_KEY_j:
    case GDK_KEY_Down:
        mv.cursor_row++;
        mv_clamp_cursor();
        break;
    case GDK_KEY_k:
    case GDK_KEY_Up:
        mv.cursor_row--;
        mv_clamp_cursor();
        break;
    case GDK_KEY_l:
    case GDK_KEY_Right:
        mv.cursor_col++;
        mv_clamp_cursor();
        break;
    case GDK_KEY_w:
        mv_word_forward();
        break;
    case GDK_KEY_b:
        mv_word_backward();
        break;
    case GDK_KEY_e:
        mv_word_end();
        break;
    case GDK_KEY_0:
    case GDK_KEY_Home:
        mv.cursor_col = 0;
        break;
    case GDK_KEY_dollar:
    case GDK_KEY_End:
        mv.cursor_col = mv_line_len(mv.cursor_row);
        break;
    case GDK_KEY_g:
        mv.pending_op = 'g';
        break;
    case GDK_KEY_G:
        mv.cursor_row = mv.line_count - 1;
        mv.cursor_col = 0;
        break;
    case GDK_KEY_braceleft: {
        mv.cursor_row--;
        while (mv.cursor_row > 0 && mv_line_len(mv.cursor_row) > 0) {
            mv.cursor_row--;
        }
        mv.cursor_col = 0;
        break;
    }
    case GDK_KEY_braceright: {
        mv.cursor_row++;
        while (mv.cursor_row < mv.line_count - 1 && mv_line_len(mv.cursor_row) > 0) {
            mv.cursor_row++;
        }
        mv.cursor_col = 0;
        break;
    }
    case GDK_KEY_i:
        if (!mv_undo_push()) break;
        mv.mode = mv_insert;
        break;
    case GDK_KEY_I:
        if (!mv_undo_push()) break;
        mv.cursor_col = 0;
        mv.mode = mv_insert;
        break;
    case GDK_KEY_a:
        if (!mv_undo_push()) break;
        mv.cursor_col++;
        mv_clamp_cursor();
        mv.mode = mv_insert;
        break;
    case GDK_KEY_A:
        if (!mv_undo_push()) break;
        mv.cursor_col = mv_line_len(mv.cursor_row);
        mv.mode = mv_insert;
        break;
    case GDK_KEY_o:
        if (!mv_undo_push()) return;
        if (!mv_insert_line(mv.cursor_row + 1, "")) break;
        mv.cursor_row++;
        mv.cursor_col = 0;
        mv.mode = mv_insert;
        mv.modified = true;
        break;
    case GDK_KEY_O:
        if (!mv_undo_push()) return;
        if (!mv_insert_line(mv.cursor_row, "")) break;
        mv.cursor_col = 0;
        mv.mode = mv_insert;
        mv.modified = true;
        break;
    case GDK_KEY_x:
        mv_delete_char_under_cursor();
        break;
    case GDK_KEY_X:
        mv_delete_char_before_cursor();
        break;
    case GDK_KEY_d:
        mv.pending_op = 'd';
        break;
    case GDK_KEY_c:
        mv.pending_op = 'c';
        break;
    case GDK_KEY_y:
        mv.pending_op = 'y';
        break;
    case GDK_KEY_p:
        mv_paste(true);
        break;
    case GDK_KEY_P:
        mv_paste(false);
        break;
    case GDK_KEY_u:
        mv_undo_perform();
        break;
    case GDK_KEY_J:
        mv_join_lines();
        break;
    case GDK_KEY_D:
        mv_delete_to_end();
        break;
    case GDK_KEY_C:
        mv_delete_to_end();
        mv.mode = mv_insert;
        break;
    case GDK_KEY_S:
        mv_delete_line_op();
        mv.mode = mv_insert;
        break;
    case GDK_KEY_asciitilde: {
        int row = mv.cursor_row;
        int col = mv.cursor_col;
        int len = mv_line_len(row);
        if (col < len) {
            if (!mv_undo_push()) return;
            char c = mv.lines[row][col];
            mv.lines[row][col] = isupper((unsigned char) c) ? tolower((unsigned char) c) : toupper((unsigned char) c);
            mv.cursor_col++;
            mv.modified = true;
            mv_clamp_cursor();
        }
        break;
    }
    case GDK_KEY_slash:
        mv.mode = mv_search;
        mv.search_forward = true;
        mv.search_len = 0;
        mv.search_buf[0] = '\0';
        break;
    case GDK_KEY_question:
        mv.mode = mv_search;
        mv.search_forward = false;
        mv.search_len = 0;
        mv.search_buf[0] = '\0';
        break;
    case GDK_KEY_n:
        mv_search_execute(mv.search_forward);
        break;
    case GDK_KEY_N:
        mv_search_execute(!mv.search_forward);
        break;
    case GDK_KEY_asterisk: {
        int row = mv.cursor_row;
        int col = mv.cursor_col;
        int len = mv_line_len(row);
        int start = col;
        while (start > 0 && mv_is_word_char(mv.lines[row][start - 1])) {
            start--;
        }
        int end = col;
        while (end < len && mv_is_word_char(mv.lines[row][end])) {
            end++;
        }
        if (end > start) {
            int wlen = end - start;
            if (wlen < 255) {
                strncpy(mv.search_buf, mv.lines[row] + start, (size_t) wlen);
                mv.search_buf[wlen] = '\0';
                mv.search_len = wlen;
                mv.search_forward = true;
                mv_search_execute(true);
            }
        }
        break;
    }
    case GDK_KEY_colon:
        mv.mode = mv_command;
        mv.command_len = 0;
        mv.command_buf[0] = '\0';
        break;
    case GDK_KEY_Escape:
        break;
    default:
        break;
    }
}

static void mv_handle_insert_key(guint keyval, guint keycode, GdkModifierType state) {
    (void) keycode;
    (void) state;
    unsigned key = keyval;
    if (key == GDK_KEY_Escape) {
        mv.mode = mv_normal;
        if (mv.cursor_col > 0) {
            mv.cursor_col--;
        }
        return;
    }
    if (key == GDK_KEY_BackSpace) {
        mv_backspace();
        return;
    }
    if (key == GDK_KEY_Delete) {
        mv_delete_char_under_cursor();
        return;
    }
    if (key == GDK_KEY_Return || key == GDK_KEY_KP_Enter) {
        mv_insert_newline();
        return;
    }
    if (key == GDK_KEY_Left) {
        mv.cursor_col--;
        mv_clamp_cursor();
        return;
    }
    if (key == GDK_KEY_Right) {
        mv.cursor_col++;
        mv_clamp_cursor();
        return;
    }
    if (key == GDK_KEY_Up) {
        mv.cursor_row--;
        mv_clamp_cursor();
        return;
    }
    if (key == GDK_KEY_Down) {
        mv.cursor_row++;
        mv_clamp_cursor();
        return;
    }
    if (key == GDK_KEY_Home) {
        mv.cursor_col = 0;
        return;
    }
    if (key == GDK_KEY_End) {
        mv.cursor_col = mv_line_len(mv.cursor_row);
        return;
    }
    if ((state & GDK_CONTROL_MASK) && (key == GDK_KEY_w)) {
        /* Delete word backward */
        if (!mv_undo_push()) return;
        int col = mv.cursor_col;
        while (col > 0 && mv.lines[mv.cursor_row][col - 1] == ' ') {
            col--;
        }
        while (col > 0 && mv_is_word_char(mv.lines[mv.cursor_row][col - 1])) {
            col--;
        }
        int deleted = mv.cursor_col - col;
        if (deleted > 0) {
            char *line = mv.lines[mv.cursor_row];
            int len = mv_line_len(mv.cursor_row);
            memmove(line + col, line + mv.cursor_col, (size_t) (len - mv.cursor_col + 1));
            mv.cursor_col = col;
            mv.modified = true;
        }
        return;
    }
    if (key >= 32 && key < 127) {
        mv_insert_char((char) key);
        return;
    }
    if (key >= GDK_KEY_space && key <= GDK_KEY_asciitilde) {
        mv_insert_char((char) key);
    }
}

static void mv_handle_command_key(guint keyval, guint keycode, GdkModifierType state) {
    (void) keycode;
    (void) state;
    unsigned key = keyval;
    if (key == GDK_KEY_Escape) {
        mv.mode = mv_normal;
        mv.command_len = 0;
        mv.command_buf[0] = '\0';
        return;
    }
    if (key == GDK_KEY_BackSpace) {
        if (mv.command_len > 0) {
            mv.command_len--;
            mv.command_buf[mv.command_len] = '\0';
        }
        return;
    }
    if (key == GDK_KEY_Return || key == GDK_KEY_KP_Enter) {
        mv_execute_command();
        return;
    }
    if (key >= 32 && key < 127 && mv.command_len < 254) {
        mv.command_buf[mv.command_len++] = (char) key;
        mv.command_buf[mv.command_len] = '\0';
    }
}

static void mv_handle_search_key(guint keyval, guint keycode, GdkModifierType state) {
    (void) keycode;
    (void) state;
    unsigned key = keyval;
    if (key == GDK_KEY_Escape) {
        mv.mode = mv_normal;
        return;
    }
    if (key == GDK_KEY_BackSpace) {
        if (mv.search_len > 0) {
            mv.search_len--;
            mv.search_buf[mv.search_len] = '\0';
        }
        return;
    }
    if (key == GDK_KEY_Return || key == GDK_KEY_KP_Enter) {
        mv_search_execute(mv.search_forward);
        mv.mode = mv_normal;
        return;
    }
    if (key >= 32 && key < 127 && mv.search_len < 254) {
        mv.search_buf[mv.search_len++] = (char) key;
        mv.search_buf[mv.search_len] = '\0';
    }
}

/* ------------------------------------------------------------------ */
/* Rendering                                                            */
/* ------------------------------------------------------------------ */
void microvim_ensure_tags(GtkTextBuffer *buffer) {
    if (!gtk_text_tag_table_lookup(gtk_text_buffer_get_tag_table(buffer), "mv_normal")) {
        gtk_text_buffer_create_tag(buffer, "mv_normal", "foreground", "#cdd6e4", NULL);
        gtk_text_buffer_create_tag(buffer, "mv_linenum", "foreground", "#5f7387", NULL);
        gtk_text_buffer_create_tag(buffer, "mv_comment", "foreground", "#6a9955", NULL);
        gtk_text_buffer_create_tag(buffer, "mv_section", "foreground", "#56b6c2", "weight", PANGO_WEIGHT_BOLD, NULL);
        gtk_text_buffer_create_tag(buffer, "mv_key", "foreground", "#e5c07b", NULL);
        gtk_text_buffer_create_tag(buffer, "mv_value", "foreground", "#cdd6e4", NULL);
        gtk_text_buffer_create_tag(buffer, "mv_tilde", "foreground", "#3a4556", NULL);
        gtk_text_buffer_create_tag(buffer, "mv_status", "foreground", "#0b111a", "background", "#cdd6e4", "weight",
                                   PANGO_WEIGHT_BOLD, NULL);
        gtk_text_buffer_create_tag(buffer, "mv_insert_label", "foreground", "#8be28b", "weight", PANGO_WEIGHT_BOLD,
                                   NULL);
    }
}

void microvim_render(GtkTextBuffer *buffer) {
    if (!buffer) {
        return;
    }
    microvim_ensure_tags(buffer);
    gtk_text_buffer_set_text(buffer, "", -1);

    /* Adjust scroll */
    if (mv.cursor_row < mv.scroll_offset) {
        mv.scroll_offset = mv.cursor_row;
    }
    if (mv.cursor_row >= mv.scroll_offset + mv_view_height - 2) {
        mv.scroll_offset = mv.cursor_row - mv_view_height + 3;
    }
    if (mv.scroll_offset < 0) {
        mv.scroll_offset = 0;
    }

    GtkTextIter end_iter;
    char line_buf[mv_max_line_len + 64];

    for (int view_line = 0; view_line < mv_view_height - 2; view_line++) {
        int row = mv.scroll_offset + view_line;
        if (row < mv.line_count) {
            if (mv.show_line_numbers) {
                snprintf(line_buf, sizeof(line_buf), "%4d ", row + 1);
                gtk_text_buffer_get_end_iter(buffer, &end_iter);
                gtk_text_buffer_insert_with_tags_by_name(buffer, &end_iter, line_buf, -1, "mv_linenum", NULL);
            }
            const char *line = mv.lines[row];
            const char *tag = "mv_normal";
            if (line[0] == '#') {
                tag = "mv_comment";
            } else if (line[0] == '[') {
                tag = "mv_section";
            } else {
                char *eq = strchr(line, '=');
                if (eq && (eq > line)) {
                    int key_len = (int) (eq - line);
                    gtk_text_buffer_get_end_iter(buffer, &end_iter);
                    gtk_text_buffer_insert_with_tags_by_name(buffer, &end_iter, line, key_len, "mv_key", NULL);
                    gtk_text_buffer_get_end_iter(buffer, &end_iter);
                    gtk_text_buffer_insert_with_tags_by_name(buffer, &end_iter, eq, -1, "mv_value", NULL);
                    line = NULL;
                }
            }
            if (line) {
                gtk_text_buffer_get_end_iter(buffer, &end_iter);
                gtk_text_buffer_insert_with_tags_by_name(buffer, &end_iter, line, -1, tag, NULL);
            }
        } else {
            if (mv.show_line_numbers) {
                gtk_text_buffer_get_end_iter(buffer, &end_iter);
                gtk_text_buffer_insert_with_tags_by_name(buffer, &end_iter, "     ", -1, "mv_linenum", NULL);
            }
            gtk_text_buffer_get_end_iter(buffer, &end_iter);
            gtk_text_buffer_insert_with_tags_by_name(buffer, &end_iter, "~", -1, "mv_tilde", NULL);
        }
        gtk_text_buffer_get_end_iter(buffer, &end_iter);
        gtk_text_buffer_insert(buffer, &end_iter, "\n", -1);
    }

    /* Status bar */
    char status_buf[1024];
    const char *mode_text = "";
    if (mv.mode == mv_insert) {
        mode_text = "-- INSERT -- ";
    } else if (mv.mode == mv_command) {
        mode_text = ":";
    } else if (mv.mode == mv_search) {
        mode_text = mv.search_forward ? "/" : "?";
    }

    if (mv.mode == mv_command) {
        snprintf(status_buf, sizeof(status_buf), ":%s", mv.command_buf);
    } else if (mv.mode == mv_search) {
        snprintf(status_buf, sizeof(status_buf), "%s%s", mv.search_forward ? "/" : "?", mv.search_buf);
    } else {
        int pct = (mv.line_count > 1) ? ((mv.cursor_row + 1) * 100 / mv.line_count) : 100;
        const char *pos_text = (mv.cursor_row == 0) ? "Top" : (mv.cursor_row == mv.line_count - 1) ? "Bot" : "";
        /* MPE_TASK_V15R2_STATUS_NEW_FILE_BEGIN */
        const char *file_status_text = "";
        if (!mv.file_exists && !mv.modified) {
            file_status_text = " [New]";
        } else if (mv.modified) {
            file_status_text = " [Modified]";
        }
        snprintf(status_buf, sizeof(status_buf), "%s\"%s\" %dL%s%s%s%d,%d %s %d%%", mode_text, mv.filename,
                 mv.line_count, file_status_text, mode_text[0] ? "" : "   ", pos_text[0] ? "" : "", mv.cursor_row + 1,
                 mv.cursor_col + 1, pos_text[0] ? pos_text : "", pct);
        /* MPE_TASK_V15R2_STATUS_NEW_FILE_END */
    }
    gtk_text_buffer_get_end_iter(buffer, &end_iter);
    gtk_text_buffer_insert_with_tags_by_name(buffer, &end_iter, status_buf, -1,
                                             (mv.mode == mv_insert) ? "mv_insert_label" : "mv_status", NULL);

    /* Place cursor */
    GtkTextIter cursor_iter;
    int target_line = mv.cursor_row - mv.scroll_offset;
    if (target_line < 0) {
        target_line = 0;
    }
    if (target_line >= mv_view_height - 2) {
        target_line = mv_view_height - 3;
    }
    int col_offset = mv.cursor_col + (mv.show_line_numbers ? 5 : 0);
    gtk_text_buffer_get_iter_at_line_offset(buffer, &cursor_iter, target_line, col_offset);
    gtk_text_buffer_place_cursor(buffer, &cursor_iter);
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */
mv_mode microvim_get_mode(void) {
    return mv.mode;
}
bool microvim_is_active(void) {
    return mv.active;
}

void microvim_open(const char *filename) {
    microvim_close();
    memset(&mv, 0, sizeof(mv));
    if (!filename) return;
    mv.lines = NULL;
    mv.line_capacity = 0;
    mv.line_count = 0;
    strncpy(mv.filename, filename, sizeof(mv.filename) - 1);
    mv.filename[sizeof(mv.filename) - 1] = '\0';
    mv.file_exists = mv_load_file(filename); /* MPE_TASK_V15R2_FILE_EXISTS_STORE */
    if (mv.line_count == 0 && !mv_insert_line(0, "")) {
        microvim_close();
        return;
    }
    mv.cursor_row = 0;
    mv.cursor_col = 0;
    mv.mode = mv_normal;
    mv.modified = false;
    mv.active = true;
    mv.quit_requested = false;
    mv.show_line_numbers = true;
    mv.scroll_offset = 0;
    mv.undo_top = 0;
    mv.redo_top = 0;
    mv.yank_text = NULL;
    mv.pending_op = 0;
    mv.command_len = 0;
    mv.search_len = 0;
    /* Default to showing line numbers for config files */
    const char *ext = strrchr(filename, '.');
    if (ext && (strcmp(ext, ".cfg") == 0 || strcmp(ext, ".ini") == 0)) {
        mv.show_line_numbers = true;
    }
}

void microvim_close(void) {
    mv_clear_lines();
    if (mv.lines) {
        free(mv.lines);
        mv.lines = NULL;
    }
    if (mv.yank_text) {
        free(mv.yank_text);
        mv.yank_text = NULL;
    }
    for (int i = 0; i < mv_undo_depth; i++) {
        mv_snapshot *s = &mv.undo_stack[i];
        if (s->lines) {
            for (int j = 0; j < s->line_count; j++) {
                free(s->lines[j]);
            }
            free(s->lines);
            s->lines = NULL;
        }
    }
    mv.active = false;
    mv.quit_requested = false;
}

void microvim_handle_key(guint keyval, guint keycode, GdkModifierType state) {
    if (!mv.active) {
        return;
    }
    switch (mv.mode) {
    case mv_normal:
        mv_handle_normal_key(keyval, keycode, state);
        break;
    case mv_insert:
        mv_handle_insert_key(keyval, keycode, state);
        break;
    case mv_command:
        mv_handle_command_key(keyval, keycode, state);
        break;
    case mv_search:
        mv_handle_search_key(keyval, keycode, state);
        break;
    }
    if (mv.quit_requested) {
        microvim_close();
    }
}
/* MPE_TASK_V15R2_MICROVIM_IMPL_END */


#else
/* MPE_TASK_V15R2_MICROVIM_IMPL_BEGIN */
#include "../mpe_engine.h"
#include "microvim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <stdint.h>
#include <ctype.h>
#include <gdk/gdkkeysyms.h>

#define mv_view_height 40
#define mv_undo_depth 64
#define mv_max_line_len 4096

/* ------------------------------------------------------------------ */
/* State                                                                */
/* ------------------------------------------------------------------ */
typedef struct {
    char **lines;
    int line_count;
    int line_capacity;
} mv_snapshot;

static struct {
    char **lines;
    int line_count;
    int line_capacity;
    int cursor_row;
    int cursor_col;
    int preferred_col;
    mv_mode mode;
    char pending_op;
    char command_buf[256];
    int command_len;
    char search_buf[256];
    int search_len;
    bool search_forward;
    bool modified;
    char filename[512];
    bool show_line_numbers;
    char *yank_text;
    bool yank_is_linewise;
    int scroll_offset;
    bool active;
    bool quit_requested;
    bool file_exists; /* MPE_TASK_V15R2_FILE_EXISTS_FLAG */
    mv_snapshot undo_stack[mv_undo_depth];
    int undo_top;
    int redo_top;
} mv;

/* ------------------------------------------------------------------ */
/* Line buffer helpers                                                  */
/* ------------------------------------------------------------------ */
static void mv_clear_lines(void) {
    for (int i = 0; i < mv.line_count; i++) {
        if (mv.lines[i]) {
            free(mv.lines[i]);
            mv.lines[i] = NULL;
        }
    }
    mv.line_count = 0;
}

static bool mv_ensure_capacity(int needed) {
    if (needed < 0 || (size_t)needed > SIZE_MAX / sizeof(*mv.lines)) return false;
    if (needed <= mv.line_capacity) {
        return true;
    }
    int new_cap = mv.line_capacity < 64 ? 64
                  : mv.line_capacity <= INT_MAX / 2 ? mv.line_capacity * 2
                  : needed;
    if (new_cap < needed) {
        new_cap = needed;
    }
    if ((size_t)new_cap > SIZE_MAX / sizeof(*mv.lines)) return false;
    char **new_lines = (char **)realloc(mv.lines, (size_t)new_cap * sizeof(*mv.lines));
    if (!new_lines) return false;
    for (int i = mv.line_capacity; i < new_cap; i++) {
        new_lines[i] = NULL;
    }
    mv.lines = new_lines;
    mv.line_capacity = new_cap;
    return true;
}

static bool mv_set_line(int index, const char *text) {
    if (index < 0) return false;
    char *replacement = text && strlen(text) >= (size_t)mv_max_line_len
                            ? term_strndup(text, mv_max_line_len - 1)
                            : term_strdup(text ? text : "");
    if (!replacement || !mv_ensure_capacity(index + 1)) {
        free(replacement);
        return false;
    }
    if (mv.lines[index]) {
        free(mv.lines[index]);
    }
    mv.lines[index] = replacement;
    return true;
}

static bool mv_insert_line(int index, const char *text) {
    if (index < 0 || index > mv.line_count || mv.line_count == INT_MAX) return false;
    char *replacement = text && strlen(text) >= (size_t)mv_max_line_len
                            ? term_strndup(text, mv_max_line_len - 1)
                            : term_strdup(text ? text : "");
    if (!replacement || !mv_ensure_capacity(mv.line_count + 1)) {
        free(replacement);
        return false;
    }
    for (int i = mv.line_count; i > index; i--) {
        mv.lines[i] = mv.lines[i - 1];
    }
    mv.lines[index] = replacement;
    mv.line_count++;
    return true;
}

static void mv_delete_line(int index) {
    if ((index < 0) || (index >= mv.line_count)) {
        return;
    }
    free(mv.lines[index]);
    for (int i = index; i < mv.line_count - 1; i++) {
        mv.lines[i] = mv.lines[i + 1];
    }
    mv.lines[mv.line_count - 1] = NULL;
    mv.line_count--;
    if (mv.line_count == 0) {
        if (!mv_insert_line(0, "")) microvim_close();
    }
}

static int mv_line_len(int row) {
    if ((row < 0) || (row >= mv.line_count) || !mv.lines || !mv.lines[row]) {
        return 0;
    }
    return (int) strlen(mv.lines[row]);
}

static void mv_clamp_cursor(void) {
    if (mv.line_count <= 0) {
        mv.cursor_row = 0;
        mv.cursor_col = 0;
        return;
    }
    if (mv.cursor_row < 0) {
        mv.cursor_row = 0;
    }
    if (mv.cursor_row >= mv.line_count) {
        mv.cursor_row = mv.line_count - 1;
    }
    int len = mv_line_len(mv.cursor_row);
    if (mv.cursor_col > len) {
        mv.cursor_col = len;
    }
    if (mv.cursor_col < 0) {
        mv.cursor_col = 0;
    }
}

/* ------------------------------------------------------------------ */
/* Undo                                                                 */
/* ------------------------------------------------------------------ */
static void mv_snapshot_clear(mv_snapshot *snap) {
    if (!snap) return;
    if (snap->lines) {
        for (int i = 0; i < snap->line_count; i++) free(snap->lines[i]);
        free(snap->lines);
    }
    *snap = (mv_snapshot){0};
}

static void mv_clear_undo_history(void) {
    for (int i = 0; i < mv_undo_depth; i++) mv_snapshot_clear(&mv.undo_stack[i]);
    mv.undo_top = 0;
    mv.redo_top = 0;
}

static bool mv_undo_push(void) {
    if (mv.line_count <= 0 || !mv.lines) return false;
    int snapshot_count = mv.line_count;
    char **snapshot_lines = (char **)calloc((size_t)snapshot_count, sizeof(*snapshot_lines));
    if (!snapshot_lines) return false;
    for (int i = 0; i < snapshot_count; i++) {
        if (!mv.lines[i] || !(snapshot_lines[i] = term_strdup(mv.lines[i]))) {
            for (int j = 0; j < i; j++) free(snapshot_lines[j]);
            free(snapshot_lines);
            return false;
        }
    }
    for (int i = mv.undo_top; i < mv.redo_top && i < mv_undo_depth; i++)
        mv_snapshot_clear(&mv.undo_stack[i]);
    mv.redo_top = mv.undo_top;
    if (mv.undo_top >= mv_undo_depth) {
        mv_snapshot_clear(&mv.undo_stack[0]);
        for (int i = 1; i < mv_undo_depth; i++) mv.undo_stack[i - 1] = mv.undo_stack[i];
        mv.undo_stack[mv_undo_depth - 1] = (mv_snapshot){0};
        mv.undo_top = mv_undo_depth - 1;
        mv.redo_top = mv.undo_top;
    }
    mv.undo_stack[mv.undo_top].lines = snapshot_lines;
    mv.undo_stack[mv.undo_top].line_count = snapshot_count;
    mv.undo_stack[mv.undo_top].line_capacity = snapshot_count;
    mv.undo_top++;
    mv.redo_top = mv.undo_top;
    return true;
}

static bool mv_restore_snapshot(const mv_snapshot *snap) {
    if (!snap || snap->line_count <= 0 || !snap->lines) return false;
    char **restored = (char **)calloc((size_t)snap->line_count, sizeof(*restored));
    if (!restored) return false;
    for (int i = 0; i < snap->line_count; i++) {
        if (!snap->lines[i] || !(restored[i] = term_strdup(snap->lines[i]))) {
            for (int j = 0; j < i; j++) free(restored[j]);
            free(restored);
            return false;
        }
    }
    if (!mv_ensure_capacity(snap->line_count)) {
        for (int i = 0; i < snap->line_count; i++) free(restored[i]);
        free(restored);
        return false;
    }
    mv_clear_lines();
    memcpy(mv.lines, restored, (size_t)snap->line_count * sizeof(*restored));
    mv.line_count = snap->line_count;
    free(restored);
    return true;
}

static void mv_undo_perform(void) {
    if (mv.undo_top <= 0) {
        return;
    }
    mv_snapshot *snap = &mv.undo_stack[mv.undo_top - 1];
    if (!mv_restore_snapshot(snap)) return;
    mv.undo_top--;
    mv.modified = true;
    mv_clamp_cursor();
}

static void mv_redo_perform(void) {
    if (mv.undo_top >= mv.redo_top) {
        return;
    }
    mv_snapshot *snap = &mv.undo_stack[mv.undo_top];
    if (!mv_restore_snapshot(snap)) return;
    mv.undo_top++;
    mv.modified = true;
    mv_clamp_cursor();
}

/* ------------------------------------------------------------------ */
/* File I/O                                                             */
/* ------------------------------------------------------------------ */
static bool mv_load_file(const char *filename) {
    mv_clear_lines();
    if (!filename || !mv_ensure_capacity(64)) return false;
    FILE *f = fopen(filename, "r");
    if (!f) {
        if (!mv_insert_line(0, "")) return false;
        return false;
    }
    char line_buf[mv_max_line_len];
    while (fgets(line_buf, sizeof(line_buf), f)) {
        size_t len = strlen(line_buf);
        while ((len > 0) && ((line_buf[len - 1] == '\n') || (line_buf[len - 1] == '\r'))) {
            line_buf[--len] = '\0';
        }
        if (!mv_insert_line(mv.line_count, line_buf)) {
            fclose(f);
            mv_clear_lines();
            return false;
        }
    }
    fclose(f);
    if (mv.line_count == 0) {
        if (!mv_insert_line(0, "")) return false;
    }
    return true;
}

static bool mv_save_file(void) {
    char backup_path[560];
    snprintf(backup_path, sizeof(backup_path), "%s.bak", mv.filename);
    FILE *existing = fopen(mv.filename, "r");
    if (existing) {
        fclose(existing);
        FILE *src = fopen(mv.filename, "r");
        FILE *dst = fopen(backup_path, "w");
        if (src && dst) {
            char buf[4096];
            size_t n;
            while ((n = fread(buf, 1, sizeof(buf), src)) > 0) {
                fwrite(buf, 1, n, dst);
            }
        }
        if (src) {
            fclose(src);
        }
        if (dst) {
            fclose(dst);
        }
    }
    FILE *f = fopen(mv.filename, "w");
    if (!f) {
        return false;
    }
    for (int i = 0; i < mv.line_count; i++) {
        fprintf(f, "%s\n", mv.lines[i]);
    }
    fclose(f);
    mv.modified = false;
    return true;
}

/* ------------------------------------------------------------------ */
/* Word motion                                                          */
/* ------------------------------------------------------------------ */
static bool mv_is_word_char(char c) {
    return isalnum((unsigned char) c) || (c == '_');
}

static void mv_word_forward(void) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    int len = mv_line_len(row);
    while (col < len && mv_is_word_char(mv.lines[row][col])) {
        col++;
    }
    while (col < len && !mv_is_word_char(mv.lines[row][col]) && mv.lines[row][col] != ' ') {
        col++;
    }
    while (col < len && mv.lines[row][col] == ' ') {
        col++;
    }
    if (col >= len) {
        if (row + 1 < mv.line_count) {
            row++;
            col = 0;
        }
    }
    mv.cursor_row = row;
    mv.cursor_col = col;
    mv_clamp_cursor();
}

static void mv_word_backward(void) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    if (col > 0) {
        col--;
    }
    while (col > 0 && mv.lines[row][col] == ' ') {
        col--;
    }
    if (col > 0 && mv_is_word_char(mv.lines[row][col])) {
        while (col > 0 && mv_is_word_char(mv.lines[row][col - 1])) {
            col--;
        }
    } else if (col > 0) {
        while (col > 0 && !mv_is_word_char(mv.lines[row][col - 1]) && mv.lines[row][col - 1] != ' ') {
            col--;
        }
    }
    mv.cursor_row = row;
    mv.cursor_col = col;
    mv_clamp_cursor();
}

static void mv_word_end(void) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    int len = mv_line_len(row);
    if (col < len) {
        col++;
    }
    while (col < len && mv.lines[row][col] == ' ') {
        col++;
    }
    if (col < len && mv_is_word_char(mv.lines[row][col])) {
        while (col < len && mv_is_word_char(mv.lines[row][col])) {
            col++;
        }
    } else {
        while (col < len && !mv_is_word_char(mv.lines[row][col]) && mv.lines[row][col] != ' ') {
            col++;
        }
    }
    if (col > 0) {
        col--;
    }
    mv.cursor_row = row;
    mv.cursor_col = col;
    mv_clamp_cursor();
}

/* ------------------------------------------------------------------ */
/* Search                                                               */
/* ------------------------------------------------------------------ */
static void mv_search_execute(bool forward) {
    if (mv.search_len == 0 || mv.line_count <= 0) {
        return;
    }
    /* Lower pattern once instead of per-line per-attempt (old code allocated
     * two strings per line scanned). */
    char *pattern_lower = term_ascii_strdown(mv.search_buf);
    if (!pattern_lower) {
        return;
    }
    size_t pat_len = strlen(pattern_lower);
    int start_row = mv.cursor_row;
    for (int attempt = 0; attempt < mv.line_count * 2; attempt++) {
        int row = (forward ? (start_row + attempt) : (start_row - attempt + mv.line_count * 2)) % mv.line_count;
        const char *line = mv.lines[row];
        if (!line) {
            continue;
        }
        /* Case-insensitive substring search without per-line allocation. */
        const char *match = NULL;
        size_t line_len = strlen(line);
        if (pat_len <= line_len) {
            for (size_t i = 0; i + pat_len <= line_len; i++) {
                bool eq = true;
                for (size_t k = 0; k < pat_len; k++) {
                    if (term_ascii_tolower(line[i + k]) != pattern_lower[k]) {
                        eq = false;
                        break;
                    }
                }
                if (eq) {
                    match = line + i;
                    /* For forward/backward skip semantics we need the first
                     * valid match per row; for simplicity take the first and
                     * apply cursor gating below. A second scan finds later
                     * matches on the start row if the first is behind cursor. */
                    int col0 = (int) (match - line);
                    if ((row == start_row) && forward && (col0 <= mv.cursor_col)) {
                        /* Look for a later match on the same row. */
                        const char *later = NULL;
                        for (size_t j = (size_t)(mv.cursor_col + 1); j + pat_len <= line_len; j++) {
                            bool eq2 = true;
                            for (size_t k = 0; k < pat_len; k++) {
                                if (term_ascii_tolower(line[j + k]) != pattern_lower[k]) {
                                    eq2 = false;
                                    break;
                                }
                            }
                            if (eq2) {
                                later = line + j;
                                break;
                            }
                        }
                        if (later) {
                            match = later;
                        } else {
                            continue;
                        }
                    }
                    if ((row == start_row) && !forward && ((int) (match - line) >= mv.cursor_col)) {
                        continue;
                    }
                    mv.cursor_row = row;
                    mv.cursor_col = (int) (match - line);
                    free(pattern_lower);
                    return;
                }
            }
        }
    }
    free(pattern_lower);
}

/* ------------------------------------------------------------------ */
/* Editing operations                                                   */
/* ------------------------------------------------------------------ */
static void mv_delete_char_under_cursor(void) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    int len = mv_line_len(row);
    if (col >= len) {
        return;
    }
    if (!mv_undo_push()) return;
    char *line = mv.lines[row];
    memmove(line + col, line + col + 1, (size_t) (len - col));
    mv.modified = true;
    mv_clamp_cursor();
}

static void mv_delete_char_before_cursor(void) {
    int col = mv.cursor_col;
    if (col <= 0) {
        return;
    }
    if (!mv_undo_push()) return;
    char *line = mv.lines[mv.cursor_row];
    int len = mv_line_len(mv.cursor_row);
    memmove(line + col - 1, line + col, (size_t) (len - col + 1));
    mv.cursor_col--;
    mv.modified = true;
}

static void mv_delete_line_op(void) {
    if (!mv_undo_push()) return;
    if (mv.yank_text) {
        free(mv.yank_text);
    }
    mv.yank_text = term_strdup(mv.lines[mv.cursor_row]);
    mv.yank_is_linewise = true;
    mv_delete_line(mv.cursor_row);
    mv.modified = true;
    mv_clamp_cursor();
}

static void mv_delete_to_end(void) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    int len = mv_line_len(row);
    if (col >= len) {
        return;
    }
    if (!mv_undo_push()) return;
    if (mv.yank_text) {
        free(mv.yank_text);
    }
    mv.yank_text = term_strdup(mv.lines[row] + col);
    mv.yank_is_linewise = false;
    mv.lines[row][col] = '\0';
    mv.modified = true;
    mv_clamp_cursor();
}

static void mv_delete_to_start(void) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    if (col <= 0) {
        return;
    }
    if (!mv_undo_push()) return;
    if (mv.yank_text) {
        free(mv.yank_text);
    }
    mv.yank_text = term_strndup(mv.lines[row], col);
    mv.yank_is_linewise = false;
    char *new_line = term_strdup(mv.lines[row] + col);
    free(mv.lines[row]);
    mv.lines[row] = new_line;
    mv.cursor_col = 0;
    mv.modified = true;
}

static void mv_yank_line(void) {
    if (mv.yank_text) {
        free(mv.yank_text);
    }
    mv.yank_text = term_strdup(mv.lines[mv.cursor_row]);
    mv.yank_is_linewise = true;
}

static void mv_paste(bool after) {
    if (!mv.yank_text) {
        return;
    }
    if (!mv_undo_push()) return;
    if (mv.yank_is_linewise) {
        int insert_row = after ? mv.cursor_row + 1 : mv.cursor_row;
        if (!mv_insert_line(insert_row, mv.yank_text)) return;
        mv.cursor_row = insert_row;
        mv.cursor_col = 0;
    } else {
        int row = mv.cursor_row;
        int col = after ? mv.cursor_col + 1 : mv.cursor_col;
        int len = mv_line_len(row);
        if (col > len) {
            col = len;
        }
        int yank_len = (int) strlen(mv.yank_text);
        char *new_line = (char *) malloc((size_t) (len + yank_len + 1));
        memcpy(new_line, mv.lines[row], (size_t) col);
        memcpy(new_line + col, mv.yank_text, (size_t) yank_len);
        memcpy(new_line + col + yank_len, mv.lines[row] + col, (size_t) (len - col + 1));
        free(mv.lines[row]);
        mv.lines[row] = new_line;
        mv.cursor_col = col + yank_len - 1;
    }
    mv.modified = true;
    mv_clamp_cursor();
}

static void mv_join_lines(void) {
    if (mv.cursor_row >= mv.line_count - 1) {
        return;
    }
    if (!mv_undo_push()) return;
    int len_a = mv_line_len(mv.cursor_row);
    char *joined = term_format("%s %s", mv.lines[mv.cursor_row], mv.lines[mv.cursor_row + 1]);
    if (!mv_set_line(mv.cursor_row, joined)) {
        free(joined);
        return;
    }
    free(joined);
    mv_delete_line(mv.cursor_row + 1);
    mv.cursor_col = len_a;
    mv.modified = true;
    mv_clamp_cursor();
}

static void mv_insert_char(char c) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    int len = mv_line_len(row);
    if (len >= mv_max_line_len - 1) {
        return;
    } /* FIX_039: prevent overflow */
    char *line = mv.lines[row];
    char *new_line = (char *) malloc((size_t) (len + 2));
    if (!new_line) return;
    memcpy(new_line, line, (size_t) col);
    new_line[col] = c;
    memcpy(new_line + col + 1, line + col, (size_t) (len - col + 1));
    free(mv.lines[row]);
    mv.lines[row] = new_line;
    mv.cursor_col++;
    mv.modified = true;
}

static void mv_insert_newline(void) {
    int row = mv.cursor_row;
    int col = mv.cursor_col;
    char *second_half = term_strdup(mv.lines[row] + col);
    if (!second_half || !mv_insert_line(row + 1, second_half)) {
        free(second_half);
        return;
    }
    mv.lines[row][col] = '\0';
    free(second_half);
    mv.cursor_row++;
    mv.cursor_col = 0;
    mv.modified = true;
}

static void mv_backspace(void) {
    if (mv.cursor_col > 0) {
        mv_delete_char_before_cursor();
    } else if (mv.cursor_row > 0) {
        if (!mv_undo_push()) return;
        int prev_len = mv_line_len(mv.cursor_row - 1);
        char *joined = term_format("%s%s", mv.lines[mv.cursor_row - 1], mv.lines[mv.cursor_row]);
        if (!mv_set_line(mv.cursor_row - 1, joined)) {
            free(joined);
            return;
        }
        free(joined);
        mv_delete_line(mv.cursor_row);
        mv.cursor_row--;
        mv.cursor_col = prev_len;
        mv.modified = true;
    }
}

/* ------------------------------------------------------------------ */
/* Command mode execution                                               */
/* ------------------------------------------------------------------ */
static void mv_execute_command(void) {
    char *cmd = mv.command_buf;
    if (cmd[0] == 'w' && cmd[1] == 'q') {
        mv_save_file();
        mv.file_exists = true; /* MPE_TASK_V15R2_WQ_SETS_EXISTS */
        mv.quit_requested = true;
    } else if (cmd[0] == 'w' && cmd[1] == '\0') {
        mv_save_file();
        mv.file_exists = true; /* MPE_TASK_V15R2_SAVE_SETS_EXISTS */
        /* Config integration: reload if editing engine.cfg */
        if (strstr(mv.filename, "engine.cfg")) {
            mpe_config_load("status/engine.cfg");
            contact_cache_clear(physics_world_get_primary());
        }
        mv.mode = mv_normal;
    } else if (cmd[0] == 'q' && cmd[1] == '!') {
        mv.quit_requested = true;
    } else if (cmd[0] == 'q' && cmd[1] == '\0') {
        if (mv.modified) {
            /* MPE_TASK_V15R2_QUIT_MODIFIED_ERROR_BEGIN */
            mv.mode = mv_normal;
            mv.command_len = 0;
            mv.command_buf[0] = '\0';
            /* Render an error message in the status area by temporarily
setting a flag that microvim_render will pick up */
            mv.command_len =
                snprintf(mv.command_buf, sizeof(mv.command_buf), "E37: No write since last change (use :q! or :wq)");
            mv.mode = mv_command; /* Show error in command line area */
            return;
            /* MPE_TASK_V15R2_QUIT_MODIFIED_ERROR_END */
        }
        mv.quit_requested = true;
    } else if (cmd[0] == 'x') {
        mv_save_file();
        mv.quit_requested = true;
    } else if (cmd[0] == 'e' && cmd[1] == ' ') {
        if (!mv.modified || (cmd[2] == '!')) {
            mv_load_file(cmd + 2);
            if (mv.line_count == 0) {
                microvim_close();
                return;
            }
            mv.cursor_row = 0;
            mv.cursor_col = 0;
            mv.modified = false;
            mv_clear_undo_history();
        }
        mv.mode = mv_normal;
    } else if (strncmp(cmd, "set number", 10) == 0 || strncmp(cmd, "set nu", 6) == 0) {
        mv.show_line_numbers = true;
        mv.mode = mv_normal;
    } else if (strncmp(cmd, "set nonumber", 12) == 0 || strncmp(cmd, "set nonu", 8) == 0) {
        mv.show_line_numbers = false;
        mv.mode = mv_normal;
    } else if (cmd[0] == 's' && cmd[1] == '/') {
        /* Substitute on current line: s/old/new/ or s/old/new/g */
        char *p1 = cmd + 2;
        char *p2 = strchr(p1, '/');
        if (p2) {
            *p2 = '\0';
            char *p3 = p2 + 1;
            char *p4 = strchr(p3, '/');
            bool global_replace = false;
            if (p4) {
                *p4 = '\0';
                if (*(p4 + 1) == 'g') {
                    global_replace = true;
                }
            }
            if (!p1[0]) {
                mv.command_len = snprintf(mv.command_buf, sizeof(mv.command_buf),
                                          "E486: empty substitute pattern");
                return;
            }
            char *line = mv.lines[mv.cursor_row];
            char *lower_line = term_ascii_strdown(line);
            char *lower_old = term_ascii_strdown(p1);
            if (!lower_line || !lower_old) {
                free(lower_line);
                free(lower_old);
                return;
            }
            char *match = strstr(lower_line, lower_old);
            if (match) {
                if (!mv_undo_push()) {
                    free(lower_line);
                    free(lower_old);
                    return;
                }
                int old_len = (int) strlen(p1);
                int new_len = (int) strlen(p3);
                int pos = (int) (match - lower_line);
                do {
                    int line_len = (int) strlen(mv.lines[mv.cursor_row]);
                    if (new_len > mv_max_line_len - 1 - (line_len - old_len)) break;
                    char *new_line = (char *) malloc((size_t) (line_len - old_len + new_len + 1));
                    if (!new_line) break;
                    memcpy(new_line, mv.lines[mv.cursor_row], (size_t) pos);
                    memcpy(new_line + pos, p3, (size_t) new_len);
                    memcpy(new_line + pos + new_len, mv.lines[mv.cursor_row] + pos + old_len,
                           (size_t) (line_len - pos - old_len + 1));
                    free(mv.lines[mv.cursor_row]);
                    mv.lines[mv.cursor_row] = new_line;
                    mv.modified = true;
                    if (!global_replace) {
                        break;
                    }
                    free(lower_line);
                    lower_line = term_ascii_strdown(mv.lines[mv.cursor_row]);
                    if (!lower_line) break;
                    pos += new_len;
                    match = strstr(lower_line + pos, lower_old);
                    if (match) {
                        pos = (int) (match - lower_line);
                    }
                } while (match);
            }
            free(lower_line);
            free(lower_old);
        }
        mv.mode = mv_normal;
    } else if (isdigit((unsigned char) cmd[0])) {
        int target_line = atoi(cmd) - 1;
        if (target_line < 0) {
            target_line = 0;
        }
        if (target_line >= mv.line_count) {
            target_line = mv.line_count - 1;
        }
        mv.cursor_row = target_line;
        mv.cursor_col = 0;
        mv.mode = mv_normal;
    } else {
        mv.mode = mv_normal;
    }
    mv.command_len = 0;
    mv.command_buf[0] = '\0';
}

/* ------------------------------------------------------------------ */
/* Key handling                                                         */
/* ------------------------------------------------------------------ */
static void mv_handle_normal_key(GdkEventKey *event) {
    unsigned int key = event->keyval;
    bool ctrl = (event->state & GDK_CONTROL_MASK) != 0;

    if (ctrl) {
        if (key == GDK_KEY_f) {
            mv.cursor_row += mv_view_height;
            mv_clamp_cursor();
            return;
        }
        if (key == GDK_KEY_b) {
            mv.cursor_row -= mv_view_height;
            mv_clamp_cursor();
            return;
        }
        if (key == GDK_KEY_d) {
            mv.cursor_row += mv_view_height / 2;
            mv_clamp_cursor();
            return;
        }
        if (key == GDK_KEY_u) {
            mv.cursor_row -= mv_view_height / 2;
            mv_clamp_cursor();
            return;
        }
        if (key == GDK_KEY_r) {
            mv_redo_perform();
            return;
        }
        return;
    }

    if (mv.pending_op) {
        char op = mv.pending_op;
        mv.pending_op = 0;
        if (key == GDK_KEY_d && op == 'd') {
            mv_delete_line_op();
            return;
        }
        if (key == GDK_KEY_c && op == 'c') {
            mv_delete_line_op();
            mv.mode = mv_insert;
            return;
        }
        if (key == GDK_KEY_y && op == 'y') {
            mv_yank_line();
            return;
        }
        if (key == GDK_KEY_w) {
            if (op == 'd') {
                mv_delete_to_end();
                return;
            }
            if (op == 'y') {
                mv_yank_line();
                return;
            }
        }
        if (key == GDK_KEY_0 || key == GDK_KEY_Home) {
            if (op == 'd') {
                mv_delete_to_start();
                return;
            }
            if (op == 'c') {
                mv_delete_to_start();
                mv.mode = mv_insert;
                return;
            }
        }
        if (key == GDK_KEY_dollar || key == GDK_KEY_End) {
            if (op == 'd') {
                mv_delete_to_end();
                return;
            }
            if (op == 'c') {
                mv_delete_to_end();
                mv.mode = mv_insert;
                return;
            }
        }
        if (key == GDK_KEY_G) {
            if (op == 'd') {
                if (!mv_undo_push()) return;
                while (mv.line_count > mv.cursor_row + 1) {
                    mv_delete_line(mv.line_count - 1);
                }
                mv.modified = true;
            }
            return;
        }
        if (key == GDK_KEY_g && op == 'g') {
            mv.cursor_row = 0;
            mv.cursor_col = 0;
            return;
        }
        return;
    }

    switch (key) {
    case GDK_KEY_h:
    case GDK_KEY_Left:
        mv.cursor_col--;
        mv_clamp_cursor();
        break;
    case GDK_KEY_j:
    case GDK_KEY_Down:
        mv.cursor_row++;
        mv_clamp_cursor();
        break;
    case GDK_KEY_k:
    case GDK_KEY_Up:
        mv.cursor_row--;
        mv_clamp_cursor();
        break;
    case GDK_KEY_l:
    case GDK_KEY_Right:
        mv.cursor_col++;
        mv_clamp_cursor();
        break;
    case GDK_KEY_w:
        mv_word_forward();
        break;
    case GDK_KEY_b:
        mv_word_backward();
        break;
    case GDK_KEY_e:
        mv_word_end();
        break;
    case GDK_KEY_0:
    case GDK_KEY_Home:
        mv.cursor_col = 0;
        break;
    case GDK_KEY_dollar:
    case GDK_KEY_End:
        mv.cursor_col = mv_line_len(mv.cursor_row);
        break;
    case GDK_KEY_g:
        mv.pending_op = 'g';
        break;
    case GDK_KEY_G:
        mv.cursor_row = mv.line_count - 1;
        mv.cursor_col = 0;
        break;
    case GDK_KEY_braceleft: {
        mv.cursor_row--;
        while (mv.cursor_row > 0 && mv_line_len(mv.cursor_row) > 0) {
            mv.cursor_row--;
        }
        mv.cursor_col = 0;
        break;
    }
    case GDK_KEY_braceright: {
        mv.cursor_row++;
        while (mv.cursor_row < mv.line_count - 1 && mv_line_len(mv.cursor_row) > 0) {
            mv.cursor_row++;
        }
        mv.cursor_col = 0;
        break;
    }
    case GDK_KEY_i:
        if (!mv_undo_push()) break;
        mv.mode = mv_insert;
        break;
    case GDK_KEY_I:
        if (!mv_undo_push()) break;
        mv.cursor_col = 0;
        mv.mode = mv_insert;
        break;
    case GDK_KEY_a:
        if (!mv_undo_push()) break;
        mv.cursor_col++;
        mv_clamp_cursor();
        mv.mode = mv_insert;
        break;
    case GDK_KEY_A:
        if (!mv_undo_push()) break;
        mv.cursor_col = mv_line_len(mv.cursor_row);
        mv.mode = mv_insert;
        break;
    case GDK_KEY_o:
        if (!mv_undo_push()) return;
        if (!mv_insert_line(mv.cursor_row + 1, "")) break;
        mv.cursor_row++;
        mv.cursor_col = 0;
        mv.mode = mv_insert;
        mv.modified = true;
        break;
    case GDK_KEY_O:
        if (!mv_undo_push()) return;
        if (!mv_insert_line(mv.cursor_row, "")) break;
        mv.cursor_col = 0;
        mv.mode = mv_insert;
        mv.modified = true;
        break;
    case GDK_KEY_x:
        mv_delete_char_under_cursor();
        break;
    case GDK_KEY_X:
        mv_delete_char_before_cursor();
        break;
    case GDK_KEY_d:
        mv.pending_op = 'd';
        break;
    case GDK_KEY_c:
        mv.pending_op = 'c';
        break;
    case GDK_KEY_y:
        mv.pending_op = 'y';
        break;
    case GDK_KEY_p:
        mv_paste(true);
        break;
    case GDK_KEY_P:
        mv_paste(false);
        break;
    case GDK_KEY_u:
        mv_undo_perform();
        break;
    case GDK_KEY_J:
        mv_join_lines();
        break;
    case GDK_KEY_D:
        mv_delete_to_end();
        break;
    case GDK_KEY_C:
        mv_delete_to_end();
        mv.mode = mv_insert;
        break;
    case GDK_KEY_S:
        mv_delete_line_op();
        mv.mode = mv_insert;
        break;
    case GDK_KEY_asciitilde: {
        int row = mv.cursor_row;
        int col = mv.cursor_col;
        int len = mv_line_len(row);
        if (col < len) {
            if (!mv_undo_push()) return;
            char c = mv.lines[row][col];
            mv.lines[row][col] = isupper((unsigned char) c) ? tolower((unsigned char) c) : toupper((unsigned char) c);
            mv.cursor_col++;
            mv.modified = true;
            mv_clamp_cursor();
        }
        break;
    }
    case GDK_KEY_slash:
        mv.mode = mv_search;
        mv.search_forward = true;
        mv.search_len = 0;
        mv.search_buf[0] = '\0';
        break;
    case GDK_KEY_question:
        mv.mode = mv_search;
        mv.search_forward = false;
        mv.search_len = 0;
        mv.search_buf[0] = '\0';
        break;
    case GDK_KEY_n:
        mv_search_execute(mv.search_forward);
        break;
    case GDK_KEY_N:
        mv_search_execute(!mv.search_forward);
        break;
    case GDK_KEY_asterisk: {
        int row = mv.cursor_row;
        int col = mv.cursor_col;
        int len = mv_line_len(row);
        int start = col;
        while (start > 0 && mv_is_word_char(mv.lines[row][start - 1])) {
            start--;
        }
        int end = col;
        while (end < len && mv_is_word_char(mv.lines[row][end])) {
            end++;
        }
        if (end > start) {
            int wlen = end - start;
            if (wlen < 255) {
                strncpy(mv.search_buf, mv.lines[row] + start, (size_t) wlen);
                mv.search_buf[wlen] = '\0';
                mv.search_len = wlen;
                mv.search_forward = true;
                mv_search_execute(true);
            }
        }
        break;
    }
    case GDK_KEY_colon:
        mv.mode = mv_command;
        mv.command_len = 0;
        mv.command_buf[0] = '\0';
        break;
    case GDK_KEY_Escape:
        break;
    default:
        break;
    }
}

static void mv_handle_insert_key(GdkEventKey *event) {
    unsigned int key = event->keyval;
    if (key == GDK_KEY_Escape) {
        mv.mode = mv_normal;
        if (mv.cursor_col > 0) {
            mv.cursor_col--;
        }
        return;
    }
    if (key == GDK_KEY_BackSpace) {
        mv_backspace();
        return;
    }
    if (key == GDK_KEY_Delete) {
        mv_delete_char_under_cursor();
        return;
    }
    if (key == GDK_KEY_Return || key == GDK_KEY_KP_Enter) {
        mv_insert_newline();
        return;
    }
    if (key == GDK_KEY_Left) {
        mv.cursor_col--;
        mv_clamp_cursor();
        return;
    }
    if (key == GDK_KEY_Right) {
        mv.cursor_col++;
        mv_clamp_cursor();
        return;
    }
    if (key == GDK_KEY_Up) {
        mv.cursor_row--;
        mv_clamp_cursor();
        return;
    }
    if (key == GDK_KEY_Down) {
        mv.cursor_row++;
        mv_clamp_cursor();
        return;
    }
    if (key == GDK_KEY_Home) {
        mv.cursor_col = 0;
        return;
    }
    if (key == GDK_KEY_End) {
        mv.cursor_col = mv_line_len(mv.cursor_row);
        return;
    }
    if ((event->state & GDK_CONTROL_MASK) && (key == GDK_KEY_w)) {
        /* Delete word backward */
        if (!mv_undo_push()) return;
        int col = mv.cursor_col;
        while (col > 0 && mv.lines[mv.cursor_row][col - 1] == ' ') {
            col--;
        }
        while (col > 0 && mv_is_word_char(mv.lines[mv.cursor_row][col - 1])) {
            col--;
        }
        int deleted = mv.cursor_col - col;
        if (deleted > 0) {
            char *line = mv.lines[mv.cursor_row];
            int len = mv_line_len(mv.cursor_row);
            memmove(line + col, line + mv.cursor_col, (size_t) (len - mv.cursor_col + 1));
            mv.cursor_col = col;
            mv.modified = true;
        }
        return;
    }
    if (key >= 32 && key < 127) {
        mv_insert_char((char) key);
        return;
    }
    if (key >= GDK_KEY_space && key <= GDK_KEY_asciitilde) {
        mv_insert_char((char) key);
    }
}

static void mv_handle_command_key(GdkEventKey *event) {
    unsigned int key = event->keyval;
    if (key == GDK_KEY_Escape) {
        mv.mode = mv_normal;
        mv.command_len = 0;
        mv.command_buf[0] = '\0';
        return;
    }
    if (key == GDK_KEY_BackSpace) {
        if (mv.command_len > 0) {
            mv.command_len--;
            mv.command_buf[mv.command_len] = '\0';
        }
        return;
    }
    if (key == GDK_KEY_Return || key == GDK_KEY_KP_Enter) {
        mv_execute_command();
        return;
    }
    if (key >= 32 && key < 127 && mv.command_len < 254) {
        mv.command_buf[mv.command_len++] = (char) key;
        mv.command_buf[mv.command_len] = '\0';
    }
}

static void mv_handle_search_key(GdkEventKey *event) {
    unsigned int key = event->keyval;
    if (key == GDK_KEY_Escape) {
        mv.mode = mv_normal;
        return;
    }
    if (key == GDK_KEY_BackSpace) {
        if (mv.search_len > 0) {
            mv.search_len--;
            mv.search_buf[mv.search_len] = '\0';
        }
        return;
    }
    if (key == GDK_KEY_Return || key == GDK_KEY_KP_Enter) {
        mv_search_execute(mv.search_forward);
        mv.mode = mv_normal;
        return;
    }
    if (key >= 32 && key < 127 && mv.search_len < 254) {
        mv.search_buf[mv.search_len++] = (char) key;
        mv.search_buf[mv.search_len] = '\0';
    }
}

/* ------------------------------------------------------------------ */
/* Rendering                                                            */
/* ------------------------------------------------------------------ */
void microvim_ensure_tags(GtkTextBuffer *buffer) {
    if (!gtk_text_tag_table_lookup(gtk_text_buffer_get_tag_table(buffer), "mv_normal")) {
        gtk_text_buffer_create_tag(buffer, "mv_normal", "foreground", "#cdd6e4", NULL);
        gtk_text_buffer_create_tag(buffer, "mv_linenum", "foreground", "#5f7387", NULL);
        gtk_text_buffer_create_tag(buffer, "mv_comment", "foreground", "#6a9955", NULL);
        gtk_text_buffer_create_tag(buffer, "mv_section", "foreground", "#56b6c2", "weight", PANGO_WEIGHT_BOLD, NULL);
        gtk_text_buffer_create_tag(buffer, "mv_key", "foreground", "#e5c07b", NULL);
        gtk_text_buffer_create_tag(buffer, "mv_value", "foreground", "#cdd6e4", NULL);
        gtk_text_buffer_create_tag(buffer, "mv_tilde", "foreground", "#3a4556", NULL);
        gtk_text_buffer_create_tag(buffer, "mv_status", "foreground", "#0b111a", "background", "#cdd6e4", "weight",
                                   PANGO_WEIGHT_BOLD, NULL);
        gtk_text_buffer_create_tag(buffer, "mv_insert_label", "foreground", "#8be28b", "weight", PANGO_WEIGHT_BOLD,
                                   NULL);
    }
}

void microvim_render(GtkTextBuffer *buffer) {
    if (!buffer) {
        return;
    }
    microvim_ensure_tags(buffer);
    gtk_text_buffer_set_text(buffer, "", -1);

    /* Adjust scroll */
    if (mv.cursor_row < mv.scroll_offset) {
        mv.scroll_offset = mv.cursor_row;
    }
    if (mv.cursor_row >= mv.scroll_offset + mv_view_height - 2) {
        mv.scroll_offset = mv.cursor_row - mv_view_height + 3;
    }
    if (mv.scroll_offset < 0) {
        mv.scroll_offset = 0;
    }

    GtkTextIter end_iter;
    char line_buf[mv_max_line_len + 64];

    for (int view_line = 0; view_line < mv_view_height - 2; view_line++) {
        int row = mv.scroll_offset + view_line;
        if (row < mv.line_count) {
            if (mv.show_line_numbers) {
                snprintf(line_buf, sizeof(line_buf), "%4d ", row + 1);
                gtk_text_buffer_get_end_iter(buffer, &end_iter);
                gtk_text_buffer_insert_with_tags_by_name(buffer, &end_iter, line_buf, -1, "mv_linenum", NULL);
            }
            const char *line = mv.lines[row];
            const char *tag = "mv_normal";
            if (line[0] == '#') {
                tag = "mv_comment";
            } else if (line[0] == '[') {
                tag = "mv_section";
            } else {
                char *eq = strchr(line, '=');
                if (eq && (eq > line)) {
                    int key_len = (int) (eq - line);
                    gtk_text_buffer_get_end_iter(buffer, &end_iter);
                    gtk_text_buffer_insert_with_tags_by_name(buffer, &end_iter, line, key_len, "mv_key", NULL);
                    gtk_text_buffer_get_end_iter(buffer, &end_iter);
                    gtk_text_buffer_insert_with_tags_by_name(buffer, &end_iter, eq, -1, "mv_value", NULL);
                    line = NULL;
                }
            }
            if (line) {
                gtk_text_buffer_get_end_iter(buffer, &end_iter);
                gtk_text_buffer_insert_with_tags_by_name(buffer, &end_iter, line, -1, tag, NULL);
            }
        } else {
            if (mv.show_line_numbers) {
                gtk_text_buffer_get_end_iter(buffer, &end_iter);
                gtk_text_buffer_insert_with_tags_by_name(buffer, &end_iter, "     ", -1, "mv_linenum", NULL);
            }
            gtk_text_buffer_get_end_iter(buffer, &end_iter);
            gtk_text_buffer_insert_with_tags_by_name(buffer, &end_iter, "~", -1, "mv_tilde", NULL);
        }
        gtk_text_buffer_get_end_iter(buffer, &end_iter);
        gtk_text_buffer_insert(buffer, &end_iter, "\n", -1);
    }

    /* Status bar */
    char status_buf[1024];
    const char *mode_text = "";
    if (mv.mode == mv_insert) {
        mode_text = "-- INSERT -- ";
    } else if (mv.mode == mv_command) {
        mode_text = ":";
    } else if (mv.mode == mv_search) {
        mode_text = mv.search_forward ? "/" : "?";
    }

    if (mv.mode == mv_command) {
        snprintf(status_buf, sizeof(status_buf), ":%s", mv.command_buf);
    } else if (mv.mode == mv_search) {
        snprintf(status_buf, sizeof(status_buf), "%s%s", mv.search_forward ? "/" : "?", mv.search_buf);
    } else {
        int pct = (mv.line_count > 1) ? ((mv.cursor_row + 1) * 100 / mv.line_count) : 100;
        const char *pos_text = (mv.cursor_row == 0) ? "Top" : (mv.cursor_row == mv.line_count - 1) ? "Bot" : "";
        /* MPE_TASK_V15R2_STATUS_NEW_FILE_BEGIN */
        const char *file_status_text = "";
        if (!mv.file_exists && !mv.modified) {
            file_status_text = " [New]";
        } else if (mv.modified) {
            file_status_text = " [Modified]";
        }
        snprintf(status_buf, sizeof(status_buf), "%s\"%s\" %dL%s%s%s%d,%d %s %d%%", mode_text, mv.filename,
                 mv.line_count, file_status_text, mode_text[0] ? "" : "   ", pos_text[0] ? "" : "", mv.cursor_row + 1,
                 mv.cursor_col + 1, pos_text[0] ? pos_text : "", pct);
        /* MPE_TASK_V15R2_STATUS_NEW_FILE_END */
    }
    gtk_text_buffer_get_end_iter(buffer, &end_iter);
    gtk_text_buffer_insert_with_tags_by_name(buffer, &end_iter, status_buf, -1,
                                             (mv.mode == mv_insert) ? "mv_insert_label" : "mv_status", NULL);

    /* Place cursor */
    GtkTextIter cursor_iter;
    int target_line = mv.cursor_row - mv.scroll_offset;
    if (target_line < 0) {
        target_line = 0;
    }
    if (target_line >= mv_view_height - 2) {
        target_line = mv_view_height - 3;
    }
    int col_offset = mv.cursor_col + (mv.show_line_numbers ? 5 : 0);
    gtk_text_buffer_get_iter_at_line_offset(buffer, &cursor_iter, target_line, col_offset);
    gtk_text_buffer_place_cursor(buffer, &cursor_iter);
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */
mv_mode microvim_get_mode(void) {
    return mv.mode;
}
bool microvim_is_active(void) {
    return mv.active;
}

void microvim_open(const char *filename) {
    microvim_close();
    memset(&mv, 0, sizeof(mv));
    if (!filename) return;
    mv.lines = NULL;
    mv.line_capacity = 0;
    mv.line_count = 0;
    strncpy(mv.filename, filename, sizeof(mv.filename) - 1);
    mv.filename[sizeof(mv.filename) - 1] = '\0';
    mv.file_exists = mv_load_file(filename); /* MPE_TASK_V15R2_FILE_EXISTS_STORE */
    if (mv.line_count == 0 && !mv_insert_line(0, "")) {
        microvim_close();
        return;
    }
    mv.cursor_row = 0;
    mv.cursor_col = 0;
    mv.mode = mv_normal;
    mv.modified = false;
    mv.active = true;
    mv.quit_requested = false;
    mv.show_line_numbers = true;
    mv.scroll_offset = 0;
    mv.undo_top = 0;
    mv.redo_top = 0;
    mv.yank_text = NULL;
    mv.pending_op = 0;
    mv.command_len = 0;
    mv.search_len = 0;
    /* Default to showing line numbers for config files */
    const char *ext = strrchr(filename, '.');
    if (ext && (strcmp(ext, ".cfg") == 0 || strcmp(ext, ".ini") == 0)) {
        mv.show_line_numbers = true;
    }
}

void microvim_close(void) {
    mv_clear_lines();
    if (mv.lines) {
        free(mv.lines);
        mv.lines = NULL;
    }
    if (mv.yank_text) {
        free(mv.yank_text);
        mv.yank_text = NULL;
    }
    for (int i = 0; i < mv_undo_depth; i++) {
        mv_snapshot *s = &mv.undo_stack[i];
        if (s->lines) {
            for (int j = 0; j < s->line_count; j++) {
                free(s->lines[j]);
            }
            free(s->lines);
            s->lines = NULL;
        }
    }
    mv.active = false;
    mv.quit_requested = false;
}

void microvim_handle_key(GdkEventKey *event) {
    if (!mv.active) {
        return;
    }
    switch (mv.mode) {
    case mv_normal:
        mv_handle_normal_key(event);
        break;
    case mv_insert:
        mv_handle_insert_key(event);
        break;
    case mv_command:
        mv_handle_command_key(event);
        break;
    case mv_search:
        mv_handle_search_key(event);
        break;
    }
    if (mv.quit_requested) {
        microvim_close();
    }
}
/* MPE_TASK_V15R2_MICROVIM_IMPL_END */

#endif /* MPE_GTK4 */

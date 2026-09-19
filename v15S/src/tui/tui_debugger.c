/* MPE TUI debugger — ncurses real-time inspector for bodies, joints,
 * constraints, islands and the mathematics behind them.
 *
 * Screens (tui_mode_t): OVERVIEW (body table + engine sidebar),
 * OBJECT_DETAIL (full characteristics + math of the selected body),
 * JOINT_DETAIL (spring + constraint pools with live endpoint geometry),
 * SCENE_GRAPH (pairwise relative positions, islands, solver stats),
 * HELP (keys + conventions).
 *
 * Plain-text snapshot/stream output lives in tui_dump.c and shares the
 * formatting helpers below, so piped output and the live UI agree.
 */
#include "tui_debugger.h"
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include "../physics/broadphase.h"
#include "../physics/islands.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------ */
/* Small shared helpers                                                */
/* ------------------------------------------------------------------ */

static const char *tui_type_str(object_type t) {
    switch (t) {
        case object_sphere:
            return "SPH";
        case object_cube:
            return "CUB";
        case object_cylinder:
            return "CYL";
        default:
            return "???";
    }
}

static const char *tui_state_str(const rigidbody *rb) {
    if (!rb) {
        return "----";
    }
    if (rb->static_state) {
        return "STATIC";
    }
    if (rb->kinematic) {
        return "KINEMA";
    }
    if (rb->is_sleeping) {
        return "SLEEP";
    }
    return "AWAKE";
}

static const char *tui_constraint_str(constraint_type t) {
    switch (t) {
        case constraint_revolute:
            return "revolute";
        case constraint_fixed:
            return "fixed";
        case constraint_prismatic:
            return "prismatic";
        case constraint_distance:
            return "distance";
        case constraint_rope:
            return "rope";
        case constraint_spring:
            return "spring";
        default:
            return "unknown";
    }
}

/* World-space position of a body-local anchor. */
static vector3 tui_anchor_world(const rigidbody *rb, vector3 local) {
    vector3 off = vector4_rotate_to_vector3(rb->orientation, local);
    return vector3_addition(rb->position, off);
}

/* Linear index of a body id, or -1. */
static int tui_index_by_id(physics_world *world, uint32_t id) {
    if (!world || id == 0) {
        return -1;
    }
    for (int i = 0; i < world->body_count; i++) {
        if (world->bodies[i].object_id == id) {
            return i;
        }
    }
    return -1;
}

/* Full world-space inertia tensor W = R * I_local * R^T. */
static math3 tui_inertia_world(const rigidbody *rb) {
    math3 r = vector4_to_math3(rb->orientation);
    math3 rt = math3_transposition(r);
    return math3_multiplication(r, math3_multiplication(rb->inertia_tensor_local, rt));
}

/* Extract inspection-only XYZ euler angles (radians) from R.
 * ex = rot about X, ey = rot about Y, ez = rot about Z. */
static void tui_euler_xyz(math3 r, float *ex, float *ey, float *ez) {
    float sy = -(r.matrix[2][0]);
    if (sy > 1.0f) {
        sy = 1.0f;
    } else if (sy < -1.0f) {
        sy = -1.0f;
    }
    *ey = asinf(sy);
    *ex = atan2f(r.matrix[2][1], r.matrix[2][2]);
    *ez = atan2f(r.matrix[1][0], r.matrix[0][0]);
}

void tui_format_vector3(char *buf, size_t sz, vector3 v, const char *label) {
    if (!buf || sz == 0) {
        return;
    }
    snprintf(buf, sz, "%s(%+.4f,%+.4f,%+.4f)", label ? label : "", v.x, v.y, v.z);
}

void tui_format_vector4(char *buf, size_t sz, vector4 q, const char *label) {
    if (!buf || sz == 0) {
        return;
    }
    snprintf(buf, sz, "%s(%+.4f,%+.4f,%+.4f,%+.4f)", label ? label : "", q.w, q.x, q.y, q.z);
}

void tui_format_euler(char *buf, size_t sz, vector4 q, const char *label) {
    if (!buf || sz == 0) {
        return;
    }
    math3 r = vector4_to_math3(q);
    float ex, ey, ez;
    tui_euler_xyz(r, &ex, &ey, &ez);
    const float d = 180.0f / (float) M_PI;
    snprintf(buf, sz, "%sXYZ(%+.2f,%+.2f,%+.2f)deg", label ? label : "", ex * d, ey * d, ez * d);
}

void tui_format_matrix3(char *buf, size_t sz, math3 m, const char *label) {
    if (!buf || sz == 0) {
        return;
    }
    snprintf(buf, sz, "%s[%.3f %.3f %.3f; %.3f %.3f %.3f; %.3f %.3f %.3f]", label ? label : "",
             m.matrix[0][0], m.matrix[0][1], m.matrix[0][2], m.matrix[1][0], m.matrix[1][1],
             m.matrix[1][2], m.matrix[2][0], m.matrix[2][1], m.matrix[2][2]);
}

float tui_quat_to_yaw(vector4 q) {
    math3 r = vector4_to_math3(q);
    float ex, ey, ez;
    tui_euler_xyz(r, &ex, &ey, &ez);
    return ey;
}

float tui_quat_to_pitch(vector4 q) {
    math3 r = vector4_to_math3(q);
    float ex, ey, ez;
    tui_euler_xyz(r, &ex, &ey, &ez);
    return ex;
}

float tui_quat_to_roll(vector4 q) {
    math3 r = vector4_to_math3(q);
    float ex, ey, ez;
    tui_euler_xyz(r, &ex, &ey, &ez);
    return ez;
}

/* One-line body summary for tables. */
static void tui_body_line(physics_world *world, int idx, char *buf, size_t sz) {
    if (!buf || sz == 0) {
        return;
    }
    if (!world || idx < 0 || idx >= world->body_count) {
        snprintf(buf, sz, "--");
        return;
    }
    rigidbody *rb = &world->bodies[idx];
    float sp = vector3_length(rb->velocity);
    float ws = vector3_length(rb->angular_velocity);
    float ke = rb_get_kinetic_energy(rb);
    snprintf(buf, sz, "%3d id=%-5u %-3s p=(%+7.3f,%+7.3f,%+7.3f) |v|=%7.3f |w|=%7.3f m=%8.3f %-6s KE=%10.4f", idx,
             rb->object_id, tui_type_str(rb->type), rb->position.x, rb->position.y, rb->position.z, sp, ws,
             rb->mass, tui_state_str(rb), ke);
}

static bool tui_body_visible(const tui_debugger_t *dbg, const rigidbody *rb) {
    if (!dbg || !rb) {
        return false;
    }
    if (dbg->filter_len <= 0) {
        return true;
    }
    char line[256];
    /* Match against "id type state" text. */
    snprintf(line, sizeof(line), "%u %s %s", rb->object_id, tui_type_str(rb->type), tui_state_str(rb));
    for (char *p = line; *p; p++) {
        if (*p >= 'A' && *p <= 'Z') {
            *p = (char) (*p + 32);
        }
    }
    char needle[64];
    size_t n = (size_t) dbg->filter_len < sizeof(needle) - 1 ? (size_t) dbg->filter_len : sizeof(needle) - 1;
    for (size_t i = 0; i < n; i++) {
        char c = dbg->filter_text[i];
        needle[i] = (char) ((c >= 'A' && c <= 'Z') ? c + 32 : c);
    }
    needle[n] = '\0';
    return strstr(line, needle) != NULL;
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

void tui_debugger_init(tui_debugger_t *dbg, physics_world *world) {
    if (!dbg) {
        return;
    }
    memset(dbg, 0, sizeof(*dbg));
    dbg->world = world;
    dbg->mode = TUI_MODE_OVERVIEW;
    dbg->time_scale = 1.0f;
}

void tui_debugger_cleanup(tui_debugger_t *dbg) {
    if (!dbg) {
        return;
    }
    dbg->main_win = NULL;
    dbg->sidebar_win = NULL;
    dbg->detail_win = NULL;
    dbg->status_win = NULL;
}

void tui_debugger_step(tui_debugger_t *dbg, float dt) {
    if (!dbg || !dbg->world || dbg->paused || !(dt > 0.0f)) {
        return;
    }
    physics_world_step(dbg->world, dt);
    dbg->frame_count++;
}

/* ------------------------------------------------------------------ */
/* Input                                                               */
/* ------------------------------------------------------------------ */

void tui_debugger_handle_input(tui_debugger_t *dbg, int ch) {
    if (!dbg) {
        return;
    }
    physics_world *world = dbg->world;
    int n = world ? world->body_count : 0;

    /* Text filter entry takes precedence. */
    if (dbg->filter_active) {
        if (ch == 27 || ch == '\n' || ch == '\r') { /* ESC / Enter: done */
            dbg->filter_active = false;
            if (ch == 27) {
                dbg->filter_len = 0;
                dbg->filter_text[0] = '\0';
            }
            return;
        }
        if ((ch == KEY_BACKSPACE || ch == 127 || ch == 8) && dbg->filter_len > 0) {
            dbg->filter_len--;
            dbg->filter_text[dbg->filter_len] = '\0';
            return;
        }
        if (ch >= 32 && ch < 127 && dbg->filter_len < (int) sizeof(dbg->filter_text) - 1) {
            dbg->filter_text[dbg->filter_len++] = (char) ch;
            dbg->filter_text[dbg->filter_len] = '\0';
        }
        return;
    }

    switch (ch) {
        case '1':
            dbg->mode = TUI_MODE_OVERVIEW;
            break;
        case '2':
            dbg->mode = TUI_MODE_OBJECT_DETAIL;
            break;
        case '3':
            dbg->mode = TUI_MODE_JOINT_DETAIL;
            break;
        case '4':
            dbg->mode = TUI_MODE_SCENE_GRAPH;
            break;
        case '5':
            dbg->mode = TUI_MODE_HELP;
            break;
        case '\t':
        case 'm':
            dbg->mode = (tui_mode_t) ((dbg->mode + 1) % 5);
            break;
        case ' ':
            dbg->paused = !dbg->paused;
            break;
        case 's':
            dbg->paused = true;
            if (world) {
                physics_world_step(world, 1.0f / 60.0f);
                dbg->frame_count++;
            }
            break;
        case '+':
        case '=':
            dbg->time_scale *= 2.0f;
            if (dbg->time_scale > 4.0f) {
                dbg->time_scale = 4.0f;
            }
            break;
        case '-':
        case '_':
            dbg->time_scale *= 0.5f;
            if (dbg->time_scale < 0.125f) {
                dbg->time_scale = 0.125f;
            }
            break;
        case '/':
            dbg->filter_active = true;
            break;
        case 'c':
            dbg->filter_len = 0;
            dbg->filter_text[0] = '\0';
            break;
        case KEY_UP:
        case 'k':
            if (dbg->selected_object > 0) {
                dbg->selected_object--;
            }
            if (dbg->selected_joint > 0) {
                dbg->selected_joint--;
            }
            break;
        case KEY_DOWN:
        case 'j':
            if (dbg->selected_object < n - 1) {
                dbg->selected_object++;
            }
            if (dbg->selected_joint < 4096) {
                dbg->selected_joint++;
            }
            break;
        case KEY_PPAGE:
            dbg->selected_object -= 10;
            if (dbg->selected_object < 0) {
                dbg->selected_object = 0;
            }
            break;
        case KEY_NPAGE:
            dbg->selected_object += 10;
            if (dbg->selected_object > n - 1) {
                dbg->selected_object = n - 1;
            }
            break;
        case KEY_HOME:
            dbg->selected_object = 0;
            dbg->selected_joint = 0;
            break;
        case KEY_END:
            dbg->selected_object = n - 1;
            break;
        default:
            break;
    }
    if (dbg->selected_object < 0) {
        dbg->selected_object = 0;
    }
    if (n > 0 && dbg->selected_object > n - 1) {
        dbg->selected_object = n - 1;
    }
    if (dbg->selected_joint < 0) {
        dbg->selected_joint = 0;
    }
}

/* ------------------------------------------------------------------ */
/* Render helpers                                                      */
/* ------------------------------------------------------------------ */

static void tui_draw_header(tui_debugger_t *dbg) {
    int h, w;
    getmaxyx(stdscr, h, w);
    (void) h;
    const char *mode =
        dbg->mode == TUI_MODE_OVERVIEW ? "OVERVIEW" : dbg->mode == TUI_MODE_OBJECT_DETAIL ? "OBJECT" :
        dbg->mode == TUI_MODE_JOINT_DETAIL ? "JOINTS" :
        dbg->mode == TUI_MODE_SCENE_GRAPH  ? "GRAPH" :
                                              "HELP";
    int n = dbg->world ? dbg->world->body_count : 0;
    mvprintw(0, 0, "MPE-TUI [%s] tick=%llu bodies=%d %s x%.2f filter='%s'%s", mode,
             (unsigned long long) dbg->frame_count, n, dbg->paused ? "PAUSED" : "LIVE", dbg->time_scale,
             dbg->filter_text, dbg->filter_active ? "<typing>" : "");
    clrtoeol();
    if (w > 0) {
        mvchgat(0, 0, w, A_REVERSE, 0, NULL);
    }
}

/* Engine/math truth strip shared by sidebar + snapshot. */
static int tui_engine_lines(physics_world *world, unsigned long long tick, char out[][128], int cap) {
    int n = 0;
    if (!world || cap <= 0) {
        return 0;
    }
    double ke = 0.0;
    vector3 mom = {0.0f, 0.0f, 0.0f};
    int awake = 0, sleeping = 0;
    for (int i = 0; i < world->body_count && n < cap; i++) {
        rigidbody *rb = &world->bodies[i];
        ke += (double) rb_get_kinetic_energy(rb);
        mom = vector3_addition(mom, vector3_scaling(rb->velocity, rb->mass));
        if (rb->is_sleeping) {
            sleeping++;
        } else if (!rb->static_state) {
            awake++;
        }
    }
    snprintf(out[n++], 128, "tick=%llu bodies=%d awake=%d sleep=%d", tick, world ? world->body_count : 0, awake,
             sleeping);
    if (n < cap) {
        snprintf(out[n++], 128, "KE_total=%.5f P=(%+.3f,%+.3f,%+.3f)|P|=%.4f", ke, mom.x, mom.y, mom.z,
                 vector3_length(mom));
    }
    if (n < cap) {
        const mpe_config_t *dc = mpe_world_cfg(world);
        snprintf(out[n++], 128, "grav=%+.2f drag=%.4f angScale=%.3f iters=%d dt=1/60", dc->world.gravity,
                 dc->world.drag, dc->world.angular_damping_scale, dc->timestep.solver_iterations);
    }
    if (n < cap) {
        const mpe_config_t *dc = mpe_world_cfg(world);
        snprintf(out[n++], 128, "slop=%.3f beta=%.2f maxBias=%.1f islands=%d/%d", dc->solver.penetration_slop,
                 dc->solver.bias_factor, dc->solver.max_separation_bias, islands_count(world),
                 world ? world->island_total : 0);
    }
    if (n < cap) {
        snprintf(out[n++], 128, "cache hit=%d miss=%d ovfl=%d cell=%.2f", contact_cache_get_hits(world),
                 contact_cache_get_misses(world), world ? world->manifold_overflow_count : 0,
                 broadphase_get_current_cell_size(world));
    }
    if (n < cap) {
        snprintf(out[n++], 128, "bp node=%d/%d pairOv=%d dedupOv=%d big=%d", world && world->broadphase ?
                 world->broadphase->node_count : 0,
                 world && world->broadphase ? world->broadphase->node_pool_capacity : 0,
                 broadphase_get_pair_overflow_count(world), broadphase_get_pair_dedupe_overflow_count(world),
                 broadphase_get_large_object_clamp_count(world));
    }
    return n;
}

void tui_render_sidebar(tui_debugger_t *dbg) {
    if (!dbg || !dbg->sidebar_win) {
        return;
    }
    werase(dbg->sidebar_win);
    int h, w;
    getmaxyx(dbg->sidebar_win, h, w);
    if (w < 20 || h < 6) {
        return;
    }
    wmove(dbg->sidebar_win, 0, 0);
    wattron(dbg->sidebar_win, A_BOLD);
    wprintw(dbg->sidebar_win, "ENGINE/TRUTH");
    wattroff(dbg->sidebar_win, A_BOLD);
    char lines[8][128];
    int n = tui_engine_lines(dbg->world, dbg->frame_count, lines, 8);
    for (int i = 0; i < n && i + 1 < h; i++) {
        mvwprintw(dbg->sidebar_win, i + 1, 0, "%.*s", w - 1, lines[i]);
    }
    /* Joint pool summary. */
    int row = n + 2;
    if (dbg->world && row + 3 < h) {
        int springs = 0;
        for (int i = 0; i < mpe_max_joints; i++) {
            if (dbg->world->spring_joints[i].is_active) {
                springs++;
            }
        }
        mvwprintw(dbg->sidebar_win, row++, 0, "springs=%d constr=%d", springs,
                  dbg->world ? dbg->world->revolute_constraint_count : 0);
        const mpe_config_t *dc = mpe_world_cfg(dbg->world);
        mvwprintw(dbg->sidebar_win, row++, 0, "sleepEn=%d slop=%.3f", dc->sleep.enable,
                  dc->solver.penetration_slop);
    }
    wrefresh(dbg->sidebar_win);
}

void tui_render_status(tui_debugger_t *dbg) {
    if (!dbg || !dbg->status_win) {
        return;
    }
    werase(dbg->status_win);
    mvwprintw(dbg->status_win, 0, 0,
              "[1]Overview [2]Object [3]Joints [4]Graph [5]Help  j/k/Up/Dn select  Space pause  s step  +/-speed  /filter c-clear  q quit");
    wrefresh(dbg->status_win);
}

/* ------------------------------------------------------------------ */
/* Screens                                                             */
/* ------------------------------------------------------------------ */

void tui_render_overview(tui_debugger_t *dbg) {
    if (!dbg || !dbg->main_win) {
        return;
    }
    werase(dbg->main_win);
    int h, w;
    getmaxyx(dbg->main_win, h, w);
    if (h < 4 || w < 40) {
        mvwprintw(dbg->main_win, 0, 0, "terminal too small");
        wrefresh(dbg->main_win);
        return;
    }
    mvwprintw(dbg->main_win, 0, 0, "%-56s %8s %8s %9s %-6s %10s", "idx id type position", "|v|", "|w|", "mass",
              "state", "KE");
    /* Clamp selection into visible filtered list. */
    int row = 1;
    char line[256];
    for (int i = 0; i < (dbg->world ? dbg->world->body_count : 0) && row < h; i++) {
        rigidbody *rb = &dbg->world->bodies[i];
        if (!tui_body_visible(dbg, rb)) {
            continue;
        }
        if (row - 1 < dbg->scroll_offset) {
            row++;
            continue;
        }
        tui_body_line(dbg->world, i, line, sizeof(line));
        if (i == dbg->selected_object) {
            wattron(dbg->main_win, A_REVERSE);
        }
        mvwprintw(dbg->main_win, row, 0, "%.*s", w - 1, line);
        if (i == dbg->selected_object) {
            wattroff(dbg->main_win, A_REVERSE);
        }
        row++;
    }
    /* Keep selection on screen. */
    int vis = row - 1;
    if (dbg->selected_object < dbg->scroll_offset) {
        dbg->scroll_offset = dbg->selected_object;
    }
    if (vis >= h && dbg->scroll_offset < dbg->selected_object) {
        dbg->scroll_offset = dbg->selected_object - h + 2;
        if (dbg->scroll_offset < 0) {
            dbg->scroll_offset = 0;
        }
    }
    (void) vis;
    wrefresh(dbg->main_win);
}

void tui_render_object_detail(tui_debugger_t *dbg) {
    if (!dbg || !dbg->main_win) {
        return;
    }
    werase(dbg->main_win);
    int h, w;
    getmaxyx(dbg->main_win, h, w);
    physics_world *world = dbg->world;
    if (!world || world->body_count <= 0) {
        mvwprintw(dbg->main_win, 0, 0, "no bodies");
        wrefresh(dbg->main_win);
        return;
    }
    int idx = dbg->selected_object;
    if (idx < 0) {
        idx = 0;
    }
    if (idx >= world->body_count) {
        idx = world->body_count - 1;
    }
    rigidbody *rb = &world->bodies[idx];
    char b0[128], b1[160], b2[256];
    int r = 0;
    mvwprintw(dbg->main_win, r++, 0, "OBJECT [%d] id=%u gen=%u type=%s %s", idx, rb->object_id,
              rb->object_generation, tui_type_str(rb->type), tui_state_str(rb));
    tui_format_vector3(b0, sizeof(b0), rb->position, "pos=");
    tui_format_vector3(b1, sizeof(b1), rb->velocity, "vel=");
    mvwprintw(dbg->main_win, r++, 0, "%.*s %.*s", w - 1, b0, (int) (w - 1), b1);
    tui_format_vector3(b0, sizeof(b0), rb->acceleration, "acc=");
    tui_format_vector3(b1, sizeof(b1), rb->angular_velocity, "angVel=");
    mvwprintw(dbg->main_win, r++, 0, "%.*s %.*s", w - 1, b0, (int) (w - 1), b1);
    tui_format_vector4(b0, sizeof(b0), rb->orientation, "quat=");
    tui_format_euler(b1, sizeof(b1), rb->orientation, "euler=");
    mvwprintw(dbg->main_win, r++, 0, "%.*s", w - 1, b0);
    mvwprintw(dbg->main_win, r++, 0, "%.*s", w - 1, b1);
    tui_format_vector3(b0, sizeof(b0), rb->angular_acceleration, "angAcc=");
    mvwprintw(dbg->main_win, r++, 0, "%.*s", w - 1, b0);
    mvwprintw(dbg->main_win, r++, 0, "mass=%.5f invM=%.6f rest=%.3f fricS=%.3f fricK=%.3f nice=%d", rb->mass,
              rb->inverse_mass, rb->restitution, rb->friction_static, rb->friction_kinetic, rb->nice_value);
    mvwprintw(dbg->main_win, r++, 0, "r=%.4f halfLen=%.4f halfExt=(%.3f,%.3f,%.3f) boundR=%.4f", rb->radius,
              rb->cylinder_half_length, rb->half_extensions.x, rb->half_extensions.y, rb->half_extensions.z,
              broadphase_bounding_radius(rb));
    math3 wl = rb->inertia_tensor_local;
    mvwprintw(dbg->main_win, r++, 0, "I_local=diag(%.5f,%.5f,%.5f)", wl.matrix[0][0], wl.matrix[1][1],
              wl.matrix[2][2]);
    math3 ww = tui_inertia_world(rb);
    mvwprintw(dbg->main_win, r++, 0, "I_world=diag(%.5f,%.5f,%.5f)", ww.matrix[0][0], ww.matrix[1][1],
              ww.matrix[2][2]);
    tui_format_matrix3(b2, sizeof(b2), ww, "Iw=");
    if (r < h) {
        mvwprintw(dbg->main_win, r++, 0, "%.*s", w - 1, b2);
    }
    tui_format_vector3(b0, sizeof(b0), rb->force_accumulator, "Facc=");
    tui_format_vector3(b1, sizeof(b1), rb->torque_accumulator, "Tacc=");
    if (r < h) {
        mvwprintw(dbg->main_win, r++, 0, "%.*s %.*s", w - 1, b0, (int) (w - 1), b1);
    }
    vector3 mom = vector3_scaling(rb->velocity, rb->mass);
    vector3 am = math3_multiplication_vector3(ww, rb->angular_velocity);
    if (r < h) {
        mvwprintw(dbg->main_win, r++, 0, "P=(%+.3f,%+.3f,%+.3f)|P|=%.4f L=(%+.4f,%+.4f,%+.4f)|L|=%.5f KE=%.5f",
                  mom.x, mom.y, mom.z, vector3_length(mom), am.x, am.y, am.z, vector3_length(am),
                  rb_get_kinetic_energy(rb));
    }
    int isl = islands_body_island(world, rb);
    int has = (world->has_contact && idx < mpe_max_bodies) ? world->has_contact[idx] : -1;
    float rem = (world->ccd_time_remaining && idx < mpe_max_bodies) ? world->ccd_time_remaining[idx] : -1.0f;
    if (r < h) {
        mvwprintw(dbg->main_win, r++, 0, "sleepT=%.2f island=%d awake=%d hasContact=%d ccdRem=%.5f effInvM=%.6f",
                  rb->sleep_timer, isl, islands_body_awake(world, rb) ? 1 : 0, has, rem,
                  rigidbody_effective_inv_mass(rb));
    }
    tui_format_vector3(b0, sizeof(b0), rb->cached_axes[0], "axX=");
    tui_format_vector3(b1, sizeof(b1), rb->cached_axes[1], "axY=");
    if (r < h) {
        mvwprintw(dbg->main_win, r++, 0, "%.*s %.*s", w - 1, b0, (int) (w - 1), b1);
    }
    (void) h;
    wrefresh(dbg->main_win);
}

/* Count active joints of both pools. */
static int tui_joint_total(physics_world *world) {
    if (!world) {
        return 0;
    }
    int n = 0;
    for (int i = 0; i < mpe_max_joints; i++) {
        if (world->spring_joints[i].is_active) {
            n++;
        }
    }
    for (int i = 0; i < mpe_max_joints; i++) {
        if (world->revolute_constraints[i].is_active) {
            n++;
        }
    }
    return n;
}

/* Describe joint #sel (springs first, then constraints). kind: 0=spring,1=constraint. */
static bool tui_joint_at(physics_world *world, int sel, int *kind, int *slot) {
    if (!world || sel < 0) {
        return false;
    }
    int n = 0;
    for (int i = 0; i < mpe_max_joints; i++) {
        if (world->spring_joints[i].is_active) {
            if (n == sel) {
                *kind = 0;
                *slot = i;
                return true;
            }
            n++;
        }
    }
    for (int i = 0; i < mpe_max_joints; i++) {
        if (world->revolute_constraints[i].is_active) {
            if (n == sel) {
                *kind = 1;
                *slot = i;
                return true;
            }
            n++;
        }
    }
    return false;
}

static void tui_joint_line(physics_world *world, int n, char *buf, size_t sz) {
    int kind, slot;
    if (!tui_joint_at(world, n, &kind, &slot)) {
        snprintf(buf, sz, "--");
        return;
    }
    if (kind == 0) {
        spring_joint *sj = &world->spring_joints[slot];
        int ia = tui_index_by_id(world, sj->object_id_a);
        int ib = tui_index_by_id(world, sj->object_id_b);
        float len = -1.0f;
        if (ia >= 0 && ib >= 0) {
            len = vector3_length(
                vector3_subtraction(world->bodies[ib].position, world->bodies[ia].position));
        }
        snprintf(buf, sz, "SPR [%d] A=%d(id%u) B=%d(id%u) L0=%.3f len=%.3f ext=%+.3f k=%.2f c=%.2f", slot, ia,
                 sj->object_id_a, ib, sj->object_id_b, sj->equilibrium_length, len, len - sj->equilibrium_length,
                 sj->spring_constant, sj->damping_coefficient);
    } else {
        constraint *c = &world->revolute_constraints[slot];
        int ia = tui_index_by_id(world, c->body_id_a);
        int ib = tui_index_by_id(world, c->body_id_b);
        snprintf(buf, sz, "%s [%d] A=%d(id%u) B=%d(id%u)", tui_constraint_str(c->type), slot, ia, c->body_id_a,
                 ib, c->body_id_b);
    }
}

void tui_render_joint_detail(tui_debugger_t *dbg) {
    if (!dbg || !dbg->main_win) {
        return;
    }
    werase(dbg->main_win);
    int h, w;
    getmaxyx(dbg->main_win, h, w);
    physics_world *world = dbg->world;
    int total = tui_joint_total(world);
    if (dbg->selected_joint >= total && total > 0) {
        dbg->selected_joint = total - 1;
    }
    int list_h = dbg->detail_win ? h - 9 : h - 1;
    if (list_h < 2) {
        list_h = h - 1;
    }
    mvwprintw(dbg->main_win, 0, 0, "JOINTS total=%d (springs + revolute/fixed/prismatic/distance/rope)", total);
    char line[256];
    int row = 1;
    for (int n = 0; n < total && row < list_h; n++) {
        tui_joint_line(world, n, line, sizeof(line));
        if (n == dbg->selected_joint) {
            wattron(dbg->main_win, A_REVERSE);
        }
        mvwprintw(dbg->main_win, row++, 0, "%.*s", w - 1, line);
        if (n == dbg->selected_joint) {
            wattroff(dbg->main_win, A_REVERSE);
        }
    }
    wrefresh(dbg->main_win);

    /* Selected joint full characteristics in the detail window. */
    if (!dbg->detail_win || !world) {
        return;
    }
    werase(dbg->detail_win);
    int dh, dw;
    getmaxyx(dbg->detail_win, dh, dw);
    int kind, slot;
    if (dh < 4 || !tui_joint_at(world, dbg->selected_joint, &kind, &slot)) {
        mvwprintw(dbg->detail_win, 0, 0, "no joint selected");
        wrefresh(dbg->detail_win);
        return;
    }
    int r = 0;
    if (kind == 0) {
        spring_joint *sj = &world->spring_joints[slot];
        int ia = tui_index_by_id(world, sj->object_id_a);
        int ib = tui_index_by_id(world, sj->object_id_b);
        mvwprintw(dbg->detail_win, r++, 0, "SPRING slot=%d L0=%.4f k=%.4f c=%.4f", slot, sj->equilibrium_length,
                  sj->spring_constant, sj->damping_coefficient);
        if (ia >= 0 && ib >= 0 && r < dh) {
            rigidbody *a = &world->bodies[ia];
            rigidbody *b = &world->bodies[ib];
            vector3 d = vector3_subtraction(b->position, a->position);
            float len = vector3_length(d);
            vector3 axis = len > 1e-9f ? vector3_scaling(d, 1.0f / len) : (vector3){1.0f, 0.0f, 0.0f};
            float ext = len - sj->equilibrium_length;
            vector3 rv = vector3_subtraction(b->velocity, a->velocity);
            float vr = vector3_dot(rv, axis);
            mvwprintw(dbg->detail_win, r++, 0, "A[%d]=(%+.3f,%+.3f,%+.3f) B[%d]=(%+.3f,%+.3f,%+.3f)", ia,
                      a->position.x, a->position.y, a->position.z, ib, b->position.x, b->position.y, b->position.z);
            if (r < dh) {
                mvwprintw(dbg->detail_win, r++, 0, "len=%.4f ext=%+.4f axis=(%+.3f,%+.3f,%+.3f) vrel=%+.4f", len,
                          ext, axis.x, axis.y, axis.z, vr);
            }
            if (r < dh) {
                mvwprintw(dbg->detail_win, r++, 0, "Hooke(raw)=%+.4f damp(raw)=%+.4f [unsaturated estimate]", ext *
                          sj->spring_constant, vr * sj->damping_coefficient);
            }
        } else if (r < dh) {
            mvwprintw(dbg->detail_win, r++, 0, "endpoint missing (transient gap)");
        }
    } else {
        constraint *c = &world->revolute_constraints[slot];
        int ia = tui_index_by_id(world, c->body_id_a);
        int ib = tui_index_by_id(world, c->body_id_b);
        mvwprintw(dbg->detail_win, r++, 0, "%s slot=%d A=%d B=%d", tui_constraint_str(c->type), slot, ia, ib);
        if (ia < 0 || ib < 0) {
            if (r < dh) {
                mvwprintw(dbg->detail_win, r++, 0, "endpoint missing");
            }
        } else {
            rigidbody *a = &world->bodies[ia];
            rigidbody *b = &world->bodies[ib];
            char b0[128], b1[128];
            if (c->type == constraint_revolute) {
                vector3 wa = tui_anchor_world(a, c->p.revolute.anchor_a);
                vector3 wb = tui_anchor_world(b, c->p.revolute.anchor_b);
                vector3 err = vector3_subtraction(wb, wa);
                vector3 axw = vector4_rotate_to_vector3(a->orientation,
                                                        vector3_normalisation(c->p.revolute.axis_a));
                vector3 relw = vector3_subtraction(b->angular_velocity, a->angular_velocity);
                tui_format_vector3(b0, sizeof(b0), wa, "wA=");
                tui_format_vector3(b1, sizeof(b1), wb, "wB=");
                if (r < dh) {
                    mvwprintw(dbg->detail_win, r++, 0, "%.*s", dw - 1, b0);
                }
                if (r < dh) {
                    mvwprintw(dbg->detail_win, r++, 0, "%.*s", dw - 1, b1);
                }
                if (r < dh) {
                    mvwprintw(dbg->detail_win, r++, 0, "err=|%.5f| axis=(%+.3f,%+.3f,%+.3f) wAx=%+.4f", vector3_length(
                                  err), axw.x, axw.y, axw.z, vector3_dot(relw, axw));
                }
                if (r < dh) {
                    mvwprintw(dbg->detail_win, r++, 0, "angle=%+.2fdeg init=%d lim[%s %.1f..%.1f]deg motor[%s v=%+.2f max=%.2f]",
                              c->p.revolute.accumulated_angle * 180.0f / (float) M_PI,
                              c->p.revolute.angle_initialized ? 1 : 0, c->p.revolute.limits_enabled ? "on" : "off",
                              c->p.revolute.limit_min_rad * 180.0f / (float) M_PI,
                              c->p.revolute.limit_max_rad * 180.0f / (float) M_PI,
                              c->p.revolute.motor_enabled ? "on" : "off", c->p.revolute.motor_target_speed,
                              c->p.revolute.motor_max_torque);
                }
            } else if (c->type == constraint_prismatic) {
                vector3 axw = vector4_rotate_to_vector3(a->orientation,
                                                        vector3_normalisation(c->p.prismatic.axis_a));
                vector3 ra = tui_anchor_world(a, c->p.prismatic.anchor_a);
                vector3 rb2 = tui_anchor_world(b, c->p.prismatic.anchor_b);
                float cur = vector3_dot(vector3_subtraction(rb2, ra), axw);
                if (r < dh) {
                    mvwprintw(dbg->detail_win, r++, 0, "axis=(%+.3f,%+.3f,%+.3f) cur=%+.4f track=%+.4f", axw.x,
                              axw.y, axw.z, cur, c->p.prismatic.accumulated_position);
                }
                if (r < dh) {
                    mvwprintw(dbg->detail_win, r++, 0, "lim[%s %.3f..%.3f] motor[%s v=%+.2f F=%.2f] init=%d",
                              c->p.prismatic.limits_enabled ? "on" : "off", c->p.prismatic.limit_min,
                              c->p.prismatic.limit_max, c->p.prismatic.motor_enabled ? "on" : "off",
                              c->p.prismatic.motor_target_speed, c->p.prismatic.motor_max_force,
                              c->p.prismatic.position_initialized ? 1 : 0);
                }
            } else if (c->type == constraint_fixed) {
                vector3 wa = tui_anchor_world(a, c->p.fixed.anchor_a);
                vector3 wb = tui_anchor_world(b, c->p.fixed.anchor_b);
                if (r < dh) {
                    mvwprintw(dbg->detail_win, r++, 0, "wA=(%+.3f,%+.3f,%+.3f) wB=(%+.3f,%+.3f,%+.3f) gap=%.5f",
                              wa.x, wa.y, wa.z, wb.x, wb.y, wb.z, vector3_length(vector3_subtraction(wb, wa)));
                }
            } else if (c->type == constraint_distance) {
                vector3 wa = tui_anchor_world(a, c->p.distance.anchor_a);
                vector3 wb = tui_anchor_world(b, c->p.distance.anchor_b);
                float d = vector3_length(vector3_subtraction(wb, wa));
                if (r < dh) {
                    mvwprintw(dbg->detail_win, r++, 0, "dist=%.4f rest=%.4f err=%+.4f", d,
                              c->p.distance.rest_length, d - c->p.distance.rest_length);
                }
            } else if (c->type == constraint_rope) {
                vector3 wa = tui_anchor_world(a, c->p.rope.anchor_a);
                vector3 wb = tui_anchor_world(b, c->p.rope.anchor_b);
                float d = vector3_length(vector3_subtraction(wb, wa));
                if (r < dh) {
                    mvwprintw(dbg->detail_win, r++, 0, "dist=%.4f max=%.4f %s", d, c->p.rope.rest_length,
                              d > c->p.rope.rest_length ? "TAUT" : "slack");
                }
            }
        }
    }
    (void) dw;
    wrefresh(dbg->detail_win);
}

void tui_render_scene_graph(tui_debugger_t *dbg) {
    if (!dbg || !dbg->main_win) {
        return;
    }
    werase(dbg->main_win);
    int h, w;
    getmaxyx(dbg->main_win, h, w);
    physics_world *world = dbg->world;
    if (!world || world->body_count <= 0) {
        mvwprintw(dbg->main_win, 0, 0, "no bodies");
        wrefresh(dbg->main_win);
        return;
    }
    int n = world->body_count;
    int r = 0;
    mvwprintw(dbg->main_win, r++, 0, "RELATIVE POSITIONS (pairwise distances, showing up to %d pairs)",
              h > 4 ? h - 4 : 0);
    /* Closest / farthest over all pairs (cap scan for huge scenes). */
    int cap = n > 96 ? 96 : n;
    float cmin = 1e30f, cmax = -1e30f;
    int cmini = -1, cminj = -1, cmaxi = -1, cmaxj = -1;
    for (int i = 0; i < cap; i++) {
        for (int j = i + 1; j < cap; j++) {
            float d = vector3_length(vector3_subtraction(world->bodies[j].position, world->bodies[i].position));
            if (d < cmin) {
                cmin = d;
                cmini = i;
                cminj = j;
            }
            if (d > cmax) {
                cmax = d;
                cmaxi = i;
                cmaxj = j;
            }
        }
    }
    if (r < h) {
        mvwprintw(dbg->main_win, r++, 0, "closest: [%d]<->[%d] d=%.4f   farthest: [%d]<->[%d] d=%.4f%s", cmini, cminj,
                  cmin, cmaxi, cmaxj, cmax, n > cap ? " (first 96 bodies)" : "");
    }
    if (r < h) {
        mvwprintw(dbg->main_win, r++, 0, "  A   B      dist        dx        dy        dz   islA islB");
    }
    int shown = 0;
    for (int i = 0; i < cap && r < h; i++) {
        for (int j = i + 1; j < cap && r < h; j++) {
            vector3 d = vector3_subtraction(world->bodies[j].position, world->bodies[i].position);
            float dist = vector3_length(d);
            int ia = islands_body_island(world, &world->bodies[i]);
            int ib = islands_body_island(world, &world->bodies[j]);
            mvwprintw(dbg->main_win, r++, 0, "%3d %3d %9.4f %+.4f %+.4f %+.4f %4d %4d", i, j, dist, d.x, d.y,
                      d.z, ia, ib);
            shown++;
            if (shown >= h - 4 && h > 4) {
                break;
            }
        }
    }
    (void) w;
    wrefresh(dbg->main_win);
}

void tui_render_help(tui_debugger_t *dbg) {
    if (!dbg || !dbg->main_win) {
        return;
    }
    werase(dbg->main_win);
    const char *lines[] = {
        "MPE-TUI — terminal debugger and debug-output suite",
        "",
        "SCREENS: 1 overview  2 object+math  3 joints  4 graph  5 help   (Tab cycles)",
        "SELECT:  j/k or Up/Down, PgUp/PgDn, Home/End   FILTER: / type, Enter/Esc done, c clear",
        "TIME:    Space pause/resume, s single-step, +/- time scale (0.125x..4x)",
        "",
        "MATH CONVENTIONS (inspection only):",
        "  quat (w,x,y,z), unit, world-frame left-multiplied rotor",
        "  euler XYZ degrees: ex=atan2(R21,R22) ey=asin(-R20) ez=atan2(R10,R00)",
        "  I_world = R * I_local * R^T (full tensor shown in object view)",
        "  P = m*v,  L = I_world * w,  KE = translational + rotational",
        "  joint anchors: world = position + q * local",
        "  rope: dist>max TAUT else slack; spring forces are raw Hooke+damp",
        "",
        "NON-INTERACTIVE SUITE (no TTY needed):",
        "  mpe-tui --snapshot [ticks] [--scene NAME]   one full state dump",
        "  mpe-tui --stream T [--every K] [--scene N]  dumps every K ticks",
        "  scenes: demo tower pendulum springlab f10 (all scriptable/diffable)",
        NULL,
    };
    int h, w;
    getmaxyx(dbg->main_win, h, w);
    for (int i = 0; lines[i] && i < h; i++) {
        mvwprintw(dbg->main_win, i, 0, "%.*s", w - 1, lines[i]);
    }
    wrefresh(dbg->main_win);
}

/* ------------------------------------------------------------------ */
/* Frame                                                               */
/* ------------------------------------------------------------------ */

void tui_debugger_render(tui_debugger_t *dbg) {
    if (!dbg) {
        return;
    }
    int H, W;
    getmaxyx(stdscr, H, W);
    dbg->term_height = H;
    dbg->term_width = W;
    if (H < 10 || W < 50) {
        clear();
        mvprintw(0, 0, "MPE-TUI needs >=50x10 (have %dx%d). Resize or 'q'.", W, H);
        clrtoeol();
        refresh();
        return;
    }
    tui_draw_header(dbg);

    /* Layout: header(1) + body + status(1). Body splits into main/sidebar
     * when wide, plus a detail strip in joint mode. */
    int body_y = 1, body_h = H - 2, side_w = (W >= 110) ? 38 : 0;
    int main_w = W - side_w;
    bool want_detail = (dbg->mode == TUI_MODE_JOINT_DETAIL);
    int detail_h = (want_detail && body_h > 14) ? 7 : 0;
    int main_h = body_h - detail_h;

    if (!dbg->main_win) {
        dbg->main_win = newwin(main_h, main_w, body_y, 0);
        dbg->sidebar_win = side_w ? newwin(main_h, side_w, body_y, main_w) : NULL;
        dbg->detail_win = detail_h ? newwin(detail_h, W, body_y + main_h, 0) : NULL;
        dbg->status_win = newwin(1, W, H - 1, 0);
    } else {
        wresize(dbg->main_win, main_h, main_w);
        mvwin(dbg->main_win, body_y, 0);
        if (side_w) {
            if (!dbg->sidebar_win) {
                dbg->sidebar_win = newwin(main_h, side_w, body_y, main_w);
            } else {
                wresize(dbg->sidebar_win, main_h, side_w);
                mvwin(dbg->sidebar_win, body_y, main_w);
            }
        } else if (dbg->sidebar_win) {
            delwin(dbg->sidebar_win);
            dbg->sidebar_win = NULL;
        }
        if (detail_h) {
            if (!dbg->detail_win) {
                dbg->detail_win = newwin(detail_h, W, body_y + main_h, 0);
            } else {
                wresize(dbg->detail_win, detail_h, W);
                mvwin(dbg->detail_win, body_y + main_h, 0);
            }
        } else if (dbg->detail_win) {
            delwin(dbg->detail_win);
            dbg->detail_win = NULL;
        }
        wresize(dbg->status_win, 1, W);
        mvwin(dbg->status_win, H - 1, 0);
    }
    /* Separators. */
    if (side_w) {
        for (int y = body_y; y < body_y + main_h; y++) {
            mvaddch(y, main_w - 1, ACS_VLINE);
        }
    }
    if (detail_h) {
        mvhline(body_y + main_h, 0, ACS_HLINE, W);
    }
    refresh();

    switch (dbg->mode) {
        case TUI_MODE_OVERVIEW:
            tui_render_overview(dbg);
            break;
        case TUI_MODE_OBJECT_DETAIL:
            tui_render_object_detail(dbg);
            break;
        case TUI_MODE_JOINT_DETAIL:
            tui_render_joint_detail(dbg);
            break;
        case TUI_MODE_SCENE_GRAPH:
            tui_render_scene_graph(dbg);
            break;
        case TUI_MODE_HELP:
        default:
            tui_render_help(dbg);
            break;
    }
    if (dbg->sidebar_win) {
        tui_render_sidebar(dbg);
    } else if (dbg->mode == TUI_MODE_OVERVIEW) {
        /* Narrow terminal: stats already visible? No — sidebar hidden.
         * Overview keeps the table; stats live in snapshot/dump. */
    }
    tui_render_status(dbg);
}

/* Plain-text helpers shared with tui_dump.c (no curses here). */
void tui_describe_body(physics_world *world, int idx, char *buf, size_t sz);

void tui_describe_body(physics_world *world, int idx, char *buf, size_t sz) {
    tui_body_line(world, idx, buf, sz);
}

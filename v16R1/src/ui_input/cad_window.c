/* MIG WP4 4.6c: the miniature CAD WINDOW. See cad_window.h.
 *
 * A real graphics window you sit inside and edit with — a GtkGLArea
 * viewport with its OWN self-contained GL setup (own inline-shader
 * program, own VAOs/VBOs built from unit meshes compiled in this TU,
 * own matrices): it never touches the main renderer's globals, shaders
 * or meshes, exactly like the 4.4 panel never touched the scene. The
 * window recipe is cloned from ui_input/mig_editor.c (plain GtkWindow,
 * transient for the main window, CSS keyed on the widget name, Esc
 * closes, destroy handler nulls the globals, never modal) and the GL
 * recipe from root_gtk.c (has_depth_buffer, allowed GL, realize ->
 * make_current + init, render -> viewport + draw).
 *
 * Nothing here calls the main renderer or the physics world; the
 * window drives the LANDED headless truth and the GATED view math:
 *   - cad_design + the whole verb set (cad/cad_editor.c) — the model;
 *   - cad_view_camera / ray-from-screen / pick_* / ray_plane
 *     (cad/cad_view_math.c) — the clicking and dragging.
 *
 * Interaction (WP4 4.6h + 4.6i + 4.6j). 4.6h/4.6i ported the engine's
 * MOUSE SEMANTICS (the drag_weld grab, the S4c weld-or-settle drop,
 * the engine's MMB delete — all from drag_weld.c / object_selector.c /
 * the dispatch, not invented). 4.6j completes it on the user's ruling:
 * "I want the CAD editor just like the controls of the actual MPE
 * engine itself BUT with cad specific keybinds and capabilities and
 * everything" — the window now runs the ENGINE'S OWN CONTROL SCHEME:
 * pointer-locked mouse-look, fly movement, crosshair actions
 * (input_control.c / simulation_camera.c / mouse_lock.c patterns).
 * The lock machinery is REUSED as-is (mouse_lock_enable/disable touch
 * no engine state), and the engine's per-frame delta poll is gated on
 * main_inputs.is_mouse_locked — which this window NEVER sets — so the
 * 4.6f isolation holds with zero engine-core changes:
 *   - UNLOCKED LMB: the engine's re-engage — the click ACQUIRES the
 *     pointer lock and nothing else (the S4b rule: the click that
 *     acquires the lock must not also grab). RMB picks under the
 *     cursor (miss deselects); MMB deletes the selection; wheel
 *     zooms.
 *   - LOCKED (pointer hidden, crosshair drawn): mouse-look steers the
 *     camera at the engine's steer_sensitivity (Wayland: the
 *     relative-pointer deltas drained once per tick — the
 *     input_control_poll_locked_mouse pattern; X11: motion-event
 *     deltas + the re-centre warp — the on_mouse_movements pattern).
 *     LMB grabs under the crosshair (the drag_weld_press port: pick
 *     selects, miss clears, grab at the hit depth, every-frame
 *     follow, floor clamp), LMB release runs the 4.6i DROP RULE
 *     (covering bounds touch/overlap = the pose JOINS the composite;
 *     free space = straight-down gravity settle to the highest
 *     supporter or the floor, landed EXACTLY); RMB selects under the
 *     crosshair; MMB deletes; WASD + Space/Shift FLY (the engine's
 *     debug-fly translate — W/S along the forward, A/D along the
 *     side, Space up / Shift down — at move_speed x0.2 = 5 m/s: the
 *     25 m/s default is tuned for the engine's 500 m arena, this
 *     scene is ~10 m; steering sensitivity is used unscaled); Esc
 *     frees the pointer (the engine's Esc convention). You can fly
 *     WHILE carrying (the engine's grab is camera-relative too).
 *   - Esc while unlocked closes the window — the two-stage engine
 *     convention (lock -> unlock -> close; the 4.6c close ruling
 *     preserved on the second stage).
 *   - CAD-SPECIFIC (the "but with cad capabilities" layer):
 *     1/2/3 add sphere/cube/cylinder at the spawn point, Delete/MMB
 *     delete, D duplicate (+0.5 m X), A array (3 copies, 0.5 m X), X/
 *     Y/Z rotate the selection +-15 degrees (Shift = -15), G toggles
 *     snap (0.25 m; new designs start ON), R toggles rot-snap
 *     (0 <-> 15 deg), M toggles the MAGNET (a carried part snaps to
 *     TOUCH the nearest other part along the line of centers at
 *     covering-sphere distance, within 0.25 m; the touch is written
 *     EXACTLY so the weld-on-drop rule holds), F drops the selection
 *     to REST (the settle's support seek — onto a stack or the floor,
 *     exact), S saves, L loads the spec selected in the dropdown, P
 *     spawns the design into the shared world (the 4.6e hookup; also
 *     the Spawn button). M/F/G/R/X/Y/Z/P/1/2/3/Delete work in BOTH
 *     pointer modes; D/A/S/L collide with the fly keys and run
 *     UNLOCKED only — engine controls win while the pointer is
 *     locked, which is the "just like the engine" half of the
 *     ruling. Shift+LMB while UNLOCKED stays the CURSOR precision
 *     fallback: pan on empty space, the y-locked work-plane carry on
 *     a part (the pose stays exactly where slid, NO settle —
 *     floating and interpenetrating assemblies the engine would
 *     never hold stay buildable on purpose).
 *   - The lock releases when the window loses keyboard focus (the
 *     engine's on_focus_out pattern) and on destroy. Typing-block
 *     ruling (WP4 4.6g) unchanged: while the name entry holds focus
 *     the CAD keys AND the fly latches are dead (typing must not
 *     spawn parts or fly the camera), only Esc works; the window
 *     opens viewport-focused and a click into the viewport returns
 *     the keys.
 *
 * Persistence is the frozen spec grammar staged through the shared
 * mig_spec_store (the terminal, microvim and the 4.4 panel all read
 * the same store): save = cad_design_to_spec -> canonical emit ->
 * temp file -> full parse round trip -> store add (write-on-success);
 * load = the selected store spec -> cad_design_from_spec.
 *
 * Connection ruling history (the user's calls): WP4 4.6c shipped this
 * window as DEAD CODE (no keybind, nothing called it). WP4 4.6d lifted
 * that: key 2 in the FREE state toggles the window through
 * simulation_input_dispatch (the resize-panel T pattern). WP4 4.6e
 * completed the connect: P / the Spawn button validate through
 * cad_build (the gated seam) and hand the RAW assembly-frame parts to
 * physics_world_add_composite — the body spawns at the design's COM,
 * i.e. exactly as drawn, because the CAD frame IS the world frame.
 * This is the ONLY TU of the 4.6 family that touches the physics
 * world; the editor model cannot mutate a world BY DESIGN, which is
 * what keeps it gateable (mpe_engine.h already carries
 * core/physics_world.h transitively — no new include).
 *
 * Input isolation (WP4 4.6f, the user's ruling): while the user is
 * inside this window, the MAIN engine window must not react to any
 * input. The mechanism: opening (or focusing) this window releases the
 * main window's pointer lock (the relative-pointer grab is GLOBAL —
 * while live, mouse motion over THIS window fed the main camera) and
 * the main window's focus-out clears every input latch; keys route to
 * whichever toplevel has focus, so the main engine sees none. The lock
 * re-engages on the next deliberate main-window click — main becomes
 * active again only when the user deliberately returns to it.
 *
 * Determinism: this is authoring-path math (a human holds the mouse);
 * sinf/cosf here are the documented cold-path ruling, the per-tick
 * solver ban is untouched. MATH_PI (math3d.h), never M_PI.
 */
#include "cad_window.h"
#include "../mpe_engine.h"
#include "../cad/cad_editor.h"
#include "../cad/cad_view_math.h"
#include "term_priv.h"
#include "mouse_lock.h"
#include <gtk/gtk.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ---- widget handles --------------------------------------------------- */

static GtkWidget *cw_window = NULL;
static GtkWidget *cw_gl_area = NULL;
static GtkWidget *cw_name_entry = NULL;
static GtkWidget *cw_spec_dropdown = NULL;
static GtkWidget *cw_status_label = NULL;
static guint cw_timeout_id = 0;

/* ---- the model + camera (window-owned statics; the design survives
 * close/reopen so a toggle never loses work) ---- */

static cad_design cw_design;
static cad_view_camera cw_camera;
static int cw_design_ready = 0;

/* Interaction modes: one motion at a time. WP4 4.6h split the old
 * CW_MODE_DRAG: CW_MODE_CARRY is the ENGINE grab (the drag_weld
 * precedent — grab at the hit point, the part rides the ray at the
 * grabbed depth, full 3D); CW_MODE_PLANE_DRAG keeps the old y-locked
 * work-plane slide for flat layout work (Shift+LMB). WP4 4.6i added
 * CW_MODE_SETTLING: the post-release gravity drop (the engine's
 * dropped-body rule — see cw_release_carry). WP4 4.6j REMOVED
 * CW_MODE_ORBIT: unlocked LMB now ACQUIRES the pointer lock (the
 * engine's re-engage rule) instead of orbiting; looking around is the
 * locked mouse-look. */
enum {
    CW_MODE_IDLE = 0,
    CW_MODE_PAN,        /* Shift+LMB on empty space */
    CW_MODE_CARRY,      /* LMB: engine-style ray carry (crosshair while locked) */
    CW_MODE_PLANE_DRAG, /* Shift+LMB on a part: y-locked plane move */
    CW_MODE_SETTLING    /* post-release gravity drop to rest */
};
static int cw_mode = CW_MODE_IDLE;
static double cw_last_x = 0.0;
static double cw_last_y = 0.0;
static int cw_drag_part = -1;     /* index while CARRY / PLANE_DRAG */
static float cw_drag_plane_y = 0.0f;  /* PLANE_DRAG: the work plane's height */
static vector3 cw_drag_offset = {0.0f, 0.0f, 0.0f}; /* part - grab point */
/* WP4 4.6h carry + placement state (window-local, like the camera —
 * the design struct stays pure model data). cw_drag_t = the grabbed
 * depth along the pick ray (drag_weld's grab-distance semantics);
 * cw_drag_floor_y = the part's oriented down-extent at grab time (the
 * carry never dips it under the floor); cw_magnet_on = the M toggle. */
static float cw_drag_t = 0.0f;
static float cw_drag_floor_y = 0.0f;
static int cw_magnet_on = 0;
/* WP4 4.6i: the tick-driven carry recomputes the target EVERY frame
 * (the drag_weld_tick analog — the engine's carry also runs from the
 * frame dispatch, not from motion events), so it needs the last
 * cursor position: zoom mid-carry keeps the part glued. */
static double cw_cursor_x = 0.0;
static double cw_cursor_y = 0.0;
/* WP4 4.6i: the SETTLE — a part released in free space falls straight
 * down (the engine zeroes the carried body's velocity every frame, so
 * its release is a pure vertical fall) and lands at rest on the floor
 * or the highest supporter below. One slot; a new press snaps a
 * falling part to its rest instantly. */
static int cw_settle_part = -1;
static float cw_settle_y = 0.0f;
static float cw_settle_v = 0.0f;
static float cw_settle_rest_y = 0.0f;
/* WP4 4.6j: the ENGINE'S pointer lock, held by THIS window.
 * cw_look_locked is window-local (main_inputs.is_mouse_locked stays
 * FALSE — that flag is the engine's own lock state and setting it
 * would feed CAD deltas to the main camera; the engine's poll never
 * sees this lock). The cw_key_* flags are the FLY latches (the
 * engine's on_keypress/on_key_released pattern: press sets, release
 * clears — polled by the tick, so key repeat never matters). */
static int cw_look_locked = 0;
static int cw_key_w = 0;
static int cw_key_a = 0;
static int cw_key_s = 0;
static int cw_key_d = 0;
static int cw_key_up = 0;    /* Space */
static int cw_key_down = 0;  /* Shift */

/* ---- GL state (this window's own context; rebuilt per open) ---- */

static GLuint cw_program = 0;
static GLint cw_u_viewproj = -1;
static GLint cw_u_model = -1;
static GLint cw_u_inv_scale = -1;
static GLint cw_u_color = -1;
static GLint cw_u_selected = -1;
static GLint cw_u_unlit = -1;
static GLuint cw_vao_sphere = 0;
static GLuint cw_vbo_sphere = 0;
static int cw_count_sphere = 0;
static GLuint cw_vao_cube = 0;
static GLuint cw_vbo_cube = 0;
static int cw_count_cube = 0;
static GLuint cw_vao_cylinder = 0;
static GLuint cw_vbo_cylinder = 0;
static int cw_count_cylinder = 0;
static GLuint cw_vao_grid = 0;
static GLuint cw_vbo_grid = 0;
static int cw_count_grid = 0;
static GLuint cw_vao_crosshair = 0;
static GLuint cw_vbo_crosshair = 0;
static int cw_count_crosshair = 0;

/* Vertex format: {pos3, normal3} interleaved (6 floats). The unit
 * meshes are built once per open into these static buffers:
 *   sphere: 12 rings x 16 sectors x 2 tris x 3 verts x 6 floats
 *   cube:   6 faces x 2 tris x 3 verts x 6 floats
 *   cyl:    24 segs x (2 side + 2 caps) tris x 3 verts x 6 floats
 *   grid:   21 lines x 2 axes x 2 verts x 6 floats
 *   cross:  2 lines x 2 verts x 6 floats (NDC, 4.6j) */
#define CW_SPHERE_FLOATS (12 * 16 * 2 * 3 * 6)
#define CW_CUBE_FLOATS (6 * 2 * 3 * 6)
#define CW_CYLINDER_FLOATS (24 * 4 * 3 * 6)
#define CW_GRID_FLOATS (21 * 2 * 2 * 6)
#define CW_CROSSHAIR_FLOATS (2 * 2 * 6)
static float cw_mesh_sphere [CW_SPHERE_FLOATS];
static float cw_mesh_cube [CW_CUBE_FLOATS];
static float cw_mesh_cylinder [CW_CYLINDER_FLOATS];
static float cw_mesh_grid [CW_GRID_FLOATS];
static float cw_mesh_crosshair [CW_CROSSHAIR_FLOATS];

/* ---- shaders (inline string literals: no file dependencies, the
 * window's own program — the main renderer's shader files are
 * untouched) ---- */

static const char *CW_VERT_SRC =
    "#version 330 core\n"
    "layout (location = 0) in vec3 a_pos;\n"
    "layout (location = 1) in vec3 a_normal;\n"
    "uniform mat4 u_viewproj;\n"
    "uniform mat4 u_model;\n"
    "uniform vec3 u_inv_scale;\n"
    "out vec3 v_normal;\n"
    "void main () {\n"
    "    gl_Position = u_viewproj * u_model * vec4 (a_pos, 1.0);\n"
    "    /* exact axis-aligned-scale normal fixup: world normal =\n"
    "     * R * diag(1/s) * n (normalized in the fragment stage) */\n"
    "    v_normal = mat3 (u_model) * (u_inv_scale * a_normal);\n"
    "}\n";

static const char *CW_FRAG_SRC =
    "#version 330 core\n"
    "in vec3 v_normal;\n"
    "uniform vec3 u_color;\n"
    "uniform float u_selected;\n"
    "uniform float u_unlit;\n"
    "out vec4 frag;\n"
    "void main () {\n"
    "    vec3 n = normalize (v_normal);\n"
    "    vec3 light = normalize (vec3 (0.4, 0.85, 0.35));\n"
    "    float diff = dot (n, light) * 0.55 + 0.45;\n"
    "    vec3 col = u_color * diff;\n"
    "    if (u_selected > 0.5) { col = mix (col, vec3 (1.0, 0.85, 0.25), 0.4); }\n"
    "    if (u_unlit > 0.5) { col = u_color; }\n"
    "    frag = vec4 (col, 1.0);\n"
    "}\n";

/* ---- mesh fill helpers ------------------------------------------------ */

static int cw_vert (float *out, int n, const float p [3], const float nrm [3]) {
    out [n] = p [0]; out [n + 1] = p [1]; out [n + 2] = p [2];
    out [n + 3] = nrm [0]; out [n + 4] = nrm [1]; out [n + 5] = nrm [2];
    return n + 6;
}

static int cw_tri (float *out, int n, const float a [3], const float b [3],
                   const float c [3], const float na [3], const float nb [3],
                   const float nc [3]) {
    n = cw_vert (out, n, a, na);
    n = cw_vert (out, n, b, nb);
    return cw_vert (out, n, c, nc);
}

/* Unit sphere, radius 1 about the origin (normals = positions). */
static int cw_fill_sphere (float *out) {
    const int lat_rings = 12;
    const int lon_sectors = 16;
    int n = 0;
    for (int i = 0; i < lat_rings; i++) {
        for (int j = 0; j < lon_sectors; j++) {
            const float p0 = ((float) i / (float) lat_rings) * (float) MATH_PI -
                             (float) MATH_PI * 0.5f;
            const float p1 = ((float) (i + 1) / (float) lat_rings) * (float) MATH_PI -
                             (float) MATH_PI * 0.5f;
            const float t0 = ((float) j / (float) lon_sectors) * (float) MATH_PI * 2.0f;
            const float t1 = ((float) (j + 1) / (float) lon_sectors) * (float) MATH_PI * 2.0f;
            const float v00 [3] = {cosf (p0) * sinf (t0), sinf (p0), cosf (p0) * cosf (t0)};
            const float v01 [3] = {cosf (p0) * sinf (t1), sinf (p0), cosf (p0) * cosf (t1)};
            const float v10 [3] = {cosf (p1) * sinf (t0), sinf (p1), cosf (p1) * cosf (t0)};
            const float v11 [3] = {cosf (p1) * sinf (t1), sinf (p1), cosf (p1) * cosf (t1)};
            n = cw_tri (out, n, v00, v10, v11, v00, v10, v11);
            n = cw_tri (out, n, v00, v11, v01, v00, v11, v01);
        }
    }
    return n / 6;
}

/* Unit cube, half-extents 1 (verts at +-1), flat face normals. */
static int cw_fill_cube (float *out) {
    static const float faces [6] [4] [3] = {
        {{1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, -1.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, -1.0f, 1.0f}},
        {{-1.0f, -1.0f, 1.0f}, {-1.0f, 1.0f, 1.0f}, {-1.0f, 1.0f, -1.0f}, {-1.0f, -1.0f, -1.0f}},
        {{-1.0f, 1.0f, -1.0f}, {-1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, -1.0f}},
        {{-1.0f, -1.0f, 1.0f}, {-1.0f, -1.0f, -1.0f}, {1.0f, -1.0f, -1.0f}, {1.0f, -1.0f, 1.0f}},
        {{-1.0f, -1.0f, 1.0f}, {1.0f, -1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {-1.0f, 1.0f, 1.0f}},
        {{1.0f, -1.0f, -1.0f}, {-1.0f, -1.0f, -1.0f}, {-1.0f, 1.0f, -1.0f}, {1.0f, 1.0f, -1.0f}}
    };
    static const float normals [6] [3] = {
        {1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
        {0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f}
    };
    int n = 0;
    for (int f = 0; f < 6; f++) {
        n = cw_tri (out, n, faces [f] [0], faces [f] [1], faces [f] [2],
                    normals [f], normals [f], normals [f]);
        n = cw_tri (out, n, faces [f] [0], faces [f] [2], faces [f] [3],
                    normals [f], normals [f], normals [f]);
    }
    return n / 6;
}

/* Unit cylinder: radius 1 in the Y/Z plane, half length 1 along X —
 * the axle is LOCAL X (MPE_FTC_090). Smooth sides, fanned end caps. */
static int cw_fill_cylinder (float *out) {
    const int segs = 24;
    int n = 0;
    for (int j = 0; j < segs; j++) {
        const float a0 = ((float) j / (float) segs) * (float) MATH_PI * 2.0f;
        const float a1 = ((float) (j + 1) / (float) segs) * (float) MATH_PI * 2.0f;
        const float c0 [2] = {cosf (a0), sinf (a0)}; /* (y, z) on the ring */
        const float c1 [2] = {cosf (a1), sinf (a1)};
        const float n00 [3] = {0.0f, c0 [0], c0 [1]};
        const float n01 [3] = {0.0f, c0 [0], c0 [1]};
        const float n10 [3] = {0.0f, c1 [0], c1 [1]};
        const float n11 [3] = {0.0f, c1 [0], c1 [1]};
        const float p00 [3] = {-1.0f, c0 [0], c0 [1]};
        const float p01 [3] = {1.0f, c0 [0], c0 [1]};
        const float p10 [3] = {-1.0f, c1 [0], c1 [1]};
        const float p11 [3] = {1.0f, c1 [0], c1 [1]};
        n = cw_tri (out, n, p00, p10, p11, n00, n10, n11);
        n = cw_tri (out, n, p00, p11, p01, n00, n11, n01);
    }
    for (int cap = 0; cap < 2; cap++) {
        const float x = (cap == 0) ? 1.0f : -1.0f;
        const float cn [3] = {x, 0.0f, 0.0f};
        const float center [3] = {x, 0.0f, 0.0f};
        for (int j = 0; j < segs; j++) {
            const float a0 = ((float) j / (float) segs) * (float) MATH_PI * 2.0f;
            const float a1 = ((float) (j + 1) / (float) segs) * (float) MATH_PI * 2.0f;
            const float r0 [3] = {x, cosf (a0), sinf (a0)};
            const float r1 [3] = {x, cosf (a1), sinf (a1)};
            n = cw_tri (out, n, center, r0, r1, cn, cn, cn);
        }
    }
    return n / 6;
}

/* The floor grid the window drags on: y = 0 lines, x and z from
 * -10..10 step 1 (normals unused — drawn unlit). */
static int cw_fill_grid (float *out) {
    const float nrm [3] = {0.0f, 1.0f, 0.0f};
    int n = 0;
    for (int k = -10; k <= 10; k++) {
        const float f = (float) k;
        const float a [3] = {f, 0.0f, -10.0f};
        const float b [3] = {f, 0.0f, 10.0f};
        const float c [3] = {-10.0f, 0.0f, f};
        const float d [3] = {10.0f, 0.0f, f};
        n = cw_vert (out, n, a, nrm);
        n = cw_vert (out, n, b, nrm);
        n = cw_vert (out, n, c, nrm);
        n = cw_vert (out, n, d, nrm);
    }
    return n / 6;
}

/* The crosshair shown while the pointer is LOCKED (4.6j): two short
 * NDC lines through (0, 0) — with identity u_viewproj + u_model the
 * positions pass straight through as clip coords, so the crosshair is
 * always exactly at the screen center no matter the camera. Normals
 * unused (drawn unlit). */
static int cw_fill_crosshair (float *out) {
    const float nrm [3] = {0.0f, 0.0f, 1.0f};
    const float h = 0.045f;
    const float a [3] = {-h, 0.0f, 0.0f};
    const float b [3] = {h, 0.0f, 0.0f};
    const float c [3] = {0.0f, -h, 0.0f};
    const float d [3] = {0.0f, h, 0.0f};
    int n = 0;
    n = cw_vert (out, n, a, nrm);
    n = cw_vert (out, n, b, nrm);
    n = cw_vert (out, n, c, nrm);
    n = cw_vert (out, n, d, nrm);
    return n / 6;
}

/* ---- GL build ---------------------------------------------------------- */

static GLuint cw_compile_shader (GLenum type, const char *source) {
    GLuint shader = glCreateShader (type);
    glShaderSource (shader, 1, &source, NULL);
    glCompileShader (shader);
    GLint ok = 0;
    glGetShaderiv (shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log [1024];
        GLsizei len = 0;
        glGetShaderInfoLog (shader, (GLsizei) sizeof (log), &len, log);
        printf ("[cad] shader compile failed: %.*s\n", (int) len, log);
        glDeleteShader (shader);
        return 0;
    }
    return shader;
}

static GLuint cw_build_program (void) {
    GLuint vert = cw_compile_shader (GL_VERTEX_SHADER, CW_VERT_SRC);
    GLuint frag = cw_compile_shader (GL_FRAGMENT_SHADER, CW_FRAG_SRC);
    if ((vert == 0) || (frag == 0)) {
        if (vert) { glDeleteShader (vert); }
        if (frag) { glDeleteShader (frag); }
        return 0;
    }
    GLuint program = glCreateProgram ();
    glAttachShader (program, vert);
    glAttachShader (program, frag);
    glLinkProgram (program);
    glDeleteShader (vert);
    glDeleteShader (frag);
    GLint ok = 0;
    glGetProgramiv (program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log [1024];
        GLsizei len = 0;
        glGetProgramInfoLog (program, (GLsizei) sizeof (log), &len, log);
        printf ("[cad] program link failed: %.*s\n", (int) len, log);
        glDeleteProgram (program);
        return 0;
    }
    return program;
}

static void cw_make_vao (const float *data, int floats, GLuint *vao,
                         GLuint *vbo, int *verts) {
    glGenVertexArrays (1, vao);
    glGenBuffers (1, vbo);
    glBindVertexArray (*vao);
    glBindBuffer (GL_ARRAY_BUFFER, *vbo);
    glBufferData (GL_ARRAY_BUFFER,
                  (GLsizeiptr) ((GLsizeiptr) floats * (GLsizeiptr) sizeof (float)),
                  data, GL_STATIC_DRAW);
    glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE,
                           (GLsizei) (6 * sizeof (float)), (const void *) 0);
    glEnableVertexAttribArray (0);
    glVertexAttribPointer (1, 3, GL_FLOAT, GL_FALSE,
                           (GLsizei) (6 * sizeof (float)), (const void *) 12);
    glEnableVertexAttribArray (1);
    glBindVertexArray (0);
    glBindBuffer (GL_ARRAY_BUFFER, 0);
    *verts = floats / 6;
}

/* ---- matrices (column-major float[16], GL convention; self-contained
 * helpers — the window's own math, no renderer globals) ---- */

typedef float cw_mat4 [16];

static void cw_mat_identity (cw_mat4 m) {
    for (int i = 0; i < 16; i++) { m [i] = 0.0f; }
    m [0] = 1.0f; m [5] = 1.0f; m [10] = 1.0f; m [15] = 1.0f;
}

static void cw_mat_multiply (cw_mat4 out, const cw_mat4 a, const cw_mat4 b) {
    cw_mat4 result;
    for (int col = 0; col < 4; col++) {
        for (int row = 0; row < 4; row++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++) {
                sum += a [k * 4 + row] * b [col * 4 + k];
            }
            result [col * 4 + row] = sum;
        }
    }
    for (int i = 0; i < 16; i++) { out [i] = result [i]; }
}

static void cw_mat_perspective (cw_mat4 m, float fov_y_deg, float aspect,
                                float near_z, float far_z) {
    for (int i = 0; i < 16; i++) { m [i] = 0.0f; }
    const float f = 1.0f / tanf (fov_y_deg * (float) MATH_PI / 180.0f * 0.5f);
    m [0] = f / aspect;
    m [5] = f;
    m [10] = (far_z + near_z) / (near_z - far_z);
    m [11] = -1.0f;
    m [14] = (2.0f * far_z * near_z) / (near_z - far_z);
}

/* View matrix straight from the GATED camera basis (cad_view_forward/
 * right/up/eye): the window re-uses the math the suite pins. */
static void cw_mat_look_at (cw_mat4 m, const cad_view_camera *camera) {
    const vector3 eye = cad_view_eye (camera);
    const vector3 f = cad_view_forward (camera);
    const vector3 r = cad_view_right (camera);
    const vector3 u = cad_view_up (camera);
    m [0] = r.x; m [4] = r.y; m [8] = r.z;
    m [1] = u.x; m [5] = u.y; m [9] = u.z;
    m [2] = -f.x; m [6] = -f.y; m [10] = -f.z;
    m [3] = 0.0f; m [7] = 0.0f; m [11] = 0.0f;
    m [12] = -vector3_dot (r, eye);
    m [13] = -vector3_dot (u, eye);
    m [14] = vector3_dot (f, eye);
    m [15] = 1.0f;
}

/* Model matrix T(pos) * R(quat) * S(scale) built directly (w-first
 * quat -> rotation matrix, per-axis scale, no matrix stack). */
static void cw_mat_model (cw_mat4 m, vector3 position, vector4 orientation,
                          vector3 scale) {
    const float w = orientation.w;
    const float x = orientation.x;
    const float y = orientation.y;
    const float z = orientation.z;
    const float r00 = 1.0f - 2.0f * (y * y + z * z);
    const float r01 = 2.0f * (x * y - w * z);
    const float r02 = 2.0f * (x * z + w * y);
    const float r10 = 2.0f * (x * y + w * z);
    const float r11 = 1.0f - 2.0f * (x * x + z * z);
    const float r12 = 2.0f * (y * z - w * x);
    const float r20 = 2.0f * (x * z - w * y);
    const float r21 = 2.0f * (y * z + w * x);
    const float r22 = 1.0f - 2.0f * (x * x + y * y);
    /* column 0 = R * scale.x, etc. */
    m [0] = r00 * scale.x; m [1] = r10 * scale.x; m [2] = r20 * scale.x; m [3] = 0.0f;
    m [4] = r01 * scale.y; m [5] = r11 * scale.y; m [6] = r21 * scale.y; m [7] = 0.0f;
    m [8] = r02 * scale.z; m [9] = r12 * scale.z; m [10] = r22 * scale.z; m [11] = 0.0f;
    m [12] = position.x; m [13] = position.y; m [14] = position.z; m [15] = 1.0f;
}

/* ---- widget pixels -> the camera's aspect ------------------------------ */

static int cw_widget_pixels (int *out_w, int *out_h) {
    if (!cw_gl_area) { return 0; }
    const int scale = gtk_widget_get_scale_factor (cw_gl_area);
    const int w = gtk_widget_get_width (cw_gl_area) * scale;
    const int h = gtk_widget_get_height (cw_gl_area) * scale;
    if ((w <= 0) || (h <= 0)) { return 0; }
    *out_w = w;
    *out_h = h;
    return 1;
}

/* ---- picking ----------------------------------------------------------- */

/* Nearest part under the pixel. Returns the index or -1; the hit point
 * (when picked) goes to out_point. */
static int cw_pick (double px, double py, float *out_t, vector3 *out_point) {
    int w = 0;
    int h = 0;
    if (!cw_widget_pixels (&w, &h)) { return -1; }
    cw_camera.aspect = (float) w / (float) h;
    /* pixel -> ndc: x right, y UP (screen y grows downward). */
    const float ndc_x = (float) (2.0 * px / (double) w - 1.0);
    const float ndc_y = 1.0f - (float) (2.0 * py / (double) h);
    const cad_view_ray ray = cad_view_ray_from_screen (&cw_camera, ndc_x, ndc_y);
    int best = -1;
    float best_t = 0.0f;
    for (int i = 0; i < cw_design.count; i++) {
        const mig_part *p = &cw_design.parts [i];
        cad_view_hit hit;
        if (p -> type == mig_part_sphere) {
            hit = cad_view_pick_sphere (ray, p -> position, p -> radius);
        } else if (p -> type == mig_part_cube) {
            hit = cad_view_pick_box (ray, p -> position, p -> half_extensions,
                                     p -> orientation);
        } else {
            hit = cad_view_pick_cylinder (ray, p -> position, p -> radius,
                                          p -> cylinder_half_length,
                                          p -> orientation);
        }
        if (hit.hit && ((best < 0) || (hit.t < best_t))) {
            best = i;
            best_t = hit.t;
            if (out_point) { *out_point = hit.point; }
        }
    }
    if ((best >= 0) && out_t) { *out_t = best_t; }
    return best;
}

/* ---- status ------------------------------------------------------------ */

static const char *cw_err_name (int err) {
    switch (err) {
        case cad_ok: return "ok";
        case cad_null: return "null";
        case cad_full: return "full";
        case cad_bad_index: return "bad-index";
        case cad_bad_type: return "bad-type";
        case cad_bad_dims: return "bad-dims";
        case cad_bad_pose: return "bad-pose";
        case cad_bad_quat: return "bad-quat";
        case cad_bad_mass: return "bad-mass";
        case cad_bad_name: return "bad-name";
        case cad_bad_arg: return "bad-arg";
        case cad_io: return "io";
        case cad_empty: return "empty";
        case cad_spec_reject: return "spec-reject";
        default: return "?";
    }
}

static void cw_status_refresh (void) {
    if (!cw_status_label) { return; }
    const char *sel = "none";
    char selbuf [32];
    if ((cw_design.selected >= 0) && (cw_design.selected < cw_design.count)) {
        static const char *type_names [3] = {"sphere", "cube", "cylinder"};
        snprintf (selbuf, sizeof (selbuf), "%d %s", cw_design.selected,
                  type_names [cw_design.parts [cw_design.selected].type]);
        sel = selbuf;
    }
    char buf [256];
    snprintf (buf, sizeof (buf),
              "parts %d/%d | sel %s | snap %.3g m | rotsnap %.3g deg | magnet %s | look %s | %s",
              cw_design.count, cw_design.max_parts, sel,
              (double) cw_design.snap_step, (double) cw_design.rot_snap_deg,
              cw_magnet_on ? "on" : "off",
              cw_look_locked ? "locked" : "free", cw_design.name);
    gtk_label_set_text (GTK_LABEL (cw_status_label), buf);
}

static void cw_status_ok (const char *message) {
    if (cw_status_label) {
        gtk_label_set_text (GTK_LABEL (cw_status_label), message);
    }
    term_ok ("mpe: cad: ");
    term_out (message);
    term_out ("\n");
}

static void cw_status_err (const char *message) {
    if (cw_status_label) {
        gtk_label_set_text (GTK_LABEL (cw_status_label), message);
    }
    term_err ("mpe: cad: ");
    term_out (message);
    term_out ("\n");
}

/* ---- spec dropdown (mirrors the shared store, 4.4-panel pattern) ------- */

static void cw_refresh_dropdown (void) {
    if (!cw_spec_dropdown) { return; }
    GtkStringList *list = gtk_string_list_new (NULL);
    int count = mig_spec_store_count ();
    int keep_idx = -1;
    for (int i = 0; i < count; i++) {
        const char *name = mig_spec_store_name_at (i);
        gtk_string_list_append (list, name);
        if (strcmp (name, cw_design.name) == 0) {
            keep_idx = i;
        }
    }
    gtk_drop_down_set_model (GTK_DROP_DOWN (cw_spec_dropdown), G_LIST_MODEL (list));
    g_object_unref (list);
    if (keep_idx >= 0) {
        gtk_drop_down_set_selected (GTK_DROP_DOWN (cw_spec_dropdown),
                                    (guint) keep_idx);
    } else {
        gtk_drop_down_set_selected (GTK_DROP_DOWN (cw_spec_dropdown),
                                    GTK_INVALID_LIST_POSITION);
    }
}

/* ---- actions ----------------------------------------------------------- */

/* Save = the frozen grammar with a FULL temp-file round trip (the 4.4
 * panel pattern): emit canonical text -> write -> parse back -> stage
 * into the shared store. Write-on-success everywhere. */
static void cw_save_design (void) {
    mig_spec spec;
    int rc = cad_design_to_spec (&cw_design, &spec);
    if (rc != cad_ok) {
        char buf [128];
        snprintf (buf, sizeof (buf), "save refused (%s)", cw_err_name (rc));
        cw_status_err (buf);
        return;
    }
    char text [8192];
    size_t n = mig_spec_emit_normalized (&spec, text, sizeof (text));
    if ((n == 0) || (n >= sizeof (text))) {
        cw_status_err ("save refused (emit overflow)");
        return;
    }
    char path [512];
    snprintf (path, sizeof (path), "./.cad_window_%s.spec", cw_design.name);
    FILE *f = fopen (path, "w");
    if (!f) {
        cw_status_err ("save refused (cannot write temp spec)");
        return;
    }
    fwrite (text, 1, n, f);
    fclose (f);
    mig_spec parsed;
    int prc = mig_spec_parse_file (path, cw_design.max_parts, &parsed);
    remove (path);
    if (prc != mig_spec_ok) {
        char buf [128];
        snprintf (buf, sizeof (buf), "save refused (round-trip parse %d)", prc);
        cw_status_err (buf);
        return;
    }
    if (mig_spec_store_add (parsed.name, &parsed) < 0) {
        cw_status_err ("save refused (spec store full)");
        return;
    }
    cw_refresh_dropdown ();
    char buf [192];
    snprintf (buf, sizeof (buf), "saved %s (%d parts) -> spec store",
              parsed.name, parsed.count);
    cw_status_ok (buf);
}

/* Load = the dropdown's store spec -> the design (write-on-success:
 * a refused spec leaves the current design untouched). */
static void cw_load_selected (void) {
    if (!cw_spec_dropdown) { return; }
    const guint idx = gtk_drop_down_get_selected (GTK_DROP_DOWN (cw_spec_dropdown));
    if (idx == GTK_INVALID_LIST_POSITION) {
        cw_status_err ("load: no spec selected (store empty)");
        return;
    }
    const mig_spec *src = mig_spec_store_spec_at ((int) idx);
    if (!src) { return; }
    cad_design loaded;
    int rc = cad_design_from_spec (src, CAD_MAX_PARTS, &loaded);
    if (rc != cad_ok) {
        char buf [128];
        snprintf (buf, sizeof (buf), "load refused (%s)", cw_err_name (rc));
        cw_status_err (buf);
        return;
    }
    cw_design = loaded;
    cw_design_ready = 1;
    if (cw_name_entry) {
        gtk_editable_set_text (GTK_EDITABLE (cw_name_entry), cw_design.name);
    }
    cw_refresh_dropdown ();
    char buf [192];
    snprintf (buf, sizeof (buf), "loaded %s (%d parts) from spec store",
              cw_design.name, cw_design.count);
    cw_status_ok (buf);
}

static void cw_rename_from_entry (void) {
    if (!cw_name_entry) { return; }
    const char *text = gtk_editable_get_text (GTK_EDITABLE (cw_name_entry));
    int rc = cad_set_name (&cw_design, text);
    if (rc != cad_ok) {
        char buf [128];
        snprintf (buf, sizeof (buf), "rename refused (%s)", cw_err_name (rc));
        cw_status_err (buf);
        return;
    }
    cw_refresh_dropdown ();
    char buf [192];
    snprintf (buf, sizeof (buf), "renamed to %s", cw_design.name);
    cw_status_ok (buf);
}

/* ---- the world hookup (WP4 4.6e) --------------------------------------
 * The ONLY place the 4.6 family touches the physics world. Flow: the
 * gated build seam validates the design first (cad_build -> the compose
 * refusal battery + totals for the status line), then the engine's
 * public MIG API spawns it. The RAW assembly-frame parts are handed
 * over — physics_world_add_composite composes and re-anchors
 * internally and spawns the body at the parts' COM, and the CAD frame
 * IS the world frame (y-up, floor y = 0 in both), so the composite
 * lands exactly as drawn (the drag-weld caller passes world-frame
 * parts on the same principle). Handing the pre-rebased bridge copy
 * instead would subtract the COM twice and drop the body at the
 * origin, discarding the authored placement. */
static void cw_spawn_to_world (void) {
    if (cw_design.count == 0) {
        cw_status_err ("spawn: design is empty (1/2/3 add parts)");
        return;
    }
    mig_part bridge_parts [CAD_MAX_PARTS];
    mig_bridge bridge;
    const int rc = cad_build (&cw_design, bridge_parts, &bridge);
    if (rc != cad_ok) {
        char buf [128];
        snprintf (buf, sizeof (buf), "spawn refused (%s)", cw_err_name (rc));
        cw_status_err (buf);
        return;
    }
    physics_world *world = physics_world_get_primary ();
    const int body_index = physics_world_add_composite (world,
                                                        cw_design.parts,
                                                        cw_design.count);
    if (body_index < 0) {
        cw_status_err ("spawn refused (engine: mass/bound window or part cap)");
        return;
    }
    char buf [256];
    snprintf (buf, sizeof (buf),
              "spawned %s: %d parts, %.4g kg, COM (%.2f, %.2f, %.2f) -> body #%d",
              cw_design.name, cw_design.count, (double) bridge.totals.mass,
              (double) bridge.totals.com.x, (double) bridge.totals.com.y,
              (double) bridge.totals.com.z, body_index);
    cw_status_ok (buf);
}

/* Add at the spawn point: the camera target's XZ, rest height of the
 * identity orientation (sphere r, cube he.y, cylinder r — the axle is
 * LOCAL X, so the tube lies along the ground with radius up). The
 * standing-vs-lying spawn ruling stays deferred to the connect round;
 * identity orientation is the spec grammar's default pose. */
static void cw_add_part (int type) {
    const vector3 spawn = {cw_camera.target.x, 0.25f, cw_camera.target.z};
    int rc = cad_full;
    if (type == mig_part_sphere) {
        rc = cad_add_sphere (&cw_design, 0.25f, spawn);
    } else if (type == mig_part_cube) {
        rc = cad_add_cube (&cw_design, (vector3) {0.25f, 0.25f, 0.25f}, spawn);
    } else {
        rc = cad_add_cylinder (&cw_design, 0.25f, 0.5f, spawn);
    }
    if (rc != cad_ok) {
        char buf [128];
        snprintf (buf, sizeof (buf), "add refused (%s)", cw_err_name (rc));
        cw_status_err (buf);
        return;
    }
    cad_select (&cw_design, cw_design.count - 1);
    cw_status_refresh ();
}

static void cw_delete_selected (void) {
    int rc = cad_delete (&cw_design, cw_design.selected);
    if (rc != cad_ok) {
        char buf [128];
        snprintf (buf, sizeof (buf), "delete refused (%s)", cw_err_name (rc));
        cw_status_err (buf);
        return;
    }
    cw_status_refresh ();
}

static void cw_duplicate_selected (void) {
    /* First-cut offset: +0.5 m on X (snapped by the model when snap is
     * on); the copy becomes the new selection. */
    const vector3 offset = {0.5f, 0.0f, 0.0f};
    int rc = cad_duplicate (&cw_design, cw_design.selected, offset);
    if (rc != cad_ok) {
        char buf [128];
        snprintf (buf, sizeof (buf), "duplicate refused (%s)", cw_err_name (rc));
        cw_status_err (buf);
        return;
    }
    cad_select (&cw_design, cw_design.count - 1);
    cw_status_refresh ();
}

static void cw_array_selected (void) {
    /* First-cut pattern: 3 total (original + 2 copies), 0.5 m X step. */
    const vector3 step = {0.5f, 0.0f, 0.0f};
    int rc = cad_array (&cw_design, cw_design.selected, 3, step);
    if (rc != cad_ok) {
        char buf [128];
        snprintf (buf, sizeof (buf), "array refused (%s)", cw_err_name (rc));
        cw_status_err (buf);
        return;
    }
    cw_status_refresh ();
}

static void cw_rotate_selected (int axis, float degrees) {
    int rc = cad_rotate_degrees (&cw_design, cw_design.selected, axis, degrees);
    if (rc != cad_ok) {
        char buf [128];
        snprintf (buf, sizeof (buf), "rotate refused (%s)", cw_err_name (rc));
        cw_status_err (buf);
        return;
    }
    cw_status_refresh ();
}

static void cw_toggle_snap (void) {
    const float next = (cw_design.snap_step > 0.0f) ? 0.0f : 0.25f;
    int rc = cad_set_snap (&cw_design, next);
    if (rc != cad_ok) {
        char buf [128];
        snprintf (buf, sizeof (buf), "snap toggle refused (%s)", cw_err_name (rc));
        cw_status_err (buf);
        return;
    }
    cw_status_refresh ();
}

static void cw_toggle_rot_snap (void) {
    const float next = (cw_design.rot_snap_deg > 0.0f) ? 0.0f : 15.0f;
    int rc = cad_set_rot_snap (&cw_design, next);
    if (rc != cad_ok) {
        char buf [128];
        snprintf (buf, sizeof (buf), "rot-snap toggle refused (%s)", cw_err_name (rc));
        cw_status_err (buf);
        return;
    }
    cw_status_refresh ();
}

/* ---- placement helpers (WP4 4.6h) -------------------------------------- */

#define CW_MAGNET_ENGAGE 0.25f

/* Covering-sphere radius of ONE part — the same bound math
 * physics_world_add_composite enforces on whole assemblies: a sphere's
 * radius, a cube's vertex distance sqrt(hx^2+hy^2+hz^2), a cylinder's
 * sqrt(hl^2+r^2). Two parts whose centers sit exactly the sum of these
 * apart along the line of centers have TOUCHING bounds (conservative:
 * geometry may still sit a little inside for elongated shapes, but can
 * never overlap). */
static float cw_cover_radius (const mig_part *p) {
    if (p -> type == mig_part_sphere) { return p -> radius; }
    if (p -> type == mig_part_cube) {
        return sqrtf (p -> half_extensions.x * p -> half_extensions.x +
                      p -> half_extensions.y * p -> half_extensions.y +
                      p -> half_extensions.z * p -> half_extensions.z);
    }
    return sqrtf (p -> cylinder_half_length * p -> cylinder_half_length +
                  p -> radius * p -> radius);
}

/* The part's LOCAL half-dims: sphere (r, r, r), cube (hx, hy, hz),
 * cylinder (hl, r, r) — the axle is local X (the engine's FTC
 * convention). Shared by the oriented-extent helpers below. */
static void cw_local_dims (const mig_part *p, float *dx, float *dy,
                           float *dz) {
    if (p -> type == mig_part_sphere) {
        *dx = p -> radius; *dy = p -> radius; *dz = p -> radius;
        return;
    }
    if (p -> type == mig_part_cube) {
        *dx = p -> half_extensions.x;
        *dy = p -> half_extensions.y;
        *dz = p -> half_extensions.z;
        return;
    }
    *dx = p -> cylinder_half_length;
    *dy = p -> radius;
    *dz = p -> radius;
}

/* World-Y half-extents at the CURRENT orientation: the absolute values
 * of rotation-matrix rows 1 (down) and 2 (up) dotted with the part's
 * local half-dims. For the engine's (w,x,y,z) quaternion:
 * r10 = 2(xy+wz), r11 = 1-2(x^2+z^2), r12 = 2(yz-wx); r20 = 2(xz-wy),
 * r21 = 2(yz+wx), r22 = 1-2(x^2+y^2). A part placed at y =
 * down-extent rests exactly ON the floor for any orientation;
 * center + up-extent is its top surface. */
static float cw_down_extent (const mig_part *p) {
    const float x = (float) p -> orientation.x;
    const float y = (float) p -> orientation.y;
    const float z = (float) p -> orientation.z;
    const float w = (float) p -> orientation.w;
    const float r10 = 2.0f * (x * y + w * z);
    const float r11 = 1.0f - 2.0f * (x * x + z * z);
    const float r12 = 2.0f * (y * z - w * x);
    float dx = 0.0f;
    float dy = 0.0f;
    float dz = 0.0f;
    cw_local_dims (p, &dx, &dy, &dz);
    return fabsf (r10) * dx + fabsf (r11) * dy + fabsf (r12) * dz;
}

static float cw_up_extent (const mig_part *p) {
    const float x = (float) p -> orientation.x;
    const float y = (float) p -> orientation.y;
    const float z = (float) p -> orientation.z;
    const float w = (float) p -> orientation.w;
    const float r20 = 2.0f * (x * z - w * y);
    const float r21 = 2.0f * (y * z + w * x);
    const float r22 = 1.0f - 2.0f * (x * x + y * y);
    float dx = 0.0f;
    float dy = 0.0f;
    float dz = 0.0f;
    cw_local_dims (p, &dx, &dy, &dz);
    return fabsf (r20) * dx + fabsf (r21) * dy + fabsf (r22) * dz;
}

/* The oriented XZ footprint half-extents along world X and world Z
 * (|row 0| and |row 2| dotted with the local dims) — the
 * support-overlap test for the drop rest (per-axis extents, tighter
 * than a circumscribing circle: a sphere's footprint stays its disc
 * radius whatever the orientation). */
static void cw_xz_extents (const mig_part *p, float *ex, float *ez) {
    const float x = (float) p -> orientation.x;
    const float y = (float) p -> orientation.y;
    const float z = (float) p -> orientation.z;
    const float w = (float) p -> orientation.w;
    const float r00 = 1.0f - 2.0f * (y * y + z * z);
    const float r01 = 2.0f * (x * y - w * z);
    const float r02 = 2.0f * (x * z + w * y);
    const float r20 = 2.0f * (x * z - w * y);
    const float r21 = 2.0f * (y * z + w * x);
    const float r22 = 1.0f - 2.0f * (x * x + y * y);
    float dx = 0.0f;
    float dy = 0.0f;
    float dz = 0.0f;
    cw_local_dims (p, &dx, &dy, &dz);
    *ex = fabsf (r00) * dx + fabsf (r01) * dy + fabsf (r02) * dz;
    *ez = fabsf (r20) * dx + fabsf (r21) * dy + fabsf (r22) * dz;
}

/* Magnet assist while carrying: snap the target to TOUCH the nearest
 * other part along the line of centers — centers exactly the sum of
 * the covering radii apart. Engages only within CW_MAGNET_ENGAGE of a
 * valid touch position, so free placement away from parts stays free;
 * ties break to the nearest part; coincident centers (no line of
 * centers) skip. The result is written through cad_set_position like
 * any move, so grid snap still rounds it — a snapped touch can sit up
 * to half a step off the exact contact — and the floor clamp (applied
 * after) can lift a would-be under-floor touch: floor wins, snap
 * honors the grid. */
static void cw_magnet_adjust (const mig_part *carried, vector3 *target) {
    const float r_carry = cw_cover_radius (carried);
    float best_err = 0.0f;
    vector3 best = {0.0f, 0.0f, 0.0f};
    int have = 0;
    for (int i = 0; i < cw_design.count; i++) {
        if (i == cw_drag_part) { continue; }
        const mig_part *n = &cw_design.parts [i];
        const vector3 delta = vector3_subtraction (*target, n -> position);
        const float dist = vector3_length (delta);
        if (dist <= 1e-6f) { continue; }
        const float want = r_carry + cw_cover_radius (n);
        const float err = fabsf (dist - want);
        if (err > CW_MAGNET_ENGAGE) { continue; }
        if ((!have) || (err < best_err)) {
            have = 1;
            best_err = err;
            best = vector3_addition (n -> position,
                                     vector3_scaling (delta, want / dist));
        }
    }
    if (have) { *target = best; }
}

/* The window's weld-on-drop test (the drag_weld S4c rule ported):
 * TRUE when the part's covering bound touches or overlaps ANOTHER
 * part's — centers within the sum of the covering radii, plus
 * CW_WELD_EPS touch tolerance. The engine welds a dropped body whose
 * covering sphere overlaps another AT THAT POSE; here the design is
 * already one composite, so "joined" means the release keeps the pose
 * (no settle). The magnet places at the EXACT touch distance, so
 * touch counts (the engine needs a real push-in; the magnet IS the
 * push-in). */
#define CW_WELD_EPS 1e-3f
static int cw_joined (int idx) {
    const mig_part *p = &cw_design.parts [idx];
    const float r_p = cw_cover_radius (p);
    for (int i = 0; i < cw_design.count; i++) {
        if (i == idx) { continue; }
        const vector3 delta = vector3_subtraction (
            p -> position, cw_design.parts [i].position);
        if (vector3_length (delta) <=
            (r_p + cw_cover_radius (&cw_design.parts [i]) + CW_WELD_EPS)) {
            return 1;
        }
    }
    return 0;
}

/* The drop rest height (center y) for a part: the highest supporter
 * below it — another part whose oriented top is at/below the part's
 * bottom AND whose oriented XZ footprint overlaps the part's (per-axis
 * extents; tangent counts, conservative and stable) — plus the part's
 * oriented down-extent; the floor (y = 0) otherwise. Computed once at
 * release; supporters don't move during the fall. */
static float cw_rest_y_for (int idx) {
    const mig_part *p = &cw_design.parts [idx];
    const float bottom = p -> position.y - cw_down_extent (p);
    float ex_p = 0.0f;
    float ez_p = 0.0f;
    cw_xz_extents (p, &ex_p, &ez_p);
    float best_top = 0.0f; /* the floor */
    for (int i = 0; i < cw_design.count; i++) {
        if (i == idx) { continue; }
        const mig_part *n = &cw_design.parts [i];
        const float top = n -> position.y + cw_up_extent (n);
        if (top > bottom + CW_WELD_EPS) { continue; } /* not below me */
        float ex_n = 0.0f;
        float ez_n = 0.0f;
        cw_xz_extents (n, &ex_n, &ez_n);
        const float adx = fabsf (p -> position.x - n -> position.x);
        const float adz = fabsf (p -> position.z - n -> position.z);
        if ((adx > ex_p + ex_n) || (adz > ez_p + ez_n)) { continue; }
        if (top > best_top) { best_top = top; }
    }
    return best_top + cw_down_extent (p);
}

/* Write a position with the grid snap bypassed for THIS write (the
 * model reads snap_step per call — the G-off semantics): landings and
 * magnet touches are surface truths and must be EXACT, while the snap
 * stays honored for every other write. A drop that hovers one snap
 * step over the floor reads broken; a drag that quantizes reads as
 * the grid. */
static void cw_write_exact (int idx, vector3 position) {
    const float saved = cw_design.snap_step;
    cw_design.snap_step = 0.0f;
    cad_set_position (&cw_design, idx, position);
    cw_design.snap_step = saved;
}

/* The settle's gravity magnitude: the world's own gravity (config
 * world.gravity, negative = down — the window matches the world it
 * spawns into), floored at a small positive value so a zeroed config
 * still settles instead of falling forever. */
static float cw_gravity (void) {
    float g = -g_cfg.world.gravity;
    if ((!isfinite (g)) || (g < 0.5f)) { g = 9.81f; }
    return g;
}

/* Carry floor clamp: the raw y, but never below the grabbed part's
 * oriented down-extent. With grid snap on, the model rounds the
 * written position — pass at least the first snap MULTIPLE at or
 * above the floor rest (a multiple is snap-invariant) so the rounding
 * can never dip the part under the floor. */
static float cw_carry_clamp_y (float raw_y) {
    float y = (raw_y < cw_drag_floor_y) ? cw_drag_floor_y : raw_y;
    const float step = cw_design.snap_step;
    if (step > 0.0f) {
        const float y_min = ceilf (cw_drag_floor_y / step) * step;
        if (y < y_min) { y = y_min; }
    }
    return y;
}

/* The carry target from a SCREEN-CENTER ray in NDC (0, 0) while the
 * pointer is locked: the part rides the crosshair ray at the grabbed
 * depth (t along the UNIT direction) plus the frozen grip offset —
 * the drag_weld_tick math ported to the crosshair. Magnet-adjusted
 * targets are written EXACT (the touch must survive the model's snap
 * rounding for the weld-on-drop rule to hold at release); plain
 * carries honor the grid. */
static void cw_carry_apply_ndc (float ndc_x, float ndc_y) {
    int w = 0;
    int h = 0;
    if (!cw_widget_pixels (&w, &h)) { return; }
    cw_camera.aspect = (float) w / (float) h;
    const cad_view_ray ray = cad_view_ray_from_screen (&cw_camera, ndc_x, ndc_y);
    vector3 target = vector3_addition (
        vector3_scaling (ray.direction, cw_drag_t), cw_drag_offset);
    target = vector3_addition (ray.origin, target);
    const int magnet = cw_magnet_on;
    if (magnet) {
        cw_magnet_adjust (&cw_design.parts [cw_drag_part], &target);
    }
    target.y = cw_carry_clamp_y (target.y);
    if (magnet) {
        cw_write_exact (cw_drag_part, target);
    } else {
        cad_set_position (&cw_design, cw_drag_part, target);
    }
}

/* The unlocked (cursor-driven) flavor: pixels -> NDC, then the same
 * apply. Called from the motion handler AND the tick — the
 * every-frame follow (zoom mid-carry keeps the part glued). */
static void cw_carry_update (double x, double y) {
    int w = 0;
    int h = 0;
    if (!cw_widget_pixels (&w, &h)) { return; }
    const float ndc_x = (float) (2.0 * x / (double) w - 1.0);
    const float ndc_y = 1.0f - (float) (2.0 * y / (double) h);
    cw_carry_apply_ndc (ndc_x, ndc_y);
}

/* Land a settling part: write the rest height EXACTLY, report, clear
 * the slot (and the mode if the settle owned it). */
static void cw_settle_finish (void) {
    if (cw_settle_part < 0) { return; }
    const mig_part *p = &cw_design.parts [cw_settle_part];
    vector3 pos = p -> position;
    pos.y = cw_settle_rest_y;
    cw_write_exact (cw_settle_part, pos);
    char buf [128];
    snprintf (buf, sizeof (buf), "settled at y=%.3g",
              (double) cw_settle_rest_y);
    cw_status_ok (buf);
    cw_settle_part = -1;
    if (cw_mode == CW_MODE_SETTLING) { cw_mode = CW_MODE_IDLE; }
}

/* Release of a carried part — the S4c drop rule ported to the design:
 * covering bounds touching/overlapping another part = the pose JOINS
 * the composite (the weld rule; stays exactly where placed); free
 * space = the part FALLS. The engine zeroes the carried body's
 * velocity every frame while held, so its release is a pure vertical
 * fall — the settle is the same: straight down from the release
 * height to the support rest (cw_rest_y_for), integrated in the tick
 * at the world's own gravity. */
static void cw_release_carry (void) {
    const int idx = cw_drag_part;
    cw_drag_part = -1;
    cw_mode = CW_MODE_IDLE;
    if (idx < 0) { return; }
    if (cw_joined (idx)) {
        cw_status_ok ("dropped: joined at the touch (weld rule)");
        return;
    }
    cw_settle_part = idx;
    cw_settle_y = cw_design.parts [idx].position.y;
    cw_settle_v = 0.0f;
    cw_settle_rest_y = cw_rest_y_for (idx);
    cw_mode = CW_MODE_SETTLING;
    if (cw_settle_y <= cw_settle_rest_y) {
        cw_settle_finish (); /* released at/below rest: land instantly */
        return;
    }
    cw_status_ok ("dropped: falling to rest");
}

/* The settle frame: gravity in, integrate down, land at the rest.
 * The fall writes EXACT positions (gravity is a surface truth; the
 * grid governs placement, not physics). */
static void cw_settle_tick (float dt) {
    if (cw_settle_part < 0) {
        cw_mode = CW_MODE_IDLE;
        return;
    }
    cw_settle_v -= cw_gravity () * dt;
    cw_settle_y += cw_settle_v * dt;
    if (cw_settle_y <= cw_settle_rest_y) {
        cw_settle_finish ();
        return;
    }
    const mig_part *p = &cw_design.parts [cw_settle_part];
    vector3 pos = p -> position;
    pos.y = cw_settle_y;
    cw_write_exact (cw_settle_part, pos);
}

/* M: the magnet placement toggle (the status line shows the state). */
static void cw_toggle_magnet (void) {
    cw_magnet_on = !cw_magnet_on;
    cw_status_refresh ();
}

/* F: drop the selection to REST — the settle's support seek (the
 * highest supporter below it, or the floor), landed EXACTLY (the
 * snap-bypass write; gravity is a surface truth). WP4 4.6i upgraded
 * the 4.6h plain floor: a part above a stack now drops ONTO the
 * stack, like the engine's gravity. XZ stays where it was. */
static void cw_floor_selected (void) {
    if ((cw_design.selected < 0) || (cw_design.selected >= cw_design.count)) {
        cw_status_err ("floor: nothing selected");
        return;
    }
    const int idx = cw_design.selected;
    vector3 pos = cw_design.parts [idx].position;
    pos.y = cw_rest_y_for (idx);
    cw_write_exact (idx, pos);
    char buf [128];
    snprintf (buf, sizeof (buf), "dropped to rest at y=%.3g", (double) pos.y);
    cw_status_ok (buf);
}

/* ---- the engine's control scheme (WP4 4.6j) ----------------------------
 * Pointer lock, mouse-look steering and fly movement, cloned from the
 * engine's own input path (input_control.c + simulation_camera.c +
 * mouse_lock.c) with ZERO engine-core changes: mouse_lock_enable/
 * disable touch no engine state, and this window never sets
 * main_inputs.is_mouse_locked, so the engine's delta poll (gated on
 * that flag) never sees CAD deltas — the 4.6f isolation holds. */

/* Steering applied in the ENGINE'S delta convention (dx right+,
 * dy up+). The CAD camera is the orbit camera: cad_view_forward's
 * forward_x = -sin(yaw)*cos(pitch), so yaw_deg+ turns LEFT (the eye
 * swings +x while looking at the target) — mouse-right must turn
 * right, hence the yaw negation; and pitch_deg is the EYE's elevation
 * above the target, so looking UP lowers it — the pitch negation.
 * Both flips live HERE and nowhere else. Sensitivity is the engine's
 * own config (camera.steer_sensitivity, default 0.12). The pitch
 * clamp is the view-math contract's caller-side clamp (±85, the same
 * bound the 4.6c orbit already used). */
static void cw_apply_steer (float dx_right, float dy_up) {
    const float s = g_cfg.camera.steer_sensitivity;
    cw_camera.yaw_deg -= dx_right * s;
    cw_camera.pitch_deg -= dy_up * s;
    if (cw_camera.pitch_deg > 85.0f) { cw_camera.pitch_deg = 85.0f; }
    if (cw_camera.pitch_deg < -85.0f) { cw_camera.pitch_deg = -85.0f; }
}

/* Wayland path: drain the relative-pointer deltas ONCE per tick (the
 * input_control_poll_locked_mouse pattern — the compositor stops
 * wl_pointer.motion while the lock is live, so per-event draining
 * starves). mouse_lock_relative_to_camera is the single source of the
 * sign convention (+x right, +y negated to up). X11 has no relative
 * protocol: steering happens per motion event in cw_on_motion. */
static void cw_look_steer (void) {
    if (!mouse_lock_relative_active ()) { return; }
    double rdx = 0.0;
    double rdy = 0.0;
    if (!mouse_lock_take_relative_delta (&rdx, &rdy)) { return; }
    float sx = 0.0f;
    float sy = 0.0f;
    (void) mouse_lock_relative_to_camera (rdx, rdy, &sx, &sy);
    cw_apply_steer (sx, sy);
}

/* Fly translation: the engine's debug-fly block verbatim
 * (simulation_camera.c) — W/S along the forward, A/D along the side,
 * Space up / Shift down along world Y — at move_speed x0.2. The
 * scale is the one deliberate deviation, documented: the engine's
 * 25 m/s default is tuned for its 500 m arena and crosses this ~10 m
 * scene in half a second; 5 m/s flies it like the engine flies its
 * world. Moving the TARGET moves the eye with it (the eye is
 * target - forward*distance), so this IS first-person flight. */
static void cw_fly_translate (float dt) {
    const float speed = g_cfg.camera.move_speed * 0.2f * dt;
    const vector3 fwd = cad_view_forward (&cw_camera);
    const vector3 right = cad_view_right (&cw_camera);
    vector3 t = cw_camera.target;
    if (cw_key_w) { t = vector3_addition (t, vector3_scaling (fwd, speed)); }
    if (cw_key_s) { t = vector3_subtraction (t, vector3_scaling (fwd, speed)); }
    if (cw_key_a) { t = vector3_subtraction (t, vector3_scaling (right, speed)); }
    if (cw_key_d) { t = vector3_addition (t, vector3_scaling (right, speed)); }
    if (cw_key_up) { t.y += speed; }
    if (cw_key_down) { t.y -= speed; }
    cw_camera.target = t;
}

static void cw_kill_fly_latches (void) {
    cw_key_w = 0;
    cw_key_a = 0;
    cw_key_s = 0;
    cw_key_d = 0;
    cw_key_up = 0;
    cw_key_down = 0;
}

/* Acquire the pointer lock (the engine's re-engage). Dead latches are
 * cleared on BOTH edges so a key held across a lock edge never flies
 * the camera uninvited. */
static void cw_look_lock (void) {
    if (cw_look_locked || !cw_gl_area) { return; }
    mouse_lock_enable (cw_gl_area);
    cw_look_locked = 1;
    cw_kill_fly_latches ();
    cw_status_refresh ();
}

static void cw_look_unlock (void) {
    if (!cw_look_locked) { return; }
    cw_look_locked = 0;
    if (cw_gl_area && GTK_IS_WIDGET (cw_gl_area)) {
        mouse_lock_disable (cw_gl_area);
    }
    cw_kill_fly_latches ();
    cw_status_refresh ();
}

/* Crosshair grab — the drag_weld_press port with the ray pinned to
 * the screen center: the pick IS the selector (miss clears the
 * selection — the selector's own contract), then grab at the hit
 * depth and carry on the crosshair ray. Only valid while locked (the
 * crosshair exists then). */
static void cw_crosshair_grab (void) {
    if (cw_mode == CW_MODE_CARRY) { return; } /* second press while held: ignore (drag_weld's guard) */
    int w = 0;
    int h = 0;
    if (!cw_widget_pixels (&w, &h)) { return; }
    float t = 0.0f;
    vector3 point = {0.0f, 0.0f, 0.0f};
    const int picked = cw_pick ((double) w * 0.5, (double) h * 0.5, &t, &point);
    if (picked < 0) {
        cad_select (&cw_design, -1);
        cw_status_refresh ();
        return;
    }
    cad_select (&cw_design, picked);
    cw_drag_part = picked;
    cw_drag_t = t;
    cw_drag_floor_y = cw_down_extent (&cw_design.parts [picked]);
    cw_drag_offset = vector3_subtraction (cw_design.parts [picked].position, point);
    cw_mode = CW_MODE_CARRY;
    cw_status_refresh ();
}

/* Crosshair select (RMB while locked): the selector at screen center,
 * miss deselects — select_object_by_index's contract. */
static void cw_crosshair_select (void) {
    int w = 0;
    int h = 0;
    if (!cw_widget_pixels (&w, &h)) { return; }
    cad_select (&cw_design, cw_pick ((double) w * 0.5, (double) h * 0.5,
                                     NULL, NULL));
    cw_status_refresh ();
}

/* ---- input handlers ---------------------------------------------------- */

static void cw_on_press (GtkGestureClick *gesture, gint n_press, gdouble x,
                         gdouble y, gpointer user_data) {
    (void) n_press;
    (void) user_data;
    /* A click into the viewport leaves the typing block: focus moves
     * to the viewport and the CAD keybinds come back (GTK's own
     * click-to-focus does the same via focusable; this is the
     * deterministic guarantee). */
    gtk_widget_grab_focus (cw_gl_area);
    const guint button = gtk_gesture_single_get_current_button (
        GTK_GESTURE_SINGLE (gesture));
    GdkModifierType state = 0;
    GdkEvent *event = gtk_event_controller_get_current_event (
        GTK_EVENT_CONTROLLER (gesture));
    if (event) { state = gdk_event_get_modifier_state (event); }
    cw_last_x = x;
    cw_last_y = y;
    cw_cursor_x = x;
    cw_cursor_y = y;
    /* Any press cancels an active settle: a falling part is snapped to
     * its rest instantly (a part frozen mid-fall forever would be a
     * leak). */
    if (cw_settle_part >= 0) { cw_settle_finish (); }
    /* RMB (button 3): the engine's selector — pick and select only,
     * never a drag; empty space deselects (-1 is the model's
     * documented deselect, matching select_object_by_index's miss
     * contract). Under the crosshair while locked, under the cursor
     * while unlocked. */
    if (button == 3) {
        if (cw_look_locked) {
            cw_crosshair_select ();
        } else {
            cad_select (&cw_design, cw_pick (x, y, NULL, NULL));
            cw_status_refresh ();
        }
        return;
    }
    /* MMB (button 2): DELETE the selected part — the engine's own
     * middle-click rule (scene_remove_object_by_index on the
     * selection), in both pointer modes. */
    if (button == 2) {
        cw_delete_selected ();
        return;
    }
    if (button != 1) { return; }
    if (cw_look_locked) {
        /* LOCKED LMB: the engine's grab under the crosshair
         * (drag_weld_press: pick selects, miss clears, grab at the
         * hit depth; release runs the drop rule). */
        cw_crosshair_grab ();
        return;
    }
    const int shift = (state & GDK_SHIFT_MASK) ? 1 : 0;
    if (!shift) {
        /* UNLOCKED plain LMB: the engine's re-engage — the click
         * ACQUIRES the pointer lock and nothing else (the S4b rule:
         * the click that acquires the lock must not also grab). The
         * next LMB, now crosshair-aimed, grabs. */
        cw_look_lock ();
        return;
    }
    /* Shift+LMB while unlocked: the CAD precision fallback (cursor-
     * driven, no lock) — pan on empty space, the y-locked work-plane
     * carry on a part (the pre-4.6h drag; the grab offset freezes the
     * grip so the part doesn't jump; NO settle on release — the
     * precision tool places poses gravity would never hold). */
    vector3 point = {0.0f, 0.0f, 0.0f};
    const int picked = cw_pick (x, y, NULL, &point);
    if (picked < 0) {
        cw_mode = CW_MODE_PAN;
        return;
    }
    cad_select (&cw_design, picked);
    cw_drag_part = picked;
    cw_drag_plane_y = cw_design.parts [picked].position.y;
    cw_drag_offset = vector3_subtraction (cw_design.parts [picked].position, point);
    cw_mode = CW_MODE_PLANE_DRAG;
    cw_status_refresh ();
}

static void cw_on_release (GtkGestureClick *gesture, gint n_press, gdouble x,
                           gdouble y, gpointer user_data) {
    (void) gesture;
    (void) n_press;
    (void) x;
    (void) y;
    (void) user_data;
    /* WP4 4.6i: a released CARRY goes through the drop rule (the S4c
     * port — joined at the touch stays, free space falls to rest). A
     * released PLANE_DRAG is the precision tool: the pose stays exactly
     * where slid, no settle. */
    if (cw_mode == CW_MODE_CARRY) {
        cw_release_carry ();
        return;
    }
    cw_mode = CW_MODE_IDLE;
    cw_drag_part = -1;
}

static void cw_on_motion (GtkEventControllerMotion *controller, gdouble x,
                          gdouble y, gpointer user_data) {
    (void) controller;
    (void) user_data;
    const float dx = (float) (x - cw_last_x);
    const float dy = (float) (y - cw_last_y);
    cw_last_x = x;
    cw_last_y = y;
    cw_cursor_x = x;
    cw_cursor_y = y;
    if (cw_look_locked) {
        /* The pointer is the camera now. With a live relative-pointer
         * lock the absolute coordinates are meaningless (the engine's
         * contract: ignore them entirely) — the tick drains the
         * relative deltas. Without one (X11, or a compositor without
         * the protocol): steer from the absolute deltas and re-centre
         * the warped cursor, the engine's on_mouse_movements pattern
         * (deltas clamped to half the surface like there). */
        if (!mouse_lock_relative_active ()) {
            int w = 0;
            int h = 0;
            float cdx = dx;
            float cdy = dy;
            if (cw_widget_pixels (&w, &h)) {
                if (cdx > (float) w / 2.0f) { cdx = (float) w / 2.0f; }
                if (cdx < (float) -w / 2.0f) { cdx = (float) -w / 2.0f; }
                if (cdy > (float) h / 2.0f) { cdy = (float) h / 2.0f; }
                if (cdy < (float) -h / 2.0f) { cdy = (float) -h / 2.0f; }
            }
            cw_apply_steer (cdx, -cdy);
            mouse_lock_reset_centre (cw_gl_area);
        }
        return;
    }
    if (cw_mode == CW_MODE_PAN) {
        int w = 0;
        int h = 0;
        if (!cw_widget_pixels (&w, &h)) { return; }
        cw_camera.aspect = (float) w / (float) h;
        /* Grab-world pan: the scene follows the cursor, so the target
         * moves opposite the drag, scaled by the orbit distance. */
        const float k = cw_camera.distance * 0.0015f;
        const vector3 right = cad_view_right (&cw_camera);
        const vector3 up = cad_view_up (&cw_camera);
        cw_camera.target = vector3_subtraction (cw_camera.target,
            vector3_scaling (right, dx * k));
        cw_camera.target = vector3_addition (cw_camera.target,
            vector3_scaling (up, dy * k));
        return;
    }
    if (cw_mode == CW_MODE_CARRY) {
        /* The engine carry, recomputed for this motion AND again in
         * the tick every frame (the drag_weld_tick analog): no jump at
         * grab, no plane lock, full 3D. */
        cw_carry_update (x, y);
        return;
    }
    if (cw_mode == CW_MODE_PLANE_DRAG) {
        int w = 0;
        int h = 0;
        if (!cw_widget_pixels (&w, &h)) { return; }
        cw_camera.aspect = (float) w / (float) h;
        const float ndc_x = (float) (2.0 * x / (double) w - 1.0);
        const float ndc_y = 1.0f - (float) (2.0 * y / (double) h);
        const cad_view_ray ray = cad_view_ray_from_screen (&cw_camera, ndc_x, ndc_y);
        const vector3 plane_point = {0.0f, cw_drag_plane_y, 0.0f};
        const vector3 plane_normal = {0.0f, 1.0f, 0.0f};
        vector3 hit = {0.0f, 0.0f, 0.0f};
        if (cad_view_ray_plane (ray, plane_point, plane_normal, &hit)) {
            cad_set_position (&cw_design, cw_drag_part,
                              vector3_addition (hit, cw_drag_offset));
            cw_status_refresh ();
        }
        /* Plane behind the ray (camera under the part plane): the
         * view-math contract says keep the part in place. */
        return;
    }
}

static void cw_on_scroll (GtkEventControllerScroll *controller, gdouble dx,
                          gdouble dy, gpointer user_data) {
    (void) controller;
    (void) dx;
    (void) user_data;
    if (dy == 0.0) { return; } /* horizontal scroll is not a zoom */
    /* Scroll down = zoom out. */
    const float factor = (dy > 0.0) ? 1.1f : (1.0f / 1.1f);
    cw_camera.distance *= factor;
    if (cw_camera.distance < 1.5f) { cw_camera.distance = 1.5f; }
    if (cw_camera.distance > 80.0f) { cw_camera.distance = 80.0f; }
}

/* Typing-block guard: TRUE while keyboard focus is inside the name
 * entry — which in GTK4 means the entry's INTERNAL GtkText leaf, not
 * the GtkEntry itself (gtk_window_get_focus returns the deep focus
 * leaf, so an equality check against the entry never fires — the 4.6g
 * keybinds-while-typed fix). */
static gboolean cw_focus_in_name_entry (void) {
    if (!cw_window || !cw_name_entry) { return FALSE; }
    GtkWidget *focus = gtk_window_get_focus (GTK_WINDOW (cw_window));
    return focus && ((focus == cw_name_entry) ||
                     gtk_widget_is_ancestor (focus, cw_name_entry));
}

/* CAD keys skip the name entry (typing "4sphere2" must not spawn
 * spheres): while the typing block holds focus, only Esc is handled
 * here and everything else propagates to the entry. Outside it —
 * viewport, buttons, dropdown, nothing focused — keybinds work as
 * normal. */
static gboolean cw_on_key (GtkEventControllerKey *controller, guint keyval,
                           guint keycode, GdkModifierType state,
                           gpointer user_data) {
    (void) controller;
    (void) keycode;
    (void) user_data;
    if (keyval == GDK_KEY_Escape) {
        /* Two-stage, the engine's own convention: locked Esc FREES the
         * pointer (the engine's Esc rule); unlocked Esc closes the
         * window (the 4.6c ruling — and while typing only Esc still
         * works, the 4.6g ruling). */
        if (cw_look_locked) {
            cw_look_unlock ();
            return TRUE;
        }
        if (cw_window) { gtk_window_destroy (GTK_WINDOW (cw_window)); }
        return TRUE;
    }
    /* The FLY latches come first: while the pointer is locked the
     * engine's own control scheme wins (WASD + Space/Shift fly; the
     * colliding CAD verbs D/A/S/L wait for unlock), and the latching
     * sits BEFORE the mid-motion guard so flying coexists with a
     * crosshair carry (the engine flies while grabbing). Non-fly keys
     * fall through to the verbs below, which work in both modes. */
    if (cw_look_locked && !cw_focus_in_name_entry ()) {
        switch (keyval) {
            case GDK_KEY_w: case GDK_KEY_W: cw_key_w = 1; return TRUE;
            case GDK_KEY_a: case GDK_KEY_A: cw_key_a = 1; return TRUE;
            case GDK_KEY_s: case GDK_KEY_S: cw_key_s = 1; return TRUE;
            case GDK_KEY_d: case GDK_KEY_D: cw_key_d = 1; return TRUE;
            case GDK_KEY_space: cw_key_up = 1; return TRUE;
            case GDK_KEY_Shift_L:
            case GDK_KEY_Shift_R: cw_key_down = 1; return TRUE;
            default: break;
        }
    }
    if (cw_focus_in_name_entry ()) {
        return FALSE;
    }
    /* Mid-motion the selection indices can shift under a Delete (the
     * model shifts part indices down), which would retarget the carry
     * or the SETTLE at the wrong part — CAD verbs are frozen until the
     * motion ends, in BOTH carry modes and during the gravity drop.
     * Esc still closes. */
    if ((cw_mode == CW_MODE_CARRY) || (cw_mode == CW_MODE_PLANE_DRAG) ||
        (cw_mode == CW_MODE_SETTLING)) {
        return FALSE;
    }
    const int shift = (state & GDK_SHIFT_MASK) ? 1 : 0;
    switch (keyval) {
        case GDK_KEY_1: cw_add_part (mig_part_sphere); return TRUE;
        case GDK_KEY_2: cw_add_part (mig_part_cube); return TRUE;
        case GDK_KEY_3: cw_add_part (mig_part_cylinder); return TRUE;
        case GDK_KEY_Delete: cw_delete_selected (); return TRUE;
        case GDK_KEY_d:
        case GDK_KEY_D: cw_duplicate_selected (); return TRUE;
        case GDK_KEY_a:
        case GDK_KEY_A: cw_array_selected (); return TRUE;
        case GDK_KEY_x:
        case GDK_KEY_X: cw_rotate_selected (0, shift ? -15.0f : 15.0f); return TRUE;
        case GDK_KEY_y:
        case GDK_KEY_Y: cw_rotate_selected (1, shift ? -15.0f : 15.0f); return TRUE;
        case GDK_KEY_z:
        case GDK_KEY_Z: cw_rotate_selected (2, shift ? -15.0f : 15.0f); return TRUE;
        case GDK_KEY_g:
        case GDK_KEY_G: cw_toggle_snap (); return TRUE;
        case GDK_KEY_r:
        case GDK_KEY_R: cw_toggle_rot_snap (); return TRUE;
        case GDK_KEY_m:
        case GDK_KEY_M: cw_toggle_magnet (); return TRUE;
        case GDK_KEY_f:
        case GDK_KEY_F: cw_floor_selected (); return TRUE;
        case GDK_KEY_s:
        case GDK_KEY_S: cw_save_design (); return TRUE;
        case GDK_KEY_l:
        case GDK_KEY_L: cw_load_selected (); return TRUE;
        case GDK_KEY_p:
        case GDK_KEY_P: cw_spawn_to_world (); return TRUE;
        default: return FALSE;
    }
}

/* The engine's on_key_released pattern (input_control.c): a latch set
 * on press dies on release — polled latches make key repeat irrelevant
 * and a key held across a lock/unlock edge dies with the latches
 * (cw_kill_fly_latches). Returns FALSE so releases keep propagating. */
static gboolean cw_on_key_release (GtkEventControllerKey *controller,
                                   guint keyval, guint keycode,
                                   GdkModifierType state, gpointer user_data) {
    (void) controller;
    (void) keycode;
    (void) state;
    (void) user_data;
    switch (keyval) {
        case GDK_KEY_w: case GDK_KEY_W: cw_key_w = 0; break;
        case GDK_KEY_a: case GDK_KEY_A: cw_key_a = 0; break;
        case GDK_KEY_s: case GDK_KEY_S: cw_key_s = 0; break;
        case GDK_KEY_d: case GDK_KEY_D: cw_key_d = 0; break;
        case GDK_KEY_space: cw_key_up = 0; break;
        case GDK_KEY_Shift_L:
        case GDK_KEY_Shift_R: cw_key_down = 0; break;
        default: break;
    }
    return FALSE;
}

/* ---- GL callbacks ------------------------------------------------------ */

static void cw_on_realize (GtkGLArea *area, gpointer user_data) {
    (void) user_data;
    if (gtk_gl_area_get_error (area) != NULL) {
        printf ("[cad] gl area error: %s\n", gtk_gl_area_get_error (area) -> message);
        return;
    }
    gtk_gl_area_make_current (area);
    glEnable (GL_DEPTH_TEST);
    cw_program = cw_build_program ();
    if (cw_program == 0) { return; }
    cw_u_viewproj = glGetUniformLocation (cw_program, "u_viewproj");
    cw_u_model = glGetUniformLocation (cw_program, "u_model");
    cw_u_inv_scale = glGetUniformLocation (cw_program, "u_inv_scale");
    cw_u_color = glGetUniformLocation (cw_program, "u_color");
    cw_u_selected = glGetUniformLocation (cw_program, "u_selected");
    cw_u_unlit = glGetUniformLocation (cw_program, "u_unlit");
    cw_count_sphere = cw_fill_sphere (cw_mesh_sphere);
    cw_count_cube = cw_fill_cube (cw_mesh_cube);
    cw_count_cylinder = cw_fill_cylinder (cw_mesh_cylinder);
    cw_count_grid = cw_fill_grid (cw_mesh_grid);
    cw_count_crosshair = cw_fill_crosshair (cw_mesh_crosshair);
    cw_make_vao (cw_mesh_sphere, CW_SPHERE_FLOATS, &cw_vao_sphere,
                 &cw_vbo_sphere, &cw_count_sphere);
    cw_make_vao (cw_mesh_cube, CW_CUBE_FLOATS, &cw_vao_cube, &cw_vbo_cube,
                 &cw_count_cube);
    cw_make_vao (cw_mesh_cylinder, CW_CYLINDER_FLOATS, &cw_vao_cylinder,
                 &cw_vbo_cylinder, &cw_count_cylinder);
    cw_make_vao (cw_mesh_grid, CW_GRID_FLOATS, &cw_vao_grid, &cw_vbo_grid,
                 &cw_count_grid);
    cw_make_vao (cw_mesh_crosshair, CW_CROSSHAIR_FLOATS, &cw_vao_crosshair,
                 &cw_vbo_crosshair, &cw_count_crosshair);
}

static void cw_gl_release (void) {
    if (!cw_gl_area) { return; }
    if (gtk_widget_get_realized (cw_gl_area)) {
        gtk_gl_area_make_current (GTK_GL_AREA (cw_gl_area));
        if (gtk_gl_area_get_error (GTK_GL_AREA (cw_gl_area)) == NULL) {
            if (cw_program) { glDeleteProgram (cw_program); }
            const GLuint vaos [5] = {cw_vao_sphere, cw_vao_cube,
                                     cw_vao_cylinder, cw_vao_grid,
                                     cw_vao_crosshair};
            const GLuint vbos [5] = {cw_vbo_sphere, cw_vbo_cube,
                                     cw_vbo_cylinder, cw_vbo_grid,
                                     cw_vbo_crosshair};
            glDeleteVertexArrays (5, vaos);
            glDeleteBuffers (5, vbos);
        }
    }
    cw_program = 0;
    cw_vao_sphere = 0; cw_vbo_sphere = 0; cw_count_sphere = 0;
    cw_vao_cube = 0; cw_vbo_cube = 0; cw_count_cube = 0;
    cw_vao_cylinder = 0; cw_vbo_cylinder = 0; cw_count_cylinder = 0;
    cw_vao_grid = 0; cw_vbo_grid = 0; cw_count_grid = 0;
    cw_vao_crosshair = 0; cw_vbo_crosshair = 0; cw_count_crosshair = 0;
}

static gboolean cw_on_render (GtkGLArea *area, GdkGLContext *context,
                              gpointer user_data) {
    (void) area;
    (void) context;
    (void) user_data;
    int w = 0;
    int h = 0;
    if (!cw_widget_pixels (&w, &h)) { return TRUE; }
    if (cw_program == 0) { return TRUE; }
    glViewport (0, 0, (GLint) w, (GLint) h);
    glClearColor (0.12f, 0.13f, 0.16f, 1.0f);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    cw_camera.aspect = (float) w / (float) h;
    cw_mat4 view;
    cw_mat4 proj;
    cw_mat4 vp;
    cw_mat_look_at (view, &cw_camera);
    cw_mat_perspective (proj, cw_camera.fov_y_deg, cw_camera.aspect, 0.05f, 200.0f);
    cw_mat_multiply (vp, proj, view);
    glUseProgram (cw_program);
    glUniformMatrix4fv (cw_u_viewproj, 1, GL_FALSE, vp);
    /* the grid: unlit, identity model */
    cw_mat4 identity;
    cw_mat_identity (identity);
    const float grid_inv [3] = {1.0f, 1.0f, 1.0f};
    const float grid_col [3] = {0.34f, 0.37f, 0.42f};
    glUniformMatrix4fv (cw_u_model, 1, GL_FALSE, identity);
    glUniform3fv (cw_u_inv_scale, 1, grid_inv);
    glUniform3fv (cw_u_color, 1, grid_col);
    glUniform1f (cw_u_selected, 0.0f);
    glUniform1f (cw_u_unlit, 1.0f);
    glBindVertexArray (cw_vao_grid);
    glDrawArrays (GL_LINES, 0, (GLint) cw_count_grid);
    /* the parts */
    glUniform1f (cw_u_unlit, 0.0f);
    for (int i = 0; i < cw_design.count; i++) {
        const mig_part *p = &cw_design.parts [i];
        vector3 scale = {1.0f, 1.0f, 1.0f};
        float color [3] = {0.5f, 0.5f, 0.5f};
        GLuint vao = cw_vao_sphere;
        int count = cw_count_sphere;
        if (p -> type == mig_part_sphere) {
            scale = (vector3) {p -> radius, p -> radius, p -> radius};
            vao = cw_vao_sphere;
            count = cw_count_sphere;
            color [0] = 0.36f; color [1] = 0.55f; color [2] = 0.90f;
        } else if (p -> type == mig_part_cube) {
            scale = p -> half_extensions;
            vao = cw_vao_cube;
            count = cw_count_cube;
            color [0] = 0.40f; color [1] = 0.72f; color [2] = 0.45f;
        } else {
            /* axle LOCAL X: X extent = half length, Y/Z = radius */
            scale = (vector3) {p -> cylinder_half_length, p -> radius, p -> radius};
            vao = cw_vao_cylinder;
            count = cw_count_cylinder;
            color [0] = 0.90f; color [1] = 0.60f; color [2] = 0.30f;
        }
        const vector3 inv_scale = {1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z};
        cw_mat4 model;
        cw_mat_model (model, p -> position, p -> orientation, scale);
        glUniformMatrix4fv (cw_u_model, 1, GL_FALSE, model);
        glUniform3fv (cw_u_inv_scale, 1, &inv_scale.x);
        glUniform3fv (cw_u_color, 1, color);
        glUniform1f (cw_u_selected, (i == cw_design.selected) ? 1.0f : 0.0f);
        glBindVertexArray (vao);
        glDrawArrays (GL_TRIANGLES, 0, (GLint) count);
    }
    /* WP4 4.6j: the CROSSHAIR while the pointer is locked — the cursor
     * is gone, so the aim point must be drawn. Identity viewproj +
     * model pass the NDC lines straight through (always the exact
     * screen center), depth test off so it reads over anything. */
    if (cw_look_locked) {
        glDisable (GL_DEPTH_TEST);
        glUniformMatrix4fv (cw_u_viewproj, 1, GL_FALSE, identity);
        glUniformMatrix4fv (cw_u_model, 1, GL_FALSE, identity);
        glUniform3fv (cw_u_inv_scale, 1, grid_inv);
        const float cross_col [3] = {0.85f, 0.90f, 0.98f};
        glUniform3fv (cw_u_color, 1, cross_col);
        glUniform1f (cw_u_selected, 0.0f);
        glUniform1f (cw_u_unlit, 1.0f);
        glBindVertexArray (cw_vao_crosshair);
        glDrawArrays (GL_LINES, 0, (GLint) cw_count_crosshair);
        glEnable (GL_DEPTH_TEST);
    }
    glBindVertexArray (0);
    return TRUE;
}

static gboolean cw_tick (gpointer user_data) {
    (void) user_data;
    /* The window's frame loop (16 ms) now runs the ENGINE control
     * scheme (4.6j) AND the 4.6i interaction follow:
     *   - steering: the Wayland relative deltas drain ONCE here (the
     *     input_control_poll_locked_mouse analog; X11 steers per
     *     motion event instead);
     *   - fly: the latched WASD/Space/Shift translate;
     *   - the carried part follows EVERY frame — the crosshair ray
     *     while locked, the stored cursor otherwise (zoom mid-carry
     *     keeps it glued); a settling part falls at the world's own
     *     gravity. dt is the tick period (16 ms). */
    if (cw_look_locked) {
        cw_look_steer ();
        cw_fly_translate (0.016f);
    }
    if (cw_mode == CW_MODE_CARRY) {
        if (cw_look_locked) {
            cw_carry_apply_ndc (0.0f, 0.0f);
        } else {
            cw_carry_update (cw_cursor_x, cw_cursor_y);
        }
    } else if (cw_mode == CW_MODE_SETTLING) {
        cw_settle_tick (0.016f);
    }
    if (cw_gl_area) {
        gtk_gl_area_queue_render (GTK_GL_AREA (cw_gl_area));
    }
    return G_SOURCE_CONTINUE;
}

/* One-shot after present: the window opens VIEWPORT-focused so the CAD
 * keybinds work immediately (the name entry is opt-in by click or
 * Tab). The idle runs after the map, so the grab lands instead of
 * racing GTK's first-focusable-child assignment — the entry is first
 * in the box and would otherwise own the keys on open. */
static gboolean cw_focus_viewport_idle (gpointer user_data) {
    (void) user_data;
    if (cw_gl_area) { gtk_widget_grab_focus (cw_gl_area); }
    return G_SOURCE_REMOVE;
}

static void cw_on_destroy (GtkWidget *widget, gpointer user_data) {
    (void) widget;
    (void) user_data;
    /* WP4 4.6j: free the pointer FIRST — the lock is live GL/GDK
     * state tied to this window's surface and the handlers below
     * null it. */
    cw_look_unlock ();
    if (cw_timeout_id) {
        g_source_remove (cw_timeout_id);
        cw_timeout_id = 0;
    }
    cw_gl_release ();
    cw_window = NULL;
    cw_gl_area = NULL;
    cw_name_entry = NULL;
    cw_spec_dropdown = NULL;
    cw_status_label = NULL;
    cw_mode = CW_MODE_IDLE;
    cw_drag_part = -1;
    /* WP4 4.6i: nothing may survive a close pointing at the design —
     * the settle slot too (the design persists; a stale settle would
     * resume falling on reopen). */
    cw_settle_part = -1;
    cw_settle_v = 0.0f;
    /* WP4 4.6j: the look state dies with the window (idempotent —
     * cw_look_unlock already ran above; this covers a destroy that
     * skipped it). */
    cw_look_locked = 0;
    cw_kill_fly_latches ();
}

/* The engine's on_focus_out analog (root_gtk.c's focus "leave"
 * controller): losing the window frees the pointer and kills the fly
 * latches — a held W must not fly a window the user has already
 * left. */
static void cw_on_focus_leave (GtkEventControllerFocus *ctrl,
                               gpointer user_data) {
    (void) ctrl;
    (void) user_data;
    cw_look_unlock ();
}

static gboolean cw_on_close_request (GtkWidget *widget, gpointer user_data) {
    (void) widget;
    (void) user_data;
    return FALSE; /* let the destroy proceed */
}

static void cw_on_save_clicked (GtkButton *button, gpointer user_data) {
    (void) button;
    (void) user_data;
    cw_save_design ();
}

static void cw_on_load_clicked (GtkButton *button, gpointer user_data) {
    (void) button;
    (void) user_data;
    cw_load_selected ();
}

static void cw_on_spawn_clicked (GtkButton *button, gpointer user_data) {
    (void) button;
    (void) user_data;
    cw_spawn_to_world ();
}

static void cw_on_name_activate (GtkEntry *entry, gpointer user_data) {
    (void) entry;
    (void) user_data;
    cw_rename_from_entry ();
}

/* ---- the window -------------------------------------------------------- */

void cad_window_toggle (void) {
    if (!cw_design_ready) {
        cad_design_init (&cw_design);
        /* A default pose so the first open shows a usable rig: the
         * camera looks at the grid origin from the usual 3/4 angle. */
        cw_camera.target = (vector3) {0.0f, 0.4f, 0.0f};
        cw_camera.yaw_deg = 30.0f;
        cw_camera.pitch_deg = 25.0f;
        cw_camera.distance = 9.0f;
        cw_camera.fov_y_deg = 50.0f;
        cw_camera.aspect = 1.5f;
        /* WP4 4.6h: new designs start with grid snap ON at 0.25 m —
         * placement-in-grids is the default; G frees the placement.
         * (The gated model's own default stays 0; this is the window's
         * GUI preference.) */
        cw_design.snap_step = 0.25f;
        cw_design_ready = 1;
    }
    if (cw_window) {
        gtk_window_destroy (GTK_WINDOW (cw_window));
        return;
    }
    /* MIG WP4 4.6f: taking over the pointer means the engine's pointer
     * grab must die FIRST. With the relative-pointer lock live, mouse
     * motion over any window — this one included — fed the main camera
     * (the 2-window report), and on X11 the re-centre warp fought the
     * cursor. mouse_lock_disable is idempotent; the main window's
     * focus-out releases it too (the generic fix), this explicit
     * release is the guarantee if focus transfer never happens. The
     * lock re-engages on the next deliberate main-window click. */
    if (main_inputs.is_mouse_locked) {
        main_inputs.is_mouse_locked = false;
        if (g_gl_area && GTK_IS_WIDGET (g_gl_area)) {
            mouse_lock_disable (g_gl_area);
        }
    }
    cw_window = gtk_window_new ();
    gtk_widget_set_name (cw_window, "mpe-cad-window");
    gtk_window_set_default_size (GTK_WINDOW (cw_window), 900, 640);
    gtk_window_set_title (GTK_WINDOW (cw_window), "MPE CAD");
    if (g_gl_area) {
        GtkRoot *toplevel = gtk_widget_get_root (g_gl_area);
        if (toplevel && GTK_IS_WINDOW (toplevel)) {
            gtk_window_set_transient_for (GTK_WINDOW (cw_window),
                                          GTK_WINDOW (toplevel));
        }
    }
    g_signal_connect (cw_window, "destroy", G_CALLBACK (cw_on_destroy), NULL);
    g_signal_connect (cw_window, "close-request",
                      G_CALLBACK (cw_on_close_request), NULL);
    GtkEventController *keys = gtk_event_controller_key_new ();
    gtk_event_controller_set_propagation_phase (keys, GTK_PHASE_CAPTURE);
    gtk_widget_add_controller (cw_window, keys);
    g_signal_connect (keys, "key-pressed", G_CALLBACK (cw_on_key), NULL);
    /* WP4 4.6j: the engine's latch pattern needs the release edge too
     * (root_gtk.c wires the same pair on the main window). */
    g_signal_connect (keys, "key-released", G_CALLBACK (cw_on_key_release),
                      NULL);
    /* The engine's focus-out release (root_gtk.c:118 pattern): losing
     * the window frees the pointer and the fly latches. */
    GtkEventController *focus_ctrl = gtk_event_controller_focus_new ();
    g_signal_connect (focus_ctrl, "leave", G_CALLBACK (cw_on_focus_leave),
                      NULL);
    gtk_widget_add_controller (cw_window, focus_ctrl);

    static bool css_installed = false;
    if (!css_installed) {
        GtkCssProvider *css = gtk_css_provider_new ();
        gtk_css_provider_load_from_string (css,
            "#mpe-cad-window { background: rgba(16,17,22,0.96); }\n"
            "#mpe-cad-window label { color: #e8e8f0; font-size: 12px; }\n");
        gtk_style_context_add_provider_for_display (
            gdk_display_get_default (), GTK_STYLE_PROVIDER (css),
            GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
        g_object_unref (css);
        css_installed = true;
    }

    /* top bar: name entry + spec dropdown + load/save */
    GtkWidget *bar = gtk_box_new (GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_widget_set_margin_top (bar, 8);
    gtk_widget_set_margin_bottom (bar, 8);
    gtk_widget_set_margin_start (bar, 8);
    gtk_widget_set_margin_end (bar, 8);
    GtkWidget *name_label = gtk_label_new ("Name");
    gtk_label_set_xalign (GTK_LABEL (name_label), 0.0f);
    gtk_box_append (GTK_BOX (bar), name_label);
    cw_name_entry = gtk_entry_new ();
    gtk_editable_set_text (GTK_EDITABLE (cw_name_entry), cw_design.name);
    gtk_widget_set_hexpand (cw_name_entry, TRUE);
    g_signal_connect (cw_name_entry, "activate",
                      G_CALLBACK (cw_on_name_activate), NULL);
    gtk_box_append (GTK_BOX (bar), cw_name_entry);
    GtkWidget *load_btn = gtk_button_new_with_label ("Load");
    g_signal_connect (load_btn, "clicked", G_CALLBACK (cw_on_load_clicked), NULL);
    gtk_box_append (GTK_BOX (bar), load_btn);
    GtkWidget *save_btn = gtk_button_new_with_label ("Save");
    gtk_widget_add_css_class (save_btn, "suggested-action");
    g_signal_connect (save_btn, "clicked", G_CALLBACK (cw_on_save_clicked), NULL);
    gtk_box_append (GTK_BOX (bar), save_btn);
    /* WP4 4.6e: the world hookup button (P does the same in the
     * viewport). The spawned composite is the design's own body in the
     * shared world — the main viewport shows it exactly where the CAD
     * grid had it. */
    GtkWidget *spawn_btn = gtk_button_new_with_label ("Spawn to world");
    g_signal_connect (spawn_btn, "clicked", G_CALLBACK (cw_on_spawn_clicked),
                      NULL);
    gtk_box_append (GTK_BOX (bar), spawn_btn);
    cw_spec_dropdown = gtk_drop_down_new (NULL, NULL);
    gtk_widget_set_hexpand (cw_spec_dropdown, TRUE);
    gtk_box_append (GTK_BOX (bar), cw_spec_dropdown);

    cw_status_label = gtk_label_new ("MPE CAD");
    gtk_label_set_xalign (GTK_LABEL (cw_status_label), 0.0f);
    gtk_widget_set_margin_start (cw_status_label, 8);
    gtk_widget_set_margin_end (cw_status_label, 8);

    /* the viewport */
    cw_gl_area = gtk_gl_area_new ();
    gtk_gl_area_set_has_depth_buffer (GTK_GL_AREA (cw_gl_area), TRUE);
    gtk_gl_area_set_allowed_apis (GTK_GL_AREA (cw_gl_area), GDK_GL_API_GL);
    gtk_widget_set_vexpand (cw_gl_area, TRUE);
    /* WP4 4.6g: the viewport is a focus stop, so a click into it
     * returns the keyboard from the typing block (the CAD keybinds
     * follow the focus). */
    gtk_widget_set_focusable (cw_gl_area, TRUE);
    g_signal_connect (cw_gl_area, "realize", G_CALLBACK (cw_on_realize), NULL);
    g_signal_connect (cw_gl_area, "render", G_CALLBACK (cw_on_render), NULL);
    GtkEventController *motion = gtk_event_controller_motion_new ();
    g_signal_connect (motion, "motion", G_CALLBACK (cw_on_motion), NULL);
    gtk_widget_add_controller (cw_gl_area, motion);
    GtkGesture *click = gtk_gesture_click_new ();
    gtk_gesture_single_set_button (GTK_GESTURE_SINGLE (click), 0);
    g_signal_connect (click, "pressed", G_CALLBACK (cw_on_press), NULL);
    g_signal_connect (click, "released", G_CALLBACK (cw_on_release), NULL);
    gtk_widget_add_controller (cw_gl_area, GTK_EVENT_CONTROLLER (click));
    GtkEventController *scroll = gtk_event_controller_scroll_new (
        GTK_EVENT_CONTROLLER_SCROLL_BOTH_AXES);
    g_signal_connect (scroll, "scroll", G_CALLBACK (cw_on_scroll), NULL);
    gtk_widget_add_controller (cw_gl_area, scroll);

    GtkWidget *box = gtk_box_new (GTK_ORIENTATION_VERTICAL, 4);
    gtk_box_append (GTK_BOX (box), bar);
    gtk_box_append (GTK_BOX (box), cw_status_label);
    gtk_box_append (GTK_BOX (box), cw_gl_area);
    gtk_window_set_child (GTK_WINDOW (cw_window), box);

    cw_timeout_id = g_timeout_add (16, cw_tick, NULL);
    cw_refresh_dropdown ();
    cw_status_refresh ();
    gtk_window_present (GTK_WINDOW (cw_window));
    g_idle_add (cw_focus_viewport_idle, NULL);
}

int cad_window_active (void) {
    return (cw_window != NULL) ? 1 : 0;
}

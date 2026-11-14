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
 * Interaction (first-cut toolset, the user's confirmed set):
 *   - LMB on a part: select + drag-move on the part-height work plane
 *     (y-up; the plane follows the part's current height). LMB on
 *     empty space: orbit (drag), Shift+drag: pan, wheel: zoom.
 *   - Keys: 1/2/3 add sphere/cube/cylinder at the spawn point (the
 *     camera target's XZ at the part's identity-orientation rest
 *     height), Delete removes the selection, D duplicates it (+0.5 m
 *     X, first-cut offset), A arrays it (3 copies, 0.5 m X step —
 *     first-cut), X/Y/Z rotate the selection +-15 degrees about the
 *     ASSEMBLY axis (Shift = -15), G toggles snap 0 <-> 0.25 m,
 *     R toggles rot-snap 0 <-> 15 degrees, S saves, L loads the spec
 *     selected in the dropdown, P spawns the design into the shared
 *     world (the 4.6e hookup; there is also a Spawn button), Esc
 *     closes.
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
 * Determinism: this is authoring-path math (a human holds the mouse);
 * sinf/cosf here are the documented cold-path ruling, the per-tick
 * solver ban is untouched. MATH_PI (math3d.h), never M_PI.
 */
#include "cad_window.h"
#include "../mpe_engine.h"
#include "../cad/cad_editor.h"
#include "../cad/cad_view_math.h"
#include "term_priv.h"
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

/* Interaction modes: one drag at a time. */
enum {
    CW_MODE_IDLE = 0,
    CW_MODE_ORBIT, /* LMB on empty space */
    CW_MODE_PAN,   /* Shift+LMB */
    CW_MODE_DRAG   /* LMB on a part: work-plane move */
};
static int cw_mode = CW_MODE_IDLE;
static double cw_last_x = 0.0;
static double cw_last_y = 0.0;
static int cw_drag_part = -1;     /* index while CW_MODE_DRAG */
static float cw_drag_plane_y = 0.0f;  /* the work plane's height */
static vector3 cw_drag_offset = {0.0f, 0.0f, 0.0f}; /* part - plane hit */

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

/* Vertex format: {pos3, normal3} interleaved (6 floats). The unit
 * meshes are built once per open into these static buffers:
 *   sphere: 12 rings x 16 sectors x 2 tris x 3 verts x 6 floats
 *   cube:   6 faces x 2 tris x 3 verts x 6 floats
 *   cyl:    24 segs x (2 side + 2 caps) tris x 3 verts x 6 floats
 *   grid:   21 lines x 2 axes x 2 verts x 6 floats */
#define CW_SPHERE_FLOATS (12 * 16 * 2 * 3 * 6)
#define CW_CUBE_FLOATS (6 * 2 * 3 * 6)
#define CW_CYLINDER_FLOATS (24 * 4 * 3 * 6)
#define CW_GRID_FLOATS (21 * 2 * 2 * 6)
static float cw_mesh_sphere [CW_SPHERE_FLOATS];
static float cw_mesh_cube [CW_CUBE_FLOATS];
static float cw_mesh_cylinder [CW_CYLINDER_FLOATS];
static float cw_mesh_grid [CW_GRID_FLOATS];

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
              "parts %d/%d | sel %s | snap %.3g m | rotsnap %.3g deg | %s",
              cw_design.count, cw_design.max_parts, sel,
              (double) cw_design.snap_step, (double) cw_design.rot_snap_deg,
              cw_design.name);
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

/* ---- input handlers ---------------------------------------------------- */

static void cw_on_press (GtkGestureClick *gesture, gint n_press, gdouble x,
                         gdouble y, gpointer user_data) {
    (void) n_press;
    (void) user_data;
    const guint button = gtk_gesture_single_get_current_button (
        GTK_GESTURE_SINGLE (gesture));
    if (button != 1) { return; } /* LMB only */
    GdkModifierType state = 0;
    GdkEvent *event = gtk_event_controller_get_current_event (
        GTK_EVENT_CONTROLLER (gesture));
    if (event) { state = gdk_event_get_modifier_state (event); }
    cw_last_x = x;
    cw_last_y = y;
    if (state & GDK_SHIFT_MASK) {
        cw_mode = CW_MODE_PAN;
        return;
    }
    float t = 0.0f;
    vector3 point = {0.0f, 0.0f, 0.0f};
    const int picked = cw_pick (x, y, &t, &point);
    if (picked < 0) {
        cw_mode = CW_MODE_ORBIT;
        return;
    }
    cad_select (&cw_design, picked);
    /* Drag on the part-height work plane: y = the part's CURRENT
     * height; the grab offset freezes the part's local grip point so
     * the part doesn't jump to the cursor. */
    cw_drag_part = picked;
    cw_drag_plane_y = cw_design.parts [picked].position.y;
    cw_drag_offset = vector3_subtraction (cw_design.parts [picked].position, point);
    cw_mode = CW_MODE_DRAG;
    cw_status_refresh ();
}

static void cw_on_release (GtkGestureClick *gesture, gint n_press, gdouble x,
                           gdouble y, gpointer user_data) {
    (void) gesture;
    (void) n_press;
    (void) x;
    (void) y;
    (void) user_data;
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
    if (cw_mode == CW_MODE_ORBIT) {
        /* Turntable: drag right orbits the camera right around the
         * target; drag down lifts the eye. Pitch clamped HERE (the
         * caller's job per the view-math contract). */
        cw_camera.yaw_deg += dx * 0.4f;
        cw_camera.pitch_deg += dy * 0.4f;
        if (cw_camera.pitch_deg > 85.0f) { cw_camera.pitch_deg = 85.0f; }
        if (cw_camera.pitch_deg < -85.0f) { cw_camera.pitch_deg = -85.0f; }
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
    if (cw_mode == CW_MODE_DRAG) {
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

/* CAD keys skip the name entry (typing "4sphere2" must not spawn
 * spheres): only Esc is handled while the entry holds focus. */
static gboolean cw_on_key (GtkEventControllerKey *controller, guint keyval,
                           guint keycode, GdkModifierType state,
                           gpointer user_data) {
    (void) controller;
    (void) keycode;
    (void) user_data;
    if (keyval == GDK_KEY_Escape) {
        if (cw_window) { gtk_window_destroy (GTK_WINDOW (cw_window)); }
        return TRUE;
    }
    if (cw_window && (gtk_window_get_focus (GTK_WINDOW (cw_window)) ==
                      cw_name_entry)) {
        return FALSE;
    }
    /* Mid-drag the selection indices can shift under a Delete (the
     * model shifts part indices down), which would retarget the drag
     * at the wrong part — CAD verbs are frozen until the button
     * releases. Esc still closes. */
    if (cw_mode == CW_MODE_DRAG) {
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
        case GDK_KEY_s:
        case GDK_KEY_S: cw_save_design (); return TRUE;
        case GDK_KEY_l:
        case GDK_KEY_L: cw_load_selected (); return TRUE;
        case GDK_KEY_p:
        case GDK_KEY_P: cw_spawn_to_world (); return TRUE;
        default: return FALSE;
    }
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
    cw_make_vao (cw_mesh_sphere, CW_SPHERE_FLOATS, &cw_vao_sphere,
                 &cw_vbo_sphere, &cw_count_sphere);
    cw_make_vao (cw_mesh_cube, CW_CUBE_FLOATS, &cw_vao_cube, &cw_vbo_cube,
                 &cw_count_cube);
    cw_make_vao (cw_mesh_cylinder, CW_CYLINDER_FLOATS, &cw_vao_cylinder,
                 &cw_vbo_cylinder, &cw_count_cylinder);
    cw_make_vao (cw_mesh_grid, CW_GRID_FLOATS, &cw_vao_grid, &cw_vbo_grid,
                 &cw_count_grid);
}

static void cw_gl_release (void) {
    if (!cw_gl_area) { return; }
    if (gtk_widget_get_realized (cw_gl_area)) {
        gtk_gl_area_make_current (GTK_GL_AREA (cw_gl_area));
        if (gtk_gl_area_get_error (GTK_GL_AREA (cw_gl_area)) == NULL) {
            if (cw_program) { glDeleteProgram (cw_program); }
            const GLuint vaos [4] = {cw_vao_sphere, cw_vao_cube,
                                     cw_vao_cylinder, cw_vao_grid};
            const GLuint vbos [4] = {cw_vbo_sphere, cw_vbo_cube,
                                     cw_vbo_cylinder, cw_vbo_grid};
            glDeleteVertexArrays (4, vaos);
            glDeleteBuffers (4, vbos);
        }
    }
    cw_program = 0;
    cw_vao_sphere = 0; cw_vbo_sphere = 0; cw_count_sphere = 0;
    cw_vao_cube = 0; cw_vbo_cube = 0; cw_count_cube = 0;
    cw_vao_cylinder = 0; cw_vbo_cylinder = 0; cw_count_cylinder = 0;
    cw_vao_grid = 0; cw_vbo_grid = 0; cw_count_grid = 0;
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
    glBindVertexArray (0);
    return TRUE;
}

static gboolean cw_tick (gpointer user_data) {
    (void) user_data;
    if (cw_gl_area) {
        gtk_gl_area_queue_render (GTK_GL_AREA (cw_gl_area));
    }
    return G_SOURCE_CONTINUE;
}

static void cw_on_destroy (GtkWidget *widget, gpointer user_data) {
    (void) widget;
    (void) user_data;
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
        cw_design_ready = 1;
    }
    if (cw_window) {
        gtk_window_destroy (GTK_WINDOW (cw_window));
        return;
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
}

int cad_window_active (void) {
    return (cw_window != NULL) ? 1 : 0;
}

/* MIG WP4 4.6: miniature CAD editor — design model. See cad_editor.h
 * for the contract (standalone: nothing in the shipped GUI calls this
 * yet; persistence = the frozen spec grammar; build = the mig_bridge
 * seam; every verb is write-on-success with named errors). */
#include "cad_editor.h"
#include <stdio.h>
#include <string.h>

/* ---- internals ------------------------------------------------------ */

/* The engine's own unit check, mirrored (mig_mass.c mig_compose pass 0):
 * n2 = |q|^2 in double, reject outside 1 +- 1e-4 or non-finite. */
static bool cad_quat_unit (vector4 q) {
    const double n2 = (double) q.w * (double) q.w +
                      (double) q.x * (double) q.x +
                      (double) q.y * (double) q.y +
                      (double) q.z * (double) q.z;
    const double a = (n2 > 1.0) ? (n2 - 1.0) : (1.0 - n2);
    return isfinite (n2) && isfinite (a) && (a <= 1e-4);
}

static bool cad_finite3 (vector3 v) {
    return isfinite (v.x) && isfinite (v.y) && isfinite (v.z);
}

/* mig_part_tensor_local's dims rules, mirrored for early refusal:
 * per-type subsets, every dimension finite and strictly positive. */
static bool cad_dims_valid (const mig_part *p) {
    if (!isfinite (p -> radius) || !isfinite (p -> cylinder_half_length) ||
        !cad_finite3 (p -> half_extensions)) {
        return false;
    }
    switch (p -> type) {
        case mig_part_sphere: return p -> radius > 0.0f;
        case mig_part_cube: return (p -> half_extensions.x > 0.0f) &&
                                   (p -> half_extensions.y > 0.0f) &&
                                   (p -> half_extensions.z > 0.0f);
        case mig_part_cylinder: return (p -> radius > 0.0f) &&
                                       (p -> cylinder_half_length > 0.0f);
        default: return false;
    }
}

/* One shared validator for every part-entering verb. Returns the
 * same-named cad_err the compose path would surface later — refusing
 * at the door keeps the design always buildable. */
static int cad_validate_part (const mig_part *p) {
    if (!p) { return cad_null; }
    if ((p -> type != mig_part_sphere) && (p -> type != mig_part_cube) &&
        (p -> type != mig_part_cylinder)) {
        return cad_bad_type;
    }
    if (!cad_dims_valid (p)) { return cad_bad_dims; }
    if (!cad_finite3 (p -> position)) { return cad_bad_pose; }
    if (!cad_quat_unit (p -> orientation)) { return cad_bad_quat; }
    if (!isfinite (p -> mass) || (p -> mass <= 0.0f)) { return cad_bad_mass; }
    return cad_ok;
}

/* Snapping is a mode: every position write lands on a snap_step
 * multiple (axis-wise) when enabled. roundf is exact (not trig). */
static float cad_snap_component (float value, float step) {
    if (!(step > 0.0f)) { return value; }
    return roundf (value / step) * step;
}

static vector3 cad_snap_position (vector3 position, float step) {
    return (vector3) {cad_snap_component (position.x, step),
                      cad_snap_component (position.y, step),
                      cad_snap_component (position.z, step)};
}

/* The spec grammar's name charset ([A-Za-z0-9_-], 1..63 bytes). */
static bool cad_name_legal (const char *name) {
    if (!name) { return false; }
    size_t len = strlen (name);
    if ((len == 0) || (len >= MIG_SPEC_NAME_LEN)) { return false; }
    for (size_t i = 0; i < len; i++) {
        const char c = name [i];
        if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
              (c >= '0' && c <= '9') || (c == '_') || (c == '-'))) {
            return false;
        }
    }
    return true;
}

/* ---- lifecycle ------------------------------------------------------ */

void cad_design_init (cad_design *design) {
    if (!design) { return; }
    memset (design, 0, sizeof (*design));
    design -> count = 0;
    design -> max_parts = CAD_MAX_PARTS;
    design -> name [0] = '\0';
    design -> selected = -1;
    design -> snap_step = 0.0f;
    design -> rot_snap_deg = 0.0f;
}

int cad_set_name (cad_design *design, const char *name) {
    if ((!design) || (!name)) { return cad_null; }
    if (!cad_name_legal (name)) { return cad_bad_name; }
    memset (design -> name, 0, sizeof (design -> name));
    strcpy (design -> name, name);
    return cad_ok;
}

/* ---- part CRUD ------------------------------------------------------ */

static int cad_append_validated (cad_design *design, const mig_part *part) {
    int e = cad_validate_part (part);
    if (e != cad_ok) { return e; }
    if (!design) { return cad_null; }
    if (design -> count >= design -> max_parts) { return cad_full; }
    design -> parts [design -> count++] = *part;
    return cad_ok;
}

int cad_add_part (cad_design *design, const mig_part *part) {
    return cad_append_validated (design, part);
}

int cad_add_sphere (cad_design *design, float radius, vector3 position) {
    mig_part p;
    memset (&p, 0, sizeof (p));
    p.type = mig_part_sphere;
    p.radius = radius;
    p.position = position;
    p.orientation = vector4_identity ();
    p.mass_mode = mig_mass_mode_density;
    p.mass = CAD_DEFAULT_DENSITY;
    return cad_append_validated (design, &p);
}

int cad_add_cube (cad_design *design, vector3 half_extensions, vector3 position) {
    mig_part p;
    memset (&p, 0, sizeof (p));
    p.type = mig_part_cube;
    p.half_extensions = half_extensions;
    p.position = position;
    p.orientation = vector4_identity ();
    p.mass_mode = mig_mass_mode_density;
    p.mass = CAD_DEFAULT_DENSITY;
    return cad_append_validated (design, &p);
}

int cad_add_cylinder (cad_design *design, float radius, float half_length,
                      vector3 position) {
    mig_part p;
    memset (&p, 0, sizeof (p));
    p.type = mig_part_cylinder;
    p.radius = radius;
    p.cylinder_half_length = half_length;
    p.position = position;
    p.orientation = vector4_identity ();
    p.mass_mode = mig_mass_mode_density;
    p.mass = CAD_DEFAULT_DENSITY;
    return cad_append_validated (design, &p);
}

int cad_delete (cad_design *design, int index) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    for (int i = index; i < (design -> count - 1); i++) {
        design -> parts [i] = design -> parts [i + 1];
    }
    design -> count--;
    memset (&design -> parts [design -> count], 0, sizeof (mig_part));
    /* Selection follows the part, or drops off the end. */
    if (design -> selected == index) {
        design -> selected = (design -> selected < design -> count) ? design -> selected : -1;
    } else if (design -> selected > index) {
        design -> selected--;
    }
    return cad_ok;
}

int cad_duplicate (cad_design *design, int index, vector3 offset) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    if (!cad_finite3 (offset)) { return cad_bad_pose; }
    if (design -> count >= design -> max_parts) { return cad_full; }
    mig_part copy = design -> parts [index];
    copy.position = cad_snap_position (
        (vector3) {copy.position.x + offset.x, copy.position.y + offset.y,
                   copy.position.z + offset.z},
        design -> snap_step);
    return cad_append_validated (design, &copy);
}

int cad_array (cad_design *design, int index, int n, vector3 step) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    if (n <= 0) { return cad_bad_arg; }
    if (!cad_finite3 (step)) { return cad_bad_pose; }
    if ((design -> count + (n - 1)) > design -> max_parts) { return cad_full; }
    const mig_part original = design -> parts [index];
    /* k = 1..n-1 copies, cumulative steps, appended in order. The whole
     * pattern is validated BEFORE the first append (a mid-pattern
     * refusal must not leave a half-pattern behind). */
    mig_part staged [CAD_MAX_PARTS];
    for (int k = 1; k < n; k++) {
        staged [k] = original;
        staged [k].position = cad_snap_position (
            (vector3) {original.position.x + step.x * (float) k,
                       original.position.y + step.y * (float) k,
                       original.position.z + step.z * (float) k},
            design -> snap_step);
        int e = cad_validate_part (&staged [k]);
        if (e != cad_ok) { return e; }
    }
    for (int k = 1; k < n; k++) {
        design -> parts [design -> count++] = staged [k];
    }
    return cad_ok;
}

/* ---- transforms ------------------------------------------------------ */

int cad_move (cad_design *design, int index, vector3 delta) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    if (!cad_finite3 (delta)) { return cad_bad_pose; }
    vector3 p = design -> parts [index].position;
    design -> parts [index].position = cad_snap_position (
        (vector3) {p.x + delta.x, p.y + delta.y, p.z + delta.z},
        design -> snap_step);
    return cad_ok;
}

int cad_set_position (cad_design *design, int index, vector3 position) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    if (!cad_finite3 (position)) { return cad_bad_pose; }
    design -> parts [index].position = cad_snap_position (position, design -> snap_step);
    return cad_ok;
}

int cad_rotate_degrees (cad_design *design, int index, int axis, float degrees) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    if ((axis < 0) || (axis > 2)) { return cad_bad_arg; }
    if (!isfinite (degrees)) { return cad_bad_arg; }
    float d = degrees;
    if (design -> rot_snap_deg > 0.0f) {
        d = roundf (d / design -> rot_snap_deg) * design -> rot_snap_deg;
    }
    const vector3 axes [3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    /* The documented cold-path authoring helper (libm trig); degrees ->
     * radians via the house fixed double-pi literal (math3d.h), never
     * the non-standard M_PI macro. */
    vector4 q_axis = vector4_from_axis_with_angle (axes [axis], d * (float) MATH_PI / 180.0f);
    /* Compose on the LEFT: the axis is an ASSEMBLY-frame direction, so
     * the part's current orientation must not rotate it. */
    design -> parts [index].orientation =
        vector4_multiplication (q_axis, design -> parts [index].orientation);
    return cad_ok;
}

int cad_set_mass (cad_design *design, int index, mig_mass_mode mode, float value) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    if ((mode != mig_mass_mode_explicit) && (mode != mig_mass_mode_density)) {
        return cad_bad_arg;
    }
    if (!isfinite (value) || (value <= 0.0f)) { return cad_bad_mass; }
    design -> parts [index].mass_mode = mode;
    design -> parts [index].mass = value;
    return cad_ok;
}

int cad_set_snap (cad_design *design, float snap_step) {
    if (!design) { return cad_null; }
    if (!isfinite (snap_step) || (snap_step < 0.0f)) { return cad_bad_arg; }
    design -> snap_step = snap_step;
    return cad_ok;
}

int cad_set_rot_snap (cad_design *design, float rot_snap_deg) {
    if (!design) { return cad_null; }
    if (!isfinite (rot_snap_deg) || (rot_snap_deg < 0.0f)) { return cad_bad_arg; }
    design -> rot_snap_deg = rot_snap_deg;
    return cad_ok;
}

int cad_snap_all (cad_design *design) {
    if (!design) { return cad_null; }
    for (int i = 0; i < design -> count; i++) {
        design -> parts [i].position =
            cad_snap_position (design -> parts [i].position, design -> snap_step);
    }
    return cad_ok;
}

int cad_select (cad_design *design, int index) {
    if (!design) { return cad_null; }
    if ((index != -1) && ((index < 0) || (index >= design -> count))) {
        return cad_bad_index;
    }
    design -> selected = index;
    return cad_ok;
}

/* ---- persistence ------------------------------------------------------ */

int cad_design_to_spec (const cad_design *design, mig_spec *out) {
    if ((!design) || (!out)) { return cad_null; }
    if (design -> count <= 0) { return cad_empty; }
    if (!cad_name_legal (design -> name)) { return cad_bad_name; }
    /* Write-on-success: build the whole spec in a local first — a late
     * part refusal in the loop must leave *out untouched, not hand the
     * caller a half-written spec. */
    mig_spec staged;
    memset (&staged, 0, sizeof (staged));
    strcpy (staged.name, design -> name);
    staged.has_frame = 1; /* the assembly frame is the design's frame */
    for (int i = 0; i < design -> count; i++) {
        const mig_part *p = &design -> parts [i];
        int e = cad_validate_part (p);
        if (e != cad_ok) { return e; }
        mig_spec_part *s = &staged.parts [i];
        s -> type = (int) p -> type;
        s -> radius = (double) p -> radius;
        s -> cylinder_half_length = (double) p -> cylinder_half_length;
        s -> half_extensions_x = (double) p -> half_extensions.x;
        s -> half_extensions_y = (double) p -> half_extensions.y;
        s -> half_extensions_z = (double) p -> half_extensions.z;
        s -> position_x = (double) p -> position.x;
        s -> position_y = (double) p -> position.y;
        s -> position_z = (double) p -> position.z;
        s -> orientation_w = (double) p -> orientation.w;
        s -> orientation_x = (double) p -> orientation.x;
        s -> orientation_y = (double) p -> orientation.y;
        s -> orientation_z = (double) p -> orientation.z;
        s -> mass_mode = (int) p -> mass_mode;
        s -> mass = (double) p -> mass;
        s -> has_mass = 1;
        s -> has_dims = 1;
        s -> has_pose = 1;
    }
    staged.count = design -> count;
    *out = staged;
    return cad_ok;
}

int cad_design_from_spec (const mig_spec *spec, int max_parts, cad_design *out) {
    if ((!spec) || (!out)) { return cad_null; }
    if (max_parts <= 0) { return cad_bad_arg; }
    if ((spec -> count <= 0) || (spec -> count > max_parts)) { return cad_full; }
    if (!cad_name_legal (spec -> name)) { return cad_bad_name; }
    cad_design staged;
    cad_design_init (&staged);
    staged.max_parts = max_parts;
    int e = cad_set_name (&staged, spec -> name);
    if (e != cad_ok) { return e; }
    for (int i = 0; i < spec -> count; i++) {
        const mig_spec_part *s = &spec -> parts [i];
        mig_part p;
        memset (&p, 0, sizeof (p));
        p.type = (mig_part_type) s -> type;
        p.radius = (float) s -> radius;
        p.cylinder_half_length = (float) s -> cylinder_half_length;
        p.half_extensions = (vector3) {(float) s -> half_extensions_x,
                                       (float) s -> half_extensions_y,
                                       (float) s -> half_extensions_z};
        p.position = (vector3) {(float) s -> position_x, (float) s -> position_y,
                                (float) s -> position_z};
        p.orientation = (vector4) {(float) s -> orientation_w, (float) s -> orientation_x,
                                   (float) s -> orientation_y, (float) s -> orientation_z};
        p.mass_mode = (mig_mass_mode) s -> mass_mode;
        p.mass = (float) s -> mass;
        e = cad_append_validated (&staged, &p);
        if (e != cad_ok) { return e; }
    }
    /* Whole-design success only now: hand the staged design over. */
    *out = staged;
    return cad_ok;
}

int cad_save (const cad_design *design, const char *path) {
    if ((!design) || (!path) || (path [0] == '\0')) { return cad_null; }
    if (design -> count <= 0) { return cad_empty; }
    mig_spec spec;
    int e = cad_design_to_spec (design, &spec);
    if (e != cad_ok) { return e; }
    /* Worst-case normalized text: 16 parts x ~200 bytes + head — the
     * frozen grammar's canonical form is line-per-field; 16384 is
     * generous headroom over MIG_SPEC_LINE_CAP 512 x ~32 lines. NOTE:
     * this emitter has NO cap-0 size probe (with cap 0 it returns 0
     * immediately), so measure by emitting for real — the return value
     * is the byte offset written, and the overflow case returns the cap
     * itself, which the check below refuses. (A cap-0 probe here wrote
     * empty files while reporting cad_ok — pinned by the 4.6 gate.) */
    char text [16384];
    size_t needed = mig_spec_emit_normalized (&spec, text, sizeof (text));
    if (needed >= sizeof (text)) { return cad_io; }
    FILE *f = fopen (path, "w");
    if (!f) { return cad_io; }
    size_t written = fwrite (text, 1, needed, f);
    int closed = fclose (f);
    if ((written != needed) || (closed != 0)) { return cad_io; }
    return cad_ok;
}

int cad_load (cad_design *design, const char *path, int max_parts, int *spec_err_out) {
    if ((!design) || (!path) || (path [0] == '\0')) { return cad_null; }
    if (max_parts <= 0) { return cad_bad_arg; }
    if (spec_err_out) { *spec_err_out = mig_spec_ok; }
    mig_spec parsed;
    int rc = mig_spec_parse_file (path, max_parts, &parsed);
    if (rc != mig_spec_ok) {
        if (spec_err_out) { *spec_err_out = rc; }
        return cad_spec_reject; /* design untouched (write-on-success) */
    }
    cad_design staged;
    int e = cad_design_from_spec (&parsed, max_parts, &staged);
    if (e != cad_ok) { return e; }
    *design = staged;
    return cad_ok;
}

/* ---- the engine seam -------------------------------------------------- */

int cad_build (const cad_design *design, mig_part *out_parts, mig_bridge *out) {
    if ((!design) || (!out_parts) || (!out)) { return cad_null; }
    if (design -> count <= 0) { return cad_empty; }
    int rc = mig_make_bridge (design -> parts, design -> count, out_parts, out);
    switch (rc) {
        case mig_compose_ok: return cad_ok;
        case mig_compose_null: return cad_null;
        case mig_compose_bad_type: return cad_bad_type;
        case mig_compose_bad_dims: return cad_bad_dims;
        case mig_compose_bad_quat: return cad_bad_quat;
        case mig_compose_bad_mass: return cad_bad_mass;
        default: return cad_bad_arg; /* unexpected compose code — refuse loudly */
    }
}

/* ---- the ASCII workbench ---------------------------------------------- */

static char cad_plan_glyph (mig_part_type type) {
    switch (type) {
        case mig_part_sphere: return 'o';
        case mig_part_cube: return '#';
        case mig_part_cylinder: return '|';
        default: return '?';
    }
}

int cad_preview (const cad_design *design, char *buffer, int cap) {
    if ((!design) || (!buffer)) { return cad_null; }
    if (cap < CAD_PREVIEW_MIN_CAP) { return cad_bad_arg; }
    const int gw = 64, gh = 24; /* columns x rows */
    const float cells_per_metre = 4.0f; /* 1 cell = 0.25 m */
    const int origin_col = gw / 2, origin_row = gh / 2;
    static char grid [24][65];
    /* Clear EVERY cell including the last row: static storage means a
     * partial memset (all but the final 65 bytes) would let row 23 keep
     * stale glyphs from a previous call after the design mutates. The
     * NUL terminators are re-written below for the same reason. */
    memset (grid, ' ', sizeof (grid));
    for (int r = 0; r < gh; r++) { grid [r][gw] = '\0'; }
    /* Axis ticks through the origin cell (a design always has parts, so
     * the origin is on screen by construction of the fixed window). */
    for (int c = 0; c < gw; c++) { grid [origin_row][c] = (c == origin_col) ? '+' : '-'; }
    for (int r = 0; r < gh; r++) { grid [r][origin_col] = (r == origin_row) ? '+' : '|'; }
    int outside = 0;
    double mass_total = 0.0;
    for (int i = 0; i < design -> count; i++) {
        const mig_part *p = &design -> parts [i];
        double m = 0.0;
        if (mig_part_mass (p, &m) == mig_compose_ok) { mass_total += m; }
        const int col = origin_col + (int) roundf (p -> position.x * cells_per_metre);
        const int row = origin_row + (int) roundf (p -> position.z * cells_per_metre);
        if ((col < 0) || (col >= gw) || (row < 0) || (row >= gh)) {
            outside++;
            continue;
        }
        grid [row][col] = cad_plan_glyph (p -> type);
    }
    int offset = 0;
    offset += snprintf (buffer + offset, (size_t) (cap - offset),
                        "CAD %-63s parts %d/%d mass %.4g kg | %d outside view\n",
                        (design -> name [0]) ? design -> name : "(unnamed)",
                        design -> count, design -> max_parts, mass_total, outside);
    for (int r = 0; r < gh; r++) {
        offset += snprintf (buffer + offset, (size_t) (cap - offset), "%s\n", grid [r]);
    }
    offset += snprintf (buffer + offset, (size_t) (cap - offset),
                        "plan view: 1 cell = 0.25 m, +X right, +Z down "
                        "(top view, heights not shown)\n");
    return cad_ok;
}

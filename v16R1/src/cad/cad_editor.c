/* MIG WP4 4.6: miniature CAD editor — design model. See cad_editor.h
 * for the contract (standalone: nothing in the shipped GUI calls this
 * yet; persistence = the frozen spec grammar; build = the mig_bridge
 * seam; every verb is write-on-success with named errors). */
#include "cad_editor.h"
#include "../core/mig_hull.h"
#include "../core/mig_mass.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ---- forward declarations ------------------------------------------ */

static int cad_validate_part (const mig_part *p);
static float cad_snap_component (float value, float step);
static vector3 cad_snap_position (vector3 position, float step);
static char cad_plan_glyph (mig_part_type type);

/* ---- quaternion / vector helpers ----------------------------------- */

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
 * per-type subsets, every dimension finite and strictly positive.
 * WP7 7.0b: a VOID record validates by its GEOMETRY TWIN's rules —
 * the same dim fields carry the void's size. */
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
        case mig_part_void_sphere: return p -> radius > 0.0f;
        case mig_part_void_cube: return (p -> half_extensions.x > 0.0f) &&
                                        (p -> half_extensions.y > 0.0f) &&
                                        (p -> half_extensions.z > 0.0f);
        case mig_part_void_cylinder: return (p -> radius > 0.0f) &&
                                            (p -> cylinder_half_length > 0.0f);
        default: return false;
    }
}

/* One shared validator for every part-entering verb. Returns the
 * same-named cad_err the compose path would surface later — refusing
 * at the door keeps the design always buildable. WP7 7.0b: a VOID
 * record enters on its twin's dims rules with DENSITY interpretation
 * ONLY (explicit mass on a void is meaningless — cad_bad_mass_mode,
 * the same named rule the signed compose enforces); the density value
 * itself still passes the strictly-positive mass check below. */
static int cad_validate_part (const mig_part *p) {
    if (!p) { return cad_null; }
    if ((p -> type != mig_part_sphere) && (p -> type != mig_part_cube) &&
        (p -> type != mig_part_cylinder) &&
        (p -> type != mig_part_void_sphere) && (p -> type != mig_part_void_cube) &&
        (p -> type != mig_part_void_cylinder)) {
        return cad_bad_type;
    }
    if (!cad_dims_valid (p)) { return cad_bad_dims; }
    if (!cad_finite3 (p -> position)) { return cad_bad_pose; }
    if (!cad_quat_unit (p -> orientation)) { return cad_bad_quat; }
    if ((p -> type >= mig_part_void_sphere) &&
        (p -> mass_mode != mig_mass_mode_density)) {
        return cad_bad_mass_mode;
    }
    if (!isfinite (p -> mass) || (p -> mass <= 0.0f)) { return cad_bad_mass; }
    return cad_ok;
}

/* ---- undo/redo stack -------------------------------------------------
 * WP8: multi-level undo/redo. Every mutating verb pushes a snapshot
 * onto the undo stack (via cad_undo_push_internal). cad_undo_multi
 * restores the top snapshot and pushes the current design onto the
 * redo stack; cad_redo_multi does the reverse. Stack depth is
 * CAD_UNDO_DEPTH; pushing past the depth discards the oldest entry
 * (ring buffer). */

/* WP7 7.0c: single-level undo snapshot (legacy cad_undo contract). */
static cad_design cw_undo_snap;
static int cw_undo_available = 0;

static void cad_undo_snapshot (const cad_design *design) {
    cw_undo_snap = *design;
    cw_undo_available = 1;
}

static cad_design cw_undo_stack [CAD_UNDO_DEPTH];
static int cw_undo_top = -1;      /* -1 = empty */
static cad_design cw_redo_stack [CAD_UNDO_DEPTH];
static int cw_redo_top = -1;      /* -1 = empty */

static void cad_undo_push_internal (const cad_design *design) {
    /* Push current design onto undo stack, discarding oldest if full. */
    if (cw_undo_top >= CAD_UNDO_DEPTH - 1) {
        /* Shift down: discard oldest (index 0). */
        memmove (cw_undo_stack, cw_undo_stack + 1,
                 (size_t) (CAD_UNDO_DEPTH - 1) * sizeof (cad_design));
        cw_undo_top = CAD_UNDO_DEPTH - 1;
    } else {
        cw_undo_top++;
    }
    cw_undo_stack [cw_undo_top] = *design;
    /* A new mutate clears the redo stack. */
    cw_redo_top = -1;
}

/* cad_undo_pop and cad_redo_pop are internal helpers used by
 * cad_undo_multi and cad_redo_multi; declared here for clarity. */

/* ---- group id management --------------------------------------------
 * Group ids are allocated from a small pool; -1 means ungrouped.
 * Group 0 is reserved (never allocated). */

#define CAD_GROUP_MAX 32

static int cw_group_next_id = 1;   /* next id to try */
static int cw_group_in_use [CAD_GROUP_MAX]; /* bitmap: 1 = in use */

static int cad_group_alloc (void) {
    for (int i = 0; i < CAD_GROUP_MAX; i++) {
        int id = (cw_group_next_id + i) % CAD_GROUP_MAX;
        if (id == 0) continue;          /* skip reserved */
        if (!cw_group_in_use [id]) {
            cw_group_in_use [id] = 1;
            cw_group_next_id = (id + 1) % CAD_GROUP_MAX;
            return id;
        }
    }
    return -1; /* no free ids */
}

static void cad_group_free (int id) {
    if (id <= 0 || id >= CAD_GROUP_MAX) return;
    cw_group_in_use [id] = 0;
}

/* ---- part helpers -------------------------------------------------- */

/* Apply `fn(design, index, ctx)` to the selected part and all its group
 * members. Returns the first non-ok error, or cad_ok if all succeeded. */
/* cad_apply_to_group is no longer used; group iteration is inlined
 * in each verb. */

/* ---- clipboard -----------------------------------------------------
 * WP8: copy/paste clipboard. Holds a list of parts copied via cad_copy.
 * cad_paste appends clipboard contents (offset by the given delta) to
 * the design. The clipboard persists until overwritten by a new copy. */

static mig_part cw_clipboard [CAD_MAX_PARTS];
static int cw_clipboard_count = 0;

/* ---- spec grammar name charset -------------------------------------
 * [A-Za-z0-9_-], 1..63 bytes. */
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
    for (int i = 0; i < CAD_MAX_PARTS; i++) {
        design -> group_id [i] = -1;
        design -> parts [i].orientation = vector4_identity ();
        design -> parts [i].mass_mode = mig_mass_mode_density;
        design -> parts [i].mass = CAD_DEFAULT_DENSITY;
        design -> part_color [i] = (vector3) {0.5f, 0.5f, 0.5f};
        design -> part_name [i][0] = '\0';
    }
    cw_undo_top = -1;
    cw_redo_top = -1;
    cw_clipboard_count = 0;
    cw_undo_available = 0;
    for (int i = 0; i < CAD_GROUP_MAX; i++) cw_group_in_use [i] = 0;
    cw_group_next_id = 1;
}

int cad_set_name (cad_design *design, const char *name) {
    if ((!design) || (!name)) { return cad_null; }
    if (!cad_name_legal (name)) { return cad_bad_name; }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
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
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    int i = design -> count;
    design -> parts [i] = *part;
    design -> group_id [i] = -1;
    design -> part_color [i] = (vector3) {0.5f, 0.5f, 0.5f};
    design -> part_name [i][0] = '\0';
    design -> count++;
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

/* WP7 7.0b typed VOID adds — identical authoring shape to the material
 * adds (identity quat, CAD_DEFAULT_DENSITY) but the record is stored
 * VOID-typed and the mass field is its DENSITY in density mode (a void
 * has no explicit-kilogram meaning). The engine's signed compose
 * subtracts the twin-geometry volume at this density. */
int cad_add_void_sphere (cad_design *design, float radius, vector3 position) {
    mig_part p;
    memset (&p, 0, sizeof (p));
    p.type = mig_part_void_sphere;
    p.radius = radius;
    p.position = position;
    p.orientation = vector4_identity ();
    p.mass_mode = mig_mass_mode_density;
    p.mass = CAD_DEFAULT_DENSITY;
    return cad_append_validated (design, &p);
}

int cad_add_void_cube (cad_design *design, vector3 half_extensions,
                       vector3 position) {
    mig_part p;
    memset (&p, 0, sizeof (p));
    p.type = mig_part_void_cube;
    p.half_extensions = half_extensions;
    p.position = position;
    p.orientation = vector4_identity ();
    p.mass_mode = mig_mass_mode_density;
    p.mass = CAD_DEFAULT_DENSITY;
    return cad_append_validated (design, &p);
}

int cad_add_void_cylinder (cad_design *design, float radius, float half_length,
                           vector3 position) {
    mig_part p;
    memset (&p, 0, sizeof (p));
    p.type = mig_part_void_cylinder;
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
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    int deleted_group = design -> group_id [index];
    for (int i = index; i < (design -> count - 1); i++) {
        design -> parts [i] = design -> parts [i + 1];
        design -> group_id [i] = design -> group_id [i + 1];
        design -> part_color [i] = design -> part_color [i + 1];
        memset (design -> part_name [i], 0, sizeof (design -> part_name [i]));
        strcpy (design -> part_name [i], design -> part_name [i + 1]);
    }
    design -> count--;
    /* Clear vacated slot. */
    memset (&design -> parts [design -> count], 0, sizeof (mig_part));
    design -> parts [design -> count].orientation = vector4_identity ();
    design -> parts [design -> count].mass_mode = mig_mass_mode_density;
    design -> parts [design -> count].mass = CAD_DEFAULT_DENSITY;
    design -> group_id [design -> count] = -1;
    design -> part_color [design -> count] = (vector3) {0.5f, 0.5f, 0.5f};
    design -> part_name [design -> count][0] = '\0';
    /* If the deleted part belonged to a group, check if any members
     * remain; if not, free the group id. */
    if (deleted_group >= 0) {
        int still_members = 0;
        for (int i = 0; i < design -> count; i++) {
            if (design -> group_id [i] == deleted_group) { still_members = 1; break; }
        }
        if (!still_members) { cad_group_free (deleted_group); }
    }
    /* Selection follows the part, or drops off the end. */
    if (design -> selected == index) {
        design -> selected = (design -> selected < design -> count)
            ? design -> selected : -1;
    } else if (design -> selected > index) {
        design -> selected--;
    }
    return cad_ok;
}

int cad_duplicate (cad_design *design, int index, vector3 offset) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    mig_part p = design -> parts [index];
    p.position = cad_snap_position (vector3_addition (p.position, offset),
                                    design -> snap_step);
    return cad_append_validated (design, &p);
}

int cad_array (cad_design *design, int index, int n, vector3 step) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    if (n <= 0) { return cad_bad_arg; }
    if (n == 1) { return cad_ok; } /* no-op */
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    /* Check capacity first. */
    if (design -> count + (n - 1) > design -> max_parts) { return cad_full; }
    mig_part original = design -> parts [index];
    for (int k = 1; k < n; k++) {
        mig_part p = original;
        p.position = cad_snap_position (vector3_addition (
            p.position,
            vector3_scaling (step, (float) k)), design -> snap_step);
        int e = cad_append_validated (design, &p);
        if (e != cad_ok) { return e; }
    }
    return cad_ok;
}

/* ---- transforms (positions snap to snap_step multiples when
 * snap_step > 0 — snapping is a mode, not a verb) ---- */

int cad_move (cad_design *design, int index, vector3 delta) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    if (!cad_finite3 (delta)) { return cad_bad_pose; }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    int selected = index;
    int ga = design -> group_id [selected];
    vector3 old_pos = design -> parts [selected].position;
    vector3 new_pos = cad_snap_position (
        vector3_addition (old_pos, delta), design -> snap_step);
    vector3 applied = vector3_subtraction (new_pos, old_pos);
    design -> parts [selected].position = new_pos;
    if (ga >= 0) {
        for (int i = 0; i < design -> count; i++) {
            if (i == selected) continue;
            if (design -> group_id [i] == ga) {
                design -> parts [i].position = cad_snap_position (
                    vector3_addition (design -> parts [i].position, applied),
                    design -> snap_step);
            }
        }
    }
    return cad_ok;
}

int cad_set_position (cad_design *design, int index, vector3 position) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    if (!cad_finite3 (position)) { return cad_bad_pose; }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    int selected = index;
    int ga = design -> group_id [selected];
    vector3 old_pos = design -> parts [selected].position;
    vector3 new_pos = cad_snap_position (position, design -> snap_step);
    vector3 delta = vector3_subtraction (new_pos, old_pos);
    design -> parts [selected].position = new_pos;
    if (ga >= 0) {
        for (int i = 0; i < design -> count; i++) {
            if (i == selected) continue;
            if (design -> group_id [i] == ga) {
                design -> parts [i].position = cad_snap_position (
                    vector3_addition (design -> parts [i].position, delta),
                    design -> snap_step);
            }
        }
    }
    return cad_ok;
}

/* Rotate part `index` about an ASSEMBLY-frame axis (0=x, 1=y, 2=z) by
 * `degrees` (right-hand rule). Composes q_axis * q_part so the axis
 * stays world-anchored no matter the part's current orientation.
 * rot_snap_deg > 0 rounds the request to its multiples FIRST. */
int cad_rotate_degrees (cad_design *design, int index, int axis, float degrees) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    if ((axis < 0) || (axis > 2)) { return cad_bad_arg; }
    if (!isfinite (degrees)) { return cad_bad_arg; }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    float d = degrees;
    if (design -> rot_snap_deg > 0.0f) {
        d = roundf (d / design -> rot_snap_deg) * design -> rot_snap_deg;
    }
    const vector3 axes [3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    vector4 q_axis = vector4_from_axis_with_angle (axes [axis], d * (float) MATH_PI / 180.0f);
    int selected = index;
    int ga = design -> group_id [selected];
    /* Compose on the LEFT: the axis is an ASSEMBLY-frame direction, so
     * the part's current orientation must not rotate it. */
    design -> parts [selected].orientation =
        vector4_multiplication (q_axis, design -> parts [selected].orientation);
    if (ga >= 0) {
        for (int i = 0; i < design -> count; i++) {
            if (i == selected) continue;
            if (design -> group_id [i] == ga) {
                design -> parts [i].orientation =
                    vector4_multiplication (q_axis, design -> parts [i].orientation);
            }
        }
    }
    return cad_ok;
}

/* Rewrite either mass mode on a part. WP7 7.0b: a VOID record refuses
 * the explicit mode (cad_bad_mass_mode — a void has no kilogram
 * meaning); its density stays rewritable. */
int cad_set_mass (cad_design *design, int index, mig_mass_mode mode, float value) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    if ((mode != mig_mass_mode_explicit) && (mode != mig_mass_mode_density)) {
        return cad_bad_arg;
    }
    /* WP7 7.0b: a VOID record is density-only — explicit kilograms on a
     * cavity is meaningless (the signed compose refuses it the same
     * way). The density VALUE stays freely rewritable. */
    if ((design -> parts [index].type >= mig_part_void_sphere) &&
        (mode != mig_mass_mode_density)) {
        return cad_bad_mass_mode;
    }
    if (!isfinite (value) || (value <= 0.0f)) { return cad_bad_mass; }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    int selected = index;
    int ga = design -> group_id [selected];
    design -> parts [selected].mass_mode = mode;
    design -> parts [selected].mass = value;
    if (ga >= 0) {
        for (int i = 0; i < design -> count; i++) {
            if (i == selected) continue;
            if (design -> group_id [i] == ga) {
                design -> parts [i].mass_mode = mode;
                design -> parts [i].mass = value;
            }
        }
    }
    return cad_ok;
}

/* ---- dims resize (the window's scale verb) ----
 * Resize ONE part through the verb matching its type; a mismatch refuses
 * cad_bad_type (a sphere is not resized by the cube verb). WP7 7.0b: a
 * VOID record is resized through the verb matching its GEOMETRY TWIN
 * (void_sphere resizes via the sphere verb, etc.) — the same dim fields
 * carry the void's geometry. Dims must be finite and strictly positive
 * (the mig_compose rule). Write-on-success: a refused resize leaves the
 * part untouched. */

int cad_set_dims_sphere (cad_design *design, int index, float radius) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    const mig_part_type t = design -> parts [index].type;
    if ((t != mig_part_sphere) && (t != mig_part_void_sphere)) { return cad_bad_type; }
    if (!isfinite (radius) || (radius <= 0.0f)) { return cad_bad_dims; }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    int selected = index;
    int ga = design -> group_id [selected];
    design -> parts [selected].radius = radius;
    if (ga >= 0) {
        for (int i = 0; i < design -> count; i++) {
            if (i == selected) continue;
            if (design -> group_id [i] == ga) {
                design -> parts [i].radius = radius;
            }
        }
    }
    return cad_ok;
}

int cad_set_dims_cube (cad_design *design, int index, vector3 half_extensions) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    const mig_part_type t = design -> parts [index].type;
    if ((t != mig_part_cube) && (t != mig_part_void_cube)) { return cad_bad_type; }
    if (!cad_finite3 (half_extensions) || (half_extensions.x <= 0.0f) ||
        (half_extensions.y <= 0.0f) || (half_extensions.z <= 0.0f)) {
        return cad_bad_dims;
    }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    int selected = index;
    int ga = design -> group_id [selected];
    design -> parts [selected].half_extensions = half_extensions;
    if (ga >= 0) {
        for (int i = 0; i < design -> count; i++) {
            if (i == selected) continue;
            if (design -> group_id [i] == ga) {
                design -> parts [i].half_extensions = half_extensions;
            }
        }
    }
    return cad_ok;
}

int cad_set_dims_cylinder (cad_design *design, int index, float radius,
                           float half_length) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    const mig_part_type t = design -> parts [index].type;
    if ((t != mig_part_cylinder) && (t != mig_part_void_cylinder)) { return cad_bad_type; }
    if (!isfinite (radius) || (radius <= 0.0f) || !isfinite (half_length) ||
        (half_length <= 0.0f)) {
        return cad_bad_dims;
    }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    int selected = index;
    int ga = design -> group_id [selected];
    design -> parts [selected].radius = radius;
    design -> parts [selected].cylinder_half_length = half_length;
    if (ga >= 0) {
        for (int i = 0; i < design -> count; i++) {
            if (i == selected) continue;
            if (design -> group_id [i] == ga) {
                design -> parts [i].radius = radius;
                design -> parts [i].cylinder_half_length = half_length;
            }
        }
    }
    return cad_ok;
}

/* ---- WP7 7.0c open voids: destructive subtraction verbs -------------
 * These modify an existing MATERIAL part IN PLACE: the target cube is
 * partitioned into at most 6 slabs around the subtracted void region,
 * replacing the original part with the slabs. A one-level undo snapshot
 * is taken automatically before each subtract (cad_undo restores it).
 * Write-on-success: a refused subtract leaves the design untouched.
 * target_index must index an existing cube part; the void region must
 * lie entirely within the target's bounds; slabs of zero thickness are
 * omitted (so the actual slab count may be < 6). Round targets are
 * refused — subtraction is cube-target-only. */
int cad_subtract_box (cad_design *design, int target_index,
                      vector3 void_center, vector3 void_half_extensions) {
    if (!design) { return cad_null; }
    if ((target_index < 0) || (target_index >= design -> count)) {
        return cad_bad_index;
    }
    mig_part *t = &design -> parts [target_index];
    if (t -> type != mig_part_cube) { return cad_bad_type; }
    if (!cad_finite3 (void_center) || !cad_finite3 (void_half_extensions)) {
        return cad_bad_pose;
    }
    if ((void_half_extensions.x <= 0.0f) || (void_half_extensions.y <= 0.0f) ||
        (void_half_extensions.z <= 0.0f)) {
        return cad_bad_dims;
    }
    /* The void region must sit inside the target cube. */
    if ((void_center.x + void_half_extensions.x > t -> half_extensions.x) ||
        (void_center.x - void_half_extensions.x < -t -> half_extensions.x) ||
        (void_center.y + void_half_extensions.y > t -> half_extensions.y) ||
        (void_center.y - void_half_extensions.y < -t -> half_extensions.y) ||
        (void_center.z + void_half_extensions.z > t -> half_extensions.z) ||
        (void_center.z - void_half_extensions.z < -t -> half_extensions.z)) {
        return cad_bad_dims; /* void region pokes out of the target cube */
    }
    cad_undo_snapshot (design);
    cad_undo_push_internal (design);
    /* Build up to 6 slabs around the void region within the target cube.
     * Slabs are axis-aligned boxes; zero-thickness slabs are omitted so
     * the actual slab count may be < 6 (an exactly-filling void removes
     * the target entirely). */
    mig_part slabs [6];
    int n = 0;
    float x_lo = -t -> half_extensions.x;
    float x_hi = +t -> half_extensions.x;
    float y_lo = -t -> half_extensions.y;
    float y_hi = +t -> half_extensions.y;
    float z_lo = -t -> half_extensions.z;
    float z_hi = +t -> half_extensions.z;
    float vx_lo = void_center.x - void_half_extensions.x;
    float vx_hi = void_center.x + void_half_extensions.x;
    /* Slab 1: x in [x_lo, vx_lo] (full Y, full Z). */
    if (vx_lo > x_lo) {
        mig_part s = *t;
        s.type = mig_part_cube;
        s.half_extensions = (vector3) {0.5f * (vx_lo - x_lo), 0.5f * (y_hi - y_lo),
                                       0.5f * (z_hi - z_lo)};
        s.position = (vector3) {0.5f * (x_lo + vx_lo), t -> position.y, t -> position.z};
        slabs [n++] = s;
    }
    /* Slab 2: x in [vx_hi, x_hi] (full Y, full Z). */
    if (x_hi > vx_hi) {
        mig_part s = *t;
        s.type = mig_part_cube;
        s.half_extensions = (vector3) {0.5f * (x_hi - vx_hi), 0.5f * (y_hi - y_lo),
                                       0.5f * (z_hi - z_lo)};
        s.position = (vector3) {0.5f * (vx_hi + x_hi), t -> position.y, t -> position.z};
        slabs [n++] = s;
    }
    /* Middle X strip [vx_lo, vx_hi], Y slabs outside void.y. */
    float mx_lo = (vx_lo > x_lo) ? vx_lo : x_lo;
    float mx_hi = (vx_hi < x_hi) ? vx_hi : x_hi;
    if (mx_hi > mx_lo) {
        if (void_center.y - void_half_extensions.y > y_lo) {
            mig_part s = *t;
            s.type = mig_part_cube;
            s.half_extensions = (vector3) {0.5f * (mx_hi - mx_lo),
                                           0.5f * (void_center.y - void_half_extensions.y - y_lo),
                                           0.5f * (z_hi - z_lo)};
            s.position = (vector3) {0.5f * (mx_lo + mx_hi),
                                    0.5f * (y_lo + void_center.y - void_half_extensions.y),
                                    t -> position.z};
            slabs [n++] = s;
        }
        if (y_hi > void_center.y + void_half_extensions.y) {
            mig_part s = *t;
            s.type = mig_part_cube;
            s.half_extensions = (vector3) {0.5f * (mx_hi - mx_lo),
                                           0.5f * (y_hi - void_center.y - void_half_extensions.y),
                                           0.5f * (z_hi - z_lo)};
            s.position = (vector3) {0.5f * (mx_lo + mx_hi),
                                    0.5f * (void_center.y + void_half_extensions.y + y_hi),
                                    t -> position.z};
            slabs [n++] = s;
        }
        /* Middle Y strip, Z slabs outside void.z. */
        float my_lo = (void_center.y - void_half_extensions.y > y_lo) ?
            (void_center.y - void_half_extensions.y) : y_lo;
        float my_hi = (y_hi > void_center.y + void_half_extensions.y) ?
            (void_center.y + void_half_extensions.y) : y_hi;
        if (my_hi > my_lo) {
            if (void_center.z - void_half_extensions.z > z_lo) {
                mig_part s = *t;
                s.type = mig_part_cube;
                s.half_extensions = (vector3) {0.5f * (mx_hi - mx_lo),
                                               0.5f * (my_hi - my_lo),
                                               0.5f * (void_center.z - void_half_extensions.z - z_lo)};
                s.position = (vector3) {0.5f * (mx_lo + mx_hi),
                                        0.5f * (my_lo + my_hi),
                                        0.5f * (z_lo + void_center.z - void_half_extensions.z)};
                slabs [n++] = s;
            }
            if (z_hi > void_center.z + void_half_extensions.z) {
                mig_part s = *t;
                s.type = mig_part_cube;
                s.half_extensions = (vector3) {0.5f * (mx_hi - mx_lo),
                                               0.5f * (my_hi - my_lo),
                                               0.5f * (z_hi - void_center.z - void_half_extensions.z)};
                s.position = (vector3) {0.5f * (mx_lo + mx_hi),
                                        0.5f * (my_lo + my_hi),
                                        0.5f * (void_center.z + void_half_extensions.z + z_hi)};
                slabs [n++] = s;
            }
        }
    }
    /* Replace the target part with the slabs. If n == 0 the void exactly
     * fills the target: remove it (count-1). */
    if (n == 0) {
        for (int i = target_index; i < (design -> count - 1); i++) {
            design -> parts [i] = design -> parts [i + 1];
            design -> group_id [i] = design -> group_id [i + 1];
            design -> part_color [i] = design -> part_color [i + 1];
            memset (design -> part_name [i], 0, sizeof (design -> part_name [i]));
            strcpy (design -> part_name [i], design -> part_name [i + 1]);
        }
        design -> count--;
        if (design -> count > 0 && target_index <= design -> count - 1) {
            design -> group_id [design -> count - 1] = -1;
            design -> part_color [design -> count - 1] = (vector3) {0.5f, 0.5f, 0.5f};
            design -> part_name [design -> count - 1][0] = '\0';
        }
        int deleted_group = design -> group_id [target_index];
        if (deleted_group >= 0) {
            int still_members = 0;
            for (int i = 0; i < design -> count; i++) {
                if (design -> group_id [i] == deleted_group) { still_members = 1; break; }
            }
            if (!still_members) { cad_group_free (deleted_group); }
        }
        if (design -> selected >= design -> count) {
            design -> selected = design -> count - 1;
        }
        return cad_ok;
    }
    /* Write slabs back starting at target_index, shifting subsequent
     * parts down to make room (slabs replace one part with n parts). */
    if (n > 1) {
        if (design -> count + (n - 1) > design -> max_parts) { return cad_full; }
        for (int i = design -> count - 1; i > target_index; i--) {
            design -> parts [i + n - 1] = design -> parts [i];
            design -> group_id [i + n - 1] = design -> group_id [i];
            design -> part_color [i + n - 1] = design -> part_color [i];
            memset (design -> part_name [i + n - 1], 0, sizeof (design -> part_name [0]));
            strcpy (design -> part_name [i + n - 1], design -> part_name [i]);
        }
    }
    for (int i = 0; i < n; i++) {
        design -> parts [target_index + i] = slabs [i];
        design -> group_id [target_index + i] = -1; /* slabs are ungrouped */
        design -> part_color [target_index + i] = (vector3) {0.5f, 0.5f, 0.5f};
        design -> part_name [target_index + i][0] = '\0';
    }
    design -> count += n - 1;
    design -> selected = target_index;
    return cad_ok;
}

int cad_subtract_sphere (cad_design *design, int target_index,
                         vector3 void_center, float void_radius) {
    /* Round void on a cube target: approximate by an n-gonal ring of slabs
     * (disclosed inscribed-envelope — removes less than authored, never
     * more). Each segment of the ring becomes one box slab; the union of
     * slabs lies entirely inside the sphere, so the subtraction removes
     * exactly the inscribed ring volume, never more than authored.
     * Uses 6 segments (n_segs=6, exact sqrt3 literals). */
    if (!design) { return cad_null; }
    if (!isfinite (void_center.x) || !isfinite (void_center.y) ||
        !isfinite (void_center.z) || !isfinite (void_radius) ||
        (void_radius <= 0.0f)) {
        return cad_bad_dims;
    }
    /* Validate target before building slabs. */
    if ((target_index < 0) || (target_index >= design -> count)) {
        return cad_bad_index;
    }
    mig_part *t = &design -> parts [target_index];
    if (t -> type != mig_part_cube) { return cad_bad_type; }

    cad_undo_snapshot (design);
    cad_undo_push_internal (design);
    const int n_segs = 6;

    mig_part slabs [6];
    int n = 0;
    for (int i = 0; i < n_segs; i++) {
        double ci, si;
        static const double c6 [6][2] = {
            { 1.0,                0.0               },
            { MIG_HULL_SQRT3 / 2,  0.5              },
            { 0.5,                MIG_HULL_SQRT3 / 2},
            {-0.5,                MIG_HULL_SQRT3 / 2},
            {- MIG_HULL_SQRT3 / 2,  0.5              },
            {-0.5,               -MIG_HULL_SQRT3 / 2}
        };
        ci = c6 [i][0]; si = c6 [i][1];

        /* Slab centre: at void_radius/2 along the segment's radial axis.
         * Half-extent along each axis: void_radius/2 * max(|ci|,|si|)
         * ensures no degenerate (zero-thickness) slabs for axis-aligned
         * segments. Full height covers the entire sphere. */
        const float r_h = void_radius / 2.0f * (float)fmax(fabs(ci), fabs(si));

        mig_part s = *t;
        s.type = mig_part_cube;
        s.half_extensions = (vector3) {
            r_h, r_h, void_radius
        };
        s.position = (vector3) {
            (float)(void_radius / 2.0 * ci) + t -> position.x,
            (float)(void_radius / 2.0 * si) + t -> position.y,
            t -> position.z
        };
        slabs [n++] = s;
    }

    /* Write slabs back, replacing the target. */
    if (n == 0) {
        for (int i = target_index; i < (design -> count - 1); i++) {
            design -> parts [i] = design -> parts [i + 1];
            design -> group_id [i] = design -> group_id [i + 1];
            design -> part_color [i] = design -> part_color [i + 1];
            memset (design -> part_name [i], 0, sizeof (design -> part_name [i]));
            strcpy (design -> part_name [i], design -> part_name [i + 1]);
        }
        design -> count--;
        if (design -> count > 0 && target_index <= design -> count - 1) {
            design -> group_id [design -> count - 1] = -1;
            design -> part_color [design -> count - 1] = (vector3) {0.5f, 0.5f, 0.5f};
            design -> part_name [design -> count - 1][0] = '\0';
        }
        int deleted_group = design -> group_id [target_index];
        if (deleted_group >= 0) {
            int still_members = 0;
            for (int i = 0; i < design -> count; i++) {
                if (design -> group_id [i] == deleted_group) { still_members = 1; break; }
            }
            if (!still_members) { cad_group_free (deleted_group); }
        }
        if (design -> selected >= design -> count) {
            design -> selected = design -> count - 1;
        }
        return cad_ok;
    }
    if (n > 1) {
        if (design -> count + (n - 1) > design -> max_parts) { return cad_full; }
        for (int i = design -> count - 1; i > target_index; i--) {
            design -> parts [i + n - 1] = design -> parts [i];
            design -> group_id [i + n - 1] = design -> group_id [i];
            design -> part_color [i + n - 1] = design -> part_color [i];
            memset (design -> part_name [i + n - 1], 0, sizeof (design -> part_name [0]));
            strcpy (design -> part_name [i + n - 1], design -> part_name [i]);
        }
    }
    for (int i = 0; i < n; i++) {
        design -> parts [target_index + i] = slabs [i];
        design -> group_id [target_index + i] = -1; /* slabs are ungrouped */
        design -> part_color [target_index + i] = (vector3) {0.5f, 0.5f, 0.5f};
        design -> part_name [target_index + i][0] = '\0';
    }
    design -> count += n - 1;
    design -> selected = target_index;
    return cad_ok;
}

/* ---- WP8: multi-level undo/redo ------------------------------------ */

/* Legacy single-step undo (WP7 7.0c contract): restore the most recent
 * snapshot. Returns cad_bad_arg if nothing to undo. Only one level. */
int cad_undo (cad_design *design) {
    if (!design) { return cad_null; }
    if (!cw_undo_available) { return cad_bad_arg; }
    *design = cw_undo_snap;
    cw_undo_available = 0;
    return cad_ok;
}

/* Push the current design onto the undo stack. Called automatically by
 * every mutating verb. Returns cad_ok on success. */
int cad_undo_push (cad_design *design) {
    if (!design) { return cad_null; }
    cad_undo_push_internal (design);
    return cad_ok;
}

/* Restore the most recent undo snapshot. Returns cad_ok on success,
 * cad_undo_empty if the undo stack is empty. The current design is
 * pushed onto the redo stack. */
int cad_undo_multi (cad_design *design) {
    if (!design) { return cad_null; }
    if (cw_undo_top < 0) { return cad_undo_empty; }
    /* Push current design onto redo stack. */
    if (cw_redo_top >= CAD_UNDO_DEPTH - 1) {
        memmove (cw_redo_stack, cw_redo_stack + 1,
                 (size_t) (CAD_UNDO_DEPTH - 1) * sizeof (cad_design));
        cw_redo_top = CAD_UNDO_DEPTH - 1;
    } else {
        cw_redo_top++;
    }
    cw_redo_stack [cw_redo_top] = *design;
    /* Pop undo snapshot into design. */
    *design = cw_undo_stack [cw_undo_top];
    cw_undo_top--;
    return cad_ok;
}

int cad_redo_multi (cad_design *design) {
    if (!design) { return cad_null; }
    if (cw_redo_top < 0) { return cad_redo_empty; }
    /* Push current design onto undo stack. */
    if (cw_undo_top >= CAD_UNDO_DEPTH - 1) {
        memmove (cw_undo_stack, cw_undo_stack + 1,
                 (size_t) (CAD_UNDO_DEPTH - 1) * sizeof (cad_design));
        cw_undo_top = CAD_UNDO_DEPTH - 1;
    } else {
        cw_undo_top++;
    }
    cw_undo_stack [cw_undo_top] = *design;
    /* Pop redo snapshot into design. */
    *design = cw_redo_stack [cw_redo_top];
    cw_redo_top--;
    return cad_ok;
}

/* ---- WP8: selection groups -----------------------------------------
 * Group operations let the shell act on multiple parts at once.
 * cad_group assigns the selected part (via design->selected — only one
 * selection at a time in the model; group membership is per-part) to a
 * new group id and returns that id. cad_ungroup clears the group id
 * from the selected part's group members. cad_group_members returns the
 * count of parts sharing the same group as part `index`.
 * Group id 0 is reserved (ungrouped = -1). Group ids are recycled. */

int cad_group (cad_design *design) {
    if (!design) { return cad_null; }
    if ((design -> selected < 0) || (design -> selected >= design -> count)) {
        return cad_bad_index;
    }
    int selected = design -> selected;
    int gid = cad_group_alloc ();
    if (gid < 0) { return cad_group_full; }
    /* Assign this group id to the selected part and all its current
     * group members (if any). */
    int old_gid = design -> group_id [selected];
    if (old_gid >= 0) {
        /* Already in a group — just keep the existing id. */
        return old_gid;
    }
    /* Assign new group id to selected part. */
    design -> group_id [selected] = gid;
    /* No other members since selected was ungrouped. */
    return gid;
}

int cad_ungroup (cad_design *design) {
    if (!design) { return cad_null; }
    if ((design -> selected < 0) || (design -> selected >= design -> count)) {
        return cad_bad_index;
    }
    int selected = design -> selected;
    int gid = design -> group_id [selected];
    if (gid < 0) { return cad_ok; /* already ungrouped */ }
    /* Clear group id from all members. */
    for (int i = 0; i < design -> count; i++) {
        if (design -> group_id [i] == gid) {
            design -> group_id [i] = -1;
        }
    }
    cad_group_free (gid);
    return cad_ok;
}

int cad_group_members (const cad_design *design, int index) {
    if (!design) { return 0; }
    if ((index < 0) || (index >= design -> count)) { return 0; }
    int gid = design -> group_id [index];
    if (gid < 0) { return 0; }
    int count = 0;
    for (int i = 0; i < design -> count; i++) {
        if (design -> group_id [i] == gid) { count++; }
    }
    return count;
}

/* ---- WP8: mirror ---------------------------------------------------
 * Mirror selected parts across a cardinal plane through the origin:
 * plane 0 = YZ (negate X), plane 1 = XZ (negate Y), plane 2 = XY
 * (negate Z). Applies to the selected part and all its group members.
 * Returns cad_ok if at least one part was mirrored, cad_bad_arg for
 * invalid plane, cad_bad_index if nothing selected. */

int cad_mirror (cad_design *design, int plane) {
    if (!design) { return cad_null; }
    if ((design -> selected < 0) || (design -> selected >= design -> count)) {
        return cad_bad_index;
    }
    if (plane < 0 || plane > 2) { return cad_bad_arg; }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    int selected = design -> selected;
    int ga = design -> group_id [selected];
    int mirrored = 0;
    /* Mirror selected part. */
    design -> parts [selected].position =
        (vector3) {
            plane == 0 ? -design -> parts [selected].position.x : design -> parts [selected].position.x,
            plane == 1 ? -design -> parts [selected].position.y : design -> parts [selected].position.y,
            plane == 2 ? -design -> parts [selected].position.z : design -> parts [selected].position.z
        };
    mirrored = 1;
    /* Mirror group members. */
    if (ga >= 0) {
        for (int i = 0; i < design -> count; i++) {
            if (i == selected) continue;
            if (design -> group_id [i] == ga) {
                design -> parts [i].position =
                    (vector3) {
                        plane == 0 ? -design -> parts [i].position.x : design -> parts [i].position.x,
                        plane == 1 ? -design -> parts [i].position.y : design -> parts [i].position.y,
                        plane == 2 ? -design -> parts [i].position.z : design -> parts [i].position.z
                    };
                mirrored = 1;
            }
        }
    }
    return mirrored ? cad_ok : cad_bad_index;
}

/* ---- WP8: alignment -------------------------------------------------
 * Align selected parts along axis (0=x,1=y,2=z) to mode:
 *   0 = min (all parts' min face aligns to the global min)
 *   1 = center (all parts' center aligns to the mean center)
 *   2 = max (all parts' max face aligns to the global max)
 * Distribute spaces parts evenly between the global min and max
 * along axis. Both operate on the selected part and its group
 * members. Returns cad_ok on success. */

static int cad_align_apply (cad_design *design, int axis, int mode) {
    if ((design -> selected < 0) || (design -> selected >= design -> count)) {
        return cad_bad_index;
    }
    int selected = design -> selected;
    int ga = design -> group_id [selected];

    /* Collect all affected parts (selected + group members). */
    int members [CAD_MAX_PARTS];
    int mcount = 0;
    members [mcount++] = selected;
    if (ga >= 0) {
        for (int i = 0; i < design -> count; i++) {
            if (i == selected) continue;
            if (design -> group_id [i] == ga) {
                members [mcount++] = i;
            }
        }
    }

    /* Compute global min/max/center along axis. */
    float min_val = 0.0f, max_val = 0.0f, center_sum = 0.0f;
    for (int k = 0; k < mcount; k++) {
        float p = 0.0f;
        switch (axis) {
            case 0: p = design -> parts [members [k]].position.x; break;
            case 1: p = design -> parts [members [k]].position.y; break;
            case 2: p = design -> parts [members [k]].position.z; break;
        }
        if (k == 0) { min_val = p; max_val = p; }
        if (p < min_val) min_val = p;
        if (p > max_val) max_val = p;
        center_sum += p;
    }
    float mean_center = center_sum / (float) mcount;

    /* Compute target value per mode. */
    float target = 0.0f;
    if (mode == 0) target = min_val;       /* align min faces */
    else if (mode == 1) target = mean_center; /* align centers */
    else target = max_val;              /* align max faces */

    /* Apply offset to each part. */
    for (int k = 0; k < mcount; k++) {
        int idx = members [k];
        if (axis == 0) design -> parts [idx].position.x = target;
        else if (axis == 1) design -> parts [idx].position.y = target;
        else design -> parts [idx].position.z = target;
    }
    return cad_ok;
}

int cad_align (cad_design *design, int axis, int mode) {
    if (!design) { return cad_null; }
    if ((design -> selected < 0) || (design -> selected >= design -> count)) {
        return cad_bad_index;
    }
    if (axis < 0 || axis > 2) { return cad_bad_arg; }
    if (mode < 0 || mode > 2) { return cad_bad_arg; }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    return cad_align_apply (design, axis, mode);
}

int cad_distribute (cad_design *design, int axis) {
    if (!design) { return cad_null; }
    if ((design -> selected < 0) || (design -> selected >= design -> count)) {
        return cad_bad_index;
    }
    if (axis < 0 || axis > 2) { return cad_bad_arg; }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    int selected = design -> selected;
    int ga = design -> group_id [selected];

    /* Collect all affected parts. */
    int members [CAD_MAX_PARTS];
    int mcount = 0;
    members [mcount++] = selected;
    if (ga >= 0) {
        for (int i = 0; i < design -> count; i++) {
            if (i == selected) continue;
            if (design -> group_id [i] == ga) {
                members [mcount++] = i;
            }
        }
    }
    if (mcount < 2) { return cad_ok; } /* nothing to distribute */

    /* Sort members by position along axis (simple insertion sort). */
    for (int i = 1; i < mcount; i++) {
        int key = members [i];
        float key_pos = 0.0f;
        switch (axis) {
            case 0: key_pos = design -> parts [key].position.x; break;
            case 1: key_pos = design -> parts [key].position.y; break;
            case 2: key_pos = design -> parts [key].position.z; break;
        }
        int j = i - 1;
        float jpos = 0.0f;
        switch (axis) {
            case 0: jpos = design -> parts [members [j]].position.x; break;
            case 1: jpos = design -> parts [members [j]].position.y; break;
            case 2: jpos = design -> parts [members [j]].position.z; break;
        }
        while (j >= 0 && jpos > key_pos) {
            members [j + 1] = members [j];
            j--;
            if (j >= 0) {
                switch (axis) {
                    case 0: jpos = design -> parts [members [j]].position.x; break;
                    case 1: jpos = design -> parts [members [j]].position.y; break;
                    case 2: jpos = design -> parts [members [j]].position.z; break;
                }
            }
        }
        members [j + 1] = key;
    }

    float first_pos = 0.0f, last_pos = 0.0f;
    switch (axis) {
        case 0: first_pos = design -> parts [members [0]].position.x; break;
        case 1: first_pos = design -> parts [members [0]].position.y; break;
        case 2: first_pos = design -> parts [members [0]].position.z; break;
    }
    switch (axis) {
        case 0: last_pos = design -> parts [members [mcount - 1]].position.x; break;
        case 1: last_pos = design -> parts [members [mcount - 1]].position.y; break;
        case 2: last_pos = design -> parts [members [mcount - 1]].position.z; break;
    }
    float span = last_pos - first_pos;
    if (span <= 0.0f) { return cad_ok; } /* all at same position */

    float step = span / (float) (mcount - 1);
    float target = first_pos;
    for (int k = 0; k < mcount; k++) {
        int idx = members [k];
        switch (axis) {
            case 0: design -> parts [idx].position.x = target; break;
            case 1: design -> parts [idx].position.y = target; break;
            case 2: design -> parts [idx].position.z = target; break;
        }
        target += step;
    }
    return cad_ok;
}

/* ---- WP8: copy/paste -----------------------------------------------
 * cad_copy copies the selected part (and its group members) into the
 * module's clipboard. cad_paste appends clipboard contents to the
 * design, offset by `offset` (snapped). The clipboard persists across
 * calls until overwritten by a new copy. */

int cad_copy (cad_design *design) {
    if (!design) { return cad_null; }
    if ((design -> selected < 0) || (design -> selected >= design -> count)) {
        return cad_bad_index;
    }
    int selected = design -> selected;
    int ga = design -> group_id [selected];

    int n = 0;
    /* Copy selected part. */
    cw_clipboard [n++] = design -> parts [selected];
    /* Copy group members. */
    if (ga >= 0) {
        for (int i = 0; i < design -> count; i++) {
            if (i == selected) continue;
            if (design -> group_id [i] == ga) {
                cw_clipboard [n++] = design -> parts [i];
            }
        }
    }
    cw_clipboard_count = n;
    return cad_ok;
}

int cad_paste (cad_design *design, vector3 offset) {
    if (!design) { return cad_null; }
    if (cw_clipboard_count <= 0) { return cad_clipboard_empty; }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    /* Check capacity. */
    if (design -> count + cw_clipboard_count > design -> max_parts) {
        return cad_full;
    }
    for (int i = 0; i < cw_clipboard_count; i++) {
        mig_part p = cw_clipboard [i];
        p.position = vector3_addition (p.position, offset);
        p.orientation = vector4_identity (); /* pasted parts are unrotated */
        memset (design -> part_name [design -> count], 0, sizeof (design -> part_name [0]));
        design -> part_color [design -> count] = (vector3) {0.5f, 0.5f, 0.5f};
        int e = cad_append_validated (design, &p);
        if (e != cad_ok) { return e; }
    }
    return cad_ok;
}

/* ---- WP8: part properties ------------------------------------------
 * Per-part color (authoring only; not persisted to spec grammar —
 * engine stamps body-level materials at build). Default is (0.5,0.5,0.5).
 * Per-part name tag (empty = unnamed). Both persist through the spec
 * grammar as part-level fields (color as `color r g b`, name as
 * `name <text>` — v2 grammar extension). */

int cad_set_part_color (cad_design *design, int index, vector3 color) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    if (!isfinite (color.x) || !isfinite (color.y) || !isfinite (color.z)) {
        return cad_bad_pose;
    }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    int selected = index;
    int ga = design -> group_id [selected];
    design -> part_color [selected] = color;
    if (ga >= 0) {
        for (int i = 0; i < design -> count; i++) {
            if (i == selected) continue;
            if (design -> group_id [i] == ga) {
                design -> part_color [i] = color;
            }
        }
    }
    return cad_ok;
}

int cad_get_part_color (const cad_design *design, int index, vector3 *color) {
    if (!design || !color) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    *color = design -> part_color [index];
    return cad_ok;
}

int cad_set_part_name (cad_design *design, int index, const char *name) {
    if (!design) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    if (!name) { return cad_null; }
    if (!cad_name_legal (name)) { return cad_bad_name; }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    int selected = index;
    int ga = design -> group_id [selected];
    memset (design -> part_name [selected], 0, sizeof (design -> part_name [selected]));
    strcpy (design -> part_name [selected], name);
    if (ga >= 0) {
        for (int i = 0; i < design -> count; i++) {
            if (i == selected) continue;
            if (design -> group_id [i] == ga) {
                /* Group members share nothing here — each part has its
                 * own name. Only the selected part gets renamed. */
            }
        }
    }
    return cad_ok;
}

int cad_get_part_name (const cad_design *design, int index, char *buf, int cap) {
    if (!design || !buf) { return cad_null; }
    if ((index < 0) || (index >= design -> count)) { return cad_bad_index; }
    if (cap <= 0) { return cad_bad_arg; }
    size_t len = strlen (design -> part_name [index]);
    if ((size_t) cap <= len) { return cad_bad_arg; }
    strcpy (buf, design -> part_name [index]);
    return cad_ok;
}

/* ---- WP8: measurement ----------------------------------------------
 * Returns the distance between the centers of parts `a` and `b`.
 * Returns -1.0f on bad index. */

float cad_measure_distance (const cad_design *design, int a, int b) {
    if (!design) { return -1.0f; }
    if ((a < 0) || (a >= design -> count)) { return -1.0f; }
    if ((b < 0) || (b >= design -> count)) { return -1.0f; }
    vector3 da = design -> parts [a].position;
    vector3 db = design -> parts [b].position;
    vector3 diff = vector3_subtraction (da, db);
    return (float) sqrt (diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
}

/* ---- snap / select / persist --------------------------------------- */

int cad_snap_all (cad_design *design) {
    if (!design) { return cad_null; }
    for (int i = 0; i < design -> count; i++) {
        design -> parts [i].position =
            cad_snap_position (design -> parts [i].position, design -> snap_step);
    }
    return cad_ok;
}

int cad_set_snap (cad_design *design, float snap_step) {
    if (!design) { return cad_null; }
    if (!isfinite (snap_step) || (snap_step < 0.0f)) { return cad_bad_arg; }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    design -> snap_step = snap_step;
    return cad_ok;
}

int cad_set_rot_snap (cad_design *design, float rot_snap_deg) {
    if (!design) { return cad_null; }
    if (!isfinite (rot_snap_deg) || (rot_snap_deg < 0.0f)) { return cad_bad_arg; }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    design -> rot_snap_deg = rot_snap_deg;
    return cad_ok;
}

int cad_select (cad_design *design, int index) {
    if (!design) { return cad_null; }
    if ((index != -1) && ((index < 0) || (index >= design -> count))) {
        return cad_bad_index;
    }
    cad_undo_push_internal (design); /* WP8: snapshot before mutation */
    design -> selected = index;
    return cad_ok;
}

/* ---- persistence: the frozen spec grammar is the file format ---- */

/* Design → mig_spec (name, `frame assembly`, per-part type/dims/pose/
 * mass; no material/colour — the engine stamps body defaults at build).
 * Refuses an empty design (the grammar rejects zero-part files) and a
 * name outside the grammar. WP7 7.0d: void-bearing designs CAN be
 * persisted — the v2 spec grammar expresses void records and the scene
 * v202 format carries the per-part is_void flag. Write-on-success. */
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

/* Parsed spec → design: validates the part family/dims/quat/mass the
 * parser's grammar guarantees plus the max_parts cap (write-on-success
 * into `out`; `out` may alias nothing — the caller passes a fresh
 * design). Snap/selected state resets to init values (file has none). */
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

/* Save = to_spec + mig_spec_emit_normalized + write. The file is the
 * canonical normalized text (byte-identical with what `mig save` and
 * the 4.4 panel produce for the same spec). */
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

/* Load = mig_spec_parse_file + design_from_spec. Write-on-success at
 * the design level: a rejected file leaves `design` untouched and
 * returns cad_spec_reject with the raw mig_spec_err (when the caller
 * passes spec_err_out). */
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

/* mig_make_bridge over the design — WP7 7.0b: the routing is
 * void-aware. A MATERIAL-ONLY design keeps the untouched
 * mig_make_bridge path (bitwise); a VOID-BEARING design routes through
 * mig_void_make_bridge (the tested signed compose, same rebase rule).
 * WP4 4.6e hookup semantics: the CAD window validates through this
 * call, then hands the RAW assembly-frame parts to
 * physics_world_add_composite — the engine composes and re-anchors
 * internally and spawns the body at the parts' COM, so the authored
 * placement survives (the CAD frame is the world frame; handing the
 * pre-rebased copy would subtract the COM twice and drop the body at
 * the origin). The rebased copy still serves callers that want
 * COM-relative geometry. `out_parts` must hold count slots and must
 * NOT alias design->parts. Empty design refuses with cad_empty.
 * Compose errors map to the same-named cad_err values verbatim, plus
 * the signed-compose cases: non-positive totals → cad_nonpositive,
 * not-positive-definite → cad_not_pd, explicit-on-void →
 * cad_bad_mass_mode. */
int cad_build (const cad_design *design, mig_part *out_parts, mig_bridge *out) {
    if ((!design) || (!out_parts) || (!out)) { return cad_null; }
    if (design -> count <= 0) { return cad_empty; }
    /* WP7 7.0b: the routing is void-aware. A material-only design keeps
     * the untouched mig_make_bridge path (bitwise — the pinned parity);
     * a void-bearing design routes through the tested signed module,
     * which partitions by type and composes through mig_void_compose
     * with the SAME rebase rule (mig_rebase_parts, one code path). */
    bool has_void = false;
    for (int i = 0; i < design -> count; i++) {
        if (design -> parts [i].type >= mig_part_void_sphere) { has_void = true; break; }
    }
    int rc;
    if (has_void) {
        rc = mig_void_make_bridge (design -> parts, design -> count, out_parts, out);
    } else {
        rc = mig_make_bridge (design -> parts, design -> count, out_parts, out);
    }
    switch (rc) {
        /* The two compose enums are the same numbered family: 0-6 carry
         * the same values under both name sets (the parity contract),
         * 7-10 are mig_void-only. Cases below use the mig_compose_* names
         * for the shared values and the mig_void_* names for the
         * void-only ones — one switch table, no duplicate case values. */
        case mig_compose_ok: return cad_ok;
        case mig_compose_null: return cad_null;
        case mig_compose_bad_type: return cad_bad_type;
        case mig_compose_bad_dims: return cad_bad_dims;
        case mig_compose_bad_quat: return cad_bad_quat;
        case mig_compose_bad_mass: return cad_bad_mass;
        case mig_compose_bad_mass_mode: return cad_bad_mass_mode;
        /* WP7 7.0b: the signed compose's own cases map to the new
         * same-named cad_err values. */
        case mig_void_bad_density: return cad_bad_mass;   /* non-positive density */
        case mig_void_no_material: return cad_empty;      /* a void alone is no body */
        case mig_void_nonpositive: return cad_nonpositive;/* the voids ate the body */
        case mig_void_not_pd: return cad_not_pd;          /* mis-placed void */
        default: return cad_bad_arg; /* unexpected compose code — refuse loudly */
    }
}


/* ---- plan view preview --------------------------------------------- */

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

/* ---- plan view glyphs ---------------------------------------------- */

static char cad_plan_glyph (mig_part_type type) {
    switch (type) {
        case mig_part_sphere: return 'o';
        case mig_part_cube: return '#';
        case mig_part_cylinder: return '|';
        /* WP7 7.0b adds the VOID glyphs 'O' (void sphere), '%' (void cube), '!'
         * (void cylinder) at the rounded cell of each part position (last
         * part wins a shared cell). */
        case mig_part_void_sphere: return 'O';
        case mig_part_void_cube: return '%';
        case mig_part_void_cylinder: return '!';
        default: return '?';
    }
}

/* ---- snap position helper ------------------------------------------
 * Snaps a position component to the nearest multiple of step. If step
 * <= 0, returns the value unchanged. */

static float cad_snap_component (float value, float step) {
    if (!(step > 0.0f)) { return value; }
    return roundf (value / step) * step;
}

static vector3 cad_snap_position (vector3 position, float step) {
    return (vector3) {cad_snap_component (position.x, step),
                      cad_snap_component (position.y, step),
                      cad_snap_component (position.z, step)};
}
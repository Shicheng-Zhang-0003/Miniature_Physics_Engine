/* MIG WP4 4.6: miniature CAD editor — the DESIGN MODEL, standalone.
 *
 * "Standalone" is a hard property of this brick: this TU compiles into
 * the engine and suite builds but NOTHING in the shipped GUI or input
 * dispatch calls it (zero behavior change, verified by the full battery
 * being green). The terminal-style shell and its 1-key-terminal-style
 * screen are the NEXT brick; this file is the headless truth they will
 * both drive, exactly like WP4's headless-before-GTK discipline.
 *
 * A cad_design is one composite BODY under construction in the ASSEMBLY
 * frame: an ordered list of primitive parts (mig_part — the exact WP0
 * record the engine's composite path consumes) whose poses may place
 * them anywhere, INCLUDING with deliberate gaps between parts — the
 * "spaces" the weld-flow composites cannot express. One design = one
 * object, matching the frozen spec grammar: one file, one object.
 *
 * The module owns nothing but its caller-owned design struct: no
 * malloc, no engine calls, no world writes, no config reads (defaults
 * live in this header, not in g_cfg — the model stays decoupled).
 *
 * Persistence IS the frozen spec grammar (core/mig_spec.h): save emits
 * the canonical normalized text, load re-parses it — a saved design is
 * a legal spec file and any legal spec file is loadable, so CAD designs
 * stay compatible with the terminal `mig` store, microvim, and the 4.4
 * GTK panel forever. The frozen MIG_SPEC.md v1 rules govern: sphere/
 * cube/cylinder only, per-type dims subsets, assembly frame, mass XOR
 * density.
 *
 * The build seam: cad_build() runs mig_make_bridge over the design
 * (compose truth + COM-rebased part copy) and hands the bridge to the
 * caller — the LATER hookup calls physics_world_add_composite(world,
 * bridge-parts, count) with exactly what this returns. This TU never
 * includes physics_world.h: the editor model cannot mutate a world BY
 * DESIGN, which is what makes it gateable headless and safe to grow.
 *
 * Determinism: dims/positions are float (the mig_part boundary);
 * rotations compose quaternions via vector4_multiplication (mul/add
 * only); axis-angle→quaternion is vector4_from_axis_with_angle (libm
 * trig — the documented cold-path authoring helper); snap uses roundf
 * (exact). Nothing here feeds the per-tick solver, so the engine-
 * coordinate trig ban is untouched. The suite gate pins every contract
 * below, including the round-trip bitwise rule (float→spec double→
 * shortest-round-trip text→double→float is lossless).
 */
#ifndef cad_editor_h
#define cad_editor_h
#include "../core/math3d.h"
#include "../core/mig_mass.h"
#include "../core/mig_spec.h"

/* The design part ceiling. The spec parser caps at this same literal
 * (the engine's composite.max_parts default); cad_load passes the
 * caller's max_parts through, so a world configured for a smaller cap
 * can refuse oversized designs at load time, not at build time. */
#define CAD_MAX_PARTS MIG_SPEC_MAX_PARTS

/* Typed adds (sphere/cube/cylinder) author parts at water density;
 * cad_set_mass rewrites either mode. The model never reads g_cfg. */
#define CAD_DEFAULT_DENSITY 1000.0f

/* cad_preview buffer contract: refuses a cap below this (a full
 * 64x24 plan view plus header/list lines). */
#define CAD_PREVIEW_MIN_CAP 4096

typedef enum {
    cad_ok = 0,
    cad_null = 1,        /* NULL design / part / buffer */
    cad_full = 2,        /* design at max_parts */
    cad_bad_index = 3,   /* index outside [0, count) */
    cad_bad_type = 4,    /* part type outside the WP0 primitive family */
    cad_bad_dims = 5,    /* non-finite or non-positive dims for the type */
    cad_bad_pose = 6,    /* non-finite position component */
    cad_bad_quat = 7,    /* non-finite or non-unit (|n2-1| > 1e-4) orientation */
    cad_bad_mass = 8,    /* non-finite or <= 0 mass/density */
    cad_bad_name = 9,    /* design name outside the spec grammar's charset/length */
    cad_bad_arg = 10,    /* axis outside 0..2, n <= 0 array, non-finite degrees,
                          * snap cap too small, empty-name set, ... */
    cad_io = 11,         /* fopen/fwrite/close failed (save) */
    cad_empty = 12,      /* build/save with zero parts (compose would reject) */
    cad_spec_reject = 13 /* load: the parser rejected the file; the raw
                          * mig_spec_err lands in the caller's optional
                          * out-int and the design is untouched */
} cad_err;

/* The design record. Parts keep ASSEMBLY-frame poses — the frame the
 * spec file stores and the compose math consumes; the COM re-anchor is
 * the build seam's job (mig_make_bridge inside cad_build), never the
 * editor's. `selected` is shell state (index or -1) carried with the
 * design so a screen can round-trip it through save/load-free paths. */
typedef struct {
    mig_part parts [CAD_MAX_PARTS];
    int count;
    int max_parts;   /* set at init; add/array refuse past it */
    char name [MIG_SPEC_NAME_LEN];
    int selected;    /* -1 = nothing selected */
    float snap_step; /* metres; 0 = position snapping off */
    float rot_snap_deg; /* degrees; 0 = free rotation */
} cad_design;

/* ---- lifecycle ---- */

/* Zeroes everything: count 0, max_parts CAD_MAX_PARTS, name "",
 * selected -1, both snaps 0 (free). No calloc needed — plain struct. */
void cad_design_init (cad_design *design);

/* Design name: the spec grammar's charset ([A-Za-z0-9_-], 1..63 bytes)
 * because the name becomes the `mig <name>` line. Write-on-success. */
int cad_set_name (cad_design *design, const char *name);

/* ---- part CRUD (indices are positions 0..count-1 and SHIFT on
 * delete — the spec grammar's strictly-ordered part indices) ---- */

/* Typed adds: identity orientation, CAD_DEFAULT_DENSITY, caller pose.
 * All go through cad_add_part's full validation (write-on-success). */
int cad_add_sphere (cad_design *design, float radius, vector3 position);
int cad_add_cube (cad_design *design, vector3 half_extensions, vector3 position);
int cad_add_cylinder (cad_design *design, float radius, float half_length,
                      vector3 position);
/* Generic add with the caller-built part: validates type/dims/pose/quat/
 * mass exactly as mig_compose will (the same named errors), refuses a
 * full design, appends at the END (never reorders existing parts). */
int cad_add_part (cad_design *design, const mig_part *part);

/* Remove + shift down (count-1). selected adjusts to stay on the same
 * part (or drops to -1 past the end). */
int cad_delete (cad_design *design, int index);

/* Copy part `index` to the END with `offset` (snapped) added to its
 * position — the primary "spaces" verb: dup a sphere and push the copy
 * 0.8 m away and you have a dumbbell. */
int cad_duplicate (cad_design *design, int index, vector3 offset);

/* Linear pattern: n-1 copies of part `index` appended at cumulative
 * steps (original + step*k for k = 1..n-1, each snapped). n = TOTAL
 * pattern count including the original; n == 1 is a legal no-op. */
int cad_array (cad_design *design, int index, int n, vector3 step);

/* ---- transforms (positions snap to snap_step multiples when
 * snap_step > 0 — snapping is a mode, not a verb) ---- */

int cad_move (cad_design *design, int index, vector3 delta);
int cad_set_position (cad_design *design, int index, vector3 position);

/* Rotate part `index` about an ASSEMBLY-frame axis (0=x, 1=y, 2=z) by
 * `degrees` (right-hand rule). Composes q_axis * q_part so the axis
 * stays world-anchored no matter the part's current orientation.
 * rot_snap_deg > 0 rounds the request to its multiples FIRST. */
int cad_rotate_degrees (cad_design *design, int index, int axis, float degrees);

/* Rewrite either mass mode on a part (explicit kg or density kg/m^3). */
int cad_set_mass (cad_design *design, int index, mig_mass_mode mode, float value);

/* ---- dims resize (the window's scale verb) ----
 * Resize ONE part through the verb matching its type; a mismatch refuses
 * cad_bad_type (a sphere is not resized by the cube verb). Dims must be
 * finite and strictly positive (the mig_compose rule). Write-on-success:
 * a refused resize leaves the part untouched. */
int cad_set_dims_sphere (cad_design *design, int index, float radius);
int cad_set_dims_cube (cad_design *design, int index, vector3 half_extensions);
int cad_set_dims_cylinder (cad_design *design, int index, float radius,
                           float half_length);

/* Snap every part position to snap_step multiples (axis-wise). A
 * no-op while snap_step is 0; refuses cad_bad_arg for step <= 0
 * through cad_set_snap (below). */
int cad_snap_all (cad_design *design);

/* Mode setters (write-on-success; snap values must be finite and > 0,
 * 0 disables). */
int cad_set_snap (cad_design *design, float snap_step);
int cad_set_rot_snap (cad_design *design, float rot_snap_deg);

/* Shell state: index in [0, count) or -1 to deselect. */
int cad_select (cad_design *design, int index);

/* ---- persistence: the frozen spec grammar is the file format ---- */

/* Design → mig_spec (name, `frame assembly`, per-part type/dims/pose/
 * mass; no material/colour — the engine stamps body defaults at build).
 * Refuses an empty design (the grammar rejects zero-part files) and a
 * name outside the grammar. Write-on-success. */
int cad_design_to_spec (const cad_design *design, mig_spec *out);

/* Parsed spec → design: validates the part family/dims/quat/mass the
 * parser's grammar guarantees plus the max_parts cap (write-on-success
 * into `out`; `out` may alias nothing — the caller passes a fresh
 * design). Snap/selected state resets to init values (file has none). */
int cad_design_from_spec (const mig_spec *spec, int max_parts, cad_design *out);

/* Save = to_spec + mig_spec_emit_normalized + write. The file is the
 * canonical normalized text (byte-identical with what `mig save` and
 * the 4.4 panel produce for the same spec). */
int cad_save (const cad_design *design, const char *path);

/* Load = mig_spec_parse_file + design_from_spec. Write-on-success at
 * the design level: a rejected file leaves `design` untouched and
 * returns cad_spec_reject with the raw mig_spec_err (when the caller
 * passes spec_err_out). */
int cad_load (cad_design *design, const char *path, int max_parts, int *spec_err_out);

/* ---- the engine seam ---- */

/* mig_make_bridge over the design: composed totals (mass, COM,
 * inertia about COM) + the COM-rebased part copy, ready for the later
 * hookup to hand to physics_world_add_composite. `out_parts` must hold
 * count slots and must NOT alias design->parts. Empty design refuses
 * with cad_empty (mig_compose's null rule). Compose errors map to the
 * same-named cad_err values verbatim. */
int cad_build (const cad_design *design, mig_part *out_parts, mig_bridge *out);

/* ---- the ASCII workbench ---- */

/* Top-down plan view (looking down -Y: +X right, +Z down), fixed scale
 * 1 cell = 0.25 m, 64 columns x 24 rows, origin cell (32, 12); part
 * glyphs 'o' (sphere), '#' (cube), '|' (cylinder, axle local X) at the
 * rounded cell of each part position (last part wins a shared cell).
 * A header line carries name, part count, total mass (mig_part_mass
 * sum) and the count of parts outside the window; fully deterministic
 * — two calls on an unchanged design are byte-identical. Requires
 * cap >= CAD_PREVIEW_MIN_CAP (refuses cad_bad_arg otherwise). */
int cad_preview (const cad_design *design, char *buffer, int cap);

#endif /* cad_editor_h */

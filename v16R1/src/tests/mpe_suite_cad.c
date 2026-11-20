/* MIG WP4 4.6 gate (tests/mpe_suite_cad.c):
 *   mpe_t_cad_editor_model — the miniature CAD editor's DESIGN MODEL
 *   (cad/cad_editor.{h,c}), standalone: nothing in the shipped GUI calls
 *   it yet, so this gate is the ONLY thing exercising it until the
 *   terminal-style shell lands (the headless-before-GTK discipline).
 *
 * Pinned contracts:
 *   - typed adds author mig_part records exactly (identity quat, water
 *     density mode) and every entering verb validates like mig_compose
 *     will — write-on-success, witness-checked by whole-design memcmp;
 *   - transforms: move/set_position snap-to-grid modes, degrees rotations
 *     composing on the LEFT (assembly-axis anchored — the y-sign of a
 *     pre-rotated part distinguishes q_axis*q_part from q_part*q_axis),
 *     rot-snap rounding, dup/array exact offsets, delete + selection
 *     follow, caller-lowered part cap;
 *   - the build seam: cad_build ≡ a DIRECT mig_compose call on the same
 *     hand-built list (mass/COM/inertia 1e-9), the COM re-anchor equals
 *     orig − COM (1e-6), the INTER-PART GAP survives re-anchoring
 *     unchanged (the "spaces" property composites-from-welds lack), and
 *     the rebased list composes to COM ≈ 0 with identical mass;
 *   - persistence IS the frozen spec grammar: design → canonical emit →
 *     file → parse → design round-trips BITWISE (float→spec double→
 *     shortest text→double→float is lossless) and the canonical emissions
 *     are byte-identical; load refusals (missing/garbage/over-cap files)
 *     leave the design untouched and surface the raw mig_spec_err;
 *   - the ASCII plan view: deterministic byte-for-byte, glyph/geometry
 *     placement, header/legend text, cap refusal, and the stale-glyph
 *     regression (a part leaving grid row 23 must not leave its glyph
 *     behind — the static grid is cleared fully, every row, every call).
 *
 * Oracle discipline: quaternion targets are exact float closed forms
 * (90/45-degree rotations) hand-derived in the comments; compose totals
 * come from a direct mig_compose call (the bridge is a wrapper — the
 * wrapper may not drift from the composed truth, which its own gate
 * mig_mass_composite holds against closed forms). No oracle here is
 * computed with the function under test.
 */
#include "mpe_test.h"
#include "../cad/cad_editor.h"
#include "../cad/cad_view_math.h"
#include "../core/rigidbody.h"
#include "../core/physics_world.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Pointer to the start of line n (0-based) of a '\n'-separated buffer. */
static const char *cad_gate_line (const char *buffer, int n) {
    for (int i = 0; i < n; i++) {
        buffer = strchr (buffer, '\n');
        if (!buffer) { return ""; }
        buffer++;
    }
    return buffer;
}

/* Byte-equality of two whole files (single-threaded suite: static bufs). */
static int cad_gate_file_eq (const char *path_a, const char *path_b) {
    static char ba [65536];
    static char bb [65536];
    FILE *fa = fopen (path_a, "rb");
    FILE *fb = fopen (path_b, "rb");
    if ((!fa) || (!fb)) {
        if (fa) { fclose (fa); }
        if (fb) { fclose (fb); }
        return 0;
    }
    size_t la = fread (ba, 1, sizeof (ba), fa);
    size_t lb = fread (bb, 1, sizeof (bb), fb);
    fclose (fa);
    fclose (fb);
    return (la == lb) && (memcmp (ba, bb, la) == 0);
}

int mpe_t_cad_editor_model (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "cad_editor_model");
    mpe_clamp_counters_reset ();
    const char *tmpd = getenv ("TMPDIR");
    const char *base = tmpd && tmpd [0] ? tmpd : "../../temp";
    char path [512];
    char path_b [512];
    snprintf (path, sizeof (path), "%s/mpe_suite_cad_roundtrip.dat", base);
    snprintf (path_b, sizeof (path_b), "%s/mpe_suite_cad_roundtrip_b.dat", base);

    /* ---- 1. lifecycle + typed adds ------------------------------------ */
    cad_design d;
    cad_design_init (&d);
    MPE_CHECK (&t, d.count == 0 && d.max_parts == CAD_MAX_PARTS && d.selected == -1);
    MPE_CHECK (&t, d.snap_step == 0.0f && d.rot_snap_deg == 0.0f);
    MPE_CHECK (&t, cad_set_name (&d, "cad_gate") == cad_ok);
    MPE_CHECK (&t, strcmp (d.name, "cad_gate") == 0);
    /* Name refusals are write-on-success (the grammar's charset/length). */
    MPE_CHECK (&t, cad_set_name (&d, "") == cad_bad_name);
    MPE_CHECK (&t, cad_set_name (&d, "bad name!") == cad_bad_name);
    MPE_CHECK (&t, cad_set_name (&d, NULL) == cad_null);
    MPE_CHECK (&t, strcmp (d.name, "cad_gate") == 0);
    /* Typed adds: identity orientation + water-density mass mode. */
    MPE_CHECK (&t, cad_add_sphere (&d, 0.5f, (vector3) {1.0f, 0.0f, 2.0f}) == cad_ok);
    MPE_CHECK (&t, cad_add_cube (&d, (vector3) {0.2f, 0.3f, 0.4f}, (vector3) {2.0f, 0.0f, 2.0f}) == cad_ok);
    MPE_CHECK (&t, cad_add_cylinder (&d, 0.25f, 0.75f, (vector3) {0.0f, 0.5f, 0.0f}) == cad_ok);
    MPE_CHECK (&t, d.count == 3);
    MPE_CHECK (&t, d.parts [0].type == mig_part_sphere && d.parts [0].radius == 0.5f);
    MPE_CHECK (&t, d.parts [0].position.x == 1.0f && d.parts [0].position.z == 2.0f);
    MPE_CHECK (&t, d.parts [0].orientation.w == 1.0f && d.parts [0].orientation.x == 0.0f &&
                   d.parts [0].orientation.y == 0.0f && d.parts [0].orientation.z == 0.0f);
    MPE_CHECK (&t, d.parts [0].mass_mode == mig_mass_mode_density && d.parts [0].mass == CAD_DEFAULT_DENSITY);
    MPE_CHECK (&t, d.parts [1].type == mig_part_cube && d.parts [1].half_extensions.y == 0.3f);
    MPE_CHECK (&t, d.parts [2].type == mig_part_cylinder && d.parts [2].cylinder_half_length == 0.75f &&
                   d.parts [2].radius == 0.25f);

    /* ---- 2. the refusal battery (whole-design write-on-success witness)
     * Every verb below must refuse with its own named error and leave the
     * design BITWISE untouched — one memcmp at the end covers them all. */
    cad_design ref = d;
    mig_part bad;
    memset (&bad, 0, sizeof (bad));
    bad.mass_mode = mig_mass_mode_density;
    bad.mass = 1000.0f;
    bad.orientation = vector4_identity ();
    /* type outside the WP0 primitive family */
    bad.type = (mig_part_type) 7;
    bad.radius = 0.5f;
    MPE_CHECK (&t, cad_add_part (&d, &bad) == cad_bad_type);
    /* dims: zero / negative / NaN, per-type subsets */
    bad.type = mig_part_sphere;
    bad.radius = 0.0f;
    MPE_CHECK (&t, cad_add_part (&d, &bad) == cad_bad_dims);
    bad.radius = -0.5f;
    MPE_CHECK (&t, cad_add_part (&d, &bad) == cad_bad_dims);
    bad.radius = (float) NAN;
    MPE_CHECK (&t, cad_add_part (&d, &bad) == cad_bad_dims);
    bad.type = mig_part_cube;
    bad.radius = 0.0f;
    bad.half_extensions = (vector3) {0.2f, 0.3f, 0.0f};
    MPE_CHECK (&t, cad_add_part (&d, &bad) == cad_bad_dims);
    bad.type = mig_part_cylinder;
    bad.half_extensions = (vector3) {0.0f, 0.0f, 0.0f};
    bad.radius = 0.25f;
    bad.cylinder_half_length = 0.0f;
    MPE_CHECK (&t, cad_add_part (&d, &bad) == cad_bad_dims);
    /* pose: NaN / infinite component */
    bad.cylinder_half_length = 0.75f;
    bad.position = (vector3) {NAN, 0.0f, 0.0f};
    MPE_CHECK (&t, cad_add_part (&d, &bad) == cad_bad_pose);
    bad.position = (vector3) {0.0f, 0.0f, INFINITY};
    MPE_CHECK (&t, cad_add_part (&d, &bad) == cad_bad_pose);
    /* orientation: non-unit and NaN components (the mig_compose rule,
     * |n2 - 1| > 1e-4 in double) */
    bad.position = (vector3) {0.0f, 0.0f, 0.0f};
    bad.orientation = (vector4) {2.0f, 0.0f, 0.0f, 0.0f};
    MPE_CHECK (&t, cad_add_part (&d, &bad) == cad_bad_quat);
    bad.orientation = (vector4) {NAN, 0.0f, 0.0f, 0.0f};
    MPE_CHECK (&t, cad_add_part (&d, &bad) == cad_bad_quat);
    bad.orientation = vector4_identity ();
    /* mass: zero / negative / NaN in either mode */
    bad.mass = 0.0f;
    MPE_CHECK (&t, cad_add_part (&d, &bad) == cad_bad_mass);
    bad.mass = -1.0f;
    MPE_CHECK (&t, cad_add_part (&d, &bad) == cad_bad_mass);
    bad.mass = (float) NAN;
    MPE_CHECK (&t, cad_add_part (&d, &bad) == cad_bad_mass);
    /* cap: the design refuses to grow past max_parts (3 + 19 > 16) */
    bad.mass = 1000.0f;
    MPE_CHECK (&t, cad_add_part (&d, &bad) == cad_ok); /* count 4 — inside the cap */
    MPE_CHECK (&t, cad_array (&d, 0, 20, (vector3) {0.1f, 0.0f, 0.0f}) == cad_full);
    MPE_CHECK (&t, cad_delete (&d, 3) == cad_ok); /* back to 3 for the witness */
    /* index refusals across every indexed verb */
    MPE_CHECK (&t, cad_delete (&d, 3) == cad_bad_index);
    MPE_CHECK (&t, cad_delete (&d, -1) == cad_bad_index);
    MPE_CHECK (&t, cad_duplicate (&d, 99, (vector3) {0.0f, 0.0f, 0.0f}) == cad_bad_index);
    MPE_CHECK (&t, cad_array (&d, -1, 2, (vector3) {0.0f, 0.0f, 0.0f}) == cad_bad_index);
    MPE_CHECK (&t, cad_move (&d, 7, (vector3) {0.0f, 0.0f, 0.0f}) == cad_bad_index);
    MPE_CHECK (&t, cad_set_position (&d, 7, (vector3) {0.0f, 0.0f, 0.0f}) == cad_bad_index);
    MPE_CHECK (&t, cad_rotate_degrees (&d, 7, 1, 30.0f) == cad_bad_index);
    MPE_CHECK (&t, cad_set_mass (&d, 7, mig_mass_mode_explicit, 5.0f) == cad_bad_index);
    MPE_CHECK (&t, cad_select (&d, 7) == cad_bad_index);
    MPE_CHECK (&t, cad_select (&d, -2) == cad_bad_index);
    /* NaN/infinite deltas and steps */
    MPE_CHECK (&t, cad_move (&d, 0, (vector3) {NAN, 0.0f, 0.0f}) == cad_bad_pose);
    MPE_CHECK (&t, cad_set_position (&d, 0, (vector3) {0.0f, INFINITY, 0.0f}) == cad_bad_pose);
    MPE_CHECK (&t, cad_duplicate (&d, 0, (vector3) {NAN, 0.0f, 0.0f}) == cad_bad_pose);
    MPE_CHECK (&t, cad_array (&d, 0, 2, (vector3) {0.0f, INFINITY, 0.0f}) == cad_bad_pose);
    /* argument refusals: pattern count, rotation axis, degrees, mass mode,
     * snap values (finite and >= 0; 0 disables) */
    MPE_CHECK (&t, cad_array (&d, 0, 0, (vector3) {0.0f, 0.0f, 0.0f}) == cad_bad_arg);
    MPE_CHECK (&t, cad_array (&d, 0, -3, (vector3) {0.0f, 0.0f, 0.0f}) == cad_bad_arg);
    MPE_CHECK (&t, cad_rotate_degrees (&d, 0, 3, 30.0f) == cad_bad_arg);
    MPE_CHECK (&t, cad_rotate_degrees (&d, 0, -1, 30.0f) == cad_bad_arg);
    MPE_CHECK (&t, cad_rotate_degrees (&d, 0, 1, NAN) == cad_bad_arg);
    MPE_CHECK (&t, cad_set_mass (&d, 0, (mig_mass_mode) 9, 5.0f) == cad_bad_arg);
    MPE_CHECK (&t, cad_set_mass (&d, 0, mig_mass_mode_explicit, 0.0f) == cad_bad_mass);
    MPE_CHECK (&t, cad_set_mass (&d, 0, mig_mass_mode_density, -2.0f) == cad_bad_mass);
    MPE_CHECK (&t, cad_set_snap (&d, -1.0f) == cad_bad_arg);
    MPE_CHECK (&t, cad_set_snap (&d, (float) NAN) == cad_bad_arg);
    MPE_CHECK (&t, cad_set_rot_snap (&d, -5.0f) == cad_bad_arg);
    /* NULL design refusals */
    MPE_CHECK (&t, cad_delete (NULL, 0) == cad_null);
    MPE_CHECK (&t, cad_add_sphere (NULL, 1.0f, (vector3) {0.0f, 0.0f, 0.0f}) == cad_null);
    MPE_CHECK (&t, cad_set_name (NULL, "x") == cad_null);
    MPE_CHECK (&t, cad_set_snap (NULL, 1.0f) == cad_null);
    MPE_CHECK (&t, cad_snap_all (NULL) == cad_null);
    /* THE WITNESS: not one verb above changed a single byte. */
    MPE_CHECK (&t, memcmp (&d, &ref, sizeof (d)) == 0);

    /* ---- 3. move / set_position + the snap MODE (snapping is a mode, not
     * a verb: every position write lands on a step multiple while on) ---- */
    MPE_CHECK (&t, cad_move (&d, 0, (vector3) {0.5f, 0.0f, -0.25f}) == cad_ok);
    MPE_CHECK (&t, d.parts [0].position.x == 1.5f && d.parts [0].position.z == 1.75f);
    MPE_CHECK (&t, cad_set_position (&d, 0, (vector3) {0.3f, 0.0f, 0.7f}) == cad_ok);
    MPE_CHECK (&t, d.parts [0].position.x == 0.3f && d.parts [0].position.z == 0.7f); /* raw, snap off */
    MPE_CHECK (&t, cad_set_snap (&d, 0.5f) == cad_ok);
    MPE_CHECK (&t, d.snap_step == 0.5f);
    MPE_CHECK (&t, cad_set_position (&d, 0, (vector3) {0.3f, 0.0f, 0.7f}) == cad_ok);
    MPE_CHECK (&t, d.parts [0].position.x == 0.5f && d.parts [0].position.z == 0.5f); /* snapped */
    MPE_CHECK (&t, cad_move (&d, 0, (vector3) {0.24f, 0.0f, 0.0f}) == cad_ok);
    MPE_CHECK (&t, d.parts [0].position.x == 0.5f); /* 0.74 -> round(1.48) -> 0.5 */
    /* snap_all: raw positions stored while off, then snapped on demand */
    MPE_CHECK (&t, cad_set_snap (&d, 0.0f) == cad_ok);
    MPE_CHECK (&t, cad_set_position (&d, 1, (vector3) {0.3f, 0.0f, 0.2f}) == cad_ok);
    MPE_CHECK (&t, d.parts [1].position.x == 0.3f && d.parts [1].position.z == 0.2f);
    MPE_CHECK (&t, cad_set_snap (&d, 0.5f) == cad_ok);
    MPE_CHECK (&t, cad_snap_all (&d) == cad_ok);
    MPE_CHECK (&t, d.parts [1].position.x == 0.5f && d.parts [1].position.z == 0.0f);

    /* ---- 4. rotations: degrees -> left-composed axis quaternions -------
     * 90 degrees about assembly Z twice on an identity part:
     *   q_z(90) = (cos45, 0, 0, sin45) then (0, 0, 0, 1) exactly. */
    MPE_CHECK (&t, cad_rotate_degrees (&d, 0, 2, 90.0f) == cad_ok);
    MPE_CHECK_NEAR (&t, d.parts [0].orientation.w, 0.7071067811865476, 1e-5, "rot90 z w=cos45");
    MPE_CHECK_NEAR (&t, d.parts [0].orientation.z, 0.7071067811865476, 1e-5, "rot90 z z=sin45");
    MPE_CHECK_NEAR (&t, d.parts [0].orientation.x, 0.0, 1e-5, "rot90 z x=0");
    MPE_CHECK_NEAR (&t, d.parts [0].orientation.y, 0.0, 1e-5, "rot90 z y=0");
    MPE_CHECK (&t, cad_rotate_degrees (&d, 0, 2, 90.0f) == cad_ok);
    MPE_CHECK_NEAR (&t, d.parts [0].orientation.w, 0.0, 1e-5, "rot180 z w=0");
    MPE_CHECK_NEAR (&t, d.parts [0].orientation.z, 1.0, 1e-5, "rot180 z z=1");
    /* rot-snap mode: a 50-degree request rounds to the 45-degree multiple */
    MPE_CHECK (&t, cad_set_rot_snap (&d, 45.0f) == cad_ok);
    MPE_CHECK (&t, d.rot_snap_deg == 45.0f);
    MPE_CHECK (&t, cad_rotate_degrees (&d, 1, 2, 50.0f) == cad_ok);
    /* 45 degrees about Z: (cos22.5, 0, 0, sin22.5) */
    MPE_CHECK_NEAR (&t, d.parts [1].orientation.w, 0.9238795325112867, 1e-5, "rot-snap 45 w");
    MPE_CHECK_NEAR (&t, d.parts [1].orientation.z, 0.3826834323650898, 1e-5, "rot-snap 45 z");
    /* LEFT-composition pin: now rotate that pre-rotated part 90 degrees
     * about assembly X. q_x(90) = (cos45, sin45, 0, 0); the product
     * q_x(90) (X) q_z(45) hand-derives to
     *   w = cos45*cos22.5, x = sin45*cos22.5, y = -sin45*sin22.5,
     *   z = cos45*sin22.5
     * — the NEGATIVE y is what distinguishes it from the right-composed
     * q_z(45) (X) q_x(90) (whose y is positive): the axis must not rotate
     * with the part. */
    MPE_CHECK (&t, cad_rotate_degrees (&d, 1, 0, 90.0f) == cad_ok);
    MPE_CHECK_NEAR (&t, d.parts [1].orientation.w, 0.6532814824381883, 1e-5, "left-compose w");
    MPE_CHECK_NEAR (&t, d.parts [1].orientation.x, 0.6532814824381883, 1e-5, "left-compose x");
    MPE_CHECK_NEAR (&t, d.parts [1].orientation.y, -0.27059805007309856, 1e-5, "left-compose y (sign)");
    MPE_CHECK_NEAR (&t, d.parts [1].orientation.z, 0.27059805007309856, 1e-5, "left-compose z");
    MPE_CHECK (&t, cad_set_rot_snap (&d, 0.0f) == cad_ok); /* free rotation again */

    /* ---- 5. dup + array + delete/selection ---------------------------- */
    /* dup carries the FULL part record (type/dims/orientation/mass) and
     * snaps the offset position: 0.5 + 0.8 = 1.3 -> 1.5 on the 0.5 grid. */
    MPE_CHECK (&t, cad_duplicate (&d, 1, (vector3) {0.8f, 0.0f, 0.0f}) == cad_ok);
    MPE_CHECK (&t, d.count == 4);
    MPE_CHECK (&t, d.parts [3].type == d.parts [1].type);
    MPE_CHECK (&t, d.parts [3].half_extensions.x == d.parts [1].half_extensions.x &&
                   d.parts [3].half_extensions.z == d.parts [1].half_extensions.z);
    MPE_CHECK (&t, d.parts [3].orientation.w == d.parts [1].orientation.w &&
                   d.parts [3].orientation.x == d.parts [1].orientation.x &&
                   d.parts [3].orientation.y == d.parts [1].orientation.y &&
                   d.parts [3].orientation.z == d.parts [1].orientation.z);
    MPE_CHECK (&t, d.parts [3].mass == d.parts [1].mass && d.parts [3].mass_mode == d.parts [1].mass_mode);
    MPE_CHECK (&t, d.parts [3].position.x == 1.5f && d.parts [3].position.y == 0.0f &&
                   d.parts [3].position.z == 0.0f);
    /* array: n is the TOTAL pattern count (original + n-1 copies at
     * cumulative steps). Part 0 sits at (0.5, 0, 0.5); step (0.5, 0, 0). */
    MPE_CHECK (&t, cad_array (&d, 0, 3, (vector3) {0.5f, 0.0f, 0.0f}) == cad_ok);
    MPE_CHECK (&t, d.count == 6);
    MPE_CHECK (&t, d.parts [4].position.x == 1.0f && d.parts [4].position.y == 0.0f &&
                   d.parts [4].position.z == 0.5f);
    MPE_CHECK (&t, d.parts [5].position.x == 1.5f && d.parts [5].position.z == 0.5f);
    MPE_CHECK (&t, d.parts [4].type == mig_part_sphere && d.parts [4].radius == 0.5f);
    MPE_CHECK (&t, d.parts [4].orientation.w == d.parts [0].orientation.w &&
                   d.parts [5].orientation.w == d.parts [0].orientation.w);
    /* array overflow refuses BEFORE appending anything (staged-validate) */
    MPE_CHECK (&t, cad_array (&d, 0, 12, (vector3) {0.5f, 0.0f, 0.0f}) == cad_full);
    MPE_CHECK (&t, d.count == 6);
    /* delete shifts indices down; selection FOLLOWS the same part */
    MPE_CHECK (&t, cad_select (&d, 2) == cad_ok);
    MPE_CHECK (&t, d.selected == 2);
    MPE_CHECK (&t, cad_delete (&d, 0) == cad_ok);
    MPE_CHECK (&t, d.count == 5);
    MPE_CHECK (&t, d.selected == 1); /* the cylinder was 2, now 1 */
    MPE_CHECK (&t, d.parts [1].type == mig_part_cylinder);
    /* deleting the selected last slot deselects */
    MPE_CHECK (&t, cad_select (&d, 4) == cad_ok);
    MPE_CHECK (&t, cad_delete (&d, 4) == cad_ok);
    MPE_CHECK (&t, d.count == 4 && d.selected == -1);

    /* ---- 6. the build seam: cad_build ≡ mig_compose, gap preserved -----
     * Fresh asymmetric two-part design (unequal masses, COM off-origin):
     * sphere r=0.5 at x=-1, cube 0.3 at x=+2 — 3 m between centres, the
     * "spaces" case no weld-flow composite can author. */
    cad_design b;
    cad_design_init (&b);
    MPE_CHECK (&t, cad_set_name (&b, "seam_probe") == cad_ok);
    MPE_CHECK (&t, cad_add_sphere (&b, 0.5f, (vector3) {-1.0f, 0.0f, 0.0f}) == cad_ok);
    MPE_CHECK (&t, cad_add_cube (&b, (vector3) {0.3f, 0.3f, 0.3f}, (vector3) {2.0f, 0.0f, 0.0f}) == cad_ok);
    /* empty + null build refusals */
    cad_design e1;
    cad_design_init (&e1);
    mig_bridge br;
    mig_part bp [CAD_MAX_PARTS];
    MPE_CHECK (&t, cad_build (&e1, bp, &br) == cad_empty);
    MPE_CHECK (&t, cad_build (NULL, bp, &br) == cad_null);
    MPE_CHECK (&t, cad_build (&b, NULL, &br) == cad_null);
    /* the bridge equals a DIRECT compose on the same hand-built list */
    mig_totals direct;
    MPE_CHECK (&t, mig_compose (b.parts, b.count, &direct) == mig_compose_ok);
    MPE_CHECK (&t, cad_build (&b, bp, &br) == cad_ok);
    MPE_CHECK (&t, br.count == b.count && br.parts == bp);
    MPE_CHECK_NEAR (&t, br.totals.mass, direct.mass, 1e-9, "bridge mass == compose");
    MPE_CHECK_NEAR (&t, br.totals.com.x, direct.com.x, 1e-9, "bridge com.x == compose");
    MPE_CHECK_NEAR (&t, br.totals.com.y, direct.com.y, 1e-9, "bridge com.y == compose");
    MPE_CHECK_NEAR (&t, br.totals.com.z, direct.com.z, 1e-9, "bridge com.z == compose");
    MPE_CHECK_NEAR (&t, br.totals.inertia.matrix [0][0], direct.inertia.matrix [0][0], 1e-9, "bridge Ixx == compose");
    MPE_CHECK_NEAR (&t, br.totals.inertia.matrix [1][1], direct.inertia.matrix [1][1], 1e-9, "bridge Iyy == compose");
    MPE_CHECK_NEAR (&t, br.totals.inertia.matrix [2][2], direct.inertia.matrix [2][2], 1e-9, "bridge Izz == compose");
    /* the re-anchor: every part moved by exactly -COM ... */
    for (int i = 0; i < b.count; i++) {
        MPE_CHECK_NEAR (&t, bp [i].position.x, b.parts [i].position.x - direct.com.x, 1e-6, "rebase x");
        MPE_CHECK_NEAR (&t, bp [i].position.y, b.parts [i].position.y - direct.com.y, 1e-6, "rebase y");
        MPE_CHECK_NEAR (&t, bp [i].position.z, b.parts [i].position.z - direct.com.z, 1e-6, "rebase z");
    }
    /* ... while the INTER-PART GAP survives untouched ... */
    MPE_CHECK_NEAR (&t, bp [1].position.x - bp [0].position.x,
                    b.parts [1].position.x - b.parts [0].position.x, 1e-6, "gap preserved");
    /* ... and the definition frame is untouched by the build */
    MPE_CHECK (&t, b.parts [0].position.x == -1.0f && b.parts [1].position.x == 2.0f);
    /* the rebased list composes to COM-origin with the same mass */
    mig_totals rebased;
    MPE_CHECK (&t, mig_compose (bp, b.count, &rebased) == mig_compose_ok);
    MPE_CHECK_NEAR (&t, rebased.com.x, 0.0, 1e-6, "rebase centers COM");
    MPE_CHECK_NEAR (&t, rebased.mass, direct.mass, 1e-9, "rebase keeps mass");

    /* ---- 7. persistence: the frozen spec grammar IS the file format ----
     * Varied 3-part design: density sphere, EXPLICIT-mass rotated cube,
     * high-density cylinder (hand-written 30-degree z-quat literals:
     * half-angle 15 deg — cos15 = 0.9659258262890683, sin15 =
     * 0.25881904510252074, the same closed forms the MIG gates use). */
    cad_design r;
    cad_design_init (&r);
    MPE_CHECK (&t, cad_set_name (&r, "cad_rtrip") == cad_ok);
    MPE_CHECK (&t, cad_add_sphere (&r, 0.5f, (vector3) {0.25f, -0.125f, 0.5f}) == cad_ok);
    mig_part cube;
    memset (&cube, 0, sizeof (cube));
    cube.type = mig_part_cube;
    cube.half_extensions = (vector3) {0.15f, 0.2f, 0.25f};
    cube.position = (vector3) {0.55f, -0.1f, 0.4f};
    cube.orientation = (vector4) {0.9659258262890683f, 0.0f, 0.0f, 0.25881904510252074f};
    cube.mass_mode = mig_mass_mode_explicit;
    cube.mass = 12.5f;
    MPE_CHECK (&t, cad_add_part (&r, &cube) == cad_ok);
    mig_part cyl;
    memset (&cyl, 0, sizeof (cyl));
    cyl.type = mig_part_cylinder;
    cyl.radius = 0.25f;
    cyl.cylinder_half_length = 0.75f;
    cyl.position = (vector3) {0.0f, 1.5f, 0.0f};
    cyl.orientation = vector4_identity ();
    cyl.mass_mode = mig_mass_mode_density;
    cyl.mass = 3000.0f;
    MPE_CHECK (&t, cad_add_part (&r, &cyl) == cad_ok);
    /* to_spec: name, assembly frame, count */
    mig_spec s1;
    MPE_CHECK (&t, cad_design_to_spec (&r, &s1) == cad_ok);
    MPE_CHECK (&t, s1.count == 3 && s1.has_frame == 1);
    MPE_CHECK (&t, strcmp (s1.name, "cad_rtrip") == 0);
    MPE_CHECK (&t, s1.parts [1].mass_mode == (int) mig_mass_mode_explicit && s1.parts [1].mass == 12.5);
    MPE_CHECK (&t, s1.parts [2].mass_mode == (int) mig_mass_mode_density && s1.parts [2].mass == 3000.0);
    /* save -> load -> BITWISE-identical parts (float -> spec double ->
     * shortest-round-trip text -> double -> float is lossless) */
    MPE_CHECK (&t, cad_save (&r, path) == cad_ok);
    cad_design l;
    int spec_err = -777;
    MPE_CHECK (&t, cad_load (&l, path, 16, &spec_err) == cad_ok);
    MPE_CHECK (&t, spec_err == mig_spec_ok);
    MPE_CHECK (&t, l.count == 3 && strcmp (l.name, "cad_rtrip") == 0);
    /* shell state resets: the file carries no snap/selection */
    MPE_CHECK (&t, l.snap_step == 0.0f && l.rot_snap_deg == 0.0f && l.selected == -1);
    for (int i = 0; i < 3; i++) {
        MPE_CHECK (&t, memcmp (&l.parts [i], &r.parts [i], sizeof (mig_part)) == 0);
    }
    /* the canonical emission is byte-stable: saving the LOADED design
     * produces the identical file (canonical normalized form). */
    MPE_CHECK (&t, cad_save (&l, path_b) == cad_ok);
    MPE_CHECK (&t, cad_gate_file_eq (path, path_b));
    /* load refusals leave the design untouched + name the raw spec err */
    cad_design lref = l;
    int garbage_err = -1;
    FILE *gf = fopen (path_b, "w");
    MPE_CHECK (&t, gf != NULL);
    if (gf) {
        fputs ("hello world\nnot a spec at all\n", gf);
        fclose (gf);
    }
    MPE_CHECK (&t, cad_load (&l, path_b, 16, &garbage_err) == cad_spec_reject);
    MPE_CHECK (&t, garbage_err != mig_spec_ok);
    MPE_CHECK (&t, memcmp (&l, &lref, sizeof (l)) == 0);
    /* missing file */
    int missing_err = -1;
    MPE_CHECK (&t, cad_load (&l, "definitely/missing_cad_file.dat", 16, &missing_err) == cad_spec_reject);
    MPE_CHECK (&t, missing_err != mig_spec_ok);
    MPE_CHECK (&t, memcmp (&l, &lref, sizeof (l)) == 0);
    /* over-cap load: a 3-part file into a max_parts=2 world refuses at
     * PARSE time (the grammar rejects oversized files, never truncates) */
    int cap_err = -1;
    MPE_CHECK (&t, cad_load (&l, path, 2, &cap_err) == cad_spec_reject);
    MPE_CHECK (&t, cap_err == mig_spec_part_cap);
    MPE_CHECK (&t, memcmp (&l, &lref, sizeof (l)) == 0);
    /* NULL / empty-path refusals */
    MPE_CHECK (&t, cad_load (NULL, path, 16, NULL) == cad_null);
    MPE_CHECK (&t, cad_load (&l, "", 16, NULL) == cad_null);
    MPE_CHECK (&t, cad_save (&r, "") == cad_null);

    /* ---- 8. caller-lowered cap + unnamed-design refusals --------------- */
    cad_design c2;
    cad_design_init (&c2);
    c2.max_parts = 2; /* the caller may lower below the engine default */
    MPE_CHECK (&t, cad_add_sphere (&c2, 0.1f, (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
    MPE_CHECK (&t, cad_add_sphere (&c2, 0.1f, (vector3) {1.0f, 0.0f, 0.0f}) == cad_ok);
    MPE_CHECK (&t, cad_add_sphere (&c2, 0.1f, (vector3) {2.0f, 0.0f, 0.0f}) == cad_full);
    MPE_CHECK (&t, c2.count == 2);
    /* an unnamed populated design cannot become a spec file (the name
     * becomes the `mig <name>` line) */
    mig_spec cs;
    MPE_CHECK (&t, cad_design_to_spec (&c2, &cs) == cad_bad_name);
    MPE_CHECK (&t, cad_save (&c2, path) == cad_bad_name);
    /* empty-design refusals across the whole persistence + seam surface */
    mig_spec es;
    MPE_CHECK (&t, cad_design_to_spec (&e1, &es) == cad_empty);
    MPE_CHECK (&t, cad_save (&e1, path) == cad_empty);
    MPE_CHECK (&t, cad_design_from_spec (NULL, 16, &e1) == cad_null);
    MPE_CHECK (&t, cad_design_to_spec (&e1, NULL) == cad_null);
    /* build on the loaded 3-part design: works and matches a direct
     * compose of the same records (the load is build-faithful) */
    mig_part lp [CAD_MAX_PARTS];
    mig_bridge lbr;
    mig_totals ldirect;
    MPE_CHECK (&t, cad_build (&l, lp, &lbr) == cad_ok);
    MPE_CHECK (&t, mig_compose (l.parts, l.count, &ldirect) == mig_compose_ok);
    MPE_CHECK_NEAR (&t, lbr.totals.mass, ldirect.mass, 1e-9, "loaded bridge mass == compose");

    /* ---- 8b. dims resize (the window's scale verb) ---------------------
     * design r: parts [sphere, cube, cylinder]; design c2: two spheres. */
    MPE_CHECK (&t, cad_set_dims_sphere (&c2, 0, 0.3f) == cad_ok);
    MPE_CHECK (&t, c2.parts [0].radius == 0.3f);
    /* a mismatched verb refuses and the design is untouched */
    cad_design c2ref = c2;
    MPE_CHECK (&t, cad_set_dims_cube (&c2, 0, (vector3) {0.1f, 0.1f, 0.1f}) == cad_bad_type);
    MPE_CHECK (&t, cad_set_dims_cylinder (&c2, 0, 0.1f, 0.1f) == cad_bad_type);
    MPE_CHECK (&t, memcmp (&c2, &c2ref, sizeof (c2)) == 0);
    /* bad dims refuse write-on-success (sphere verb on c2's sphere;
     * cube/cylinder verbs need their own types — design r has them) */
    MPE_CHECK (&t, cad_set_dims_sphere (&c2, 0, 0.0f) == cad_bad_dims);
    MPE_CHECK (&t, cad_set_dims_sphere (&c2, 0, -1.0f) == cad_bad_dims);
    MPE_CHECK (&t, cad_set_dims_sphere (&c2, 0, (float) NAN) == cad_bad_dims);
    cad_design rref = r;
    MPE_CHECK (&t, cad_set_dims_cube (&r, 1, (vector3) {0.1f, 0.0f, 0.1f}) == cad_bad_dims);
    MPE_CHECK (&t, cad_set_dims_cube (&r, 1, (vector3) {0.1f, 0.1f, INFINITY}) == cad_bad_dims);
    MPE_CHECK (&t, cad_set_dims_cylinder (&r, 2, 0.1f, 0.0f) == cad_bad_dims);
    MPE_CHECK (&t, cad_set_dims_cylinder (&r, 2, (float) NAN, 0.1f) == cad_bad_dims);
    MPE_CHECK (&t, memcmp (&r, &rref, sizeof (r)) == 0);
    /* index + NULL refusals */
    MPE_CHECK (&t, cad_set_dims_sphere (&c2, 5, 0.1f) == cad_bad_index);
    MPE_CHECK (&t, cad_set_dims_sphere (NULL, 0, 0.1f) == cad_null);
    /* each type resizes through its own verb */
    MPE_CHECK (&t, cad_set_dims_cube (&r, 1, (vector3) {0.4f, 0.25f, 0.2f}) == cad_ok);
    MPE_CHECK (&t, r.parts [1].half_extensions.x == 0.4f && r.parts [1].half_extensions.y == 0.25f &&
                   r.parts [1].half_extensions.z == 0.2f);
    MPE_CHECK (&t, cad_set_dims_cylinder (&r, 2, 0.4f, 0.9f) == cad_ok);
    MPE_CHECK (&t, r.parts [2].radius == 0.4f && r.parts [2].cylinder_half_length == 0.9f);
    /* and a resize survives the spec round trip (the emit path carries
     * the resized dims) */
    MPE_CHECK (&t, cad_save (&r, path) == cad_ok);
    cad_design l2;
    int resize_err = -1;
    MPE_CHECK (&t, cad_load (&l2, path, 16, &resize_err) == cad_ok);
    MPE_CHECK (&t, l2.parts [1].half_extensions.x == 0.4f && l2.parts [2].radius == 0.4f &&
                   l2.parts [2].cylinder_half_length == 0.9f);

    /* ---- 9. the ASCII plan view ---------------------------------------- */
    char pv [8192];
    char pv2 [8192];
    cad_design v;
    cad_design_init (&v);
    MPE_CHECK (&t, cad_set_name (&v, "plan_view") == cad_ok);
    MPE_CHECK (&t, cad_add_sphere (&v, 0.4f, (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
    MPE_CHECK (&t, cad_add_cube (&v, (vector3) {0.3f, 0.3f, 0.3f}, (vector3) {1.0f, 0.0f, 0.25f}) == cad_ok);
    MPE_CHECK (&t, cad_add_cylinder (&v, 0.2f, 0.3f, (vector3) {0.0f, 0.0f, -0.5f}) == cad_ok);
    MPE_CHECK (&t, cad_preview (&v, pv, sizeof (pv)) == cad_ok);
    MPE_CHECK (&t, cad_preview (&v, pv2, sizeof (pv2)) == cad_ok);
    /* byte-deterministic UP TO THE NUL — the buffers' tails past the
     * terminator are uninitialized stack, never written by preview */
    MPE_CHECK (&t, strcmp (pv, pv2) == 0);
    /* grid geometry: header line 0, grid rows = lines 1..24 (row r is line
     * r+1), legend line 25; 1 cell = 0.25 m, origin cell (col 32, row 12). */
    MPE_CHECK (&t, strncmp (cad_gate_line (pv, 0), "CAD plan_view", 13) == 0);
    MPE_CHECK (&t, strstr (cad_gate_line (pv, 0), "parts 3/16") != NULL);
    MPE_CHECK (&t, strstr (cad_gate_line (pv, 25), "1 cell = 0.25 m") != NULL);
    /* sphere at the origin covers the axis '+' (parts draw last) */
    MPE_CHECK (&t, cad_gate_line (pv, 13) [32] == 'o');
    /* cube at (x=1.0, z=0.25): +4 cols, +1 row */
    MPE_CHECK (&t, cad_gate_line (pv, 14) [36] == '#');
    /* cylinder at (x=0, z=-0.5): 2 rows up */
    MPE_CHECK (&t, cad_gate_line (pv, 11) [32] == '|');
    /* axis ticks still visible away from parts */
    MPE_CHECK (&t, cad_gate_line (pv, 13) [30] == '-');
    MPE_CHECK (&t, cad_gate_line (pv, 16) [32] == '|');
    /* refusals: tiny cap and NULL buffers */
    char small [64];
    MPE_CHECK (&t, cad_preview (&v, small, sizeof (small)) == cad_bad_arg);
    MPE_CHECK (&t, cad_preview (&v, NULL, sizeof (pv)) == cad_null);
    MPE_CHECK (&t, cad_preview (NULL, pv, sizeof (pv)) == cad_null);
    /* STALE-GLYPH REGRESSION: the grid is static storage — a part that
     * LEAVES a row (here grid row 23, the memset-skipped row of the
     * original partial clear) must not leave its glyph behind. */
    MPE_CHECK (&t, cad_set_position (&v, 0, (vector3) {0.0f, 0.0f, 2.75f}) == cad_ok);
    MPE_CHECK (&t, cad_preview (&v, pv, sizeof (pv)) == cad_ok);
    MPE_CHECK (&t, cad_gate_line (pv, 24) [32] == 'o'); /* row 23 = line 24 */
    MPE_CHECK (&t, cad_set_position (&v, 0, (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
    MPE_CHECK (&t, cad_preview (&v, pv, sizeof (pv)) == cad_ok);
    /* row 23 clean — memchr over exactly the 64 grid columns (strchr
     * would run past the row's newline into the legend text, which
     * contains 'o' in "not shown") */
    MPE_CHECK (&t, memchr (cad_gate_line (pv, 24), 'o', 64) == NULL);
    MPE_CHECK (&t, cad_gate_line (pv, 13) [32] == 'o'); /* and back at the origin */

    mpe_test_end (&t);
    return t.failures;
}

/* ---- WP4 4.6b gate: cad_view_math ----------------------------------
 * The CAD window's picking/camera math, gated headless (the window TU
 * itself is GUI-only and suite-unlinked — the math that decides which
 * part you clicked must not be). Oracle discipline: every target is a
 * hand-derived closed form written in the comment beside the check
 * (orbit basis from the angle definitions, ray-primitive t values from
 * the quadratic/slab geometry, the spread ray from tan(fov/2) * aspect)
 * — never computed with the function under test. */
int mpe_t_cad_view_math (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "cad_view_math");
    mpe_clamp_counters_reset ();

    /* ---- 1. orbit basis ------------------------------------------------ */
    cad_view_camera cam;
    memset (&cam, 0, sizeof (cam));
    cam.target = (vector3) {0.0f, 0.0f, 0.0f};
    cam.yaw_deg = 0.0f;
    cam.pitch_deg = 0.0f;
    cam.distance = 4.0f;
    cam.fov_y_deg = 90.0f;
    cam.aspect = 2.0f;
    /* yaw 0 / pitch 0: eye on +Z looking down -Z. */
    MPE_CHECK_NEAR (&t, cad_view_forward (&cam).z, -1.0, 1e-5, "fwd -Z");
    MPE_CHECK_NEAR (&t, cad_view_forward (&cam).x, 0.0, 1e-5, "fwd x 0");
    MPE_CHECK_NEAR (&t, cad_view_eye (&cam).z, 4.0, 1e-6, "eye z=+d");
    MPE_CHECK_NEAR (&t, cad_view_right (&cam).x, 1.0, 1e-5, "right +X");
    MPE_CHECK_NEAR (&t, cad_view_up (&cam).y, 1.0, 1e-5, "up +Y");
    /* yaw 90: the eye swings to +X, forward -X, screen-right -Z
     * (cross(fwd, up) = cross((-1,0,0), (0,1,0)) = (0,0,-1)). */
    cam.yaw_deg = 90.0f;
    MPE_CHECK_NEAR (&t, cad_view_forward (&cam).x, -1.0, 1e-5, "fwd -X");
    MPE_CHECK_NEAR (&t, cad_view_eye (&cam).x, 4.0, 1e-5, "eye x=+d");
    MPE_CHECK_NEAR (&t, cad_view_right (&cam).z, -1.0, 1e-5, "right -Z");
    /* pitch 30: eye y = d*sin(30) = 2, horizontal reach d*cos(30). */
    cam.yaw_deg = 0.0f;
    cam.pitch_deg = 30.0f;
    MPE_CHECK_NEAR (&t, cad_view_eye (&cam).y, 2.0, 1e-5, "eye y=d*sin30");
    MPE_CHECK_NEAR (&t, cad_view_eye (&cam).z, 4.0 * 0.8660254037844386, 1e-5,
                    "eye z=d*cos30");
    MPE_CHECK_NEAR (&t, cad_view_forward (&cam).y, -0.5, 1e-5, "fwd y=-sin30");
    /* general pose: the three basis vectors stay orthonormal and the
     * eye sits exactly one `distance` back along the forward axis. */
    cam.yaw_deg = 37.0f;
    cam.pitch_deg = -22.0f;
    cam.distance = 6.0f;
    {
        const vector3 f = cad_view_forward (&cam);
        const vector3 r = cad_view_right (&cam);
        const vector3 u = cad_view_up (&cam);
        const vector3 e = cad_view_eye (&cam);
        MPE_CHECK_NEAR (&t, (double) vector3_dot (f, f), 1.0, 1e-5, "|f|=1");
        MPE_CHECK_NEAR (&t, (double) vector3_dot (r, r), 1.0, 1e-5, "|r|=1");
        MPE_CHECK_NEAR (&t, (double) vector3_dot (u, u), 1.0, 1e-5, "|u|=1");
        MPE_CHECK_NEAR (&t, (double) vector3_dot (f, r), 0.0, 1e-5, "f.r=0");
        MPE_CHECK_NEAR (&t, (double) vector3_dot (f, u), 0.0, 1e-5, "f.u=0");
        MPE_CHECK_NEAR (&t, (double) vector3_dot (r, u), 0.0, 1e-5, "r.u=0");
        /* eye == target - f*6, component-wise */
        MPE_CHECK_NEAR (&t, e.x, cam.target.x - 6.0 * (double) f.x, 1e-5, "eye on axis x");
        MPE_CHECK_NEAR (&t, e.y, cam.target.y - 6.0 * (double) f.y, 1e-5, "eye on axis y");
        MPE_CHECK_NEAR (&t, e.z, cam.target.z - 6.0 * (double) f.z, 1e-5, "eye on axis z");
    }

    /* ---- 2. screen -> ray ---------------------------------------------- */
    cam.yaw_deg = 0.0f;
    cam.pitch_deg = 0.0f;
    cam.distance = 4.0f;
    cam.fov_y_deg = 90.0f; /* tan(fov/2) = 1 */
    cam.aspect = 2.0f;
    {
        const cad_view_ray center = cad_view_ray_from_screen (&cam, 0.0f, 0.0f);
        MPE_CHECK_NEAR (&t, center.direction.z, -1.0, 1e-6, "center ray -Z");
        MPE_CHECK_NEAR (&t, center.origin.z, 4.0, 1e-6, "center ray origin");
        /* ndc (1,1): spread right*1*1*2 + up*1*1 = (2,1,0);
         * dir = (2,1,-1)/sqrt(6) = (.8165, .4082, -.4082). */
        const cad_view_ray corner = cad_view_ray_from_screen (&cam, 1.0f, 1.0f);
        MPE_CHECK_NEAR (&t, corner.direction.x, 0.816496580927726, 1e-5,
                        "corner ray x");
        MPE_CHECK_NEAR (&t, corner.direction.y, 0.408248290463863, 1e-5,
                        "corner ray y");
        MPE_CHECK_NEAR (&t, corner.direction.z, -0.408248290463863, 1e-5,
                        "corner ray z");
        MPE_CHECK_NEAR (&t, (double) vector3_dot (corner.direction,
                                                  corner.direction), 1.0, 1e-6,
                        "corner ray unit");
        /* The center ray through the camera hits a sphere AT the target:
         * t = d - r = 3.5, surface point (0,0,0.5), outward normal +Z. */
        const cad_view_hit hit_target =
            cad_view_pick_sphere (center, (vector3) {0.0f, 0.0f, 0.0f}, 0.5f);
        MPE_CHECK (&t, hit_target.hit);
        MPE_CHECK_NEAR (&t, hit_target.t, 3.5, 1e-6, "target sphere t");
        MPE_CHECK_NEAR (&t, hit_target.normal.z, 1.0, 1e-6, "target sphere n");
    }

    /* ---- 3. ray-sphere exact ------------------------------------------- */
    {
        const cad_view_ray ray = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        /* forward hit: t = 2 - 0.5 = 1.5, point (0,0,-1.5), normal +Z. */
        const cad_view_hit hit = cad_view_pick_sphere (
            ray, (vector3) {0.0f, 0.0f, -2.0f}, 0.5f);
        MPE_CHECK (&t, hit.hit);
        MPE_CHECK_NEAR (&t, hit.t, 1.5, 1e-6, "sphere t");
        MPE_CHECK_NEAR (&t, hit.point.z, -1.5, 1e-6, "sphere point");
        MPE_CHECK_NEAR (&t, hit.normal.z, 1.0, 1e-6, "sphere normal");
        /* miss (off-axis) and behind-the-ray both refuse */
        MPE_CHECK (&t, !cad_view_pick_sphere (
            ray, (vector3) {0.0f, 5.0f, -2.0f}, 0.5f).hit);
        MPE_CHECK (&t, !cad_view_pick_sphere (
            ray, (vector3) {0.0f, 0.0f, 2.0f}, 0.5f).hit);
        /* origin inside: t = exit distance 0.5, normal points OUT -Z. */
        const cad_view_ray inside = {{0.0f, 0.0f, -2.0f}, {0.0f, 0.0f, -1.0f}};
        const cad_view_hit exit_hit = cad_view_pick_sphere (
            inside, (vector3) {0.0f, 0.0f, -2.0f}, 0.5f);
        MPE_CHECK (&t, exit_hit.hit);
        MPE_CHECK_NEAR (&t, exit_hit.t, 0.5, 1e-6, "inside t=exit");
        MPE_CHECK_NEAR (&t, exit_hit.normal.z, -1.0, 1e-6, "inside normal");
    }

    /* ---- 4. ray-box (slab + ORIENTATION) -------------------------------- */
    {
        const cad_view_ray ray = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        /* face hit through z = -2: t = 2, normal +Z. */
        const cad_view_hit hit = cad_view_pick_box (
            ray, (vector3) {0.0f, 0.0f, -3.0f}, (vector3) {1.0f, 1.0f, 1.0f},
            vector4_identity ());
        MPE_CHECK (&t, hit.hit);
        MPE_CHECK_NEAR (&t, hit.t, 2.0, 1e-6, "box t");
        MPE_CHECK_NEAR (&t, hit.point.z, -2.0, 1e-6, "box point");
        MPE_CHECK_NEAR (&t, hit.normal.z, 1.0, 1e-6, "box normal");
        /* parallel OUTSIDE the x slab misses outright */
        const cad_view_ray offset = {{5.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        MPE_CHECK (&t, !cad_view_pick_box (
            offset, (vector3) {0.0f, 0.0f, -3.0f}, (vector3) {1.0f, 1.0f, 1.0f},
            vector4_identity ()).hit);
        /* origin inside: t = 0 */
        const cad_view_ray inside = {{0.0f, 0.0f, -3.0f}, {0.0f, 0.0f, -1.0f}};
        const cad_view_hit inside_hit = cad_view_pick_box (
            inside, (vector3) {0.0f, 0.0f, -3.0f}, (vector3) {1.0f, 1.0f, 1.0f},
            vector4_identity ());
        MPE_CHECK (&t, inside_hit.hit && inside_hit.t == 0.0f);
        /* ORIENTATION: a 2x1x1 box rotated 90 degrees about Z occupies
         * y in [-2, 2] in the assembly frame (rotating local x-extent 2
         * onto assembly y). The ray at assembly y = 1.5 misses the
         * UNROTATED 1x1x1 box but hits the rotated one, both t = 2
         * through the same face plane — the quat did the work. */
        const vector4 z90 = {0.7071067811865476f, 0.0f, 0.0f,
                             0.7071067811865476f};
        const cad_view_ray at_y15 = {{0.5f, 1.5f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        MPE_CHECK (&t, !cad_view_pick_box (
            at_y15, (vector3) {0.0f, 0.0f, -3.0f}, (vector3) {1.0f, 1.0f, 1.0f},
            vector4_identity ()).hit);
        const cad_view_hit rot_hit = cad_view_pick_box (
            at_y15, (vector3) {0.0f, 0.0f, -3.0f}, (vector3) {2.0f, 1.0f, 1.0f},
            z90);
        MPE_CHECK (&t, rot_hit.hit);
        MPE_CHECK_NEAR (&t, rot_hit.t, 2.0, 1e-5, "rotated box t");
        MPE_CHECK_NEAR (&t, rot_hit.point.y, 1.5, 1e-5, "rotated box point y");
        MPE_CHECK_NEAR (&t, rot_hit.normal.z, 1.0, 1e-5, "rotated box normal");
    }

    /* ---- 5. ray-cylinder (axle LOCAL X: sides + caps) ------------------- */
    {
        const cad_view_ray ray = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}};
        /* side hit through the cross-section circle: (3 - t)^2 = 0.25
         * -> t = 2.5 (nearest root), point (0,0,-2.5), normal +Z. */
        const cad_view_hit side = cad_view_pick_cylinder (
            ray, (vector3) {0.0f, 0.0f, -3.0f}, 0.5f, 1.0f, vector4_identity ());
        MPE_CHECK (&t, side.hit);
        MPE_CHECK_NEAR (&t, side.t, 2.5, 1e-6, "cyl side t");
        MPE_CHECK_NEAR (&t, side.point.z, -2.5, 1e-6, "cyl side point");
        MPE_CHECK_NEAR (&t, side.normal.z, 1.0, 1e-6, "cyl side normal");
        /* cap hit: ray along -X crosses the x = +1 disc at t = 4. */
        const cad_view_ray cap_ray = {{5.0f, 0.0f, -3.0f}, {-1.0f, 0.0f, 0.0f}};
        const cad_view_hit cap = cad_view_pick_cylinder (
            cap_ray, (vector3) {0.0f, 0.0f, -3.0f}, 0.5f, 1.0f,
            vector4_identity ());
        MPE_CHECK (&t, cap.hit);
        MPE_CHECK_NEAR (&t, cap.t, 4.0, 1e-6, "cyl cap t");
        MPE_CHECK_NEAR (&t, cap.point.x, 1.0, 1e-6, "cyl cap point");
        MPE_CHECK_NEAR (&t, cap.normal.x, 1.0, 1e-6, "cyl cap normal");
        /* parallel to the axle but outside the tube radius: the cap
         * disc check refuses (0.6^2 > 0.5^2) -> miss */
        const cad_view_ray beside = {{0.0f, 0.6f, -3.0f}, {1.0f, 0.0f, 0.0f}};
        MPE_CHECK (&t, !cad_view_pick_cylinder (
            beside, (vector3) {0.0f, 0.0f, -3.0f}, 0.5f, 1.0f,
            vector4_identity ()).hit);
        /* cylinder behind the ray: miss */
        const cad_view_ray away = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
        MPE_CHECK (&t, !cad_view_pick_cylinder (
            away, (vector3) {0.0f, 0.0f, -3.0f}, 0.5f, 1.0f,
            vector4_identity ()).hit);
    }

    /* ---- 6. work-plane drag --------------------------------------------- */
    {
        /* ray from (0,2,0) along (0,-1,-1)/sqrt(2) hits y=0 at z = -2:
         * t = 2 / (1/sqrt(2)) = 2*sqrt(2), point (0, 0, -2). */
        cad_view_ray ray;
        ray.origin = (vector3) {0.0f, 2.0f, 0.0f};
        ray.direction = vector3_normalisation ((vector3) {0.0f, -1.0f, -1.0f});
        vector3 out = {0.0f, 0.0f, 0.0f};
        MPE_CHECK (&t, cad_view_ray_plane (ray, (vector3) {0.0f, 0.0f, 0.0f},
                                           (vector3) {0.0f, 1.0f, 0.0f},
                                           &out) == 1);
        MPE_CHECK_NEAR (&t, out.y, 0.0, 1e-5, "plane out y");
        MPE_CHECK_NEAR (&t, out.z, -2.0, 1e-5, "plane out z");
        /* parallel ray: 0, out untouched */
        const cad_view_ray parallel = {{0.0f, 2.0f, 0.0f}, {1.0f, 0.0f, 0.0f}};
        vector3 untouched = {7.0f, 7.0f, 7.0f};
        MPE_CHECK (&t, cad_view_ray_plane (parallel, (vector3) {0.0f, 0.0f, 0.0f},
                                           (vector3) {0.0f, 1.0f, 0.0f},
                                           &untouched) == 0);
        MPE_CHECK (&t, untouched.x == 7.0f && untouched.z == 7.0f);
        /* plane behind the ray: 0 (refuse, don't wrap) */
        const cad_view_ray away = {{0.0f, 2.0f, 0.0f}, {0.0f, 1.0f, 0.0f}};
        MPE_CHECK (&t, cad_view_ray_plane (away, (vector3) {0.0f, 0.0f, 0.0f},
                                           (vector3) {0.0f, 1.0f, 0.0f},
                                           &out) == 0);
        /* NULL out refuses */
        MPE_CHECK (&t, cad_view_ray_plane (ray, (vector3) {0.0f, 0.0f, 0.0f},
                                           (vector3) {0.0f, 1.0f, 0.0f},
                                           NULL) == 0);
    }

    mpe_test_end (&t);
    return t.failures;
}

/* MIG WP7 7.0b gate (tests/mpe_suite_cad.c):
 *   mpe_t_cad_void_model — the void side of the CAD design model:
 *   void verbs author twin-typed density records, cad_set_mass refuses
 *   explicit on void, dims verbs match the geometry twin, to_spec
 *   refuses void-bearing designs (cad_void_grammar), cad_build routes
 *   void-bearing designs through mig_void_make_bridge with the extended
 *   error mapping, the plan view uses distinct void glyphs, and the
 *   header nets negative void mass. Format-stability: a void-free
 *   design's preview is byte-identical to before 7.0b.
 *
 * Oracle discipline: every expected mass/inertia is derived here from
 * closed-form rigid-body geometry — never from the function under test.
 */
int mpe_t_cad_void_model (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "cad_void_model");
    mpe_clamp_counters_reset ();

    /* ---- 1. void verbs author twin records at default density -------- */
    {
        cad_design d;
        cad_design_init (&d);
        MPE_CHECK (&t, cad_add_void_sphere (&d, 0.25f, (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        MPE_CHECK (&t, cad_add_void_cube (&d, (vector3) {0.2f, 0.3f, 0.4f},
                                          (vector3) {1.0f, 0.0f, 0.0f}) == cad_ok);
        MPE_CHECK (&t, cad_add_void_cylinder (&d, 0.15f, 0.6f,
                                              (vector3) {0.0f, 2.0f, 0.0f}) == cad_ok);
        MPE_CHECK (&t, d.count == 3);
        /* twin geometry, void type, density mode, identity quat, default density. */
        MPE_CHECK (&t, d.parts [0].type == mig_part_void_sphere &&
                       d.parts [0].radius == 0.25f &&
                       d.parts [0].mass_mode == mig_mass_mode_density &&
                       d.parts [0].mass == CAD_DEFAULT_DENSITY &&
                       d.parts [0].orientation.w == 1.0f);
        MPE_CHECK (&t, d.parts [1].type == mig_part_void_cube &&
                       d.parts [1].half_extensions.x == 0.2f &&
                       d.parts [1].mass_mode == mig_mass_mode_density);
        MPE_CHECK (&t, d.parts [2].type == mig_part_void_cylinder &&
                       d.parts [2].radius == 0.15f &&
                       d.parts [2].cylinder_half_length == 0.6f);
        /* void dims validation: negative dims refuse (twin rules). */
        MPE_CHECK (&t, cad_add_void_sphere (&d, -0.1f, (vector3) {0.0f, 0.0f, 0.0f}) == cad_bad_dims);
        MPE_CHECK (&t, cad_add_void_cube (&d, (vector3) {0.2f, 0.0f, 0.4f},
                                          (vector3) {0.0f, 0.0f, 0.0f}) == cad_bad_dims);
        MPE_CHECK (&t, cad_add_void_cylinder (&d, 0.15f, -0.6f,
                                              (vector3) {0.0f, 0.0f, 0.0f}) == cad_bad_dims);
        MPE_CHECK (&t, d.count == 3); /* refused adds leave the design untouched */
    }

    /* ---- 2. cad_set_mass: density-only on void ---------------------- */
    {
        cad_design d;
        cad_design_init (&d);
        MPE_CHECK (&t, cad_add_void_sphere (&d, 0.25f, (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        /* explicit kg on a void is meaningless -> cad_bad_mass_mode. */
        MPE_CHECK (&t, cad_set_mass (&d, 0, mig_mass_mode_explicit, 5.0f) == cad_bad_mass_mode);
        /* density stays freely rewritable. */
        MPE_CHECK (&t, cad_set_mass (&d, 0, mig_mass_mode_density, 2000.0f) == cad_ok);
        MPE_CHECK (&t, d.parts [0].mass == 2000.0f &&
                       d.parts [0].mass_mode == mig_mass_mode_density);
    }

    /* ---- 3. dims verbs match the geometry twin ---------------------- */
    {
        cad_design d;
        cad_design_init (&d);
        MPE_CHECK (&t, cad_add_void_sphere (&d, 0.25f, (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        MPE_CHECK (&t, cad_add_void_cube (&d, (vector3) {0.2f, 0.2f, 0.2f},
                                          (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        /* void_sphere resizes via the sphere verb; wrong verb refuses. */
        MPE_CHECK (&t, cad_set_dims_sphere (&d, 0, 0.5f) == cad_ok);
        MPE_CHECK (&t, d.parts [0].radius == 0.5f);
        MPE_CHECK (&t, cad_set_dims_cube (&d, 0, (vector3) {0.3f, 0.3f, 0.3f}) == cad_bad_type);
        /* void_cube resizes via the cube verb. */
        MPE_CHECK (&t, cad_set_dims_cube (&d, 1, (vector3) {0.3f, 0.3f, 0.3f}) == cad_ok);
        MPE_CHECK (&t, d.parts [1].half_extensions.x == 0.3f);
        MPE_CHECK (&t, cad_set_dims_sphere (&d, 1, 0.5f) == cad_bad_type);
    }

    /* ---- 4. cad_design_to_spec accepts void-bearing (v2 grammar) ------ */
    {
        cad_design d;
        cad_design_init (&d);
        MPE_CHECK (&t, cad_set_name (&d, "voidy") == cad_ok);
        MPE_CHECK (&t, cad_add_void_sphere (&d, 0.25f, (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        mig_spec spec;
        memset (&spec, 0, sizeof (spec));
        /* WP7 7.0d: void-bearing designs can now be persisted — the v2
         * grammar expresses void records. */
        MPE_CHECK (&t, cad_design_to_spec (&d, &spec) == cad_ok);
        /* Written spec carries the void record. */
        MPE_CHECK (&t, spec.count == 1 &&
                       spec.parts [0].type == (int) mig_part_void_sphere &&
                       spec.parts [0].radius == 0.25);
        /* Material-only design still goes through (grammar unaffected). */
        cad_design m;
        cad_design_init (&m);
        MPE_CHECK (&t, cad_set_name (&m, "mat") == cad_ok);
        MPE_CHECK (&t, cad_add_sphere (&m, 0.5f, (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        MPE_CHECK (&t, cad_design_to_spec (&m, &spec) == cad_ok);
        MPE_CHECK (&t, spec.count == 1 &&
                       spec.parts [0].type == (int) mig_part_sphere);
    }

    /* ---- 5. cad_build routes void-bearing through the void module --- */
    {
        cad_design d;
        cad_design_init (&d);
        MPE_CHECK (&t, cad_set_name (&d, "voidasm") == cad_ok);
        MPE_CHECK (&t, cad_add_cube (&d, (vector3) {0.5f, 0.5f, 0.5f},
                                     (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        MPE_CHECK (&t, cad_add_void_sphere (&d, 0.25f, (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        mig_part out_parts [16];
        mig_bridge br;
        memset (&br, 0xAB, sizeof (br));
        int rc = cad_build (&d, out_parts, &br);
        MPE_CHECK (&t, rc == cad_ok);
        MPE_CHECK (&t, br.count == 2);
        /* pool-inclusive: both records staged, void gets is_void=1 in the
         * engine path; here the bridge carries the void record verbatim
         * (negative effective mass comes from mig_part_mass at spawn). */
        MPE_CHECK (&t, out_parts [0].type == mig_part_cube);
        MPE_CHECK (&t, out_parts [1].type == mig_part_void_sphere);
        /* net mass: box 1000 kg − sphere ~65.45 kg ≈ 934.55 kg. */
        const double m_box = 1000.0 * 1.0 * 1.0 * 1.0;
        const double m_void = 1000.0 * (4.0 / 3.0) * 3.14159265358979323846 * 0.25 * 0.25 * 0.25;
        MPE_CHECK_NEAR (&t, br.totals.mass, m_box - m_void, 1e-9, "cad build nets void mass");
        /* material-only design keeps the untouched mig_make_bridge path. */
        cad_design m;
        cad_design_init (&m);
        MPE_CHECK (&t, cad_add_cube (&m, (vector3) {0.5f, 0.5f, 0.5f},
                                     (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        mig_part out_m [16]; mig_bridge br_m;
        memset (&br_m, 0xAB, sizeof (br_m));
        MPE_CHECK (&t, cad_build (&m, out_m, &br_m) == cad_ok);
        MPE_CHECK_NEAR (&t, br_m.totals.mass, m_box, 1e-9, "material-only build == cube mass");
    }

    /* ---- 6. cad_build refusal mapping (void-only -> cad_empty) ------ */
    {
        cad_design d;
        cad_design_init (&d);
        MPE_CHECK (&t, cad_add_void_sphere (&d, 0.25f, (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        mig_part out_parts [16]; mig_bridge br;
        MPE_CHECK (&t, cad_build (&d, out_parts, &br) == cad_empty); /* void alone is no body */
    }

    /* ---- 7. plan view: void glyphs + net mass header --------------- */
    {
        cad_design d;
        cad_design_init (&d);
        MPE_CHECK (&t, cad_set_name (&d, "voidplan") == cad_ok);
        MPE_CHECK (&t, cad_add_void_sphere (&d, 0.25f, (vector3) {1.0f, 0.0f, 2.0f}) == cad_ok);
        MPE_CHECK (&t, cad_add_void_cube (&d, (vector3) {0.2f, 0.2f, 0.2f},
                                          (vector3) {-1.0f, 0.0f, -2.0f}) == cad_ok);
        MPE_CHECK (&t, cad_add_void_cylinder (&d, 0.15f, 0.6f,
                                              (vector3) {3.0f, 0.0f, 0.0f}) == cad_ok);
        char buf [CAD_PREVIEW_MIN_CAP];
        memset (buf, 0, sizeof (buf));
        MPE_CHECK (&t, cad_preview (&d, buf, sizeof (buf)) == cad_ok);
        /* void glyphs O / % / ! at rounded cells (1 cell = 0.25 m, origin (32,12)).
         * x=1.0 -> col 36; x=-1.0 -> col 28; z=3.0 -> row 12+12=24 (outside view,
         * cylinder's z=3 puts it beyond the window -> counted outside). */
        MPE_CHECK (&t, cad_gate_line (buf, 21)[36] == 'O');   /* void sphere */
        MPE_CHECK (&t, cad_gate_line (buf, 5)[28] == '%');    /* void cube */
        /* cylinder at z=3.0 -> grid row 24 >= gh=24 -> outside view (counted, no glyph). */
        /* header nets the negative void mass. */
        MPE_CHECK (&t, strstr (buf, "mass ") != NULL);
        /* byte-identical on repeat (deterministic). */
        char buf2 [CAD_PREVIEW_MIN_CAP];
        memset (buf2, 0, sizeof (buf2));
        MPE_CHECK (&t, cad_preview (&d, buf2, sizeof (buf2)) == cad_ok);
        MPE_CHECK (&t, memcmp (buf, buf2, sizeof (buf)) == 0);
        /* void-free design keeps material glyphs (format stability).
         * Sphere at (0,0,0) -> grid row 12 -> buffer line 13, col 32. */
        cad_design m;
        cad_design_init (&m);
        MPE_CHECK (&t, cad_add_sphere (&m, 0.5f, (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        char bufm [CAD_PREVIEW_MIN_CAP];
        memset (bufm, 0, sizeof (bufm));
        MPE_CHECK (&t, cad_preview (&m, bufm, sizeof (bufm)) == cad_ok);
        MPE_CHECK (&t, cad_gate_line (bufm, 13)[32] == 'o'); /* sphere glyph unchanged */
    }

    /* ---- 8. full chain: design → add_composite mass ---------------- */
    {
        cad_design d;
        cad_design_init (&d);
        MPE_CHECK (&t, cad_set_name (&d, "chain") == cad_ok);
        MPE_CHECK (&t, cad_add_cube (&d, (vector3) {0.5f, 0.5f, 0.5f},
                                     (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        MPE_CHECK (&t, cad_add_void_sphere (&d, 0.25f, (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        mig_part out_parts [16];
        mig_bridge br;
        MPE_CHECK (&t, cad_build (&d, out_parts, &br) == cad_ok);
        physics_world w;
        mpe_world_begin (&w);
        const int cb = physics_world_add_composite (&w, out_parts, br.count);
        MPE_CHECK (&t, cb == 0 && w.body_count == 1 && w.composite_part_count == 2);
        if (cb >= 0) {
            MPE_CHECK (&t, w.bodies [cb].type == object_composite);
            /* engine nets the void's negative effective mass. */
            const double m_box = 1000.0 * 1.0 * 1.0 * 1.0;
            const double m_void = 1000.0 * (4.0 / 3.0) * 3.14159265358979323846 * 0.25 * 0.25 * 0.25;
            MPE_CHECK_NEAR (&t, w.bodies [cb].mass, (float) (m_box - m_void), 1e-4f,
                            "end-to-end net mass via cad build + spawn");
            /* pool-inclusive is_void flag. */
            MPE_CHECK (&t, w.composite_parts [1].is_void == 1);
            MPE_CHECK_NEAR (&t, w.composite_parts [1].mass, (float) -m_void, 1e-6f,
                            "pool void mass negative");
        }
        physics_world_cleanup (&w);
    }

    mpe_test_end (&t);
    return t.failures;
}

/* MIG WP7 7.0c gate (tests/mpe_suite_cad.c):
 *   mpe_t_cad_subtract_void — destructive subtract verbs:
 *   cad_subtract_box partitions a cube target into up to 6 slabs around
 *   the subtracted void region; cad_subtract_sphere uses the inscribed
 *   cube (disclosed inscribed-envelope — removes less than authored,
 *   never more); cad_undo restores the one-level pre-subtract snapshot;
 *   round targets are refused; void regions poking out of the target are
 *   refused; the design stays buildable (cad_build ok) after subtract.
 *
 * Oracle discipline: every expected slab count/dim/mass is derived here
 * from axis-aligned box arithmetic — never from the function under test.
 */
int mpe_t_cad_subtract_void (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "cad_subtract_void");
    mpe_clamp_counters_reset ();

    /* ---- 1. subtract_box: centred cube void in a cube target -------- */
    {
        cad_design d;
        cad_design_init (&d);
        MPE_CHECK (&t, cad_set_name (&d, "voidbox") == cad_ok);
        /* Target cube: half 1.0 (2x2x2), centred at origin. */
        MPE_CHECK (&t, cad_add_cube (&d, (vector3) {1.0f, 1.0f, 1.0f},
                                     (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        /* Void: half 0.5 cube centred at origin — centred slab removal. */
        MPE_CHECK (&t, cad_subtract_box (&d, 0, (vector3) {0.0f, 0.0f, 0.0f},
                                         (vector3) {0.5f, 0.5f, 0.5f}) == cad_ok);
        /* 2 slabs: X-outside + (middle strip Y-outside + Z-outside) =
         * actually centred void removes face slabs: X-left, X-right,
         * Y-front, Y-back, Z-front, Z-back -> 6 slabs. */
        MPE_CHECK (&t, d.count == 6); /* 6 slabs around centred void */
        MPE_CHECK (&t, d.selected == 0); /* selection on first slab */
        /* Undo restores the single target cube. */
        MPE_CHECK (&t, cad_undo (&d) == cad_ok);
        MPE_CHECK (&t, d.count == 1 &&
                       d.parts [0].type == mig_part_cube &&
                       d.parts [0].half_extensions.x == 1.0f);
        /* Undo again: nothing to undo -> cad_bad_arg. */
        MPE_CHECK (&t, cad_undo (&d) == cad_bad_arg);
        /* cad_build stays green after subtract (design buildable). */
        mig_part out [16]; mig_bridge br;
        MPE_CHECK (&t, cad_build (&d, out, &br) == cad_ok);
    }

    /* ---- 2. subtract_box: off-centre void -> fewer slabs ------------ */
    {
        cad_design d;
        cad_design_init (&d);
        MPE_CHECK (&t, cad_add_cube (&d, (vector3) {1.0f, 1.0f, 1.0f},
                                     (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        /* Void at the +X face: touches x=1.0 face -> X+ slab has zero
         * thickness (omitted), but the middle X strip still keeps Y and
         * Z slabs: X- slab + Y- + Y+ + Z- + Z+ = 5 slabs. */
        MPE_CHECK (&t, cad_subtract_box (&d, 0, (vector3) {0.5f, 0.0f, 0.0f},
                                         (vector3) {0.5f, 0.5f, 0.5f}) == cad_ok);
        MPE_CHECK (&t, d.count == 5);
        /* X- slab: x in [-1, 0], half 0.5, centred at x=-0.5. */
        MPE_CHECK (&t, d.parts [0].type == mig_part_cube);
        MPE_CHECK (&t, d.parts [0].half_extensions.x == 0.5f);
        MPE_CHECK (&t, d.parts [0].position.x == -0.5f);
        /* cad_build stays green after subtract (design buildable). */
        {
            mig_part o [16]; mig_bridge b;
            MPE_CHECK (&t, cad_build (&d, o, &b) == cad_ok);
        }
    }

    /* ---- 3. subtract_box refusal battery ---------------------------- */
    {
        cad_design d;
        cad_design_init (&d);
        MPE_CHECK (&t, cad_add_cube (&d, (vector3) {1.0f, 1.0f, 1.0f},
                                     (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        /* Bad index. */
        MPE_CHECK (&t, cad_subtract_box (&d, 99, (vector3) {0.0f, 0.0f, 0.0f},
                                         (vector3) {0.5f, 0.5f, 0.5f}) == cad_bad_index);
        /* Non-cube target (sphere added then subtracted -> bad_type). */
        MPE_CHECK (&t, cad_add_sphere (&d, 0.5f, (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        MPE_CHECK (&t, cad_subtract_box (&d, 1, (vector3) {0.0f, 0.0f, 0.0f},
                                         (vector3) {0.5f, 0.5f, 0.5f}) == cad_bad_type);
        /* Void region pokes out of target -> bad_dims. */
        MPE_CHECK (&t, cad_subtract_box (&d, 0, (vector3) {2.0f, 0.0f, 0.0f},
                                         (vector3) {0.5f, 0.5f, 0.5f}) == cad_bad_dims);
        MPE_CHECK (&t, cad_subtract_box (&d, 0, (vector3) {0.0f, 0.0f, 0.0f},
                                         (vector3) {2.0f, 0.5f, 0.5f}) == cad_bad_dims);
        /* Zero / negative void half-extents -> bad_dims. */
        MPE_CHECK (&t, cad_subtract_box (&d, 0, (vector3) {0.0f, 0.0f, 0.0f},
                                         (vector3) {0.0f, 0.5f, 0.5f}) == cad_bad_dims);
        MPE_CHECK (&t, cad_subtract_box (&d, 0, (vector3) {0.0f, 0.0f, 0.0f},
                                         (vector3) {-0.5f, 0.5f, 0.5f}) == cad_bad_dims);
        /* Design unchanged by refusals (count still 2). */
        MPE_CHECK (&t, d.count == 2);
    }

    /* ---- 4. subtract_sphere: inscribed cube envelope --------------- */
    {
        cad_design d;
        cad_design_init (&d);
        MPE_CHECK (&t, cad_add_cube (&d, (vector3) {1.0f, 1.0f, 1.0f},
                                     (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        /* Sphere r 0.5 -> inscribed cube half 0.5/sqrt(3) ~ 0.289
         * (disclosed inscribed-envelope: removes less than the sphere,
         * never more). The removed 0.577^3 cube fits centred in the
         * 2x2x2 target -> 6 slabs. */
        MPE_CHECK (&t, cad_subtract_sphere (&d, 0, (vector3) {0.0f, 0.0f, 0.0f},
                                            0.5f) == cad_ok);
        MPE_CHECK (&t, d.count == 6); /* inscribed cube == centred box void */
        MPE_CHECK (&t, cad_undo (&d) == cad_ok);
        MPE_CHECK (&t, d.count == 1);
        /* Negative radius -> bad_dims, design untouched. */
        MPE_CHECK (&t, cad_subtract_sphere (&d, 0, (vector3) {0.0f, 0.0f, 0.0f},
                                            -0.5f) == cad_bad_dims);
        MPE_CHECK (&t, d.count == 1);
    }

    /* ---- 5. subtract on void target refuses (cube target only) ----- */
    {
        cad_design d;
        cad_design_init (&d);
        MPE_CHECK (&t, cad_add_void_cube (&d, (vector3) {0.5f, 0.5f, 0.5f},
                                          (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        MPE_CHECK (&t, cad_subtract_box (&d, 0, (vector3) {0.0f, 0.0f, 0.0f},
                                         (vector3) {0.25f, 0.25f, 0.25f}) == cad_bad_type);
    }

    /* ---- 6. subtract fills target exactly -> target removed --------- */
    {
        cad_design d;
        cad_design_init (&d);
        MPE_CHECK (&t, cad_add_cube (&d, (vector3) {1.0f, 1.0f, 1.0f},
                                     (vector3) {0.0f, 0.0f, 0.0f}) == cad_ok);
        /* Void exactly equals target -> n == 0 -> target removed. */
        MPE_CHECK (&t, cad_subtract_box (&d, 0, (vector3) {0.0f, 0.0f, 0.0f},
                                         (vector3) {1.0f, 1.0f, 1.0f}) == cad_ok);
        MPE_CHECK (&t, d.count == 0); /* target removed entirely */
        MPE_CHECK (&t, cad_undo (&d) == cad_ok);
        MPE_CHECK (&t, d.count == 1); /* restored */
    }

    mpe_test_end (&t);
    return t.failures;
}

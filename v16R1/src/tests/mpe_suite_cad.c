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

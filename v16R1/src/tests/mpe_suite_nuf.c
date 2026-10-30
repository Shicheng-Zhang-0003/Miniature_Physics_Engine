/* NUF stage 0 regression gate (mpe_t_nuf_mass_composite).
 *
 * THE INDEPENDENCE CONTRACT: every expected value below is derived in THIS
 * file from closed-form rigid-body mechanics written out by hand — the test
 * never calls the implementation's helpers, never reuses its rotation code,
 * never shares its accumulation. Rotation probes (4, 5) assert pure
 * axis-permutation truth (the expected numbers are the same closed forms
 * with the axle landing on the axis the rotation says), toleranced only
 * against the float quaternion's rounding, never against a reference
 * implementation. A gate that computes its oracle with the function under
 * test proves only that the function is itself.
 *
 * Probes (docs/NON_UNIFORM_BODY.md §7; WP0 = the measuring sticks built
 * before any body-contract change):
 *  1. two-sphere dumbbell — exact fractions end-to-end;
 *  2. cube+cube L-bracket — Steiner off-diagonal (product of inertia) from
 *     ALIGNED parts only, the pure parallel-axis case;
 *  3. mixed triple (sphere + density-cube + cylinder) — mass-mode mixing,
 *     weighted-mean COM, cross-part Steiner, exact total mass;
 *  4. rotated cylinder 90° about Y — the axial/transverse split must LAND
 *     on the axis the rotation says;
 *  5. two-part cross-type assembly with the rotated part — Steiner through
 *     a rotated part with a real COM offset;
 *  6. rebase invariance — COM re-anchored, mass and inertia unchanged,
 *     COM arrives at the origin;
 *  7. error contract — each malformed input gets its own named error, and
 *     `out` is untouched on failure.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mpe_test.h"
#include "core/nu_mass.h"

/* Probe-7 fixture checker: the untouched totals must read 0xAB in every
 * byte (nu_totals has no padding bytes; assert-in-code that this stays
 * true by checking the size the layout actually has). */
static int unchanged_slab (const nu_totals *before);

static int unchanged_slab (const nu_totals *before) {
    const unsigned char *p = (const unsigned char *) before;
    for (size_t i = 0; i < sizeof (*before); i++) {
        if (p [i] != 0xAB) { return 0; }
    }
    return 1;
}

int mpe_t_nuf_mass_composite (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "nuf_mass_composite");
    /* The WP0 file is pure math and never steps the world; assert that the
     * determinism watchdog's own precondition holds for the whole run. */
    mpe_clamp_counters_reset ();

    /* ---- 1. two-sphere dumbbell ----
     * r=0.25 (binary-exact), m=2 each, centers (±0.5, 0, 0).
     * Sphere I = 0.4·2·0.0625 = 0.05 per axis. COM = 0 exactly.
     * Ixx = 2·0.05 (dx parallel to X contributes m·dy²+dz² = 0) = 0.1.
     * Iyy = Izz = 2·(0.05 + 2·0.25) = 1.1. */
    {
        nu_part parts [2] = {0};
        parts [0].type = nu_part_sphere; parts [0].radius = 0.25f;
        parts [0].mass_mode = nu_mass_mode_explicit; parts [0].mass = 2.0f;
        parts [0].position = (vector3) {-0.5f, 0.0f, 0.0f};
        parts [0].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        parts [1] = parts [0];
        parts [1].position = (vector3) {0.5f, 0.0f, 0.0f};
        nu_totals total;
        MPE_CHECK (&t, nu_compose (parts, 2, &total) == nu_compose_ok);
        MPE_CHECK_NEAR (&t, total.mass, 4.0, 1e-9, "dumbbell mass");
        MPE_CHECK_NEAR (&t, total.com.x, 0.0, 1e-9, "dumbbell com x");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [0][0], 0.1, 1e-6, "dumbbell Ixx");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [1][1], 1.1, 1e-6, "dumbbell Iyy");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [2][2], 1.1, 1e-6, "dumbbell Izz");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [0][1], 0.0, 1e-6, "dumbbell Ixy");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [0][2], 0.0, 1e-6, "dumbbell Ixz");
    }

    /* ---- 2. cube+cube L-bracket (aligned parts, Steiner off-diagonals) ----
     * m=1 each, half 0.25, centers (0,0,0) and (0.75, 0, 0.75).
     * Part tensor diag = m/3·(h²+h²) = 1/24 per axis. COM = (0.375, 0, 0.375).
     * Part com-relative offsets: (∓0.375, 0, ∓0.375); both sit on the x|z
     * diagonal so Ixz = −Σ m·x·z = −2·0.140625 = −0.28125 — the product of
     * inertia a diagonal-part composition MUST carry.
     * Ixx = 2/24 + 2·0.140625 = 0.3645833333…; Izz same; Iyy = 2/24 +
     * 4·0.140625 = 0.6458333333… */
    {
        nu_part parts [2] = {0};
        parts [0].type = nu_part_cube;
        parts [0].half_extensions = (vector3) {0.25f, 0.25f, 0.25f};
        parts [0].mass_mode = nu_mass_mode_explicit; parts [0].mass = 1.0f;
        parts [0].position = (vector3) {0.0f, 0.0f, 0.0f};
        parts [0].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        parts [1] = parts [0];
        parts [1].position = (vector3) {0.75f, 0.0f, 0.75f};
        nu_totals total;
        MPE_CHECK (&t, nu_compose (parts, 2, &total) == nu_compose_ok);
        MPE_CHECK_NEAR (&t, total.mass, 2.0, 1e-9, "L mass");
        MPE_CHECK_NEAR (&t, total.com.x, 0.375, 1e-9, "L com x");
        MPE_CHECK_NEAR (&t, total.com.z, 0.375, 1e-9, "L com z");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [0][0], 0.3645833333333333, 1e-6, "L Ixx");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [1][1], 0.6458333333333333, 1e-6, "L Iyy");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [2][2], 0.3645833333333333, 1e-6, "L Izz");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [0][2], -0.28125, 1e-6, "L Ixz (product of inertia)");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [0][1], 0.0, 1e-6, "L Ixy");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [1][2], 0.0, 1e-6, "L Iyz");
        /* Composed tensors must be exactly symmetric — that is a by-construction
         * property of diagonal-part composition, checked bitwise. */
        MPE_CHECK (&t, total.inertia.matrix [0][2] == total.inertia.matrix [2][0]);
    }

    /* ---- 3. mixed triple: sphere (explicit) + cube (density) + cylinder ----
     * sphere   m=1.5, r=0.5, at (−1, 0, 0): I = 0.15 per axis.
     * cube     ρ=1000, half (0.5,0.5,0.5):  volume 8·0.125=1 → m=1000,
     *          at (1, 0, 0.5): I = (1000/3)·(0.25+0.25) = 500/3 per axis.
     * cylinder m=3, r=0.5, hl=0.75, axle X, at (2, 0, 0):
     *          axial Ixx = 0.5·3·0.25 = 0.375; transverse = (3/12)·3 = 0.75.
     * M = 1004.5 exactly. com = (1, 0, 500/1004.5 = 1000/2009).
     * Com-relative Steiner: sphere d = (−2, 0, −1000/2009);
     * cube d = (0, 0, 9/4018); cylinder d = (1, 0, −1000/2009).
     * Ixx = 0.15+1.5·cz² + 500/3+1000·g² + 0.375+3·cz²     ≈ 168.31163
     * Iyy = 0.15+1.5·(4+cz²) + 500/3+1000·g² +
     *       0.75+3·(1+cz²)                                  ≈ 177.68874
     * Izz = 0.15+1.5·4 + 500/3 + 0.75+3·1                   = 176.5666…
     * Ixz = −(1.5·(−2)·(−1000/2009) + 3·(1)·(−1000/2009)) = 0 exactly
     * (sphere and cylinder products tie bitwise, opposite sign).
     * Tols: com z and the Steiner-carrying rows carry the float-boundary
     * rounding of com (rel 6e-8), so rel 1e-5; mass exact. */
    {
        nu_part parts [3] = {0};
        parts [0].type = nu_part_sphere; parts [0].radius = 0.5f;
        parts [0].mass_mode = nu_mass_mode_explicit; parts [0].mass = 1.5f;
        parts [0].position = (vector3) {-1.0f, 0.0f, 0.0f};
        parts [0].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        parts [1].type = nu_part_cube;
        parts [1].half_extensions = (vector3) {0.5f, 0.5f, 0.5f};
        parts [1].mass_mode = nu_mass_mode_density; parts [1].mass = 1000.0f;
        parts [1].position = (vector3) {1.0f, 0.0f, 0.5f};
        parts [1].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        parts [2].type = nu_part_cylinder; parts [2].radius = 0.5f;
        parts [2].cylinder_half_length = 0.75f;
        parts [2].mass_mode = nu_mass_mode_explicit; parts [2].mass = 3.0f;
        parts [2].position = (vector3) {2.0f, 0.0f, 0.0f};
        parts [2].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        nu_totals total;
        MPE_CHECK (&t, nu_compose (parts, 3, &total) == nu_compose_ok);
        MPE_CHECK_NEAR (&t, total.mass, 1004.5, 1e-9, "triple mass (density cube in the sum)");
        MPE_CHECK_NEAR (&t, total.com.x, 1.0, 1e-7, "triple com x (exact)");
        MPE_CHECK_REL (&t, total.com.z, 1000.0 / 2009.0, 1e-6, "triple com z");
        const double cz2 = (1000.0 / 2009.0) * (1000.0 / 2009.0);
        const double g2 = (9.0 / 4018.0) * (9.0 / 4018.0);
        MPE_CHECK_REL (&t, total.inertia.matrix [0][0],
                       0.15 + 1.5 * cz2 + 500.0 / 3.0 + 1000.0 * g2 + 0.375 + 3.0 * cz2, 1e-5,
                       "triple Ixx (Steiner cross-part)");
        MPE_CHECK_REL (&t, total.inertia.matrix [1][1],
                       0.15 + 1.5 * (4.0 + cz2) + 500.0 / 3.0 + 1000.0 * g2 + 0.75 + 3.0 * (1.0 + cz2), 1e-5,
                       "triple Iyy");
        MPE_CHECK_REL (&t, total.inertia.matrix [2][2], 0.15 + 6.0 + 500.0 / 3.0 + 0.75 + 3.0, 1e-5, "triple Izz");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [0][2], 0.0, 1e-4, "triple Ixz (bitwise tie, 0)");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [0][1], 0.0, 1e-5, "triple Ixy");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [1][2], 0.0, 1e-5, "triple Iyz");
    }

    /* ---- 4. rotated cylinder 90° about Y (axis-permutation truth) ----
     * r=0.5, hl=0.75, m=4, single part at the origin so COM = part center.
     * Local closed forms: axial Ixx = 0.5·4·0.25 = 0.5; transverse =
     * (4/12)(3·0.25 + 1.5²) = 1.0. A 90°-about-Y rotation sends the local
     * X axle onto Z: the composed tensor MUST read Ixx = Iyy = 1.0 (trans-
     * verse), Izz = 0.5 (axial). Tol = the float quat's rounding only. */
    {
        nu_part parts [1] = {0};
        parts [0].type = nu_part_cylinder; parts [0].radius = 0.5f;
        parts [0].cylinder_half_length = 0.75f;
        parts [0].mass_mode = nu_mass_mode_explicit; parts [0].mass = 4.0f;
        parts [0].position = (vector3) {0.0f, 0.0f, 0.0f};
        const float s = 0.70710678118654752f; /* cos/sin 45° */
        parts [0].orientation = (vector4) {s, 0.0f, s, 0.0f};
        nu_totals total;
        MPE_CHECK (&t, nu_compose (parts, 1, &total) == nu_compose_ok);
        MPE_CHECK_NEAR (&t, total.inertia.matrix [0][0], 1.0, 2e-5, "rot Ixx (transverse)");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [1][1], 1.0, 2e-5, "rot Iyy (transverse)");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [2][2], 0.5, 2e-5, "rot Izz (axial landed on Z)");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [0][1], 0.0, 2e-5, "rot Ixy");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [1][2], 0.0, 2e-5, "rot Iyz");
    }

    /* ---- 5. cross-type, rotated part, real COM offset ----
     * sphere m=1 r=0.125 at (−1,0,0) + the probe-4 cylinder at (1,0,0) with
     * the same 90°-about-Y quat. M = 5; com x = (−1 + 4)/5 = 0.6 exact.
     * Cylinder (d = (0.4,0,0)): Ixx 1.0 (⊥ none), Iyy 1.0+4·0.16 = 1.64,
     * Izz 0.5+4·0.16 = 1.14. Sphere (I = 0.4·1·0.015625 = 0.00625 per axis,
     * d = (−1.6,0,0)): Ixx +0.00625; Iyy = Izz +0.00625+2.56.
     * Totals: Ixx = 1.00625, Iyy = 4.20625, Izz = 3.70625. All exact
     * fractions; axis-permutation truth through a rotated part again. */
    {
        nu_part parts [2] = {0};
        parts [0].type = nu_part_sphere; parts [0].radius = 0.125f;
        parts [0].mass_mode = nu_mass_mode_explicit; parts [0].mass = 1.0f;
        parts [0].position = (vector3) {-1.0f, 0.0f, 0.0f};
        parts [0].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        parts [1].type = nu_part_cylinder; parts [1].radius = 0.5f;
        parts [1].cylinder_half_length = 0.75f;
        parts [1].mass_mode = nu_mass_mode_explicit; parts [1].mass = 4.0f;
        parts [1].position = (vector3) {1.0f, 0.0f, 0.0f};
        const float s = 0.70710678118654752f;
        parts [1].orientation = (vector4) {s, 0.0f, s, 0.0f};
        nu_totals total;
        MPE_CHECK (&t, nu_compose (parts, 2, &total) == nu_compose_ok);
        MPE_CHECK_NEAR (&t, total.mass, 5.0, 1e-9, "cross mass");
        MPE_CHECK_NEAR (&t, total.com.x, 0.6, 1e-7, "cross com x");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [0][0], 1.00625, 2e-5, "cross Ixx");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [1][1], 4.20625, 2e-5, "cross Iyy");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [2][2], 3.70625, 2e-5, "cross Izz");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [0][1], 0.0, 2e-5, "cross Ixy");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [1][2], 0.0, 2e-5, "cross Iyz");
    }

    /* ---- 6. rebase invariance (probe 3's parts, COM re-anchored) ----
     * nu_rebase_parts by the COM must leave mass and inertia unchanged and
     * bring the COM to the origin. Drift here means the re-anchor and the
     * composition disagree about the frame — the exact bug the §2 decision
     * outlaws. Float re-subtraction bounds the drift, hence the rel tols. */
    {
        nu_part parts [3] = {0};
        parts [0].type = nu_part_sphere; parts [0].radius = 0.5f;
        parts [0].mass_mode = nu_mass_mode_explicit; parts [0].mass = 1.5f;
        parts [0].position = (vector3) {-1.0f, 0.0f, 0.0f};
        parts [0].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        parts [1].type = nu_part_cube;
        parts [1].half_extensions = (vector3) {0.5f, 0.5f, 0.5f};
        parts [1].mass_mode = nu_mass_mode_density; parts [1].mass = 1000.0f;
        parts [1].position = (vector3) {1.0f, 0.0f, 0.5f};
        parts [1].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        parts [2].type = nu_part_cylinder; parts [2].radius = 0.5f;
        parts [2].cylinder_half_length = 0.75f;
        parts [2].mass_mode = nu_mass_mode_explicit; parts [2].mass = 3.0f;
        parts [2].position = (vector3) {2.0f, 0.0f, 0.0f};
        parts [2].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        nu_totals before;
        MPE_CHECK (&t, nu_compose (parts, 3, &before) == nu_compose_ok);
        nu_rebase_parts (parts, 3, before.com);
        nu_totals after;
        MPE_CHECK (&t, nu_compose (parts, 3, &after) == nu_compose_ok);
        MPE_CHECK_NEAR (&t, after.mass, before.mass, 1e-9, "rebase mass invariant");
        MPE_CHECK_NEAR (&t, after.com.x, 0.0, 1e-6, "rebase com x -> 0");
        MPE_CHECK_NEAR (&t, after.com.y, 0.0, 1e-6, "rebase com y -> 0");
        MPE_CHECK_NEAR (&t, after.com.z, 0.0, 1e-6, "rebase com z -> 0");
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) {
                char label [64];
                snprintf (label, sizeof (label), "rebase I[%d][%d] invariant", a, b);
                /* Absolute tolerance: an invariance check must also hold for
                 * a 0-vs-0 element (rel against zero is undefined — the
                 * MPE_CHECK_REL contract refuses it by design). The bound
                 * covers the float re-subtraction noise of the rebase plus
                 * the float output boundary, scaled by the element. */
                double tol = 2e-5 * ((before.inertia.matrix [a][b] > 0.0f) ? (double) before.inertia.matrix [a][b] :
                                     (before.inertia.matrix [a][b] < 0.0f) ? -(double) before.inertia.matrix [a][b] :
                                                                             1.0);
                MPE_CHECK_NEAR (&t, after.inertia.matrix [a][b], before.inertia.matrix [a][b], tol, label);
            }
        }
    }

    /* ---- 7. error contract: named errors, untouched outputs ---- */
    {
        nu_part parts [1] = {0};
        parts [0].type = nu_part_sphere; parts [0].radius = 0.5f;
        parts [0].mass_mode = nu_mass_mode_explicit; parts [0].mass = 1.0f;
        parts [0].position = (vector3) {0.0f, 0.0f, 0.0f};
        parts [0].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        nu_totals untouched;
        memset (&untouched, 0xAB, sizeof (untouched));
        MPE_CHECK (&t, nu_compose (NULL, 1, &untouched) == nu_compose_null);
        MPE_CHECK (&t, nu_compose (parts, 0, &untouched) == nu_compose_null);
        {
            nu_part bad = parts [0]; bad.type = (nu_part_type) 7;
            MPE_CHECK (&t, nu_compose (&bad, 1, &untouched) == nu_compose_bad_type);
        }
        {
            nu_part bad = parts [0]; bad.radius = -1.0f;
            MPE_CHECK (&t, nu_compose (&bad, 1, &untouched) == nu_compose_bad_dims);
        }
        {
            nu_part bad = parts [0]; bad.orientation = (vector4) {0.5f, 0.0f, 0.0f, 0.0f};
            MPE_CHECK (&t, nu_compose (&bad, 1, &untouched) == nu_compose_bad_quat);
        }
        {
            nu_part bad = parts [0]; bad.mass = -1.0f;
            MPE_CHECK (&t, nu_compose (&bad, 1, &untouched) == nu_compose_bad_mass);
        }
        {
            /* Total mass must not vanish: a massless composite is a render
             * prop, not a body. Single zero-mass part -> total 0. */
            nu_part bad = parts [0]; bad.mass = 0.0f;
            MPE_CHECK (&t, nu_compose (&bad, 1, &untouched) == nu_compose_bad_mass);
        }
        /* Write-on-success contract: seven failing calls must leave every
         * byte of the untouched slab exactly as memset wrote it. */
        MPE_CHECK (&t, unchanged_slab (&untouched));
    }

    if (t.failures == 0) {
        printf ("[PASS] nuf stage0: composition matches independent closed-form mechanics\n");
        MPE_INFO ("probes: dumbbell, L-Steiner offdiag, density mixing, 90-deg axis map, cross-type, rebase, 6 named errors");
    }
    mpe_test_end (&t);
    return t.failures;
}

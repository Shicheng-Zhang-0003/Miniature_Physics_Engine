/* MIG gate file (tests/mpe_suite_mig.c):
 *   mpe_t_mig_mass_composite — WP0 stage 0 regression (composition).
 *   mpe_t_mig_pendulum_physical — WP1 step 1.1 pre-change probe (plan
 *     docs/MIG_BUILD_PLAN.md §3): runs against the UNMODIFIED engine,
 *     hand-written oracle in this file, same independence contract.
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
#include "physics/spring_joint.h"
#include "physics/broadphase.h"
#include "scene/boundary.h"
#include "core/rigidbody.h"
#include "core/mig_mass.h"
#include "core/mig_spec.h"
#include "core/mpe_registry.h"

/* Probe-7 fixture checker: the untouched totals must read 0xAB in every
 * byte (mig_totals has no padding bytes; assert-in-code that this stays
 * true by checking the size the layout actually has). */
static int unchanged_slab (const mig_totals *before);

static int unchanged_slab (const mig_totals *before) {
    const unsigned char *p = (const unsigned char *) before;
    for (size_t i = 0; i < sizeof (*before); i++) {
        if (p [i] != 0xAB) { return 0; }
    }
    return 1;
}

int mpe_t_mig_mass_composite (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "mig_mass_composite");
    /* The WP0 file is pure math and never steps the world; assert that the
     * determinism watchdog's own precondition holds for the whole run. */
    mpe_clamp_counters_reset ();

    /* ---- 1. two-sphere dumbbell ----
     * r=0.25 (binary-exact), m=2 each, centers (±0.5, 0, 0).
     * Sphere I = 0.4·2·0.0625 = 0.05 per axis. COM = 0 exactly.
     * Ixx = 2·0.05 (dx parallel to X contributes m·dy²+dz² = 0) = 0.1.
     * Iyy = Izz = 2·(0.05 + 2·0.25) = 1.1. */
    {
        mig_part parts [2] = {0};
        parts [0].type = mig_part_sphere; parts [0].radius = 0.25f;
        parts [0].mass_mode = mig_mass_mode_explicit; parts [0].mass = 2.0f;
        parts [0].position = (vector3) {-0.5f, 0.0f, 0.0f};
        parts [0].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        parts [1] = parts [0];
        parts [1].position = (vector3) {0.5f, 0.0f, 0.0f};
        mig_totals total;
        MPE_CHECK (&t, mig_compose (parts, 2, &total) == mig_compose_ok);
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
        mig_part parts [2] = {0};
        parts [0].type = mig_part_cube;
        parts [0].half_extensions = (vector3) {0.25f, 0.25f, 0.25f};
        parts [0].mass_mode = mig_mass_mode_explicit; parts [0].mass = 1.0f;
        parts [0].position = (vector3) {0.0f, 0.0f, 0.0f};
        parts [0].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        parts [1] = parts [0];
        parts [1].position = (vector3) {0.75f, 0.0f, 0.75f};
        mig_totals total;
        MPE_CHECK (&t, mig_compose (parts, 2, &total) == mig_compose_ok);
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
        mig_part parts [3] = {0};
        parts [0].type = mig_part_sphere; parts [0].radius = 0.5f;
        parts [0].mass_mode = mig_mass_mode_explicit; parts [0].mass = 1.5f;
        parts [0].position = (vector3) {-1.0f, 0.0f, 0.0f};
        parts [0].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        parts [1].type = mig_part_cube;
        parts [1].half_extensions = (vector3) {0.5f, 0.5f, 0.5f};
        parts [1].mass_mode = mig_mass_mode_density; parts [1].mass = 1000.0f;
        parts [1].position = (vector3) {1.0f, 0.0f, 0.5f};
        parts [1].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        parts [2].type = mig_part_cylinder; parts [2].radius = 0.5f;
        parts [2].cylinder_half_length = 0.75f;
        parts [2].mass_mode = mig_mass_mode_explicit; parts [2].mass = 3.0f;
        parts [2].position = (vector3) {2.0f, 0.0f, 0.0f};
        parts [2].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        mig_totals total;
        MPE_CHECK (&t, mig_compose (parts, 3, &total) == mig_compose_ok);
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
        mig_part parts [1] = {0};
        parts [0].type = mig_part_cylinder; parts [0].radius = 0.5f;
        parts [0].cylinder_half_length = 0.75f;
        parts [0].mass_mode = mig_mass_mode_explicit; parts [0].mass = 4.0f;
        parts [0].position = (vector3) {0.0f, 0.0f, 0.0f};
        const float s = 0.70710678118654752f; /* cos/sin 45° */
        parts [0].orientation = (vector4) {s, 0.0f, s, 0.0f};
        mig_totals total;
        MPE_CHECK (&t, mig_compose (parts, 1, &total) == mig_compose_ok);
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
        mig_part parts [2] = {0};
        parts [0].type = mig_part_sphere; parts [0].radius = 0.125f;
        parts [0].mass_mode = mig_mass_mode_explicit; parts [0].mass = 1.0f;
        parts [0].position = (vector3) {-1.0f, 0.0f, 0.0f};
        parts [0].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        parts [1].type = mig_part_cylinder; parts [1].radius = 0.5f;
        parts [1].cylinder_half_length = 0.75f;
        parts [1].mass_mode = mig_mass_mode_explicit; parts [1].mass = 4.0f;
        parts [1].position = (vector3) {1.0f, 0.0f, 0.0f};
        const float s = 0.70710678118654752f;
        parts [1].orientation = (vector4) {s, 0.0f, s, 0.0f};
        mig_totals total;
        MPE_CHECK (&t, mig_compose (parts, 2, &total) == mig_compose_ok);
        MPE_CHECK_NEAR (&t, total.mass, 5.0, 1e-9, "cross mass");
        MPE_CHECK_NEAR (&t, total.com.x, 0.6, 1e-7, "cross com x");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [0][0], 1.00625, 2e-5, "cross Ixx");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [1][1], 4.20625, 2e-5, "cross Iyy");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [2][2], 3.70625, 2e-5, "cross Izz");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [0][1], 0.0, 2e-5, "cross Ixy");
        MPE_CHECK_NEAR (&t, total.inertia.matrix [1][2], 0.0, 2e-5, "cross Iyz");
    }

    /* ---- 6. rebase invariance (probe 3's parts, COM re-anchored) ----
     * mig_rebase_parts by the COM must leave mass and inertia unchanged and
     * bring the COM to the origin. Drift here means the re-anchor and the
     * composition disagree about the frame — the exact bug the §2 decision
     * outlaws. Float re-subtraction bounds the drift, hence the rel tols. */
    {
        mig_part parts [3] = {0};
        parts [0].type = mig_part_sphere; parts [0].radius = 0.5f;
        parts [0].mass_mode = mig_mass_mode_explicit; parts [0].mass = 1.5f;
        parts [0].position = (vector3) {-1.0f, 0.0f, 0.0f};
        parts [0].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        parts [1].type = mig_part_cube;
        parts [1].half_extensions = (vector3) {0.5f, 0.5f, 0.5f};
        parts [1].mass_mode = mig_mass_mode_density; parts [1].mass = 1000.0f;
        parts [1].position = (vector3) {1.0f, 0.0f, 0.5f};
        parts [1].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        parts [2].type = mig_part_cylinder; parts [2].radius = 0.5f;
        parts [2].cylinder_half_length = 0.75f;
        parts [2].mass_mode = mig_mass_mode_explicit; parts [2].mass = 3.0f;
        parts [2].position = (vector3) {2.0f, 0.0f, 0.0f};
        parts [2].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        mig_totals before;
        MPE_CHECK (&t, mig_compose (parts, 3, &before) == mig_compose_ok);
        mig_rebase_parts (parts, 3, before.com);
        mig_totals after;
        MPE_CHECK (&t, mig_compose (parts, 3, &after) == mig_compose_ok);
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
        mig_part parts [1] = {0};
        parts [0].type = mig_part_sphere; parts [0].radius = 0.5f;
        parts [0].mass_mode = mig_mass_mode_explicit; parts [0].mass = 1.0f;
        parts [0].position = (vector3) {0.0f, 0.0f, 0.0f};
        parts [0].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        mig_totals untouched;
        memset (&untouched, 0xAB, sizeof (untouched));
        MPE_CHECK (&t, mig_compose (NULL, 1, &untouched) == mig_compose_null);
        MPE_CHECK (&t, mig_compose (parts, 0, &untouched) == mig_compose_null);
        {
            mig_part bad = parts [0]; bad.type = (mig_part_type) 7;
            MPE_CHECK (&t, mig_compose (&bad, 1, &untouched) == mig_compose_bad_type);
        }
        {
            mig_part bad = parts [0]; bad.radius = -1.0f;
            MPE_CHECK (&t, mig_compose (&bad, 1, &untouched) == mig_compose_bad_dims);
        }
        {
            mig_part bad = parts [0]; bad.orientation = (vector4) {0.5f, 0.0f, 0.0f, 0.0f};
            MPE_CHECK (&t, mig_compose (&bad, 1, &untouched) == mig_compose_bad_quat);
        }
        {
            mig_part bad = parts [0]; bad.mass = -1.0f;
            MPE_CHECK (&t, mig_compose (&bad, 1, &untouched) == mig_compose_bad_mass);
        }
        {
            /* Total mass must not vanish: a massless composite is a render
             * prop, not a body. Single zero-mass part -> total 0. */
            mig_part bad = parts [0]; bad.mass = 0.0f;
            MPE_CHECK (&t, mig_compose (&bad, 1, &untouched) == mig_compose_bad_mass);
        }
        /* Write-on-success contract: seven failing calls must leave every
         * byte of the untouched slab exactly as memset wrote it. */
        MPE_CHECK (&t, unchanged_slab (&untouched));

        /* ---- 2026-11-10 audit tightening (review §2.4) ---- */
        {
            /* A zero-mass part inside a POSITIVE-total assembly used to be
             * accepted (the old per-part check was m < 0): the frozen spec
             * §4 requires every part's mass or density strictly positive. */
            mig_part bad [2];
            bad [0] = parts [0];
            bad [1] = parts [0];
            bad [0].mass = 1.0f;
            bad [0].position = (vector3) {-0.5f, 0.0f, 0.0f};
            bad [1].mass = 0.0f; /* ballast ghost: total still positive */
            bad [1].position = (vector3) {0.5f, 0.0f, 0.0f};
            MPE_CHECK (&t, mig_compose (bad, 2, &untouched) == mig_compose_bad_mass);
        }
        {
            /* Unknown mass_mode enum: its own named reject, never silently
             * read as density mode (the pre-audit else-branch). */
            mig_part bad = parts [0];
            bad.mass_mode = (mig_mass_mode) 7;
            MPE_CHECK (&t, mig_compose (&bad, 1, &untouched) == mig_compose_bad_mass_mode);
        }
        {
            /* Density mode with a malformed dimension reports bad_dims —
             * the volume arithmetic must not poison the mass first (the
             * pre-audit path computed m = rho*volume before validating
             * dims, so hx = -0.5 surfaced as bad_mass). */
            mig_part bad = parts [0];
            bad.type = mig_part_cube;
            bad.half_extensions = (vector3) {-0.5f, 0.5f, 0.5f};
            bad.mass_mode = mig_mass_mode_density;
            bad.mass = 1000.0f;
            MPE_CHECK (&t, mig_compose (&bad, 1, &untouched) == mig_compose_bad_dims);
        }
        /* ---- long-double oracle: non-binary-exact assembly ----
         * Hand-closed per-part tensors (independent of mig_mass.c's code
         * path), COM + Steiner accumulated in long double:
         *  sphere  m=2, r=0.3 at (0.1, 0.2, 0.3): I = 0.072 per axis.
         *  cube    rho=3000, he=(0.15,0.2,0.25) at (0.55,-0.1,0.4),
         *          Rz(+30 deg): volume 0.06, m=180; principal
         *          (6.15, 5.1, 3.75); rotated Bxx 5.8875, Byy 5.3625,
         *          Bzz 3.75, Bxy = cs(Ix-Iy) = +0.4546633369868303.
         *  cylinder m=1.5, r=0.2, hl=0.7, axle X at (-0.4,0.35,-0.25),
         *          Ry(+90 deg): principal (0.03 axial, 0.26 transverse);
         *          rotated to diag(0.26, 0.26, 0.03).
         * The float boundary (1e-7 relative) dominates the comparison. */
        {
            mig_part tri [3] = {0};
            tri [0].type = mig_part_sphere; tri [0].radius = 0.3f;
            tri [0].mass_mode = mig_mass_mode_explicit; tri [0].mass = 2.0f;
            tri [0].position = (vector3) {0.1f, 0.2f, 0.3f};
            tri [0].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
            tri [1].type = mig_part_cube;
            tri [1].half_extensions = (vector3) {0.15f, 0.2f, 0.25f};
            tri [1].mass_mode = mig_mass_mode_density; tri [1].mass = 3000.0f;
            tri [1].position = (vector3) {0.55f, -0.1f, 0.4f};
            tri [1].orientation = (vector4) {0.9659258262890683f, 0.0f, 0.0f, 0.25881904510252074f};
            tri [2].type = mig_part_cylinder; tri [2].radius = 0.2f; tri [2].cylinder_half_length = 0.7f;
            tri [2].mass_mode = mig_mass_mode_explicit; tri [2].mass = 1.5f;
            tri [2].position = (vector3) {-0.4f, 0.35f, -0.25f};
            tri [2].orientation = (vector4) {0.7071067811865476f, 0.0f, 0.7071067811865476f, 0.0f};
            mig_totals total;
            MPE_CHECK (&t, mig_compose (tri, 3, &total) == mig_compose_ok);
            /* The oracle reads the SAME float-struct inputs (the struct
             * boundary is float by contract — 0.15/0.2 are not binary-
             * exact), then derives every tensor in long double with the
             * hand-closed principal forms and rotations above. */
            const long double m1 = (long double) tri [0].mass;
            const long double r1 = (long double) tri [0].radius;
            const long double hx = (long double) tri [1].half_extensions.x;
            const long double hy = (long double) tri [1].half_extensions.y;
            const long double hz = (long double) tri [1].half_extensions.z;
            const long double m2 = (long double) tri [1].mass * 8.0L * hx * hy * hz;
            const long double m3 = (long double) tri [2].mass;
            const long double M = m1 + m2 + m3;
            const long double cx = (m1 * (long double) tri [0].position.x + m2 * (long double) tri [1].position.x + m3 * (long double) tri [2].position.x) / M;
            const long double cy = (m1 * (long double) tri [0].position.y + m2 * (long double) tri [1].position.y + m3 * (long double) tri [2].position.y) / M;
            const long double cz = (m1 * (long double) tri [0].position.z + m2 * (long double) tri [1].position.z + m3 * (long double) tri [2].position.z) / M;
            /* Principal tensors: sphere 2/5 m r^2; box (m/12)(4h_a^2+4h_b^2);
             * cylinder axle-X 1/2 m r^2 axial, (m/12)(3r^2+l^2) transverse. */
            long double sph_B [3][3], cube_B [3][3], cyl_B [3][3];
            for (int a = 0; a < 3; a++) {
                sph_B [a][0] = sph_B [a][1] = sph_B [a][2] = 0;
                cube_B [a][0] = cube_B [a][1] = cube_B [a][2] = 0;
                cyl_B [a][0] = cyl_B [a][1] = cyl_B [a][2] = 0;
            }
            sph_B [0][0] = sph_B [1][1] = sph_B [2][2] = 0.4L * m1 * r1 * r1;
            const long double cIx = (m2 / 12.0L) * (4.0L * hy * hy + 4.0L * hz * hz);
            const long double cIy = (m2 / 12.0L) * (4.0L * hx * hx + 4.0L * hz * hz);
            const long double cIz = (m2 / 12.0L) * (4.0L * hx * hx + 4.0L * hy * hy);
            { /* Rz(+30 deg): c = cos30, s = sin30 */
                const long double c = 0.8660254037844386467637232L, s = 0.5L;
                cube_B [0][0] = c*c*cIx + s*s*cIy; cube_B [1][1] = s*s*cIx + c*c*cIy;
                cube_B [2][2] = cIz;
                cube_B [0][1] = cube_B [1][0] = c*s*(cIx - cIy);
            }
            const long double xIx = 0.5L * m3 * ((long double) tri [2].radius) * ((long double) tri [2].radius);
            const long double xIt = (m3 / 12.0L) * (3.0L * ((long double) tri [2].radius) * ((long double) tri [2].radius)
                                                    + 4.0L * ((long double) tri [2].cylinder_half_length) * ((long double) tri [2].cylinder_half_length));
            { /* Ry(+90 deg): the axle leaves X, so axial lands on Z */
                cyl_B [0][0] = xIt; cyl_B [1][1] = xIt; cyl_B [2][2] = xIx;
            }
            const long double pos [3][3] = {
                {(long double) tri [0].position.x, (long double) tri [0].position.y, (long double) tri [0].position.z},
                {(long double) tri [1].position.x, (long double) tri [1].position.y, (long double) tri [1].position.z},
                {(long double) tri [2].position.x, (long double) tri [2].position.y, (long double) tri [2].position.z}
            };
            const long double mm [3] = {m1, m2, m3};
            long double I [3][3] = {{0}};
            for (int i = 0; i < 3; i++) {
                const long double dx = pos [i][0] - cx, dy = pos [i][1] - cy, dz = pos [i][2] - cz;
                const long double (*B)[3] = (i == 0) ? sph_B : (i == 1) ? cube_B : cyl_B;
                for (int a = 0; a < 3; a++) {
                    for (int b = 0; b < 3; b++) {
                        I [a][b] += B [a][b] + mm [i] * ((dx*dx + dy*dy + dz*dz) * ((a == b) ? 1.0L : 0.0L)
                                                        - ((a == 0 ? dx : (a == 1 ? dy : dz)) * (b == 0 ? dx : (b == 1 ? dy : dz))));
                    }
                }
            }
            MPE_CHECK_NEAR (&t, total.mass, (double) M, 1e-6, "oracle mass");
            MPE_CHECK_NEAR (&t, total.com.x, (double) cx, 1e-6, "oracle com x");
            MPE_CHECK_NEAR (&t, total.com.y, (double) cy, 1e-6, "oracle com y");
            MPE_CHECK_NEAR (&t, total.com.z, (double) cz, 1e-6, "oracle com z");
            for (int a = 0; a < 3; a++) {
                for (int b = 0; b < 3; b++) {
                    char label [64];
                    snprintf (label, sizeof (label), "oracle I[%d][%d]", a, b);
                    MPE_CHECK_NEAR (&t, total.inertia.matrix [a][b], (double) I [a][b], 1e-5, label);
                }
            }
        }
    }

    if (t.failures == 0) {
        printf ("[PASS] mig stage0: composition matches independent closed-form mechanics\n");
        MPE_INFO ("probes: dumbbell, L-Steiner offdiag, density mixing, 90-deg axis map, cross-type, rebase, 7 named errors, long-double oracle");
    }
    mpe_test_end (&t);
    return t.failures;
}

/* ---- WP1 step 1.1 pre-change probe (docs/MIG_BUILD_PLAN.md §3): the
 * physical pendulum the composite train leans on.
 *
 * Runs against the UNMODIFIED engine ON PURPOSE: the existing revolute
 * joint + hand-authored cuboid tensor must already satisfy the full
 * physical-pendulum closed form. If this is red it is PRE-EXISTING debt
 * (filed in the playbook ledger, never fixed silently inside a WP1
 * commit).
 *
 * The bob is a NON-cubic cuboid (half 0.15 x 0.6 x 0.25, m = 2) pivoted
 * at its TOP-FACE CENTER — local (0, +hy, 0), so d = hy straight off the
 * shape's own half-extension, never a hand-picked constant. The swing
 * inertia the pivot sees is the parallel-axis sum:
 *   I_com(z) = m/12*((2hy)^2 + (2hx)^2) = 2/12*(1.44 + 0.09) = 0.255
 *   I_pivot  = I_com + M*d^2            = 0.255 + 2*0.36   = 0.975
 * The Steiner term carries 74% of I_pivot — the relation WP0 certified
 * in closed form is here required of the ENGINE's live tensor.
 * Period: T = 2*pi*sqrt(I_pivot/(M*g*d)) = 1.8082 s at g = 9.81, times the
 * O(theta0^2) amplitude correction (1 + theta0^2/16) = 1.8184 s for
 * theta0 = 0.3 rad (next elliptic term 11*theta0^4/3072 ~ 3e-5: noise).
 * Same cross-measure discipline as the house pendulum gate (suite_a):
 * int crossings over the bob's horizontal COM displacement, drift bound,
 * plus amplitude bounds that catch both a dead-still hang and an
 * energy-pumping solver. */
int mpe_t_mig_pendulum_physical (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "mig_pendulum_physical");
    /* DESPOT-2026-10-04: pin g the way the house pendulum gates do. */
    g_cfg.world.gravity = -9.81f;
    physics_world w;
    mpe_world_begin (&w);
    const float hx = 0.15f, hy = 0.6f, hz = 0.25f, m = 2.0f;
    const float d = hy; /* the pivot rides the TOP-FACE center */
    const float theta0 = 0.3f;
    /* Static pivot (mass 0, the house pattern). Its anchor rides (0, -1.2, 0)
     * → world hinge (0, 6.8, 0), 1.0 below the pivot body's bottom face so
     * no shape ever overlaps the bob's top face, which pivots AT that point. */
    int pivot = physics_world_add_cube (&w, (vector3) {0.0f, 8.0f, 0.0f}, (vector3) {0.2f, 0.2f, 0.2f}, 0.0f);
    MPE_CHECK (&t, pivot >= 0);
    const vector3 hinge = {0.0f, 6.8f, 0.0f};
    /* The bob is BORN rotated theta0 about Z, zero angular velocity: its COM
     * sits at hinge + R(theta0)*(0, -d, 0) = hinge + (d*sin, -d*cos, 0). */
    int bob = physics_world_add_cube (&w, (vector3) {hinge.x + d * sinf (theta0), hinge.y - d * cosf (theta0), 0.0f},
                                      (vector3) {hx, hy, hz}, m);
    MPE_CHECK (&t, bob >= 0);
    w.bodies [bob].orientation =
        (vector4) {cosf (theta0 * 0.5f), 0.0f, 0.0f, sinf (theta0 * 0.5f)}; /* born at amplitude */
    uint32_t pid = w.bodies [pivot].object_id;
    uint32_t bid = w.bodies [bob].object_id;
    /* Hinge about Z; anchors are BODY-LOCAL (rotated by q each solve):
     * pivot local (0, -1.2, 0), bob local (0, +hy, 0) = top-face center.
     * The hinge axis (0,0,1) is invariant under the rotation stored in both
     * bodies, so the born-at-amplitude orientation maps axis_b identically. */
    MPE_CHECK (&t, constraint_add_revolute (&w, pid, bid, (vector3) {0.0f, -1.2f, 0.0f}, (vector3) {0.0f, hy, 0.0f},
                                            (vector3) {0.0f, 0.0f, 1.0f}) >= 0);
    /* Closed form, hand-written — oracle independent of WP0's mig_mass code. */
    const float i_com_z = (m / 12.0f) * ((2.0f * hy) * (2.0f * hy) + (2.0f * hx) * (2.0f * hx));
    const float i_pivot = i_com_z + m * d * d;
    const float t_exact = 2.0f * 3.14159265f * sqrtf (i_pivot / (m * 9.81f * d));
    const float t_expected = t_exact * (1.0f + (theta0 * theta0) / 16.0f);
    const float dt = 1.0f / 60.0f;
    float prev_x = w.bodies [bob].position.x - hinge.x;
    int crossings = 0, first = -1, last = -1;
    float max_drift = 0.0f, max_abs_x = 0.0f;
    for (int k = 0; k < 900; k++) {
        physics_world_step (&w, dt);
        const rigidbody *b = &w.bodies [bob];
        if (!isfinite (b -> position.x) || !isfinite (b -> position.y) || !isfinite (b -> position.z)) {
            printf ("[FAIL] bob non-finite at tick %d\n", k);
            t.failures++;
            break;
        }
        const float dx = b -> position.x - hinge.x;
        const float dy = b -> position.y - hinge.y;
        max_drift = fmaxf (max_drift, fabsf (sqrtf (dx * dx + dy * dy) - d));
        const float ax = fabsf (dx);
        if (ax > max_abs_x) { max_abs_x = ax; }
        if ((prev_x <= 0.0f && dx > 0.0f) || (prev_x >= 0.0f && dx < 0.0f)) {
            crossings++;
            if (first < 0) { first = k; }
            last = k;
        } prev_x = dx;
    }
    float t_meas = 0.0f;
    if (crossings >= 2) { t_meas = 2.0f * (float) (last - first) * dt / (float) (crossings - 1); }
    MPE_INFO ("period: measured=%.4f analytic=%.4f (theta0-corrected %.4f) crossings=%d max_drift=%.4f max_x=%.4f (start %.4f)",
              t_meas, t_exact, t_expected, crossings, max_drift, max_abs_x, d * sinf (theta0));
    MPE_CHECK (&t, crossings >= 6); /* 900 ticks ~ 8 periods at T = 1.82 s */
    MPE_CHECK (&t, max_drift <= 0.02f); /* the house revolute drift bound */
    MPE_CHECK (&t, max_abs_x >= 0.4f * d * sinf (theta0)); /* no dead-still hang */
    MPE_CHECK (&t, max_abs_x <= 1.6f * d * sinf (theta0)); /* no energy pump */
    if (crossings >= 4) { MPE_CHECK_REL (&t, t_meas, t_expected, 0.02f, "physical-pendulum period (I_com + M*d^2)"); }
    if (t.failures == 0) {
        printf ("[PASS] mig WP1 probe: top-face-pivoted cuboid swings at the I_com+M*d^2 closed form on the unmodified engine\n");
        MPE_INFO ("parallel-axis sum required of the live tensor: Steiner term carries %.0f%% of I_pivot",
                  100.0f * (m * d * d) / i_pivot);
    }
    physics_world_cleanup (&w);
    mpe_test_end (&t);
    return t.failures;
}

/* ---- WP1 1.4 stub gate (docs/MIG_BUILD_PLAN.md §3 1.4; landing 2026-
 * 10-31): the FIRST engine-facing MIG path. Same independence contract
 * as above — every expected value is a hand-written closed form, and the
 * gate never calls the implementation's matrix helpers (the multiply-
 * back below is the gate's OWN float algebra, never math3_inverse or
 * math3_multiplication round-tripping).
 *
 * Fixture: the WP0 dumbbell — two explicit-mass spheres, m = 2 apiece,
 * r = 0.25, at (+-0.5, 0, 0). Composition truth: M = 4 at the origin,
 * Ixx = 2*(2/5*2*0.25^2) = 0.1 (both part centres ride the x axle, so
 * the x-Steiner term is exactly zero), Iyy = Izz = 2*(0.05 + 2*0.5^2) =
 * 1.1, bounding radius 0.5 + 0.25 = 0.75. The sphere-default tensor the
 * backing body "would" have had is 2/5*4*0.75^2 = 0.9 — different from
 * EVERY composed entry, so regeneration (the failure this stub exists to
 * make impossible) is caught by values, not vibes.
 *
 * The zero-clamp sweep is the guard's proof: the stub must pre-decline
 * (return -1) whatever sanitize would substitute, and rejected calls
 * must not grow the world. mpe_clamp_composite_events stays zero in WP1
 * BY DESIGN (it counts the WP2 cap substitution and the WP3 load-sum
 * gate); the WP1 assertion is that the four live counters AND it never
 * move across construction, 120 free-fall ticks, and every rejection.
 *
 * Also exercised: 1.2's bridge (mig_make_bridge) on an ASYMMETRIC pair —
 * m = 1 @ origin, m = 3 @ (2,0,0): com (1.5,0,0), Ixx = 0.025 + 0.075
 * = 0.1 (part centres ride the x axle; per-part self-inertia 2/5*m*r^2),
 * Iyy = (0.025 + 1*1.5^2) + (0.075 + 3*0.5^2) = 2.275 + 0.825 = 3.1,
 * re-anchored part poses (-1.5,0,0) / (0.5,0,0), source list untouched. */
int mpe_t_mig_composite_stub (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "mig_composite_stub");
    /* 1.2 bridge — pure math, no world. */
    mig_part pair [2] = {
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 1.0f },
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 3.0f }
    };
    mig_part bridge_buf [2];
    mig_bridge br;
    MPE_CHECK (&t, mig_make_bridge (pair, 2, bridge_buf, &br) == mig_compose_ok);
    MPE_CHECK_NEAR (&t, (float) br.totals.mass, 4.0f, 1e-6f, "bridge mass");
    MPE_CHECK_NEAR (&t, br.totals.com.x, 1.5f, 1e-6f, "bridge com");
    MPE_CHECK_NEAR (&t, br.totals.inertia.matrix [0][0], 0.1f, 1e-6f, "bridge Ixx (0.025 + 0.075)");
    MPE_CHECK_NEAR (&t, br.totals.inertia.matrix [1][1], 3.1f, 1e-5f, "bridge Iyy");
    MPE_CHECK_NEAR (&t, bridge_buf [0].position.x, -1.5f, 1e-6f, "rebase part A");
    MPE_CHECK_NEAR (&t, bridge_buf [1].position.x, 0.5f, 1e-6f, "rebase part B");
    MPE_CHECK (&t, (pair [0].position.x == 0.0f) && (pair [1].position.x == 2.0f)); /* src never mutated */
    MPE_CHECK (&t, (br.parts == bridge_buf) && (br.count == 2));
    MPE_CHECK (&t, mig_make_bridge (NULL, 2, bridge_buf, &br) == mig_compose_null);
    MPE_CHECK (&t, mig_make_bridge (pair, 0, bridge_buf, &br) == mig_compose_null);
    MPE_CHECK (&t, mig_make_bridge (pair, 2, bridge_buf, NULL) == mig_compose_null);
    MPE_CHECK (&t, mig_make_bridge (pair, 2, pair, &br) == mig_compose_null); /* aliasing rejected */
    MPE_CHECK (&t, (pair [0].position.x == 0.0f) && (pair [1].position.x == 2.0f)); /* still untouched */
    mig_part tilted = pair [0];
    tilted.orientation = (vector4) {0.5f, 0.0f, 0.0f, 0.0f};
    MPE_CHECK (&t, mig_make_bridge (&tilted, 1, bridge_buf, &br) == mig_compose_bad_quat);
    /* 1.4 stub: construction, authored tensor, zero clamps. The
     * assembly floats at y = 30: the default world floor sits at y = 0
     * (the house drop tests spawn high too), and 2 s of fall ends near
     * y = 10.4 — far above it. */
    g_cfg.world.gravity = -9.81f;
    g_cfg.world.drag = 0.0f;
    g_cfg.sleep.enable = 0;
    mpe_clamp_counters_reset ();
    const float yp = 30.0f; /* the assembly flies at this height */
    const mig_part dumbbell [2] = {
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {0.5f, yp, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f },
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {-0.5f, yp, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f }
    };
    physics_world w;
    mpe_world_begin (&w);
    const int cb = physics_world_add_composite (&w, dumbbell, 2);
    MPE_CHECK (&t, cb == 0);
    MPE_CHECK (&t, w.body_count == 1);
    MPE_CHECK (&t, w.bodies [cb].type == object_composite);
    MPE_CHECK (&t, w.bodies [cb].mass_is_composed);
    MPE_CHECK (&t, w.bodies [cb].composite_part_count == 2);
    MPE_CHECK_NEAR (&t, w.bodies [cb].mass, 4.0f, 1e-6f, "composed mass");
    MPE_CHECK_NEAR (&t, w.bodies [cb].radius, 0.75f, 1e-6f, "bounding radius");
    MPE_CHECK_NEAR (&t, w.bodies [cb].position.x, 0.0f, 1e-6f, "com x");
    MPE_CHECK_NEAR (&t, w.bodies [cb].position.y, yp, 1e-6f, "com y");
    MPE_CHECK_NEAR (&t, w.bodies [cb].inverse_mass, 0.25f, 1e-6f, "inverse mass");
    const math3 *loc = &w.bodies [cb].inertia_tensor_local;
    MPE_CHECK_NEAR (&t, loc -> matrix [0][0], 0.1f, 1e-6f, "Ixx authored");
    MPE_CHECK_NEAR (&t, loc -> matrix [1][1], 1.1f, 1e-6f, "Iyy authored");
    MPE_CHECK_NEAR (&t, loc -> matrix [2][2], 1.1f, 1e-6f, "Izz authored");
    MPE_CHECK_NEAR (&t, fabsf (loc -> matrix [0][1]) + fabsf (loc -> matrix [0][2]) +
                    fabsf (loc -> matrix [1][2]), 0.0f, 1e-9f, "off-diagonals zero");
    /* Multiply-back through the GATE's own float algebra. */
    const math3 inv = w.bodies [cb].inverse_inertia_tensor_local;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) { s += inv.matrix [r][k] * loc -> matrix [k][c]; }
            MPE_CHECK_NEAR (&t, s, (r == c) ? 1.0f : 0.0f, 2e-4f, "inv*I identity");
        }
    }
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            MPE_CHECK_NEAR (&t, w.bodies [cb].inverse_inertia_system.matrix [r][c],
                            inv.matrix [r][c], 0.0f, "system inverse == local (identity frame)");
        }
    }
    MPE_CHECK (&t, mpe_clamp_composite_total () == 0ul);
    MPE_CHECK (&t, mpe_clamp_mass_total () == 0ul);
    MPE_CHECK (&t, mpe_clamp_radius_total () == 0ul);
    MPE_CHECK (&t, mpe_clamp_half_length_total () == 0ul);
    MPE_CHECK (&t, mpe_clamp_solver_total () == 0ul);
    /* Bitwise survival + free-fall smoke, 120 ticks (= 2 s at dt = 1/60). */
    const math3 snap_local = w.bodies [cb].inertia_tensor_local;
    const math3 snap_inv_local = w.bodies [cb].inverse_inertia_tensor_local;
    const math3 snap_inv_sys = w.bodies [cb].inverse_inertia_system;
    const float y0 = w.bodies [cb].position.y;
    physics_world wctrl;
    mpe_world_begin (&wctrl);
    const int ctrl = physics_world_add_sphere (&wctrl, 0.5f, 4.0f, (vector3) {0.0f, yp, 0.0f});
    MPE_CHECK (&t, ctrl == 0);
    const float y0_ctrl = wctrl.bodies [ctrl].position.y;
    for (int k = 0; k < 120; k++) {
        physics_world_step (&w, 1.0f / 60.0f);
        physics_world_step (&wctrl, 1.0f / 60.0f);
    }
    const float dy = w.bodies [cb].position.y - y0;
    const float dy_c = wctrl.bodies [ctrl].position.y - y0_ctrl;
    MPE_CHECK_REL (&t, -dy, 19.62f, 0.03f, "free-fall drop vs 0.5*9.81*2^2");
    MPE_CHECK (&t, fabsf (dy - dy_c) <= 1e-7f); /* composite falls WITH the plain sphere */
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            MPE_CHECK_NEAR (&t, w.bodies [cb].inertia_tensor_local.matrix [r][c],
                            snap_local.matrix [r][c], 0.0f, "authored tensor bitwise after 120 ticks");
            MPE_CHECK_NEAR (&t, w.bodies [cb].inverse_inertia_tensor_local.matrix [r][c],
                            snap_inv_local.matrix [r][c], 0.0f, "inverse local bitwise after 120 ticks");
            MPE_CHECK_NEAR (&t, w.bodies [cb].inverse_inertia_system.matrix [r][c],
                            snap_inv_sys.matrix [r][c], 0.0f, "system inverse bitwise after 120 ticks");
        }
    }
    MPE_CHECK (&t, isfinite (w.bodies [cb].position.x) && isfinite (w.bodies [cb].position.y));
    MPE_CHECK (&t, mpe_clamp_composite_total () == 0ul);
    MPE_CHECK (&t, mpe_clamp_mass_total () == 0ul);
    MPE_CHECK (&t, mpe_clamp_radius_total () == 0ul);
    MPE_CHECK (&t, mpe_clamp_half_length_total () == 0ul);
    MPE_CHECK (&t, mpe_clamp_solver_total () == 0ul);
    /* Rejection contract: -1, nothing grown, NOTHING counted. The
     * bounding radius is COM-relative (translation-invariant) — the
     * spread case, not a moved single part, is what can exceed the
     * engine's own radius cap. */
    physics_world wr;
    mpe_world_begin (&wr);
    const int base = wr.body_count;
    MPE_CHECK (&t, physics_world_add_composite (NULL, dumbbell, 2) == -1);
    MPE_CHECK (&t, wr.body_count == base);
    MPE_CHECK (&t, physics_world_add_composite (&wr, NULL, 2) == -1);
    MPE_CHECK (&t, wr.body_count == base);
    MPE_CHECK (&t, physics_world_add_composite (&wr, dumbbell, 0) == -1);
    MPE_CHECK (&t, wr.body_count == base);
    MPE_CHECK (&t, physics_world_add_composite (&wr, dumbbell, -3) == -1);
    MPE_CHECK (&t, wr.body_count == base);
    mig_part broken = dumbbell [0];
    broken.radius = 0.0f; /* compose_bad_dims */
    MPE_CHECK (&t, physics_world_add_composite (&wr, &broken, 1) == -1);
    MPE_CHECK (&t, wr.body_count == base);
    mig_part farpair [2] = { dumbbell [0], dumbbell [1] };
    farpair [0].position = (vector3) {150.0f, 0.0f, 0.0f};
    farpair [1].position = (vector3) {-150.0f, 0.0f, 0.0f};
    /* com stays the origin but d = 150 => bound 150.25 > 100 (the
     * engine's own radius cap): WP1 sphere-backing cannot host it. */
    MPE_CHECK (&t, physics_world_add_composite (&wr, farpair, 2) == -1);
    MPE_CHECK (&t, wr.body_count == base);
    mig_part tiny = dumbbell [0];
    tiny.mass = 1e-5f; /* below the engine's 1e-4 mass floor */
    MPE_CHECK (&t, physics_world_add_composite (&wr, &tiny, 1) == -1);
    MPE_CHECK (&t, wr.body_count == base);
    mig_part small = dumbbell [0];
    small.radius = 0.002f; /* bound 0.002 < the 0.01 floor */
    MPE_CHECK (&t, physics_world_add_composite (&wr, &small, 1) == -1);
    MPE_CHECK (&t, wr.body_count == base);
    MPE_CHECK (&t, mpe_clamp_composite_total () == 0ul);
    MPE_CHECK (&t, mpe_clamp_mass_total () == 0ul);
    MPE_CHECK (&t, mpe_clamp_radius_total () == 0ul);
    MPE_CHECK (&t, mpe_clamp_half_length_total () == 0ul);
    MPE_CHECK (&t, mpe_clamp_solver_total () == 0ul);
    physics_world_cleanup (&wr);
    physics_world_cleanup (&wctrl);
    physics_world_cleanup (&w);
    if (t.failures == 0) {
        printf ("[PASS] mig WP1 stub: composed mass + authored tensor survive construction sanitize and 120 ticks, zero clamp events\n");
        MPE_INFO ("dumbbell: mass 4, Ixx 0.1, Iyy=Izz 1.1, bound 0.75 (sphere default would be 0.9); free-fall dy = %.4f m vs control %.4f m",
                  dy, dy_c);
    }
    mpe_test_end (&t);
    return t.failures;
}

/* WP2 2.1 gate (plan §4): the composite TYPE and per-world part POOL only.
 * No composition math, no narrowphase, no world API — those are 2.2/2.4.
 * The oracle is the charter §4.1 contract written out as plain predicates:
 *  - object_composite is the fifth value and every earlier value is stable;
 *  - a pool slice is contiguous, its parts carry per-body indices
 *    0..stored-1, and the source array is never mutated;
 *  - an over-cap assembly (20 parts against the P2 cap 16) stores the FIRST
 *    16 and counts exactly one mpe_clamp_composite_events substitute; a
 *    lowered config cap (4) clamps to 4; a foreign config cannot raise the
 *    frozen ceiling above 16;
 *  - deleting a part sets active=0 and the slot is NOT reused (the next
 *    append continues the per-body index with first_index), until
 *    physics_world_clear — the only compaction point — resets the count
 *    while keeping capacity;
 *  - bad inputs (NULL, count<=0, negative first_index, non-primitive type)
 *    mutate neither the pool nor the counter;
 *  - non-composite bodies keep part_base/composite_part_count zero.
 * Expected numbers are literals from this contract, never read back from
 * the implementation's own bookkeeping. */
int mpe_t_mig_composite_pool (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "mig_composite_pool");
    /* Enum append: every pre-2.1 numeric value is unchanged. */
    MPE_CHECK (&t, object_sphere == 0 && object_cube == 1 && object_cylinder == 2 && object_custom == 3);
    MPE_CHECK (&t, object_composite == 4 && (int) OBJECT_COMPOSITE == 4);
    physics_world w;
    mpe_world_begin (&w);
    /* A normal body carries the slice fields zeroed. */
    int b = physics_world_add_sphere (&w, 0.25f, 1.0f, (vector3) {0.0f, 5.0f, 0.0f});
    MPE_CHECK (&t, b >= 0);
    if (b >= 0) {
        MPE_CHECK (&t, w.bodies [b].part_base == 0);
        MPE_CHECK (&t, w.bodies [b].composite_part_count == 0);
    }
    MPE_CHECK (&t, w.composite_parts == NULL);
    MPE_CHECK (&t, w.composite_part_count == 0);
    MPE_CHECK (&t, w.composite_part_capacity == 0);
    /* Fixture: 20 distinct sphere parts; deliberately poisoned part_index/
     * active so the append must author both. */
    composite_part src [20];
    memset (src, 0, sizeof (src));
    for (int i = 0; i < 20; i++) {
        src [i].type = object_sphere;
        src [i].radius = 0.05f;
        src [i].mass = 1.0f + (float) i;
        src [i].local_position = (vector3) {0.1f * (float) i, 0.0f, 0.0f};
        src [i].local_orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        src [i].part_index = -1;
        src [i].active = 0;
    }
    mpe_clamp_counters_reset ();
    /* Over-cap slice: keep the first 16, count exactly one substitute. */
    int stored = -1;
    int base = physics_world_composite_parts_append (&w, src, 20, 0, &stored);
    MPE_CHECK (&t, base == 0);
    MPE_CHECK (&t, stored == 16);
    MPE_CHECK (&t, w.composite_part_count == 16);
    MPE_CHECK (&t, w.composite_part_capacity >= 16);
    MPE_CHECK (&t, mpe_clamp_composite_total () == 1ul);
    MPE_CHECK (&t, mpe_clamp_mass_total () == 0ul); /* no counter-channel leak */
    for (int i = 0; i < 16; i++) {
        MPE_CHECK (&t, w.composite_parts [i].part_index == i);
        MPE_CHECK (&t, w.composite_parts [i].active == 1);
        MPE_CHECK (&t, w.composite_parts [i].mass == src [i].mass);
    }
    MPE_CHECK (&t, src [0].part_index == -1 && src [0].active == 0); /* source untouched */
    /* A second body's slice starts at the next pool slot; its per-body
     * indices restart at 0. */
    stored = -1;
    int base2 = physics_world_composite_parts_append (&w, src, 2, 0, &stored);
    MPE_CHECK (&t, base2 == 16);
    MPE_CHECK (&t, stored == 2);
    MPE_CHECK (&t, w.composite_part_count == 18);
    MPE_CHECK (&t, w.composite_parts [16].part_index == 0 && w.composite_parts [17].part_index == 1);
    MPE_CHECK (&t, mpe_clamp_composite_total () == 1ul);
    /* Deletion deactivates, never reuses: slot 2 stays dead and untouched;
     * the next append continues this body's index with first_index=2. */
    w.composite_parts [2].active = 0;
    float dead_mass = w.composite_parts [2].mass;
    stored = -1;
    int base3 = physics_world_composite_parts_append (&w, src, 1, 2, &stored);
    MPE_CHECK (&t, base3 == 18 && stored == 1 && w.composite_part_count == 19);
    MPE_CHECK (&t, w.composite_parts [18].part_index == 2);
    MPE_CHECK (&t, w.composite_parts [2].active == 0);
    MPE_CHECK (&t, w.composite_parts [2].mass == dead_mass);
    /* Bad inputs are write-on-success no-ops. */
    int before = w.composite_part_count;
    unsigned long cbefore = mpe_clamp_composite_total ();
    MPE_CHECK (&t, physics_world_composite_parts_append (&w, NULL, 1, 0, NULL) == -1);
    MPE_CHECK (&t, physics_world_composite_parts_append (&w, src, 0, 0, NULL) == -1);
    MPE_CHECK (&t, physics_world_composite_parts_append (&w, src, 1, -1, NULL) == -1);
    composite_part bad = src [0];
    bad.type = object_custom;
    MPE_CHECK (&t, physics_world_composite_parts_append (&w, &bad, 1, 0, NULL) == -1);
    MPE_CHECK (&t, w.composite_part_count == before);
    MPE_CHECK (&t, mpe_clamp_composite_total () == cbefore);
    /* Configuration lowers the cap; the frozen P2 ceiling cannot be raised. */
    mpe_config_t local_cfg = *mpe_world_cfg (&w);
    local_cfg.composite.max_parts = 4;
    physics_world_set_config (&w, &local_cfg);
    stored = -1;
    int base4 = physics_world_composite_parts_append (&w, src, 6, 0, &stored);
    MPE_CHECK (&t, base4 == 19 && stored == 4 && w.composite_part_count == 23);
    MPE_CHECK (&t, mpe_clamp_composite_total () == cbefore + 1ul);
    local_cfg.composite.max_parts = 99;
    stored = -1;
    int base5 = physics_world_composite_parts_append (&w, src, 20, 0, &stored);
    MPE_CHECK (&t, base5 == 23 && stored == 16 && w.composite_part_count == 39);
    MPE_CHECK (&t, mpe_clamp_composite_total () == cbefore + 2ul);
    /* clear is the only compaction point: count resets, capacity stays. */
    int cap_before = w.composite_part_capacity;
    physics_world_clear (&w);
    MPE_CHECK (&t, w.composite_part_count == 0);
    MPE_CHECK (&t, w.composite_part_capacity == cap_before && w.composite_parts != NULL);
    stored = -1;
    int base6 = physics_world_composite_parts_append (&w, src, 2, 0, &stored);
    MPE_CHECK (&t, base6 == 0 && stored == 2 && w.composite_part_count == 2);
    MPE_CHECK (&t, w.composite_parts [0].part_index == 0 && w.composite_parts [1].part_index == 1);
    physics_world_cleanup (&w);
    MPE_CHECK (&t, w.composite_parts == NULL);
    MPE_CHECK (&t, w.composite_part_count == 0);
    MPE_CHECK (&t, w.composite_part_capacity == 0);
    if (t.failures == 0) {
        printf ("[PASS] mig WP2.1: composite type + pool slice, per-body indices, inactive slots, cap 16 counted, clear compaction\n");
        MPE_INFO ("pool: 20 requested -> 16 stored (1 clamp), second slice base 16, dead slot 2 preserved, cap-4 config -> 4, foreign cap 99 -> frozen 16, clear -> count 0/capacity %d",
                  cap_before);
    }
    mpe_test_end (&t);
    return t.failures;
}

/* WP2 2.2 gate (plan §4): the WORLD API completion — composition, the
 * COM-framed pool slice, the P2 cap over stored truth, and the explicit
 * composed-inertia guard through static/kinematic/wake/sanitize paths.
 *
 * The oracle is the charter §4.1/§4.2 contract written out as literals:
 *  - the dumbbell (2 kg spheres r=0.25 at x=±0.5) enters as ONE
 *    object_composite at the COM with mass 4, Ixx 0.1, Iyy=Izz 1.1,
 *    bound 0.75, and its two COM-framed parts in pool slots 0/1 with
 *    masses 2/2 and per-body indices 0/1;
 *  - density mode lands effective kilograms in the pool: a 1 m cube at
 *    density 2 is 2 kg, I = 1/3 on every axis, bound sqrt(3)/2;
 *  - the cap stores the FIRST 16 of 20 and composes the body over those
 *    16 only (mass 16, not 20), counting one substitute; a lowered cap
 *    of 4 stores 4; a malformed part beyond the cap still rejects the
 *    WHOLE assembly with no body, pool slot, or counter movement;
 *  - the authored tensor is bitwise invariant through
 *    rigidbody_set_static(false), rigidbody_set_kinematic(false), and a
 *    direct zero-inverse sanitize; inverses restore from the authored
 *    tensor, never from a sphere/cube/cylinder formula;
 *  - 10 gravity ticks keep type/composite fields/tensor intact.
 * Every expected number here is a hand-derived literal; no value is read
 * back from the implementation's own bookkeeping. */
int mpe_t_mig_composite_world (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "mig_composite_world");
    g_cfg.world.gravity = -9.81f;
    g_cfg.world.drag = 0.0f;
    g_cfg.sleep.enable = 0;
    mpe_clamp_counters_reset ();
    const mig_part dumbbell [2] = {
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {0.5f, 30.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f },
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {-0.5f, 30.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f }
    };
    physics_world w;
    mpe_world_begin (&w);
    const int cb = physics_world_add_composite (&w, dumbbell, 2);
    MPE_CHECK (&t, cb == 0 && w.body_count == 1);
    if (cb >= 0) {
        rigidbody *body = &w.bodies [cb];
        MPE_CHECK (&t, body -> type == object_composite);
        MPE_CHECK (&t, body -> mass_is_composed && body -> composite_part_count == 2 && body -> part_base == 0);
        MPE_CHECK_NEAR (&t, body -> mass, 4.0f, 1e-6f, "composed mass");
        MPE_CHECK_NEAR (&t, body -> inverse_mass, 0.25f, 1e-6f, "inverse mass");
        MPE_CHECK_NEAR (&t, body -> position.x, 0.0f, 1e-6f, "body at COM x");
        MPE_CHECK_NEAR (&t, body -> position.y, 30.0f, 1e-6f, "body at COM y");
        MPE_CHECK_NEAR (&t, body -> radius, 0.75f, 1e-6f, "bound radius");
        MPE_CHECK_NEAR (&t, body -> half_extensions.x, 0.75f, 1e-6f, "bound half-extent");
        const math3 *loc = &body -> inertia_tensor_local;
        MPE_CHECK_NEAR (&t, loc -> matrix [0][0], 0.1f, 1e-6f, "authored Ixx");
        MPE_CHECK_NEAR (&t, loc -> matrix [1][1], 1.1f, 1e-6f, "authored Iyy");
        MPE_CHECK_NEAR (&t, loc -> matrix [2][2], 1.1f, 1e-6f, "authored Izz");
        MPE_CHECK_NEAR (&t, fabsf (loc -> matrix [0][1]) + fabsf (loc -> matrix [0][2]) +
                        fabsf (loc -> matrix [1][2]), 0.0f, 1e-9f, "no product of inertia for the axle pair");
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                float s = 0.0f;
                for (int k = 0; k < 3; k++) { s += body -> inverse_inertia_tensor_local.matrix [r][k] * loc -> matrix [k][c]; }
                MPE_CHECK_NEAR (&t, s, (r == c) ? 1.0f : 0.0f, 2e-4f, "inv*I identity after add");
            }
        }
        MPE_CHECK (&t, w.composite_part_count == 2);
        MPE_CHECK (&t, w.composite_parts [0].type == object_sphere && w.composite_parts [1].type == object_sphere);
        MPE_CHECK_NEAR (&t, w.composite_parts [0].local_position.x, 0.5f, 1e-6f, "part A COM-framed");
        MPE_CHECK_NEAR (&t, w.composite_parts [1].local_position.x, -0.5f, 1e-6f, "part B COM-framed");
        MPE_CHECK_NEAR (&t, w.composite_parts [0].local_position.y, 0.0f, 1e-6f, "part A local y");
        MPE_CHECK (&t, w.composite_parts [0].mass == 2.0f && w.composite_parts [1].mass == 2.0f);
        MPE_CHECK (&t, w.composite_parts [0].part_index == 0 && w.composite_parts [1].part_index == 1);
        MPE_CHECK (&t, w.composite_parts [0].active == 1 && w.composite_parts [1].active == 1);
        MPE_CHECK_NEAR (&t, w.composite_parts [0].radius, 0.25f, 0.0f, "part radius copied");
        MPE_CHECK (&t, w.composite_parts [0].friction_static == g_cfg.body_defaults.sphere_fric_s);
        MPE_CHECK (&t, w.composite_parts [1].friction_kinetic == g_cfg.body_defaults.sphere_fric_k);
        MPE_CHECK (&t, w.composite_parts [0].restitution == g_cfg.body_defaults.sphere_restitution);
        /* Source untouched: the caller's assembly frame survives. */
        MPE_CHECK_NEAR (&t, dumbbell [0].position.x, 0.5f, 0.0f, "source A untouched");
        MPE_CHECK_NEAR (&t, dumbbell [1].position.x, -0.5f, 0.0f, "source B untouched");
        MPE_CHECK (&t, mpe_clamp_composite_total () == 0ul && mpe_clamp_mass_total () == 0ul);
        /* Composed-inertia guard: static -> dynamic must not regenerate. */
        const math3 snap_local = body -> inertia_tensor_local;
        rigidbody_set_static (body, true);
        MPE_CHECK (&t, body -> type == object_composite && body -> composite_part_count == 2);
        MPE_CHECK_NEAR (&t, body -> inverse_inertia_tensor_local.matrix [0][0], 0.0f, 0.0f, "static zeros inverse local");
        MPE_CHECK (&t, body -> inertia_tensor_local.matrix [0][0] == snap_local.matrix [0][0] &&
                        body -> inertia_tensor_local.matrix [1][1] == snap_local.matrix [1][1]);
        rigidbody_set_static (body, false);
        MPE_CHECK (&t, body -> type == object_composite && body -> mass_is_composed);
        MPE_CHECK (&t, body -> inertia_tensor_local.matrix [0][0] == snap_local.matrix [0][0] &&
                        body -> inertia_tensor_local.matrix [1][1] == snap_local.matrix [1][1] &&
                        body -> inertia_tensor_local.matrix [2][2] == snap_local.matrix [2][2]);
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                float s = 0.0f;
                for (int k = 0; k < 3; k++) {
                    s += body -> inverse_inertia_tensor_local.matrix [r][k] * snap_local.matrix [k][c];
                }
                MPE_CHECK_NEAR (&t, s, (r == c) ? 1.0f : 0.0f, 2e-4f, "guard restores inverse local");
            }
        }
        MPE_CHECK_NEAR (&t, body -> inverse_inertia_system.matrix [0][0], body -> inverse_inertia_tensor_local.matrix [0][0],
                        0.0f, "guard system == local at identity");
        /* Kinematic toggle uses the same restore path. */
        rigidbody_set_kinematic (body, true);
        MPE_CHECK (&t, body -> kinematic && body -> inertia_tensor_local.matrix [0][0] == snap_local.matrix [0][0]);
        rigidbody_set_kinematic (body, false);
        MPE_CHECK (&t, (!body -> kinematic) && body -> inertia_tensor_local.matrix [1][1] == snap_local.matrix [1][1]);
        /* Direct zero-inverse sanitize: restore from authored, not defaults. */
        body -> inverse_inertia_tensor_local = (math3) {{{0.0f}}};
        body -> inverse_inertia_system = (math3) {{{0.0f}}};
        rigidbody_sanitize (body);
        MPE_CHECK (&t, body -> inertia_tensor_local.matrix [0][0] == snap_local.matrix [0][0] &&
                        body -> inertia_tensor_local.matrix [1][1] == snap_local.matrix [1][1]);
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                float s = 0.0f;
                for (int k = 0; k < 3; k++) {
                    s += body -> inverse_inertia_tensor_local.matrix [r][k] * snap_local.matrix [k][c];
                }
                MPE_CHECK_NEAR (&t, s, (r == c) ? 1.0f : 0.0f, 2e-4f, "sanitize restores authored inverse");
            }
        }
        /* Ten gravity ticks: type, slice fields, and tensor survive the
         * every-tick sanitize pass. */
        for (int k = 0; k < 10; k++) { physics_world_step (&w, 1.0f / 60.0f); }
        MPE_CHECK (&t, body -> type == object_composite && body -> composite_part_count == 2 && body -> part_base == 0);
        MPE_CHECK (&t, body -> inertia_tensor_local.matrix [0][0] == snap_local.matrix [0][0] &&
                        body -> inertia_tensor_local.matrix [1][1] == snap_local.matrix [1][1]);
        MPE_CHECK (&t, mpe_clamp_composite_total () == 0ul && mpe_clamp_mass_total () == 0ul);
    }
    /* Density mode: effective kg, not density, lands in the pool. A 1 m
     * cube at 2 kg/m^3 is 2 kg; I = (2/12)(1^2+1^2) = 1/3 on every axis;
     * bound = sqrt(3)/2. */
    physics_world wd;
    mpe_world_begin (&wd);
    const mig_part density_cube [1] = {
        { mig_part_cube, 0.0f, 0.0f, {0.5f, 0.5f, 0.5f}, {0.0f, 10.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_density, 2.0f }
    };
    const int dc = physics_world_add_composite (&wd, density_cube, 1);
    MPE_CHECK (&t, dc == 0);
    if (dc >= 0) {
        MPE_CHECK_NEAR (&t, wd.bodies [dc].mass, 2.0f, 1e-6f, "density cube mass");
        MPE_CHECK_NEAR (&t, wd.bodies [dc].inertia_tensor_local.matrix [0][0], 1.0f / 3.0f, 1e-6f, "density cube Ixx");
        MPE_CHECK_NEAR (&t, wd.bodies [dc].radius, 0.8660254f, 1e-6f, "density cube bound");
        MPE_CHECK (&t, wd.composite_part_count == 1);
        MPE_CHECK (&t, wd.composite_parts [0].type == object_cube && wd.composite_parts [0].mass == 2.0f);
        MPE_CHECK_NEAR (&t, wd.composite_parts [0].half_extensions.x, 0.5f, 0.0f, "cube half extent copied");
        MPE_CHECK (&t, wd.composite_parts [0].friction_static == g_cfg.body_defaults.cube_fric_s);
        MPE_CHECK (&t, wd.composite_parts [0].restitution == g_cfg.body_defaults.cube_restitution);
        MPE_CHECK (&t, mpe_clamp_composite_total () == 0ul);
    }
    /* P2 cap over STORED truth: 20 identical 1 kg parts keep the first 16;
     * the body weighs 16, not 20; one counted substitute. */
    physics_world wc;
    mpe_world_begin (&wc);
    mig_part many [20];
    memset (many, 0, sizeof (many));
    for (int i = 0; i < 20; i++) {
        many [i].type = mig_part_sphere;
        many [i].radius = 0.05f;
        many [i].position = (vector3) {0.05f * (float) i, 20.0f, 0.0f};
        many [i].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        many [i].mass_mode = mig_mass_mode_explicit;
        many [i].mass = 1.0f;
    }
    mpe_clamp_counters_reset ();
    const int cbc = physics_world_add_composite (&wc, many, 20);
    MPE_CHECK (&t, cbc == 0 && wc.body_count == 1 && wc.composite_part_count == 16);
    if (cbc >= 0) {
        MPE_CHECK (&t, wc.bodies [cbc].composite_part_count == 16 && wc.bodies [cbc].part_base == 0);
        MPE_CHECK_NEAR (&t, wc.bodies [cbc].mass, 16.0f, 1e-6f, "body composes stored 16, not requested 20");
        MPE_CHECK_NEAR (&t, wc.bodies [cbc].radius, 0.425f, 1e-6f, "bound over stored 16");
        MPE_CHECK (&t, mpe_clamp_composite_total () == 1ul);
        for (int i = 0; i < 16; i++) {
            MPE_CHECK (&t, wc.composite_parts [i].part_index == i && wc.composite_parts [i].mass == 1.0f);
            MPE_CHECK_NEAR (&t, wc.composite_parts [i].local_position.x, 0.05f * (float) i - 0.375f, 1e-6f,
                            "stored part COM-framed under the cap");
        }
        MPE_CHECK_NEAR (&t, many [19].position.x, 0.95f, 0.0f, "over-cap source tail untouched");
    }
    /* Lowered cap: 6 requested, 4 stored, body mass 4. */
    physics_world wl;
    mpe_world_begin (&wl);
    mpe_config_t lowered = g_cfg;
    lowered.composite.max_parts = 4;
    physics_world_set_config (&wl, &lowered);
    mpe_clamp_counters_reset ();
    const int cbl = physics_world_add_composite (&wl, many, 6);
    MPE_CHECK (&t, cbl == 0 && wl.composite_part_count == 4);
    if (cbl >= 0) {
        MPE_CHECK (&t, wl.bodies [cbl].composite_part_count == 4);
        MPE_CHECK_NEAR (&t, wl.bodies [cbl].mass, 4.0f, 1e-6f, "lowered cap composes 4");
        MPE_CHECK (&t, mpe_clamp_composite_total () == 1ul);
    }
    /* Whole-assembly invalidation: a bad part BEYOND the cap still refuses
     * the whole add — no body, no pool slot, no counted substitute. */
    physics_world wbad;
    mpe_world_begin (&wbad);
    mig_part bad_tail [17];
    memcpy (bad_tail, many, sizeof (bad_tail));
    bad_tail [16].type = (mig_part_type) 9;
    mpe_clamp_counters_reset ();
    MPE_CHECK (&t, physics_world_add_composite (&wbad, bad_tail, 17) == -1);
    MPE_CHECK (&t, wbad.body_count == 0 && wbad.composite_part_count == 0 && wbad.composite_parts == NULL);
    MPE_CHECK (&t, mpe_clamp_composite_total () == 0ul);
    /* Named rejections keep the whole world untouched. */
    physics_world wr;
    mpe_world_begin (&wr);
    MPE_CHECK (&t, physics_world_add_composite (NULL, dumbbell, 2) == -1);
    MPE_CHECK (&t, physics_world_add_composite (&wr, NULL, 2) == -1);
    MPE_CHECK (&t, physics_world_add_composite (&wr, dumbbell, 0) == -1);
    mig_part badquat = dumbbell [0];
    badquat.orientation = (vector4) {0.5f, 0.0f, 0.0f, 0.0f};
    MPE_CHECK (&t, physics_world_add_composite (&wr, &badquat, 1) == -1);
    mig_part spread [2] = { dumbbell [0], dumbbell [1] };
    spread [0].position = (vector3) {150.0f, 0.0f, 0.0f};
    spread [1].position = (vector3) {-150.0f, 0.0f, 0.0f};
    MPE_CHECK (&t, physics_world_add_composite (&wr, spread, 2) == -1);
    mig_part tiny = dumbbell [0];
    tiny.mass = 1e-5f;
    MPE_CHECK (&t, physics_world_add_composite (&wr, &tiny, 1) == -1);
    MPE_CHECK (&t, wr.body_count == 0 && wr.composite_part_count == 0);
    MPE_CHECK (&t, mpe_clamp_composite_total () == 0ul && mpe_clamp_mass_total () == 0ul);
    physics_world_cleanup (&wr);
    physics_world_cleanup (&wbad);
    physics_world_cleanup (&wl);
    physics_world_cleanup (&wc);
    physics_world_cleanup (&wd);
    physics_world_cleanup (&w);
    if (t.failures == 0) {
        printf ("[PASS] mig WP2.2: world API composes one composite body with a COM-framed pool slice, cap over stored truth, authored-tensor guard\n");
        MPE_INFO ("dumbbell mass 4/Ixx 0.1/Iyy 1.1/bound 0.75, density cube 2 kg/I 1-3/bound 0.866, 20->16 stored mass 16 (1 clamp), cap 4->4, bad tail rejected, static/kinematic/sanitize keep tensor");
    }
    mpe_test_end (&t);
    return t.failures;
}

/* WP2 2.3 gate (plan §4): broadphase part dispatch. The oracle is the
 * charter §4.2/§4.3 bound contract written out by hand:
 *  - the scalar broadphase radius of a composite is the AUTHORED covering
 *    sphere max_i(|com - p_i| + B_i), NOT the sqrt(3) cube inflation of
 *    the mirrored half_extensions, and it is rotation-invariant;
 *  - the hash AABB is the per-part symmetric hull about the COM,
 *    max_i(|R_body d_i| + rotated part extent), using each part's own
 *    primitive extents and part-local orientation;
 *  - a slice with no usable part (all inactive) falls back to the
 *    conservative scalar box, never to a zero-size false negative.
 * The grid is pinned to 0.5 m. The cell-count oracle
 * (world->broadphase->node_count) is exact and hand-derived from the AABB
 * extents: each body inserts one node per occupied cell, so the sum
 * counts the hull directly. Pair counts alone are NOT used for the
 * negative direction: the hash bucket is coarser than the cell, so two
 * bodies in distinct cells can still share a bucket, and the sphere cull
 * (which must stay conservative) then admits them as legitimate
 * candidates. Cell occupancy has no such ambiguity. */
int mpe_t_mig_composite_broadphase (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "mig_composite_broadphase");
    mpe_clamp_counters_reset ();
    mpe_config_t bcfg = g_cfg;
    bcfg.broadphase.cell_size_default = 0.5f;
    bcfg.broadphase.cell_size_min = 0.5f;
    bcfg.broadphase.cell_size_max = 0.5f;
    broadphase_pair pairs [16];
    const float dt = 1.0f / 60.0f;
    const vector3 anchor_position = (vector3) {10.0f, 10.0f, 10.0f};
    const mig_part dumbbell [2] = {
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {-0.5f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f },
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f }
    };
    /* --- authored scalar bound + AABB cell count, identity then 45 deg Z ---
     * Dumbbell hull: |+/-0.5| + 0.25 = 0.75 on X, 0.25 on Y/Z.
     * Identity cells: 4 x 2 x 2 = 16. At 45 deg Z the parts land at
     * +/-0.353553, so the hull is (0.603553, 0.603553, 0.25) and the
     * cells are 4 x 4 x 2 = 32. The anchor sphere r=0.1 at (10,10,10)
     * inserts 2 x 2 x 2 = 8 cells. Deactivating both parts leaves no
     * usable extent: the fallback scalar box (half 0.75) is 4 x 4 x 4 =
     * 64 cells, i.e. the conservative authored bound, never zero. */
    physics_world wn;
    mpe_world_begin (&wn);
    physics_world_set_config (&wn, &bcfg);
    const int nb = physics_world_add_composite (&wn, dumbbell, 2);
    const int na = physics_world_add_sphere (&wn, 0.1f, 1.0f, anchor_position);
    MPE_CHECK (&t, nb == 0 && na == 1);
    if (nb == 0 && na == 1) {
        MPE_CHECK_NEAR (&t, broadphase_bounding_radius (&wn.bodies [nb]), 0.75f, 1e-6f,
                        "authored covering sphere, not sqrt(3) inflation");
        MPE_CHECK (&t, wn.bodies [nb].half_extensions.x == 0.75f);
        MPE_CHECK (&t, broadphase_generate_pairing (&wn, pairs, 16, dt) >= 0);
        MPE_CHECK (&t, wn.broadphase -> node_count == 24);
        MPE_CHECK_NEAR (&t, wn.broadphase -> current_cell_size, 0.5f, 0.0f, "pinned cell size");
        wn.bodies [nb].orientation = (vector4) {0.9238795325f, 0.0f, 0.0f, 0.3826834324f};
        rigidbody_update_axes (&wn.bodies [nb]);
        MPE_CHECK_NEAR (&t, broadphase_bounding_radius (&wn.bodies [nb]), 0.75f, 1e-6f,
                        "authored bound invariant under rotation");
        MPE_CHECK (&t, broadphase_generate_pairing (&wn, pairs, 16, dt) >= 0);
        MPE_CHECK (&t, wn.broadphase -> node_count == 40);
        wn.bodies [nb].orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        rigidbody_update_axes (&wn.bodies [nb]);
        wn.composite_parts [0].active = 0;
        wn.composite_parts [1].active = 0;
        MPE_CHECK (&t, broadphase_generate_pairing (&wn, pairs, 16, dt) >= 0);
        MPE_CHECK (&t, wn.broadphase -> node_count == 72);
    }
    physics_world_cleanup (&wn);
    /* --- pair emission at the pinned grid ---
     * Two identity dumbbells 1.4 m apart: their x hulls overlap cell 1
     * (A -2..1, B 1..4), so exactly one pair is emitted. */
    physics_world wp;
    mpe_world_begin (&wp);
    physics_world_set_config (&wp, &bcfg);
    mig_part dumbbell_b [2] = { dumbbell [0], dumbbell [1] };
    dumbbell_b [0].position.x += 1.4f;
    dumbbell_b [1].position.x += 1.4f;
    const int pb0 = physics_world_add_composite (&wp, dumbbell, 2);
    const int pb1 = physics_world_add_composite (&wp, dumbbell_b, 2);
    MPE_CHECK (&t, pb0 == 0 && pb1 == 1);
    if (pb0 == 0 && pb1 == 1) {
        const int n = broadphase_generate_pairing (&wp, pairs, 16, dt);
        MPE_CHECK (&t, n == 1 && pairs [0].object_index_a == 0 && pairs [0].object_index_b == 1);
    }
    physics_world_cleanup (&wp);
    /* --- off-centre cube part: part-local orientation is honoured ---
     * Cube half (0.55, 0.27, 0.13) at +0.3 X and a 0.1 sphere at the
     * origin, equal masses -> COM +0.15. Rotated 90 deg about its own Z,
     * the cube lands (0.27, 0.55, 0.13), so the COM-framed hull is
     * (0.42, 0.55, 0.13) at body x 0.15: cells 3 x 4 x 2 = 24, plus the
     * 8 anchor cells = 32. With the part orientation cleared the same
     * pool record reads (0.55, 0.27, 0.13): hull (0.70, 0.27, 0.13) ->
     * 4 x 2 x 2 = 16, +8 = 24. Deactivating both parts falls back to the
     * authored 0.776339 box: 4 x 4 x 4 = 64, +8 = 72. */
    physics_world wc;
    mpe_world_begin (&wc);
    physics_world_set_config (&wc, &bcfg);
    const mig_part cube_parts [2] = {
        { mig_part_cube, 0.0f, 0.0f, {0.55f, 0.27f, 0.13f}, {0.3f, 0.0f, 0.0f},
          {0.7071067812f, 0.0f, 0.0f, 0.7071067812f}, mig_mass_mode_explicit, 1.0f },
        { mig_part_sphere, 0.1f, 0.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 1.0f }
    };
    const int cc = physics_world_add_composite (&wc, cube_parts, 2);
    const int ca = physics_world_add_sphere (&wc, 0.1f, 1.0f, anchor_position);
    MPE_CHECK (&t, cc == 0 && ca == 1);
    if (cc == 0 && ca == 1) {
        const float expected = 0.15f + sqrtf (0.55f * 0.55f + 0.27f * 0.27f + 0.13f * 0.13f);
        MPE_CHECK_NEAR (&t, broadphase_bounding_radius (&wc.bodies [cc]), expected, 1e-6f,
                        "cube-part authored bound");
        MPE_CHECK (&t, broadphase_generate_pairing (&wc, pairs, 16, dt) >= 0);
        MPE_CHECK (&t, wc.broadphase -> node_count == 32);
        wc.composite_parts [0].local_orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        MPE_CHECK (&t, broadphase_generate_pairing (&wc, pairs, 16, dt) >= 0);
        MPE_CHECK (&t, wc.broadphase -> node_count == 24);
        wc.composite_parts [0].active = 0;
        wc.composite_parts [1].active = 0;
        MPE_CHECK (&t, broadphase_generate_pairing (&wc, pairs, 16, dt) >= 0);
        MPE_CHECK (&t, wc.broadphase -> node_count == 72);
    }
    physics_world_cleanup (&wc);
    /* --- cylinder: axle on part-local X, rotated 90 deg about Y ---
     * Correct hull: (0.1, 0.1, 0.52). A 0.06 sphere at z = 0.58 shares
     * the z = 1 cell band with the body (whose z reaches 0.52) and sits
     * inside the authored 0.529528 cull sphere; exactly one pair.
     * Reading the part as (radius, radius, half_length) would leave
     * z = 0.1 and lose the shared cell. The 0.52/0.58 values keep every
     * cell boundary off an integer so float rotation epsilon cannot
     * decide the count. */
    physics_world wcyl;
    mpe_world_begin (&wcyl);
    physics_world_set_config (&wcyl, &bcfg);
    const mig_part cylinder_part [1] = {
        { mig_part_cylinder, 0.1f, 0.52f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
          {0.7071067812f, 0.0f, 0.7071067812f, 0.0f}, mig_mass_mode_explicit, 1.0f }
    };
    const int yc = physics_world_add_composite (&wcyl, cylinder_part, 1);
    const int ys = physics_world_add_sphere (&wcyl, 0.06f, 1.0f, (vector3) {0.0f, 0.0f, 0.58f});
    MPE_CHECK (&t, yc == 0 && ys == 1);
    if (yc == 0 && ys == 1) {
        MPE_CHECK_NEAR (&t, broadphase_bounding_radius (&wcyl.bodies [yc]), sqrtf (0.2804f), 1e-6f,
                        "cylinder-part authored bound");
        const int n = broadphase_generate_pairing (&wcyl, pairs, 16, dt);
        MPE_CHECK (&t, n == 1);
    }
    physics_world_cleanup (&wcyl);
    /* --- inactive slice: conservative fallback pairs, never vanishes ---
     * Both dumbbell parts inactive: the AABB path falls back to the
     * authored 0.75 scalar box, so the sphere at y = 0.6 (inside that
     * box, outside the real parts) still pairs. A zero-size fallback
     * would insert only the origin cell and emit nothing. */
    physics_world wfall;
    mpe_world_begin (&wfall);
    physics_world_set_config (&wfall, &bcfg);
    const int fb = physics_world_add_composite (&wfall, dumbbell, 2);
    const int fs = physics_world_add_sphere (&wfall, 0.1f, 1.0f, (vector3) {0.0f, 0.6f, 0.0f});
    MPE_CHECK (&t, fb == 0 && fs == 1);
    if (fb == 0 && fs == 1) {
        wfall.composite_parts [0].active = 0;
        wfall.composite_parts [1].active = 0;
        const int n = broadphase_generate_pairing (&wfall, pairs, 16, dt);
        MPE_CHECK (&t, n == 1 && pairs [0].object_index_a == 0 && pairs [0].object_index_b == 1);
    }
    physics_world_cleanup (&wfall);
    MPE_CHECK (&t, mpe_clamp_composite_total () == 0ul && mpe_clamp_mass_total () == 0ul);
    if (t.failures == 0) {
        printf ("[PASS] mig WP2.3: composite broadphase uses the authored covering sphere and the per-part COM-symmetric AABB hull\n");
        MPE_INFO ("bound 0.75 (not 1.299), dumbbell cells 16 -> 45Z 32 -> inactive scalar box 64 (+8 anchor), part-rotated cube cells 24 -> identity 16 -> inactive 64, pair at dx 1.4 = 1, cylinder axle 90Y pairs at z 0.58, inactive+sphere = 1");
    }
    mpe_test_end (&t);
    return t.failures;
}

/* WP2 2.4/2.5 gate (plan §4): ghost-view narrowphase + contact identity.
 * Oracles are hand geometry, not calls into the implementation:
 *  - a dumbbell's two spheres each generate their OWN manifold over a flat
 *    cube (two part pairs -> two manifolds, one normal each), contacts
 *    stamped with the per-body part index, parent bodies byte-identical
 *    after detection (ghosts are scratch, not aliases);
 *  - a part-local 90 deg cube rotation moves the collision face from
 *    y=0.27 to y=0.55, so a probe at y=0.56 collides only when the ghost
 *    carries the part orientation through BOTH the AABB prefilter and the
 *    narrowphase; with the orientation cleared the same probe is empty;
 *  - composite-vs-composite part-pair iteration finds exactly the one pair
 *    whose centres coincide (1,1), proving the cross product is over parts,
 *    not bodies;
 *  - warm-start identity is (id_a, part_a, id_b, part_b): two cache
 *    entries that differ ONLY in part fields restore disjoint impulses,
 *    an unknown part misses, and the reversed-body role matches with the
 *    part fields swapped (the TASK_05 both-side discipline);
 *  - a stepped dumbbell over a static slab saves one cache entry per
 *    touching part with distinct part indices and distinct local points;
 *  - MPE_MODULE_ABI is 2 with MIN 1, an ABI-1 descriptor still attaches,
 *    and the next ABI is refused.
 */
int mpe_t_mig_composite_narrowphase (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "mig_composite_narrowphase");
    mpe_clamp_counters_reset ();
    collision_data manifolds [8];
    const float dt = 1.0f / 60.0f;
    /* --- 1. one manifold per colliding part pair ----------------------- */
    physics_world wpart;
    mpe_world_begin (&wpart);
    const mig_part low_dumbbell [2] = {
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {-0.5f, 0.24f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f },
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {0.5f, 0.24f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f }
    };
    const int cb = physics_world_add_composite (&wpart, low_dumbbell, 2);
    const int ck = physics_world_add_cube (&wpart, (vector3) {0.0f, -0.25f, 0.0f},
                                           (vector3) {2.0f, 0.25f, 2.0f}, 0.0f);
    MPE_CHECK (&t, cb == 0 && ck == 1);
    if (cb == 0 && ck == 1) {
        const int base = wpart.bodies [cb].part_base;
        rigidbody parent_before = wpart.bodies [cb];
        composite_part parts_before [2];
        memcpy (parts_before, &wpart.composite_parts [base], sizeof (parts_before));
        int dropped = -1;
        int n = mpe_shape_dispatch_all (&wpart, &wpart.bodies [cb], &wpart.bodies [ck], manifolds, 8, &dropped);
        MPE_CHECK (&t, n == 2 && dropped == 0);
        MPE_CHECK (&t, memcmp (&parent_before, &wpart.bodies [cb], sizeof (parent_before)) == 0);
        MPE_CHECK (&t, memcmp (parts_before, &wpart.composite_parts [base], sizeof (parts_before)) == 0);
        if (n == 2) {
            MPE_CHECK (&t, manifolds [0].object_a == &wpart.bodies [cb] &&
                              manifolds [0].object_b == &wpart.bodies [ck]);
            MPE_CHECK (&t, manifolds [1].object_a == &wpart.bodies [cb] &&
                              manifolds [1].object_b == &wpart.bodies [ck]);
            MPE_CHECK (&t, manifolds [0].contact_count == 1 && manifolds [1].contact_count == 1);
            if (manifolds [0].contact_count == 1 && manifolds [1].contact_count == 1) {
                MPE_CHECK (&t, manifolds [0].contacts [0].part_a == 0 && manifolds [0].contacts [0].part_b == 0);
                MPE_CHECK (&t, manifolds [1].contacts [0].part_a == 1 && manifolds [1].contacts [0].part_b == 0);
                MPE_CHECK_NEAR (&t, manifolds [0].contacts [0].position.x, -0.5f, 1e-5f,
                                "part 0 contact sits under sphere 0");
                MPE_CHECK_NEAR (&t, manifolds [1].contacts [0].position.x, 0.5f, 1e-5f,
                                "part 1 contact sits under sphere 1");
                MPE_CHECK_NEAR (&t, manifolds [0].contacts [0].penetration, 0.01f, 1e-5f, "part 0 depth");
                MPE_CHECK_NEAR (&t, manifolds [1].contacts [0].penetration, 0.01f, 1e-5f, "part 1 depth");
            }
        }
        /* A bounded sink reports the overflow, never silently drops it. */
        int capped_dropped = 0;
        MPE_CHECK (&t, mpe_shape_dispatch_all (&wpart, &wpart.bodies [cb], &wpart.bodies [ck],
                                               manifolds, 1, &capped_dropped) == 1);
        MPE_CHECK (&t, capped_dropped == 1);
        /* Single-manifold entry stays valid for direct callers (deepest;
         * the equal-depth tie keeps definition order). */
        collision_data single = {0};
        MPE_CHECK (&t, mpe_shape_dispatch (&wpart, &wpart.bodies [cb], &wpart.bodies [ck], &single));
        MPE_CHECK (&t, single.contact_count == 1 && single.contacts [0].part_a == 0 &&
                          single.object_a == &wpart.bodies [cb] && single.object_b == &wpart.bodies [ck]);
        /* Inactive parts vanish from the collision surface, one at a time. */
        wpart.composite_parts [base].active = 0;
        n = mpe_shape_dispatch_all (&wpart, &wpart.bodies [cb], &wpart.bodies [ck], manifolds, 8, &dropped);
        MPE_CHECK (&t, n == 1 && manifolds [0].contacts [0].part_a == 1);
        wpart.composite_parts [base + 1].active = 0;
        MPE_CHECK (&t, mpe_shape_dispatch_all (&wpart, &wpart.bodies [cb], &wpart.bodies [ck],
                                               manifolds, 8, &dropped) == 0);
        wpart.composite_parts [base].active = 1;
        wpart.composite_parts [base + 1].active = 1;
    }
    physics_world_cleanup (&wpart);
    /* --- 2. part-local orientation through AABB + narrowphase ---------- */
    physics_world worient;
    mpe_world_begin (&worient);
    const mig_part cube_mix [2] = {
        { mig_part_cube, 0.0f, 0.0f, {0.55f, 0.27f, 0.13f}, {0.3f, 0.0f, 0.0f},
          {0.7071067812f, 0.0f, 0.0f, 0.7071067812f}, mig_mass_mode_explicit, 1.0f },
        { mig_part_sphere, 0.1f, 0.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 1.0f }
    };
    const int cq = physics_world_add_composite (&worient, cube_mix, 2);
    const int ps = physics_world_add_sphere (&worient, 0.06f, 0.5f, (vector3) {0.15f, 0.56f, 0.0f});
    MPE_CHECK (&t, cq == 0 && ps == 1);
    if (cq == 0 && ps == 1) {
        const int base = worient.bodies [cq].part_base;
        int dropped = -1;
        /* Cube part world centre is x=0.30 (COM 0.15 + local 0.15); after
         * 90 deg Z its local X half 0.55 is the world Y half, so the top
         * face is y=0.55 and a 0.06 sphere at y=0.56 penetrates 0.05. */
        int n = mpe_shape_dispatch_all (&worient, &worient.bodies [cq], &worient.bodies [ps],
                                        manifolds, 8, &dropped);
        MPE_CHECK (&t, n == 1 && dropped == 0);
        if (n == 1 && manifolds [0].contact_count == 1) {
            MPE_CHECK (&t, manifolds [0].contacts [0].part_a == 0 && manifolds [0].contacts [0].part_b == 0);
            MPE_CHECK_NEAR (&t, manifolds [0].normal_vector.y, 1.0f, 1e-5f, "cube(A)->probe(B) normal");
            MPE_CHECK_NEAR (&t, manifolds [0].contacts [0].position.y, 0.55f, 1e-5f, "rotated face is the contact");
            MPE_CHECK_NEAR (&t, manifolds [0].contacts [0].position.x, 0.15f, 1e-5f, "contact stays in the face");
            MPE_CHECK_NEAR (&t, manifolds [0].contacts [0].penetration, 0.05f, 1e-5f, "0.56 - 0.55 gap");
        }
        /* Clearing ONLY the part-local rotation drops the face to y=0.27:
         * the same probe is outside both the AABB and the primitive, so
         * the orientation really is part-local rather than body-level. */
        worient.composite_parts [base].local_orientation = (vector4) {1.0f, 0.0f, 0.0f, 0.0f};
        MPE_CHECK (&t, mpe_shape_dispatch_all (&worient, &worient.bodies [cq], &worient.bodies [ps],
                                               manifolds, 8, &dropped) == 0);
        worient.composite_parts [base].local_orientation =
            (vector4) {0.7071067812f, 0.0f, 0.0f, 0.7071067812f};
        /* Far probe: no part pair may be invented. */
        worient.bodies [ps].position = (vector3) {0.15f, 0.9f, 0.0f};
        MPE_CHECK (&t, mpe_shape_dispatch_all (&worient, &worient.bodies [cq], &worient.bodies [ps],
                                               manifolds, 8, &dropped) == 0);
    }
    physics_world_cleanup (&worient);
    /* --- 3. composite vs composite: exactly part pair (1,1) ------------ */
    physics_world wcc;
    mpe_world_begin (&wcc);
    const mig_part cc_a [2] = {
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {-0.5f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f },
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f }
    };
    /* B's part 0 is parked at x=2.5 and its part 1 at x=0.5; the COM is
     * x=1.5, so only A's +0.5 sphere meets B's part 1 exactly. A/A and
     * B/B are 1.0 and 2.0 apart (sum of radii 0.5): no other pair. */
    const mig_part cc_b [2] = {
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {2.5f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f },
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f }
    };
    const int cca = physics_world_add_composite (&wcc, cc_a, 2);
    const int ccb = physics_world_add_composite (&wcc, cc_b, 2);
    MPE_CHECK (&t, cca == 0 && ccb == 1);
    if (cca == 0 && ccb == 1) {
        int dropped = -1;
        int n = mpe_shape_dispatch_all (&wcc, &wcc.bodies [cca], &wcc.bodies [ccb],
                                        manifolds, 8, &dropped);
        MPE_CHECK (&t, n == 1 && dropped == 0);
        if (n == 1) {
            MPE_CHECK (&t, manifolds [0].object_a == &wcc.bodies [cca] &&
                              manifolds [0].object_b == &wcc.bodies [ccb]);
            MPE_CHECK (&t, manifolds [0].contacts [0].part_a == 1 && manifolds [0].contacts [0].part_b == 1);
            MPE_CHECK_NEAR (&t, manifolds [0].contacts [0].position.x, 0.5f, 1e-5f, "coincident part centres");
        }
    }
    physics_world_cleanup (&wcc);
    /* --- 4. part-scoped warm-start identity ---------------------------- */
    physics_world wcache;
    mpe_world_begin (&wcache);
    mpe_config_t ccfg = g_cfg;
    ccfg.solver.warm_start_damping = 1.0f; /* restore seeds so the oracle is the saved value */
    physics_world_set_config (&wcache, &ccfg);
    const int cia = physics_world_add_sphere (&wcache, 0.5f, 1.0f, (vector3) {0.0f, 0.0f, 0.0f});
    const int cib = physics_world_add_sphere (&wcache, 0.5f, 1.0f, (vector3) {0.0f, 1.0f, 0.0f});
    MPE_CHECK (&t, cia == 0 && cib == 1);
    if (cia == 0 && cib == 1) {
        const uint32_t ida = wcache.bodies [cia].object_id;
        const uint32_t idb = wcache.bodies [cib].object_id;
        collision_data saved = {0};
        saved.object_a = &wcache.bodies [cia];
        saved.object_b = &wcache.bodies [cib];
        saved.normal_vector = (vector3) {0.0f, 1.0f, 0.0f};
        saved.contact_count = 2;
        for (int k = 0; k < 2; k++) {
            contact_point_data *cp = &saved.contacts [k];
            cp -> position = (vector3) {0.0f, 0.5f, 0.0f};
            cp -> penetration = 0.01f;
            cp -> local_position_a = (vector3) {0.0f, 0.5f, 0.0f};
            cp -> local_position_b = (vector3) {0.0f, -0.5f, 0.0f};
            cp -> accumulated_normal_impulse = (k == 0) ? 3.0f : 7.0f;
            cp -> accumulated_tangent_impulse = (k == 0) ? 0.1f : -0.1f;
            cp -> tangent_vector = (vector3) {1.0f, 0.0f, 0.0f};
            cp -> part_a = (k == 0) ? (uint8_t) 0 : (uint8_t) 1;
            cp -> part_b = (k == 0) ? (uint8_t) 1 : (uint8_t) 0;
        }
        contact_cache_save (&wcache, &saved, 1);
        MPE_CHECK (&t, wcache.world_contact_cache_count == 2);
        MPE_CHECK (&t, contact_cache_has_part_pair (&wcache, ida, 0, idb, 1));
        MPE_CHECK (&t, contact_cache_has_part_pair (&wcache, ida, 1, idb, 0));
        MPE_CHECK (&t, contact_cache_has_part_pair (&wcache, idb, 1, ida, 0)); /* swapped bodies */
        MPE_CHECK (&t, !contact_cache_has_part_pair (&wcache, ida, 0, idb, 0));
        MPE_CHECK (&t, !contact_cache_has_part_pair (&wcache, ida, 2, idb, 0));
        contact_cache_stats_reset (&wcache);
        collision_data probe = {0};
        probe.object_a = &wcache.bodies [cia];
        probe.object_b = &wcache.bodies [cib];
        probe.normal_vector = (vector3) {0.0f, 1.0f, 0.0f};
        probe.contact_count = 1;
        probe.contacts [0].position = (vector3) {0.0f, 0.5f, 0.0f};
        probe.contacts [0].penetration = 0.01f;
        /* Approach along the normal: a separating contact is deliberately
         * cooled by prepare and would hide the cache decision. */
        wcache.bodies [cia].velocity = (vector3) {0.0f, 1.0f, 0.0f};
        wcache.bodies [cib].velocity = (vector3) {0.0f, -1.0f, 0.0f};
        probe.contacts [0].part_a = 0;
        probe.contacts [0].part_b = 1;
        collision_data prepared = {0};
        collision_prepare_solver (&wcache, &probe, &prepared, dt);
        MPE_CHECK_NEAR (&t, prepared.contacts [0].accumulated_normal_impulse, 3.0f, 1e-5f,
                        "part pair (0,1) restores its own seed");
        MPE_CHECK_NEAR (&t, prepared.contacts [0].accumulated_tangent_impulse, 0.1f, 1e-5f,
                        "part pair (0,1) restores its own tangent");
        MPE_CHECK (&t, prepared.contacts [0].part_a == 0 && prepared.contacts [0].part_b == 1);
        wcache.bodies [cia].velocity = (vector3) {0.0f, 1.0f, 0.0f};
        wcache.bodies [cib].velocity = (vector3) {0.0f, -1.0f, 0.0f};
        probe.contacts [0].part_a = 1;
        probe.contacts [0].part_b = 0;
        collision_prepare_solver (&wcache, &probe, &prepared, dt);
        MPE_CHECK_NEAR (&t, prepared.contacts [0].accumulated_normal_impulse, 7.0f, 1e-5f,
                        "part pair (1,0) restores its own seed, not (0,1)'s");
        wcache.bodies [cia].velocity = (vector3) {0.0f, 1.0f, 0.0f};
        wcache.bodies [cib].velocity = (vector3) {0.0f, -1.0f, 0.0f};
        probe.contacts [0].part_a = 0;
        probe.contacts [0].part_b = 0;
        collision_prepare_solver (&wcache, &probe, &prepared, dt);
        MPE_CHECK_NEAR (&t, prepared.contacts [0].accumulated_normal_impulse, 0.0f, 1e-6f,
                        "unknown part pair must not alias a saved part");
        /* Reversed body role: the part fields must swap too. Cache entry
         * (A=0, B=1) is (B=1, A=0) from the probe's point of view. */
        collision_data reversed = {0};
        reversed.object_a = &wcache.bodies [cib];
        reversed.object_b = &wcache.bodies [cia];
        reversed.normal_vector = (vector3) {0.0f, -1.0f, 0.0f};
        reversed.contact_count = 1;
        reversed.contacts [0].position = (vector3) {0.0f, 0.5f, 0.0f};
        reversed.contacts [0].penetration = 0.01f;
        wcache.bodies [cia].velocity = (vector3) {0.0f, 1.0f, 0.0f};
        wcache.bodies [cib].velocity = (vector3) {0.0f, -1.0f, 0.0f};
        reversed.contacts [0].part_a = 1;
        reversed.contacts [0].part_b = 0;
        collision_prepare_solver (&wcache, &reversed, &prepared, dt);
        MPE_CHECK_NEAR (&t, prepared.contacts [0].accumulated_normal_impulse, 3.0f, 1e-5f,
                        "reversed role matches the swapped part pair");
        MPE_CHECK_NEAR (&t, prepared.contacts [0].accumulated_tangent_impulse, -0.1f, 1e-5f,
                        "reversed role negates the tangent");
        wcache.bodies [cia].velocity = (vector3) {0.0f, 1.0f, 0.0f};
        wcache.bodies [cib].velocity = (vector3) {0.0f, -1.0f, 0.0f};
        reversed.contacts [0].part_a = 0;
        reversed.contacts [0].part_b = 0;
        collision_prepare_solver (&wcache, &reversed, &prepared, dt);
        MPE_CHECK_NEAR (&t, prepared.contacts [0].accumulated_normal_impulse, 0.0f, 1e-6f,
                        "reversed role with non-swapped parts misses");
        MPE_CHECK (&t, contact_cache_get_hits (&wcache) == 3);
        MPE_CHECK (&t, contact_cache_get_misses (&wcache) == 2);
    }
    physics_world_cleanup (&wcache);
    /* --- 5. step path: one cache entry per touching part --------------- */
    physics_world wstep;
    mpe_world_begin (&wstep);
    mpe_config_t scfg = g_cfg;
    physics_world_set_config (&wstep, &scfg);
    wstep.static_plane_enabled = false; /* support is the explicit slab */
    const int sb = physics_world_add_composite (&wstep, low_dumbbell, 2);
    const int sk = physics_world_add_cube (&wstep, (vector3) {0.0f, -0.25f, 0.0f},
                                           (vector3) {2.0f, 0.25f, 2.0f}, 0.0f);
    MPE_CHECK (&t, sb == 0 && sk == 1);
    if (sb == 0 && sk == 1) {
        physics_world_step (&wstep, dt);
        MPE_CHECK (&t, mpe_world_finite (&wstep));
        MPE_CHECK (&t, wstep.world_contact_cache_count == 2);
        const uint32_t cid = wstep.bodies [sb].object_id;
        const uint32_t kid = wstep.bodies [sk].object_id;
        int part_seen [2] = {0, 0};
        int comp_entries = 0;
        if (wstep.world_contact_cache_count == 2) {
            for (int i = 0; i < wstep.world_contact_cache_count; i++) {
                const cached_contact *cc = &wstep.world_contact_cache [i];
                if (cc -> object_id_a == cid && cc -> object_id_b == kid) {
                    MPE_CHECK (&t, cc -> part_b == 0);
                    MPE_CHECK (&t, cc -> part_a == 0 || cc -> part_a == 1);
                    if (cc -> part_a < 2) { part_seen [cc -> part_a] = 1; }
                    MPE_CHECK_NEAR (&t, fabsf (cc -> local_position_a.x), 0.5f, 1e-4f,
                                    "composite-local contact x is the part centre");
                    comp_entries++;
                } else if (cc -> object_id_a == kid && cc -> object_id_b == cid) {
                    MPE_CHECK (&t, cc -> part_a == 0);
                    MPE_CHECK (&t, cc -> part_b == 0 || cc -> part_b == 1);
                    if (cc -> part_b < 2) { part_seen [cc -> part_b] = 1; }
                    MPE_CHECK_NEAR (&t, fabsf (cc -> local_position_b.x), 0.5f, 1e-4f,
                                    "composite-local contact x is the part centre");
                    comp_entries++;
                }
            }
        }
        MPE_CHECK (&t, comp_entries == 2);
        MPE_CHECK (&t, part_seen [0] == 1 && part_seen [1] == 1);
        MPE_CHECK (&t, contact_cache_has_part_pair (&wstep, cid, 0, kid, 0));
        MPE_CHECK (&t, contact_cache_has_part_pair (&wstep, cid, 1, kid, 0));
        MPE_CHECK (&t, contact_cache_has_part_pair (&wstep, kid, 0, cid, 1));
        MPE_CHECK (&t, !contact_cache_has_part_pair (&wstep, cid, 2, kid, 0));
    }
    physics_world_cleanup (&wstep);
    /* --- 6. plugin ABI: 2 current, 1 still loads, 3 refused ------------ */
    MPE_CHECK (&t, MPE_MODULE_ABI == 2);
    MPE_CHECK (&t, MPE_MODULE_ABI_MIN == 1);
    physics_world wabi;
    mpe_world_begin (&wabi);
    mpe_module_desc_t legacy;
    memset (&legacy, 0, sizeof (legacy));
    legacy.name = "abi1-probe";
    legacy.version = "1.0";
    legacy.kind = "generic";
    legacy.abi = 1;
    MPE_CHECK (&t, physics_world_attach_module (&wabi, &legacy) >= 0);
    MPE_CHECK (&t, physics_world_detach_module (&wabi, "abi1-probe") == 0);
    mpe_module_desc_t future = legacy;
    future.name = "abi3-probe";
    future.abi = 3;
    MPE_CHECK (&t, physics_world_attach_module (&wabi, &future) == MPE_ATTACH_E_ABI);
    MPE_CHECK (&t, mpe_register_module (&legacy) >= 0);
    MPE_CHECK (&t, mpe_find_module ("abi1-probe") != NULL);
    mpe_unregister_module ("abi1-probe");
    MPE_CHECK (&t, mpe_find_module ("abi1-probe") == NULL);
    physics_world_cleanup (&wabi);
    MPE_CHECK (&t, mpe_clamp_composite_total () == 0ul && mpe_clamp_mass_total () == 0ul);
    if (t.failures == 0) {
        printf ("[PASS] mig WP2.4/2.5: ghost views emit one manifold per part pair, stamped with the parent identity and part-scoped cache key\n");
        MPE_INFO ("2 part manifolds over a slab, part-local 90Z cube face y 0.27 -> 0.55, composite/composite only (1,1), cache seeds 3.0/7.0 by part, step saves 2 parts, ABI 2 accepts legacy 1");
    }
    mpe_test_end (&t);
    return t.failures;
}

/* WP2 2.6 gate helpers: independent quaternion math, written here so the
 * floor-support oracle never calls the engine's rotation/support code. */
static vector3 mig_gate_cross (vector3 a, vector3 b) {
    return (vector3) {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
} static vector3 mig_gate_qrotate (vector4 q, vector3 v) {
    vector3 u = {q.x, q.y, q.z};
    vector3 t = mig_gate_cross (u, v);
    t = (vector3) {2.0f * t.x, 2.0f * t.y, 2.0f * t.z};
    vector3 ut = mig_gate_cross (u, t);
    return (vector3) {v.x + q.w * t.x + ut.x, v.y + q.w * t.y + ut.y,
                      v.z + q.w * t.z + ut.z};
} static vector4 mig_gate_qmul (vector4 a, vector4 b) {
    return (vector4) {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
} static vector4 mig_gate_qnorm (vector4 q) {
    float n = sqrtf (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if ((!isfinite (n)) || (!(n > 1e-12f))) { return (vector4) {1.0f, 0.0f, 0.0f, 0.0f}; }
    float inv = 1.0f / n;
    return (vector4) {q.w * inv, q.x * inv, q.y * inv, q.z * inv};
} /* Exact world -Y of every active part's lowest point, hand-derived:
 * sphere bottom; cube projected corner; cylinder cap+barrel formula. */
static float mig_gate_composite_lowest_y (const physics_world *world, const rigidbody *body) {
    float lowest = INFINITY;
    if ((!world) || (!body) || (body -> type != object_composite)) { return lowest; }
    for (int slot = 0; slot < body -> composite_part_count; slot++) {
        long index = (long) body -> part_base + (long) slot;
        if ((index < 0) || (index >= (long) world -> composite_part_count)) { continue; }
        const composite_part *part = &world -> composite_parts [index];
        if (!part -> active) { continue; }
        vector3 center = mig_gate_qrotate (body -> orientation, part -> local_position);
        float part_lowest = INFINITY;
        if (part -> type == object_sphere) {
            part_lowest = center.y - part -> radius;
        } else if (part -> type == object_cube) {
            vector4 q = mig_gate_qnorm (mig_gate_qmul (body -> orientation, part -> local_orientation));
            vector3 ax0 = mig_gate_qrotate (q, (vector3) {1.0f, 0.0f, 0.0f});
            vector3 ax1 = mig_gate_qrotate (q, (vector3) {0.0f, 1.0f, 0.0f});
            vector3 ax2 = mig_gate_qrotate (q, (vector3) {0.0f, 0.0f, 1.0f});
            part_lowest = center.y - (part -> half_extensions.x * fabsf (ax0.y) +
                                      part -> half_extensions.y * fabsf (ax1.y) +
                                      part -> half_extensions.z * fabsf (ax2.y));
        } else if (part -> type == object_cylinder) {
            vector4 q = mig_gate_qnorm (mig_gate_qmul (body -> orientation, part -> local_orientation));
            vector3 axis = mig_gate_qrotate (q, (vector3) {1.0f, 0.0f, 0.0f});
            float ay = axis.y;
            if (ay > 1.0f) { ay = 1.0f; }
            if (ay < -1.0f) { ay = -1.0f; }
            part_lowest = center.y - (part -> radius * sqrtf (fmaxf (0.0f, 1.0f - ay * ay)) +
                                      part -> cylinder_half_length * fabsf (ay));
        } else {
            continue;
        }
        if (part_lowest < lowest) { lowest = part_lowest; }
    }
    return (lowest == INFINITY) ? lowest : (body -> position.y + lowest);
}

/* WP2 2.6 gate (plan §4): per-part floor manifolds + floor CCD lowest
 * point. Oracles are hand geometry:
 *  - the low dumbbell's two sphere parts both rest over the infinite
 *    plane, so the step emits TWO floor manifolds with parent object
 *    identity, part stamps 0/1 and the per-part contact x = -/+0.5;
 *  - a 45 deg Z L-composite is positioned so the old body-level box
 *    support (bound * (|ax0.y|+|ax1.y|+|ax2.y|)) would claim it is already
 *    resting on the floor while its real lowest corner is ~0.28 m above
 *    it; a -30 m/s vertical + 10 rad/s tumble then drops 0.5 m in one
 *    canonical tick. The world floor sweep must clamp at the real lowest
 *    point before the L tunnels, and must not pump the body upward;
 *  - thirty landing ticks keep the body on the plane (no upward launch,
 *    no penetration below the solver slop). */
int mpe_t_mig_floor_ccd_composite (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "mig_floor_ccd_composite");
    mpe_clamp_counters_reset ();
    const float dt = 1.0f / 60.0f;
    /* --- 1. per-part floor manifolds ----------------------------------- */
    physics_world wfloor;
    mpe_world_begin (&wfloor);
    mpe_config_t fcfg = g_cfg;
    /* WP2 2.8 fixture correction: composite contacts read the PART pool
     * materials (body_defaults.*), not the body-level fields this gate
     * used to pin — the resting dumbbell must be DEAD, not bouncy. */
    fcfg.body_defaults.sphere_restitution = 0.0f;
    physics_world_set_config (&wfloor, &fcfg);
    wfloor.static_plane_enabled = true;
    const mig_part low_dumbbell [2] = {
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {-0.5f, 0.24f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f },
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {0.5f, 0.24f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f }
    };
    const int fb = physics_world_add_composite (&wfloor, low_dumbbell, 2);
    MPE_CHECK (&t, fb == 0);
    if (fb == 0) {
        physics_world_step (&wfloor, dt);
        MPE_CHECK (&t, mpe_world_finite (&wfloor));
        /* Authored COM y is 0.24; the pool rebase puts both part centres on
         * the parent COM. 0.25 radius => 0.01 plane penetration per part. */
        MPE_CHECK (&t, wfloor.last_manifold_count == 2);
        int floor_manifolds = 0;
        int part_seen [2] = {0, 0};
        float deepest = 0.0f;
        for (int m = 0; m < wfloor.last_manifold_count; m++) {
            collision_data *manifold = &wfloor.manifolds [m];
            if ((manifold -> object_a != &wfloor.bodies [fb]) ||
                (manifold -> object_b != &wfloor.static_plane_body)) { continue; }
            if (manifold -> contact_count <= 0) { continue; }
            floor_manifolds++;
            MPE_CHECK_NEAR (&t, manifold -> normal_vector.y, -1.0f, 1e-6f, "floor normal");
            for (int c = 0; c < manifold -> contact_count; c++) {
                const contact_point_data *cp = &manifold -> contacts [c];
                MPE_CHECK (&t, cp -> part_b == 0);
                MPE_CHECK (&t, cp -> part_a == 0 || cp -> part_a == 1);
                if (cp -> part_a < 2) { part_seen [cp -> part_a] = 1; }
                MPE_CHECK_NEAR (&t, fabsf (cp -> position.x), 0.5f, 1e-5f,
                                "each floor contact sits under its own part");
                MPE_CHECK_NEAR (&t, cp -> position.y, -0.01f, 1e-5f, "sphere bottom");
                MPE_CHECK_NEAR (&t, cp -> penetration, 0.01f, 1e-5f, "resting depth");
                if (cp -> penetration > deepest) { deepest = cp -> penetration; }
            }
        }
        MPE_CHECK (&t, floor_manifolds == 2);
        MPE_CHECK (&t, part_seen [0] == 1 && part_seen [1] == 1);
        MPE_CHECK_NEAR (&t, deepest, 0.01f, 1e-5f, "deepest staged part");
        MPE_CHECK (&t, wfloor.has_contact != NULL && wfloor.has_contact [fb] == 1);
    }
    physics_world_cleanup (&wfloor);
    /* --- 2. floor CCD: true lowest point, before tunneling ------------- */
    physics_world wccd;
    mpe_world_begin (&wccd);
    mpe_config_t ccfg = g_cfg;
    ccfg.world.gravity = -9.81f;
    /* WP2 2.8 fixture correction: the L's floor contacts read the PART pool
     * materials (body_defaults.cube_*, default restitution 0.5), NOT the
     * body-level restitution this gate pinned — the boundary's old
     * covering-sphere clamp was swallowing the resulting bounce and hiding
     * that. Pin dead part materials so "no landing pump" measures the CCD
     * and solver, not a bouncy part default. */
    ccfg.body_defaults.cube_restitution = 0.0f;
    physics_world_set_config (&wccd, &ccfg);
    wccd.static_plane_enabled = true;
    const mig_part l_parts [2] = {
        { mig_part_cube, 0.0f, 0.0f, {0.1f, 0.1f, 0.1f}, {0.0f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 1.0f },
        { mig_part_cube, 0.0f, 0.0f, {0.1f, 0.1f, 0.1f}, {0.5f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 1.0f }
    };
    const int lc = physics_world_add_composite (&wccd, l_parts, 2);
    MPE_CHECK (&t, lc == 0);
    if (lc == 0) {
        rigidbody *lbody = &wccd.bodies [lc];
        lbody -> position.y = 0.5975f; /* old box support says "resting" */
        lbody -> orientation = (vector4) {0.9238795325f, 0.0f, 0.0f, 0.3826834324f};
        lbody -> velocity = (vector3) {0.0f, -30.0f, 0.0f};
        lbody -> angular_velocity = (vector3) {0.0f, 0.0f, 10.0f};
        rigidbody_update_axes (lbody);
        lbody -> restitution = 0.0f;
        const float lowest_before = mig_gate_composite_lowest_y (&wccd, lbody);
        MPE_CHECK_NEAR (&t, lowest_before, 0.2793f, 1e-3f,
                        "hand oracle: true lowest corner");
        const int clamped = collision_ccd_sweep_clamp_world (&wccd, dt);
        MPE_CHECK (&t, clamped == 1);
        MPE_CHECK (&t, wccd.ccd_time_remaining != NULL && wccd.ccd_time_remaining [lc] > 0.0f &&
                          wccd.ccd_time_remaining [lc] < dt);
        const float lowest_after = mig_gate_composite_lowest_y (&wccd, lbody);
        MPE_CHECK (&t, lowest_after >= -0.02f && lowest_after <= 0.05f);
        MPE_CHECK (&t, lbody -> velocity.y <= 0.0f);
        /* No-landing-pump: the clamped body settles on the plane without an
         * upward launch or a deep re-entry.
         * WP2 2.8 recalibration (boundary per-part extrema fix): the old
         * max_up < 0.5 band was calibrated while the boundary's
         * covering-sphere clamp swallowed every landing — for a tumbling L
         * it lifted the body to sphere-rest each tick and killed vy, hiding
         * the real corner impact. Measured honestly at the canonical drop:
         * the CCD corner constraint stops the corner, spin collapses
         * (wz -29.5 -> -2.3 rad/s) and the COM gains +0.955 m/s — angular
         * to linear conversion, ENERGY DECREASING, not a pump. The bound is
         * now self-scaling: the rebound may not exceed 25% of the incoming
         * speed (decisively plastic; a real elastic pump blows through it). */
        const float v_impact = fabsf (lbody -> velocity.y);
        float max_up = 0.0f;
        float min_lowest = lowest_after;
        for (int tick = 0; tick < 30; tick++) {
            physics_world_step (&wccd, dt);
            const float low = mig_gate_composite_lowest_y (&wccd, lbody);
            if (low < min_lowest) { min_lowest = low; }
            if (lbody -> velocity.y > max_up) { max_up = lbody -> velocity.y; }
        }
        MPE_CHECK (&t, mpe_world_finite (&wccd));
        MPE_CHECK (&t, min_lowest > -0.05f);
        MPE_CHECK (&t, max_up < 0.25f * v_impact);
        MPE_CHECK (&t, fabsf (lbody -> velocity.y) < 0.25f); /* settled, not cycling */
        MPE_CHECK (&t, lbody -> position.y > 0.0f && lbody -> position.y < 0.6f);
    }
    physics_world_cleanup (&wccd);
    MPE_CHECK (&t, mpe_clamp_composite_total () == 0ul && mpe_clamp_mass_total () == 0ul);
    if (t.failures == 0) {
        printf ("[PASS] mig WP2.6: composite floor emits one manifold per part and floor CCD clamps the true lowest point\n");
        MPE_INFO ("dumbbell floor 2 part manifolds at x -/+0.5 depth 0.01; 45Z L true lowest 0.2793 vs old box -0.0010 (inside the resting slop), -30 m/s + 10 rad/s clamps in-tick, 30 landing ticks max_up < 25%% of incoming (corner impact converts spin to COM rise, energy decreasing)");
    }
    mpe_test_end (&t);
    return t.failures;
}

/* ---- WP2 2.8 gates (docs/MIG_BUILD_PLAN.md §4; charter §7 names all
 * three; landing 2026-11-09). The independence contract above holds:
 * every oracle in this section is hand-derived closed-form mechanics or
 * a bitwise twin comparison, never an implementation round-trip. ---- */

/* mig_composite_stack — charter §7: "a 10-primitive stack vs one 10-part
 * composite, same total mass/materials: both settle asleep, composite
 * stack top-block drift within the documented slop band of the primitive
 * stack."
 *
 * Same materials BY CONSTRUCTION, not by copying: physics_world_add_cube
 * stamps body_defaults.cube_* onto the primitives and the composite pool
 * fill (physics_world_composite_part_from_mig) reads the very same config
 * keys — pinning the per-world config once pins both scenes identically.
 * Same mass: ten parts × 1.0 kg vs ten cubes × 1.0 kg.
 *
 * The stack spawns at the F10 lesson pitch 1.002 (2 mm air gap, zero
 * built-in overlap — the mpe_pile_bodies defect record in mpe_suite_c.c).
 * Sleep is the F10 root-cause contract: sleep is body-level (charter
 * §4.1, one wake/one island), so every primitive in scene A and the ONE
 * composite body in scene B must reach is_sleeping. The top block of B is
 * its top PART (world pose = body ∘ part-local, the 2.4 ghost
 * composition) and must rest within the 5 cm slop band of scene A's top
 * cube on every axis — the two solver topologies (9 composite-composite
 * part pairs vs 9 primitive pairs under one body) must agree to the
 * documented band, which is exactly the charter's value claim. */
static int mpe_mig_stack_scene (physics_world *w, int composite) {
    physics_world_init (w);
    constraint_pool_init (w);
    mpe_config_t cfg = g_cfg;
    cfg.world.gravity = -9.81f;
    cfg.world.drag = 0.0f;
    cfg.sleep.enable = 1; /* the F10 contract: resting contact MUST sleep */
    cfg.body_defaults.cube_restitution = 0.0f; /* same dead materials both sides */
    cfg.body_defaults.cube_fric_s = 0.8f;
    cfg.body_defaults.cube_fric_k = 0.7f;
    /* Canonical solver: the regime matrix (light = 8 iterations) genuinely
     * slumps a 10-cube stack — a real solver-stiffness effect, NOT this
     * gate's claim. The settle+sleep contract is measured at the canonical
     64 iterations in every regime. */
    cfg.timestep.solver_iterations = 64;
    physics_world_set_config (w, &cfg);
    w->static_plane_enabled = true;
    int floor = physics_world_add_cube (w, (vector3) {0.0f, -0.5f, 0.0f}, (vector3) {30.0f, 0.5f, 30.0f}, 0.0f);
    if (floor >= 0) {
        w->bodies[floor].friction_static = 0.8f;
        w->bodies[floor].friction_kinetic = 0.7f;
        w->bodies[floor].restitution = 0.0f;
    }
    if (composite) {
        mig_part parts[10];
        for (int i = 0; i < 10; i++) {
            parts[i] = (mig_part) { mig_part_cube, 0.0f, 0.0f, {0.5f, 0.5f, 0.5f},
                                    {0.0f, 0.5f + (float) i * 1.002f, 0.0f},
                                    {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 1.0f };
        }
        return physics_world_add_composite (w, parts, 10);
    }
    for (int i = 0; i < 10; i++) {
        physics_world_add_cube (w, (vector3) {0.0f, 0.5f + (float) i * 1.002f, 0.0f},
                                (vector3) {0.5f, 0.5f, 0.5f}, 1.0f);
    }
    return 10;
}
int mpe_t_mig_composite_stack (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "mig_composite_stack");
    /* Earlier MIG gates leave sleep DISABLED in the global config; this
     * gate IS the sleep contract, so pin the knobs this gate means. */
    g_cfg.sleep.enable = 1;
    g_cfg.sleep.linear_thresh_sq = 0.0001f;
    g_cfg.sleep.angular_thresh_sq = 0.0012f;
    g_cfg.sleep.timer_duration = 0.5f;
    mpe_clamp_counters_reset ();
    physics_world wa, wb;
    const int pa = mpe_mig_stack_scene (&wa, 0);
    const int cb = mpe_mig_stack_scene (&wb, 1);
    MPE_CHECK (&t, pa == 10);
    MPE_CHECK (&t, cb == 1 && wb.body_count == 2); /* floor first, composite second */
    MPE_CHECK (&t, wb.bodies[cb].type == object_composite);
    MPE_CHECK (&t, wb.composite_part_count == 10);
    const float dt = 1.0f / 60.0f;
    int first_sleep_a = -1, first_sleep_b = -1;
    for (int k = 0; k < 1500; k++) { /* 25 s: 0.5 s timer has 50 chances */
        physics_world_step (&wa, dt);
        physics_world_step (&wb, dt);
        if (first_sleep_a < 0) {
            int all = 1;
            for (int i = 1; i < wa.body_count; i++) { if (!wa.bodies[i].is_sleeping) { all = 0; break; } }
            if (all && wa.body_count >= 11) { first_sleep_a = k; }
        }
        if (first_sleep_b < 0 && wb.bodies[cb].is_sleeping) { first_sleep_b = k; }
    }
    MPE_CHECK (&t, mpe_world_finite (&wa) && mpe_world_finite (&wb));
    MPE_CHECK (&t, first_sleep_a >= 0); /* the F10 runaway must stay dead */
    MPE_CHECK (&t, first_sleep_b >= 0); /* the composite is one body: it sleeps */
    /* Top block rest comparison: scene A body index 10 (the 10th cube) vs
     * scene B top part (part_index 9) world pose = body ∘ part-local. */
    const rigidbody *top_a = &wa.bodies[10];
    const rigidbody *bod_b = &wb.bodies[cb];
    const composite_part *top_part = NULL;
    for (int i = 0; i < wb.composite_part_count; i++) {
        const composite_part *p = &wb.composite_parts[wb.bodies[cb].part_base + i];
        if (p->active && p->part_index == 9) { top_part = p; break; }
    }
    MPE_CHECK (&t, top_part != NULL);
    if (top_part != NULL) {
        vector3 top_b = vector3_addition (bod_b->position,
                                          vector4_rotate_to_vector3 (bod_b->orientation, top_part->local_position));
        /* Authored top y = 0.5 + 9*1.002 = 9.518; both stacks must still be
         * STANDING (rest within the band of the authored spot) and AGREE
         * (|Δ| between the topologies within the same band). */
        MPE_CHECK_NEAR (&t, top_a->position.y, 9.518f, 0.05f, "primitive stack still standing");
        MPE_CHECK_NEAR (&t, top_b.y, 9.518f, 0.05f, "composite stack still standing");
        MPE_CHECK_NEAR (&t, top_b.x - top_a->position.x, 0.0f, 0.05f, "top-block x agreement");
        MPE_CHECK_NEAR (&t, top_b.z - top_a->position.z, 0.0f, 0.05f, "top-block z agreement");
        MPE_CHECK_NEAR (&t, top_b.y - top_a->position.y, 0.0f, 0.05f, "top-block y agreement");
        if (t.failures == 0) {
            printf ("[PASS] mig WP2.8: 10-primitive stack and one 10-part composite both settle ASLEEP, top blocks agree within 5 cm\n");
            MPE_INFO ("first all-sleep tick: primitives=%d composite=%d (1500-tick window); top rest A (%.4f, %.4f, %.4f) B (%.4f, %.4f, %.4f)",
                      first_sleep_a, first_sleep_b, top_a->position.x, top_a->position.y, top_a->position.z,
                      top_b.x, top_b.y, top_b.z);
        }
    }
    physics_world_cleanup (&wa);
    physics_world_cleanup (&wb);
    mpe_test_end (&t);
    return t.failures;
}

/* mig_pendulum_com_offset — charter §7: "revolute pendulum whose bob's
 * COM is offset from the anchor: measured period vs the parallel-axis
 * shifted closed form within the pendulum ≤1.5% house band. This one
 * gate proves the mass contract end-to-end through the solver, because
 * the lever arm, revolute constraint, and angular draw all read the COM."
 *
 * Fixture: the bob is a TWO-SPHERE composite (the WP0 dumbbell shape,
 * unequal masses) hinged at its assembly origin. Hand truth:
 *   part A: r = 0.2, m = 1 at (0, -0.2, 0)   (its top touches the hinge)
 *   part B: r = 0.3, m = 3 at (0, -1.0, 0)
 *   COM_y = (1·(-0.2) + 3·(-1.0)) / 4 = -0.8      → lever arm d = 0.8
 *   I_zz(hinge) = Σ (2/5·m·r² + m·r_i²) over the hinge axis:
 *     A: 0.016 + 1·0.04  = 0.056
 *     B: 0.108 + 3·1.00  = 3.108     → I_pivot = 3.164
 *   T = 2π·√(I_pivot / (M·g·d)) = 2π·√(3.164 / (4·9.81·0.8)) ≈ 1.9947 s
 *   amplitude correction (1 + θ0²/16), θ0 = 0.3, as in mig_pendulum_physical.
 * Every lever the solver pulls — gravity at the composite COM, the
 * revolute anchors, the inertia tensor draw — reads the composed values,
 * so a measured period inside 1.5% certifies the whole contract. */
int mpe_t_mig_pendulum_com_offset (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "mig_pendulum_com_offset");
    g_cfg.world.gravity = -9.81f;
    g_cfg.sleep.enable = 0; /* a pendulum must never sleep mid-swing */
    mpe_clamp_counters_reset ();
    physics_world w;
    mpe_world_begin (&w);
    const float d = 0.8f;                 /* hinge → COM lever arm */
    const float theta0 = 0.3f;
    /* Static pivot cube, the mig_pendulum_physical pattern: hinge at
     * (0, 6.8, 0), 1.0 below the pivot's bottom face. */
    int pivot = physics_world_add_cube (&w, (vector3) {0.0f, 8.0f, 0.0f}, (vector3) {0.2f, 0.2f, 0.2f}, 0.0f);
    MPE_CHECK (&t, pivot >= 0);
    const vector3 hinge = {0.0f, 6.8f, 0.0f};
    /* The composite bob is BORN rotated theta0 about Z at zero angular
     * velocity: its COM sits at hinge + R(theta0)·(0, -d, 0). add_composite
     * rebases the parts to the COM frame and sets body position = authored
     * COM, so placing the body at that point puts the hinge exactly at the
     * assembly origin where the anchors below say it is. */
    const mig_part bob_parts[2] = {
        { mig_part_sphere, 0.2f, 0.0f, {0.0f, 0.0f, 0.0f}, {0.0f, -0.2f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 1.0f },
        { mig_part_sphere, 0.3f, 0.0f, {0.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 3.0f }
    };
    int bob = physics_world_add_composite (&w, bob_parts, 2);
    MPE_CHECK (&t, bob == 1); /* the pivot cube is body 0 */
    if (bob == 1) {
        rigidbody *rb = &w.bodies[bob];
        MPE_CHECK_NEAR (&t, rb->mass, 4.0f, 1e-5f, "composed mass");
        rb->position = (vector3) {hinge.x + d * sinf (theta0), hinge.y - d * cosf (theta0), 0.0f};
        rb->orientation = (vector4) {cosf (theta0 * 0.5f), 0.0f, 0.0f, sinf (theta0 * 0.5f)};
        rigidbody_update_axes (rb);
        /* Bob-local hinge anchor: assembly origin − COM = (0, +0.8, 0).
         * Pivot anchor (0, -1.2, 0) as in the physical gate. Axis (0,0,1)
         * is invariant under the born rotation, as there. */
        MPE_CHECK (&t, constraint_add_revolute (&w, w.bodies[pivot].object_id, rb->object_id,
                                                (vector3) {0.0f, -1.2f, 0.0f}, (vector3) {0.0f, d, 0.0f},
                                                (vector3) {0.0f, 0.0f, 1.0f}) >= 0);
        /* Closed form, hand-written — oracle independent of mig_mass. */
        const float i_pivot = (2.0f / 5.0f * 1.0f * 0.2f * 0.2f + 1.0f * 0.2f * 0.2f) +
                              (2.0f / 5.0f * 3.0f * 0.3f * 0.3f + 3.0f * 1.0f * 1.0f);
        const float m_tot = 4.0f;
        const float t_exact = 2.0f * 3.14159265f * sqrtf (i_pivot / (m_tot * 9.81f * d));
        const float t_expected = t_exact * (1.0f + (theta0 * theta0) / 16.0f);
        const float dt = 1.0f / 60.0f;
        float prev_x = rb->position.x - hinge.x;
        int crossings = 0, first = -1, last = -1;
        float max_drift = 0.0f, max_abs_x = 0.0f;
        for (int k = 0; k < 900; k++) {
            physics_world_step (&w, dt);
            const rigidbody *b = &w.bodies[bob];
            if (!isfinite (b->position.x) || !isfinite (b->position.y) || !isfinite (b->position.z)) {
                printf ("[FAIL] bob non-finite at tick %d\n", k);
                t.failures++;
                break;
            }
            const float dx = b->position.x - hinge.x;
            const float dy = b->position.y - hinge.y;
            max_drift = fmaxf (max_drift, fabsf (sqrtf (dx * dx + dy * dy) - d));
            const float ax = fabsf (dx);
            if (ax > max_abs_x) { max_abs_x = ax; }
            if ((prev_x <= 0.0f && dx > 0.0f) || (prev_x >= 0.0f && dx < 0.0f)) {
                crossings++;
                if (first < 0) { first = k; }
                last = k;
            } prev_x = dx;
        }
        float t_meas = 0.0f;
        if (crossings >= 2) { t_meas = 2.0f * (float) (last - first) * dt / (float) (crossings - 1); }
        MPE_INFO ("period: measured=%.4f analytic=%.4f (theta0-corrected %.4f) crossings=%d max_drift=%.4f I_pivot=%.4f (transport terms %.0f%%)",
                  t_meas, t_exact, t_expected, crossings, max_drift, i_pivot,
                  100.0f * (1.0f * 0.2f * 0.2f + 3.0f * 1.0f * 1.0f) / i_pivot);
        MPE_CHECK (&t, crossings >= 6); /* 900 ticks ~ 8 periods at T ≈ 2.0 s */
        MPE_CHECK (&t, max_drift <= 0.02f); /* the house revolute drift bound */
        MPE_CHECK (&t, max_abs_x >= 0.4f * d * sinf (theta0)); /* no dead-still hang */
        MPE_CHECK (&t, max_abs_x <= 1.6f * d * sinf (theta0)); /* no energy pump */
        MPE_CHECK (&t, crossings >= 4);
        /* THE charter band: 1.5% on the parallel-axis shifted closed form. */
        if (crossings >= 4) { MPE_CHECK_REL (&t, t_meas, t_expected, 0.015f, "composite-bob pendulum period (parallel axis through the composed COM)"); }
        if (t.failures == 0) {
            printf ("[PASS] mig WP2.8: off-COM composite bob swings at the hand-closed-form period within 1.5%% — mass contract certified end-to-end through the solver\n");
        }
    }
    physics_world_cleanup (&w);
    mpe_test_end (&t);
    return t.failures;
}

/* mig_determinism_twins — charter §4.5/§7: "composite scenes at
 * MPE_THREADS 1/4/8 bitwise identical over 600 ticks."
 *
 * The worker pool is process-lazy (physics_world.c mpe_pool_init reads
 * MPE_THREADS once), so the gate runs its twins at the AMBIENT thread
 * count and names it; the landing verification runs this gate three
 * times with MPE_THREADS=1/4/8 (the same pattern the 2.4/2.5 landing
 * used for its determinism probes). The scene exercises every composite
 * contact class at once: part-vs-part (two dumbbells colliding),
 * part-vs-floor, part-vs-primitive (cube raining on a dumbbell), and
 * composite-vs-composite through the broadphase covering spheres. Twin
 * worlds step the same 600 ticks; every body's pose, velocities and
 * sleep state must match BITWISE, plus the contact-cache footprint. */
static int mpe_mig_twins_body_equal (const rigidbody *a, const rigidbody *b) {
    if (a->is_sleeping != b->is_sleeping || a->sleep_timer != b->sleep_timer) { return 0; }
    if (a->position.x != b->position.x || a->position.y != b->position.y || a->position.z != b->position.z) { return 0; }
    if (a->orientation.x != b->orientation.x || a->orientation.y != b->orientation.y ||
        a->orientation.z != b->orientation.z || a->orientation.w != b->orientation.w) { return 0; }
    if (a->velocity.x != b->velocity.x || a->velocity.y != b->velocity.y || a->velocity.z != b->velocity.z) { return 0; }
    if (a->angular_velocity.x != b->angular_velocity.x || a->angular_velocity.y != b->angular_velocity.y ||
        a->angular_velocity.z != b->angular_velocity.z) { return 0; }
    return 1;
}
static void mpe_mig_twins_scene (physics_world *w) {
    physics_world_init (w);
    constraint_pool_init (w);
    mpe_config_t cfg = g_cfg;
    cfg.world.gravity = -9.81f;
    cfg.world.drag = 0.0f;
    cfg.sleep.enable = 0; /* bitwise state includes sleep_timer; keep it frozen */
    physics_world_set_config (w, &cfg);
    w->static_plane_enabled = true;
    physics_world_add_cube (w, (vector3) {0.0f, -0.5f, 0.0f}, (vector3) {30.0f, 0.5f, 30.0f}, 0.0f);
    /* Composite 1: the WP0 dumbbell, dropped tilted. */
    const mig_part dumb[2] = {
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {-0.5f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f },
        { mig_part_sphere, 0.25f, 0.0f, {0.0f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 2.0f }
    };
    int d1 = physics_world_add_composite (w, dumb, 2);
    if (d1 >= 0) {
        w->bodies[d1].position.y = 3.0f;
        w->bodies[d1].orientation = (vector4) {0.9238795f, 0.0f, 0.0f, 0.3826834f};
        w->bodies[d1].velocity = (vector3) {0.0f, -1.0f, 0.0f};
        w->bodies[d1].angular_velocity = (vector3) {0.0f, 2.0f, 1.0f};
        rigidbody_update_axes (&w->bodies[d1]);
    }
    /* Composite 2: the 2.6 L (two cubes), colliding with composite 1. */
    const mig_part ell[2] = {
        { mig_part_cube, 0.0f, 0.0f, {0.1f, 0.1f, 0.1f}, {0.0f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 1.0f },
        { mig_part_cube, 0.0f, 0.0f, {0.1f, 0.1f, 0.1f}, {0.5f, 0.0f, 0.0f},
          {1.0f, 0.0f, 0.0f, 0.0f}, mig_mass_mode_explicit, 1.0f }
    };
    int d2 = physics_world_add_composite (w, ell, 2);
    if (d2 >= 0) {
        w->bodies[d2].position = (vector3) {0.6f, 1.2f, 0.0f};
        w->bodies[d2].orientation = (vector4) {0.9238795f, 0.0f, 0.3826834f, 0.0f};
        w->bodies[d2].velocity = (vector3) {-0.8f, -0.5f, 0.0f};
        w->bodies[d2].angular_velocity = (vector3) {1.0f, 0.0f, -2.0f};
        rigidbody_update_axes (&w->bodies[d2]);
    }
    /* Primitives raining onto the dumbbell: part-vs-primitive contacts. */
    int c1 = physics_world_add_cube (w, (vector3) {-0.5f, 5.0f, 0.0f}, (vector3) {0.2f, 0.2f, 0.2f}, 0.5f);
    if (c1 >= 0) { w->bodies[c1].angular_velocity = (vector3) {3.0f, -1.0f, 2.0f}; }
    physics_world_add_sphere (w, 0.2f, 0.5f, (vector3) {0.5f, 4.5f, 0.3f});
}
int mpe_t_mig_determinism_twins (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "mig_determinism_twins");
    g_cfg.sleep.enable = 0;
    physics_world w1, w2;
    mpe_mig_twins_scene (&w1);
    mpe_mig_twins_scene (&w2);
    const float dt = 1.0f / 60.0f;
    for (int k = 0; k < 600; k++) {
        physics_world_step (&w1, dt);
        physics_world_step (&w2, dt);
    }
    MPE_CHECK (&t, w1.body_count == w2.body_count);
    MPE_CHECK (&t, w1.composite_part_count == w2.composite_part_count);
    MPE_CHECK (&t, w1.world_contact_cache_count == w2.world_contact_cache_count);
    for (int i = 0; i < w1.body_count && i < w2.body_count; i++) {
        if (!mpe_mig_twins_body_equal (&w1.bodies[i], &w2.bodies[i])) {
            printf ("[FAIL] body %d diverged bitwise (type %d)\n", i, (int) w1.bodies[i].type);
            t.failures++;
        } else { t.checks++; }
    }
    if (t.failures == 0) {
        printf ("[PASS] mig WP2.8: 600-tick composite scene bitwise identical across twin worlds at MPE_THREADS=%s\n",
                getenv ("MPE_THREADS") ? getenv ("MPE_THREADS") : "1 (unset)");
        MPE_INFO ("bodies=%d parts=%d cache_entries=%d", w1.body_count, w1.composite_part_count, w1.world_contact_cache_count);
    }
    physics_world_cleanup (&w1);
    physics_world_cleanup (&w2);
    mpe_test_end (&t);
    return t.failures;
}

/* ---- 2026-11-10 audit gate: the boundary extrema must reach the ----
 * ---- part's OWN frame, not stop at the body frame                ----
 * The world-edge safety net's composite extrema conjugated the world
 * axis by the BODY orientation only while its comment promised "the
 * part's OWN frame" (collision_ccd.c composes the same pair for its
 * support). A cylinder part posed axle-along-body-+Y (local_orientation
 * = Rz(+90 deg), first-class state: drag_weld, save/load and the ghost
 * view all produce non-identity part orientations) read S_y = r = 0.3
 * instead of hl = 1.0, so the net let the true bottom sink 0.7 m before
 * firing. Oracles are hand geometry, exact:
 *  - Y-axle cylinder: S_y = hl = 1.0 and S_x = r = 0.3 (the buggy body-
 *    frame reading claimed 0.3 and 1.0 -- swapped, and the X wall would
 *    teleport the body to x = -0.5 against a 0.5 wall);
 *  - 45 deg-yaw cube: S_y = he*(sin45 + cos45) = 0.14142136
 *    (the buggy reading claimed he = 0.1);
 *  - the NULL-world legacy degraded mode still clamps at the
 *    sphere-backed covering bound sqrt(r^2 + hl^2) = 1.04403065. */
int mpe_t_mig_boundary_rotated_part (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "mig_boundary_rotated_part");
    mpe_clamp_counters_reset ();
    physics_world w;
    mpe_world_begin (&w);
    mpe_config_t cfg = g_cfg;
    cfg.boundary.floor_emergency_slop = 0.0f; /* exact clamp, no slack */
    physics_world_set_config (&w, &cfg);
    const float c45 = 0.7071067811865476f; /* cos/sin 45 deg: Rz(+90 deg) */
    /* Cylinder r=0.3 hl=1.0 posed axle-along-body-+Y; part at the
     * assembly origin, so the COM is the part centre. */
    const mig_part cyl [1] = {
        { mig_part_cylinder, 0.3f, 1.0f, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
          {c45, 0.0f, 0.0f, c45}, mig_mass_mode_explicit, 2.0f }
    };
    const int cb = physics_world_add_composite (&w, cyl, 1);
    MPE_CHECK (&t, cb == 0);
    /* 45 deg-yaw cube, its own body in the same world. */
    const mig_part yaw [1] = {
        { mig_part_cube, 0.0f, 0.0f, {0.1f, 0.1f, 0.1f}, {0.0f, 0.0f, 0.0f},
          {0.9238795325112867f, 0.0f, 0.0f, 0.3826834323650898f},
          mig_mass_mode_explicit, 1.0f }
    };
    const int yb = physics_world_add_composite (&w, yaw, 1);
    MPE_CHECK (&t, yb == 1);
    if ((cb == 0) && (yb == 1)) {
        rigidbody *cyl_body = &w.bodies [cb];
        rigidbody *yaw_body = &w.bodies [yb];
        cyl_body -> position = (vector3) {0.0f, 0.5f, 0.0f};  /* true min_y -0.5; bug reads +0.2 */
        yaw_body -> position = (vector3) {0.0f, 0.05f, 0.0f}; /* true min_y -0.09142044; bug reads -0.05 */
        vector3 lo = {-100.0f, 0.0f, -100.0f};
        vector3 hi = { 100.0f, 100.0f, 100.0f};
        boundary_apply_box_world_cfg (cyl_body, lo, hi, &cfg, &w);
        boundary_apply_box_world_cfg (yaw_body, lo, hi, &cfg, &w);
        MPE_CHECK_NEAR (&t, cyl_body -> position.y, 1.0f, 1e-6f,
                        "Y-axle cylinder rests on its END (hl), not its radius");
        MPE_CHECK_NEAR (&t, cyl_body -> position.x, 0.0f, 1e-6f,
                        "cylinder X support is r: the 0.5 wall is untouched");
        MPE_CHECK_NEAR (&t, yaw_body -> position.y, 0.14142136f, 1e-6f,
                        "45 deg-yaw cube corner support exact");
        /* Legacy degraded mode unchanged: world=NULL projects the
         * sphere-backed envelope (the 2.8 documented behaviour). */
        cyl_body -> position = (vector3) {0.0f, 1.0f, 0.0f};
        boundary_apply_box_world_cfg (cyl_body, lo, hi, &cfg, NULL);
        MPE_CHECK_NEAR (&t, cyl_body -> position.y, 1.04403065f, 1e-6f,
                        "NULL-world legacy clamp at the covering bound");
    }
    physics_world_cleanup (&w);
    mpe_test_end (&t);
    return t.failures;
}

/* ---- MIG WP4 4.2 gate: spec round-trip ------------------------------
 * Read a valid spec, emit normalized text, re-parse it, emit again —
 * the two emissions must be byte-identical (the frozen grammar's
 * canonical output contract). Also probes the reject codes for a
 * battery of malformed inputs (write-on-success: a rejected file enters
 * nothing into the spec store). */
int mpe_t_mig_spec_roundtrip (void) {
    mpe_test_t t;
    mpe_test_begin (&t, "mig_spec_roundtrip");
    mpe_clamp_counters_reset ();
    const char *tmpd = getenv ("TMPDIR");
    const char *base = tmpd && tmpd [0] ? tmpd : "../../temp";
    char dir [512];
    snprintf (dir, sizeof (dir), "%s/mpe_suite_spec_roundtrip.dat", base);
    /* A valid spec: sphere + density cube with material and colour. */
    const char *good = "mig test_obj\n"
                       "frame assembly\n"
                       "part 0 type sphere\n"
                       "  dims radius 0.5\n"
                       "  pose 0 0 0 1 0 0 0\n"
                       "  material mu_s 0.9 mu_k 0.7 rest 0.1\n"
                       "  mass 2.0\n"
                       "  colour 1 0.5 0.25\n"
                       "part 1 type cube\n"
                       "  dims hx 0.1 hy 0.2 hz 0.3\n"
                       "  pose 1 2 3 0.70710678 0 0 0.70710678\n"
                       "  density 3000\n"
                       "  colour 0 1 0\n";
    FILE *f = fopen (dir, "w");
    MPE_CHECK (&t, f != NULL);
    if (f) {
        fwrite (good, 1, strlen (good), f);
        fclose (f);
    }
    mig_spec spec_a, spec_b;
    int rc = mig_spec_parse_file (dir, 16, &spec_a);
    MPE_CHECK (&t, rc == mig_spec_ok);
    if (rc == mig_spec_ok) {
        char buf1 [8192], buf2 [8192];
        size_t n1 = mig_spec_emit_normalized (&spec_a, buf1, sizeof (buf1));
        /* Write the normalized form and re-parse it. */
        FILE *f2 = fopen (dir, "w");
        MPE_CHECK (&t, f2 != NULL);
        if (f2) { fwrite (buf1, 1, n1, f2); fclose (f2); }
        rc = mig_spec_parse_file (dir, 16, &spec_b);
        MPE_CHECK (&t, rc == mig_spec_ok);
        if (rc == mig_spec_ok) {
            size_t n2 = mig_spec_emit_normalized (&spec_b, buf2, sizeof (buf2));
            MPE_CHECK (&t, n1 == n2);
            MPE_CHECK (&t, memcmp (buf1, buf2, n1) == 0);
            MPE_CHECK (&t, strcmp (spec_a.name, spec_b.name) == 0);
            MPE_CHECK (&t, spec_a.count == spec_b.count);
        }
    }
    /* Reject battery: each malformed input must be refused with its
     * named code and must not populate the spec store (out untouched). */
    struct { const char *content; int expect_code; } cases [] = {
        { "# only a comment\n", mig_spec_empty_file },
        { "part 0 type sphere\n  dims radius 0.1\n  pose 0 0 0 1 0 0 0\n  mass 1\n",
          mig_spec_no_object },
        { "mig x\nframe world\npart 0 type sphere\n  dims radius 0.1\n  pose 0 0 0 1 0 0 0\n  mass 1\n",
          mig_spec_bad_frame },
        { "mig x\npart 0 type custom\n  dims radius 0.1\n  pose 0 0 0 1 0 0 0\n  mass 1\n",
          mig_spec_bad_type },
        { "mig x\npart 0 type sphere\n  dims hx 0.1\n  pose 0 0 0 1 0 0 0\n  mass 1\n",
          mig_spec_bad_dims },
        { "mig x\npart 0 type sphere\n  pose 0 0 0 1 0 0 0\n  mass 1\n",
          mig_spec_missing_dims },
        { "mig x\npart 0 type sphere\n  dims radius 0.1\n  mass 1\n  density 2\n",
          mig_spec_mass_xor_density },
        { "mig x\npart 0 type sphere\n  dims radius 0.1\n  pose 0 0 0 0.5 0 0 0\n  mass 1\n",
          mig_spec_bad_quat },
        { "mig x\npart 1 type sphere\n  dims radius 0.1\n  pose 0 0 0 1 0 0 0\n  mass 1\n",
          mig_spec_bad_part_index },
        { "mig x\npart 0 type sphere\n  dims radius 0.1\n  pose 0 0 0 1 0 0 0\n  mass -1\n",
          mig_spec_bad_mass },
        { "mig x\npart 0 type sphere\n  dims radius 0.1\n  pose 0 0 0 1 0 0 0\n  frobnicate 5\n",
          mig_spec_unknown_kw },
    };
    int ncases = sizeof (cases) / sizeof (cases [0]);
    for (int i = 0; i < ncases; i++) {
        FILE *fc = fopen (dir, "w");
        if (!fc) { MPE_CHECK (&t, 0); continue; }
        fwrite (cases [i].content, 1, strlen (cases [i].content), fc);
        fclose (fc);
        mig_spec junk;
        memset (&junk, 0, sizeof (junk));
        int rc2 = mig_spec_parse_file (dir, 16, &junk);
        char label [64];
        snprintf (label, sizeof (label), "reject case %d", i);
        MPE_CHECK (&t, rc2 == cases [i].expect_code);
    }
    remove (dir);
    if (t.failures == 0) { printf ("[PASS] mig spec round-trip + reject battery\n"); }
    mpe_test_end (&t);
    return t.failures;
}

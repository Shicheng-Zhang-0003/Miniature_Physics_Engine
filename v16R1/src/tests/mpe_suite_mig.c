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
#include "core/rigidbody.h"
#include "core/mig_mass.h"

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
    }

    if (t.failures == 0) {
        printf ("[PASS] mig stage0: composition matches independent closed-form mechanics\n");
        MPE_INFO ("probes: dumbbell, L-Steiner offdiag, density mixing, 90-deg axis map, cross-type, rebase, 6 named errors");
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

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
#include "core/rigidbody.h"
#include "core/mig_mass.h"
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

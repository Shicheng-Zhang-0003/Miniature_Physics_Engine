/* MIG WP2.5 2.5.1: convex hull point collection — pure data, no physics.
 * See mig_hull.h for the discipline contract.
 */
#include "core/mig_hull.h"
#include <math.h>

/* Note: no fixed-π literal here — unlike mig_mass, no hull formula in
 * this TU touches π (polygon/tetra integrals are rational in the
 * coordinates). Keep it that way: any future formula that needs π must
 * define the fixed literal locally, never math.h's macro. */

mig_hull_err mig_hull_point_set_init (mig_hull_point_set *set,
                                      mig_hull_point *buf, int cap) {
    if ((!set) || (!buf) || (cap <= 0)) { return mig_hull_null; }
    set -> points = buf;
    set -> count = 0;
    set -> capacity = cap;
    return mig_hull_ok;
}

mig_hull_err mig_hull_point_set_add (mig_hull_point_set *set,
                                     double x, double y, double z) {
    if (!set) { return mig_hull_null; }
    if (set -> count >= set -> capacity) { return mig_hull_bad_count; }
    if (!isfinite (x) || !isfinite (y) || !isfinite (z)) {
        return mig_hull_bad_point; /* garbage rejection: point not added */
    }
    set -> points [set -> count].x = x;
    set -> points [set -> count].y = y;
    set -> points [set -> count].z = z;
    set -> count += 1;
    return mig_hull_ok;
}

mig_hull_err mig_hull_build_box (double hx, double hy, double hz,
                                 mig_hull_point_set *out) {
    if (!out) { return mig_hull_null; }
    if (!isfinite (hx) || !isfinite (hy) || !isfinite (hz) ||
        hx <= 0.0 || hy <= 0.0 || hz <= 0.0) {
        return mig_hull_bad_point;
    }
    /* 8 corners: (±hx, ±hy, ±hz) — enumeration order fixed for bitwise
     * reproducibility across builds and platforms. */
    static const int sgn [8][3] = {
        {+1, +1, +1}, {+1, +1, -1}, {+1, -1, +1}, {+1, -1, -1},
        {-1, +1, +1}, {-1, +1, -1}, {-1, -1, +1}, {-1, -1, -1},
    };
    for (int i = 0; i < 8; i++) {
        mig_hull_err e = mig_hull_point_set_add (out,
            (double) sgn [i][0] * hx,
            (double) sgn [i][1] * hy,
            (double) sgn [i][2] * hz);
        if (e != mig_hull_ok) { return e; }
    }
    return mig_hull_ok;
}

mig_hull_err mig_hull_build_tetra (double scale,
                                   mig_hull_point_set *out) {
    if (!out) { return mig_hull_null; }
    if (!isfinite (scale) || scale <= 0.0) { return mig_hull_bad_point; }
    /* Regular tetrahedron centred at origin: (±1,±1,±1) with an even
     * number of minus signs — all four edges equal 2·√2·scale exactly. */
    static const double v [4][3] = {
        { 1.0,  1.0,  1.0},
        { 1.0, -1.0, -1.0},
        {-1.0,  1.0, -1.0},
        {-1.0, -1.0,  1.0},
    };
    for (int i = 0; i < 4; i++) {
        mig_hull_err e = mig_hull_point_set_add (out,
            v [i][0] * scale, v [i][1] * scale, v [i][2] * scale);
        if (e != mig_hull_ok) { return e; }
    }
    return mig_hull_ok;
}

mig_hull_err mig_hull_point_set_validate (const mig_hull_point_set *set) {
    if (!set) { return mig_hull_null; }
    for (int i = 0; i < set -> count; i++) {
        const mig_hull_point *p = &set -> points [i];
        if (!isfinite (p -> x) || !isfinite (p -> y) || !isfinite (p -> z)) {
            return mig_hull_bad_point;
        }
    }
    return mig_hull_ok;
}

/* ---- WP2.5 2.5.2: the 2D proof ----
 * Green's theorem on a plane polygon, edge by edge. Each edge
 * i -> i+1 contributes the signed trapezoid weight
 * c_i = x_i·y_{i+1} − x_{i+1}·y_i, and the divergence-theorem field
 * choices give the moments:
 *   area      = 1/2  Σ c_i                              (F = (−y/2, x/2))
 *   ∫∫ x  dA  = 1/6  Σ (x_i + x_{i+1}) c_i              (F = (x²/2, 0))
 *   ∫∫ x² dA  = 1/12 Σ (x_i² + x_i x_{i+1} + x_{i+1}²) c_i
 *   ∫∫ xy dA  = 1/24 Σ (2x_i y_i + x_i y_{i+1} + x_{i+1} y_i
 *                      + 2x_{i+1} y_{i+1}) c_i
 * (y-moments mirror the x ones with x↔y swapped). Verified against the
 * unit right triangle (area 1/2, ∫x = 1/6, ∫x² = 1/12, ∫xy = 1/24) and
 * the rectangle [−a,a]×[−b,b] closed forms (4ab, 0, (4/3)a³b, 0). */
mig_hull_err mig_hull_polygon_props (const mig_hull_point *verts, int count,
                                     mig_hull_area2 *out) {
    if ((!verts) || (!out)) { return mig_hull_null; }
    if (count < 3) { return mig_hull_bad_count; }
    for (int i = 0; i < count; i++) {
        if (!isfinite (verts [i].x) || !isfinite (verts [i].y)) {
            return mig_hull_bad_point;
        }
    }
    double area = 0.0, mx = 0.0, my = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0;
    for (int i = 0; i < count; i++) {
        const mig_hull_point *p = &verts [i];
        const mig_hull_point *q = &verts [(i + 1) % count];
        const double c = p -> x * q -> y - q -> x * p -> y;
        area += c;
        mx  += (p -> x + q -> x) * c;
        my  += (p -> y + q -> y) * c;
        sxx += (p -> x * p -> x + p -> x * q -> x + q -> x * q -> x) * c;
        syy += (p -> y * p -> y + p -> y * q -> y + q -> y * q -> y) * c;
        sxy += (2.0 * p -> x * p -> y + p -> x * q -> y +
                q -> x * p -> y + 2.0 * q -> x * q -> y) * c;
    }
    /* EVERY accumulator is guarded, not just `area`: finite coordinates
     * near the double ceiling (e.g. x ≈ 1e200 against y ≈ 1e-200) can keep
     * the signed area cancellations finite while the all-positive second
     * moments overflow to +Inf. Writing Inf into `out` would be a silent
     * quality break; the named-error discipline refuses instead. */
    if (!isfinite (area) || !isfinite (mx) || !isfinite (my) ||
        !isfinite (sxx) || !isfinite (syy) || !isfinite (sxy)) {
        return mig_hull_bad_point;
    }
    out -> area      = 0.5 * area;
    out -> moment_x  = mx / 6.0;
    out -> moment_y  = my / 6.0;
    out -> second_xx = sxx / 12.0;
    out -> second_yy = syy / 12.0;
    out -> second_xy = sxy / 24.0;
    return mig_hull_ok;
}

/* ---- WP2.5 2.5.3: hull mass properties ----
 * Divergence theorem over face tetras anchored at the origin. For a face
 * (a, b, c) wound outward: signed dV = (a×b)·c / 6, and with the tetra
 * vertices {0, a, b, c} the monomial integrals are
 *   ∫ x_i  dV = (dV/4)·S_i                      (S_i = a_i+b_i+c_i)
 *   ∫ x_i x_j dV = (dV/20)·[S_i S_j + Q_ij]     (Q_ij = Σ_k k_i k_j)
 * both derived from the barycentric integrals ∫λ_k² dV = V/10,
 * ∫λ_k λ_l dV = V/20 (k ≠ l), then x = Σ λ_k v_k. Signed dVs cancel
 * across the closed surface, so accumulation order is face order.
 * Finally com = M/V, J_com = J − V·com⊗com, and
 * I = trace(J_com)·Id − J_com (L = I·ω). Verified against the solid box
 * (V = 8abc, I_xx = m(b²+c²)/3) and the regular tetra (±s,±s,±s)
 * even-minus (V = 8s³/3, I = m·a²/20 = 16s⁵/15 at density 1). */
mig_hull_err mig_hull_mesh_props (const mig_hull_point *verts, int vert_count,
                                  const int *faces, int face_count,
                                  double density, mig_hull_mass *out) {
    if ((!verts) || (!faces) || (!out)) { return mig_hull_null; }
    if (vert_count < 4 || face_count < 4) { return mig_hull_bad_count; }
    if (!isfinite (density) || density <= 0.0) { return mig_hull_bad_mesh; }
    for (int i = 0; i < vert_count; i++) {
        if (!isfinite (verts [i].x) || !isfinite (verts [i].y) ||
            !isfinite (verts [i].z)) {
            return mig_hull_bad_point;
        }
    }
    for (int f = 0; f < face_count; f++) {
        for (int k = 0; k < 3; k++) {
            const int idx = faces [3 * f + k];
            if (idx < 0 || idx >= vert_count) { return mig_hull_bad_index; }
        }
    }
    double V = 0.0, M [3] = {0.0, 0.0, 0.0}, J [3][3] = {{0.0}};
    for (int f = 0; f < face_count; f++) {
        const mig_hull_point *tv [3] = {
            &verts [faces [3 * f + 0]],
            &verts [faces [3 * f + 1]],
            &verts [faces [3 * f + 2]],
        };
        const double abx = tv [0] -> y * tv [1] -> z - tv [0] -> z * tv [1] -> y;
        const double aby = tv [0] -> z * tv [1] -> x - tv [0] -> x * tv [1] -> z;
        const double abz = tv [0] -> x * tv [1] -> y - tv [0] -> y * tv [1] -> x;
        const double dV = (abx * tv [2] -> x + aby * tv [2] -> y +
                           abz * tv [2] -> z) / 6.0;
        double S [3] = {0.0, 0.0, 0.0};
        double Q [3][3] = {{0.0}};
        for (int k = 0; k < 3; k++) {
            const double kv [3] = {tv [k] -> x, tv [k] -> y, tv [k] -> z};
            for (int i = 0; i < 3; i++) {
                S [i] += kv [i];
                for (int j = 0; j < 3; j++) { Q [i][j] += kv [i] * kv [j]; }
            }
        }
        V += dV;
        for (int i = 0; i < 3; i++) { M [i] += dV * S [i] / 4.0; }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                J [i][j] += dV * (S [i] * S [j] + Q [i][j]) / 20.0;
            }
        }
    }
    if (!isfinite (V) || V <= 0.0) { return mig_hull_bad_mesh; }
    for (int i = 0; i < 3; i++) {
        if (!isfinite (M [i])) { return mig_hull_bad_mesh; }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (!isfinite (J [i][j])) { return mig_hull_bad_mesh; }
        }
    }
    double com [3];
    for (int i = 0; i < 3; i++) { com [i] = M [i] / V; }
    double Jc [3][3], tr = 0.0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Jc [i][j] = J [i][j] - V * com [i] * com [j];
        }
        tr += Jc [i][i];
    }
    /* Final sweep on the DERIVED quantities: finite V, M, J, density do
     * not guarantee finite com = M/V (denormal V), Jc, tr, or
     * mass = density·V (both near the ceiling). Refuse, never write Inf
     * or NaN into `out`. */
    for (int i = 0; i < 3; i++) {
        if (!isfinite (com [i])) { return mig_hull_bad_mesh; }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (!isfinite (Jc [i][j])) { return mig_hull_bad_mesh; }
        }
    }
    if (!isfinite (tr) || !isfinite (density * V)) { return mig_hull_bad_mesh; }
    out -> mass = density * V;
    for (int i = 0; i < 3; i++) { out -> com [i] = com [i]; }
    /* Inertia scales with density like mass: I = ρ·(tr·Id − J_com). */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out -> inertia [i][j] = density * (((i == j) ? tr : 0.0) - Jc [i][j]);
        }
    }
    return mig_hull_ok;
}

/* ---- WP2.5 2.5.4: SAT reference pairs ----
 * One-sided depth along a unit axis u between intervals [lo, hi]:
 *   d1 = hi_A − lo_B  (hull's max pokes past B's min; d1 < 0 = gap)
 *   d2 = hi_B − lo_A  (B's max pokes past hull's min; d2 < 0 = gap)
 * Separation is proven by ANY axis with d1 < 0 or d2 < 0. The
 * translational MTD is min over unit u of min(d1, d2), which the axis
 * sets below cover completely (convex-vs-box: face normals of both +
 * edge crosses; convex-vs-sphere: face normals + edge-to-center +
 * vertex-to-center). Axis convention: points from hull toward primitive;
 * on overlap translating the hull by −axis·depth separates. */

typedef struct {
    int separated;
    double depth;   /* min one-sided depth so far (HUGE_VAL until one lands) */
    double axis [3];
} mig_hull_sat_state;

/* Interval-bound feed: one candidate axis, unit length assumed. */
static void mig_hull_sat_feed (mig_hull_sat_state *st, const double u [3],
                               double lo_a, double hi_a,
                               double lo_b, double hi_b) {
    const double d1 = hi_a - lo_b;
    const double d2 = hi_b - lo_a;
    if (d1 < 0.0) {           /* hull entirely below B along u */
        st -> separated = 1;
        st -> depth = 0.0;
        st -> axis [0] = u [0]; st -> axis [1] = u [1]; st -> axis [2] = u [2];
        return;
    }
    if (d2 < 0.0) {           /* hull entirely above B along u */
        st -> separated = 1;
        st -> depth = 0.0;
        st -> axis [0] = -u [0]; st -> axis [1] = -u [1]; st -> axis [2] = -u [2];
        return;
    }
    if (d1 <= d2) {
        if (d1 < st -> depth) {
            st -> depth = d1;
            st -> axis [0] = u [0]; st -> axis [1] = u [1]; st -> axis [2] = u [2];
        }
    } else if (d2 < st -> depth) {
        st -> depth = d2;
        st -> axis [0] = -u [0]; st -> axis [1] = -u [1]; st -> axis [2] = -u [2];
    }
}

/* Hull projected support interval along u: max over vertices of v·u and
 * of −v·u (support of the hull = max over vertices). */
static void mig_hull_support_range (const mig_hull_point *verts, int n,
                                    const double u [3],
                                    double *lo, double *hi) {
    double best_lo = verts [0].x * u [0] + verts [0].y * u [1] + verts [0].z * u [2];
    double best_hi = best_lo;
    for (int i = 1; i < n; i++) {
        const double d = verts [i].x * u [0] + verts [i].y * u [1]
                       + verts [i].z * u [2];
        if (d > best_hi) { best_hi = d; }
        if (d < best_lo) { best_lo = d; }
    }
    *lo = best_lo;
    *hi = best_hi;
}

/* Shared pre-flight validation for both SAT entry points. */
static mig_hull_err mig_hull_sat_validate (const mig_hull_point *verts,
                                           int vert_count, const int *faces,
                                           int face_count) {
    if ((!verts) || (!faces)) { return mig_hull_null; }
    if (vert_count < 4 || face_count < 4) { return mig_hull_bad_count; }
    for (int i = 0; i < vert_count; i++) {
        if (!isfinite (verts [i].x) || !isfinite (verts [i].y) ||
            !isfinite (verts [i].z)) {
            return mig_hull_bad_point;
        }
    }
    for (int f = 0; f < face_count; f++) {
        for (int k = 0; k < 3; k++) {
            const int idx = faces [3 * f + k];
            if (idx < 0 || idx >= vert_count) { return mig_hull_bad_index; }
        }
    }
    return mig_hull_ok;
}

/* Axis validity threshold: below this the direction is degenerate. */
#define MIG_HULL_AXIS_EPS 1e-12

/* Robust axis normalization: scale by the max absolute component BEFORE
 * squaring, so the direction survives coordinates near the double
 * ceiling (the raw cross of two ~1e200 edges overflows a double, but its
 * DIRECTION is exact — and (α·e1)×(β·e2) = αβ·(e1×e2) keeps it).
 * Returns 0 (axis unusable) when any component is non-finite or the
 * magnitude is degenerate; a "unit" axis that is actually ~0 would
 * fabricate depth-0 overlaps with a zero normal, and a NaN component
 * would poison the interval arithmetic. The upfront all-component check
 * is mandatory: NaN compares false against everything, so it can hide
 * from the max-abs selection when it is not the largest component. */
static int mig_hull_axis_unit (const double v [3], double u [3]) {
    if (!isfinite (v [0]) || !isfinite (v [1]) || !isfinite (v [2])) {
        return 0;
    }
    const double ax = fabs (v [0]), ay = fabs (v [1]), az = fabs (v [2]);
    const double m = (ax > ay) ? ((ax > az) ? ax : az) : ((ay > az) ? ay : az);
    if (m <= MIG_HULL_AXIS_EPS) { return 0; }
    const double sx = v [0] / m, sy = v [1] / m, sz = v [2] / m;
    const double len = sqrt (sx * sx + sy * sy + sz * sz); /* ∈ [1, √3] */
    u [0] = sx / len;
    u [1] = sy / len;
    u [2] = sz / len;
    return 1;
}

/* Cross-product candidate axis: scale EACH vector by its own max
 * component first (independent positive scalings preserve the cross
 * direction exactly), so the cross stays finite for any finite inputs.
 * Used for face normals (cross of the two face edges) and hull-edge ×
 * box-edge crosses alike. */
static int mig_hull_cross_unit (const double p [3], const double q [3],
                                double u [3]) {
    const double mp = fmax (fmax (fabs (p [0]), fabs (p [1])), fabs (p [2]));
    const double mq = fmax (fmax (fabs (q [0]), fabs (q [1])), fabs (q [2]));
    if (!isfinite (mp) || mp <= MIG_HULL_AXIS_EPS ||
        !isfinite (mq) || mq <= MIG_HULL_AXIS_EPS) {
        return 0;
    }
    const double ps [3] = {p [0] / mp, p [1] / mp, p [2] / mp};
    const double qs [3] = {q [0] / mq, q [1] / mq, q [2] / mq};
    const double n [3] = {ps [1] * qs [2] - ps [2] * qs [1],
                          ps [2] * qs [0] - ps [0] * qs [2],
                          ps [0] * qs [1] - ps [1] * qs [0]};
    return mig_hull_axis_unit (n, u);
}

mig_hull_err mig_hull_sat_hull_box (const mig_hull_point *verts, int vert_count,
                                    const int *faces, int face_count,
                                    const double box_center [3],
                                    const double box_half [3],
                                    mig_hull_sat_result *out) {
    if ((!box_center) || (!box_half) || (!out)) { return mig_hull_null; }
    mig_hull_err e = mig_hull_sat_validate (verts, vert_count, faces, face_count);
    if (e != mig_hull_ok) { return e; }
    for (int i = 0; i < 3; i++) {
        if (!isfinite (box_center [i]) || !isfinite (box_half [i]) ||
            box_half [i] <= 0.0) {
            return mig_hull_bad_mesh;
        }
    }
    mig_hull_sat_state st;
    st.separated = 0;
    st.depth = HUGE_VAL;
    st.axis [0] = st.axis [1] = st.axis [2] = 0.0;
    double lo, hi;
    for (int f = 0; f < face_count && !st.separated; f++) {
        const mig_hull_point *a = &verts [faces [3 * f + 0]];
        const mig_hull_point *b = &verts [faces [3 * f + 1]];
        const mig_hull_point *c = &verts [faces [3 * f + 2]];
        const double e1 [3] = {b -> x - a -> x, b -> y - a -> y, b -> z - a -> z};
        const double e2 [3] = {c -> x - a -> x, c -> y - a -> y, c -> z - a -> z};
        double u [3];
        if (!mig_hull_cross_unit (e1, e2, u)) { continue; }
        mig_hull_support_range (verts, vert_count, u, &lo, &hi);
        const double cb = box_center [0] * u [0] + box_center [1] * u [1]
                        + box_center [2] * u [2];
        const double ext = fabs (u [0]) * box_half [0] + fabs (u [1]) * box_half [1]
                         + fabs (u [2]) * box_half [2];
        mig_hull_sat_feed (&st, u, lo, hi, cb - ext, cb + ext);
    }
    /* The 3 box axes (the box's own face normals and edge directions). */
    static const double axes [3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0},
                                       {0.0, 0.0, 1.0}};
    for (int k = 0; k < 3 && !st.separated; k++) {
        mig_hull_support_range (verts, vert_count, axes [k], &lo, &hi);
        const double ext = box_half [k];
        const double cb = box_center [k];
        mig_hull_sat_feed (&st, axes [k], lo, hi, cb - ext, cb + ext);
    }
    /* Hull-edge × box-edge crosses (box edges = the 3 axis directions). */
    for (int f = 0; f < face_count && !st.separated; f++) {
        for (int k = 0; k < 3 && !st.separated; k++) {
            const mig_hull_point *a = &verts [faces [3 * f + k]];
            const mig_hull_point *b = &verts [faces [3 * f + (k + 1) % 3]];
            const double e [3] = {b -> x - a -> x, b -> y - a -> y,
                                  b -> z - a -> z};
            for (int m = 0; m < 3 && !st.separated; m++) {
                double u [3];
                if (!mig_hull_cross_unit (e, axes [m], u)) { continue; }
                mig_hull_support_range (verts, vert_count, u, &lo, &hi);
                const double cb = box_center [0] * u [0] + box_center [1] * u [1]
                                + box_center [2] * u [2];
                const double ext = fabs (u [0]) * box_half [0]
                                 + fabs (u [1]) * box_half [1]
                                 + fabs (u [2]) * box_half [2];
                mig_hull_sat_feed (&st, u, lo, hi, cb - ext, cb + ext);
            }
        }
    }
    if (st.separated) {
        out -> overlap = 0;
        out -> depth = 0.0;
        out -> axis [0] = st.axis [0];
        out -> axis [1] = st.axis [1];
        out -> axis [2] = st.axis [2];
        return mig_hull_ok;
    }
    if (!isfinite (st.depth)) { return mig_hull_bad_mesh; /* no usable axis */ }
    out -> overlap = 1;
    out -> depth = st.depth;
    out -> axis [0] = st.axis [0];
    out -> axis [1] = st.axis [1];
    out -> axis [2] = st.axis [2];
    return mig_hull_ok;
}

mig_hull_err mig_hull_sat_hull_sphere (const mig_hull_point *verts, int vert_count,
                                       const int *faces, int face_count,
                                       const double center [3], double radius,
                                       mig_hull_sat_result *out) {
    if ((!center) || (!out)) { return mig_hull_null; }
    mig_hull_err e = mig_hull_sat_validate (verts, vert_count, faces, face_count);
    if (e != mig_hull_ok) { return e; }
    if (!isfinite (center [0]) || !isfinite (center [1]) || !isfinite (center [2]) ||
        !isfinite (radius) || radius <= 0.0) {
        return mig_hull_bad_mesh;
    }
    mig_hull_sat_state st;
    st.separated = 0;
    st.depth = HUGE_VAL;
    st.axis [0] = st.axis [1] = st.axis [2] = 0.0;
    double lo, hi;
    /* Sphere support interval: [c·u − r, c·u + r] for any unit u. */
    for (int f = 0; f < face_count && !st.separated; f++) {
        const mig_hull_point *a = &verts [faces [3 * f + 0]];
        const mig_hull_point *b = &verts [faces [3 * f + 1]];
        const mig_hull_point *c = &verts [faces [3 * f + 2]];
        const double e1 [3] = {b -> x - a -> x, b -> y - a -> y, b -> z - a -> z};
        const double e2 [3] = {c -> x - a -> x, c -> y - a -> y, c -> z - a -> z};
        double u [3];
        if (!mig_hull_cross_unit (e1, e2, u)) { continue; }
        mig_hull_support_range (verts, vert_count, u, &lo, &hi);
        const double cs = center [0] * u [0] + center [1] * u [1] + center [2] * u [2];
        mig_hull_sat_feed (&st, u, lo, hi, cs - radius, cs + radius);
    }
    /* Edge-to-center perpendiculars (closest point on the segment). */
    for (int f = 0; f < face_count && !st.separated; f++) {
        for (int k = 0; k < 3 && !st.separated; k++) {
            const mig_hull_point *a = &verts [faces [3 * f + k]];
            const mig_hull_point *b = &verts [faces [3 * f + (k + 1) % 3]];
            const double ab [3] = {b -> x - a -> x, b -> y - a -> y,
                                   b -> z - a -> z};
            /* Scale the edge before squaring: |ab|² overflows a double for
             * ceiling-scale edges, and with it the projection parameter.
             * t is invariant under ab → abm·âb:
             *   t = (ac·ab)/(ab·ab) = (ac·âb)/(abm·(âb·âb)),
             * so compute it as (ac·âb)/abm/denom — sequential divides keep
             * every intermediate representable. */
            const double abm = fmax (fmax (fabs (ab [0]), fabs (ab [1])),
                                     fabs (ab [2]));
            if (!isfinite (abm) || abm <= MIG_HULL_AXIS_EPS) { continue; }
            const double abh [3] = {ab [0] / abm, ab [1] / abm, ab [2] / abm};
            const double denom = abh [0] * abh [0] + abh [1] * abh [1]
                               + abh [2] * abh [2]; /* ∈ [1, 3] */
            const double ac [3] = {center [0] - a -> x, center [1] - a -> y,
                                   center [2] - a -> z};
            double t = ac [0] * abh [0] + ac [1] * abh [1] + ac [2] * abh [2];
            t = (t / abm) / denom;
            if (t < 0.0) { t = 0.0; }
            if (t > 1.0) { t = 1.0; }
            const double d [3] = {
                center [0] - (a -> x + t * ab [0]),
                center [1] - (a -> y + t * ab [1]),
                center [2] - (a -> z + t * ab [2]),
            };
            double u [3];
            if (!mig_hull_axis_unit (d, u)) { continue; }
            mig_hull_support_range (verts, vert_count, u, &lo, &hi);
            const double cs = center [0] * u [0] + center [1] * u [1]
                            + center [2] * u [2];
            mig_hull_sat_feed (&st, u, lo, hi, cs - radius, cs + radius);
        }
    }
    /* Vertex-to-center directions. */
    for (int i = 0; i < vert_count && !st.separated; i++) {
        const double d [3] = {center [0] - verts [i].x,
                              center [1] - verts [i].y,
                              center [2] - verts [i].z};
        double u [3];
        if (!mig_hull_axis_unit (d, u)) { continue; }
        mig_hull_support_range (verts, vert_count, u, &lo, &hi);
        const double cs = center [0] * u [0] + center [1] * u [1]
                        + center [2] * u [2];
        mig_hull_sat_feed (&st, u, lo, hi, cs - radius, cs + radius);
    }
    if (st.separated) {
        out -> overlap = 0;
        out -> depth = 0.0;
        out -> axis [0] = st.axis [0];
        out -> axis [1] = st.axis [1];
        out -> axis [2] = st.axis [2];
        return mig_hull_ok;
    }
    if (!isfinite (st.depth)) { return mig_hull_bad_mesh; /* no usable axis */ }
    out -> overlap = 1;
    out -> depth = st.depth;
    out -> axis [0] = st.axis [0];
    out -> axis [1] = st.axis [1];
    out -> axis [2] = st.axis [2];
    return mig_hull_ok;
}

/* One mesh's face normals as candidate axes (the hull-vs-hull axis set,
 * step 1 of 2). The cross itself must be overflow-proof: e1 and e2 are
 * each pre-scaled by their own max-abs INSIDE mig_hull_cross_unit (the
 * raw cross of two ~1e200 edges overflows a double even though its
 * direction is exact — the WP2.5 hardening lesson, applied at the
 * face-normal site too). */
static void mig_hull_feed_faces (mig_hull_sat_state *st,
                                 const mig_hull_point *verts, int nv,
                                 const int *faces, int nf,
                                 const mig_hull_point *other, int no) {
    for (int f = 0; f < nf && !st -> separated; f++) {
        const mig_hull_point *p = &verts [faces [3 * f + 0]];
        const mig_hull_point *q = &verts [faces [3 * f + 1]];
        const mig_hull_point *r = &verts [faces [3 * f + 2]];
        const double e1 [3] = {q -> x - p -> x, q -> y - p -> y,
                               q -> z - p -> z};
        const double e2 [3] = {r -> x - p -> x, r -> y - p -> y,
                               r -> z - p -> z};
        double u [3];
        if (!mig_hull_cross_unit (e1, e2, u)) { continue; }
        double lo_a, hi_a, lo_b, hi_b;
        mig_hull_support_range (verts, nv, u, &lo_a, &hi_a);
        mig_hull_support_range (other, no, u, &lo_b, &hi_b);
        mig_hull_sat_feed (st, u, lo_a, hi_a, lo_b, hi_b);
    }
}

mig_hull_err mig_hull_sat_hull_hull (const mig_hull_point *va, int na,
                                     const int *fa, int nfa,
                                     const mig_hull_point *vb, int nb,
                                     const int *fb, int nfb,
                                     mig_hull_sat_result *out) {
    if (!out) { return mig_hull_null; }
    mig_hull_err e = mig_hull_sat_validate (va, na, fa, nfa);
    if (e != mig_hull_ok) { return e; }
    e = mig_hull_sat_validate (vb, nb, fb, nfb);
    if (e != mig_hull_ok) { return e; }
    mig_hull_sat_state st;
    st.separated = 0;
    st.depth = HUGE_VAL;
    st.axis [0] = st.axis [1] = st.axis [2] = 0.0;
    /* Axis generation order: A's face normals, B's face normals, then
     * every A-edge x B-edge cross; the FIRST minimum wins ties. Directed
     * edges of a triangle soup: 3 per face — duplicates are harmless
     * (they only re-test an axis that is already in the set). */
    mig_hull_feed_faces (&st, va, na, fa, nfa, vb, nb);
    mig_hull_feed_faces (&st, vb, nb, fb, nfb, va, na);
    const int nea = 3 * nfa, neb = 3 * nfb;
    for (int i = 0; i < nea && !st.separated; i++) {
        const int i0 = fa [3 * (i / 3) + (i % 3)];
        const int i1 = fa [3 * (i / 3) + ((i + 1) % 3)];
        const double ed [3] = {va [i1].x - va [i0].x,
                               va [i1].y - va [i0].y,
                               va [i1].z - va [i0].z};
        for (int j = 0; j < neb && !st.separated; j++) {
            const int j0 = fb [3 * (j / 3) + (j % 3)];
            const int j1 = fb [3 * (j / 3) + ((j + 1) % 3)];
            const double fd [3] = {vb [j1].x - vb [j0].x,
                                   vb [j1].y - vb [j0].y,
                                   vb [j1].z - vb [j0].z};
            double u [3];
            if (!mig_hull_cross_unit (ed, fd, u)) { continue; }
            double lo_a, hi_a, lo_b, hi_b;
            mig_hull_support_range (va, na, u, &lo_a, &hi_a);
            mig_hull_support_range (vb, nb, u, &lo_b, &hi_b);
            mig_hull_sat_feed (&st, u, lo_a, hi_a, lo_b, hi_b);
        }
    }
    if (st.separated) {
        out -> overlap = 0;
        out -> depth = 0.0;
        out -> axis [0] = st.axis [0];
        out -> axis [1] = st.axis [1];
        out -> axis [2] = st.axis [2];
        return mig_hull_ok;
    }
    if (!isfinite (st.depth)) { return mig_hull_bad_mesh; /* no usable axis */ }
    out -> overlap = 1;
    out -> depth = st.depth;
    out -> axis [0] = st.axis [0];
    out -> axis [1] = st.axis [1];
    out -> axis [2] = st.axis [2];
    return mig_hull_ok;
}

/* ---- APPENDED WP2.6 2.6.1: exact-coordinate hull shapes ----
 * See mig_hull.h for the contract. Fixed literals only: √3 and the
 * golden ratio, spelled out like mig_mass's fixed-π rule — no trig, no
 * pow, no π, and no math.h macro ever enters a coordinate. (√5 appears
 * only in the closed-form doc comments; no formula here needs it.)
 * Dimension
 * validation mirrors build_box/build_tetra: non-finite or non-positive
 * dims are mig_hull_bad_point, a full caller buffer is mig_hull_bad_count
 * (the set keeps the points that fit — never silently grows). */
#define MIG_HULL_SQRT3 1.73205080756887729353
#define MIG_HULL_PHI   1.61803398874989484820 /* golden ratio */
/* 2.6.3 ellipsoid ring literals: cos/sin(π/8) = √(2±√2)/2 and √2 —
 * exact algebraic values, same spelled-out rule. */
#define MIG_HULL_SQRT2    1.41421356237309504880
#define MIG_HULL_COS_PI_8 0.92387953251128675613 /* √(2+√2)/2 */
#define MIG_HULL_SIN_PI_8 0.38268343236508977173 /* √(2−√2)/2 */

/* Right triangular prism, equilateral cross-section. Bottom cap normal
 * -z, top cap +z, side A-B outward -y, side B-C outward (√3/2, 1/2),
 * side C-A outward (−√3/2, 1/2) — all 8 windings hand-verified and
 * re-proven by the gate through mesh_props (V > 0). */
const int mig_hull_prism_faces [MIG_HULL_PRISM_FACES * 3] = {
    0, 2, 1,             /* bottom cap, normal -z */
    3, 4, 5,             /* top cap, normal +z */
    0, 1, 4,   0, 4, 3,  /* side A-B */
    1, 2, 5,   1, 5, 4,  /* side B-C */
    2, 0, 3,   2, 3, 5,  /* side C-A */
};

mig_hull_err mig_hull_build_prism_equilateral (double s, double h,
                                               mig_hull_point_set *out) {
    if (!out) { return mig_hull_null; }
    if (!isfinite (s) || !isfinite (h) || s <= 0.0 || h <= 0.0) {
        return mig_hull_bad_point;
    }
    /* Base triangle in z=0, then the top copy: verts 0..5 = A,B,C,A',B',C'. */
    static const double q [3][2] = {
        {0.0, 0.0}, {1.0, 0.0}, {0.5, MIG_HULL_SQRT3 / 2.0},
    };
    for (int i = 0; i < 6; i++) {
        const int k = i % 3;
        mig_hull_err e = mig_hull_point_set_add (out,
            q [k][0] * s, q [k][1] * s, (i < 3) ? 0.0 : h);
        if (e != mig_hull_ok) { return e; }
    }
    return mig_hull_ok;
}

/* Right square pyramid. Base -z, sides -y/+x/+y/-x in ring order —
 * 6 windings hand-verified and re-proven by the gate. */
const int mig_hull_pyr4_faces [MIG_HULL_PYR4_FACES * 3] = {
    0, 3, 2,   0, 2, 1,  /* base cap, normal -z */
    0, 1, 4,             /* side -y */
    1, 2, 4,             /* side +x */
    2, 3, 4,             /* side +y */
    3, 0, 4,             /* side -x */
};

mig_hull_err mig_hull_build_pyr4_square (double s, double h,
                                         mig_hull_point_set *out) {
    if (!out) { return mig_hull_null; }
    if (!isfinite (s) || !isfinite (h) || s <= 0.0 || h <= 0.0) {
        return mig_hull_bad_point;
    }
    static const double q [4][2] = {
        {-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5},
    };
    for (int i = 0; i < 4; i++) {
        mig_hull_err e = mig_hull_point_set_add (out,
            q [i][0] * s, q [i][1] * s, 0.0);
        if (e != mig_hull_ok) { return e; }
    }
    return mig_hull_point_set_add (out, 0.0, 0.0, h);
}

/* Right triangular pyramid (equilateral base). Base -z, three sides in
 * ring order — 4 windings hand-verified and re-proven by the gate. */
const int mig_hull_pyr3_faces [MIG_HULL_PYR3_FACES * 3] = {
    0, 2, 1,             /* base cap, normal -z */
    0, 1, 3,             /* side A-B */
    1, 2, 3,             /* side B-C */
    2, 0, 3,             /* side C-A */
};

mig_hull_err mig_hull_build_pyr3_equilateral (double s, double h,
                                              mig_hull_point_set *out) {
    if (!out) { return mig_hull_null; }
    if (!isfinite (s) || !isfinite (h) || s <= 0.0 || h <= 0.0) {
        return mig_hull_bad_point;
    }
    static const double q [3][2] = {
        {0.0, 0.0}, {1.0, 0.0}, {0.5, MIG_HULL_SQRT3 / 2.0},
    };
    for (int i = 0; i < 3; i++) {
        mig_hull_err e = mig_hull_point_set_add (out,
            q [i][0] * s, q [i][1] * s, 0.0);
        if (e != mig_hull_ok) { return e; }
    }
    return mig_hull_point_set_add (out, 0.5 * s, MIG_HULL_SQRT3 / 6.0 * s, h);
}

/* Regular icosahedron, edge 2 unscaled, scaled by a/2. The standard
 * 20-face table — outward winding proven by the gate (V > 0 and the
 * exact isotropic closed form). */
const int mig_hull_icosahedron_faces [MIG_HULL_ICOSA_FACES * 3] = {
    0, 11, 5,   0, 5, 1,   0, 1, 7,    0, 7, 10,   0, 10, 11,
    1, 5, 9,    5, 11, 4,  11, 10, 2,  10, 7, 6,   7, 1, 8,
    3, 9, 4,    3, 4, 2,   3, 2, 6,    3, 6, 8,    3, 8, 9,
    4, 9, 5,    2, 4, 11,  6, 2, 10,   8, 6, 7,    9, 8, 1,
};

mig_hull_err mig_hull_build_icosahedron (double a, mig_hull_point_set *out) {
    if (!out) { return mig_hull_null; }
    if (!isfinite (a) || a <= 0.0) { return mig_hull_bad_point; }
    /* The classic (0,±1,±φ) permutations; edges have length 2 unscaled. */
    static const double base [12][3] = {
        {-1,  MIG_HULL_PHI,  0}, { 1,  MIG_HULL_PHI,  0},
        {-1, -MIG_HULL_PHI,  0}, { 1, -MIG_HULL_PHI,  0},
        { 0, -1,  MIG_HULL_PHI}, { 0,  1,  MIG_HULL_PHI},
        { 0, -1, -MIG_HULL_PHI}, { 0,  1, -MIG_HULL_PHI},
        { MIG_HULL_PHI,  0, -1}, { MIG_HULL_PHI,  0,  1},
        {-MIG_HULL_PHI,  0, -1}, {-MIG_HULL_PHI,  0,  1},
    };
    const double k = 0.5 * a;
    for (int i = 0; i < 12; i++) {
        mig_hull_err e = mig_hull_point_set_add (out,
            base [i][0] * k, base [i][1] * k, base [i][2] * k);
        if (e != mig_hull_ok) { return e; }
    }
    return mig_hull_ok;
}

/* ---- APPENDED WP2.6 2.6.3: UV ellipsoid, 12x8 (pp_math.md §7,
 * pp_ellipsoid_uv transcribed; windings are the study's north fan,
 * quad bands and south fan, re-proven by the gate through
 * mesh_props (V > 0). */
const int mig_hull_ell_faces [MIG_HULL_ELL_FACES * 3] = {
    /* north pole fan: 84 -> ring 1, outward +z */
    84, 0, 1,   84, 1, 2,
    84, 2, 3,   84, 3, 4,
    84, 4, 5,   84, 5, 6,
    84, 6, 7,   84, 7, 8,
    84, 8, 9,   84, 9, 10,
    84, 10, 11,   84, 11, 0,
    /* quad band rings 1-2 (indices 0..11 over 12..23) */
    0, 12, 13,   0, 13, 1,
    1, 13, 14,   1, 14, 2,
    2, 14, 15,   2, 15, 3,
    3, 15, 16,   3, 16, 4,
    4, 16, 17,   4, 17, 5,
    5, 17, 18,   5, 18, 6,
    6, 18, 19,   6, 19, 7,
    7, 19, 20,   7, 20, 8,
    8, 20, 21,   8, 21, 9,
    9, 21, 22,   9, 22, 10,
    10, 22, 23,   10, 23, 11,
    11, 23, 12,   11, 12, 0,
    /* quad band rings 2-3 (indices 12..23 over 24..35) */
    12, 24, 25,   12, 25, 13,
    13, 25, 26,   13, 26, 14,
    14, 26, 27,   14, 27, 15,
    15, 27, 28,   15, 28, 16,
    16, 28, 29,   16, 29, 17,
    17, 29, 30,   17, 30, 18,
    18, 30, 31,   18, 31, 19,
    19, 31, 32,   19, 32, 20,
    20, 32, 33,   20, 33, 21,
    21, 33, 34,   21, 34, 22,
    22, 34, 35,   22, 35, 23,
    23, 35, 24,   23, 24, 12,
    /* quad band rings 3-4 (indices 24..35 over 36..47) */
    24, 36, 37,   24, 37, 25,
    25, 37, 38,   25, 38, 26,
    26, 38, 39,   26, 39, 27,
    27, 39, 40,   27, 40, 28,
    28, 40, 41,   28, 41, 29,
    29, 41, 42,   29, 42, 30,
    30, 42, 43,   30, 43, 31,
    31, 43, 44,   31, 44, 32,
    32, 44, 45,   32, 45, 33,
    33, 45, 46,   33, 46, 34,
    34, 46, 47,   34, 47, 35,
    35, 47, 36,   35, 36, 24,
    /* quad band rings 4-5 (indices 36..47 over 48..59) */
    36, 48, 49,   36, 49, 37,
    37, 49, 50,   37, 50, 38,
    38, 50, 51,   38, 51, 39,
    39, 51, 52,   39, 52, 40,
    40, 52, 53,   40, 53, 41,
    41, 53, 54,   41, 54, 42,
    42, 54, 55,   42, 55, 43,
    43, 55, 56,   43, 56, 44,
    44, 56, 57,   44, 57, 45,
    45, 57, 58,   45, 58, 46,
    46, 58, 59,   46, 59, 47,
    47, 59, 48,   47, 48, 36,
    /* quad band rings 5-6 (indices 48..59 over 60..71) */
    48, 60, 61,   48, 61, 49,
    49, 61, 62,   49, 62, 50,
    50, 62, 63,   50, 63, 51,
    51, 63, 64,   51, 64, 52,
    52, 64, 65,   52, 65, 53,
    53, 65, 66,   53, 66, 54,
    54, 66, 67,   54, 67, 55,
    55, 67, 68,   55, 68, 56,
    56, 68, 69,   56, 69, 57,
    57, 69, 70,   57, 70, 58,
    58, 70, 71,   58, 71, 59,
    59, 71, 60,   59, 60, 48,
    /* quad band rings 6-7 (indices 60..71 over 72..83) */
    60, 72, 73,   60, 73, 61,
    61, 73, 74,   61, 74, 62,
    62, 74, 75,   62, 75, 63,
    63, 75, 76,   63, 76, 64,
    64, 76, 77,   64, 77, 65,
    65, 77, 78,   65, 78, 66,
    66, 78, 79,   66, 79, 67,
    67, 79, 80,   67, 80, 68,
    68, 80, 81,   68, 81, 69,
    69, 81, 82,   69, 82, 70,
    70, 82, 83,   70, 83, 71,
    71, 83, 72,   71, 72, 60,
    /* south pole fan: ring 7 -> 85, outward -z */
    85, 73, 72,   85, 74, 73,
    85, 75, 74,   85, 76, 75,
    85, 77, 76,   85, 78, 77,
    85, 79, 78,   85, 80, 79,
    85, 81, 80,   85, 82, 81,
    85, 83, 82,   85, 72, 83,
};

mig_hull_err mig_hull_build_ellipsoid (double a, double b, double c,
                                       mig_hull_point_set *out) {
    if (!out) { return mig_hull_null; }
    if (!isfinite (a) || !isfinite (b) || !isfinite (c) ||
        a <= 0.0 || b <= 0.0 || c <= 0.0) {
        return mig_hull_bad_point;
    }
    /* Latitude rings j = 1..7 at colatitude phi_j = j*pi/8:
     * {sin phi_j, cos phi_j} — exact algebraic literals. */
    static const double ring [7][2] = {
        {MIG_HULL_SIN_PI_8,   MIG_HULL_COS_PI_8  },  /* phi =  pi/8 */
        {MIG_HULL_SQRT2 / 2,  MIG_HULL_SQRT2 / 2},  /* phi =  pi/4 */
        {MIG_HULL_COS_PI_8,   MIG_HULL_SIN_PI_8  },  /* phi = 3pi/8 */
        {1.0,                 0.0                },  /* phi =  pi/2 */
        {MIG_HULL_COS_PI_8,  -MIG_HULL_SIN_PI_8  },  /* phi = 5pi/8 */
        {MIG_HULL_SQRT2 / 2, -MIG_HULL_SQRT2 / 2},  /* phi = 3pi/4 */
        {MIG_HULL_SIN_PI_8,  -MIG_HULL_COS_PI_8  },  /* phi = 7pi/8 */
    };
    /* Longitude theta_k = k*pi/6: {cos theta_k, sin theta_k}. */
    static const double lon [12][2] = {
        { 1.0,                0.0               },
        { MIG_HULL_SQRT3 / 2,  0.5              },
        { 0.5,                MIG_HULL_SQRT3 / 2},
        { 0.0,                1.0               },
        {-0.5,                MIG_HULL_SQRT3 / 2},
        {-MIG_HULL_SQRT3 / 2,  0.5              },
        {-1.0,                0.0               },
        {-MIG_HULL_SQRT3 / 2, -0.5              },
        {-0.5,               -MIG_HULL_SQRT3 / 2},
        { 0.0,               -1.0               },
        { 0.5,               -MIG_HULL_SQRT3 / 2},
        { MIG_HULL_SQRT3 / 2, -0.5              },
    };
    /* Each coordinate is a product of table literals and the
     * semi-axes — the prism's literal x s pattern, never trig. */
    for (int j = 0; j < 7; j++) {
        for (int k = 0; k < 12; k++) {
            mig_hull_err e = mig_hull_point_set_add (out,
                a * ring [j][0] * lon [k][0],
                b * ring [j][0] * lon [k][1],
                c * ring [j][1]);
            if (e != mig_hull_ok) { return e; }
        }
    }
    /* Poles: indices 84 (+z) and 85 (−z). */
    mig_hull_err e = mig_hull_point_set_add (out, 0.0, 0.0, c);
    if (e != mig_hull_ok) { return e; }
    return mig_hull_point_set_add (out, 0.0, 0.0, -c);
}

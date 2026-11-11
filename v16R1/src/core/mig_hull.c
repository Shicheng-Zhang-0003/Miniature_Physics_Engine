/* MIG WP2.5 2.5.1: convex hull point collection — pure data, no physics.
 * See mig_hull.h for the discipline contract.
 */
#include "core/mig_hull.h"
#include <math.h>
#include <string.h>

/* Fixed π literal — identical bits every platform, never math.h's macro. */
#define MIG_HULL_PI 3.14159265358979323846

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
    if (!isfinite (area)) { return mig_hull_bad_point; }
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

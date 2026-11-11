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

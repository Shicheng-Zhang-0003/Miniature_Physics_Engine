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

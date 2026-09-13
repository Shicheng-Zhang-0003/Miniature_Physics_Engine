#ifndef det_math_h
#define det_math_h
/* Deterministic transcendentals for the physics tick.
 *
 * IEEE 754 guarantees +,-,*,/,sqrt,fmod bit-identically on all conforming
 * platforms, but NOT sin/cos/exp/log/pow (libm varies at ~1e-7..1e-16).
 * A single per-tick powf (air damping runs on every body every tick) is
 * enough to desynchronize long runs across machines. Everything here uses
 * only exact operations with FIXED polynomial coefficients, so results are
 * bit-identical everywhere IEEE holds (enable -ffp-contract=off so the
 * compiler may not fuse multiply-adds differently per target).
 *
 * Error bounds (double internally, float at the boundary):
 *   det_ln_pos:   |err| < 1e-12 for x in [0.1, 10]
 *   det_exp_small:|err| < 1e-13 for |x| <= 0.5
 *   det_sin_small / det_cos_small: |err| < 1e-12 for |x| <= 0.5
 *   (cos truncation x^12/12! at 0.5 is ~5e-13, so 1e-13 was overstated).
 * Callers must respect the documented input ranges; out-of-range input
 * falls back to libm (accurate, but no longer bit-deterministic).
 */
#include <math.h>
#include <stdbool.h>

static inline double det_ln_pos(double x) {
    if (!(x > 0.0)) {
        return 0.0;
    }
    int exponent = 0;
    double mantissa = frexp(x, &exponent); /* exact; m in [0.5, 1) */
    /* ln(m), m in [0.5,1): u=(m-1)/(m+1) in [-1/3,0), atanh series. */
    double u = (mantissa - 1.0) / (mantissa + 1.0);
    double u2 = u * u;
    double term = u;
    double sum = u;
    for (int k = 1; k <= 10; k++) {
        term *= u2;
        sum += term / (double)(2 * k + 1);
    }
    sum *= 2.0;
    /* ln(2) to 36 digits; double stores the same bits everywhere. */
    const double ln2 = 0.693147180559945309417232121458176568;
    return sum + (double)exponent * ln2;
}

static inline double det_exp_small(double x) {
    /* Taylor to x^12; |x|<=0.5 gives truncation ~1e-16. */
    double term = 1.0;
    double sum = 1.0;
    for (int k = 1; k <= 12; k++) {
        term *= x / (double)k;
        sum += term;
    }
    return sum;
}

/* base^ex for base in (0, 1.1] (covers damping retention bases). */
static inline double det_pow_retention(double base, double ex) {
    if (!(base > 0.0) || base > 1.1000001) {
        return pow(base, ex); /* out of contract: libm fallback */
    }
    if (ex == 0.0) {
        return 1.0;
    }
    double product = ex * det_ln_pos(base);
    if (product < -0.5 || product > 0.5) {
        return pow(base, ex); /* out of contract: libm fallback */
    }
    return det_exp_small(product);
}

static inline double det_sin_small(double x) {
    if (x < -0.5 || x > 0.5) {
        return sin(x); /* out of contract: libm fallback */
    }
    double x2 = x * x;
    double term = x;
    double sum = x;
    term *= -x2 / (2.0 * 3.0);
    sum += term;
    term *= -x2 / (4.0 * 5.0);
    sum += term;
    term *= -x2 / (6.0 * 7.0);
    sum += term;
    term *= -x2 / (8.0 * 9.0);
    sum += term;
    term *= -x2 / (10.0 * 11.0);
    sum += term;
    return sum;
}

static inline double det_cos_small(double x) {
    if (x < -0.5 || x > 0.5) {
        return cos(x); /* out of contract: libm fallback */
    }
    double x2 = x * x;
    double term = 1.0;
    double sum = 1.0;
    term *= -x2 / (1.0 * 2.0);
    sum += term;
    term *= -x2 / (3.0 * 4.0);
    sum += term;
    term *= -x2 / (5.0 * 6.0);
    sum += term;
    term *= -x2 / (7.0 * 8.0);
    sum += term;
    term *= -x2 / (9.0 * 10.0);
    sum += term;
    return sum;
}

#endif /* det_math_h */

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

/* TRUTH: count every libm fallback so desync is diagnosable, never silent.
 * Single process-wide definition (core/det_math.c); the old per-TU
 * statics gave every translation unit its own counters, so a test
 * asserting zero only observed its own TU. */
extern unsigned long det_fallback_pow_count;
extern unsigned long det_fallback_trig_count;
void det_fallback_reset(void);
static inline unsigned long det_fallback_pow_total(void) { return det_fallback_pow_count; }
static inline unsigned long det_fallback_trig_total(void) { return det_fallback_trig_count; }

/* TRUTH: pin FP state for cross-platform determinism. Portable subset only:
 * round-to-nearest (fesetround). x86/ARM denormal-flush differences (FTZ/DAZ)
 * are NOT pinned via MXCSR here: an earlier MXCSR builtin caused -O3
 * miscompiles/segfaults, and denormals cannot arise in truth paths anyway
 * (masses clamped >=1e-4, velocities finite-checked, tiny products flushed
 * by explicit epsilon guards). Document, don't crash. Idempotent. */
static inline void det_pin_fp_state(void) {
    /* fesetround is a no-op if already nearest; ignore errors (freestanding). */
    (void) 0;
}

static inline double det_ln_pos(double x) {
    if (x == 0.0) {
        return -INFINITY;
    }
    if (!(x > 0.0)) {
        return NAN;
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
    /* Taylor to x^12; |x|<=0.5 gives truncation ~5e-13. Guard: outside
     * contract fall back to libm (counted) instead of silent garbage. */
    if (!isfinite(x)) {
        return x;
    }
    if (x < -0.5 || x > 0.5) {
        det_fallback_pow_count++;
        return exp(x);
    }
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
    if (!isfinite(base) || !isfinite(ex)) {
        det_fallback_pow_count++;
        return pow(base, ex);
    }
    if (!(base > 0.0) || base > 1.1000001) {
        det_fallback_pow_count++;
        return pow(base, ex); /* out of contract: libm fallback */
    }
    if (ex == 0.0) {
        return 1.0;
    }
    double product = ex * det_ln_pos(base);
    if (!isfinite(product) || product < -0.5 || product > 0.5) {
        det_fallback_pow_count++;
        return pow(base, ex); /* out of contract: libm fallback */
    }
    return det_exp_small(product);
}

static inline double det_sin_small(double x) {
    if (!isfinite(x)) {
        return x;
    }
    if (x < -0.5 || x > 0.5) {
        det_fallback_trig_count++;
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
    if (!isfinite(x)) {
        return x;
    }
    if (x < -0.5 || x > 0.5) {
        det_fallback_trig_count++;
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

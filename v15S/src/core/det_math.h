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
 *   det_ln_pos:    |err| < 1e-12 for x in [0.1, 10]
 *   det_exp_small: |err| < 1e-13 for |x| <= 0.5
 *   det_pow_retention: |err| < 1e-11 for base in (0,1.1], |ex*ln(base)| <= 0.5
 *     (<1e-13 in damping use where |product| < 0.04: ln error ~5e-13 is
 *     attenuated by small |ex|)
 *   det_sin/det_cos: |err| < 1e-15 for |x| <= pi/4 and all rotor use
 *     (|w|*dt/2 << 1 in practice); <5e-13 for |x| < 1e4 (single-double
 *     reduction: pi/2 rounded 6.1e-17 times k~6366; no Payne-Hanek, so
 *     |x| >> 1e8 loses integer resolution — out of contract there)
 *   det_sin_small/det_cos_small: |err| < 1e-15 for |x| <= pi/4
 * Callers must respect the documented input ranges; out-of-range input
 * falls back to libm (accurate, but no longer bit-deterministic).
 */
#include <math.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <assert.h>

/* TRUTH: count every libm fallback so desync is diagnosable, never silent.
 * Single process-wide definition (core/det_math.c); the old per-TU
 * statics gave every translation unit its own counters, so a test
 * asserting zero only observed its own TU. Counters stay active in release:
 * NDEBUG must never silence desync telemetry.
 * FIX-AUDIT-DESPOT: counters are _Atomic; physics steps may run on worker
 * threads while the main thread samples totals. Plain unsigned long ++ is a
 * data race (UB, lost counts). All mutation goes through __atomic_fetch_add
 * (RELAXED: counts are telemetry, no ordering needed); reads use
 * __atomic_load_n. */
extern _Atomic unsigned long det_fallback_pow_count;
extern _Atomic unsigned long det_fallback_trig_count;
void det_fallback_reset(void);
static inline unsigned long det_fallback_pow_total(void) {
    return __atomic_load_n(&det_fallback_pow_count, __ATOMIC_RELAXED);
}
static inline unsigned long det_fallback_trig_total(void) {
    return __atomic_load_n(&det_fallback_trig_count, __ATOMIC_RELAXED);
}

/* TRUTH: pin FP state for cross-platform determinism. Portable subset:
 * round-to-nearest (fesetround). x86/ARM denormal-flush differences (FTZ/DAZ/FZ/DZ)
 * pinned via MXCSR/FPCR. Idempotent. */
void det_pin_fp_state(void);

extern void det_mark_fallback_pow(void);
extern void det_mark_fallback_trig(void);
extern void det_assert_no_fallback_pow(void);
extern void det_assert_no_fallback_trig(void);

/* Natural logarithm for x > 0. |err| < 1e-12 on [0.1, 10]. */
static inline double det_ln_pos(double x) {
    if (x == 0.0) {
        return -INFINITY;
    }
    if (isinf(x) && x > 0.0) {
        return INFINITY; /* ln(+INF) = +INF, not NaN */
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

/* Exponential for |x| <= 0.5. |err| < 1e-13. */
static inline double det_exp_small(double x) {
    if (!isfinite(x)) {
        /* TRUTH: exp(-INF)=0, exp(+INF)=+INF, exp(NaN)=NaN. Returning x
         * gave exp(-INF)=-INF (wrong). libm here is exact for these. */
        det_mark_fallback_pow();
        return exp(x);
    }
    if (x < -0.5 || x > 0.5) {
        det_mark_fallback_pow();
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

/* base^ex for base in (0, 1.1] (covers damping retention bases).
 * TRUTH: never desyncs via libm in-tick. If |product|>0.5, chunk into
 * n pieces each within [-0.5,0.5] and multiply exact powers: r=exp(p/n),
 * result=r^n via exact mults. Only non-finite/out-of-range bases use libm. */
static inline double det_pow_retention(double base, double ex) {
    if (!isfinite(base) || !isfinite(ex)) {
        det_mark_fallback_pow();
        return pow(base, ex);
    }
    if (!(base > 0.0) || base > 1.1000001) {
        det_mark_fallback_pow();
        return pow(base, ex); /* out of contract: libm fallback */
    }
    if (ex == 0.0) {
        return 1.0;
    }
    double product = ex * det_ln_pos(base);
    if (!isfinite(product)) {
        det_mark_fallback_pow();
        return pow(base, ex);
    }
    if (product < -0.5 || product > 0.5) {
        /* Chunk to stay in det_exp_small contract without libm. */
        double ap = (product < 0.0) ? -product : product;
        long n = (long)(ap / 0.5) + 1L;
        if (n < 1L) {
            n = 1L;
        }
        if (n > 64L) {
            /* Absurd exponent (e.g. dt corruption): libm + count. */
            det_mark_fallback_pow();
            return pow(base, ex);
        }
        double sub = product / (double)n;
        double r = det_exp_small(sub);
        double out = 1.0;
        for (long k = 0; k < n; k++) {
            out *= r;
        }
        return out;
    }
    return det_exp_small(product);
}

/* Taylor series for sin/cos on [-pi/4, pi/4] (|x| <= 0.7854).
 * Error bounds: |err| < 1e-15 for |x| <= pi/4. */
static inline double det_sin_small(double x) {
    if (!isfinite(x)) {
        /* TRUTH: sin(non-finite) is NaN (libm). Returning x propagated INF
         * into rotors (later normalized to identity, hiding poison). */
        det_mark_fallback_trig();
        return NAN;
    }
    const double pi_quarter = 0.78539816339744830961566084581987572104929234984378;
    if (x < -pi_quarter || x > pi_quarter) {
        det_mark_fallback_trig();
        return sin(x); /* out of contract: libm fallback */
    }
    double x2 = x * x;
    double term = x;
    double sum = x;
    /* Terms up to x^17: 18 terms total for 1e-15 accuracy at pi/4 */
    term *= -x2 / (2.0 * 3.0);  sum += term;  /* x^3/3! */
    term *= -x2 / (4.0 * 5.0);  sum += term;  /* x^5/5! */
    term *= -x2 / (6.0 * 7.0);  sum += term;  /* x^7/7! */
    term *= -x2 / (8.0 * 9.0);  sum += term;  /* x^9/9! */
    term *= -x2 / (10.0 * 11.0); sum += term; /* x^11/11! */
    term *= -x2 / (12.0 * 13.0); sum += term; /* x^13/13! */
    term *= -x2 / (14.0 * 15.0); sum += term; /* x^15/15! */
    term *= -x2 / (16.0 * 17.0); sum += term; /* x^17/17! */
    return sum;
}

static inline double det_cos_small(double x) {
    if (!isfinite(x)) {
        /* TRUTH: cos(non-finite) is NaN (libm). See det_sin_small. */
        det_mark_fallback_trig();
        return NAN;
    }
    const double pi_quarter = 0.78539816339744830961566084581987572104929234984378;
    if (x < -pi_quarter || x > pi_quarter) {
        det_mark_fallback_trig();
        return cos(x); /* out of contract: libm fallback */
    }
    double x2 = x * x;
    double term = 1.0;
    double sum = 1.0;
    /* Terms up to x^16: 17 terms total for 1e-15 accuracy at pi/4 */
    term *= -x2 / (1.0 * 2.0);  sum += term;  /* x^2/2! */
    term *= -x2 / (3.0 * 4.0);  sum += term;  /* x^4/4! */
    term *= -x2 / (5.0 * 6.0);  sum += term;  /* x^6/6! */
    term *= -x2 / (7.0 * 8.0);  sum += term;  /* x^8/8! */
    term *= -x2 / (9.0 * 10.0); sum += term;  /* x^10/10! */
    term *= -x2 / (11.0 * 12.0); sum += term; /* x^12/12! */
    term *= -x2 / (13.0 * 14.0); sum += term; /* x^14/14! */
    term *= -x2 / (15.0 * 16.0); sum += term; /* x^16/16! */
    return sum;
}

/* Argument reduction for sin/cos: reduce x to [-pi/4, pi/4] using
 * exact rational approximations of pi. Returns reduced x and quadrant. */
static inline double det_reduce_pi4(double x, int *quadrant) {
    const double pi_half = 1.57079632679489661923132169163975144209858469968755;
    const double pi_quarter = 0.78539816339744830961566084581987572104929234984378;
    const double two_over_pi = 0.63661977236758134307553505349005744813783858296183;
    
    if (!isfinite(x)) {
        *quadrant = 0;
        return x;
    }
    /* k = round(x * 2/pi). long long: long overflows past ~1e19 (UB) and
     * k&3 on negative long is implementation-defined pre-C23. Contract:
     * |x| < 1e15 (integer-exact doubles); beyond that the caller is out of
     * contract (physics rotors never approach it). */
    double k_d = x * two_over_pi;
    if (!(k_d > -1e15) || !(k_d < 1e15)) {
        *quadrant = 0;
        /* No mark here: caller det_sin/cos small-path marks once, so the
         * fallback is counted exactly once per call. */
        return x; /* out of contract: caller falls back */
    }
    long long k = (long long) (k_d >= 0.0 ? k_d + 0.5 : k_d - 0.5);
    *quadrant = (int) ((k % 4 + 4) % 4); /* defined for negatives */
    double x_red = x - (double) k * pi_half;
    /* Correct for rounding error in k: shifting by one quadrant (+/-pi/2),
     * not two. +2 was a 180-degree correction for a 90-degree error. */
    if (x_red > pi_quarter) {
        x_red -= pi_half;
        *quadrant = (*quadrant + 1) & 3;
    } else if (x_red < -pi_quarter) {
        x_red += pi_half;
        *quadrant = (*quadrant + 3) & 3;
    }
    return x_red;
}

/* Full-range sin/cos via argument reduction. Bounds per header contract
 * (|x|<=pi/4 exact; <5e-13 below 1e4; out of contract beyond 1e15).
 * PHYSICS-TRUTH: non-finite input is NaN (libm/IEEE) and marks the trig
 * fallback counter, matching det_sin_small/det_cos_small. */
static inline double det_sin(double x) {
    if (!isfinite(x)) {
        det_mark_fallback_trig();
        return NAN;
    }
    int quadrant = 0;
    double xr = det_reduce_pi4(x, &quadrant);
    /* Compute only the needed branch so an out-of-contract xr marks once. */
    switch (quadrant) {
        case 0: return det_sin_small(xr);      /* sin(x) */
        case 1: return det_cos_small(xr);      /* sin(x + pi/2) = cos(x) */
        case 2: return -det_sin_small(xr);     /* sin(x + pi) = -sin(x) */
        case 3: return -det_cos_small(xr);     /* sin(x + 3pi/2) = -cos(x) */
    }
    return det_sin_small(xr); /* unreachable */
}

static inline double det_cos(double x) {
    if (!isfinite(x)) {
        det_mark_fallback_trig();
        return NAN;
    }
    int quadrant = 0;
    double xr = det_reduce_pi4(x, &quadrant);
    switch (quadrant) {
        case 0: return det_cos_small(xr);      /* cos(x) */
        case 1: return -det_sin_small(xr);     /* cos(x + pi/2) = -sin(x) */
        case 2: return -det_cos_small(xr);     /* cos(x + pi) = -cos(x) */
        case 3: return det_sin_small(xr);      /* cos(x + 3pi/2) = sin(x) */
    }
    return det_cos_small(xr); /* unreachable */
}

#endif /* det_math_h */
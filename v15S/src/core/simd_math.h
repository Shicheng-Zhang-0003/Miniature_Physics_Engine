#ifndef simd_math_h
#define simd_math_h
/* SIMD fast path (DESPOT-2026-10-08): SSE2 vector3 ops, deterministic.
 *
 * Contract:
 * - IEEE-exact same results as scalar math3d.h (add/sub/mul are correctly
 *   rounded in both SSE2 and scalar-SSE; no FMA anywhere; -ffp-contract=off
 *   required). Bitwise twins hold with or without this header.
 * - Order of operations identical to scalar (x,y,z lanes independent;
 *   dot uses x*x + y*y + z*z left-to-right in double, same as scalar
 *   length_squared; callers needing float-dot use vector3_dot directly).
 * - Fallback scalar when __SSE2__ absent (ARM/RISC-V/MSVC without SSE).
 * - No unaligned loads: vector3 is 12 bytes, NOT 16; SIMD path uses
 *   _mm_set_ps (register shuffles, no memory alignment requirement).
 *
 * Use in hot loops (broadphase AABB, narrowphase deltas) via
 * simd_add/sub/scale; scalar code remains canonical for audits.
 */
#include <math.h>
#include <float.h>
#include "math3d.h"
#ifdef __SSE2__
#include <emmintrin.h>
static inline vector3 simd_add (vector3 a, vector3 b) {
    __m128 va = _mm_set_ps (0.0f, a.z, a.y, a.x);
    __m128 vb = _mm_set_ps (0.0f, b.z, b.y, b.x);
    __m128 vc = _mm_add_ps (va, vb);
    float out [4];
    _mm_storeu_ps (out, vc);
    return (vector3) {out [0], out [1], out [2]};
}
static inline vector3 simd_sub (vector3 a, vector3 b) {
    __m128 va = _mm_set_ps (0.0f, a.z, a.y, a.x);
    __m128 vb = _mm_set_ps (0.0f, b.z, b.y, b.x);
    __m128 vc = _mm_sub_ps (va, vb);
    float out [4];
    _mm_storeu_ps (out, vc);
    return (vector3) {out [0], out [1], out [2]};
}
static inline vector3 simd_scale (vector3 a, float s) {
    __m128 va = _mm_set_ps (0.0f, a.z, a.y, a.x);
    __m128 vs = _mm_set1_ps (s);
    __m128 vc = _mm_mul_ps (va, vs);
    float out [4];
    _mm_storeu_ps (out, vc);
    return (vector3) {out [0], out [1], out [2]};
}
static inline int simd_available (void) {
    return 1;
}
#else
static inline vector3 simd_add (vector3 a, vector3 b) {
    return vector3_addition (a, b);
}
static inline vector3 simd_sub (vector3 a, vector3 b) {
    return vector3_subtraction (a, b);
}
static inline vector3 simd_scale (vector3 a, float s) {
    return vector3_scaling (a, s);
}
static inline int simd_available (void) {
    return 0;
}
#endif
#endif

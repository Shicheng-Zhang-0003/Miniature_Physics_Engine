#ifndef simd_math_h
#define simd_math_h
/* SIMD lane-wise vector3 ops (DESPOT-2026-10-08, full since 2026-10-07).
 *
 * Covered: add, sub, scale, cross — every lane computes EXACTLY the scalar
 * op sequence (one correctly-rounded mul/add/sub per lane, no FMA;
 * -ffp-contract=off required), so results are bitwise identical to
 * math3d.h on every IEEE target. cross uses lane shuffles only (no
 * reduction), hence exact too. vector3 is 12 bytes (not 16): operands are
 * built with _mm_set_ps (register-only, no alignment requirement).
 * NOT covered, by proof: dot/length/normalize are REDUCTIONS — lane-sum
 * order changes rounding, so no SIMD order matches scalar left-to-right.
 * They stay scalar in math3d.h; this is completeness, not a gap.
 * Backends: SSE2 (x86_64, tested here) / MSVC <intrin.h> (same SSE2
 * intrinsics) / scalar fallback (MPE_SIMD_OFF, ARM, RISC-V — tested via
 * the scalar suite binary). NEON is future work, not a silent fallback. */
#include <math.h>
#include <float.h>
#include "math3d.h"
#if defined(__SSE2__) && !defined(MPE_SIMD_OFF)
#if defined(_MSC_VER)
#include <intrin.h>
#else
#include <emmintrin.h>
#endif
static inline __m128 simd_pack (vector3 a) {return _mm_set_ps (0.0f, a.z, a.y, a.x);}
static inline vector3 simd_unpack (__m128 v) {float out [4]; _mm_storeu_ps (out, v); return (vector3) {out [0], out [1], out [2]};}
static inline vector3 simd_add (vector3 a, vector3 b) {return simd_unpack (_mm_add_ps (simd_pack (a), simd_pack (b)));}
static inline vector3 simd_sub (vector3 a, vector3 b) {return simd_unpack (_mm_sub_ps (simd_pack (a), simd_pack (b)));}
static inline vector3 simd_scale (vector3 a, float s) {return simd_unpack (_mm_mul_ps (simd_pack (a), _mm_set1_ps (s)));}
static inline vector3 simd_cross (vector3 a, vector3 b) {
    /* Lane-exact cross: a1=[ay,az,ax], b1=[bz,bx,by] -> t1=[aybz,azbx,axby];
     * a2=[az,ax,ay], b2=[by,bz,bx] -> t2=[azby,axb z,aybx]; c=t1-t2. Each lane
     * repeats the scalar op sequence exactly (mul,mul,sub). NOTE the pairing:
     * mul(a1,b2)-mul(a2,b1) — same-mask pairing computes dot-like garbage
     * (caught by the suite going 22/42 red, DESPOT-2026-10-07). */
    __m128 va = simd_pack (a);
    __m128 vb = simd_pack (b);
    __m128 a1 = _mm_shuffle_ps (va, va, _MM_SHUFFLE (3, 0, 2, 1));
    __m128 b2 = _mm_shuffle_ps (vb, vb, _MM_SHUFFLE (3, 1, 0, 2));
    __m128 a2 = _mm_shuffle_ps (va, va, _MM_SHUFFLE (3, 1, 0, 2));
    __m128 b1 = _mm_shuffle_ps (vb, vb, _MM_SHUFFLE (3, 0, 2, 1));
    return simd_unpack (_mm_sub_ps (_mm_mul_ps (a1, b2), _mm_mul_ps (a2, b1)));
} static inline int simd_available (void) {return 1;}
#else
static inline vector3 simd_add (vector3 a, vector3 b) {return vector3_addition (a, b);}
static inline vector3 simd_sub (vector3 a, vector3 b) {return vector3_subtraction (a, b);}
static inline vector3 simd_scale (vector3 a, float s) {return vector3_scaling (a, s);}
static inline vector3 simd_cross (vector3 a, vector3 b) {return vector3_cross (a, b);}
static inline int simd_available (void) {return 0;}
#endif
#endif

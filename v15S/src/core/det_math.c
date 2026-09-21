/* Deterministic transcendentals — implementation file.
 * Contains FPU state pinning and fallback counters (process-wide).
 */
#include "det_math.h"
#include <assert.h>
#include <fenv.h>
#include <stdint.h>

unsigned long det_fallback_pow_count = 0;
unsigned long det_fallback_trig_count = 0;

void det_fallback_reset(void) {
    det_fallback_pow_count = 0;
    det_fallback_trig_count = 0;
}

void det_pin_fp_state(void) {
    fesetround(FE_TONEAREST);

#if defined(__x86_64__) || defined(__i386__)
    #ifdef __SSE__
    unsigned int mxcsr = 0;
    __asm__ volatile("stmxcsr %0" : "=m"(mxcsr));
    mxcsr &= ~(1u << 15);  // clear FTZ (bit 15)
    mxcsr &= ~(1u << 6);   // clear DAZ (bit 6; was 1u<<24 reserved — DAZ stayed as-boot)
    __asm__ volatile("ldmxcsr %0" :: "m"(mxcsr));
    #endif
#elif defined(__aarch64__)
    uint64_t fpcr;
    __asm__ volatile("mrs %0, fpcr" : "=r"(fpcr));
    fpcr &= ~((1ULL << 24) | (1ULL << 25));  // clear FZ (bit 24), DZ (bit 25)
    __asm__ volatile("msr fpcr, %0" :: "r"(fpcr));
#elif defined(__arm__) && defined(__VFP_FP__)
    uint32_t fpscr;
    __asm__ volatile("vmrs %0, fpscr" : "=r"(fpscr));
    fpscr &= ~((1u << 24) | (1u << 25));  // clear FZ, DZ
    __asm__ volatile("vmsr fpscr, %0" :: "r"(fpscr));
#endif
}

#ifndef NDEBUG
static _Thread_local bool det_fallback_pow_used = false;
static _Thread_local bool det_fallback_trig_used = false;

void det_assert_no_fallback_pow(void) {
    /* TRUTH: old body cleared the flag and returned unconditionally (never
     * fired). A desync tripwire that cannot trip is a lie. */
    assert(!det_fallback_pow_used);
}

void det_assert_no_fallback_trig(void) {
    assert(!det_fallback_trig_used);
}

void det_mark_fallback_pow(void) {
    det_fallback_pow_used = true;
    det_fallback_pow_count++;
}

void det_mark_fallback_trig(void) {
    det_fallback_trig_used = true;
    det_fallback_trig_count++;
}
#else
void det_mark_fallback_pow(void) { det_fallback_pow_count++; }
void det_mark_fallback_trig(void) { det_fallback_trig_count++; }
/* Release no-ops so the assert declarations in det_math.h always link.
 * Counters above stay active in release; only the debug tripwire is a no-op. */
void det_assert_no_fallback_pow(void) { }
void det_assert_no_fallback_trig(void) { }
#endif
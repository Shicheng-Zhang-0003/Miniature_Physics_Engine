/* Process-wide libm-fallback counters for det_math.h (see header). */
#include "det_math.h"

unsigned long det_fallback_pow_count = 0;
unsigned long det_fallback_trig_count = 0;

void det_fallback_reset(void) {
    det_fallback_pow_count = 0;
    det_fallback_trig_count = 0;
}

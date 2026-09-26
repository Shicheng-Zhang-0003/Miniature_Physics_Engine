/* MFS_PHASE_A: physics halt state extracted from simulation.c.
 * Owns physics_halted / physics_halt_ticks_remaining and the halt API.
 * GTK redraw/overlay stays in simulation.c (it needs the widget pointer).
 */
/* GTK4-PREP: zero GUI headers in core. */
#include <stdatomic.h>
#include <stdbool.h>

/* MPE_TASK_V15R2_PHYSICS_HALT_BEGIN */
/* FIX-AUDIT-DESPOT: the halt flag is written by UI/debug threads
 * (physics_halt_set/for_ticks) and read by the physics tick
 * (physics_halt_tick_update / physics_is_halted). Plain bool/int made
 * that a data race (UB; torn reads on some targets). _Atomic with
 * sequential consistency is the cheapest correct fix — no mutex needed,
 * single-word flag, tick path never blocks. */
static _Atomic int physics_halt_ticks_remaining = 0;
static _Atomic bool physics_halted = false;

void physics_halt_set(bool halted) {
    physics_halted = halted;
    if (!halted) {
        physics_halt_ticks_remaining = 0;
    }
}

void physics_halt_for_ticks(int ticks) {
    if (ticks <= 0) {
        ticks = 1;
    }
    physics_halt_ticks_remaining = ticks;
    physics_halted = true;
}

bool physics_is_halted(void) {
    return physics_halted;
}
/* MPE_TASK_V15R2_PHYSICS_HALT_END */


/* MFS_PHASE_A: per-tick halt bookkeeping. Returns true if physics should be
 * skipped this tick (timed halt counting down, or indefinite halt). */
bool physics_halt_tick_update(void) {
    if (physics_halt_ticks_remaining > 0) {
        physics_halt_ticks_remaining--;
        if (physics_halt_ticks_remaining == 0) {
            physics_halted = false;
        }
        return true;
    }
    if (physics_halted) {
        return true;
    }
    return false;
}

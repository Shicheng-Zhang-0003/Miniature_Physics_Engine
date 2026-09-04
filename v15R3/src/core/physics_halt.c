/* MFS_PHASE_A: physics halt state extracted from simulation.c.
 * Owns physics_halted / physics_halt_ticks_remaining and the halt API.
 * GTK redraw/overlay stays in simulation.c (it needs the widget pointer).
 */
#include "../mpe_engine.h"

/* MPE_TASK_V15R2_PHYSICS_HALT_BEGIN */
static int physics_halt_ticks_remaining = 0;
static bool physics_halted = false;

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

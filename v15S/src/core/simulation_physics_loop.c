/* Fixed-timestep GUI driver.
 *
 * The GTK callback owns only wall-clock accumulation, the anti-spiral clamp,
 * and the overlay counters. The simulation stage sequence lives in exactly
 * one place: physics_world_step().
 *
 * DESPOT-2026-10-23 (driver unification): this file used to carry a second,
 * hand-maintained copy of the whole pipeline. Every stage had to be mirrored
 * by hand, and it had already drifted (unclamped drag, stale sleep maxima,
 * integrate-before-narrow, late manifolds after split). The driver no longer
 * contains physics, so it cannot drift from the canonical step.
 */
#include "../config/mpe_config.h"
#include "physics_world.h"
#include "debug_counters.h"
#include <math.h>

void simulation_physics_tick (float frame_delta_time) {
    static double physics_time_accumulator = 0.0;
    const float fixed_physics_dt = 1.0f / 60.0f;
    physics_world *world = physics_world_get_primary ();
    if (!world) { return; }
    const mpe_config_t *cfg = mpe_world_cfg (world);
    int max_substeps_per_frame = cfg -> timestep.max_substeps;
    if (max_substeps_per_frame < 1) { max_substeps_per_frame = 1; }
    if (max_substeps_per_frame > 20) { max_substeps_per_frame = 20; }
    if (!isfinite (frame_delta_time) || (frame_delta_time < 0.0f)) { frame_delta_time = 0.0f; }
    physics_time_accumulator += (double) frame_delta_time;
    double catchup_cap = (double) fixed_physics_dt * (double) max_substeps_per_frame;
    if (physics_time_accumulator > catchup_cap) {
        /* Anti-spiral: drop excess wall time (sim lags wall) rather than
         * running unbounded catch-up steps. */
        physics_time_accumulator = catchup_cap;
    }
    /* The overlay value is per-frame; each substep contributes its delta. */
    debug_last_manifold_overflow_count = 0;
    while (physics_time_accumulator >= (double) fixed_physics_dt) {
        int overflow_before = world -> manifold_overflow_count;
        physics_world_step (world, fixed_physics_dt);
        debug_last_broadphase_pair_count = world -> last_broadphase_pair_count;
        debug_last_manifold_count = world -> last_manifold_count;
        debug_last_manifold_overflow_count += world -> manifold_overflow_count - overflow_before;
        physics_time_accumulator -= (double) fixed_physics_dt;
    }
}

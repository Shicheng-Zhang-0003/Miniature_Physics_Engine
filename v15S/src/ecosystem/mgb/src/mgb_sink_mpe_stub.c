/* mgb_sink_mpe_stub.c — MPE-sink CONTRACT for copy time into 475.
 *
 * Implemented at copy time against MPI (mpe_module.h) + physics_world.h.
 * This stub exists so the standalone tree builds green WITHOUT MPE
 * headers; it documents the exact mapping the real sink must perform.
 * The real sink MUST satisfy every MUST below; the suite gates them
 * against a recorded frame (see tests/test_mgb.c golden section).
 *
 * CONTRACT (MPE sink):
 *  MUST create one STATIC body per mapped body() call (static => the
 *    solver skips it; display-only becomes architectural fact, not a
 *    promise). Position = mapped mm as metres (1 mm display == 1e-3 m;
 *    the stage volume sits at a quarantined offset, e.g. y+=500, so S2
 *    content can never touch MPE play bodies).
 *  MUST reuse stable body ids across frames: S2 atom index i maps to the
 *    same MPE body every frame (move, never respawn). Added/removed atoms
 *    create/destroy bodies explicitly.
 *  MUST render bonds through the joint-line overlay path with zero-force
 *    springs (or a dedicated wire pass) — never as physical springs.
 *  MUST NOT write forces, velocities, or masses from S2 into live MPE
 *    physics state; MUST NOT touch determinism counters.
 *  MUST surface link health in the TUI ([s2bridge] section: frame seq,
 *    atom count, scale readout, error count) and freeze last-good frame
 *    with a banner on child death.
 *  MUST pin the module descriptor deterministic=false (display path).
 */
#include "mgb.h"

/* Standalone stub: present for link-compat, refuses at runtime. */
int mgb_sink_mpe_write (void *world, const mgb_frame_t *frame,
                        double mm_per_a) {
    (void)world;
    (void)frame;
    (void)mm_per_a;
    return -1;  /* not implemented until the 475 copy */
}

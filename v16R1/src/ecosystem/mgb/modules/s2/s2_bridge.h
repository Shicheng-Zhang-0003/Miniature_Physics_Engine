/* s2_bridge.h — MPI tick module owning a live s2tui session per world. */
#ifndef S2_BRIDGE_H
#define S2_BRIDGE_H

#include "core/mpe_module.h"

typedef struct {
    char s2tui_path[512];   /* explicit, else $MGB_S2TUI_BIN, else sibling */
    char scene[32];         /* "petri" (Phase 0 set) */
    double mm_per_a;        /* display scale (default MGB_MM_PER_A) */
    int every;              /* refresh cadence in ticks (0 = stage-once) */
    /* Stage offset. This is the point the dish's bounding-box CENTRE is
     * placed at — NOT a vector added to raw S2 coordinates — so it means
     * the same thing for every scene. Default (0,20,-60) is on the engine's
     * default view axis (camera at (0,20,50) looking down -Z, 45 deg
     * vertical FOV). Override with $MGB_OX / $MGB_OY / $MGB_OZ. */
    float ox, oy, oz;
} s2_bridge_cfg_t;

void s2_bridge_default_cfg (s2_bridge_cfg_t *cfg);
/* attach/configure/stage; detach tears down. pre_step refreshes. */
extern const mpe_module_desc_t mpe_module_desc; /* registry name is still "s2-bridge" */
/* headless/test query: frames staged, link errors, last atom count. */
int s2_bridge_stats (mpe_world_t *world, int *frames, int *errors,
                     int *atoms);

#endif
/* Resolution is exposed for gating: a hit must be an existing executable
 * with NO unresolved '..' segments. Returns 1 on a hit, 0 on a miss. */
int s2_bridge_resolve_s2tui (char *out, size_t out_cap);

/* mgb_sink_mpe.c — the REAL sink: S2 frames become MPE bodies + joints.
 *
 * Contract (from mgb_sink_mpe_stub.c, now implemented):
 *  - one STATIC sphere per body() call (solver skips statics: inverse mass
 *    zeroed by rigidbody_set_static — display-only is architectural fact).
 *  - positions: mapped mm used numerically as MPE metres ("1 unit = 1 mm
 *    S2-display", labeled in dumps) plus the stage offset (quarantine from
 *    play bodies; default (0,120,0), inside the +/-250 m boundary box).
 *  - bonds: spring joints with k=0, c=0, L0 = current length. Hooke and
 *    damping are both exactly zero, so the solver is untouched AND the
 *    renderer still draws the magenta joint lines. Never physical springs.
 *  - double->float conversion happens once here (documented rounding).
 *  - no forces, velocities, masses, or determinism counters are written.
 */
#include "mgb.h"
#include "core/physics_world.h"
#include "core/rigidbody.h"
#include "physics/spring_joint.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>

typedef struct {
    physics_world *world;
    /* Stage offset MINUS the frame's bounding-box centre. The offset is
     * therefore "where the dish's CENTRE goes", not a vector added to raw
     * S2 coordinates.
     *
     * DESPOT-2026-10-09: the offset used to be added to the raw coordinates,
     * which is only meaningful if the scene happens to sit at the origin. The
     * petri dish spans x 2..46, y 23..65, z -16..26 in S2 space, so a fixed
     * offset left its centre 21 degrees off the camera axis against a
     * 22.5-degree half-FOV — only 2 of the 8 bounding-box corners were on
     * screen and the attach still logged success. Centring on the measured
     * bounds makes the offset mean the same thing for every scene. */
    float ox, oy, oz;
    int *index_of;      /* S2 idx -> MPE body index (caller-sized) */
    int cap;
    int nbodies, nbonds;
    int failed;
} mpe_ctx_t;

static int mpe_body (void *ctx, int idx, int Z, double x, double y,
                     double z, double r, const float rgb[3]) {
    (void)Z;
    (void)rgb;  /* Phase 0: engine colour stays authorial; mapping recorded */
    mpe_ctx_t *c = ctx;
    if (!c || !c->world || idx < 0 || idx >= c->cap) {
        if (c) c->failed = 1;
        return -1;
    }
    if (!isfinite (x + y + z + r) || r <= 0.0) {
        c->failed = 1;
        return -1;
    }
    vector3 pos = { (float)(x + c->ox), (float)(y + c->oy),
                    (float)(z + c->oz) };
    int bi = physics_world_add_sphere (c->world, (float)r, 1.0f, pos);
    if (bi < 0) {
        c->failed = 1;
        return -1;
    }
    rigidbody_set_static (&c->world->bodies[bi], true);
    c->index_of[idx] = bi;
    c->nbodies++;
    return 0;
}

static int mpe_bond (void *ctx, int a, int b) {
    mpe_ctx_t *c = ctx;
    if (!c || !c->world || a < 0 || b < 0 || a >= c->cap || b >= c->cap) {
        if (c) c->failed = 1;
        return -1;
    }
    int ia = c->index_of[a], ib = c->index_of[b];
    if (ia < 0 || ib < 0) {
        c->failed = 1;
        return -1;
    }
    vector3 d = vector3_subtraction (c->world->bodies[ib].position,
                                     c->world->bodies[ia].position);
    float len = vector3_length (d);
    if (!isfinite (len)) {
        c->failed = 1;
        return -1;
    }
    /* k=0, c=0: force-free by construction, still rendered. */
    int j = add_joint_by_ids (c->world, c->world->bodies[ia].object_id,
                              c->world->bodies[ib].object_id, len, 0.0f,
                              0.0f);
    if (j < 0) {
        c->failed = 1;
        return -1;
    }
    c->nbonds++;
    return 0;
}

static int mpe_finish (void *ctx) {
    (void)ctx;
    return 0;
}

static const mgb_sink_t MPE_SINK = { mpe_body, mpe_bond, mpe_finish };

/* Stage a whole frame: bodies then bonds. index_map must hold natoms ints
 * (filled with MPE body indices). Returns staged body count, -1 on failure
 * (world left with whatever staged before the failure — caller tears down
 * by body-count checkpoint; documented, Phase 0). */
int mgb_sink_mpe_stage (physics_world *world, const mgb_frame_t *frame,
                        double mm_per_a, float ox, float oy, float oz,
                        int *index_map, int map_cap) {
    if (!world || !frame || !index_map || map_cap < frame->natoms)
        return -1;
    for (int i = 0; i < frame->natoms; i++) index_map[i] = -1;
    mpe_ctx_t c;
    c.world = world;
    c.index_of = index_map;
    c.cap = map_cap;
    c.nbodies = 0;
    c.nbonds = 0;
    c.failed = 0;
    /* Measure the frame's bounds (in the same scaled units the mapper emits)
     * so the offset can target the dish centre rather than the scene origin. */
    double lo [3] = {1e30, 1e30, 1e30}, hi [3] = {-1e30, -1e30, -1e30};
    for (int i = 0; i < frame->natoms; i++) {
        const mgb_atom_t *a = &frame->atoms [i];
        double p [3] = {a->x * mm_per_a, a->y * mm_per_a, a->z * mm_per_a};
        for (int k = 0; k < 3; k++) {
            if (p [k] < lo [k]) lo [k] = p [k];
            if (p [k] > hi [k]) hi [k] = p [k];
        }
    }
    if (frame->natoms > 0) {
        c.ox = (float) (ox - (lo [0] + hi [0]) * 0.5);
        c.oy = (float) (oy - (lo [1] + hi [1]) * 0.5);
        c.oz = (float) (oz - (lo [2] + hi [2]) * 0.5);
    } else {
        c.ox = ox;
        c.oy = oy;
        c.oz = oz;
    }
    joint_init_pool (world);
    if (mgb_map (frame, &MPE_SINK, &c, mm_per_a) != 0 || c.failed) return -1;
    return c.nbodies;
}

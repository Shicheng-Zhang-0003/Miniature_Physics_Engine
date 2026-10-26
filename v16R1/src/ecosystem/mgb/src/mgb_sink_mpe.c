/* mgb_sink_mpe.c — the REAL sink: S2 frames become MPE bodies + joints.
 *
 * Phase 0 (attach): one kinematic body per mapped body() call. Phase 1
 * (refresh): move() repositions existing
 * bodies in place (never respawns — stable S2 idx -> MPE body mapping).
 *
 * Contract:
 *  - bodies: kinematic (effective inverse mass 0), so gravity and solver
 *    impulses cannot move the display objects. MPE bodies can still collide
 *    with them as immovable obstacles. S2 positions are applied on refresh.
 *  - bonds: spring joints k=0, c=0, L0 = current length. Force-free by
 *    construction, renderer still draws magenta joint lines.
 *  - double->float conversion happens once here (documented rounding).
 *  - the bridge is display-only: it applies positions and clears staged
 *    velocities/forces, but does not couple MPE state back into S2.
 */
#include "mgb.h"
#include "core/physics_world.h"
#include "core/rigidbody.h"
#include "physics/spring_joint.h"
#include "physics/constraint.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    physics_world *world;
    /* Stage offset MINUS the frame's bounding-box centre. */
    float ox, oy, oz;
    int *index_of;      /* S2 idx -> MPE body index (caller-sized) */
    int cap;
    int nbodies, nbonds;
    int failed;
    /* Phase 1: generation counter to detect stale index_map entries. */
    unsigned stage_gen;
} mpe_ctx_t;

/* Find an existing spring joint between two object ids, or -1. */
static int find_spring_by_ids (physics_world *world, uint32_t id_a, uint32_t id_b) {
    if (!world) { return -1; }
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!world -> spring_joints [i].is_active) { continue; }
        if ((world -> spring_joints [i].object_id_a == id_a &&
             world -> spring_joints [i].object_id_b == id_b) ||
            (world -> spring_joints [i].object_id_a == id_b &&
             world -> spring_joints [i].object_id_b == id_a)) {
            return i;
        }
    } return -1;
}
static int mpe_body (void *ctx, int idx, int Z, double x, double y,
                     double z, double r, const float rgb[3]) {
    (void)Z;
    (void)rgb;
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
    /* Phase 1: move-or-create. If the S2 index already maps to a body,
     * reposition it in place (never respawn — stable mapping). */
    int bi = c->index_of [idx];
    if (bi >= 0 && bi < c->world->body_count) {
        rigidbody *rb = &c->world->bodies [bi];
        rigidbody_set_kinematic (rb, true);
        rb -> position = pos;
        /* Kinematic integration is skipped; keep imported display state
         * free of residual motion and forces as a defensive invariant. */
        rb -> velocity = vector3_zero ();
        rb -> angular_velocity = vector3_zero ();
        rb -> force_accumulator = vector3_zero ();
        rb -> torque_accumulator = vector3_zero ();
        return 0;
    }
    /* Not yet staged: create a sphere then make it kinematic. This keeps it
     * visible and collidable while excluding it from gravity and impulses. */
    bi = physics_world_add_sphere (c->world, (float)r, 1.0f, pos);
    if (bi < 0) {
        c->failed = 1;
        return -1;
    }
    rigidbody_set_kinematic (&c->world->bodies [bi], true);
    c->index_of [idx] = bi;
    c->nbodies++;
    return 0;
}

static int mpe_bond (void *ctx, int a, int b) {
    mpe_ctx_t *c = ctx;
    if (!c || !c->world || a < 0 || b < 0 || a >= c->cap || b >= c->cap) {
        if (c) c->failed = 1;
        return -1;
    }
    int ia = c->index_of [a], ib = c->index_of [b];
    if (ia < 0 || ib < 0) {
        c->failed = 1;
        return -1;
    }
    uint32_t id_a = c->world->bodies [ia].object_id;
    uint32_t id_b = c->world->bodies [ib].object_id;
    /* Phase 1: update-or-create. Reuse an existing spring joint between
     * these two bodies, refreshing L0 to the current separation. */
    int ji = find_spring_by_ids (c->world, id_a, id_b);
    if (ji >= 0) {
        c->world->spring_joints [ji].equilibrium_length =
            vector3_length (vector3_subtraction (c->world->bodies [ib].position,
                                                 c->world->bodies [ia].position));
        c->nbonds++;
        return 0;
    }
    vector3 d = vector3_subtraction (c->world->bodies [ib].position,
                                     c->world->bodies [ia].position);
    float len = vector3_length (d);
    if (!isfinite (len)) {
        c->failed = 1;
        return -1;
    }
    /* k=0, c=0: force-free by construction, still rendered. */
    int j = add_joint_by_ids (c->world, id_a, id_b, len, 0.0f, 0.0f);
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

/* Phase 1: reposition every already-staged body to its new S2 location.
 * Called from bridge_pre_step on a refresh tick. index_map must already be
 * populated (from initial stage). Returns 0 ok, -1 on failure. */
int mgb_sink_mpe_move (physics_world *world, const mgb_frame_t *frame,
                       double mm_per_a, float ox, float oy, float oz,
                       int *index_map, int map_cap) {
    if (!world || !frame || !index_map || map_cap < frame->natoms)
        return -1;
    mpe_ctx_t c;
    c.world = world;
    c.index_of = index_map;
    c.cap = map_cap;
    c.nbodies = 0;
    c.nbonds = 0;
    c.failed = 0;
    c.ox = ox;
    c.oy = oy;
    c.oz = oz;
    /* Recompute offset to target the frame's bounding-box centre (same as
     * stage, so the dish doesn't drift between stage and refresh). */
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
    }
    /* Bodies: reposition existing, create any new. */
    if (mgb_map (frame, &MPE_SINK, &c, mm_per_a) != 0 || c.failed) return -1;
    /* Bonds: reposition existing, create any new. */
    return 0;
}

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
    c.stage_gen = 0;
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
    /* physics_world_init owns pool initialization. Clearing either pool
     * here would erase constraints already present in the caller's scene. */
    if (mgb_map (frame, &MPE_SINK, &c, mm_per_a) != 0 || c.failed) return -1;
    return c.nbodies;
}

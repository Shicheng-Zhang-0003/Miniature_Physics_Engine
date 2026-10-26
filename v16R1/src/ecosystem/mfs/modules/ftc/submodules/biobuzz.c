/* BIOBUZZ regulation field + league implementation (FTC 2026-2027 Manual V1).
 * See biobuzz.h for the contract, orientation convention, and what is NOT
 * modelled (dynamic bi-stable damper solve, match clock, fouls, vision).
 *
 * Coordinate truth: metres, y=0 at tile top, field centre at origin.
 * -Z = audience, +Z = scoring/far, -X = RED wall, +X = BLUE wall.
 * All repose geometry below is NOMINAL (manual ±1 in tolerance culture);
 * gates assert self-consistency + manual-exact anchor numbers (field half,
 * pivot height, cell gap, opening size, piece sizes/counts), never derived
 * nominals dressed as spec.
 */
#include "biobuzz.h"
#include "robot.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ---- small builders ---- */
static int bb_static_box (physics_world *world, vector3 pos, vector3 half, vector4 ori, float mus, float muk,
                          float rest, vector3 col, int no_collide) {
    int bi = physics_world_add_cube (world, pos, half, 0.0f);
    if (bi < 0) { return -1; }
    rigidbody *b = &world -> bodies [bi];
    b -> friction_static = mus;
    b -> friction_kinetic = muk;
    b -> restitution = rest;
    b -> colour = col;
    b -> orientation = ori;
    rigidbody_update_axes (b);
    b -> no_collide = no_collide;
    return bi;
}
static vector4 bb_quat_x (float angle) {
    /* Rotation about +X by angle (engine quat convention (w,x,y,z),
     * verified against bucket 20 deg literals: roll about X has x=sin). */
    float h = 0.5f * angle;
    vector4 q = {cosf (h), sinf (h), 0.0f, 0.0f};
    return q;
}
static vector4 bb_quat_yaw (float yaw) {
    float h = 0.5f * yaw;
    vector4 q = {cosf (h), 0.0f, sinf (h), 0.0f};
    return q;
}
/* Rotate local->world by orientation quaternion. */
static vector3 bb_rotate (vector4 q, vector3 v) {return vector4_rotate_to_vector3 (q, v);}
/* World->local (transpose = inverse for unit quats). */
static vector3 bb_unrotate (vector4 q, vector3 v) {
    math3 R = vector4_to_math3 (q);
    math3 Rt = math3_transposition (R);
    return math3_multiplication_vector3 (Rt, v);
}
static int bb_ball_interior (vector3 p, vector3 c, vector3 half) {
    return (fabsf (p.x - c.x) <= half.x) && (fabsf (p.y - c.y) <= half.y) && (fabsf (p.z - c.z) <= half.z);
}
/* Ball at least partially inside an axis box (Manual: flower/garden score
 * "at least partially in"). Box expanded by ball radius. */
static int bb_ball_partial (vector3 p, float r, vector3 c, vector3 half) {
    return (fabsf (p.x - c.x) <= half.x + r) && (fabsf (p.y - c.y) <= half.y + r) &&
           (fabsf (p.z - c.z) <= half.z + r);
}

/* ---- hive pose ---- */
/* Cell tilt quats (rotation about X; mouth local +Z). Derived in biobuzz.c
 * header comment from the 30-deg-elevation mouth normals:
 * up/out=+Z:a=-30, up/out=-Z:a=-150, down/out=+Z:a=+30, down/out=-Z:a=+150. */
static vector4 bb_cell_quat (int up, float outboard) {
    float deg = up ? (outboard > 0.0f ? -30.0f : -150.0f) : (outboard > 0.0f ? 30.0f : 150.0f);
    return bb_quat_x (deg * 0.01745329252f);
}
/* Cell centre for hive `hive` (0 red @ -X, 1 blue @ +X), end `end` (-1 = -Z
 * cellA, +1 = +Z cellB), given which end is UP (`up_end` = ±1). */
static void bb_cell_pose (int hive, int end, int up_end, vector3 *out_c, vector4 *out_q) {
    float hx = (hive == BIOBUZZ_HIVE_RED) ? -BIOBUZZ_HIVE_DX : BIOBUZZ_HIVE_DX;
    int is_up = (end == up_end) ? 1 : 0;
    float outboard = (float) end; /* mouth faces its own end, tilted */
    /* Mirror-symmetric about the pivot: up cells ride high/out, down low/out. */
    float uy = is_up ? BIOBUZZ_CELL_UP_DY : -BIOBUZZ_CELL_UP_DY;
    float uz = (float) end * BIOBUZZ_CELL_END_DZ; /* 18.8 in / 2: manual-exact */
    out_c -> x = hx;
    out_c -> y = BIOBUZZ_PIVOT_H + uy;
    out_c -> z = uz;
    *out_q = bb_cell_quat (is_up, outboard);
}
/* The 5 wall boxes of a cell in cell-local frame (mouth +Z, interior
 * W x H x D centred at origin). Returns count (5). */
static int bb_cell_walls (vector3 pos_out [5], vector3 half_out [5]) {
    float W = BIOBUZZ_CELL_W, H = BIOBUZZ_CELL_H, D = BIOBUZZ_CELL_D, t = BIOBUZZ_CELL_SKIN;
    pos_out [0] = (vector3) {0.0f, -H * 0.5f - t * 0.5f, 0.0f};
    half_out [0] = (vector3) {W * 0.5f + t, t * 0.5f, D * 0.5f + t};
    pos_out [1] = (vector3) {0.0f, H * 0.5f + t * 0.5f, 0.0f};
    half_out [1] = (vector3) {W * 0.5f + t, t * 0.5f, D * 0.5f + t};
    pos_out [2] = (vector3) {-W * 0.5f - t * 0.5f, 0.0f, 0.0f};
    half_out [2] = (vector3) {t * 0.5f, H * 0.5f + t, D * 0.5f + t};
    pos_out [3] = (vector3) {W * 0.5f + t * 0.5f, 0.0f, 0.0f};
    half_out [3] = (vector3) {t * 0.5f, H * 0.5f + t, D * 0.5f + t};
    pos_out [4] = (vector3) {0.0f, 0.0f, -D * 0.5f - t * 0.5f};
    half_out [4] = (vector3) {W * 0.5f + t, H * 0.5f + t, t * 0.5f};
    return 5;
}
/* Up-cell interior census box (world axis box around the tilted cell is
 * wrong for tilted cells, so census transforms into the cell frame). */
static void bb_up_cell_frame (int hive, int up_end, vector3 *out_c, vector4 *out_q) {
    bb_cell_pose (hive, up_end, up_end, out_c, out_q);
}
/* Weight of tracked dynamic balls inside the up-cell interior. */
static float bb_up_cell_weight (physics_world *world, int hive, int up_end, const int *piece_ids, int piece_n,
                                const int *piece_is_nectar) {
    vector3 cc;
    vector4 cq;
    bb_up_cell_frame (hive, up_end, &cc, &cq);
    vector3 half = {BIOBUZZ_CELL_W * 0.5f - 0.005f, BIOBUZZ_CELL_H * 0.5f - 0.005f,
                    BIOBUZZ_CELL_D * 0.5f - 0.005f};
    float wsum = 0.0f;
    for (int i = 0; i < piece_n; i++) {
        int bi = physics_world_index_by_id (world, (uint32_t) piece_ids [i]);
        if (bi < 0 || bi >= world -> body_count) { continue; }
        rigidbody *b = &world -> bodies [bi];
        if (b -> kinematic || b -> static_state) { continue; }
        vector3 rel = {b -> position.x - cc.x, b -> position.y - cc.y, b -> position.z - cc.z};
        vector3 loc = bb_unrotate (cq, rel);
        if (bb_ball_interior (loc, (vector3) {0.0f, 0.0f, 0.0f}, half)) {
            wsum += piece_is_nectar [i] ? BIOBUZZ_NECTAR_WEIGHT : BIOBUZZ_POLLEN_WEIGHT;
        }
    }
    return wsum;
}

/* ---- field build ---- */
static const vector3 BB_RED = {0.85f, 0.15f, 0.15f};
static const vector3 BB_BLUE = {0.20f, 0.35f, 0.90f};
static const vector3 BB_FRAME = {0.45f, 0.47f, 0.50f};
static const vector3 BB_CELL_RED = {0.90f, 0.25f, 0.25f};
static const vector3 BB_CELL_BLUE = {0.30f, 0.45f, 0.95f};
static const vector3 BB_FLOWER = {0.20f, 0.65f, 0.25f};
static const vector3 BB_TAPE_RED = {0.95f, 0.30f, 0.30f};
static const vector3 BB_TAPE_BLUE = {0.30f, 0.50f, 1.00f};

/* Idempotent probe: the REGULATION slab (half ~= 1.8288, top ~= 0). A
 * bigger pad (arcade 60 m, test 20 m) does NOT satisfy it — coplanar
 * stacking with a big pad is physical (same contact plane) and intended:
 * the pad is the outside ground, the slab is the tile field. */
static int bb_has_reg_floor (physics_world *world) {
    for (int i = 0; i < world -> body_count; i++) {
        rigidbody *b = &world -> bodies [i];
        if (!b -> static_state && !(b -> mass == 0.0f)) { continue; }
        float top = b -> position.y + b -> half_extensions.y;
        if (fabsf (top) < 0.01f && fabsf (b -> half_extensions.x - BIOBUZZ_FIELD_HALF) < 0.02f &&
            fabsf (b -> half_extensions.z - BIOBUZZ_FIELD_HALF) < 0.02f) { return 1; }
    }
    return 0;
}
static int bb_has_walls (physics_world *world) {
    /* All four regulation wall stations occupied by a static thin box. */
    float H = BIOBUZZ_FIELD_HALF;
    const float px [4] = {H, -H, 0.0f, 0.0f};
    const float pz [4] = {0.0f, 0.0f, H, -H};
    for (int p = 0; p < 4; p++) {
        int have = 0;
        for (int i = 0; i < world -> body_count; i++) {
            rigidbody *b = &world -> bodies [i];
            if (!b -> static_state && !(b -> mass == 0.0f)) { continue; }
            if (fabsf (b -> position.x - px [p]) < 0.3f && fabsf (b -> position.z - pz [p]) < 0.3f &&
                (b -> half_extensions.x < 0.5f || b -> half_extensions.z < 0.5f)) {
                have = 1;
                break;
            }
        }
        if (!have) { return 0; }
    }
    return 1;
}

int biobuzz_build_field (physics_world *world, biobuzz_state_t *st) {
    if (!world) { return -1; }
    /* State-verified idempotency: same state + live beams => no-op. A new
     * state on an existing field rebuilds (clear the scene first — same
     * rule as robots; append-only pool has no removal). A cleared scene
     * under a live state falls through and rebuilds. */
    if (st && st -> field_built) {
        int live = 1;
        for (int h = 0; h < 2; h++) {
            if (physics_world_index_by_id (world, (uint32_t) st -> hive_bodies [h][0]) < 0) { live = 0; }
        }
        if (live) { return 0; }
        st -> field_built = 0;
    }
    vector4 ident = vector4_identity ();
    /* Regulation floor slab (foam tiles; mu nominal — Manual states no
     * friction value; 1.0/0.8 matches the proven tile envelope. Neutral
     * restitution so pieces keep their own bounce via min-combine). */
    if (!bb_has_reg_floor (world)) {
        float H = BIOBUZZ_FIELD_HALF;
        int f = bb_static_box (world, (vector3) {0.0f, -0.5f, 0.0f}, (vector3) {H, 0.5f, H}, ident, 1.0f, 0.8f, 1.0f,
                               (vector3) {0.55f, 0.58f, 0.60f}, 0);
        if (f < 0) { return -1; }
    }
    /* Perimeter walls: inner faces exactly at ±FIELD_HALF (controlled dim),
     * 12 in tall. Neutral restitution (pieces keep their bounce). */
    if (!bb_has_walls (world)) {
        float H = BIOBUZZ_FIELD_HALF, T = BIOBUZZ_WALL_T, WH = BIOBUZZ_WALL_H;
        const vector3 pp [4] = {{H + T, WH * 0.5f, 0.0f}, {-H - T, WH * 0.5f, 0.0f}, {0.0f, WH * 0.5f, H + T},
                                {0.0f, WH * 0.5f, -H - T}};
        const vector3 ss [4] = {{T, WH * 0.5f, H + T}, {T, WH * 0.5f, H + T}, {H + T, WH * 0.5f, T},
                                {H + T, WH * 0.5f, T}};
        for (int p = 0; p < 4; p++) {
            vector3 col = (p < 2) ? ((p == 0) ? BB_BLUE : BB_RED) : BB_FRAME;
            if (bb_static_box (world, pp [p], ss [p], ident, 1.0f, 0.8f, 1.0f, col, 0) < 0) { return -1; }
        }
    }
    /* Hive frame: 4 legs at the frame corners + apex crossbar along X at
     * pivot height + 2 pivot blocks. Nominal triangular-leg approximation
     * (manual frame is two triangular structures + crossbar; legs carry the
     * same footprint and pivot line). */
    {
        float fw = BIOBUZZ_FRAME_W * 0.5f, fd = BIOBUZZ_FRAME_D * 0.5f;
        float leg_h = BIOBUZZ_PIVOT_H;
        const vector3 legs [4] = {{-fw, leg_h * 0.5f, -fd}, {fw, leg_h * 0.5f, -fd}, {-fw, leg_h * 0.5f, fd},
                                  {fw, leg_h * 0.5f, fd}};
        for (int i = 0; i < 4; i++) {
            if (bb_static_box (world, legs [i], (vector3) {0.03f, leg_h * 0.5f, 0.03f}, ident, 0.6f, 0.5f, 0.2f,
                               BB_FRAME, 0) < 0) { return -1; }
        }
        if (bb_static_box (world, (vector3) {0.0f, BIOBUZZ_PIVOT_H, 0.0f}, (vector3) {fw + 0.03f, 0.025f, 0.025f},
                           ident, 0.6f, 0.5f, 0.2f, BB_FRAME, 0) < 0) { return -1; }
        for (int h = 0; h < 2; h++) {
            float hx = (h == 0) ? -BIOBUZZ_HIVE_DX : BIOBUZZ_HIVE_DX;
            if (bb_static_box (world, (vector3) {hx, BIOBUZZ_PIVOT_H, 0.0f}, (vector3) {0.04f, 0.04f, 0.04f}, ident,
                               0.6f, 0.5f, 0.2f, BB_FRAME, 0) < 0) { return -1; }
        }
    }
    /* Hives: beam + 2 open cells each, posed per §11.1 setup (red cellA
     * (-Z/audience) UP; blue cellB (+Z/scoring) UP). Bodies recorded for
     * kinematic tip re-pose. */
    if (st) {
        const int up_end [2] = {-1, 1};
        for (int h = 0; h < 2; h++) {
            st -> tilt [h] = up_end [h];
            vector3 piv = {(h == 0) ? -BIOBUZZ_HIVE_DX : BIOBUZZ_HIVE_DX, BIOBUZZ_PIVOT_H, 0.0f};
            vector3 ca, cb;
            vector4 qa, qb;
            bb_cell_pose (h, -1, up_end [h], &ca, &qa);
            bb_cell_pose (h, 1, up_end [h], &cb, &qb);
            (void) piv;
            /* Beam between cell centres. */
            vector3 span = vector3_subtraction (cb, ca);
            float sl = sqrtf (span.x * span.x + span.y * span.y + span.z * span.z);
            if (sl < 1e-6f) { sl = 1e-6f; }
            float ang = atan2f (-(span.y / sl), span.z / sl);
            vector3 mid = {(ca.x + cb.x) * 0.5f, (ca.y + cb.y) * 0.5f, (ca.z + cb.z) * 0.5f};
            vector3 bcol = (h == 0) ? BB_CELL_RED : BB_CELL_BLUE;
            int beam = bb_static_box (world, mid, (vector3) {0.03f, 0.03f, sl * 0.5f}, bb_quat_x (ang), 0.6f, 0.5f,
                                      0.2f, bcol, 0);
            if (beam < 0) { return -1; }
            st -> hive_bodies [h][0] = world -> bodies [beam].object_id;
            const vector3 centres [2] = {ca, cb};
            const vector4 quats [2] = {qa, qb};
            for (int c = 0; c < 2; c++) {
                vector3 lp [5], lh [5];
                bb_cell_walls (lp, lh);
                for (int k = 0; k < 5; k++) {
                    vector3 off = bb_rotate (quats [c], lp [k]);
                    vector3 wp = {centres [c].x + off.x, centres [c].y + off.y, centres [c].z + off.z};
                    int bi = bb_static_box (world, wp, lh [k], quats [c], 0.5f, 0.4f, 0.3f, bcol, 0);
                    if (bi < 0) { return -1; }
                    st -> hive_bodies [h][1 + c * 5 + k] = world -> bodies [bi].object_id;
                }
            }
            /* Up-cell census volume (mouth centre + interior half). */
            vector3 ucc, dummy_c;
            vector4 ucq;
            bb_up_cell_frame (h, up_end [h], &ucc, &ucq);
            (void) dummy_c;
            st -> up_mouth [h][0] = ucc.x;
            st -> up_mouth [h][1] = ucc.y;
            st -> up_mouth [h][2] = ucc.z;
            st -> up_half [h][0] = BIOBUZZ_CELL_W * 0.5f;
            st -> up_half [h][1] = BIOBUZZ_CELL_H * 0.5f;
            st -> up_half [h][2] = BIOBUZZ_CELL_D * 0.5f;
        }
        st -> field_built = 1;
    }
    /* Flowers x4: nominal stations (±36 in, on audience/far walls). Each:
     * 4 pipes, top ring (4-box 4 in opening), backstop (outer side), middle
     * ring (4-box mid opening), lower ring seat, retrieval gap facing the
     * field centre. Static. Anchor id recorded. */
    {
        const float fx [4] = {-0.9144f, 0.9144f, -0.9144f, 0.9144f};
        const float fz [4] = {-BIOBUZZ_FIELD_HALF + 0.10f, -BIOBUZZ_FIELD_HALF + 0.10f, BIOBUZZ_FIELD_HALF - 0.10f,
                              BIOBUZZ_FIELD_HALF - 0.10f};
        for (int f = 0; f < 4; f++) {
            float cx = fx [f], cz = fz [f];
            float inward = (cz < 0.0f) ? 1.0f : -1.0f; /* field-centre facing */
            float top_y = BIOBUZZ_FLOWER_TOP_H;
            float mid_y = top_y - BIOBUZZ_FLOWER_RING_GAP;
            float topen = BIOBUZZ_FLOWER_TOP_D * 0.5f;
            float mopen = BIOBUZZ_FLOWER_MID_OPEN * 0.5f;
            float pr = BIOBUZZ_FLOWER_PIPE_R;
            /* Pipes: 4 at ring corners, lower ring -> top ring. */
            for (int q = 0; q < 4; q++) {
                float ox = (q % 2 == 0) ? -topen : topen;
                float oz = (q < 2) ? -topen : topen;
                float ph = (top_y - 0.01f) * 0.5f;
                if (bb_static_box (world, (vector3) {cx + ox, 0.01f + ph, cz + oz},
                                   (vector3) {pr, ph, pr}, ident, 0.5f, 0.4f, 0.3f, BB_FLOWER, 0) < 0) { return -1; }
            }
            /* Top ring: 4 boxes around a 4 in opening. */
            float tt = 0.012f;
            const vector3 trp [4] = {{cx, top_y, cz - topen - tt * 0.5f}, {cx, top_y, cz + topen + tt * 0.5f},
                                     {cx - topen - tt * 0.5f, top_y, cz}, {cx + topen + tt * 0.5f, top_y, cz}};
            const vector3 trh [4] = {{topen + tt, tt * 0.5f, tt * 0.5f},
                                     {topen + tt, tt * 0.5f, tt * 0.5f},
                                     {tt * 0.5f, tt * 0.5f, topen + tt},
                                     {tt * 0.5f, tt * 0.5f, topen + tt}};
            int anchor = -1;
            for (int q = 0; q < 4; q++) {
                int bi = bb_static_box (world, trp [q], trh [q], ident, 0.5f, 0.4f, 0.3f, BB_FLOWER, 0);
                if (bi < 0) { return -1; }
                if (q == 0) { anchor = (int) world -> bodies [bi].object_id; }
            }
            /* Backstop: outer side (wall side), 1.25 in tall. */
            if (bb_static_box (world, (vector3) {cx, top_y + BIOBUZZ_FLOWER_BACKSTOP_H * 0.5f,
                                                 cz - inward * (topen + 0.006f)},
                               (vector3) {topen + tt, BIOBUZZ_FLOWER_BACKSTOP_H * 0.5f, 0.006f}, ident, 0.5f, 0.4f,
                               0.3f, BB_FLOWER, 0) < 0) { return -1; }
            /* Middle ring: 4 boxes around the mid opening (pollen passes). */
            for (int q = 0; q < 4; q++) {
                vector3 mp = trp [q];
                mp.y = mid_y;
                vector3 mh = trh [q];
                /* widen the ring boxes to the mid opening */
                if (q < 2) { mh.x = mopen + tt; } else { mh.z = mopen + tt; }
                mp.x = cx + ((q == 2) ? -(mopen + tt * 0.5f) : ((q == 3) ? (mopen + tt * 0.5f) : 0.0f));
                mp.z = cz + ((q == 0) ? -(mopen + tt * 0.5f) : ((q == 1) ? (mopen + tt * 0.5f) : 0.0f));
                if (bb_static_box (world, mp, mh, ident, 0.5f, 0.4f, 0.3f, BB_FLOWER, 0) < 0) { return -1; }
            }
            /* Lower ring seat: flat ring with the pollen seat hole; the
             * retrieval gap faces the field centre (front segment omitted
             * below mid height on the inward side). */
            float ly = 0.005f;
            float lopen = BIOBUZZ_FLOWER_LOW_HOLE_D * 0.5f;
            const vector3 lrp [3] = {{cx, ly, cz - lopen - 0.02f}, {cx - lopen - 0.02f, ly, cz},
                                     {cx + lopen + 0.02f, ly, cz}};
            const vector3 lrh [3] = {{lopen + 0.02f, 0.005f, 0.02f}, {0.02f, 0.005f, lopen + 0.02f},
                                     {0.02f, 0.005f, lopen + 0.02f}};
            for (int q = 0; q < 3; q++) {
                if (bb_static_box (world, lrp [q], lrh [q], ident, 0.6f, 0.5f, 0.1f, BB_FLOWER, 0) < 0) { return -1; }
            }
            /* AprilTag-stance marker: dark underside plate (rendering has no
             * decals; tag cluster IDs live in state/docs for autonomy). */
            if (st) { st -> flower_ids [f] = anchor; }
        }
    }
    /* Zone tape proxies (thin no-collide slabs, §9-accurate footprints):
     * loading zones (A5 red / F2 blue), gardens (A1 red / F6 blue),
     * alliance areas (outside side walls). */
    {
        float H = BIOBUZZ_FIELD_HALF;
        /* Loading: tile A5 centre (-1.524, +0.914), tile F2 (+1.524, -0.914);
         * 11 in deep from the side wall. */
        const float lzx [2] = {-(H - BIOBUZZ_LOAD_D * 0.5f), H - BIOBUZZ_LOAD_D * 0.5f};
        const float lzz [2] = {0.9144f, -0.9144f};
        /* Gardens: 23 in along the side wall from the corner, 2 in out. */
        const float gzx [2] = {-(H - BIOBUZZ_GARDEN_D * 0.5f), H - BIOBUZZ_GARDEN_D * 0.5f};
        const float gzz [2] = {-(H - BIOBUZZ_GARDEN_W * 0.5f), H - BIOBUZZ_GARDEN_W * 0.5f};
        /* Alliance areas outside the side walls. */
        const float ax [2] = {-(H + BIOBUZZ_ALLIANCE_D * 0.5f), H + BIOBUZZ_ALLIANCE_D * 0.5f};
        for (int a = 0; a < 2; a++) {
            vector3 tc = (a == 0) ? BB_TAPE_RED : BB_TAPE_BLUE;
            if (bb_static_box (world, (vector3) {lzx [a], 0.002f, lzz [a]},
                               (vector3) {BIOBUZZ_LOAD_D * 0.5f, 0.002f, BIOBUZZ_LOAD_W * 0.5f}, ident, 0.5f, 0.5f,
                               0.0f, tc, 1) < 0) { return -1; }
            if (bb_static_box (world, (vector3) {gzx [a], 0.002f, gzz [a]},
                               (vector3) {BIOBUZZ_GARDEN_D * 0.5f, 0.002f, BIOBUZZ_GARDEN_W * 0.5f}, ident, 0.5f,
                               0.5f, 0.0f, tc, 1) < 0) { return -1; }
            if (bb_static_box (world, (vector3) {ax [a], 0.002f, 0.0f},
                               (vector3) {BIOBUZZ_ALLIANCE_D * 0.5f, 0.002f, BIOBUZZ_ALLIANCE_W * 0.5f}, ident, 0.5f,
                               0.5f, 0.0f, tc, 1) < 0) { return -1; }
        }
    }
    return 0;
}

/* ---- staging ---- */
static int bb_spawn_piece (physics_world *world, biobuzz_state_t *st, int is_nectar, int alliance, vector3 pos) {
    float r = is_nectar ? BIOBUZZ_NECTAR_RADIUS : BIOBUZZ_POLLEN_RADIUS;
    float m = is_nectar ? BIOBUZZ_NECTAR_MASS : BIOBUZZ_POLLEN_MASS;
    int bi = physics_world_add_sphere (world, r, m, pos);
    if (bi < 0) { return -1; }
    rigidbody *b = &world -> bodies [bi];
    b -> restitution = INTAKE_BALL_RESTITUTION;
    b -> friction_static = INTAKE_BALL_FRICTION_S;
    b -> friction_kinetic = INTAKE_BALL_FRICTION_K;
    if (!is_nectar) { b -> colour = (vector3) {1.0f, 0.9f, 0.2f}; } else if (alliance == BIOBUZZ_RED) {
        b -> colour = (vector3) {0.95f, 0.25f, 0.25f};
    } else {
        b -> colour = (vector3) {0.25f, 0.40f, 1.00f};
    }
    rigidbody_wake (b);
    if (st && st -> piece_count < BIOBUZZ_MAX_PIECES) {
        int k = st -> piece_count++;
        st -> piece_ids [k] = (int) b -> object_id;
        st -> piece_is_nectar [k] = is_nectar;
        st -> piece_alliance [k] = is_nectar ? alliance : -1;
    }
    return 0;
}

int biobuzz_stage_match (physics_world *world, biobuzz_state_t *st) {
    if (!world || !st || !st -> field_built) { return -1; }
    int made = 0;
    /* Flowers: 4 pollen each, resting in the lower-ring seat (manual: pollen
     * flows through the middle ring to the floor seat). 2x2 at seat height. */
    {
        const float fx [4] = {-0.9144f, 0.9144f, -0.9144f, 0.9144f};
        const float fz [4] = {-BIOBUZZ_FIELD_HALF + 0.10f, -BIOBUZZ_FIELD_HALF + 0.10f,
                              BIOBUZZ_FIELD_HALF - 0.10f, BIOBUZZ_FIELD_HALF - 0.10f};
        for (int f = 0; f < 4; f++) {
            for (int k = 0; k < BIOBUZZ_FLOWER_POLLEN; k++) {
                float ox = ((k % 2 == 0) ? -1.0f : 1.0f) * BIOBUZZ_POLLEN_RADIUS;
                float oz = ((k < 2) ? -1.0f : 1.0f) * BIOBUZZ_POLLEN_RADIUS;
                vector3 p = {fx [f] + ox, BIOBUZZ_POLLEN_RADIUS + 0.012f, fz [f] + oz};
                if (bb_spawn_piece (world, st, 0, -1, p) == 0) { made++; }
            }
        }
    }
    /* Gardens: 4 pollen each in a line along the wall, biased to the
     * alliance-area corner, contacting the tape (Manual §10.3.1). */
    {
        float H = BIOBUZZ_FIELD_HALF;
        for (int a = 0; a < 2; a++) {
            float gx = (a == 0) ? -(H - 0.06f) : (H - 0.06f);
            float z0 = (a == 0) ? -(H - 0.05f) : (H - 0.05f);
            float dir = (a == 0) ? 1.0f : -1.0f;
            for (int k = 0; k < BIOBUZZ_GARDEN_POLLEN; k++) {
                vector3 p = {gx, BIOBUZZ_POLLEN_RADIUS + 0.001f, z0 + dir * (float) k * 0.075f};
                if (bb_spawn_piece (world, st, 0, -1, p) == 0) { made++; }
            }
        }
    }
    /* Up-cells: 3 nectar each, against the back wall biased to the alliance
     * side (Manual §10.3.1). Cell-local: back = -Z, alliance side = -X for
     * red (hive at -X), +X for blue. */
    for (int h = 0; h < 2; h++) {
        int up = st -> tilt [h];
        vector3 cc;
        vector4 cq;
        bb_up_cell_frame (h, up, &cc, &cq);
        for (int k = 0; k < BIOBUZZ_CELL_NECTAR; k++) {
            /* local: back row, stepped toward the alliance side */
            float side = (h == 0) ? -1.0f : 1.0f;
            vector3 loc = {side * (0.10f - 0.0f * (float) k) - side * (float) k * 0.095f, -0.10f,
                           -BIOBUZZ_CELL_D * 0.5f + BIOBUZZ_NECTAR_RADIUS + 0.005f};
            vector3 off = bb_rotate (cq, loc);
            vector3 p = {cc.x + off.x, cc.y + off.y, cc.z + off.z};
            if (bb_spawn_piece (world, st, 1, h, p) == 0) { made++; }
        }
    }
    /* Alliance trays: 5 nectar each outside the side walls (tray zone). */
    {
        float H = BIOBUZZ_FIELD_HALF;
        for (int a = 0; a < 2; a++) {
            float tx = (a == 0) ? -(H + 0.30f) : (H + 0.30f);
            for (int k = 0; k < BIOBUZZ_TRAY_NECTAR; k++) {
                vector3 p = {tx, BIOBUZZ_NECTAR_RADIUS + 0.001f, -0.30f + (float) k * 0.15f};
                if (bb_spawn_piece (world, st, 1, a, p) == 0) { made++; }
            }
        }
    }
    return made;
}

int biobuzz_preload (physics_world *world, biobuzz_state_t *st, const int *robot_body, int n_robots) {
    if (!world || !st) { return -1; }
    int made = 0;
    /* Present robots: 4 pollen each in a row contacting the chassis side. */
    for (int r = 0; r < n_robots; r++) {
        int bi = robot_body [r];
        if (bi < 0 || bi >= world -> body_count) { continue; }
        vector3 c = world -> bodies [bi].position;
        for (int k = 0; k < BIOBUZZ_PRELOAD_POLLEN; k++) {
            vector3 p = {c.x - 0.30f + (float) k * 0.0f, BIOBUZZ_POLLEN_RADIUS + 0.001f,
                         c.z - 0.27f + (float) k * 0.18f};
            if (bb_spawn_piece (world, st, 0, -1, p) == 0) { made++; }
        }
    }
    /* Absent robots (a 2v2 match stages 4 preload sets): remainder goes to
     * the loading-zone centre vs the wall (Manual §10.3.1). */
    int want_sets = 4;
    for (int s = n_robots; s < want_sets; s++) {
        int a = (s < 2) ? 0 : 1; /* first two sets red, next two blue */
        float H = BIOBUZZ_FIELD_HALF;
        float lzx = (a == 0) ? -(H - BIOBUZZ_LOAD_D * 0.5f) : (H - BIOBUZZ_LOAD_D * 0.5f);
        float lzz = (a == 0) ? 0.9144f : -0.9144f;
        for (int k = 0; k < BIOBUZZ_PRELOAD_POLLEN; k++) {
            vector3 p = {lzx, BIOBUZZ_POLLEN_RADIUS + 0.001f, lzz - 0.15f + (float) k * 0.10f};
            if (bb_spawn_piece (world, st, 0, -1, p) == 0) { made++; }
        }
    }
    return made;
}

/* Enter one tray nectar for `alliance` (tip unlock): teleport the first
 * tray-staged nectar (outside the walls) to the loading-zone edge, live and
 * dynamic. Returns 0 ok, -1 none left. Human-entered in real matches; the
 * terminal verb drives this in-sim. */
int biobuzz_enter_nectar (physics_world *world, biobuzz_state_t *st, int alliance) {
    if (!world || !st || (alliance != 0 && alliance != 1)) { return -1; }
    if (st -> unlocked [alliance] <= st -> entered [alliance]) { return -1; }
    float H = BIOBUZZ_FIELD_HALF;
    for (int i = 0; i < st -> piece_count; i++) {
        if (!st -> piece_is_nectar [i] || st -> piece_alliance [i] != alliance) { continue; }
        int bi = physics_world_index_by_id (world, (uint32_t) st -> piece_ids [i]);
        if (bi < 0 || bi >= world -> body_count) { continue; }
        rigidbody *b = &world -> bodies [bi];
        if (b -> kinematic || b -> static_state) { continue; }
        /* Tray nectar lives outside the side walls; field nectar inside. */
        float ax = (b -> position.x > 0.0f) ? b -> position.x : -b -> position.x;
        if (ax < H + 0.05f) { continue; }
        float lzx = (alliance == 0) ? -(H - BIOBUZZ_LOAD_D * 0.5f) : (H - BIOBUZZ_LOAD_D * 0.5f);
        float lzz = (alliance == 0) ? 0.9144f : -0.9144f;
        b -> position = (vector3) {lzx, BIOBUZZ_NECTAR_RADIUS + 0.05f, lzz};
        b -> velocity = vector3_zero ();
        b -> angular_velocity = vector3_zero ();
        rigidbody_wake (b);
        rigidbody_update_axes (b);
        st -> entered [alliance]++;
        return 0;
    }
    return -1;
}

void biobuzz_tick (physics_world *world, biobuzz_state_t *st, const int *piece_ids, int piece_n,
                   const int *piece_is_nectar, float dt) {
    (void) dt;
    if (!world || !st || !st -> field_built) { return; }
    for (int h = 0; h < 2; h++) {
        int up = st -> tilt [h];
        float wsum = bb_up_cell_weight (world, h, up, piece_ids, piece_n, piece_is_nectar);
        if (wsum < BIOBUZZ_TIP_WEIGHT) { continue; }
        /* TIP: re-pose the hive mirrored (damper holds it — no bounce-back
         * model needed), spill the tipping cell with a deterministic nudge,
         * award + unlock. Hysteresis is structural: the new up-cell starts
         * near-empty, so immediate re-tip needs a fresh full load. */
        int ids [11];
        for (int k = 0; k < 11; k++) {
            ids [k] = physics_world_index_by_id (world, (uint32_t) st -> hive_bodies [h][k]);
        }
        /* Spill first (walls teleport away under the load): deterministic
         * outboard + down nudge so the pour reads as a pour, not a pop. */
        {
            vector3 cc;
            vector4 cq;
            bb_up_cell_frame (h, up, &cc, &cq);
            vector3 half = {BIOBUZZ_CELL_W * 0.5f, BIOBUZZ_CELL_H * 0.5f, BIOBUZZ_CELL_D * 0.5f};
            float outboard = (float) up; /* up-cell end == outboard side */
            for (int i = 0; i < piece_n; i++) {
                int bi = physics_world_index_by_id (world, (uint32_t) piece_ids [i]);
                if (bi < 0 || bi >= world -> body_count) { continue; }
                rigidbody *b = &world -> bodies [bi];
                if (b -> kinematic || b -> static_state) { continue; }
                vector3 rel = {b -> position.x - cc.x, b -> position.y - cc.y, b -> position.z - cc.z};
                vector3 loc = bb_unrotate (cq, rel);
                if (!bb_ball_interior (loc, (vector3) {0.0f, 0.0f, 0.0f}, half)) { continue; }
                b -> velocity = (vector3) {b -> velocity.x, -0.5f, b -> velocity.z + outboard * 1.2f};
                rigidbody_wake (b);
            }
        }
        st -> tilt [h] = -up;
        /* Re-pose by body id (indices shift as bodies spawn; ids stable). */
        {
            /* rebuild index list from ids for bb_pose_hive */
            int idx [11];
            for (int k = 0; k < 11; k++) { idx [k] = ids [k]; }
            /* bb_pose_hive wants indices; feed current ones. */
            vector3 ca, cb;
            vector4 qa, qb;
            bb_cell_pose (h, -1, st -> tilt [h], &ca, &qa);
            bb_cell_pose (h, 1, st -> tilt [h], &cb, &qb);
            vector3 span = vector3_subtraction (cb, ca);
            float sl = sqrtf (span.x * span.x + span.y * span.y + span.z * span.z);
            if (sl < 1e-6f) { sl = 1e-6f; }
            float ang = atan2f (-(span.y / sl), span.z / sl);
            vector3 mid = {(ca.x + cb.x) * 0.5f, (ca.y + cb.y) * 0.5f, (ca.z + cb.z) * 0.5f};
            if (idx [0] >= 0 && idx [0] < world -> body_count) {
                world -> bodies [idx [0]].position = mid;
                world -> bodies [idx [0]].orientation = bb_quat_x (ang);
                rigidbody_update_axes (&world -> bodies [idx [0]]);
            }
            const vector3 centres [2] = {ca, cb};
            const vector4 quats [2] = {qa, qb};
            for (int c = 0; c < 2; c++) {
                vector3 lp [5], lh [5];
                bb_cell_walls (lp, lh);
                for (int k = 0; k < 5; k++) {
                    int bi = idx [1 + c * 5 + k];
                    if (bi < 0 || bi >= world -> body_count) { continue; }
                    vector3 off = bb_rotate (quats [c], lp [k]);
                    world -> bodies [bi].position =
                        (vector3) {centres [c].x + off.x, centres [c].y + off.y, centres [c].z + off.z};
                    world -> bodies [bi].orientation = quats [c];
                    rigidbody_update_axes (&world -> bodies [bi]);
                    rigidbody_wake (&world -> bodies [bi]);
                }
            }
            /* Refresh the up-cell census anchor. */
            vector3 ucc;
            vector4 ucq;
            bb_up_cell_frame (h, st -> tilt [h], &ucc, &ucq);
            (void) ucq;
            st -> up_mouth [h][0] = ucc.x;
            st -> up_mouth [h][1] = ucc.y;
            st -> up_mouth [h][2] = ucc.z;
        }
        st -> tips [h]++;
        st -> unlocked [h]++;
        fprintf (stderr, "[biobuzz] HIVE TIP %s (tips=%d, unlocked nectar=%d)\n", h == 0 ? "red" : "blue",
                 st -> tips [h], st -> unlocked [h]);
    }
}

/* ---- scoring ---- */
/* Flower anchor positions (must match build). */
static void bb_flower_anchor (int f, float *cx, float *cz) {
    const float fx [4] = {-0.9144f, 0.9144f, -0.9144f, 0.9144f};
    const float fz [4] = {-BIOBUZZ_FIELD_HALF + 0.10f, -BIOBUZZ_FIELD_HALF + 0.10f,
                          BIOBUZZ_FIELD_HALF - 0.10f, BIOBUZZ_FIELD_HALF - 0.10f};
    *cx = fx [f & 3];
    *cz = fz [f & 3];
}
/* Garden rect centre/half for alliance a. */
static void bb_garden_rect (int a, vector3 *c, vector3 *half) {
    float H = BIOBUZZ_FIELD_HALF;
    c -> x = (a == 0) ? -(H - BIOBUZZ_GARDEN_D * 0.5f) : (H - BIOBUZZ_GARDEN_D * 0.5f);
    c -> y = 0.05f;
    c -> z = (a == 0) ? -(H - BIOBUZZ_GARDEN_W * 0.5f) : (H - BIOBUZZ_GARDEN_W * 0.5f);
    half -> x = BIOBUZZ_GARDEN_D * 0.5f;
    half -> y = 0.10f;
    half -> z = BIOBUZZ_GARDEN_W * 0.5f;
}
static void bb_loading_rect (int a, vector3 *c, vector3 *half) {
    float H = BIOBUZZ_FIELD_HALF;
    c -> x = (a == 0) ? -(H - BIOBUZZ_LOAD_D * 0.5f) : (H - BIOBUZZ_LOAD_D * 0.5f);
    c -> y = 0.10f;
    c -> z = (a == 0) ? 0.9144f : -0.9144f;
    half -> x = BIOBUZZ_LOAD_D * 0.5f;
    half -> y = 0.20f;
    half -> z = BIOBUZZ_LOAD_W * 0.5f;
}

int biobuzz_score (physics_world *world, biobuzz_state_t *st, const int *piece_ids, int piece_n,
                   const int *piece_is_nectar, const int *piece_alliance, biobuzz_alliance alliance,
                   const int *robot_body, int n_robots, biobuzz_alliance_score *out) {
    if (!world || !out) { return -1; }
    memset (out, 0, sizeof (*out));
    int A = (int) alliance;
    /* Tips. */
    if (st) {
        out -> tips = st -> tips [A];
        out -> tip_points = out -> tips * BIOBUZZ_PTS_TIP;
    }
    /* Cells: pieces resting in the CURRENT up-cell interior (assessed at
     * rest; caller steps to settle first). */
    if (st && st -> field_built) {
        vector3 cc;
        vector4 cq;
        bb_up_cell_frame (A, st -> tilt [A], &cc, &cq);
        vector3 half = {BIOBUZZ_CELL_W * 0.5f - 0.005f, BIOBUZZ_CELL_H * 0.5f - 0.005f,
                        BIOBUZZ_CELL_D * 0.5f - 0.005f};
        for (int i = 0; i < piece_n; i++) {
            int bi = physics_world_index_by_id (world, (uint32_t) piece_ids [i]);
            if (bi < 0 || bi >= world -> body_count) { continue; }
            rigidbody *b = &world -> bodies [bi];
            if (b -> kinematic || b -> static_state) { continue; }
            vector3 rel = {b -> position.x - cc.x, b -> position.y - cc.y, b -> position.z - cc.z};
            if (!bb_ball_interior (bb_unrotate (cq, rel), (vector3) {0.0f, 0.0f, 0.0f}, half)) { continue; }
            out -> cell_pieces++;
        }
        out -> cell_points = out -> cell_pieces * BIOBUZZ_PTS_CELL;
    }
    /* Flowers: per-flower bottom-first census. Owner = alliance of the
     * top-most NECTAR; owner scores every piece inside x2; each alliance
     * with a nectar inside takes +5 for its bottom-most nectar. */
    {
        vector3 fc = {0.0f, BIOBUZZ_FLOWER_TOP_H - BIOBUZZ_FLOWER_RING_GAP * 0.5f, 0.0f};
        vector3 fh = {0.06f, BIOBUZZ_FLOWER_RING_GAP * 0.5f, 0.06f};
        for (int f = 0; f < BIOBUZZ_MAX_FLOWERS; f++) {
            float cx, cz;
            bb_flower_anchor (f, &cx, &cz);
            fc.x = cx;
            fc.z = cz;
            /* collect inside pieces bottom-first (insertion by y; n small) */
            int ids [24], isn [24], isa [24];
            float ys [24];
            int n = 0;
            for (int i = 0; i < piece_n && n < 24; i++) {
                int bi = physics_world_index_by_id (world, (uint32_t) piece_ids [i]);
                if (bi < 0 || bi >= world -> body_count) { continue; }
                rigidbody *b = &world -> bodies [bi];
                if (b -> kinematic || b -> static_state) { continue; }
                float r = b -> radius;
                if (r <= 0.0f) { r = 0.04f; }
                if (!bb_ball_partial (b -> position, r, fc, fh)) { continue; }
                int k = n;
                while (k > 0 && ys [k - 1] > b -> position.y) {
                    ids [k] = ids [k - 1];
                    isn [k] = isn [k - 1];
                    isa [k] = isa [k - 1];
                    ys [k] = ys [k - 1];
                    k--;
                }
                ids [k] = piece_ids [i];
                isn [k] = piece_is_nectar [i];
                isa [k] = piece_alliance ? piece_alliance [i] : -1;
                ys [k] = b -> position.y;
                n++;
            }
            if (n == 0) { continue; }
            /* top-most nectar => owner */
            int owner = -1;
            for (int k = n - 1; k >= 0; k--) {
                if (isn [k]) { owner = isa [k]; break; }
            }
            if (owner == A) {
                out -> flower_owned_pieces += n;
                out -> flower_owned_points += n * BIOBUZZ_PTS_FLOWER_OWNED;
            }
            /* bottom-most nectar per colour => +5 for that alliance */
            for (int k = 0; k < n; k++) {
                if (isn [k] && isa [k] == A) {
                    out -> flower_bottom_bonus += BIOBUZZ_PTS_FLOWER_BOTTOM;
                    break;
                }
            }
            if (st) {
                biobuzz_flower_state *fs = &st -> flowers [f];
                fs -> present = 1;
                fs -> piece_count = (n > 16) ? 16 : n;
                fs -> nectar_red = 0;
                fs -> nectar_blue = 0;
                fs -> pollen = 0;
                fs -> owner = owner;
                fs -> top_y = -1.0f;
                fs -> bottom_red_y = -1.0f;
                fs -> bottom_blue_y = -1.0f;
                for (int k = 0; k < fs -> piece_count; k++) {
                    fs -> piece_ids [k] = ids [k];
                    if (!isn [k]) { fs -> pollen++; continue; }
                    if (isa [k] == 0) {
                        fs -> nectar_red++;
                        if (fs -> bottom_red_y < 0.0f) { fs -> bottom_red_y = ys [k]; }
                    } else {
                        fs -> nectar_blue++;
                        if (fs -> bottom_blue_y < 0.0f) { fs -> bottom_blue_y = ys [k]; }
                    }
                    fs -> top_y = ys [k];
                }
            }
        }
    }
    /* Gardens: any piece at least partially in the zone scores 1 for the
     * zone alliance, either colour (Manual §10.5.3). y-gated to resting. */
    {
        vector3 gc, gh;
        bb_garden_rect (A, &gc, &gh);
        for (int i = 0; i < piece_n; i++) {
            int bi = physics_world_index_by_id (world, (uint32_t) piece_ids [i]);
            if (bi < 0 || bi >= world -> body_count) { continue; }
            rigidbody *b = &world -> bodies [bi];
            if (b -> kinematic || b -> static_state) { continue; }
            if (b -> position.y > 0.12f) { continue; }
            float r = b -> radius;
            if (r <= 0.0f) { r = 0.04f; }
            if (!bb_ball_partial (b -> position, r, gc, gh)) { continue; }
            out -> garden_pieces++;
        }
        out -> garden_points = out -> garden_pieces * BIOBUZZ_PTS_GARDEN;
    }
    /* Leave/park predicates (caller labels AUTO vs TELEOP assessment). */
    {
        float H = BIOBUZZ_FIELD_HALF;
        for (int r = 0; r < n_robots; r++) {
            int bi = robot_body [r];
            if (bi < 0 || bi >= world -> body_count) { continue; }
            rigidbody *ch = &world -> bodies [bi];
            float hw = 0.22f; /* chassis + wheel outer envelope (0.20 + 0.02 half-width) */
            /* contacting a wall = surface within 5 mm of an inner face */
            float dx = H - (ch -> position.x < 0 ? -(ch -> position.x) : ch -> position.x);
            float dz = H - (ch -> position.z < 0 ? -(ch -> position.z) : ch -> position.z);
            float gap = (dx < dz) ? dx - hw : dz - hw;
            if (gap > 0.005f) { out -> leave++; }
            /* park = chassis box overlaps the alliance loading zone */
            for (int a = 0; a < 2; a++) {
                vector3 zc, zh;
                bb_loading_rect (a, &zc, &zh);
                float ox = (zh.x + hw) - fabsf (ch -> position.x - zc.x);
                float oz = (zh.z + hw) - fabsf (ch -> position.z - zc.z);
                if (ox > 0.0f && oz > 0.0f && ch -> position.y < 0.5f) {
                    if (a == A) { out -> park++; }
                    break;
                }
            }
        }
    }
    out -> total = out -> tip_points + out -> cell_points + out -> flower_owned_points + out -> flower_bottom_bonus +
                   out -> garden_points;
    return 0;
}

int biobuzz_rp_swarm (int leave_park_points) {return (leave_park_points >= BIOBUZZ_RP_SWARM_PTS) ? 1 : 0;}
int biobuzz_rp_pollinator1 (int tips_total) {return (tips_total >= BIOBUZZ_RP_P1_TIPS) ? 1 : 0;}
int biobuzz_rp_pollinator2 (int tips_total) {return (tips_total >= BIOBUZZ_RP_P2_TIPS) ? 1 : 0;}

void biobuzz_start_pose (biobuzz_alliance alliance, int slot, float out_xyz [3], float *out_yaw) {
    /* Touching the own side wall (wheel outer 0.29 + 2 mm), clear of the
     * loading zone (|z|<=0.64 vs zone edge 0.774) and gardens, facing the
     * centre (+X for red, -X for blue). Robot forward = chassis +Z, so yaw
     * ±90 deg. 18 in footprint verified vs CHASSIS 0.45 m. */
    float H = BIOBUZZ_FIELD_HALF;
    float x = (alliance == BIOBUZZ_RED) ? -(H - 0.222f) : (H - 0.222f); /* outer 0.22 + 2 mm to touch */
    float z = (slot == 0) ? -0.35f : 0.35f;
    out_xyz [0] = x;
    out_xyz [1] = 0.30f; /* chassis rest height class (robot sets exact) */
    out_xyz [2] = z;
    if (out_yaw) { *out_yaw = (alliance == BIOBUZZ_RED) ? 1.57079633f : -1.57079633f; }
}

int biobuzz_hive_ids (const biobuzz_state_t *st, int hive, int out_ids [11]) {
    if (!st || !out_ids || (hive != 0 && hive != 1)) { return -1; }
    for (int k = 0; k < 11; k++) { out_ids [k] = st -> hive_bodies [hive][k]; }
    return 0;
}
int biobuzz_up_cell_frame (int hive, int up_end, vector3 *out_c, vector4 *out_q) {
    if (!out_c || !out_q || (hive != 0 && hive != 1) || (up_end != -1 && up_end != 1)) { return -1; }
    bb_up_cell_frame (hive, up_end, out_c, out_q);
    return 0;
}
vector3 biobuzz_unrotate (vector4 q, vector3 v) { return bb_unrotate (q, v); }

/* Rigidly yaw a spawned robot about its chassis centre (spawn API takes no
 * yaw; start poses face the centre per regulation). Rotates chassis, wheels
 * and rollers; sets odometry to the start pose. */
void biobuzz_place_robot (struct physics_world *world, ftc_robot *robot, float yaw) {
    if (!world || !robot || robot -> chassis_body < 0 || robot -> chassis_body >= world -> body_count) { return; }
    rigidbody *ch = &world -> bodies [robot -> chassis_body];
    vector3 c = ch -> position;
    vector4 q = bb_quat_yaw (yaw);
    /* rotate every robot body about c (chassis, wheels, rollers) */
    float cs = cosf (yaw), sn = sinf (yaw);
    int total_slots = 1 + robot -> wheel_count + robot -> wheel_count * 8;
    for (int k = -1; k < total_slots - 1; k++) {
        int bi = -1;
        if (k < 0) {
            bi = robot -> chassis_body;
        } else if (k < robot -> wheel_count) {
            bi = robot -> wheel_bodies [k];
        } else {
            int wi = (k - robot -> wheel_count) / 8;
            int ri = (k - robot -> wheel_count) % 8;
            if (wi < 0 || wi >= robot -> wheel_count) { continue; }
            if (wi >= FTC_MAX_WHEELS) { continue; }
            if (ri < 0 || ri >= robot -> roller_count [wi]) { continue; }
            bi = robot -> roller_bodies [wi][ri];
        }
        if (bi < 0 || bi >= world -> body_count) { continue; }
        rigidbody *b = &world -> bodies [bi];
        vector3 rel = {b -> position.x - c.x, b -> position.y - c.y, b -> position.z - c.z};
        /* yaw about Y: (x,z) -> (x cos + z sin, -x sin + z cos) for the
         * engine's +Y convention (+90 deg maps +Z to +X; asserted in-gate
         * via the intake mouth direction). */
        vector3 rr = {rel.x * cs + rel.z * sn, rel.y, -rel.x * sn + rel.z * cs};
        b -> position = (vector3) {c.x + rr.x, c.y + rr.y, c.z + rr.z};
        /* orientation: q * o (vector4_multiplication(a,b) applies b first) */
        b -> orientation = vector4_multiplication (q, b -> orientation);
        rigidbody_update_axes (b);
        rigidbody_wake (b);
    }
    robot -> odom_x = c.x;
    robot -> odom_z = c.z;
    robot -> odom_theta = yaw;
}

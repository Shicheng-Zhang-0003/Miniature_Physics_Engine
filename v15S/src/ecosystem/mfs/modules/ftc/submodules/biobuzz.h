/* BIOBUZZ regulation field + league (FTC 2026-2027, Competition Manual V1).
 *
 * WHAT THIS IS: the full physical BIOBUZZ field in SI metres, built from
 * engine bodies (statics for structures, spheres for pieces), plus the
 * league state machine (tip events, nectar unlocks, flower/garden/cell
 * census, Table 10-2 scoring, ranking points). All dimensions are Manual
 * V1 §9/§10 + Event/Initial Field Setup Guides; every number cites its
 * section below. Nominal dimensions carry the manual's ±1 in (±2.5 cm)
 * tolerance — the sim builds nominal.
 *
 * WHAT THIS IS NOT:
 * - The arcade league (intake.h balls/bucket) is untouched; this is additive.
 * - Hive tips are EVENT-driven (weight census vs the manual's ~8-pollen /
 *   ~5-nectar thresholds, re-posed kinematically with a deterministic spill),
 *   not a bi-stable damper solve. A revolute-joint dynamic hive is ticketed
 *   follow-up; the event form is exact on thresholds and deterministic.
 * - No match clock, no fouls (human refs), no AprilTag vision (tag IDs are
 *   recorded per cell for autonomy use; rendering has no decal system, so
 *   cells carry a dark underside marker box instead — disclosed).
 * - Orientation convention (manual figures are not machine-readable, so this
 *   is nominal and DOCUMENTED, not claimed): origin at field centre, y=0 at
 *   tile top; -Z = audience side, +Z = scoring/far side; -X = RED side wall,
 *   +X = BLUE side wall. Red garden corner A1 = (-X,-Z); blue F6 = (+X,+Z);
 *   red loading A5 (-X wall, row 5); blue loading F2 (+X wall, row 2).
 *   Flowers: 2 on the audience wall + 2 on the far wall at x=±36 in
 *   (nominal — manual places 4 "around the perimeter", down-cells point at
 *   the audience/far flowers per the reset guide).
 */
#ifndef mfs_biobuzz_h
#define mfs_biobuzz_h
#include "core/math3d.h"
#include "core/rigidbody.h"
#include "core/physics_world.h"
#include "robot.h"  /* ftc_robot (placement helper) */
#include "intake.h" /* BIOBUZZ_POLLEN/NECTAR_RADIUS+MASS (Manual §9.8) */

/* ---- §9.2 FIELD: 144x144 in inside the perimeter, 6x6 tiles ---- */
#define BIOBUZZ_FIELD_HALF 1.8288f   /* 72 in, inside faces (controlled dim) */
#define BIOBUZZ_TILE 0.6096f         /* 24 in */
#define BIOBUZZ_TILE_THICK 0.0150f   /* 0.59 in nominal */
#define BIOBUZZ_WALL_H 0.3048f       /* 12 in perimeter walls */
#define BIOBUZZ_WALL_T 0.0300f       /* panel half-thickness, nominal: perimeter
                                      * versions vary (Manual §9.2); INSIDE
                                      * faces are the controlled dimension and
                                      * are exact here */
/* ---- §9.6 HIVE structure: frame 49.46 W x 38.95 D, pivots 43.95 up ---- */
#define BIOBUZZ_FRAME_W 1.2565f      /* 49.46 in */
#define BIOBUZZ_FRAME_D 0.9896f      /* 38.95 in at base (widest) */
#define BIOBUZZ_PIVOT_H 1.1165f      /* 43.95 in above tiles */
#define BIOBUZZ_CELL_GAP 0.4775f     /* 18.8 in between the 2 cells of a hive */
#define BIOBUZZ_CELL_W 0.5080f       /* opening 20 in wide */
#define BIOBUZZ_CELL_H 0.3556f       /* opening 14 in tall */
#define BIOBUZZ_CELL_D 0.3048f       /* 12 in deep */
#define BIOBUZZ_CELL_SKIN 0.0120f    /* transparent-sheet + rib approximation */
#define BIOBUZZ_HIVE_DX 0.3239f      /* 25.5 in / 2 hive centre offset (Manual
                                      * Fig 10-3 derived; disclosed) */
#define BIOBUZZ_CELL_UP_DY 0.30f      /* up-cell centre height above pivot */
#define BIOBUZZ_CELL_END_DZ 0.2388f   /* cell centre outboard offset (18.8/2) */
#define BIOBUZZ_TIP_TILT 0.5236f     /* 30 deg cell repose (derived nominal;
                                      * opening normal 30 deg off axis) */
/* ---- §9.7 FLOWER: top dia 4 in @ 21.5 in, backstop 1.25 in ---- */
#define BIOBUZZ_FLOWER_TOP_D 0.1016f /* 4 in opening */
#define BIOBUZZ_FLOWER_TOP_H 0.5461f /* 21.5 in above tiles */
#define BIOBUZZ_FLOWER_BACKSTOP_H 0.0318f /* 1.25 in guide */
#define BIOBUZZ_FLOWER_MID_OPEN 0.0810f   /* nominal: passes pollen (0.071),
                                           * blocks nectar (0.091) — Manual:
                                           * "pollen flows through, nectar
                                           * cannot fit through middle ring" */
#define BIOBUZZ_FLOWER_RETR_H 0.0902f     /* 3.55 in retrieval opening */
#define BIOBUZZ_FLOWER_RETR_D 0.0907f     /* 3.57 in deep */
#define BIOBUZZ_FLOWER_RING_GAP 0.4445f   /* top-middle rings 17.5 in o-o */
#define BIOBUZZ_FLOWER_LOW_HOLE_D 0.0709f /* 2.79 in lower-ring seat */
#define BIOBUZZ_FLOWER_PIPE_R 0.0120f     /* HIPS-pipe nominal */
/* ---- §9.3/§9.5 zones (tape proxies: thin no-collide slabs, §9-accurate) */
#define BIOBUZZ_ALLIANCE_W 2.4638f   /* 97 in wide */
#define BIOBUZZ_ALLIANCE_D 1.3716f   /* 54 in deep, outside field */
#define BIOBUZZ_LOAD_W 0.5842f       /* 23 in */
#define BIOBUZZ_LOAD_D 0.2794f       /* 11 in */
#define BIOBUZZ_GARDEN_W 0.5842f     /* 23 in */
#define BIOBUZZ_GARDEN_D 0.0508f     /* 2 in */
/* ---- §9.9 AprilTags: 3.25 in 36h11, 4-cluster IDs per cell underside ---- */
#define BIOBUZZ_TAG_RED_FAR_0 30
#define BIOBUZZ_TAG_RED_AUD_0 34
#define BIOBUZZ_TAG_BLUE_AUD_0 38
#define BIOBUZZ_TAG_BLUE_FAR_0 42
/* ---- §10.3 staging counts ---- */
#define BIOBUZZ_POLLEN_TOTAL 40
#define BIOBUZZ_NECTAR_PER_ALLIANCE 8
#define BIOBUZZ_FLOWER_POLLEN 4
#define BIOBUZZ_GARDEN_POLLEN 4
#define BIOBUZZ_PRELOAD_POLLEN 4
#define BIOBUZZ_CELL_NECTAR 3
#define BIOBUZZ_TRAY_NECTAR 5
/* ---- tip thresholds (field walkthrough: ~8 pollen / 5 nectar / 3+3) ----
 * Weight units: pollen 1.0, nectar 1.7 (nominal — Manual says "approximately"
 * and placement-dependent; the sim counts weight inside the up-cell volume,
 * position-independent, disclosed). 8x1.0=8.0, 5x1.7=8.5, 3+3x1.7=8.1. */
#define BIOBUZZ_POLLEN_WEIGHT 1.0f
#define BIOBUZZ_NECTAR_WEIGHT 1.7f
#define BIOBUZZ_TIP_WEIGHT 8.0f
/* ---- Table 10-2 scoring (MATCH points) ---- */
#define BIOBUZZ_PTS_LEAVE 3
#define BIOBUZZ_PTS_PARK 5
#define BIOBUZZ_PTS_TIP 20
#define BIOBUZZ_PTS_CELL 2
#define BIOBUZZ_PTS_FLOWER_BOTTOM 5
#define BIOBUZZ_PTS_FLOWER_OWNED 2
#define BIOBUZZ_PTS_GARDEN 1
/* ---- ranking-point thresholds (§10.5: SWARM leave+park, P1/P2 tips) ---- */
#define BIOBUZZ_RP_SWARM_PTS 16
#define BIOBUZZ_RP_P1_TIPS 4
#define BIOBUZZ_RP_P2_TIPS 7
/* ---- registry ---- */
#define BIOBUZZ_MAX_PIECES 64        /* 56 staged + headroom for entered */
#define BIOBUZZ_MAX_FLOWERS 4
#define BIOBUZZ_HIVE_RED 0
#define BIOBUZZ_HIVE_BLUE 1

typedef enum { BIOBUZZ_RED = 0, BIOBUZZ_BLUE = 1 } biobuzz_alliance;

/* Per-flower census (scoring volume = between top and middle rings). */
typedef struct {
    int present;                 /* field built */
    int piece_ids [16];          /* ball body ids inside, bottom-first */
    int piece_count;
    int nectar_red;              /* nectar count by colour inside */
    int nectar_blue;
    int pollen;
    int owner;                   /* -1 none, else alliance of top-most nectar */
    float top_y;                 /* top-most nectar height (ownership) */
    float bottom_red_y;          /* bottom-most red nectar height */
    float bottom_blue_y;
} biobuzz_flower_state;

/* League state (per world, owned by the ftc-fleet tick state). */
typedef struct {
    int field_built;
    int tilt [2];                /* +1 = cellA(-Z end) up, -1 = cellB(+Z) up */
    int tips [2];
    int unlocked [2];            /* nectar releases earned (tips) */
    int entered [2];             /* nectar placed from trays */
    int piece_ids [BIOBUZZ_MAX_PIECES];
    int piece_is_nectar [BIOBUZZ_MAX_PIECES];
    int piece_alliance [BIOBUZZ_MAX_PIECES]; /* nectar colour; pollen -1 */
    int piece_count;
    int flower_ids [BIOBUZZ_MAX_FLOWERS];    /* anchor body id per flower */
    biobuzz_flower_state flowers [BIOBUZZ_MAX_FLOWERS];
    /* hive group body ids for kinematic re-pose: [hive][0]=beam,
     * [hive][1..5]=cellA boxes, [hive][6..10]=cellB boxes */
    int hive_bodies [2][11];
    float up_mouth [2][3];       /* up-cell mouth centre (census volume) */
    float up_half [2][3];        /* up-cell census half extents */
} biobuzz_state_t;

/* Match-score breakdown for one alliance (caller labels the period for
 * leave/park: AUTO end vs match end — the engine runs no match clock). */
typedef struct {
    int tips;
    int tip_points;
    int cell_pieces;
    int cell_points;
    int flower_owned_pieces;
    int flower_owned_points;
    int flower_bottom_bonus;     /* 0/5 per flower summed */
    int garden_pieces;
    int garden_points;
    int leave;                   /* 0/1 robots predicate OUT */
    int park;                    /* 0/1/2 robots predicate OUT */
    int total;
} biobuzz_alliance_score;

/* Build the regulation field (floor slab, 4 walls, hive frame + 2 hives
 * posed per §11.1 setup, 4 flowers, zone tape proxies). Idempotent: skips
 * parts already present (re-run safe). Returns 0 ok, -1 no world/pool. */
int biobuzz_build_field (struct physics_world *world, biobuzz_state_t *st);
/* Stage the §10.3 match setup (16 flower pollen, 8 garden pollen, 6 cell
 * nectar, 10 tray nectar; robot preloads handled by biobuzz_preload).
 * Returns pieces staged, -1 on error. */
int biobuzz_stage_match (struct physics_world *world, biobuzz_state_t *st);
/* Preload 4 pollen per robot index in `robot_idx` (count n): in/on/against
 * the robot per §10.3.4. Robots absent → their 4 go to their loading zone
 * centre vs the wall (Manual §10.3.1). Returns pollen placed. */
int biobuzz_preload (struct physics_world *world, biobuzz_state_t *st, const int *robot_body, int n_robots);
/* Per-tick league update: tip census (edge-triggered re-pose + spill +
 * unlock). Inert when no field. piece_ids/piece_n/is_nectar are the
 * caller's tracked registry (fleet biobuzz registry). */
void biobuzz_tick (struct physics_world *world, biobuzz_state_t *st, const int *piece_ids, int piece_n,
                   const int *piece_is_nectar, float dt);
/* Enter one tray nectar for `alliance` (tip unlock spent): teleport the
 * first outside-wall tray nectar to the loading-zone edge, live. Returns
 * 0 ok, -1 none unlock-spendable/left. */
int biobuzz_enter_nectar (struct physics_world *world, biobuzz_state_t *st, int alliance);
/* Census + Table 10-2 scoring for one alliance (flowers/gardens/cells live,
 * tips/unlocks from state; leave/park OUT-predicates for caller context).
 * robot_body = chassis indices (n_robots, may be 0). */
int biobuzz_score (struct physics_world *world, biobuzz_state_t *st, const int *piece_ids, int piece_n,
                   const int *piece_is_nectar, const int *piece_alliance, biobuzz_alliance alliance,
                   const int *robot_body, int n_robots, biobuzz_alliance_score *out);
/* Pure ranking-point predicates (thresholds above; win/tie by caller). */
int biobuzz_rp_swarm (int leave_park_points);
int biobuzz_rp_pollinator1 (int tips_total);
int biobuzz_rp_pollinator2 (int tips_total);
/* Regulation start pose for robot i of alliance (touching own side wall,
 * clear of loading zones/flowers; faces field centre). Returns chassis xyz
 * via out[3]; yaw via out_yaw (0=+X NASCAR convention per drivetrain). */
void biobuzz_start_pose (biobuzz_alliance alliance, int slot, float out_xyz [3], float *out_yaw);
/* Rigidly yaw a spawned robot about its chassis centre to a start pose
 * (spawn takes no yaw; regulation starts face the centre). Rotates
 * chassis/wheels/rollers, sets odometry to the pose. */
void biobuzz_place_robot (struct physics_world *world, ftc_robot *robot, float yaw);
/* Hive body ids for inspection/debug (beam + 10 cell boxes, -1 unset).
 * Returns 0 ok. */
int biobuzz_hive_ids (const biobuzz_state_t *st, int hive, int out_ids [11]);
#endif /* mfs_biobuzz_h */

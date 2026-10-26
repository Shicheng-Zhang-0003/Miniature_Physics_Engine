/* Ball intake + shooter-feed ABSTRACTION (fictional/arcade league).
 *
 * This is the seam the next game step builds on: zone geometry, possession,
 * and launch live here behind plain functions on explicit arguments. No
 * game rules inside (no scoring, no BioBuzz specifics, no robot struct):
 * callers (ftc-fleet live flow, headless gates, future autonomy) own the
 * ball registry and the chassis pose and get capture/feed/aero primitives.
 *
 * FICTIONAL means arcade, stated plainly: capture is a zone predicate (not
 * roller-contact mechanics like module_1's compliant intake), carrying is
 * kinematic hold at the mouth (no joint, no compliance solve), and launch
 * is a prescribed velocity + backspin (not a flywheel contact solve). All
 * three are deterministic (fixed-point geometry + exact IEEE ops; the only
 * libm use is sqrtf/fabsf/acosf-free length checks and sinf/cosf-free axis
 * work — no transcendentals at all) and inert when unused (intake off or
 * no balls: zero writes).
 *
 * Ball spec matches the BioBuzz numbers module_1 plays (see
 * modules/module_1/mfs_module_1.h): 42 mm, 2.6 g, COR 0.65. Same balls,
 * simpler league.
 */
#ifndef mfs_intake_h
#define mfs_intake_h
#include "core/math3d.h"
#include "core/rigidbody.h"
#include "core/physics_world.h"
/* ---- arcade ball spec (BioBuzz-compatible) ---- */
#define INTAKE_BALL_RADIUS 0.021f
#define INTAKE_BALL_MASS 0.0026f
#define INTAKE_BALL_RESTITUTION 0.65f
#define INTAKE_BALL_FRICTION_S 0.45f
#define INTAKE_BALL_FRICTION_K 0.35f
/* ---- BIOBUZZ 2026-27 scoring elements (FTC Competition Manual v1 §9.8) ----
 * POLLEN: ~2.8 in (7.1 cm) diameter polyethylene ball, yellow (am-5851).
 * NECTAR: ~3.6 in (9.1 cm) diameter polyethylene ball, red/blue (am-5852).
 * The league spawns per-piece sphere bodies at these radii; intake carry
 * spacing and ball aero read the RADIUS FROM THE BODY (half_extensions.x)
 * where it matters, so the larger regulation pieces play with their real
 * frontal area/drag without retuning the small-ball league (whose bodies
 * carry the old radius and stay bit-identical). Mass scales as r^3 from the
 * arcade spec (2.6 g at 21 mm): pollen ~12.6 g, nectar ~26.4 g. */
#define BIOBUZZ_POLLEN_RADIUS 0.0355f
#define BIOBUZZ_NECTAR_RADIUS 0.0455f
#define BIOBUZZ_POLLEN_MASS 0.0126f
#define BIOBUZZ_NECTAR_MASS 0.0264f
/* ---- registry bounds ---- */
#define INTAKE_MAX_BALLS 16
#define INTAKE_MAX_CARRIED 4
/* ---- mouth geometry (chassis-relative, +Z forward, metres) ----
 * The mouth is a low box ahead of the nose, reaching down to ground balls
 * (chassis centre rides ~0.18 m up; balls rest at ~0.02 m). A ball whose
 * centre enters it while slow relative to the chassis is captured. */
#define INTAKE_MOUTH_FWD 0.34f /* ahead of chassis centre */
#define INTAKE_MOUTH_UP -0.12f /* low: ground balls, not chassis height */
#define INTAKE_MOUTH_HALF_X 0.16f
#define INTAKE_MOUTH_HALF_Y 0.10f
#define INTAKE_MOUTH_HALF_Z 0.14f
#define INTAKE_CAPTURE_SPEED 1.5f /* capture only slow balls (m/s rel) */
/* ---- fictional launch (tuned: clean arcing lob, see bucket_game gate) ---- */
#ifndef INTAKE_LAUNCH_SPEED
#define INTAKE_LAUNCH_SPEED 9.0f
#endif
#ifndef INTAKE_LAUNCH_ELEV_DEG
#define INTAKE_LAUNCH_ELEV_DEG 50.0f /* above chassis-forward */
#endif
#ifndef INTAKE_LAUNCH_SPIN
#define INTAKE_LAUNCH_SPIN 15.0f
#endif /* backspin about chassis X (Magnus lift) */
#define INTAKE_MUZZLE_UP 0.12f /* launch starts a hand above the mouth */
#define INTAKE_MUZZLE_FWD 0.48f /* ahead of chassis centre: clears the chassis
                                  * conservative CCD bubble (bounding R 0.33;
                                  * the old mouth-based 0.34 sat inside it and
                                  * CCD ate ~2-3 m/s on every first tick) */
/* Mouth centre in world space from a chassis pose. */
void intake_mouth_pose (vector3 chassis_pos, vector4 chassis_ori, vector3 *out_center, vector3 *out_half);
/* Capture step. carried[]/carried_n is the caller's possession (body ids);
 * ball_ids/ball_n is the caller's registry (body ids, -1 holes allowed).
 * Intake on + capacity left: every slow unpossessed dynamic ball whose
 * centre is inside the mouth box becomes kinematic and is appended.
 * Returns newly captured count. Never touches possessed or kinematic
 * (already scored/frozen) balls it does not own. */
int intake_update (physics_world *world, vector3 chassis_pos, vector4 chassis_ori, vector3 chassis_vel, int intake_on,
                   int carried [INTAKE_MAX_CARRIED], int *carried_n, const int *ball_ids, int ball_n);
/* Feed the oldest carried ball to the shooter: dynamic again, placed at the
 * muzzle with launch velocity (chassis velocity inherited) + backspin.
 * Returns the body id, or -1 when empty/gone. */
int intake_shoot (physics_world *world, vector3 chassis_pos, vector4 chassis_ori, vector3 chassis_vel,
                  int carried [INTAKE_MAX_CARRIED], int *carried_n);
/* Pure launch state from a chassis pose — the SINGLE source of truth for the
 * shot, shared by intake_shoot() (applies it to the real ball) and the
 * trajectory predictor (ftc_traj.c, which simulates a copy through it), so
 * the predicted line cannot drift from the shot that actually fires. No side
 * effects, no world access. */
typedef struct {
    vector3 muzzle;   /* world start position */
    vector3 dir;      /* unit launch direction (yaw-projected, world-up elevation) */
    vector3 velocity; /* muzzle velocity = inherited chassis vel + speed*dir */
    vector3 spin;     /* angular velocity (backspin about the lateral axis) */
} intake_launch_state;
void intake_launch_state_compute (vector3 chassis_pos, vector4 chassis_ori, vector3 chassis_vel,
                                  intake_launch_state *out);
/* Power-scaled shot (DESPOT-2026-10-14): the stock 9 m/s 50 deg muzzle
 * outranges the 3.66 m field (8 m range) and cannot lob softly into the
 * 1.5 m hive mouths from legal spots — every full-power hive shot from
 * inside the field smacks frame or cell undersides. Real launchers vary
 * wheel speed; power scales muzzle speed linearly (chassis inherited
 * fully) and backspin proportionally, clamped to [0.3, 1.0] (below 0.3
 * balls dribble out of the mouth). power == 1.0 takes the exact legacy
 * path (branch, bit-identical — the bucket_game calibration depends on
 * it). The trajectory predictor previews the last aimed shot through
 * intake_launch_state_compute_elev (documented). */
#define INTAKE_LAUNCH_POWER_MIN 0.3f
int intake_shoot_power (physics_world *world, vector3 chassis_pos, vector4 chassis_ori, vector3 chassis_vel,
                        int carried [INTAKE_MAX_CARRIED], int *carried_n, float power);
/* Aimed shot: power as above plus launch elevation in degrees [20, 80] (see
 * intake_launch_elev_default for the default). The stock 50-degree muzzle
 * cannot place a ball in the 1.5 m hive mouths from any legal field spot at
 * any power (proven by trajectory: arrival height falls 0.3+ m short of the
 * mouth) — regulation hive play needs the angle, not just the range.
 * Clamped, non-finite-safe. */
#define INTAKE_LAUNCH_ELEV_MIN 20.0f
#define INTAKE_LAUNCH_ELEV_MAX 80.0f
/* Menu-adjustable default launch elevation (deg, clamped [20,80], stock
 * INTAKE_LAUNCH_ELEV_DEG). Drives the un-aimed fallbacks in the shoot paths,
 * the fleet's default aim state resets and the gamepad's stick-centred
 * default. The module config menu (5 key) mutates it through the
 * mpe_module_config contract; it survives until the module dies. */
float intake_launch_elev_default (void);
void intake_launch_elev_default_set (float deg);
/* Module-config (5-key) shooter plane: the launch constants behind the
 * shooter balloon, all runtime-mutable through the module config contract.
 * Stock values are the INTAKE_LAUNCH_* defines above; the 5-menu stores one
 * value per axis (applied at the NEXT launch state computation, so the
 * purple predictor and fired balls stay consistent automatically):
 *   speed (m/s)   [3, 14]    stocked: INTAKE_LAUNCH_SPEED
 *   backspin (rad/s) [0, 30] stocked: INTAKE_LAUNCH_SPIN
 *   power floor [0.05, 0.95] stocked: INTAKE_LAUNCH_POWER_MIN */
float intake_launch_speed (void);
void intake_launch_speed_set (float speed);
float intake_launch_spin (void);
void intake_launch_spin_set (float spin);
float intake_launch_power_min (void);
void intake_launch_power_min_set (float power);
int intake_shoot_aimed (physics_world *world, vector3 chassis_pos, vector4 chassis_ori, vector3 chassis_vel,
                        int carried [INTAKE_MAX_CARRIED], int *carried_n, float power, float elev_deg);
/* Elevation-parameterized launch state (exported for the trajectory
 * predictor, which previews aimed shots through it; intake_launch_state_
 * compute() is the 50-degree specialization and is unchanged). */
void intake_launch_state_compute_elev (vector3 chassis_pos, vector4 chassis_ori, vector3 chassis_vel, float elev_deg,
                                       intake_launch_state *out);
/* Drop all carried balls: dynamic, kept at chassis velocity + small forward
 * nudge. Returns dropped count. */
int intake_eject (physics_world *world, vector3 chassis_vel, vector3 chassis_fwd, int carried [INTAKE_MAX_CARRIED],
                  int *carried_n);
/* Drop carried ids whose bodies are gone (scene cleared under us). */
void intake_prune (physics_world *world, int carried [INTAKE_MAX_CARRIED], int *carried_n);
/* Carry step: hold possessed balls at mouth slots (kinematic follow). */
void intake_carry_step (physics_world *world, vector3 chassis_pos, vector4 chassis_ori, vector3 chassis_vel,
                        int carried [INTAKE_MAX_CARRIED], int carried_n);
/* Ball aero for flying arcade balls (drag + Magnus, same formulas as
 * module_1's ball step — cited, not shared, different owner). Kinematic
 * (carried/scored) and sleeping balls are skipped. */
void intake_balls_aero_step (physics_world *world, const int *ball_ids, int ball_n);
#endif /* mfs_intake_h */

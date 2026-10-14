/* FTC robot fleet: per-world container that makes robots steppable as a
 * kernel tick module (see ftc_module.c).
 *
 * Manual (static-link) flow keeps working untouched: create an ftc_robot,
 * call drivetrain_update() yourself, step the world. The fleet exists so
 * a HOT-LOADED plugin can own robots instead: attach "ftc-fleet" to a
 * world, spawn through this API, then step the world only — the module's
 * pre_step runs drivetrain_update() for every fleet member before the
 * engine integrates velocity (same tick position as the manual flow, so
 * behavior is identical).
 *
 * The fleet holds plain ftc_robot structs (no pointers: memcpy-safe).
 * World bodies/joints created at spawn belong to the world, NOT the
 * fleet: detaching frees the fleet array only; bodies persist until the
 * world is cleared (same ownership rule as manual creation).
 *
 * LIFETIME RULE (matches bodies after physics_world_cleanup): robot
 * pointers from ftc_fleet_get() dangle after detach. Re-fetch after
 * re-attaching; body INDICES stay valid (detach never touches bodies).
 */
#ifndef ftc_fleet_h
#define ftc_fleet_h
#include "submodules/robot.h"
#include "submodules/biobuzz.h"
struct physics_world;
/* Spawn a robot into the fleet attached to `world`. Returns the fleet
 * index (>= 0), or -1 if no ftc-fleet module is attached, the fleet is
 * full, or creation fails. Robots are simulated in spawn order.
 * FIX-AUDIT-DESPOT: the FTC_FLEET_MAX (32) hard cap is an array bound, not
 * a performance claim — practical limit is 2-4 robots (each mecanum robot
 * adds 4 wheels + up to 32 roller bodies + 36 joints to one solver island;
 * cost grows superlinearly at 128 iterations). */
int ftc_fleet_spawn (struct physics_world *world, float x, float y, float z, motor_preset_id preset,
                     ftc_drivetrain_type drivetrain_type);
/* Live robot count in the world's fleet (0 if none attached). */
int ftc_fleet_count (struct physics_world *world);
/* Full league reset (scene_clear / load / rm -rf replaced the body pool):
 * forgets robots, balls, bucket, BIOBUZZ state and selection, keeps
 * allocations. The fleet pre_step self-heals the same way when it finds
 * nothing tracked alive; UI paths call this synchronously so a paused
 * engine still clears. Returns 0 ok, -1 no fleet. */
int ftc_fleet_clear (struct physics_world *world);/* Controlled-robot selection (per world; terminal `ftc select`, gamepad
 * BACK cycles). select: 0<=index<count sets and returns index, else -1
 * leaving stored untouched. selected: stored index (0 default), -1 when
 * no fleet/robots. */
int ftc_fleet_select (struct physics_world *world, int index);
int ftc_fleet_selected (struct physics_world *world);
/* Mutable robot access for commanding (drivetrain_tank/mecanum) and
 * inspection (odometry, battery). NULL on bad world/index. */
ftc_robot *ftc_fleet_get (struct physics_world *world, int index);
/* ---- arcade game league: balls, fictional intake, suspended bucket ----
 * (see submodules/intake.h for the abstraction). All dlsym-safe plain-C
 * entry points (the debug terminal and gamepad resolve these by name;
 * the engine never links FTC). Balls/bucket bodies belong to the world
 * (persist until scene clear, like robots); ids dangle safely (pruned). */
int ftc_balls_spawn (struct physics_world *world, int n);
/* BIOBUZZ scoring elements: spawn nectar_n nectar (9.1 cm dia) and pollen_n
 * pollen (7.1 cm dia) spheres ahead in lanes. Returns pieces made, -1 no
 * fleet. */
int ftc_biobuzz_spawn (struct physics_world *world, int nectar_n, int pollen_n);
int ftc_intake_set (struct physics_world *world, int index, int on);
int ftc_intake_toggle (struct physics_world *world, int index);
int ftc_eject (struct physics_world *world, int index);
int ftc_shoot (struct physics_world *world, int index);
/* Power-scaled shot (0.3..1.0, clamped; full power == ftc_shoot exactly).
 * The stock full-power lob outranges the field and cannot drop into the
 * hive mouths from legal spots; fractional power gives the soft lobs
 * regulation play needs. */
int ftc_shoot_power (struct physics_world *world, int index, float power);
/* Aimed shot (power 0.3..1.0, elevation 20..80 deg, both clamped): the
 * stock muzzle cannot place balls in the 1.5 m hive mouths from legal
 * spots at any power (proven by trajectory), so regulation hive play
 * needs the angle too. Records params for the trajectory predictor. */
int ftc_shoot_aimed (struct physics_world *world, int index, float power, float elev_deg);
/* Last aimed-shot params (defaults 1.0 / 50.0): what the predictor
 * previews. Returns 0 ok, -1 no fleet. */
int ftc_last_shot_params (struct physics_world *world, float *power_out, float *elev_out);
int ftc_bucket_spawn (struct physics_world *world, float x, float y, float z);
int ftc_bucket_clear (struct physics_world *world);
/* Totals: carried (CONTROLLED robot), tracked balls, scored, bucket active. Any out
 * pointer may be NULL. Returns 0 ok, -1 no fleet. */
int ftc_game_stats (struct physics_world *world, int *carried, int *balls, int *scored, int *bucket_on);
/* ---- BIOBUZZ regulation league (see submodules/biobuzz.h) ----
 * Field/robots/balls bodies belong to the world (persist until scene
 * clear); ids prune safely. All dlsym-safe plain-C entry points. */
int ftc_biobuzz_field (struct physics_world *world);
int ftc_biobuzz_match (struct physics_world *world);
int ftc_biobuzz_preload (struct physics_world *world);
int ftc_biobuzz_enter (struct physics_world *world, int alliance);
int ftc_biobuzz_score (struct physics_world *world, int alliance, biobuzz_alliance_score *out);
int ftc_biobuzz_place (struct physics_world *world, int index, float yaw);
int ftc_biobuzz_tips (struct physics_world *world, int *red, int *blue, int *unlocked_red, int *unlocked_blue);
int ftc_biobuzz_hive (struct physics_world *world, int hive, int out_ids [11]);
/* ---- module plumbing (used by ftc_module.c; not application API) ---- */
void *ftc_fleet_create (void);
void ftc_fleet_destroy (void *fleet_state);
void ftc_fleet_step_all (struct physics_world *world, void *fleet_state, float dt);
#endif /* ftc_fleet_h */

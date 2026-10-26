/* FTC robot fleet implementation. */
#include "ftc_fleet.h"
#include "submodules/drivetrain.h"
#include "submodules/intake.h"
#include "submodules/biobuzz.h"
#include "mfs_platform.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#define FTC_FLEET_INIT_CAP 4
/* FIX-AUDIT-DESPOT: hard cap 32 is an array bound, NOT a performance claim.
 * Practical limit is 2-4 robots: each mecanum robot adds 4 wheels + up to
 * 32 roller bodies + 36 joints to one Gauss-Seidel island, and solver cost
 * grows superlinearly with constraint count at 128 iterations. Past ~4 the
 * tick slows and contacts go unconverged; raise only with profiling. */
#define FTC_FLEET_MAX 32
/* Module registry name shared with ftc_module.c's descriptor. */
#define FTC_FLEET_MODULE_NAME "ftc-fleet"
typedef struct {
    ftc_robot *robots;
    int count;
    int cap;
    int selected; /* controlled robot (terminal `ftc select`, gamepad BACK
                   * cycles 0..count-1; default 0 via calloc). count never
                   * shrinks (no removal), so the stored index stays valid. */
    /* Arcade game league (fictional intake + suspended bucket). Balls are
     * world bodies (persist until scene clear); ids prune on lookup miss.
     * Bucket parts are static bodies; clear parks them non-colliding
     * (append-only pool has no removal). All zero-init via calloc. */
    int ball_ids [INTAKE_MAX_BALLS];
    int ball_count;
    int bucket_part_ids [6]; /* floor, +x, -x, +z, -z walls, (spare) */
    int bucket_active;
    float bucket_cx, bucket_cy, bucket_cz;
    int bucket_scored;
    /* BIOBUZZ regulation league (see submodules/biobuzz.h): own piece
     * registry (56 staged + headroom; the arcade 16-slot registry is
     * untouched) + league state. Zero-init via calloc; ids prune on miss. */
    biobuzz_state_t bb;
    /* Last aimed-shot params (defaults: full power, stock elevation). */
    float last_shot_power;
    float last_shot_elev;
    /* Live aim the NEXT shot will use. The gamepad updates this every tick
     * (trigger charge + right-stick elevation) and the trajectory predictor
     * reads it, so the purple line tracks the aim instead of the previous
     * shot. ftc_shoot_* also mirror the fired values here. */
    float aim_power;
    float aim_elev;
    /* Hive-entry aim-solve cache (ftc_aim_params_solve): pose the last
     * solve ran at + its result. aim_solve_ok = -1 (no entering arc at
     * this pose, cached so re-asks are free), 0 (valid pair), 1 (stale/
     * never run). Pose key: muzzle xyz (0.35 m cells) + yaw (5 deg). */
    float solve_muzzle [3];
    float solve_yaw;
    float solve_mouth [3]; /* target mouth centre at solve time (tips re-pose cells) */
    int solve_ok;                /* -1 no-arc cached, 0 valid pair cached, 1 stale */
    float solve_power, solve_elev;
} ftc_fleet_t;
void *ftc_fleet_create (void) {
    ftc_fleet_t *f = (ftc_fleet_t *) calloc (1, sizeof (ftc_fleet_t));
    if (!f)
        return NULL;
    f -> robots = (ftc_robot *) calloc ((size_t) FTC_FLEET_INIT_CAP, sizeof (ftc_robot));
    if (!f -> robots) {
        free (f);
        return NULL;
    } f -> cap = FTC_FLEET_INIT_CAP;
    /* Poison id slots: 0 must never read as a live body id. */
    memset (f -> ball_ids, 0xFF, sizeof (f -> ball_ids));
    memset (f -> bucket_part_ids, 0xFF, sizeof (f -> bucket_part_ids));
    /* Predictor defaults: full power, configured elevation (menu-tunable,
     * stock 65). */
    f -> last_shot_power = 1.0f;
    f -> last_shot_elev = intake_launch_elev_default ();
    f -> aim_power = 1.0f;
    f -> aim_elev = intake_launch_elev_default ();
    f -> solve_ok = 1; /* solve cache cold (calloc zero would read as "cached ok") */
    return f;
}
void ftc_fleet_destroy (void *fleet_state) {
    ftc_fleet_t *f = (ftc_fleet_t *) fleet_state;
    if (!f)
        return;
    free (f -> robots);
    free (f);
} /* Weak link into the bundle's internal registry (defined by
 * mfs_internal.c, ABSENT from standalone mpe_ftc builds). Lets fleet
 * lookup work through bundle attachments without linking bundle code:
 * when weak-unresolved the pointer is NULL and only the tick-table path
 * applies. */
#if MPE_WEAK_SUPPORTED
MPE_WEAK void *mfs_internal_module_state_for (const void *world, const char *name);
#else
/* No weak support (MSVC): avoid a link-time undefined reference (Windows
 * DLLs must resolve all symbols at link). Resolve the bundle helper at
 * runtime via GetProcAddress; NULL = tick-table path only. Works for both
 * standalone and bundle builds. */
typedef void * (*mfs_internal_lookup_fn) (const void *, const char *);
static void *mfs_bundle_lookup_runtime (const void *world, const char *name) {
    static mfs_internal_lookup_fn fn = NULL;
    static int tried = 0;
    if (!tried) {
        tried = 1;
        HMODULE h = GetModuleHandleA (NULL);
        if (h)
            fn = (mfs_internal_lookup_fn) GetProcAddress (h, "mfs_internal_module_state_for");
        if (!fn) {
            h = GetModuleHandleA ("mfs_ecosystem.dll");
            if (h)
                fn = (mfs_internal_lookup_fn) GetProcAddress (h, "mfs_internal_module_state_for");
        }
        if (!fn) {
            h = GetModuleHandleA ("mfs_ecosystem.so");
            if (h)
                fn = (mfs_internal_lookup_fn) GetProcAddress (h, "mfs_internal_module_state_for");
        }
    } return fn ? fn (world, name) : NULL;
}
#endif
/* Locate the fleet attached to a world by module name (no side table:
 * the state pointer lives in the world's own tick tables, so worlds
 * never share fleet state). Falls back to the bundle-internal attachment
 * when the world uses the mfs-simulator ecosystem instead of a direct
 * ftc-fleet tick attach. */
static ftc_fleet_t *fleet_of (struct physics_world *world) {
    if (!world)
        return NULL;
    for (int i = 0; i < world -> tick_module_count; i++) {
        if (world -> tick_modules [i] && world -> tick_modules [i] -> name &&
            strcmp (world -> tick_modules [i] -> name, FTC_FLEET_MODULE_NAME) == 0) {
            return (ftc_fleet_t *) world -> tick_module_state [i];
        }
    }
#if MPE_WEAK_SUPPORTED
    if (mfs_internal_module_state_for) { return (ftc_fleet_t *) mfs_internal_module_state_for ((const void *) world, FTC_FLEET_MODULE_NAME); }
#else
    {
        void *p = mfs_bundle_lookup_runtime ((const void *) world, FTC_FLEET_MODULE_NAME);
        if (p)
            return (ftc_fleet_t *) p;
    }
#endif
    return NULL;
}
int ftc_fleet_spawn (struct physics_world *world, float x, float y, float z, motor_preset_id preset,
                     ftc_drivetrain_type drivetrain_type) {
    ftc_fleet_t *f = fleet_of (world);
    if (!f || !f -> robots)
        return -1;
    if (f -> count >= FTC_FLEET_MAX)
        return -1;
    if (f -> count >= f -> cap) {
        int ncap = f -> cap * 2;
        /* DESPOT-FIX: old growth could realloc past FTC_FLEET_MAX (cap 32 ->
         * ncap 64) then keep spawning to 32 while holding 64 slots. Clamp so
         * capacity never exceeds the bound. */
        if (ncap > FTC_FLEET_MAX)
            ncap = FTC_FLEET_MAX;
        if (ncap <= f -> cap)
            return -1;
        ftc_robot *nr = (ftc_robot *) realloc (f -> robots, (size_t) ncap * sizeof (ftc_robot));
        if (!nr)
            return -1;
        f -> robots = nr;
        f -> cap = ncap;
    }
    if (ftc_robot_create_with_drive (world, &f -> robots [f -> count], x, y, z, preset, drivetrain_type) != 0) { return -1; }
    return f -> count++;
}
int ftc_fleet_count (struct physics_world *world) {
    ftc_fleet_t *f = fleet_of (world);
    return f ? f -> count : 0;
}
/* Full league reset: forget every robot, ball, bucket part, BIOBUZZ piece,
 * tip/unlock/score, and the selection — but keep the allocations (robots
 * array + cap), so a fresh match can spawn without realloc. Bodies already
 * belong to the world (scene_clear/load removed them); ids dangle safely
 * and every consumer re-resolves per tick. Returns 0 ok, -1 no fleet. */
int ftc_fleet_clear (struct physics_world *world) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f) { return -1; }
    if (f -> robots && f -> cap > 0) {
        memset (f -> robots, 0, (size_t) f -> cap * sizeof (ftc_robot));
        /* Mirror create-time poisoning: memset zeroes alias live id 0 /
         * joint 0 (the exact alias bug the create path documents). Slots
         * are unreachable at count 0, but poison anyway — a reader that
         * forgets the count gate must see -1, never a valid index. */
        for (int i = 0; i < f -> cap; i++) {
            ftc_robot *r = &f -> robots [i];
            r -> chassis_body = -1;
            memset (r -> wheel_bodies, 0xFF, sizeof (r -> wheel_bodies));
            memset (r -> wheel_joints, 0xFF, sizeof (r -> wheel_joints));
            memset (r -> roller_bodies, 0xFF, sizeof (r -> roller_bodies));
            memset (r -> roller_joints, 0xFF, sizeof (r -> roller_joints));
            memset (r -> intake_balls, 0xFF, sizeof (r -> intake_balls));
            for (int k = 0; k < FTC_MAX_WHEELS; k++) { r -> wheel_traction_scale [k] = 1.0f; }
        }
    }
    f -> count = 0;
    f -> selected = 0;
    memset (f -> ball_ids, 0xFF, sizeof (f -> ball_ids));
    f -> ball_count = 0;
    memset (f -> bucket_part_ids, 0xFF, sizeof (f -> bucket_part_ids));
    f -> bucket_active = 0;
    f -> bucket_scored = 0;
    f -> bucket_cx = 0.0f;
    f -> bucket_cy = 0.0f;
    f -> bucket_cz = 0.0f;
    memset (&f -> bb, 0, sizeof (f -> bb));
    f -> last_shot_power = 1.0f;
    f -> last_shot_elev = intake_launch_elev_default ();
    f -> aim_power = 1.0f;
    f -> aim_elev = intake_launch_elev_default ();
    f -> solve_ok = 1; /* the scene cleared under the solve cache */
    return 0;
}
/* Controlled-robot selection (per world; terminal `ftc select`, gamepad
 * BACK). select: 0<=index<count sets and returns index, else -1 leaving
 * the stored value untouched. selected: stored index, defensively
 * re-based to 0 when out of range (count never shrinks, so this is
 * unreachable in practice), -1 when no fleet/robots. */
int ftc_fleet_select (struct physics_world *world, int index) {
    ftc_fleet_t *f = fleet_of (world);
    if (!f || index < 0 || index >= f -> count) { return -1; }
    f -> selected = index;
    return index;
}
int ftc_fleet_selected (struct physics_world *world) {
    ftc_fleet_t *f = fleet_of (world);
    if (!f || f -> count <= 0) { return -1; }
    if (f -> selected < 0 || f -> selected >= f -> count) { return 0; }
    return f -> selected;
}
ftc_robot *ftc_fleet_get (struct physics_world *world, int index) {
    ftc_fleet_t *f = fleet_of (world);
    if (!f || !f -> robots || index < 0 || index >= f -> count)
        return NULL;
    return &f -> robots [index];
}
/* Forward: game update is defined after the API below. */
static void ftc_game_update (struct physics_world *world, ftc_fleet_t *f, float dt);
void ftc_fleet_step_all (struct physics_world *world, void *fleet_state, float dt) {
    ftc_fleet_t *f = (ftc_fleet_t *) fleet_state;
    if (!world || !f || !f -> robots || !(dt > 0.0f))
        return;
    /* Spawn order = update order: deterministic across runs and
     * across static/dynamic copies of this code. */
    for (int i = 0; i < f -> count; i++) {
        drivetrain_update (world, &f -> robots [i], dt);
    }
    ftc_game_update (world, f, dt);
}
/* ---- arcade game league ---- */
static ftc_robot *fleet_robot_at (ftc_fleet_t *f, int index) {
    if (!f || !f -> robots || index < 0 || index >= f -> count) { return NULL; }
    return &f -> robots [index];
}
static rigidbody *fleet_chassis (struct physics_world *world, ftc_robot *r) {
    if (!world || !r || r -> chassis_body < 0 || r -> chassis_body >= world -> body_count) { return NULL; }
    return &world -> bodies [r -> chassis_body];
}
/* Playable half-extent when a regulation-sized floor slab is present, else 0.
 * The BIOBUZZ league builds a 144 in (1.8288 m half) slab at y=0; the arcade
 * practice pad is 30 m half and deliberately ignored. Arcade scatter used to
 * place balls/pieces/bucket at z=2.2..4.0 m, i.e. OUTSIDE the regulation
 * perimeter wall (z=±1.83) — unreachable on the league field (user report:
 * "can't even shoot balls into the baskets"). Callers map their scatter into
 * this interior when it is available; the 60 m pad keeps its old layout. */
static float ftc_playable_half (struct physics_world *world) {
    if (!world || !world -> bodies) { return 0.0f; }
    for (int i = 0; i < world -> body_count; i++) {
        rigidbody *b = &world -> bodies [i];
        if (!b -> static_state && !(b -> mass == 0.0f)) { continue; }
        float top = b -> position.y + b -> half_extensions.y;
        if (fabsf (top) < 0.01f && b -> half_extensions.x > 0.5f && b -> half_extensions.x <= 5.0f &&
            fabsf (b -> half_extensions.x - b -> half_extensions.z) < 0.02f) {
            return b -> half_extensions.x;
        }
    }
    return 0.0f;
}
int ftc_balls_spawn (struct physics_world *world, int n) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f) { return -1; }
    if (n < 1) { n = 3; }
    int made = 0;
    int room = INTAKE_MAX_BALLS - f -> ball_count;
    int n_eff = (n < room) ? n : room;
    int rows = (n_eff + 2) / 3;
    float lim = ftc_playable_half (world);
    for (int i = 0; i < n_eff; i++) {
        /* Deterministic scatter ahead: three lanes (left/centre/right) so
         * a straight drive down the middle scoops at least the centre
         * lane through the 0.32 m mouth; tight rows so one pass catches
         * twice (the drive covers ~2.3 m). On a regulation field the rows
         * are compressed inside the perimeter instead of being staged
         * beyond the wall. */
        float x = ((float) (i % 3) - 1.0f) * 0.3f;
        float z;
        if (lim > 0.0f) {
            const float z0 = 0.55f, z1 = (lim - 0.28f > z0) ? (lim - 0.28f) : z0;
            float t = (rows > 1) ? (float) (i / 3) / (float) (rows - 1) : 0.0f;
            z = z0 + (z1 - z0) * t;
        } else {
            z = 2.2f + 0.4f * (float) (i / 3);
        }
        int bi = physics_world_add_sphere (world, INTAKE_BALL_RADIUS, INTAKE_BALL_MASS,
                                           (vector3) {x, INTAKE_BALL_RADIUS + 0.001f, z});
        if (bi < 0) { break; }
        rigidbody *b = &world -> bodies [bi];
        b -> restitution = INTAKE_BALL_RESTITUTION;
        b -> friction_static = INTAKE_BALL_FRICTION_S;
        b -> friction_kinetic = INTAKE_BALL_FRICTION_K;
        b -> colour = (vector3) {1.0f, 0.4f, 0.0f};
        rigidbody_wake (b);
        f -> ball_ids [f -> ball_count++] = (int) b -> object_id;
        made++;
    } return made;
}
/* BIOBUZZ league pieces: regulation-size NECTAR (~9.1 cm dia) and POLLEN
 * (~7.1 cm dia) spheres, staged ahead in three lanes like the small-ball
 * scatter so a straight drive scoops a lane through the mouth. Colours per
 * regulation (pollen yellow, nectar red), mass r^3 from the arcade spec.
 * Registered in the same ball registry, so intake capture/carry/launch,
 * ball aero and bucket scoring all apply unchanged. */
int ftc_biobuzz_spawn (struct physics_world *world, int nectar_n, int pollen_n) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f) { return -1; }
    if (nectar_n < 0) { nectar_n = 0; }
    if (pollen_n < 0) { pollen_n = 0; }
    int made = 0;
    int total = nectar_n + pollen_n;
    int room = INTAKE_MAX_BALLS - f -> ball_count;
    int n_eff = (total < room) ? total : room;
    int rows = (n_eff + 2) / 3;
    float lim = ftc_playable_half (world);
    for (int k = 0; k < n_eff; k++) {
        int is_nectar = (k < nectar_n) ? 1 : 0;
        float radius = is_nectar ? BIOBUZZ_NECTAR_RADIUS : BIOBUZZ_POLLEN_RADIUS;
        float mass = is_nectar ? BIOBUZZ_NECTAR_MASS : BIOBUZZ_POLLEN_MASS;
        /* Three lanes (left/centre/right); nectar staged a touch further
         * than pollen so both kinds sit in the drive path and the intake
         * mouths a lane. Regulation field: compress the rows inside the
         * walls (the 2.6-3.4 m arcade staging is outside 1.83 m). */
        float x = ((float) (k % 3) - 1.0f) * 0.3f;
        float z;
        if (lim > 0.0f) {
            const float z0 = 0.55f, z1 = (lim - 0.28f > z0) ? (lim - 0.28f) : z0;
            float t = (rows > 1) ? (float) (k / 3) / (float) (rows - 1) : 0.0f;
            z = z0 + (z1 - z0) * t;
        } else {
            z = (is_nectar ? 2.6f : 1.8f) + 0.4f * (float) (k / 3);
        }
        int bi = physics_world_add_sphere (world, radius, mass, (vector3) {x, radius + 0.001f, z});
        if (bi < 0) { break; }
        rigidbody *b = &world -> bodies [bi];
        b -> restitution = INTAKE_BALL_RESTITUTION;
        b -> friction_static = INTAKE_BALL_FRICTION_S;
        b -> friction_kinetic = INTAKE_BALL_FRICTION_K;
        b -> colour = is_nectar ? (vector3) {0.95f, 0.25f, 0.25f} : (vector3) {1.0f, 0.9f, 0.2f};
        rigidbody_wake (b);
        f -> ball_ids [f -> ball_count++] = (int) b -> object_id;
        made++;
    } return made;
}
int ftc_intake_set (struct physics_world *world, int index, int on) {
    ftc_fleet_t *f = fleet_of (world);
    ftc_robot *r = fleet_robot_at (f, index);
    if (!r) { return -1; }
    r -> intake_on = on ? 1 : 0;
    return r -> intake_on;
}
int ftc_intake_toggle (struct physics_world *world, int index) {
    ftc_fleet_t *f = fleet_of (world);
    ftc_robot *r = fleet_robot_at (f, index);
    if (!r) { return -1; }
    r -> intake_on = r -> intake_on ? 0 : 1;
    return r -> intake_on;
}
int ftc_eject (struct physics_world *world, int index) {
    ftc_robot *r = fleet_robot_at (fleet_of (world), index);
    if (!world || !r) { return -1; }
    rigidbody *ch = fleet_chassis (world, r);
    if (!ch) { return -1; }
    vector3 fwd = vector4_rotate_to_vector3 (ch -> orientation, (vector3) {0.0f, 0.0f, 1.0f});
    return intake_eject (world, ch -> velocity, fwd, r -> intake_balls, &r -> intake_count);
}
int ftc_shoot (struct physics_world *world, int index) {
    ftc_fleet_t *f = fleet_of (world);
    ftc_robot *r = fleet_robot_at (f, index);
    if (!world || !r) { return -1; }
    rigidbody *ch = fleet_chassis (world, r);
    if (!ch) { return -1; }
    float elev = intake_launch_elev_default ();
    int id = intake_shoot_aimed (world, ch -> position, ch -> orientation, ch -> velocity, r -> intake_balls,
                                 &r -> intake_count, 1.0f, elev);
    if (id >= 0) {
        fprintf (stderr, "[bucketball] fired ball id=%d\n", id);
        if (f) { f -> aim_power = 1.0f; f -> aim_elev = intake_launch_elev_default (); }
    }
    return id;
}
int ftc_shoot_power (struct physics_world *world, int index, float power) {
    ftc_fleet_t *f = fleet_of (world);
    ftc_robot *r = fleet_robot_at (f, index);
    if (!world || !r) { return -1; }
    rigidbody *ch = fleet_chassis (world, r);
    if (!ch) { return -1; }
    float elev = intake_launch_elev_default ();
    int id = intake_shoot_aimed (world, ch -> position, ch -> orientation, ch -> velocity, r -> intake_balls,
                                 &r -> intake_count, power, elev);
    if (id >= 0) {
        fprintf (stderr, "[bucketball] fired ball id=%d power=%.2f\n", id, power);
        if (f) {
            float ap = isfinite (power) ? power : 1.0f;
            if (ap < intake_launch_power_min ()) { ap = intake_launch_power_min (); }
            if (ap > 1.0f) { ap = 1.0f; }
            f -> aim_power = ap;
            f -> aim_elev = intake_launch_elev_default ();
        }
    }
    return id;
}
int ftc_shoot_aimed (struct physics_world *world, int index, float power, float elev_deg) {
    ftc_fleet_t *f = fleet_of (world);
    ftc_robot *r = fleet_robot_at (f, index);
    if (!world || !r) { return -1; }
    rigidbody *ch = fleet_chassis (world, r);
    if (!ch) { return -1; }
    if (!isfinite (power)) { power = 1.0f; }
    if (power < intake_launch_power_min ()) { power = intake_launch_power_min (); }
    if (power > 1.0f) { power = 1.0f; }
    if (!isfinite (elev_deg)) { elev_deg = intake_launch_elev_default (); }
    if (elev_deg < 20.0f) { elev_deg = 20.0f; }
    if (elev_deg > 80.0f) { elev_deg = 80.0f; }
    int id = intake_shoot_aimed (world, ch -> position, ch -> orientation, ch -> velocity, r -> intake_balls,
                                 &r -> intake_count, power, elev_deg);
    if (id >= 0) {
        fprintf (stderr, "[bucketball] fired ball id=%d power=%.2f elev=%.1f\n", id, power, (double) elev_deg);
        if (f) {
            f -> last_shot_power = power;
            f -> last_shot_elev = elev_deg;
            f -> aim_power = power;
            f -> aim_elev = elev_deg;
        }
    }
    return id;
}
int ftc_last_shot_params (struct physics_world *world, float *power_out, float *elev_out) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f) { return -1; }
    if (power_out) { *power_out = f -> last_shot_power; }
    if (elev_out) { *elev_out = f -> last_shot_elev; }
    return 0;
}
/* Live aim (the shot the controls would fire now). The gamepad writes the
 * trigger charge + stick elevation every tick so the trajectory predictor's
 * purple line matches the actual shot: before this, the line previewed the
 * LAST shot while the analog trigger fired a different power ("the motor is
 * running out of battery"). Clamps exactly like ftc_shoot_aimed; a
 * non-finite value is ignored (keeps the stored aim). */
int ftc_aim_params_set (struct physics_world *world, float power, float elev_deg) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f) { return -1; }
    if (isfinite (power)) {
        if (power < intake_launch_power_min ()) { power = intake_launch_power_min (); }
        if (power > 1.0f) { power = 1.0f; }
        f -> aim_power = power;
    }
    if (isfinite (elev_deg)) {
        if (elev_deg < 20.0f) { elev_deg = 20.0f; }
        if (elev_deg > 80.0f) { elev_deg = 80.0f; }
        f -> aim_elev = elev_deg;
    }
    return 0;
}
int ftc_aim_params_get (struct physics_world *world, float *power_out, float *elev_out) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f) { return -1; }
    if (power_out) { *power_out = f -> aim_power; }
    if (elev_out) { *elev_out = f -> aim_elev; }
    return 0;
}
/* ---- hive-entry aim solve (see ftc_fleet.h) ---- */
/* Coarse grid the DESPOT-2026-10-22 probe validated: the entering band is
 * elevation 55-75 at power 0.62-0.82 for gaps 0.9-3.4 m; links clamps are
 * the shooter's own (power 0.3-1.0, elevation 20-80), sampled densely
 * inside that proven band plus edges. */
static const float bbm_solve_elev [8] = {45.0f, 50.0f, 55.0f, 60.0f, 65.0f, 70.0f, 75.0f, 80.0f};
static const float bbm_solvePow [11] = {0.40f, 0.45f, 0.50f, 0.55f, 0.60f, 0.65f, 0.70f, 0.75f, 0.80f, 0.85f,
                                        0.90f};
/* One candidate arc: place the carried ball in the throwaway world at
 * (power, elevation) and march it with the real aero; entering = the ball
 * ends the march settled INSIDE the up-cell window (the mouth is LOADED
 * with 3 match nectar in the full world, staged by biobuzz_stage_match and
 * mirrored into the march world by the caller: an arc whose ball crosses
 * the plane but bounces back out on the loaded stack is NOT an entry).
 * Scored by the settle window margin (min of the two window margins).
 * Returns the score, or -1 on no settle. march caps at 150 ticks (~2.5 s). */
static float bbm_march_entry (physics_world *pred, int pi, uint32_t bid, ftc_robot *r, rigidbody *ch, float power,
                              float elev_deg, vector3 cc, vector4 cq, float ball_r) {
    (void) r; /* carried spec picked by the caller; kept for signature symmetry with the shooter */
    rigidbody *pb = &pred -> bodies [pi];
    intake_launch_state lstt;
    intake_launch_state_compute_elev (ch -> position, ch -> orientation, ch -> velocity, elev_deg, &lstt);
    vector3 rel = vector3_subtraction (lstt.velocity, ch -> velocity);
    pb -> position = lstt.muzzle;
    pb -> velocity = vector3_addition (ch -> velocity, vector3_scaling (rel, power));
    pb -> angular_velocity = vector3_scaling (lstt.spin, power);
    rigidbody_wake (pb);
    rigidbody_update_axes (pb);
    vector3 cn = vector4_rotate_to_vector3 (cq, (vector3) {0.0f, 0.0f, 1.0f}); /* outward mouth normal */
    int dbg = getenv ("MPE_HIVE_AIM_MARCH_DEBUG") != NULL; /* per-candidate trace, opt-in */
    float tailscore = -1.0f; /* window margin at the most recent inside-window tick */
    int inside_run = 0;      /* consecutive ticks past the plane AND inside the window */
    for (int t = 0; t < 150; t++) {
        int id = (bid <= (uint32_t) INT_MAX) ? (int) bid : -1;
        if (id < 0) { break; }
        int ids1 [1] = {id};
        intake_balls_aero_step (pred, ids1, 1);
        physics_world_step (pred, 1.0f / 60.0f);
        vector3 p = pb -> position;
        if (!isfinite (p.x) || !isfinite (p.y) || !isfinite (p.z)) { break; }
        if (dbg && (t % 10 == 0 || t == 149)) {
            fprintf (stderr, "  [march] t=%3d pos=(%.3f,%.3f,%.3f) v=(%.2f,%.2f,%.2f)\n", t, (double) p.x, (double) p.y,
                     (double) p.z, (double) pb -> velocity.x, (double) pb -> velocity.y, (double) pb -> velocity.z);
        }
        vector3 d2 = vector3_subtraction (p, cc);
        float ala = d2.x * cn.x + d2.y * cn.y + d2.z * cn.z;
        /* window bookkeeping: at/inside the plane */
        if (ala <= 0.0f) {
            vector3 lo = biobuzz_unrotate (cq, d2);
            float mx = BIOBUZZ_CELL_W * 0.5f - ball_r - 0.005f;
            float my = BIOBUZZ_CELL_H * 0.5f - ball_r - 0.005f;
            int inw = (fabsf (lo.x) <= mx && fabsf (lo.y) <= my);
            inside_run = inw ? (inside_run + 1) : 0;
            if (inw) {
                float score = mx - fabsf (lo.x);
                float score2 = my - fabsf (lo.y);
                if (score2 < score) { score = score2; }
                tailscore = score;
            }
        } else { inside_run = 0; }
        /* early break only for a floor rest: inside the tilted cell the ball
         * never reaches floor height (the cell sits ~1.4 m up) */
        if (t > 10 && p.y <= ball_r * 0.6f &&
            vector3_length_squared (pb -> velocity) < 0.05f) { break; /* settled on the floor */ }
    }
    if (dbg) {
        vector3 pe = pb -> position;
        fprintf (stderr, "  [march] verdict inside_run=%d tailscore=%.3f end=(%.3f,%.3f,%.3f)\n", inside_run,
                 (double) tailscore, (double) pe.x, (double) pe.y, (double) pe.z);
    }
    if (inside_run < 10) { return -1.0f; } /* no half-second persistent settle inside */
    return tailscore;
}
int ftc_aim_params_solve (struct physics_world *world, float *power_out, float *elev_out) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f || !f -> robots) { return -1; }
    int st = f -> selected;
    if (st < 0 || st >= f -> count) { st = 0; }
    if (f -> count <= 0) { return -1; }
    ftc_robot *r = &f -> robots [st];
    if (r -> chassis_body < 0 || r -> chassis_body >= world -> body_count) { return -1; }
    if (!f -> bb.field_built) { return -1; } /* no hive to score into */
    rigidbody *ch = &world -> bodies [r -> chassis_body];
    /* Muzzle + yaw key (elevation never moves the muzzle, so one compute
     * at any elevation gets the launch origin). */
    intake_launch_state lstt;
    intake_launch_state_compute_elev (ch -> position, ch -> orientation, ch -> velocity, 50.0f, &lstt);
    vector3 mz = lstt.muzzle;
    vector3 fwd = vector4_rotate_to_vector3 (ch -> orientation, (vector3) {0.0f, 0.0f, 1.0f});
    float cyaw = atan2f (fwd.z, fwd.x);
    if (f -> solve_ok != 1) {
        float ddx = mz.x - f -> solve_muzzle [0], ddy = mz.y - f -> solve_muzzle [1], ddz = mz.z - f -> solve_muzzle [2];
        float dyf = fabsf (cyaw - f -> solve_yaw);
        if (dyf > 3.14159265f) { dyf = 6.28318531f - dyf; }
        /* mouth centre moves on tip re-poses: a solve for the old pose of a
         * swung cell is wrong — check which mouth we solved for and compare
         * its CURRENT centre against the solved one */
        float dmx = 1e9f;
        for (int h = 0; h < 2; h++) {
            float mdx = f -> bb.up_mouth [h][0] - f -> solve_mouth [0];
            float mdy = f -> bb.up_mouth [h][1] - f -> solve_mouth [1];
            float mdz = f -> bb.up_mouth [h][2] - f -> solve_mouth [2];
            float md = mdx * mdx + mdy * mdy + mdz * mdz;
            if (md < dmx) { dmx = md; }
        }
        if (ddx * ddx + ddy * ddy + ddz * ddz < 0.35f * 0.35f && dyf < 0.09f && dmx < 0.08f * 0.08f) {
            /* cache hit: pose held and the solved cell did not re-pose */
            if (f -> solve_ok == -1) { return -1; }
            if (power_out) { *power_out = f -> solve_power; }
            if (elev_out) { *elev_out = f -> solve_elev; }
            return 0;
        }
    }
    /* Target mouth: nearest up cell for the SELECTED robot — the nose
     * does not have to be aimed yet (the march below fails cleanly when
     * it is not); picking by range avoids the approach-cone false
     * negative when the robot parked at an off-axis regulation pose
     * (LOS arrives ~40 deg off the +90 deg nose there and is still a
     * legal single turn away). tip re-poses included via the live
     * frame getter. */
    int pickh = -1, pickend = 0;
    float pickd = 1e9f;
    float losoff = 0.0f; /* kept for the failure hint */
    for (int h = 0; h < 2; h++) {
        if (f -> bb.field_built != 1) { continue; }
        vector3 cc; vector4 cq;
        int ue = f -> bb.tilt [h];
        if (biobuzz_up_cell_frame (h, ue, &cc, &cq) != 0) { continue; }
        float dx = cc.x - mz.x;
        float dz = cc.z - mz.z;
        float lyaw = atan2f (dz, dx); /* chassis-yaw convention atan2(z,x)! */
        float dyy = lyaw - cyaw;
        while (dyy > 3.14159265f) { dyy -= 6.28318531f; }
        while (dyy < -3.14159265f) { dyy += 6.28318531f; }
        float dist = dx * dx + dz * dz;
        if (dist < pickd) { pickd = dist; pickh = h; pickend = ue; losoff = dyy; }
    }
    if (pickh < 0) {
        fprintf (stderr, "[hive-aim] no up-cell mouth staged (field_built=%d)\n", (int) f -> bb.field_built);
        f -> solve_ok = -1;
        f -> solve_muzzle [0] = mz.x; f -> solve_muzzle [1] = mz.y; f -> solve_muzzle [2] = mz.z;
        f -> solve_yaw = cyaw;
        f -> solve_mouth [0] = 0.0f; f -> solve_mouth [1] = -1000.0f; f -> solve_mouth [2] = 0.0f;
        return -1;
    }
    vector3 cc; vector4 cq;
    if (biobuzz_up_cell_frame (pickh, pickend, &cc, &cq) != 0) { return -1; }
    /* Carried ball spec — falls back to the regulation pollen when the
     * intake is empty (aim geometry is independent of the payload; the
     * shooter simply has nothing to fire). */
    int bi = (r -> intake_count > 0) ? physics_world_index_by_id (world, (uint32_t) r -> intake_balls [0]) : -1;
    float brad = (bi >= 0 && isfinite (world -> bodies [bi].radius) && world -> bodies [bi].radius > 0.001f)
                     ? world -> bodies [bi].radius : BIOBUZZ_POLLEN_RADIUS;
    float bmass = (bi >= 0 && isfinite (world -> bodies [bi].mass) && world -> bodies [bi].mass > 0.0f)
                      ? world -> bodies [bi].mass : BIOBUZZ_POLLEN_MASS;
    float brest = (bi >= 0) ? world -> bodies [bi].restitution : 0.65f;
    float bfst = (bi >= 0) ? world -> bodies [bi].friction_static : 0.45f;
    float bfkn = (bi >= 0) ? world -> bodies [bi].friction_kinetic : 0.35f;
    /* Throwaway world + statics mirror (same filter as ftc_traj) + ONE
     * ball reset per candidate: statics copy is the expensive half. */
    physics_world pred;
    physics_world_init (&pred);
    constraint_pool_init (&pred);
    mpe_config_t pcf = *mpe_world_cfg (world);
    pcf.timestep.solver_iterations = 8;
    physics_world_set_config (&pred, &pcf);
    if (world -> static_plane_enabled) {
        pred.static_plane_enabled = true;
        pred.static_plane_body = world -> static_plane_body;
    }
    for (int ib = 0; ib < world -> body_count; ib++) {
        rigidbody *b = &world -> bodies [ib];
        if (!((b -> static_state || b -> mass == 0.0f) && !b -> kinematic && !b -> no_collide)) { continue; }
        int di = -1;
        if (b -> type == object_cube) {
            di = physics_world_add_cube (&pred, b -> position, b -> half_extensions, 0.0f);
        } else if (b -> type == object_sphere) {
            di = physics_world_add_sphere (&pred, b -> radius, 0.0f, b -> position);
        } else if (b -> type == object_cylinder) {
            di = physics_world_add_cylinder (&pred, b -> radius, b -> cylinder_half_length, 0.0f, b -> position);
        }
        if (di < 0) { continue; }
        rigidbody *db = &pred.bodies [di];
        db -> static_state = true;
        db -> inverse_mass = 0.0f;
        db -> friction_static = b -> friction_static;
        db -> friction_kinetic = b -> friction_kinetic;
        db -> restitution = b -> restitution;
        db -> orientation = b -> orientation;
        rigidbody_update_axes (db);
    }
    int pi = physics_world_add_sphere (&pred, brad, bmass, (vector3) {0.0f, -50.0f, 0.0f});
    if (pi >= 0) {
        rigidbody *pb = &pred.bodies [pi];
        pb -> restitution = brest;
        pb -> friction_static = bfst;
        pb -> friction_kinetic = bfkn;
        rigidbody_wake (pb);
        rigidbody_update_axes (pb);
    }
    float bestscore = -1.0f;
    float bp = 0.0f, be = 0.0f;
    /* Mouth occupants: the staged match seats 3 nectar against the cell's
     * back wall (biobuzz_stage_match, Manual §10.3.1). The march must see
     * the SAME mouth: an arc that would bounce back out on the loaded stack
     * is not an entry. Same spec as the fleet's own balls. */
    {
        int side = (pickh == 0) ? -1 : 1;
        for (int k = 0; k < BIOBUZZ_CELL_NECTAR; k++) {
            vector3 loc = {side * 0.10f - side * (float) k * 0.095f, -0.10f,
                           -BIOBUZZ_CELL_D * 0.5f + BIOBUZZ_NECTAR_RADIUS + 0.005f};
            vector3 off = vector4_rotate_to_vector3 (cq, loc);
            vector3 p = {cc.x + off.x, cc.y + off.y, cc.z + off.z};
            int oi = physics_world_add_sphere (&pred, BIOBUZZ_NECTAR_RADIUS, BIOBUZZ_NECTAR_MASS, p);
            if (oi >= 0) {
                rigidbody *ob = &pred.bodies [oi];
                ob -> restitution = INTAKE_BALL_RESTITUTION;
                ob -> friction_static = INTAKE_BALL_FRICTION_S;
                ob -> friction_kinetic = INTAKE_BALL_FRICTION_K;
                rigidbody_wake (ob);
                rigidbody_update_axes (ob);
            }
        }
    }
    if (pi >= 0 && pred.bodies [pi].object_id <= (uint32_t) INT_MAX) {
        /* Steepest-first: the first entering arc is the highest elevation
         * row that drops inside — flat arcs that scrape the mouth's
         * underside (the "firing at the belly" complaint) are skipped even
         * if the march accepts them. */
        for (int ei = 7; ei >= 0 && bestscore < 0.0f; ei--) {
            for (int qi = 0; qi < 11 && bestscore < 0.0f; qi++) {
                float sc = bbm_march_entry (&pred, pi, pred.bodies [pi].object_id, r, ch, bbm_solvePow [qi],
                                            bbm_solve_elev [ei], cc, cq, brad);
                if (sc > bestscore) { bestscore = sc; bp = bbm_solvePow [qi]; be = bbm_solve_elev [ei]; }
            }
        }
    }
    physics_world_cleanup (&pred);
    f -> solve_muzzle [0] = mz.x; f -> solve_muzzle [1] = mz.y; f -> solve_muzzle [2] = mz.z;
    f -> solve_yaw = cyaw;
    f -> solve_mouth [0] = cc.x; f -> solve_mouth [1] = cc.y; f -> solve_mouth [2] = cc.z;
    if (bestscore < 0.0f) {
        fprintf (stderr, "[hive-aim] no entering arc from this muzzle (%.2f,%.2f,%.2f): LOS azimuth is %.0f deg off the nose — yaw toward the mouth (up mouths face outboard: red -Z approach, blue +Z)\n",
                 (double) mz.x, (double) mz.y, (double) mz.z, (double) (losoff * 57.2958f));
        f -> solve_ok = -1;
        return -1;
    }
    f -> solve_ok = 0;
    f -> solve_power = bp;
    f -> solve_elev = be;
    fprintf (stderr, "[hive-aim] solved power=%.2f elev=%.0f deg\n", (double) bp, (double) be);
    if (power_out) { *power_out = bp; }
    if (elev_out) { *elev_out = be; }
    return 0;
}
/* Suspended bucket: a miniature walled field pad (floor + 4 walls, open
 * top) staged as static bodies. Re-spawning an active bucket MOVES the
 * assembly (statics teleport cleanly); clear parks parts non-colliding
 * (append-only pool has no removal — documented, same rule as robots). */
#define BUCKET_PAD_HALF 0.8f /* 1.6 m pad: reads as a miniature field and gives the zone the span to catch aim variance */
#define BUCKET_WALL_H 0.35f /* low rim: lobs clear it instead of perching (measured) */
#define BUCKET_WALL_T 0.08f
static int bucket_add_static (struct physics_world *world, vector3 pos, vector3 half, vector3 colour,
                              vector4 orient) {
    int bi = physics_world_add_cube (world, pos, half, 0.0f);
    if (bi < 0) { return -1; }
    rigidbody *b = &world -> bodies [bi];
    b -> friction_static = 0.8f;
    b -> friction_kinetic = 0.6f;
    b -> restitution = 0.0f;
    b -> colour = colour;
    b -> orientation = orient;
    rigidbody_update_axes (b);
    return (int) b -> object_id;
}
int ftc_bucket_spawn (struct physics_world *world, float x, float y, float z) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f) { return -1; }
    vector3 pad_c = {x, y, z};
    vector3 wall_c = {0.85f, 0.85f, 0.9f};
    vector3 pad_col = {0.2f, 0.4f, 0.85f};
    if (f -> bucket_active) {
        /* Move the live assembly (same five slots, new pose). */
        static const vector3 offs [5] = {
            {0.0f, 0.0f, 0.0f},
            {BUCKET_PAD_HALF - BUCKET_WALL_T * 0.5f, (0.05f + BUCKET_WALL_H) * 0.5f, 0.0f},
            {-(BUCKET_PAD_HALF - BUCKET_WALL_T * 0.5f), (0.05f + BUCKET_WALL_H) * 0.5f, 0.0f},
            {0.0f, (0.05f + BUCKET_WALL_H) * 0.5f, BUCKET_PAD_HALF - BUCKET_WALL_T * 0.5f},
            {0.0f, (0.05f + BUCKET_WALL_H) * 0.5f, -(BUCKET_PAD_HALF - BUCKET_WALL_T * 0.5f)},
        };
        for (int i = 0; i < 5; i++) {
            int bi = physics_world_index_by_id (world, (uint32_t) f -> bucket_part_ids [i]);
            if (bi < 0 || bi >= world -> body_count) {
                f -> bucket_active = 0;
                break;
            }
            world -> bodies [bi].position =
                vector3_addition (pad_c, offs [i]);
            rigidbody_update_axes (&world -> bodies [bi]);
        }
        if (f -> bucket_active) {
            f -> bucket_cx = x;
            f -> bucket_cy = y;
            f -> bucket_cz = z;
            return 0;
        }
        /* Fell apart underneath (scene cleared): fall through and rebuild. */
    }
    int ids [6];
    vector4 ident = vector4_identity ();
    /* 20° flare as hardcoded unit quaternions (w,x,y,z) — no libm, exact
     * literals, spawn-time deterministic: roll about Z ∓20° tips ±X-wall
     * tops outward, about X ±20° tips ±Z-wall tops outward. */
    vector4 roll_px = {0.98480775f, 0.0f, 0.0f, -0.17364818f}; /* +X wall */
    vector4 roll_nx = {0.98480775f, 0.0f, 0.0f, 0.17364818f}; /* -X wall */
    vector4 roll_pz = {0.98480775f, 0.17364818f, 0.0f, 0.0f}; /* +Z wall */
    vector4 roll_nz = {0.98480775f, -0.17364818f, 0.0f, 0.0f}; /* -Z wall */
    ids [0] = bucket_add_static (world, pad_c, (vector3) {BUCKET_PAD_HALF, 0.05f, BUCKET_PAD_HALF}, pad_col, ident);
    float wy = y + (0.05f + BUCKET_WALL_H) * 0.5f;
    float wo = BUCKET_PAD_HALF - BUCKET_WALL_T * 0.5f;
    /* Funnel flare (DESPOT-2026-10-12, measured): lobs arrive at rim
     * height with spent forward velocity and perch or glance off a
     * vertical lip. Rolling each wall 20° about its long axis flares the
     * mouth outward; with dead (e=0) walls a rim strike slides DOWN the
     * flare into the bucket instead of balancing or bouncing out. */
    ids [1] = bucket_add_static (world, (vector3) {x + wo, wy, z}, (vector3) {BUCKET_WALL_T * 0.5f, BUCKET_WALL_H * 0.5f,
                                                                              BUCKET_PAD_HALF}, wall_c, roll_px);
    ids [2] = bucket_add_static (world, (vector3) {x - wo, wy, z}, (vector3) {BUCKET_WALL_T * 0.5f, BUCKET_WALL_H * 0.5f,
                                                                              BUCKET_PAD_HALF}, wall_c, roll_nx);
    ids [3] = bucket_add_static (world, (vector3) {x, wy, z + wo}, (vector3) {BUCKET_PAD_HALF, BUCKET_WALL_H * 0.5f,
                                                                              BUCKET_WALL_T * 0.5f}, wall_c, roll_pz);
    ids [4] = bucket_add_static (world, (vector3) {x, wy, z - wo}, (vector3) {BUCKET_PAD_HALF, BUCKET_WALL_H * 0.5f,
                                                                              BUCKET_WALL_T * 0.5f}, wall_c, roll_nz);
    ids [5] = -1; /* spare slot (corner posts, next step) */
    for (int i = 0; i < 5; i++) {
        if (ids [i] < 0) { return -1; }
    }
    for (int i = 0; i < 6; i++) { f -> bucket_part_ids [i] = ids [i]; }
    f -> bucket_active = 1;
    f -> bucket_cx = x;
    f -> bucket_cy = y;
    f -> bucket_cz = z;
    return 0;
}
int ftc_bucket_clear (struct physics_world *world) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f || !f -> bucket_active) { return -1; }
    for (int i = 0; i < 5; i++) {
        int bi = physics_world_index_by_id (world, (uint32_t) f -> bucket_part_ids [i]);
        if (bi >= 0 && bi < world -> body_count) { world -> bodies [bi].no_collide = 1; }
    }
    f -> bucket_active = 0;
    return 0;
}
int ftc_game_stats (struct physics_world *world, int *carried, int *balls, int *scored, int *bucket_on) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f) { return -1; }
    /* carried reads the CONTROLLED robot (selection), not robot 0. */
    int sel = (f -> count > 0) ? f -> selected : 0;
    if (sel < 0 || sel >= f -> count) { sel = 0; }
    ftc_robot *r = (f -> count > 0) ? fleet_robot_at (f, sel) : NULL;
    if (carried) { *carried = r ? r -> intake_count : 0; }
    if (balls) { *balls = f -> ball_count; }
    if (scored) { *scored = f -> bucket_scored; }
    if (bucket_on) { *bucket_on = f -> bucket_active; }
    return 0;
}
/* ---- BIOBUZZ regulation league verbs (fleet-resolved state) ---- */
int ftc_biobuzz_field (struct physics_world *world) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f) { return -1; }
    return biobuzz_build_field (world, &f -> bb);
}
int ftc_biobuzz_match (struct physics_world *world) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f) { return -1; }
    return biobuzz_stage_match (world, &f -> bb);
}
int ftc_biobuzz_preload (struct physics_world *world) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f) { return -1; }
    int chassis [32];
    int n = (f -> count < 32) ? f -> count : 32;
    for (int i = 0; i < n; i++) {
        ftc_robot *r = &f -> robots [i];
        chassis [i] = r->chassis_body;
    }
    return biobuzz_preload (world, &f -> bb, chassis, n);
}
int ftc_biobuzz_enter (struct physics_world *world, int alliance) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f) { return -1; }
    return biobuzz_enter_nectar (world, &f -> bb, alliance);
}
int ftc_biobuzz_score (struct physics_world *world, int alliance, biobuzz_alliance_score *out) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f || !out) { return -1; }
    int chassis [32];
    int n = (f -> count < 32) ? f -> count : 32;
    for (int i = 0; i < n; i++) { chassis [i] = f -> robots [i].chassis_body; }
    return biobuzz_score (world, &f -> bb, f -> bb.piece_ids, f -> bb.piece_count, f -> bb.piece_is_nectar,
                          f -> bb.piece_alliance, (biobuzz_alliance) alliance, chassis, n, out);
}
int ftc_biobuzz_place (struct physics_world *world, int index, float yaw) {
    ftc_fleet_t *f = fleet_of (world);
    ftc_robot *r = fleet_robot_at (f, index);
    if (!world || !r) { return -1; }
    biobuzz_place_robot (world, r, yaw);
    return 0;
}
int ftc_biobuzz_tips (struct physics_world *world, int *red, int *blue, int *unlocked_red, int *unlocked_blue) {    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f) { return -1; }
    if (red) { *red = f -> bb.tips [0]; }
    if (blue) { *blue = f -> bb.tips [1]; }
    if (unlocked_red) { *unlocked_red = f -> bb.unlocked [0]; }
    if (unlocked_blue) { *unlocked_blue = f -> bb.unlocked [1]; }
    return 0;
}
int ftc_biobuzz_hive (struct physics_world *world, int hive, int out_ids [11]) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f || !out_ids) { return -1; }
    return biobuzz_hive_ids (&f -> bb, hive, out_ids);
}
/* Per-tick game update: intake capture/carry per robot, ball aero, bucket
 * scoring. Same pre-integration tick position as motor forces. Inert when
 * nothing is staged (no balls/bucket/commands: a handful of NULL checks). */
static void ftc_game_update (struct physics_world *world, ftc_fleet_t *f, float dt) {
    (void) dt;
    if (!world || !f) { return; }
    /* Scene-clear self-heal: scene_clear / rm -rf / scene load replace the
     * body pool wholesale without telling the fleet (bodies belong to the
     * world, state belongs to us). If NOTHING tracked is alive anymore —
     * no live chassis, ball, bucket part, or BIOBUZZ piece — the whole
     * league is stale: forget it, same as ftc_fleet_clear. This covers
     * every present and future clear path with no UI call sites to keep in
     * sync (the menu/terminal paths below also clear synchronously for
     * the paused-stepping case). Sleeping/kinematic/parked bodies are
     * still indexed-alive, so settled matches never false-trigger; only
     * wholesale removal does. */
    {
        int live = 0;
        for (int i = 0; i < f -> count && !live; i++) {
            int bi = f -> robots [i].chassis_body;
            if (bi >= 0 && bi < world -> body_count) { live = 1; }
        }
        for (int i = 0; i < f -> ball_count && !live; i++) {
            if (physics_world_index_by_id (world, (uint32_t) f -> ball_ids [i]) >= 0) { live = 1; }
        }
        if (f -> bucket_active) {
            for (int i = 0; i < 5 && !live; i++) {
                if (physics_world_index_by_id (world, (uint32_t) f -> bucket_part_ids [i]) >= 0) { live = 1; }
            }
        }
        for (int i = 0; i < f -> bb.piece_count && !live; i++) {
            if (physics_world_index_by_id (world, (uint32_t) f -> bb.piece_ids [i]) >= 0) { live = 1; }
        }
        int bb_busy = (f -> bb.field_built || f -> bb.tips [0] || f -> bb.tips [1] || f -> bb.unlocked [0] ||
                       f -> bb.unlocked [1]);
        if (!live && (f -> count > 0 || f -> ball_count > 0 || f -> bucket_active || f -> bb.piece_count > 0 ||
                      bb_busy || f -> selected != 0)) {
            ftc_fleet_clear (world);
        }
    }
    for (int i = 0; i < f -> count; i++) {
        ftc_robot *r = &f -> robots [i];
        rigidbody *ch = fleet_chassis (world, r);
        if (!ch) { continue; }
        if (r -> intake_on || r -> intake_count > 0) {
            /* Build one deduplicated candidate list so ordinary balls and
             * BIOBUZZ pieces compete for the same four possession slots.
             * Preserve the source registries: they still own their scoring
             * and lifecycle metadata. */
            int intake_candidates [INTAKE_MAX_BALLS + BIOBUZZ_MAX_PIECES];
            int intake_candidate_n = 0;
            for (int k = 0; k < f -> ball_count && k < INTAKE_MAX_BALLS && intake_candidate_n < INTAKE_MAX_BALLS + BIOBUZZ_MAX_PIECES; k++) {
                int id = f -> ball_ids [k];
                if (id < 0) { continue; }
                int seen = 0;
                for (int j = 0; j < intake_candidate_n; j++) {
                    if (intake_candidates [j] == id) { seen = 1; break; }
                }
                if (!seen) { intake_candidates [intake_candidate_n++] = id; }
            }
            for (int k = 0; k < f -> bb.piece_count && k < BIOBUZZ_MAX_PIECES && intake_candidate_n < INTAKE_MAX_BALLS + BIOBUZZ_MAX_PIECES; k++) {
                int id = f -> bb.piece_ids [k];
                if (id < 0) { continue; }
                int seen = 0;
                for (int j = 0; j < intake_candidate_n; j++) {
                    if (intake_candidates [j] == id) { seen = 1; break; }
                }
                if (!seen) { intake_candidates [intake_candidate_n++] = id; }
            }
            /* Stable nearest-first ordering: intake_update captures in list
             * order, so registry insertion history no longer decides which
             * nearby pieces win when capacity is limited. */
            vector3 intake_mc;
            intake_mouth_pose (ch -> position, ch -> orientation, &intake_mc, NULL);
            for (int a = 0; a < intake_candidate_n; a++) {
                int best = a;
                float best_d2 = INFINITY;
                for (int b = a; b < intake_candidate_n; b++) {
                    int bi = physics_world_index_by_id (world, (uint32_t) intake_candidates [b]);
                    if (bi < 0 || bi >= world -> body_count) { continue; }
                    vector3 d = vector3_subtraction (world -> bodies [bi].position, intake_mc);
                    float d2 = vector3_length_squared (d);
                    if (isfinite (d2) && d2 < best_d2) { best_d2 = d2; best = b; }
                }
                if (best != a) {
                    int tmp = intake_candidates [a];
                    intake_candidates [a] = intake_candidates [best];
                    intake_candidates [best] = tmp;
                }
            }
            intake_update (world, ch -> position, ch -> orientation, ch -> velocity, r -> intake_on,
                           r -> intake_balls, &r -> intake_count, intake_candidates, intake_candidate_n);
            intake_carry_step (world, ch -> position, ch -> orientation, ch -> velocity, r -> intake_balls,
                               r -> intake_count);
        } else {
            intake_prune (world, r -> intake_balls, &r -> intake_count);
        }
    }
    if (f -> ball_count > 0) { intake_balls_aero_step (world, f -> ball_ids, f -> ball_count); }
    if (f -> bb.piece_count > 0) { intake_balls_aero_step (world, f -> bb.piece_ids, f -> bb.piece_count); }
    /* Prune dangling ball ids (scene cleared under us): compact live. */
    {
        int w = 0;
        for (int i = 0; i < f -> ball_count; i++) {
            int id = f -> ball_ids [i];
            if (id >= 0 && physics_world_index_by_id (world, (uint32_t) id) >= 0) { f -> ball_ids [w++] = id; }
        }
        for (int i = w; i < INTAKE_MAX_BALLS; i++) { f -> ball_ids [i] = -1; }
        f -> ball_count = w;
    }
    /* BIOBUZZ registry prune (parallel arrays compact together). */
    {
        int w = 0;
        for (int i = 0; i < f -> bb.piece_count; i++) {
            int id = f -> bb.piece_ids [i];
            if (id >= 0 && physics_world_index_by_id (world, (uint32_t) id) >= 0) {
                f -> bb.piece_ids [w] = id;
                f -> bb.piece_is_nectar [w] = f -> bb.piece_is_nectar [i];
                f -> bb.piece_alliance [w] = f -> bb.piece_alliance [i];
                w++;
            }
        }
        for (int i = w; i < BIOBUZZ_MAX_PIECES; i++) {
            f -> bb.piece_ids [i] = -1;
            f -> bb.piece_is_nectar [i] = 0;
            f -> bb.piece_alliance [i] = -1;
        }
        f -> bb.piece_count = w;
    }
    /* BIOBUZZ league tick: tip census (re-pose + spill + unlock). */
    biobuzz_tick (world, &f -> bb, f -> bb.piece_ids, f -> bb.piece_count, f -> bb.piece_is_nectar, dt);
    /* Bucket scoring: mouth-entry counting. Any tracked dynamic ball whose
     * centre enters the zone volume SCORES (no speed gate: a swish at speed
     * is still a basket, and dead walls (e=0) stop it anyway; the gate that
     * was here rejected valid line-drives). Scored balls freeze kinematic
     * (visible pile, stable, never re-scored since the candidate gate
     * requires dynamic). Carried balls are kinematic at the mouth, so
     * driving the hopper through the zone cannot false-score. */
    if (f -> bucket_active) {
        float hx = BUCKET_PAD_HALF - 0.1f, hz = BUCKET_PAD_HALF - 0.1f;
        float y0 = f -> bucket_cy + 0.02f, y1 = f -> bucket_cy + 0.05f + BUCKET_WALL_H + 0.05f;
        for (int i = 0; i < f -> ball_count; i++) {
            int bi = physics_world_index_by_id (world, (uint32_t) f -> ball_ids [i]);
            if (bi < 0 || bi >= world -> body_count) { continue; }
            rigidbody *ball = &world -> bodies [bi];
            if (ball -> kinematic || ball -> static_state) { continue; }
            vector3 d = vector3_subtraction (ball -> position, (vector3) {f -> bucket_cx, ball -> position.y,
                                                                          f -> bucket_cz});
            float ax = (d.x > 0.0f) ? d.x : -d.x;
            float az = (d.z > 0.0f) ? d.z : -d.z;
            if (ax > hx || az > hz) { continue; }
            if (ball -> position.y < y0 || ball -> position.y > y1) { continue; }
            rigidbody_set_kinematic (ball, true);
            ball -> velocity = vector3_zero ();
            ball -> angular_velocity = vector3_zero ();
            f -> bucket_scored++;
            fprintf (stderr, "[bucketball] SCORED #%d ball id=%d at (%.2f,%.2f,%.2f)\n", f -> bucket_scored,
                     f -> ball_ids [i], ball -> position.x, ball -> position.y, ball -> position.z);
        }
    }
}

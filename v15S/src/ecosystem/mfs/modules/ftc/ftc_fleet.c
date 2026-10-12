/* FTC robot fleet implementation. */
#include "ftc_fleet.h"
#include "submodules/drivetrain.h"
#include "submodules/intake.h"
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
int ftc_balls_spawn (struct physics_world *world, int n) {
    ftc_fleet_t *f = fleet_of (world);
    if (!world || !f) { return -1; }
    if (n < 1) { n = 3; }
    int made = 0;
    for (int i = 0; i < n && f -> ball_count < INTAKE_MAX_BALLS; i++) {
        /* Deterministic scatter ahead: three lanes (left/centre/right) so
         * a straight drive down the middle scoops at least the centre
         * lane through the 0.32 m mouth; tight rows so one pass catches
         * twice (the drive covers ~2.3 m). */
        float x = ((float) (i % 3) - 1.0f) * 0.3f;
        float z = 2.2f + 0.4f * (float) (i / 3);
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
    for (int k = 0; k < total && f -> ball_count < INTAKE_MAX_BALLS; k++) {
        int is_nectar = (k < nectar_n) ? 1 : 0;
        float radius = is_nectar ? BIOBUZZ_NECTAR_RADIUS : BIOBUZZ_POLLEN_RADIUS;
        float mass = is_nectar ? BIOBUZZ_NECTAR_MASS : BIOBUZZ_POLLEN_MASS;
        /* Three lanes (left/centre/right); nectar staged a touch further
         * than pollen so both kinds sit in the drive path and the intake
         * mouths a lane. */
        float x = ((float) (k % 3) - 1.0f) * 0.3f;
        float z = (is_nectar ? 2.6f : 1.8f) + 0.4f * (float) (k / 3);
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
    ftc_robot *r = fleet_robot_at (fleet_of (world), index);
    if (!world || !r) { return -1; }
    rigidbody *ch = fleet_chassis (world, r);
    if (!ch) { return -1; }
    int id = intake_shoot (world, ch -> position, ch -> orientation, ch -> velocity, r -> intake_balls,
                           &r -> intake_count);
    if (id >= 0) { fprintf (stderr, "[bucketball] fired ball id=%d\n", id); }
    return id;
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
    ftc_robot *r = fleet_robot_at (f, 0);
    if (carried) { *carried = r ? r -> intake_count : 0; }
    if (balls) { *balls = f -> ball_count; }
    if (scored) { *scored = f -> bucket_scored; }
    if (bucket_on) { *bucket_on = f -> bucket_active; }
    return 0;
}
/* Per-tick game update: intake capture/carry per robot, ball aero, bucket
 * scoring. Same pre-integration tick position as motor forces. Inert when
 * nothing is staged (no balls/bucket/commands: a handful of NULL checks). */
static void ftc_game_update (struct physics_world *world, ftc_fleet_t *f, float dt) {
    (void) dt;
    if (!world || !f) { return; }
    for (int i = 0; i < f -> count; i++) {
        ftc_robot *r = &f -> robots [i];
        rigidbody *ch = fleet_chassis (world, r);
        if (!ch) { continue; }
        if (r -> intake_on || r -> intake_count > 0) {
            intake_update (world, ch -> position, ch -> orientation, ch -> velocity, r -> intake_on,
                           r -> intake_balls, &r -> intake_count, f -> ball_ids, f -> ball_count);
            intake_carry_step (world, ch -> position, ch -> orientation, ch -> velocity, r -> intake_balls,
                               r -> intake_count);
        } else {
            intake_prune (world, r -> intake_balls, &r -> intake_count);
        }
    }
    if (f -> ball_count > 0) { intake_balls_aero_step (world, f -> ball_ids, f -> ball_count); }
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

/* MPE_FTC_063: per-world constraint pool (revolute, fixed, distance, prismatic, rope).
 * No file-scope pool remains; every entry point takes the owning world.
 * Solver entry points derive bodies from the world for the same reason. */
#include "constraint.h"
#include "revolute_joint.h"
#include "islands.h"
#include "../core/physics_world.h"
#include "../config/mpe_constants.h"

void constraint_pool_init (struct physics_world *world) {
    if (!world) { return; }
    for (int i = 0; i < mpe_max_joints; i++) { world->revolute_constraints [i].is_active = false; }
    world->revolute_constraint_count = 0;
}

int constraint_add_revolute (struct physics_world *world, uint32_t id_a, uint32_t id_b, vector3 anchor_a,
                             vector3 anchor_b, vector3 axis_a) {
    if (!world) { return -1; }
    if ((id_a == 0) || (id_b == 0) || (id_a == id_b)) { return -1; }
    /* FIX-AUDIT: zero hinge axis used to silently become a ball-joint lock
     * (normalise(0)=0 -> perpendicular = full relative spin killed). Reject. */
    if (vector3_length_squared(axis_a) < 1e-12f) { return -1; }
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!world->revolute_constraints [i].is_active) {
            world->revolute_constraints [i].type = constraint_revolute;
            world->revolute_constraints [i].body_id_a = id_a;
            world->revolute_constraints [i].body_id_b = id_b;
            world->revolute_constraints [i].p.revolute.anchor_a = anchor_a;
            world->revolute_constraints [i].p.revolute.anchor_b = anchor_b;
            world->revolute_constraints [i].p.revolute.axis_a = vector3_normalisation (axis_a);
            world->revolute_constraints [i].p.revolute.axis_b = vector3_normalisation (axis_a);
            world->revolute_constraints [i].p.revolute.motor_enabled = false;
            world->revolute_constraints [i].p.revolute.limits_enabled = false;
            world->revolute_constraints [i].p.revolute.motor_target_speed = 0.0f;
            world->revolute_constraints [i].p.revolute.motor_max_torque = 0.0f;
            world->revolute_constraints [i].p.revolute.limit_min_rad = 0.0f;
            world->revolute_constraints [i].p.revolute.limit_max_rad = 0.0f;
            world->revolute_constraints [i].p.revolute.accumulated_angle = 0.0f;
            world->revolute_constraints [i].p.revolute.angle_initialized = false;
            world->revolute_constraints [i].p.revolute.reference_axis_a = vector3_zero();
            world->revolute_constraints [i].p.revolute.reference_axis_b = vector3_zero();
            world->revolute_constraints [i].is_active = true;
            world->revolute_constraint_count++;
            return i;
        }
    }
    return -1;
}

void constraint_remove (struct physics_world *world, int index) {
    if (!world) { return; }
    if ((index < 0) || (index >= mpe_max_joints)) { return; }
    if (!world->revolute_constraints [index].is_active) { return; }
    world->revolute_constraints [index].is_active = false;
    world->revolute_constraint_count--;
}

int constraint_add_fixed(struct physics_world *world, uint32_t id_a, uint32_t id_b, vector3 anchor_a,
                         vector3 anchor_b) {
    if (!world) { return -1; }
    if ((id_a == 0) || (id_b == 0) || (id_a == id_b)) { return -1; }
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!world->revolute_constraints [i].is_active) {
            world->revolute_constraints [i].type = constraint_fixed;
            world->revolute_constraints [i].body_id_a = id_a;
            world->revolute_constraints [i].body_id_b = id_b;
            world->revolute_constraints [i].p.fixed.anchor_a = anchor_a;
            world->revolute_constraints [i].p.fixed.anchor_b = anchor_b;
            world->revolute_constraints [i].is_active = true;
            world->revolute_constraint_count++;
            return i;
        }
    }
    return -1;
}

int constraint_add_distance(struct physics_world *world, uint32_t id_a, uint32_t id_b, vector3 anchor_a,
                            vector3 anchor_b, float rest_length) {
    if (!world) { return -1; }
    if ((id_a == 0) || (id_b == 0) || (id_a == id_b)) { return -1; }
    if (!isfinite(rest_length) || rest_length < 0.0f) { return -1; }
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!world->revolute_constraints [i].is_active) {
            world->revolute_constraints [i].type = constraint_distance;
            world->revolute_constraints [i].body_id_a = id_a;
            world->revolute_constraints [i].body_id_b = id_b;
            world->revolute_constraints [i].p.distance.anchor_a = anchor_a;
            world->revolute_constraints [i].p.distance.anchor_b = anchor_b;
            world->revolute_constraints [i].p.distance.rest_length = rest_length;
            world->revolute_constraints [i].is_active = true;
            world->revolute_constraint_count++;
            return i;
        }
    }
    return -1;
}

/* Prismatic (slider) joint: single-axis slide. */
int constraint_add_prismatic(struct physics_world *world, uint32_t id_a, uint32_t id_b, vector3 anchor_a,
                             vector3 anchor_b, vector3 axis_a) {
    if (!world) { return -1; }
    if ((id_a == 0) || (id_b == 0) || (id_a == id_b)) { return -1; }
    if (vector3_length_squared(axis_a) < 1e-12f) { return -1; }
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!world->revolute_constraints [i].is_active) {
            world->revolute_constraints [i].type = constraint_prismatic;
            world->revolute_constraints [i].body_id_a = id_a;
            world->revolute_constraints [i].body_id_b = id_b;
            world->revolute_constraints [i].p.prismatic.anchor_a = anchor_a;
            world->revolute_constraints [i].p.prismatic.anchor_b = anchor_b;
            world->revolute_constraints [i].p.prismatic.axis_a = vector3_normalisation(axis_a);
            world->revolute_constraints [i].p.prismatic.axis_b = vector3_normalisation(axis_a);
            world->revolute_constraints [i].p.prismatic.limits_enabled = false;
            world->revolute_constraints [i].p.prismatic.limit_min = 0.0f;
            world->revolute_constraints [i].p.prismatic.limit_max = 0.0f;
            world->revolute_constraints [i].p.prismatic.motor_enabled = false;
            world->revolute_constraints [i].p.prismatic.motor_target_speed = 0.0f;
            world->revolute_constraints [i].p.prismatic.motor_max_force = 0.0f;
            world->revolute_constraints [i].p.prismatic.accumulated_position = 0.0f;
            world->revolute_constraints [i].p.prismatic.position_initialized = false;
            world->revolute_constraints [i].p.prismatic.reference_axis_a = vector3_zero();
            world->revolute_constraints [i].p.prismatic.reference_axis_b = vector3_zero();
            world->revolute_constraints [i].is_active = true;
            world->revolute_constraint_count++;
            return i;
        }
    }
    return -1;
}

/* Rope constraint: inequality distance (pulls only). */
int constraint_add_rope(struct physics_world *world, uint32_t id_a, uint32_t id_b, vector3 anchor_a,
                        vector3 anchor_b, float max_length) {
    if (!world) { return -1; }
    if ((id_a == 0) || (id_b == 0) || (id_a == id_b)) { return -1; }
    if (!isfinite(max_length) || max_length < 0.0f) { return -1; }
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!world->revolute_constraints [i].is_active) {
            world->revolute_constraints [i].type = constraint_rope;
            world->revolute_constraints [i].body_id_a = id_a;
            world->revolute_constraints [i].body_id_b = id_b;
            world->revolute_constraints [i].p.rope.anchor_a = anchor_a;
            world->revolute_constraints [i].p.rope.anchor_b = anchor_b;
            world->revolute_constraints [i].p.rope.rest_length = max_length;
            world->revolute_constraints [i].is_active = true;
            world->revolute_constraint_count++;
            return i;
        }
    }
    return -1;
}

int constraint_get_count (const struct physics_world *world) {
    if (!world) { return 0; }
    return world->revolute_constraint_count;
}

void constraint_set_revolute_motor (struct physics_world *world, int index, bool enabled, float target_speed,
                                    float max_torque) {
    if (!world) { return; }
    if ((index < 0) || (index >= mpe_max_joints)) { return; }
    if (!world->revolute_constraints [index].is_active) { return; }
    if (world->revolute_constraints [index].type != constraint_revolute) { return; }
    world->revolute_constraints [index].p.revolute.motor_enabled = enabled;
    world->revolute_constraints [index].p.revolute.motor_target_speed = target_speed;
    world->revolute_constraints [index].p.revolute.motor_max_torque = max_torque;
}

void constraint_set_revolute_axes (struct physics_world *world, int index, vector3 axis_a, vector3 axis_b) {
    if (!world) { return; }
    if ((index < 0) || (index >= mpe_max_joints)) { return; }
    if (!world->revolute_constraints [index].is_active) { return; }
    if (world->revolute_constraints [index].type != constraint_revolute) { return; }
    if (vector3_length_squared (axis_a) < 1e-12f) { return; }
    world->revolute_constraints [index].p.revolute.axis_a = vector3_normalisation (axis_a);
    /* Zero axis_b means "same as axis_a" (legacy callers). */
    world->revolute_constraints [index].p.revolute.axis_b = (vector3_length_squared (axis_b) > 1e-12f)
        ? vector3_normalisation (axis_b)
        : world->revolute_constraints [index].p.revolute.axis_a;
}

void constraint_set_revolute_limits (struct physics_world *world, int index, bool enabled, float limit_min_rad,
                                      float limit_max_rad) {
    if (!world) { return; }
    if ((index < 0) || (index >= mpe_max_joints)) { return; }
    if (!world->revolute_constraints [index].is_active) { return; }
    if (world->revolute_constraints [index].type != constraint_revolute) { return; }
    if ((!isfinite (limit_min_rad)) || (!isfinite (limit_max_rad))) { return; }
    world->revolute_constraints [index].p.revolute.limits_enabled = enabled;
    world->revolute_constraints [index].p.revolute.limit_min_rad =
        (limit_min_rad < limit_max_rad) ? limit_min_rad : limit_max_rad;
    world->revolute_constraints [index].p.revolute.limit_max_rad =
        (limit_min_rad < limit_max_rad) ? limit_max_rad : limit_min_rad;
    /* Reset angle tracking when limits are (re)enabled so initial configuration
     * establishes the reference frame. */
    if (enabled) {
        world->revolute_constraints [index].p.revolute.accumulated_angle = 0.0f;
        world->revolute_constraints [index].p.revolute.angle_initialized = false;
        world->revolute_constraints [index].p.revolute.reference_axis_a = vector3_zero();
        world->revolute_constraints [index].p.revolute.reference_axis_b = vector3_zero();
    }
}

/* Prismatic joint setters */
void constraint_set_prismatic_axes (struct physics_world *world, int index, vector3 axis_a, vector3 axis_b) {
    if (!world) { return; }
    if ((index < 0) || (index >= mpe_max_joints)) { return; }
    if (!world->revolute_constraints [index].is_active) { return; }
    if (world->revolute_constraints [index].type != constraint_prismatic) { return; }
    if (vector3_length_squared (axis_a) < 1e-12f) { return; }
    world->revolute_constraints [index].p.prismatic.axis_a = vector3_normalisation (axis_a);
    world->revolute_constraints [index].p.prismatic.axis_b = (vector3_length_squared (axis_b) > 1e-12f)
        ? vector3_normalisation (axis_b)
        : world->revolute_constraints [index].p.prismatic.axis_a;
    /* Reset position tracking when axes change. */
    world->revolute_constraints [index].p.prismatic.accumulated_position = 0.0f;
    world->revolute_constraints [index].p.prismatic.position_initialized = false;
    world->revolute_constraints [index].p.prismatic.reference_axis_a = vector3_zero();
    world->revolute_constraints [index].p.prismatic.reference_axis_b = vector3_zero();
}

void constraint_set_prismatic_limits (struct physics_world *world, int index, bool enabled, float limit_min, float limit_max) {
    if (!world) { return; }
    if ((index < 0) || (index >= mpe_max_joints)) { return; }
    if (!world->revolute_constraints [index].is_active) { return; }
    if (world->revolute_constraints [index].type != constraint_prismatic) { return; }
    if ((!isfinite (limit_min)) || (!isfinite (limit_max))) { return; }
    world->revolute_constraints [index].p.prismatic.limits_enabled = enabled;
    world->revolute_constraints [index].p.prismatic.limit_min = (limit_min < limit_max) ? limit_min : limit_max;
    world->revolute_constraints [index].p.prismatic.limit_max = (limit_min < limit_max) ? limit_max : limit_min;
    if (enabled) {
        world->revolute_constraints [index].p.prismatic.accumulated_position = 0.0f;
        world->revolute_constraints [index].p.prismatic.position_initialized = false;
        world->revolute_constraints [index].p.prismatic.reference_axis_a = vector3_zero();
        world->revolute_constraints [index].p.prismatic.reference_axis_b = vector3_zero();
    }
}

void constraint_set_prismatic_motor (struct physics_world *world, int index, bool enabled, float target_speed, float max_force) {
    if (!world) { return; }
    if ((index < 0) || (index >= mpe_max_joints)) { return; }
    if (!world->revolute_constraints [index].is_active) { return; }
    if (world->revolute_constraints [index].type != constraint_prismatic) { return; }
    world->revolute_constraints [index].p.prismatic.motor_enabled = enabled;
    world->revolute_constraints [index].p.prismatic.motor_target_speed = target_speed;
    world->revolute_constraints [index].p.prismatic.motor_max_force = max_force;
}

int constraint_pool_capacity (void) { return mpe_max_joints; }

const constraint *constraint_pool_at (const struct physics_world *world, int index) {
    if (!world) { return NULL; }
    if ((index < 0) || (index >= mpe_max_joints)) { return NULL; }
    if (!world->revolute_constraints [index].is_active) { return NULL; }
    return &world->revolute_constraints [index];
}

static rigidbody *find_body_by_id (rigidbody *bodies, int body_count, uint32_t id) {
    if (!bodies) { return NULL; }
    for (int i = 0; i < body_count; i++) {
        if (bodies [i].object_id == id) { return &bodies [i]; }
    }
    return NULL;
}

/* TRUTH: per-tick id->index cache avoids O(J*B) linear scan per iteration
 * (1024*16384*64 worst). Rebuilt once per tick in pre_step; solve reuses. */
#define CONSTRAINT_ID_CACHE_SIZE 2048
static uint32_t s_id_cache_keys[CONSTRAINT_ID_CACHE_SIZE];
static int s_id_cache_vals[CONSTRAINT_ID_CACHE_SIZE];
static unsigned char s_id_cache_valid[CONSTRAINT_ID_CACHE_SIZE];
static int s_id_cache_body_count = -1;
static uint32_t s_id_cache_first_id = 0;
static bool s_id_cache_ready = false;

static void constraint_id_cache_rebuild(rigidbody *bodies, int body_count) {
    for (int i = 0; i < CONSTRAINT_ID_CACHE_SIZE; i++) {
        s_id_cache_valid[i] = 0;
    }
    if (!bodies || body_count <= 0) {
        s_id_cache_ready = false;
        return;
    }
    for (int i = 0; i < body_count; i++) {
        uint32_t id = bodies[i].object_id;
        if (id == 0 || id == 0xFFFFFFFFu) {
            continue;
        }
        uint32_t h = (id * 2654435761u) & (CONSTRAINT_ID_CACHE_SIZE - 1);
        for (int probe = 0; probe < 32; probe++) {
            uint32_t s = (h + (uint32_t) probe) & (CONSTRAINT_ID_CACHE_SIZE - 1);
            if (!s_id_cache_valid[s]) {
                s_id_cache_keys[s] = id;
                s_id_cache_vals[s] = i;
                s_id_cache_valid[s] = 1;
                break;
            }
        }
    }
    s_id_cache_body_count = body_count;
    s_id_cache_first_id = bodies[0].object_id;
    s_id_cache_ready = true;
}

static rigidbody *find_body_by_id_cached(rigidbody *bodies, int body_count, uint32_t id) {
    if (!bodies || body_count <= 0 || id == 0) {
        return NULL;
    }
    if (!s_id_cache_ready || s_id_cache_body_count != body_count ||
        (body_count > 0 && bodies[0].object_id != s_id_cache_first_id)) {
        constraint_id_cache_rebuild(bodies, body_count);
    }
    if (s_id_cache_ready) {
        uint32_t h = (id * 2654435761u) & (CONSTRAINT_ID_CACHE_SIZE - 1);
        for (int probe = 0; probe < 32; probe++) {
            uint32_t s = (h + (uint32_t) probe) & (CONSTRAINT_ID_CACHE_SIZE - 1);
            if (!s_id_cache_valid[s]) {
                break;
            }
            if (s_id_cache_keys[s] == id) {
                int idx = s_id_cache_vals[s];
                if (idx >= 0 && idx < body_count && bodies[idx].object_id == id) {
                    return &bodies[idx];
                }
                break;
            }
        }
    }
    return find_body_by_id(bodies, body_count, id);
}

static void constraint_dispatch (struct physics_world *world, float dt, bool motors_pass) {
    if ((!world) || (!world->bodies) || (world->body_count <= 0) || (world->revolute_constraint_count <= 0)) {
        return;
    }
    if (!(dt > 0.0f) || !isfinite(dt)) {
        return;
    }
    rigidbody *bodies = world->bodies;
    int body_count = world->body_count;
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!world->revolute_constraints [i].is_active) { continue; }
        constraint *c = &world->revolute_constraints [i];
        rigidbody *body_a = find_body_by_id_cached (bodies, body_count, c->body_id_a);
        rigidbody *body_b = find_body_by_id_cached (bodies, body_count, c->body_id_b);
        if ((!body_a) || (!body_b)) { continue; }
        /* Island skip (solve pass only): a fully-sleeping island's joint
         * solve is a no-op. Motors always apply (pre-integration drive),
         * and an ENABLED motor keeps its bodies awake: motor torque is
         * written straight to the accumulator (no force-wake trip), so a
         * sleeping robot would otherwise never hear its own drive. */
        if (motors_pass) {
            if ((c->type == constraint_revolute) && (c->p.revolute.motor_enabled)) {
                rigidbody_wake (body_a);
                rigidbody_wake (body_b);
            }
            if ((c->type == constraint_prismatic) && (c->p.prismatic.motor_enabled)) {
                rigidbody_wake (body_a);
                rigidbody_wake (body_b);
            }
        } else {
            if ((!islands_body_awake (world, body_a)) && (!islands_body_awake (world, body_b))) { continue; }
        }
        if (c->type == constraint_revolute) {
            if (motors_pass) { revolute_apply_motor (&c->p.revolute, body_a, body_b, dt); }
            else { revolute_solve (&c->p.revolute, body_a, body_b, dt); }
        } else if (c->type == constraint_fixed) {
            if (!motors_pass) { fixed_solve (&c->p.fixed, body_a, body_b, dt); }
        } else if (c->type == constraint_distance) {
            if (!motors_pass) { distance_solve (&c->p.distance, body_a, body_b, dt); }
        } else if (c->type == constraint_prismatic) {
            if (motors_pass) { prismatic_apply_motor (&c->p.prismatic, body_a, body_b, dt); }
            else { prismatic_solve (&c->p.prismatic, body_a, body_b, dt); }
        } else if (c->type == constraint_rope) {
            if (!motors_pass) { rope_solve (&c->p.rope, body_a, body_b, dt); }
        }
    }
}

void constraint_solve_all (struct physics_world *world, float dt) {
    if (!(dt > 0.0f)) { return; }
    constraint_dispatch (world, dt, false);
}

void constraint_pre_step_all(struct physics_world *world, float dt) {
    if ((!world) || (!world->bodies) || (world->body_count <= 0) || (!(dt > 0.0f))) {
        return;
    }
    constraint_id_cache_rebuild(world->bodies, world->body_count);
    if (world->revolute_constraint_count <= 0) {
        return;
    }
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!world->revolute_constraints[i].is_active) {
            continue;
        }
        constraint *c = &world->revolute_constraints[i];
        rigidbody *ba = find_body_by_id_cached(world->bodies, world->body_count, c->body_id_a);
        rigidbody *bb = find_body_by_id_cached(world->bodies, world->body_count, c->body_id_b);
        if ((!ba) || (!bb)) {
            continue;
        }
        if (c->type == constraint_revolute) {
            revolute_pre_step(&c->p.revolute, ba, bb, dt);
        } else if (c->type == constraint_prismatic) {
            prismatic_pre_step(&c->p.prismatic, ba, bb, dt);
        }
    }
}

int constraint_get_active_ids (const struct physics_world *world, uint32_t *ids_a, uint32_t *ids_b, int capacity) {
    if ((!world) || (!ids_a) || (!ids_b) || (capacity <= 0)) { return 0; }
    int count = 0;
    for (int i = 0; (i < mpe_max_joints) && (count < capacity); i++) {
        if (!world->revolute_constraints [i].is_active) { continue; }
        ids_a[count] = world->revolute_constraints [i].body_id_a;
        ids_b[count] = world->revolute_constraints [i].body_id_b;
        count++;
    }
    return count;
}

void constraint_correct_axis_drift_all (struct physics_world *world, float dt) {
    if ((!world) || (!world->bodies) || (world->body_count <= 0) || (!(dt > 0.0f)) ||
        (world->revolute_constraint_count <= 0)) { return; }
    rigidbody *bodies = world->bodies;
    int body_count = world->body_count;
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!world->revolute_constraints [i].is_active) { continue; }
        constraint *c = &world->revolute_constraints [i];
        rigidbody *body_a = find_body_by_id_cached (bodies, body_count, c->body_id_a);
        rigidbody *body_b = find_body_by_id_cached (bodies, body_count, c->body_id_b);
        if ((!body_a) || (!body_b)) { continue; }
        if ((!islands_body_awake (world, body_a)) && (!islands_body_awake (world, body_b))) { continue; }
        if (c->type == constraint_revolute) {
            revolute_correct_axis_drift (&c->p.revolute, body_a, body_b, dt);
        } else if (c->type == constraint_fixed) {
            /* TRUTH: weld angular drift corrected once per tick (see header). */
            fixed_correct_angular_drift (&c->p.fixed, body_a, body_b, dt);
        }
    }
}

void constraint_apply_motors (struct physics_world *world, float dt) {
    if (!(dt > 0.0f)) { return; }
    constraint_dispatch (world, dt, true);
}

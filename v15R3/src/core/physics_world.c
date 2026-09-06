/* MPE_FTC_059C: physics world — full pipeline.
 * Supersedes MPE_FTC_056 (free-body step).
 * Pipeline mirrors the legacy loop in simulation.c, minus:
 *   - sleep staticize hack (sleeping bodies keep real mass here),
 *   - positional depenetration pass,
 *   - joints/constraints (Phase 1).
 * NOTE: the contact warm-start cache is still engine-global. Worlds
 * must seed non-overlapping object_id ranges (see tests/two_world_test.c).
 * A per-world cache is tracked as future work.
 */
#include "physics_world.h"
#include "../physics/collision_mechanics.h"
#include "../physics/broadphase.h"
#include "../physics/constraint.h" /* MPE_FTC_067 */
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include <stdlib.h>
#include <math.h>
#include <string.h> /* MPE_FTC_076a */

static physics_world g_physics_world = {.bodies = NULL, .body_count = 0, .body_capacity = 0, .next_object_id = 1};

static broadphase_pair world_pairs[mpe_max_broadphase_pairs];
static collision_data world_manifolds[a3_max_manifolds];

void physics_world_init(physics_world *world) {
    if (!world) {
        return;
    }
    memset(world, 0, sizeof(physics_world)); /* MPE_FTC_076a */
    if (!world->bodies) {
        world->bodies = (rigidbody *) malloc((size_t) mpe_max_bodies * sizeof(rigidbody));
        world->body_capacity = mpe_max_bodies;
    }
    if (!world->world_contact_cache) {
        world->world_contact_cache =
            (cached_contact *) malloc((size_t) max_cached_contacts * sizeof(cached_contact)); /* MFS_131A */
    }
    world->body_count = 0;
    if (world->next_object_id == 0) {
        world->next_object_id = 1;
    }
}

void physics_world_cleanup(physics_world *world) {
    if (!world) {
        return;
    }
    if (world->bodies) {
        free(world->bodies);
        world->bodies = NULL;
    }
    if (world->world_contact_cache) {
        free(world->world_contact_cache); /* MFS_131A */
        world->world_contact_cache = NULL;
    }
    world->world_contact_cache_count = 0;
    world->body_count = 0;
    world->body_capacity = 0;
}

int physics_world_add_sphere(physics_world *world, float radius, float mass, vector3 position) {
    if ((!world) || (!world->bodies) || (world->body_count >= world->body_capacity)) {
        return -1;
    }
    rigidbody *rb = &world->bodies[world->body_count];
    rigidbody_initialisation_sphere(rb, radius, mass, position);
    rb->object_id = world->next_object_id++;
    rb->object_generation = 1;
    rigidbody_sanitize(rb);
    return world->body_count++;
}

int physics_world_add_cube(physics_world *world, vector3 position, vector3 half_extensions, float mass) {
    if ((!world) || (!world->bodies) || (world->body_count >= world->body_capacity)) {
        return -1;
    }
    rigidbody *rb = &world->bodies[world->body_count];
    rigidbody_initialisation_cube(rb, position, half_extensions, mass);
    rb->object_id = world->next_object_id++;
    rb->object_generation = 1;
    rigidbody_sanitize(rb);
    return world->body_count++;
}

/* MPE_FTC_091 */
int physics_world_add_cylinder(physics_world *world, float radius, float half_length, float mass,
                             vector3 position) {
    if ((!world) || (!world->bodies) || (world->body_count >= world->body_capacity)) {
        return -1;
    }
    rigidbody *rb = &world->bodies[world->body_count];
    rigidbody_initialisation_cylinder(rb, radius, half_length, mass, position);
    rb->object_id = world->next_object_id++;
    rb->object_generation = 1;
    rigidbody_sanitize(rb);
    return world->body_count++;
}

void physics_world_clear(physics_world *world) {
    if (!world) {
        return;
    }
    world->body_count = 0;
    world->world_contact_cache_count = 0; /* MFS_131A */
}

void physics_world_step(physics_world *world, float dt) {
    if ((!world) || (!world->bodies) || (dt <= 0.0f) || (world->body_count <= 0)) {
        return;
    }

    for (int i = 0; i < world->body_count; i++) {
        rigidbody_sanitize(&world->bodies[i]);
    }

    int pair_count = 0;
    if (world->body_count >= 2) {
        pair_count =
            broadphase_generate_pairing(world->bodies, world->body_count, world_pairs, mpe_max_broadphase_pairs);
    }

    int manifold_count = 0;
    for (int p = 0; p < pair_count; p++) {
        int index_a = world_pairs[p].object_index_a;
        int index_b = world_pairs[p].object_index_b;
        if ((index_a < 0) || (index_a >= world->body_count)) {
            continue;
        }
        if ((index_b < 0) || (index_b >= world->body_count)) {
            continue;
        }
        rigidbody *body_a = &world->bodies[index_a];
        rigidbody *body_b = &world->bodies[index_b];
        if ((body_a->is_sleeping) && (body_b->is_sleeping)) {
            continue;
        }
        collision_data narrowphase_collision = {0};
        bool collided = false;
        if ((body_a->type == object_sphere) && (body_b->type == object_sphere)) {
            collided = collision_dual_sphere(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_sphere) && (body_b->type == object_cube)) {
            collided = collision_sphere_cube(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_cube) && (body_b->type == object_sphere)) {
            collided = collision_sphere_cube(body_b, body_a, &narrowphase_collision);
            if (collided) {
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            }
        } else if ((body_a->type == object_cube) && (body_b->type == object_cube)) {
            collided = collision_dual_cube(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_cylinder) && (body_b->type == object_sphere)) { /* MFS_173C_REPAIRED */
            collided = collision_cylinder_sphere(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_sphere) && (body_b->type == object_cylinder)) {
            collided = collision_cylinder_sphere(body_b, body_a, &narrowphase_collision);
            if (collided) {
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            }
        } else if ((body_a->type == object_cylinder) && (body_b->type == object_cube)) {
            collided = collision_cylinder_cube(body_a, body_b, &narrowphase_collision);
        } else if ((body_a->type == object_cube) && (body_b->type == object_cylinder)) {
            collided = collision_cylinder_cube(body_b, body_a, &narrowphase_collision);
            if (collided) {
                narrowphase_collision.normal_vector = vector3_scaling(narrowphase_collision.normal_vector, -1.0f);
                narrowphase_collision.object_a = body_a;
                narrowphase_collision.object_b = body_b;
            }
        } else if ((body_a->type == object_cylinder) && (body_b->type == object_cylinder)) {
            collided = collision_cylinder_cylinder(body_a, body_b, &narrowphase_collision);
        } if ((collided) && (manifold_count < a3_max_manifolds)) {
            /* LIST4 R3-06: wake-on-contact for the physics_world path.
             *
             * The legacy path already had this. The encapsulated path did not,
             * which allowed sleeping bodies to remain frozen while active bodies
             * collided with them and silently injected velocity.
             */
            bool a_was_sleeping = body_a->is_sleeping;
            bool b_was_sleeping = body_b->is_sleeping;

            if ((a_was_sleeping) && (b_was_sleeping)) {
                /* Both sleeping: nothing to wake. */
            } else {
                float wake_linear_threshold_sq = g_cfg.sleep.wake_linear_thresh_sq;
                float wake_angular_threshold_sq = g_cfg.sleep.wake_angular_thresh_sq;

                bool a_is_active = (!a_was_sleeping) &&
                    ((vector3_length_squared(body_a->velocity) > wake_linear_threshold_sq) ||
                     (vector3_length_squared(body_a->angular_velocity) > wake_angular_threshold_sq));

                bool b_is_active = (!b_was_sleeping) &&
                    ((vector3_length_squared(body_b->velocity) > wake_linear_threshold_sq) ||
                     (vector3_length_squared(body_b->angular_velocity) > wake_angular_threshold_sq));

                if ((a_was_sleeping) && (!body_b->static_state) && (b_is_active)) {
                    rigidbody_wake(body_a);
                }

                if ((b_was_sleeping) && (!body_a->static_state) && (a_is_active)) {
                    rigidbody_wake(body_b);
                }

                collision_prepare_solver(&narrowphase_collision, &world_manifolds[manifold_count]);
                manifold_count++;
            }
        }
    }

    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];
        if ((rb->static_state) || (rb->is_sleeping)) {
            continue;
        }
        collision_data floor_collision = {0};
        if ((collision_static_plane_body(rb, 0.0f, &floor_collision)) && (manifold_count < a3_max_manifolds)) {
            collision_prepare_solver(&floor_collision, &world_manifolds[manifold_count]);
            manifold_count++;
        }
    }

    vector3 gravity = {0.0f, g_cfg.world.gravity, 0.0f};
    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];
        if ((rb->static_state) || (rb->is_sleeping)) {
            continue;
        }
        rb_apply_forces_perfect(rb, vector3_scaling(gravity, rb->mass));
    }

    float linear_damping = powf(g_cfg.world.drag, dt);
    float angular_damping = powf(g_cfg.world.drag * 0.97f, dt);
    constraint_apply_motors(world->bodies, world->body_count, dt); /* MPE_FTC_067 */
    for (int i = 0; i < world->body_count; i++) {
        rb_integrate_velocity(&world->bodies[i], dt, linear_damping, angular_damping);
    }

    int solver_iterations = g_cfg.timestep.solver_iterations;
    for (int iter = 0; iter < solver_iterations; iter++) {
        for (int m = 0; m < manifold_count; m++) {
            collision_resolve_iterative(&world_manifolds[m]);
        }
        /* MFS_SOLVER_FIX: solve joints inside the iteration loop so friction
         * impulses properly transfer through revolute constraints to the chassis */
        constraint_solve_all(world->bodies, world->body_count, dt);
    }
    /* MFS_150_WHEEL_LOCK: gearbox back-drive friction locks stationary wheels.
     * After the contact solver, any mecanum wheel with small axle omega is
     * locked to prevent the idle spin from the contact solver injecting
     * angular momentum. This is truthful: a real unpowered mecanum wheel
     * can't spin freely because the gearbox resists back-driving. */
    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];
        if (rb->is_mecanum && !rb->driven_this_tick) { /* MFS_169 */
            vector3 axle = rb->cached_axes[0];
            if (vector3_length_squared(axle) < 0.0001f) {
                axle = vector4_rotate_to_vector3(rb->orientation, (vector3){1.0f, 0.0f, 0.0f});
            }
            float axle_omega = vector3_dot(rb->angular_velocity, axle);
            if (fabsf(axle_omega) < g_cfg.solver.wheel_lock_omega_thresh) { /* MFS_166_WHEEL_LOCK_CFG */
                rb->angular_velocity = vector3_subtraction(
                    rb->angular_velocity,
                    vector3_scaling(axle, axle_omega));
            }
        }
    }

    contact_cache_save(world, world_manifolds, manifold_count);

/* MFS_169: clear driven flag at end of step */
for (int i = 0; i < world->body_count; i++) {
world->bodies[i].driven_this_tick = false;
} /* MFS_131 */

    for (int i = 0; i < world->body_count; i++) {
        rb_integrate_position(&world->bodies[i], dt);
        rigidbody_sanitize(&world->bodies[i]);
    }
}

physics_world *physics_world_get_primary(void) {
    return &g_physics_world;
}

/* R3-07: Containment walls.
 *
 * Adds four static cube bodies around the playable area.
 * The walls are placed just outside the half-extents so the
 * playable interior is exactly half_width x half_depth.
 *
 * Wall layout (top view):
 *
 *        north wall
 *   +-----------------+
 *   |                 |
 * w |    playable     | e
 * e |     area        | a
 * s |                 | s
 * t |                 | t
 *   +-----------------+
 *        south wall
 */
int physics_world_add_boundary_walls(physics_world *world,
                                     float half_width,
                                     float half_depth,
                                     float wall_height,
                                     float wall_thickness)
{
    if (!world) {
        return -1;
    }
    if ((half_width <= 0.0f) || (half_depth <= 0.0f) ||
        (wall_height <= 0.0f) || (wall_thickness <= 0.0f)) {
        return -1;
    }

    float hy = wall_height * 0.5f;
    float ht = wall_thickness * 0.5f;

    /* North wall: +Z side */
    int north = physics_world_add_cube(world,
        (vector3){0.0f, hy, half_depth + ht},
        (vector3){half_width + wall_thickness, hy, ht},
        0.0f);

    /* South wall: -Z side */
    int south = physics_world_add_cube(world,
        (vector3){0.0f, hy, -(half_depth + ht)},
        (vector3){half_width + wall_thickness, hy, ht},
        0.0f);

    /* East wall: +X side */
    int east = physics_world_add_cube(world,
        (vector3){half_width + ht, hy, 0.0f},
        (vector3){ht, hy, half_depth + wall_thickness},
        0.0f);

    /* West wall: -X side */
    int west = physics_world_add_cube(world,
        (vector3){-(half_width + ht), hy, 0.0f},
        (vector3){ht, hy, half_depth + wall_thickness},
        0.0f);

    if ((north < 0) || (south < 0) || (east < 0) || (west < 0)) {
        return -1;
    }

    return 0;
}

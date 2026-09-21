/* Solver islands: union-find over body indices, rebuilt every tick.
 * See islands.h for the contract. */
#include "islands.h"
#include "constraint.h"
#include "../core/physics_world.h"
#include "../config/mpe_constants.h"
#include <stddef.h>
#include <stdint.h>

/* Union-find scratch lives in the world (heap members); the module keeps
 * no state of its own. Queries take the world explicitly. */

static int island_find(struct physics_world *world, int x) {
    int root = x;
    while (world->island_parent[root] != root) {
        root = world->island_parent[root];
    }
    while (world->island_parent[x] != root) {
        int next = world->island_parent[x];
        world->island_parent[x] = root;
        x = next;
    }
    return root;
}

static void island_union(struct physics_world *world, int a, int b) {
    int ra = island_find(world, a);
    int rb = island_find(world, b);
    if (ra == rb) {
        return;
    }
    /* Union by smaller root: deterministic, no rank state. */
    if (ra < rb) {
        world->island_parent[rb] = ra;
    } else {
        world->island_parent[ra] = rb;
    }
}

static int island_index_of(rigidbody *bodies, rigidbody *body, int body_count) {
    if ((!bodies) || (!body) || (body_count <= 0)) {
        return -1;
    }
    int idx = body->body_index;
    if (idx < 0 || idx >= body_count) {
        return -1;
    }
    /* Paranoia: verify round-trip (catches stale indices). */
    if (&bodies[idx] != body) {
        return -1;
    }
    return idx;
}

static bool islands_ready(const struct physics_world *world) {
    return (world) && (world->bodies) && (world->island_parent) && (world->island_label) &&
           (world->island_awake_flags);
}

void islands_build(struct physics_world *world, broadphase_pair *pairs, int pair_count) {
    if (!islands_ready(world)) {
        return;
    }
    rigidbody *bodies = world->bodies;
    int body_count = world->body_count;
    world->island_base = bodies;
    world->island_body_count = 0;
    world->island_total = 0;
    if ((!bodies) || (body_count <= 0)) {
        return;
    }
    if (body_count > mpe_max_bodies) {
        body_count = mpe_max_bodies;
    }
    world->island_body_count = body_count;
    for (int i = 0; i < body_count; i++) {
        world->island_parent[i] = i;
        world->island_label[i] = -1;
    }
    if ((pairs) && (pair_count > 0)) {
        for (int p = 0; p < pair_count; p++) {
            int a = pairs[p].object_index_a;
            int b = pairs[p].object_index_b;
            if ((a < 0) || (a >= body_count) || (b < 0) || (b >= body_count) || (a == b)) {
                continue;
            }
            /* TRUTH: don't merge via both-sleeping pairs (solver skips them).
             * Old code unioned every broadphase pair incl. bounding-sphere
             * false positives, keeping giant false islands awake. */
            bool a_sleep = bodies[a].is_sleeping;
            bool b_sleep = bodies[b].is_sleeping;
            if (a_sleep && b_sleep) {
                continue;
            }
            island_union(world, a, b);
        }
    }
    /* Joints join islands. O(J) via the world's id->index cache
     * (verified + linear fallback inside); the old O(J*B) nested scan
     * stalled joint-heavy scenes. Covers every constraint type in the
     * shared pool (revolute/fixed/distance/prismatic/rope) plus springs:
     * a spring-connected sleeping pair must share an island or the sleep
     * gate and the spring force disagree for a tick. */
    {
        uint32_t ids_a[mpe_max_joints];
        uint32_t ids_b[mpe_max_joints];
        int joints = constraint_get_active_ids(world, ids_a, ids_b, mpe_max_joints);
        for (int j = 0; j < joints; j++) {
            int ia = physics_world_index_by_id(world, ids_a[j]);
            int ib = physics_world_index_by_id(world, ids_b[j]);
            if ((ia >= 0) && (ib >= 0) && (ia != ib)) {
                island_union(world, ia, ib);
            }
        }
        for (int j = 0; j < mpe_max_joints; j++) {
            if (!world->spring_joints[j].is_active) {
                continue;
            }
            int ia = physics_world_index_by_id(world, world->spring_joints[j].object_id_a);
            int ib = physics_world_index_by_id(world, world->spring_joints[j].object_id_b);
            if ((ia >= 0) && (ib >= 0) && (ia != ib)) {
                island_union(world, ia, ib);
            }
        }
    }
    /* Labels in first-seen order; awake if any dynamic member is awake. */
    for (int i = 0; i < body_count; i++) {
        int root = island_find(world, i);
        if (world->island_label[root] < 0) {
            world->island_label[root] = world->island_total;
            world->island_awake_flags[world->island_total] = 0;
            world->island_total++;
        }
        world->island_label[i] = world->island_label[root];
        if ((!bodies[i].static_state) && (!bodies[i].is_sleeping)) {
            world->island_awake_flags[world->island_label[i]] = 1;
        }
    }
}

int islands_count(const struct physics_world *world) {
    if (!world) {
        return 0;
    }
    return world->island_total;
}

int islands_body_island(struct physics_world *world, rigidbody *body) {
    if (!islands_ready(world)) {
        return -1;
    }
    if (world->bodies != world->island_base) {
        return -1;
    }
    int idx = island_index_of(world->bodies, body, world->island_body_count);
    if (idx < 0) {
        return -1;
    }
    return world->island_label[idx];
}

bool islands_body_awake(struct physics_world *world, rigidbody *body) {
    int island = islands_body_island(world, body);
    if (island < 0 || island >= world->island_total) {
        return true;
    }
    return world->island_awake_flags[island] != 0;
}

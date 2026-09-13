/* Solver islands: union-find over body indices, rebuilt every tick.
 * See islands.h for the contract. */
#include "islands.h"
#include "constraint.h"
#include "../config/mpe_constants.h"
#include <stddef.h>
#include <stdint.h>

static int island_parent[mpe_max_bodies];
static int island_label[mpe_max_bodies];
static unsigned char island_awake[16384];
static rigidbody *island_base = NULL;
static int island_body_count = 0;
static int island_total = 0;

static int island_find(int x) {
    int root = x;
    while (island_parent[root] != root) {
        root = island_parent[root];
    }
    while (island_parent[x] != root) {
        int next = island_parent[x];
        island_parent[x] = root;
        x = next;
    }
    return root;
}

static void island_union(int a, int b) {
    int ra = island_find(a);
    int rb = island_find(b);
    if (ra == rb) {
        return;
    }
    /* Union by smaller root: deterministic, no rank state. */
    if (ra < rb) {
        island_parent[rb] = ra;
    } else {
        island_parent[ra] = rb;
    }
}

static int island_index_of(rigidbody *bodies, rigidbody *body, int body_count) {
    if ((!bodies) || (!body) || (body_count <= 0)) {
        return -1;
    }
    ptrdiff_t offset = body - bodies;
    if ((offset < 0) || (offset >= body_count)) {
        return -1;
    }
    return (int) offset;
}

void islands_build(rigidbody *bodies, int body_count, broadphase_pair *pairs, int pair_count) {
    island_base = bodies;
    island_body_count = 0;
    island_total = 0;
    if ((!bodies) || (body_count <= 0)) {
        return;
    }
    if (body_count > mpe_max_bodies) {
        body_count = mpe_max_bodies;
    }
    island_body_count = body_count;
    for (int i = 0; i < body_count; i++) {
        island_parent[i] = i;
        island_label[i] = -1;
    }
    if ((pairs) && (pair_count > 0)) {
        for (int p = 0; p < pair_count; p++) {
            int a = pairs[p].object_index_a;
            int b = pairs[p].object_index_b;
            if ((a < 0) || (a >= body_count) || (b < 0) || (b >= body_count) || (a == b)) {
                continue;
            }
            island_union(a, b);
        }
    }
    /* Revolute joints join islands (wheels must solve with their chassis). */
    {
        uint32_t ids_a[mpe_max_joints];
        uint32_t ids_b[mpe_max_joints];
        int joints = constraint_get_active_ids(ids_a, ids_b, mpe_max_joints);
        for (int j = 0; j < joints; j++) {
            int ia = -1, ib = -1;
            for (int i = 0; i < body_count; i++) {
                if (bodies[i].object_id == ids_a[j]) {
                    ia = i;
                }
                if (bodies[i].object_id == ids_b[j]) {
                    ib = i;
                }
                if ((ia >= 0) && (ib >= 0)) {
                    break;
                }
            }
            if ((ia >= 0) && (ib >= 0) && (ia != ib)) {
                island_union(ia, ib);
            }
        }
    }
    /* Labels in first-seen order; awake if any dynamic member is awake. */
    for (int i = 0; i < body_count; i++) {
        int root = island_find(i);
        if (island_label[root] < 0) {
            island_label[root] = island_total;
            island_awake[island_total] = 0;
            island_total++;
        }
        island_label[i] = island_label[root];
        if ((!bodies[i].static_state) && (!bodies[i].is_sleeping)) {
            island_awake[island_label[i]] = 1;
        }
    }
}

int islands_count(void) {
    return island_total;
}

int islands_body_island(rigidbody *bodies, rigidbody *body) {
    if ((!bodies) || (bodies != island_base) || (island_body_count <= 0)) {
        return -1;
    }
    int idx = island_index_of(bodies, body, island_body_count);
    if (idx < 0) {
        return -1;
    }
    return island_label[idx];
}

bool islands_body_awake(rigidbody *bodies, rigidbody *body) {
    int island = islands_body_island(bodies, body);
    if (island < 0) {
        return true;
    }
    return island_awake[island] != 0;
}

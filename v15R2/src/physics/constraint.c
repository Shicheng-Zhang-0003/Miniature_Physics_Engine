/* MPE_FTC_063 */
#include "constraint.h"
#include "revolute_joint.h"
#include "../config/mpe_constants.h"

static constraint constraint_pool [mpe_max_joints];
static int constraint_count = 0;

void constraint_pool_init (void) {
    for (int i = 0; i < mpe_max_joints; i++) { constraint_pool [i].is_active = false; }
    constraint_count = 0;
}

int constraint_add_revolute (uint32_t id_a, uint32_t id_b, vector3 anchor_a, vector3 anchor_b, vector3 axis_a) {
    if ((id_a == 0) || (id_b == 0) || (id_a == id_b)) { return -1; }
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!constraint_pool [i].is_active) {
            constraint_pool [i].type = CONSTRAINT_REVOLUTE;
            constraint_pool [i].body_id_a = id_a;
            constraint_pool [i].body_id_b = id_b;
            constraint_pool [i].p.revolute.anchor_a = anchor_a;
            constraint_pool [i].p.revolute.anchor_b = anchor_b;
            constraint_pool [i].p.revolute.axis_a = vector3_normalisation (axis_a);
            constraint_pool [i].p.revolute.motor_enabled = false;
            constraint_pool [i].p.revolute.limits_enabled = false;
            constraint_pool [i].p.revolute.motor_target_speed = 0.0f;
            constraint_pool [i].p.revolute.motor_max_torque = 0.0f;
            constraint_pool [i].is_active = true;
            constraint_count++;
            return i;
        }
    }
    return -1;
}

void constraint_remove (int index) {
    if ((index < 0) || (index >= mpe_max_joints)) { return; }
    if (!constraint_pool [index].is_active) { return; }
    constraint_pool [index].is_active = false;
    constraint_count--;
}

int constraint_get_count (void) { return constraint_count; }

void constraint_set_revolute_motor (int index, bool enabled, float target_speed, float max_torque) {
    if ((index < 0) || (index >= mpe_max_joints)) { return; }
    if (!constraint_pool [index].is_active) { return; }
    if (constraint_pool [index].type != CONSTRAINT_REVOLUTE) { return; }
    constraint_pool [index].p.revolute.motor_enabled = enabled;
    constraint_pool [index].p.revolute.motor_target_speed = target_speed;
    constraint_pool [index].p.revolute.motor_max_torque = max_torque;
}

static rigidbody *find_body_by_id (rigidbody *bodies, int body_count, uint32_t id) {
    if (!bodies) { return NULL; }
    for (int i = 0; i < body_count; i++) {
        if (bodies [i].object_id == id) { return &bodies [i]; }
    }
    return NULL;
}

static void constraint_dispatch (rigidbody *bodies, int body_count, float dt, bool motors_pass) {
    if ((!bodies) || (body_count <= 0)) { return; }
    for (int i = 0; i < mpe_max_joints; i++) {
        if (!constraint_pool [i].is_active) { continue; }
        constraint *c = &constraint_pool [i];
        rigidbody *body_a = find_body_by_id (bodies, body_count, c->body_id_a);
        rigidbody *body_b = find_body_by_id (bodies, body_count, c->body_id_b);
        if ((!body_a) || (!body_b)) { continue; }
        if (c->type == CONSTRAINT_REVOLUTE) {
            if (motors_pass) { revolute_apply_motor (&c->p.revolute, body_a, body_b, dt); }
            else { revolute_solve (&c->p.revolute, body_a, body_b, dt); }
        }
        /* CONSTRAINT_FIXED / CONSTRAINT_PRISMATIC dispatched in 064 / 065 */
    }
}

void constraint_solve_all (rigidbody *bodies, int body_count, float dt) {
    if (dt <= 0.0f) { return; }
    constraint_dispatch (bodies, body_count, dt, false);
}

void constraint_apply_motors (rigidbody *bodies, int body_count, float dt) {
    if (dt <= 0.0f) { return; }
    constraint_dispatch (bodies, body_count, dt, true);
}

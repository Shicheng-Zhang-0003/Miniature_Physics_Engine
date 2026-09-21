/* Sleep contact-wake adversarial test.
 * A SLOW (0.05 m/s, below the 0.1 m/s velocity wake gate) kinematic pusher
 * must still wake a sleeping body on first touch — pure velocity gates
 * leave it solving as a ghost (infinite mass, momentum lost). At the same
 * time a lone sleeping body in persistent floor contact must STAY asleep
 * (no wake churn from resting contacts).
 * Built via `make test_sleep_contact_wake`.
 */
#ifdef mpe_sleep_contact_wake_test
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(&world);

    /* Sleeper: still sphere, pinned asleep like the F6 setup. */
    int sleeper = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){2.0f, 0.5f, 0.0f});
    world.bodies[sleeper].velocity = vector3_zero();
    world.bodies[sleeper].angular_velocity = vector3_zero();
    world.bodies[sleeper].is_sleeping = true;
    world.bodies[sleeper].sleep_timer = 1.0f;
    world.bodies[sleeper].restitution = 0.0f;

    /* Slow kinematic pusher: 0.05 m/s toward the sleeper, gap 0.6 m.
     * Touch at ~tick 720; velocity gate (0.1) can never fire. */
    int pusher = physics_world_add_cube(&world, (vector3){0.4f, 0.5f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    rigidbody_set_kinematic(&world.bodies[pusher], true);
    world.bodies[pusher].velocity = (vector3){0.05f, 0.0f, 0.0f};

    /* Control: lone sleeper far away, floor contact only. */
    int control = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){10.0f, 0.5f, 0.0f});
    world.bodies[control].velocity = vector3_zero();
    world.bodies[control].angular_velocity = vector3_zero();
    world.bodies[control].is_sleeping = true;
    world.bodies[control].sleep_timer = 1.0f;
    world.bodies[control].restitution = 0.0f;

    const float dt = 1.0f / 60.0f;
    int touch_tick = -1, wake_tick = -1, control_wake_tick = -1;
    int fail = 0;
    for (int t = 0; t < 1200; t++) {
        physics_world_step(&world, dt);
        rigidbody *s = &world.bodies[sleeper];
        rigidbody *p = &world.bodies[pusher];
        rigidbody *c = &world.bodies[control];
        if (!isfinite(s->position.x) || !isfinite(p->position.x) || !isfinite(c->position.x)) {
            printf("[FAIL] NaN at tick %d\n", t);
            fail = 1;
            break;
        }
        /* Geometric touch AND engine slop-band touch: narrowphase admits
         * contacts within penetration_slop (zero-depth, friction-only), so
         * first engine contact precedes geometric touch by ~slop. The wake
         * must follow ENGINE contact (slop), not geometric coincidence. */
        float gap = (s->position.x - 0.5f) - (p->position.x + 0.5f);
        if (touch_tick < 0 && gap <= g_cfg.solver.penetration_slop) {
            touch_tick = t;
        }
        if (wake_tick < 0 && !s->is_sleeping) {
            wake_tick = t;
        }
        if (control_wake_tick < 0 && !c->is_sleeping) {
            control_wake_tick = t;
        }
    }

    /* No tunneling: never deeply interpenetrated (slop riding is by design:
     * slop-band normal impulses carry the sleeper just outside touch). */
    float end_gap = (world.bodies[sleeper].position.x - 0.5f) - (world.bodies[pusher].position.x + 0.5f);
    printf("[info] touch=%d wake=%d control_wake=%d sleeper_x=%.4f pusher_x=%.4f control_y=%.4f end_gap=%.4f\n", touch_tick,
           wake_tick, control_wake_tick, world.bodies[sleeper].position.x, world.bodies[pusher].position.x,
           world.bodies[control].position.y, end_gap);
    if (touch_tick < 0) {
        printf("[FAIL] pusher never reached the sleeper\n");
        fail = 1;
    } else if (wake_tick < 0) {
        printf("[FAIL] slow pusher never woke the sleeper (ghost contact)\n");
        fail = 1;
    } else if (wake_tick - touch_tick > 5) {
        printf("[FAIL] wake lag %d ticks after first touch (sank %d ticks as ghost)\n", wake_tick - touch_tick,
               wake_tick - touch_tick);
        fail = 1;
    } else {
        printf("[PASS] first-touch wake in %d ticks at 0.05 m/s (velocity gate is 0.1)\n", wake_tick - touch_tick);
    }
    if (control_wake_tick >= 0) {
        printf("[FAIL] lone sleeper woken at tick %d by persistent floor contact (wake churn)\n", control_wake_tick);
        fail = 1;
    } else {
        printf("[PASS] lone sleeper undisturbed by resting contact\n");
    }
    /* TRUTH: control must also be WHERE it slept (position, not just flag):
     * a sleeper that fell through the floor asleep still passes the flag
     * check. And the woken sleeper must have been PUSHED (momentum
     * transfer), not just flagged awake. */
    {
        float control_y = world.bodies[control].position.y;
        if (control_y < 0.4f || control_y > 0.6f) {
            printf("[FAIL] control sleeper displaced (y=%.4f, expect ~0.5)\n", control_y);
            fail = 1;
        }
        if (wake_tick >= 0 && world.bodies[sleeper].position.x < 2.1f) {
            printf("[FAIL] sleeper woken but never pushed (x=%.4f)\n", world.bodies[sleeper].position.x);
            fail = 1;
        }
    }
    /* No tunneling: never deeply interpenetrated (slop riding is by design:
     * slop-band normal impulses carry the sleeper just outside touch). */
    if (end_gap < -0.02f) {
        printf("[FAIL] pusher tunneled into the sleeper (gap=%.4f)\n", end_gap);
        fail = 1;
    }
    if (!fail) {
        printf("[PASS] sleep contact-wake truth holds\n");
    }
    physics_world_cleanup(&world);
    return fail ? 1 : 0;
}
#endif /* mpe_sleep_contact_wake_test */

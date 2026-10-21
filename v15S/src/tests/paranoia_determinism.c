/* PARANOIA TEST: Determinism - bitwise reproducibility across runs */
#ifdef mpe_paranoia_determinism
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
static int bodies_bitwise_equal (const rigidbody *a, const rigidbody *b) {
    if (a -> position.x != b -> position.x || a -> position.y != b -> position.y || a -> position.z != b -> position.z)
        return 0;
    if (a -> velocity.x != b -> velocity.x || a -> velocity.y != b -> velocity.y || a -> velocity.z != b -> velocity.z)
        return 0;
    if (a -> acceleration.x != b -> acceleration.x || a -> acceleration.y != b -> acceleration.y ||
        a -> acceleration.z != b -> acceleration.z)
        return 0;
    if (a -> orientation.w != b -> orientation.w || a -> orientation.x != b -> orientation.x ||
        a -> orientation.y != b -> orientation.y || a -> orientation.z != b -> orientation.z)
        return 0;
    if (a -> angular_velocity.x != b -> angular_velocity.x || a -> angular_velocity.y != b -> angular_velocity.y ||
        a -> angular_velocity.z != b -> angular_velocity.z)
        return 0;
    if (a -> angular_acceleration.x != b -> angular_acceleration.x ||
        a -> angular_acceleration.y != b -> angular_acceleration.y ||
        a -> angular_acceleration.z != b -> angular_acceleration.z)
        return 0;
    if (a -> force_accumulator.x != b -> force_accumulator.x || a -> force_accumulator.y != b -> force_accumulator.y ||
        a -> force_accumulator.z != b -> force_accumulator.z)
        return 0;
    if (a -> torque_accumulator.x != b -> torque_accumulator.x || a -> torque_accumulator.y != b -> torque_accumulator.y ||
        a -> torque_accumulator.z != b -> torque_accumulator.z)
        return 0;
    return 1;
}
typedef struct {
    pthread_mutex_t lock;
    pthread_cond_t ready_cond;
    int ready;
    int go;
} concurrent_start_t;
typedef struct {
    physics_world *world;
    concurrent_start_t *start;
    int ticks;
} concurrent_step_t;
static void *concurrent_step_world (void *arg) {
    concurrent_step_t *step = (concurrent_step_t *) arg;
    pthread_mutex_lock (&step->start->lock);
    step->start->ready++;
    pthread_cond_broadcast (&step->start->ready_cond);
    while (!step->start->go) pthread_cond_wait (&step->start->ready_cond, &step->start->lock);
    pthread_mutex_unlock (&step->start->lock);
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < step->ticks; i++) physics_world_step (step->world, dt);
    return NULL;
}
static int concurrent_worlds_match_serial (void) {
    enum { BODY_COUNT = 96, TICKS = 180 };
    mpe_config_t cfg_a = g_cfg, cfg_b = g_cfg;
    cfg_a.world.gravity = -9.81f;
    cfg_b.world.gravity = -1.0f;
    physics_world concurrent_a, concurrent_b, serial_a, serial_b;
    physics_world *worlds[] = {&concurrent_a, &concurrent_b, &serial_a, &serial_b};
    mpe_config_t *configs[] = {&cfg_a, &cfg_b, &cfg_a, &cfg_b};
    int ok = 1;
    for (int wi = 0; wi < 4; wi++) {
        physics_world_init (worlds[wi]);
        constraint_pool_init (worlds[wi]);
        physics_world_set_config (worlds[wi], configs[wi]);
        for (int i = 0; i < BODY_COUNT; i++) {
            /* A close-packed but separated grid produces many simultaneous
             * contacts sharing bodies, exercising narrowphase shared-state
             * handling as well as the global worker-pool dispatch. */
            vector3 p = {(float)(i % 12) * 0.45f, 50.0f + (float)(i / 12) * 0.45f,
                         (float)(i / 12) * 0.45f};
            int bi = physics_world_add_sphere (worlds[wi], 0.25f, 1.0f, p);
            if (bi < 0) { ok = 0; break; }
            worlds[wi]->bodies[bi].velocity = (vector3) {0.01f * (float)(i % 3), 0.0f, 0.0f};
        }
        if (!ok || worlds[wi]->body_count != BODY_COUNT) ok = 0;
    }
    if (!ok) goto cleanup;

    concurrent_start_t start = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, 0, 0};
    concurrent_step_t steps[] = {{&concurrent_a, &start, TICKS}, {&concurrent_b, &start, TICKS}};
    pthread_t tids[2];
    int created = 0;
    for (int i = 0; i < 2; i++) {
        if (pthread_create (&tids[i], NULL, concurrent_step_world, &steps[i]) != 0) {
            ok = 0;
            break;
        }
        created++;
    }
    if (created == 2) {
        pthread_mutex_lock (&start.lock);
        while (start.ready < 2) pthread_cond_wait (&start.ready_cond, &start.lock);
        start.go = 1;
        pthread_cond_broadcast (&start.ready_cond);
        pthread_mutex_unlock (&start.lock);
    } else {
        pthread_mutex_lock (&start.lock);
        start.go = 1;
        pthread_cond_broadcast (&start.ready_cond);
        pthread_mutex_unlock (&start.lock);
    }
    for (int i = 0; i < created; i++) pthread_join (tids[i], NULL);
    pthread_cond_destroy (&start.ready_cond);
    pthread_mutex_destroy (&start.lock);
    if (created != 2) ok = 0;

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < TICKS; i++) {
        physics_world_step (&serial_a, dt);
        physics_world_step (&serial_b, dt);
    }
    for (int wi = 0; wi < 2; wi++) {
        physics_world *actual = worlds[wi], *expected = worlds[wi + 2];
        if (actual->body_count != expected->body_count ||
            actual->world_contact_cache_count != expected->world_contact_cache_count) ok = 0;
        for (int i = 0; ok && i < BODY_COUNT; i++) {
            if (!bodies_bitwise_equal (&actual->bodies[i], &expected->bodies[i])) ok = 0;
        }
    }
cleanup:
    for (int wi = 0; wi < 4; wi++) physics_world_cleanup (worlds[wi]);
    return ok;
}
int main (void) {
    mpe_config_init ();
    int fail = 0;
    if (setenv ("MPE_THREADS", "4", 1) != 0) {
        printf ("[FAIL] could not select four pool workers\n");
        return 1;
    }
    /* Test 1: Bitwise identical across twin worlds */
    {
        physics_world w1, w2;
        physics_world_init (&w1);
        physics_world_init (&w2);
        constraint_pool_init (&w1);
        constraint_pool_init (&w2);
        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 0.99f;
        int s1 = physics_world_add_sphere (&w1, 0.5f, 2.0f, (vector3) {-1.0f, 3.0f, 0.5f});
        w1.bodies [s1].velocity = (vector3) {1.5f, -0.5f, 0.25f};
        w1.bodies [s1].angular_velocity = (vector3) {3.0f, -1.0f, 2.0f};
        w1.bodies [s1].restitution = 0.4f;
        int c1 = physics_world_add_cube (&w1, (vector3) {1.0f, 0.5f, -0.5f}, (vector3) {0.5f, 0.5f, 0.5f}, 1.0f);
        w1.bodies [c1].velocity = (vector3) {-0.75f, 0.0f, 0.5f};
        w1.bodies [c1].angular_velocity = (vector3) {0.0f, 2.0f, -1.5f};
        w1.bodies [c1].restitution = 0.3f;
        /* Exact duplicate in w2 */
        int s2 = physics_world_add_sphere (&w2, 0.5f, 2.0f, (vector3) {-1.0f, 3.0f, 0.5f});
        w2.bodies [s2].velocity = (vector3) {1.5f, -0.5f, 0.25f};
        w2.bodies [s2].angular_velocity = (vector3) {3.0f, -1.0f, 2.0f};
        w2.bodies [s2].restitution = 0.4f;
        int c2 = physics_world_add_cube (&w2, (vector3) {1.0f, 0.5f, -0.5f}, (vector3) {0.5f, 0.5f, 0.5f}, 1.0f);
        w2.bodies [c2].velocity = (vector3) {-0.75f, 0.0f, 0.5f};
        w2.bodies [c2].angular_velocity = (vector3) {0.0f, 2.0f, -1.5f};
        w2.bodies [c2].restitution = 0.3f;
        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 1200; t++) {
            physics_world_step (&w1, dt);
            physics_world_step (&w2, dt);
        } int mismatch = 0;
        if (w1.body_count != w2.body_count) { mismatch = 1; }
        for (int i = 0; i < w1.body_count && !mismatch; i++) {
            if (!bodies_bitwise_equal (&w1.bodies [i], &w2.bodies [i])) {
                mismatch = 1;
                break;
            }
        }
        if (mismatch) {
            printf ("[FAIL] twin worlds diverged\n");
            fail = 1;
        } else {
            printf ("[PASS] twin worlds bitwise identical for 1200 ticks\n");
        } physics_world_cleanup (&w1);
        physics_world_cleanup (&w2);
    }
    /* Test 2: Deterministic across re-initialization (same initial conditions, same ticks) */
    {
        physics_world world;
        physics_world_init (&world);
        constraint_pool_init (&world);
        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 0.99f;
        int a = physics_world_add_sphere (&world, 0.5f, 1.0f, (vector3) {0.0f, 2.0f, 0.0f});
        world.bodies [a].velocity = (vector3) {1.0f, 0.0f, 0.0f};
        world.bodies [a].restitution = 0.5f;
        rigidbody_wake (&world.bodies [a]);
        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 300; t++)
            physics_world_step (&world, dt);
        /* Recreate world with exact same initial conditions */
        physics_world world2;
        physics_world_init (&world2);
        constraint_pool_init (&world2);
        int b = physics_world_add_sphere (&world2, 0.5f, 1.0f, (vector3) {0.0f, 2.0f, 0.0f});
        world2.bodies [b].velocity = (vector3) {1.0f, 0.0f, 0.0f};
        world2.bodies [b].restitution = 0.5f;
        rigidbody_wake (&world2.bodies [b]);
        for (int t = 0; t < 300; t++)
            physics_world_step (&world2, dt);
        /* Both worlds now have 300 ticks. Compare final state. */
        float pos_diff = fabsf (world.bodies [0].position.x - world2.bodies [0].position.x) +
                         fabsf (world.bodies [0].position.y - world2.bodies [0].position.y) +
                         fabsf (world.bodies [0].position.z - world2.bodies [0].position.z);
        float vel_diff = fabsf (world.bodies [0].velocity.x - world2.bodies [0].velocity.x) +
                         fabsf (world.bodies [0].velocity.y - world2.bodies [0].velocity.y) +
                         fabsf (world.bodies [0].velocity.z - world2.bodies [0].velocity.z);
        printf ("[INFO] determinism_reinit pos_diff=%.6f vel_diff=%.6f\n", pos_diff, vel_diff);
        if (pos_diff > 0.0f || vel_diff > 0.0f) {
            printf ("[FAIL] re-init not deterministic\n");
            fail = 1;
        } else {
            printf ("[PASS] re-initialization deterministic\n");
        } physics_world_cleanup (&world);
        physics_world_cleanup (&world2);
    }
    /* Test 3: Deterministic across 10 independent runs (compare to reference run) */
    {
        physics_world world_ref;
        physics_world_init (&world_ref);
        constraint_pool_init (&world_ref);
        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;
        int a = physics_world_add_cube (&world_ref, (vector3) {0.0f, 2.0f, 0.0f}, (vector3) {0.5f, 0.5f, 0.5f}, 1.0f);
        world_ref.bodies [a].velocity = (vector3) {2.0f, -1.0f, 0.5f};
        world_ref.bodies [a].angular_velocity = (vector3) {1.0f, 2.0f, -1.0f};
        world_ref.bodies [a].restitution = 0.3f;
        const float dt = 1.0f / 60.0f;
        /* Run reference world to get final position */
        for (int t = 0; t < 600; t++)
            physics_world_step (&world_ref, dt);
        vector3 pos_ref = world_ref.bodies [0].position;
        vector3 vel_ref = world_ref.bodies [0].velocity;
        int sub_fail = 0;
        for (int run = 0; run < 10; run++) {
            physics_world world2;
            physics_world_init (&world2);
            constraint_pool_init (&world2);
            int b = physics_world_add_cube (&world2, (vector3) {0.0f, 2.0f, 0.0f}, (vector3) {0.5f, 0.5f, 0.5f}, 1.0f);
            world2.bodies [b].velocity = (vector3) {2.0f, -1.0f, 0.5f};
            world2.bodies [b].angular_velocity = (vector3) {1.0f, 2.0f, -1.0f};
            world2.bodies [b].restitution = 0.3f;
            for (int t = 0; t < 600; t++)
                physics_world_step (&world2, dt);
            float pos_diff = fabsf (world2.bodies [0].position.x - pos_ref.x) +
                             fabsf (world2.bodies [0].position.y - pos_ref.y) +
                             fabsf (world2.bodies [0].position.z - pos_ref.z);
            float vel_diff = fabsf (world2.bodies [0].velocity.x - vel_ref.x) +
                             fabsf (world2.bodies [0].velocity.y - vel_ref.y) +
                             fabsf (world2.bodies [0].velocity.z - vel_ref.z);
            if (pos_diff > 0.0f || vel_diff > 0.0f) {
                printf ("[FAIL] run %d diverged (pos_diff=%.6f vel_diff=%.6f)\n", run, pos_diff, vel_diff);
                sub_fail = 1;
            } physics_world_cleanup (&world2);
        }
        if (sub_fail) { fail = 1; } else {
            printf ("[PASS] 10 independent runs bitwise identical to reference\n");
        } physics_world_cleanup (&world_ref);
    }
    /* Test 4: Floating point determinism - no NaN/Inf propagation */
    {
        physics_world world;
        physics_world_init (&world);
        constraint_pool_init (&world);
        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;
        /* Create scenario that could produce NaN */
        physics_world_add_sphere (&world, 0.0f, 1.0f,
                                  (vector3) {0.0f, 0.0f, 0.0f}); /* zero radius - should be sanitized */
        physics_world_add_cube (&world, (vector3) {0.0f, 0.0f, 0.0f}, (vector3) {0.0f, 0.0f, 0.0f},
                                1.0f); /* zero size - sanitized */
        const float dt = 1.0f / 60.0f;
        int nan_count = 0;
        for (int t = 0; t < 600; t++) {
            physics_world_step (&world, dt);
            for (int i = 0; i < world.body_count; i++) {
                rigidbody *rb = &world.bodies [i];
                if (!isfinite (rb -> position.x) || !isfinite (rb -> velocity.x))
                    nan_count++;
            }
        } printf ("[INFO] sanitization_nan_test nan_count=%d\n", nan_count);
        if (nan_count > 0) {
            printf ("[FAIL] NaN propagated\n");
            fail = 1;
        } else {
            printf ("[PASS] sanitization prevents NaN propagation\n");
        } physics_world_cleanup (&world);
    }
    /* Test 5: concurrent worlds cannot overwrite the process-wide pool phase. */
    if (concurrent_worlds_match_serial ()) {
        printf ("[PASS] concurrent worlds match independent serial stepping\n");
    } else {
        printf ("[FAIL] concurrent worlds corrupted or diverged from serial stepping\n");
        fail = 1;
    }
    return fail;
}
#endif

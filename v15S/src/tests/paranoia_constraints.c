/* PARANOIA TEST: Constraints/Joints - revolute, prismatic, distance, rope, fixed */
#ifdef mpe_paranoia_constraints
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

static double pendulum_reference_period(double length, double bob_radius, double angle) {
    /* Rigid spherical bob about a point pivot: I = 2/5*m*r^2. The finite-
     * amplitude factor is the convergent series for 2*K(sin(angle/2))/pi. */
    double k2 = pow(sin(0.5 * angle), 2.0);
    double amplitude_factor = 1.0 + k2 / 4.0 + 9.0 * k2 * k2 / 64.0 +
                              25.0 * k2 * k2 * k2 / 256.0 +
                              1225.0 * k2 * k2 * k2 * k2 / 16384.0;
    double inertia_factor = sqrt(1.0 + 0.4 * bob_radius * bob_radius / (length * length));
    return 2.0 * M_PI * sqrt(length / 9.81) * inertia_factor * amplitude_factor;
}

static vector3 fixed_world_anchor(const rigidbody *body, vector3 local_anchor) {
    vector3 offset = vector3_addition(vector3_scaling(body->cached_axes[0], local_anchor.x),
                                      vector3_addition(vector3_scaling(body->cached_axes[1], local_anchor.y),
                                                       vector3_scaling(body->cached_axes[2], local_anchor.z)));
    return vector3_addition(body->position, offset);
}

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Revolute pendulum period, including finite swing amplitude and
     * the sphere's moment of inertia. Consecutive x=0 crossings are half a
     * period, so double their mean before comparing. */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int pivot = physics_world_add_cube(&world, (vector3){0.0f, 10.0f, 0.0f}, (vector3){0.2f, 0.2f, 0.2f}, 1.0f);
        rigidbody_set_static(&world.bodies[pivot], true);
        uint32_t pivot_id = world.bodies[pivot].object_id;

        int bob = physics_world_add_sphere(&world, 0.3f, 2.0f, (vector3){1.0f, 8.0f, 0.0f});
        world.bodies[bob].restitution = 0.0f;
        world.bodies[bob].friction_static = 0.0f;
        world.bodies[bob].friction_kinetic = 0.0f;
        uint32_t bob_id = world.bodies[bob].object_id;

        vector3 pivot_point = {0.0f, 10.0f, 0.0f};
        float rod_length = vector3_length(vector3_subtraction(pivot_point, world.bodies[bob].position));

        vector3 anchor_a = {0.0f, 0.0f, 0.0f};
        vector3 anchor_b = {-1.0f, 2.0f, 0.0f};
        vector3 axis = {0.0f, 0.0f, 1.0f};
        int joint = constraint_add_revolute(&world, pivot_id, bob_id, anchor_a, anchor_b, axis);
        if (joint < 0) {
            printf("[FAIL] could not create revolute joint\n");
            physics_world_cleanup(&world);
            return 1;
        }

        const float dt = 1.0f / 60.0f;
        float periods[64];
        int period_count = 0;
        int last_cross = 0;
        float last_x = world.bodies[bob].position.x;

        for (int t = 0; t < 3000; t++) {
            physics_world_step(&world, dt);
            rigidbody *bob = &world.bodies[1];
            if (t > 60 && last_x * bob->position.x < 0) { /* zero crossing */
                if (period_count > 0 && period_count - 1 < (int)(sizeof(periods) / sizeof(periods[0]))) {
                    periods[period_count-1] = (t - last_cross) * (1.0f/60.0f);
                }
                if (period_count < (int)(sizeof(periods) / sizeof(periods[0]))) period_count++;
                last_cross = t;
            }
            last_x = bob->position.x;
        }

        float angle = atan2f(fabsf(world.bodies[bob].position.x - pivot_point.x),
                             fabsf(pivot_point.y - world.bodies[bob].position.y));
        float expected_T = (float)pendulum_reference_period(rod_length, world.bodies[bob].radius, angle);
        float avg_T = 0.0f;
        int valid = 0;
        for (int i = 0; i < period_count; i++) {
            if (periods[i] > 0.1f && periods[i] < 10.0f) {
                avg_T += periods[i];
                valid++;
            }
        }
        if (valid == 0) {
            printf("[FAIL] revolute pendulum produced no measurable crossings\n");
            fail = 1;
        } else {
            avg_T = (avg_T / valid) * 2.0f;
            float err = fabsf(avg_T - expected_T) / expected_T;
            printf("[INFO] revolute_period expected=%.4f avg=%.4f err=%.2f%%\n", expected_T, avg_T, err*100);
            if (err > 0.05f) { printf("[FAIL] pendulum period error %.2f%%\n", err*100); fail = 1; }
            else { printf("[PASS] revolute pendulum period matches finite-amplitude rigid-bob model\n"); }
        }

        physics_world_cleanup(&world);
    }

    /* Test 2: Revolute motor - target velocity tracking */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f; /* no gravity for motor test */
        g_cfg.world.drag = 1.0f;

        int base = physics_world_add_cube(&world, (vector3){0.0f, 0.0f, 0.0f}, (vector3){1.0f, 1.0f, 1.0f}, 1.0f);
        if (base < 0) {
            printf("[FAIL] could not create motor base\n");
            physics_world_cleanup(&world);
            return 1;
        }
        rigidbody_set_static(&world.bodies[base], true);
        uint32_t base_id = world.bodies[base].object_id;

        int wheel = physics_world_add_cylinder(&world, 0.5f, 0.2f, 10.0f, (vector3){0.0f, 0.0f, 5.0f});
        if (wheel < 0) {
            printf("[FAIL] could not create motor wheel\n");
            physics_world_cleanup(&world);
            return 1;
        }
        world.bodies[wheel].restitution = 0.0f;
        world.bodies[wheel].friction_static = 0.0f;
        world.bodies[wheel].friction_kinetic = 0.0f;
        uint32_t wheel_id = world.bodies[wheel].object_id;

        int joint = constraint_add_revolute(&world, base_id, wheel_id,
                                            (vector3){0,0,5}, (vector3){0,0,0}, (vector3){1,0,0});
        if (joint < 0) {
            printf("[FAIL] could not create motor joint\n");
            physics_world_cleanup(&world);
            return 1;
        }
        constraint_set_revolute_motor(&world, joint, true, 10.0f, 100.0f);

        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 180; t++) {
            physics_world_step(&world, dt);
        }

        rigidbody *w = &world.bodies[wheel];
        float actual_w = fabsf(w->angular_velocity.x);
        float target_w = 10.0f;
        float err = fabsf(actual_w - target_w) / target_w;

        printf("[INFO] motor actual_w=%.3f target=%.3f err=%.2f%% enabled=%d\n", actual_w, target_w, err*100,
               world.revolute_constraints[joint].p.revolute.motor_enabled);
        if (err > 0.1f) { printf("[FAIL] motor velocity error %.2f%%\n", err*100); fail = 1; }
        else { printf("[PASS] revolute motor tracks target velocity\n"); }

        physics_world_cleanup(&world);
    }

    /* Test 3: Prismatic joint - linear slider */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int base = physics_world_add_cube(&world, (vector3){0.0f, 0.0f, 0.0f}, (vector3){1.0f, 1.0f, 1.0f}, 1.0f);
        rigidbody_set_static(&world.bodies[base], true);
        uint32_t base_id = world.bodies[base].object_id;

        int slider = physics_world_add_cube(&world, (vector3){2.0f, 0.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        world.bodies[slider].friction_static = 0.0f;
        world.bodies[slider].friction_kinetic = 0.0f;
        uint32_t slider_id = world.bodies[slider].object_id;

        int joint = constraint_add_prismatic(&world, base_id, slider_id, (vector3){0,0,0}, (vector3){0,0,0}, (vector3){1,0,0});
        constraint_set_prismatic_limits(&world, joint, true, -2.0f, 2.0f);

        const float dt = 1.0f / 60.0f;
        for (int t = 0; t < 240; t++) {
            physics_world_step(&world, dt);
            /* Push slider */
            world.bodies[1].force_accumulator.x += 10.0f;
        }

        float pos = world.bodies[1].position.x;
        if (pos > 2.0f || pos < -2.0f) {
            printf("[FAIL] prismatic limits violated: pos=%.3f\n", pos); fail = 1;
        } else {
            printf("[PASS] prismatic limits enforced\n");
        }
        physics_world_cleanup(&world);
    }

    /* Test 4: Distance constraint - preserves distance */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.2f, 1.0f, (vector3){0.0f, 5.0f, 0.0f});
        int b = physics_world_add_sphere(&world, 0.2f, 1.0f, (vector3){1.0f, 5.0f, 0.0f});
        world.bodies[a].restitution = 0.0f;
        world.bodies[b].restitution = 0.0f;
        uint32_t id_a = world.bodies[a].object_id;
        uint32_t id_b = world.bodies[b].object_id;

        int joint = constraint_add_distance(&world, id_a, id_b, (vector3){0,0,0}, (vector3){0,0,0}, 1.0f);
        if (joint < 0) {
            printf("[FAIL] could not create distance constraint\n");
            physics_world_cleanup(&world);
            return 1;
        }

        const float dt = 1.0f / 60.0f;
        float max_err = 0.0f;

        for (int t = 0; t < 1200; t++) {
            physics_world_step(&world, dt);
            float d = vector3_length(vector3_subtraction(world.bodies[0].position, world.bodies[1].position));
            float err = fabsf(d - 1.0f);
            if (err > max_err) max_err = err;
        }

        printf("[INFO] distance_constraint max_err=%.6f\n", max_err);
        if (max_err > 0.001f) { printf("[FAIL] distance constraint drift %.6f\n", max_err); fail = 1; }
        else { printf("[PASS] distance constraint maintains exact separation\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 5: Fixed weld keeps its two local anchor points coincident. */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_cube(&world, (vector3){0.0f, 2.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        int b = physics_world_add_cube(&world, (vector3){1.5f, 2.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        world.bodies[a].restitution = 0.0f;
        world.bodies[b].restitution = 0.0f;
        uint32_t id_a = world.bodies[a].object_id;
        uint32_t id_b = world.bodies[b].object_id;

        /* The centers start 1.5 m apart. These anchors designate the same
         * initial world point; zero/zero anchors would instead ask the weld
         * to collapse the two centers and invalidate a 1.5 m target. */
        vector3 anchor_a = {0.75f, 0.0f, 0.0f};
        vector3 anchor_b = {-0.75f, 0.0f, 0.0f};
        int joint = constraint_add_fixed(&world, id_a, id_b, anchor_a, anchor_b);
        if (joint < 0) {
            printf("[FAIL] could not create fixed joint\n");
            physics_world_cleanup(&world);
            return 1;
        }

        const float dt = 1.0f / 60.0f;
        float max_gap = 0.0f;

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
            vector3 pa = fixed_world_anchor(&world.bodies[a], anchor_a);
            vector3 pb = fixed_world_anchor(&world.bodies[b], anchor_b);
            float gap = vector3_length(vector3_subtraction(pa, pb));
            if (gap > max_gap) max_gap = gap;
        }

        printf("[INFO] fixed_weld max_anchor_gap=%.6f m\n", max_gap);
        if (max_gap > 0.02f) { printf("[FAIL] fixed weld anchor drift %.6f m\n", max_gap); fail = 1; }
        else { printf("[PASS] fixed weld keeps world anchors coincident\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 6: Rope constraint - inequality distance */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.2f, 1.0f, (vector3){0.0f, 0.0f, 0.0f});
        int b = physics_world_add_sphere(&world, 0.2f, 1.0f, (vector3){0.6f, 0.0f, 0.0f});
        world.bodies[a].restitution = 0.0f;
        world.bodies[b].restitution = 0.0f;
        uint32_t id_a = world.bodies[a].object_id;
        uint32_t id_b = world.bodies[b].object_id;

        int joint = constraint_add_rope(&world, id_a, id_b, (vector3){0,0,0}, (vector3){0,0,0}, 1.0f);
        if (joint < 0) {
            printf("[FAIL] could not create rope constraint\n");
            physics_world_cleanup(&world);
            return 1;
        }

        const float dt = 1.0f / 60.0f;
        world.bodies[b].velocity.x = 2.0f;
        rigidbody_wake(&world.bodies[b]);
        float max_dist = 0.0f;

        /* Start slack, then give the endpoint outward speed. The rope must
         * become taut without exceeding its maximum by more than solver slop. */
        for (int t = 0; t < 120; t++) {
            physics_world_step(&world, dt);
            float d = vector3_length(vector3_subtraction(world.bodies[a].position, world.bodies[b].position));
            if (d > max_dist) max_dist = d;
        }

        float taut_dist = vector3_length(vector3_subtraction(world.bodies[a].position, world.bodies[b].position));
        world.bodies[b].velocity.x = -1.0f; /* an inward impulse should create slack */
        rigidbody_wake(&world.bodies[b]);
        for (int t = 0; t < 60; t++) {
            physics_world_step(&world, dt);
        }
        float slack_dist = vector3_length(vector3_subtraction(world.bodies[a].position, world.bodies[b].position));
        printf("[INFO] rope max_dist=%.3f taut=%.3f after_inward_impulse=%.3f\n",
               max_dist, taut_dist, slack_dist);
        if (max_dist > 1.10f || taut_dist > 1.10f) {
            printf("[FAIL] rope exceeded its maximum length\n"); fail = 1;
        } else if (!(slack_dist < 0.99f)) {
            printf("[FAIL] rope resisted compression instead of going slack\n"); fail = 1;
        } else {
            printf("[PASS] rope enforces maximum length and permits slack\n");
        }
        physics_world_cleanup(&world);
    }

    return fail;
}
#endif

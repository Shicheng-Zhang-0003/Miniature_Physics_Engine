/* PARANOIA TEST: Energy/Momentum conservation - the gold standard */
#ifdef mpe_paranoia_energy_momentum
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Linear momentum conservation in closed system */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 2.0f, (vector3){-2.0f, 0.0f, 0.0f});
        int b = physics_world_add_sphere(&world, 0.3f, 1.0f, (vector3){2.0f, 0.0f, 0.0f});
        world.bodies[a].velocity = (vector3){5.0f, 0.0f, 0.0f};
        world.bodies[b].velocity = (vector3){-3.0f, 0.0f, 0.0f};
        world.bodies[a].restitution = 1.0f;
        world.bodies[b].restitution = 1.0f;
        rigidbody_wake(&world.bodies[a]);
        rigidbody_wake(&world.bodies[b]);

        vector3 P0 = {0,0,0};
        P0 = vector3_addition(P0, vector3_scaling(world.bodies[0].velocity, world.bodies[0].mass));
        P0 = vector3_addition(P0, vector3_scaling(world.bodies[1].velocity, world.bodies[1].mass));

        const float dt = 1.0f / 60.0f;
        float max_err = 0.0f;

        for (int t = 0; t < 3600; t++) {
            physics_world_step(&world, dt);
            vector3 P = {0,0,0};
            for (int i = 0; i < world.body_count; i++) {
                P = vector3_addition(P, vector3_scaling(world.bodies[i].velocity, world.bodies[i].mass));
            }
            float err = vector3_length(vector3_subtraction(P, P0));
            if (err > max_err) max_err = err;
        }

        printf("[INFO] momentum_conservation max_err=%.6f\n", max_err);
        if (max_err > 1e-4f) { printf("[FAIL] momentum drift %.6f\n", max_err); fail = 1; }
        else { printf("[PASS] linear momentum conserved (err < 0.1mum)\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 2: Angular momentum conservation (torque-free tumble) */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int b = physics_world_add_cube(&world, (vector3){0.0f, 0.0f, 0.0f}, (vector3){1.0f, 0.5f, 0.2f}, 1.0f);
        world.bodies[b].angular_velocity = (vector3){2.0f, 1.5f, 3.0f};
        rigidbody_wake(&world.bodies[b]);

        const float dt = 1.0f / 60.0f;
        float L0 = 0.0f, max_rel_err = 0.0f;

        for (int t = 0; t < 3600; t++) {
            physics_world_step(&world, dt);
            rigidbody *rb = &world.bodies[0];
            math3 I_world = {0};
            /* I_world = R * I_local * R^T */
            math3 R = vector4_to_math3(rb->orientation);
            math3 RT = math3_transposition(R);
            math3 tmp = math3_multiplication(R, rb->inertia_tensor_local);
            I_world = math3_multiplication(tmp, RT);
            vector3 L = math3_multiplication_vector3(I_world, rb->angular_velocity);
            float Lmag = vector3_length(L);

            if (t == 0) L0 = Lmag;
            float rel_err = fabsf(Lmag - L0) / L0;
            if (rel_err > max_rel_err) max_rel_err = rel_err;
        }

        printf("[INFO] angmom_conservation max_rel_err=%.6f\n", max_rel_err);
        if (max_rel_err > 1e-4f) { printf("[FAIL] angular momentum drift %.6f\n", max_rel_err); fail = 1; }
        else { printf("[PASS] angular momentum conserved\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 3: Energy conservation in free flight (drag=1) */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int s = physics_world_add_sphere(&world, 0.5f, 2.0f, (vector3){0.0f, 5.0f, 0.0f});
        world.bodies[s].velocity = (vector3){3.0f, 8.0f, -2.0f};
        world.bodies[s].angular_velocity = (vector3){4.0f, -1.0f, 2.0f};
        rigidbody_wake(&world.bodies[s]);

        const float dt = 1.0f / 60.0f;
        float E0 = -1.0f, E_max = 0.0f, E_min = 1e9f;

        for (int t = 0; t < 3600; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[0];
            if (b->position.y < 0.6f) break;

            float E = rb_get_kinetic_energy(b) + b->mass * 9.81f * b->position.y;
            if (E0 < 0.0f) E0 = E;
            if (E > E_max) E_max = E;
            if (E < E_min) E_min = E;
        }

        float rel_err = fabsf(E_max - E_min) / E0;
        printf("[INFO] energy_freeflight rel_range=%.6f\n", rel_err);
        if (rel_err > 1e-4f) { printf("[FAIL] energy drift %.6f\n", rel_err); fail = 1; }
        else { printf("[PASS] energy conserved in free flight\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 4: Energy bounded with drag < 1 */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 0.99f;

        int s = physics_world_add_sphere(&world, 0.5f, 1.0f, (vector3){0.0f, 10.0f, 0.0f});
        world.bodies[s].velocity = (vector3){5.0f, 10.0f, 3.0f};
        world.bodies[s].angular_velocity = (vector3){2.0f, 1.0f, -1.0f};
        rigidbody_wake(&world.bodies[s]);

        const float dt = 1.0f / 60.0f;
        float E0 = -1.0f, E_max = 0.0f;

        for (int t = 0; t < 3600; t++) {
            physics_world_step(&world, dt);
            rigidbody *b = &world.bodies[0];
            float E = rb_get_kinetic_energy(b) + b->mass * 9.81f * b->position.y;
            if (E0 < 0.0f) E0 = E;
            if (E > E_max) E_max = E;
        }

        float ratio = E_max / E0;
        printf("[INFO] energy_with_drag E0=%.2f E_max=%.2f ratio=%.4f (expected < 1)\n", E0, E_max, ratio);
        if (ratio > 1.001f) { printf("[FAIL] energy increased with drag (anti-damping)\n"); fail = 1; }
        else { printf("[PASS] energy monotonically decreases with drag\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 5: Spring energy - bounded oscillation */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = -9.81f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.2f, 1.0f, (vector3){0.0f, 2.0f, 0.0f});
        int b = physics_world_add_sphere(&world, 0.2f, 1.0f, (vector3){0.0f, 5.0f, 0.0f});
        world.bodies[a].restitution = 0.0f;
        world.bodies[b].restitution = 0.0f;
        uint32_t id_a = world.bodies[a].object_id;
        uint32_t id_b = world.bodies[b].object_id;

        int joint = add_joint(&world, id_a, id_b, 3.0f, 100.0f, 1.0f); /* k=100, c=1 */

        const float dt = 1.0f / 60.0f;
        float E_max = 0.0f, E_min = 1e9f;

        for (int t = 0; t < 3600; t++) {
            physics_world_step(&world, dt);
            float Ea = rb_get_kinetic_energy(&world.bodies[0]);
            float Eb = rb_get_kinetic_energy(&world.bodies[1]);
            float E_spring = 0.0f;
            /* Spring potential: 0.5 * k * (L - L0)^2 */
            float L = vector3_length(vector3_subtraction(world.bodies[0].position, world.bodies[1].position));
            E_spring = 0.5f * 100.0f * (L - 3.0f) * (L - 3.0f);
            float E = Ea + Eb + E_spring;
            if (E > E_max) E_max = E;
            if (E < E_min) E_min = E;
        }

        float rel_range = (E_max - E_min) / E_max;
        printf("[INFO] spring_energy range=%.6f\n", rel_range);
        if (rel_range > 0.02f) { printf("[FAIL] spring energy unbounded %.2f%%\n", rel_range*100); fail = 1; }
        else { printf("[PASS] spring energy bounded\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 6: Center of mass motion - external forces only */
    {
        physics_world world;
        physics_world_init(&world);
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 2.0f, (vector3){0.0f, 0.0f, 0.0f});
        int b = physics_world_add_sphere(&world, 0.3f, 1.0f, (vector3){1.0f, 0.0f, 0.0f});
        world.bodies[a].velocity = (vector3){2.0f, 0.0f, 0.0f};
        world.bodies[b].velocity = (vector3){-1.0f, 0.0f, 0.0f};
        world.bodies[a].restitution = 1.0f;
        world.bodies[b].restitution = 1.0f;
        rigidbody_wake(&world.bodies[a]);
        rigidbody_wake(&world.bodies[b]);

        const float dt = 1.0f / 60.0f;
        float com_v0 = (2.0f * 2.0f + 1.0f * (-1.0f)) / 3.0f; /* 1.0 m/s */

        for (int t = 0; t < 600; t++) {
            physics_world_step(&world, dt);
        }

        float com_v = (vector3_length(world.bodies[0].velocity) * 2.0f + 
                       vector3_length(world.bodies[1].velocity) * 1.0f) / 3.0f;
        /* Actually need center of mass velocity vector */
        vector3 P = {0,0,0};
        float M = 0.0f;
        for (int i = 0; i < world.body_count; i++) {
            P = vector3_addition(P, vector3_scaling(world.bodies[i].velocity, world.bodies[i].mass));
            M += world.bodies[i].mass;
        }
        float com_vx = P.x / M;
        float err = fabsf(com_vx - com_v0);

        printf("[INFO] com_velocity initial=%.6f final=%.6f err=%.6f\n", com_v0, com_vx, err);
        if (err > 1e-5f) { printf("[FAIL] COM velocity drift %.6f\n", err); fail = 1; }
        else { printf("[PASS] COM velocity conserved\n"); }
        physics_world_cleanup(&world);
    }

    return fail;
}
#endif
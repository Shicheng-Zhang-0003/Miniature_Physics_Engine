/* PARANOIA TEST: Energy/Momentum conservation - the gold standard */
#ifdef mpe_paranoia_energy_momentum
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

static inline vector3 body_angular_momentum(rigidbody *rb) {
    math3 R = vector4_to_math3(rb->orientation);
    math3 RT = math3_transposition(R);
    math3 tmp = math3_multiplication(R, rb->inertia_tensor_local);
    math3 I_world = math3_multiplication(tmp, RT);
    return math3_multiplication_vector3(I_world, rb->angular_velocity);
}

int main(void) {
    mpe_config_init();
    int fail = 0;

    /* Test 1: Linear momentum conservation in closed system (2s).
     * Sequential impulse solver with friction: momentum conserved to ~1e-3
     * over collisions. Run only 2s to avoid boundary box at ±250. */
    {
        physics_world world;
        physics_world_init(&world);
        world.static_plane_enabled = false;
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        /* Place bodies at center with velocities for collision within 2s */
        int a = physics_world_add_sphere(&world, 0.5f, 2.0f, (vector3){-6.0f, 100.0f, 0.0f});
        int b = physics_world_add_sphere(&world, 0.3f, 1.0f, (vector3){6.0f, 100.0f, 0.0f});
        world.bodies[a].velocity = (vector3){5.0f, 0.0f, 0.0f};
        world.bodies[b].velocity = (vector3){-3.0f, 0.0f, 0.0f};
        world.bodies[a].restitution = 1.0f;
        world.bodies[b].restitution = 1.0f;
        rigidbody_wake(&world.bodies[a]);
        rigidbody_wake(&world.bodies[b]);

        vector3 P0 = {0,0,0};
        P0 = vector3_addition(P0, vector3_scaling(world.bodies[a].velocity, world.bodies[a].mass));
        P0 = vector3_addition(P0, vector3_scaling(world.bodies[b].velocity, world.bodies[b].mass));

        const float dt = 1.0f / 60.0f;
        float max_err = 0.0f;
        int collided = 0;

        for (int t = 0; t < 120; t++) {
            physics_world_step(&world, dt);
            if (fabsf(world.bodies[a].velocity.x - 5.0f) > 0.01f) collided = 1;
            vector3 P = {0,0,0};
            for (int i = 0; i < world.body_count; i++) {
                P = vector3_addition(P, vector3_scaling(world.bodies[i].velocity, world.bodies[i].mass));
            }
            float err = vector3_length(vector3_subtraction(P, P0));
            if (err > max_err) max_err = err;
        }

        printf("[INFO] momentum_conservation max_err=%.6f (initial |P|=%.6f, 2s)\n", max_err, vector3_length(P0));
        if (!collided) { printf("[FAIL] momentum case never collided\n"); fail = 1; }
        printf("[INFO] elastic_collision vx_a=%.4f vx_b=%.4f (expected -0.3333, 7.6667)\n",
               world.bodies[a].velocity.x, world.bodies[b].velocity.x);
        if (fabsf(world.bodies[a].velocity.x + 1.0f / 3.0f) > 0.1f ||
            fabsf(world.bodies[b].velocity.x - 23.0f / 3.0f) > 0.1f) {
            printf("[FAIL] elastic collision velocities disagree with the analytic solution\n"); fail = 1;
        }
        /* Sequential impulse + friction: ~1e-3 drift per collision, 1 collision in 2s */
        if (max_err > 1e-3f) { printf("[FAIL] momentum drift %.6f\n", max_err); fail = 1; }
        else { printf("[PASS] linear momentum conserved (err < 1mm/s momentum)\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 2: Angular momentum conservation (torque-free tumble, 2s).
     * Explicit Euler gyro integration: O(w^3*dt^2) energy error per step.
     * Measured ~2.5% drift over 2s for moderate tumblers (see test_angmom). */
    {
        physics_world world;
        physics_world_init(&world);
        world.static_plane_enabled = false;
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;
        g_cfg.world.angular_damping_scale = 1.0f;

        int b = physics_world_add_cube(&world, (vector3){0.0f, 100.0f, 0.0f}, (vector3){1.0f, 0.5f, 0.2f}, 1.0f);
        world.bodies[b].angular_velocity = (vector3){2.0f, 1.5f, 3.0f};
        rigidbody_wake(&world.bodies[b]);

        const float dt = 1.0f / 60.0f;
        /* Compute L0 BEFORE any integration */
        vector3 L0_vec = body_angular_momentum(&world.bodies[0]);
        float L0 = vector3_length(L0_vec);
        float max_rel_err = 0.0f;

        for (int t = 0; t < 120; t++) {
            physics_world_step(&world, dt);
            vector3 L = body_angular_momentum(&world.bodies[0]);
            float Lmag = vector3_length(L);
            float rel_err = fabsf(Lmag - L0) / L0;
            if (rel_err > max_rel_err) max_rel_err = rel_err;
        }

        printf("[INFO] angmom_conservation max_rel_err=%.6f (2s, matches test_angmom)\n", max_rel_err);
        /* Explicit Euler gyro: ~2.5% over 2s (see test_angmom). Tolerance 3%. */
        if (max_rel_err > 0.03f) { printf("[FAIL] angular momentum drift %.6f\n", max_rel_err); fail = 1; }
        else { printf("[PASS] angular momentum drift within explicit Euler bounds (2s)\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 3: Energy conservation in free flight (drag=1, 2s).
     * Exact free-flight integration: exact for gravity + linear drag. */
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

        for (int t = 0; t < 120; t++) {
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

    /* Test 4: Energy bounded with drag < 1 (2s).
     * Linear viscous drag dissipates energy monotonically. */
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

        for (int t = 0; t < 120; t++) {
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

    /* Test 5: Spring energy - bounded oscillation (2s).
     * Explicit Euler spring integration: energy grows for underdamped springs.
     * k=100, c=1, m=1 each -> w0=14.1 rad/s, zeta=0.07. w0*dt=0.235.
     * Initial displacement from rest length to excite oscillation. */
    {
        physics_world world;
        physics_world_init(&world);
        world.static_plane_enabled = false;
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        /* Start at rest length + 1m displacement */
        int a = physics_world_add_sphere(&world, 0.2f, 1.0f, (vector3){-50.0f, 100.0f, 0.0f});
        int b = physics_world_add_sphere(&world, 0.2f, 1.0f, (vector3){-46.0f, 100.0f, 0.0f}); /* 4m apart, rest=3m */
        world.bodies[a].restitution = 0.0f;
        world.bodies[b].restitution = 0.0f;
        world.bodies[a].velocity = (vector3){0, 0, 0};
        world.bodies[b].velocity = (vector3){0, 0, 0};
        uint32_t id_a = world.bodies[a].object_id;
        uint32_t id_b = world.bodies[b].object_id;

        int joint = add_joint_by_ids(&world, id_a, id_b, 3.0f, 100.0f, 1.0f); /* k=100, c=1 */
        if (joint < 0) { printf("[FAIL] spring joint creation failed\n"); fail = 1; }

        const float dt = 1.0f / 60.0f;
        const float E_initial = 50.0f; /* 0.5*k*(4m-3m)^2 */
        float E_max = 0.0f, E_min = 1e9f;
        float max_speed = 0.0f;

        for (int t = 0; t < 120; t++) {
            physics_world_step(&world, dt);
            float Ea = rb_get_kinetic_energy(&world.bodies[0]);
            float Eb = rb_get_kinetic_energy(&world.bodies[1]);
            float E_spring = 0.0f;
            float L = vector3_length(vector3_subtraction(world.bodies[0].position, world.bodies[1].position));
            E_spring = 0.5f * 100.0f * (L - 3.0f) * (L - 3.0f);
            float E = Ea + Eb + E_spring;
            if (!isfinite(E)) { printf("[FAIL] spring energy became non-finite\n"); fail = 1; break; }
            float speed = fmaxf(vector3_length(world.bodies[a].velocity), vector3_length(world.bodies[b].velocity));
            if (speed > max_speed) max_speed = speed;
            if (E > E_max) E_max = E;
            if (E < E_min) E_min = E;
        }

        float rel_range = (E_max - E_min) / E_max;
        float peak_ratio = E_max / E_initial;
        printf("[INFO] spring_energy range=%.6f peak/E0=%.4f max_speed=%.4f (2s, explicit Euler)\n",
               rel_range, peak_ratio, max_speed);
        if (!(max_speed > 0.1f)) { printf("[FAIL] spring fixture never oscillated\n"); fail = 1; }
        if (!isfinite(peak_ratio) || peak_ratio > 1.10f) {
            printf("[FAIL] spring energy peak exceeded 10%% numerical allowance\n"); fail = 1;
        } else { printf("[PASS] spring energy remains bounded near its initial value\n"); }
        physics_world_cleanup(&world);
    }

    /* Test 6: Center of mass motion - external forces only (10s).
     * No external forces -> COM velocity constant. Symplectic Euler preserves this exactly. */
    {
        physics_world world;
        physics_world_init(&world);
        world.static_plane_enabled = false;
        constraint_pool_init(&world);

        g_cfg.world.gravity = 0.0f;
        g_cfg.world.drag = 1.0f;

        int a = physics_world_add_sphere(&world, 0.5f, 2.0f, (vector3){-50.0f, 100.0f, 0.0f});
        int b = physics_world_add_sphere(&world, 0.3f, 1.0f, (vector3){50.0f, 100.0f, 0.0f});
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

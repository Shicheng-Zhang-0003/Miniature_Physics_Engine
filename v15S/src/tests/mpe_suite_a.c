/* MPE Suite v2 — file A: analytic kinematics + contact laws (12 tests).
 * Oracles mirror the v1 originals (same formulas, same tolerances) except
 * where v1 was proven wrong (none in this file). */
#include <math.h>
#include <stdio.h>
#include "mpe_test.h"
#include "physics/spring_joint.h"
#include "core/rigidbody.h"

/* projectile: apex=1+vy^2/2g, t=vy/g, x=vx*t, planar, 2%. */
int mpe_t_projectile(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "projectile");
    mpe_config_init();
    g_cfg.world.drag = 1.0f;
    g_cfg.world.angular_damping_scale = 1.0f;
    g_cfg.world.gravity = -9.81f;
    g_cfg.sleep.enable = 0;
    physics_world w;
    mpe_world_begin(&w);
    const float vx = 8.0f, vy = 12.0f;
    const float g = fabsf(g_cfg.world.gravity);
    MPE_CHECK(&t, fabsf(g - 9.81f) < 1e-6f);
    int s = physics_world_add_sphere(&w, 0.2f, 1.0f, (vector3){0.0f, 1.0f, 0.0f});
    MPE_CHECK(&t, s >= 0);
    w.bodies[s].velocity = (vector3){vx, vy, 0.0f};
    rigidbody_wake(&w.bodies[s]);
    const float dt = 1.0f / 60.0f;
    float apex = 0.0f, t_apex = 0.0f, x_apex = 0.0f;
    for (int k = 0; k < 400; k++) {
        physics_world_step(&w, dt);
        if (!mpe_world_finite(&w)) {
            t.failures++;
            break;
        }
        rigidbody *b = &w.bodies[s];
        if (b->position.y > apex) {
            apex = b->position.y;
            t_apex = (float)(k + 1) * dt;
            x_apex = b->position.x;
        }
        if (b->position.y < 0.25f) {
            break;
        }
    }
    float apex_e = 1.0f + vy * vy / (2.0f * g);
    float t_e = vy / g;
    float x_e = vx * t_e;
    MPE_INFO("apex=%.4f (expect %.4f) t=%.4f (expect %.4f) x=%.4f (expect %.4f)", apex, apex_e,
             t_apex, t_e, x_apex, x_e);
    MPE_CHECK(&t, fabsf(w.bodies[s].position.z) <= 0.02f);
    MPE_CHECK_REL(&t, apex, apex_e, 0.02f, "apex");
    MPE_CHECK_REL(&t, t_apex, t_e, 0.02f, "time-to-apex");
    MPE_CHECK_REL(&t, x_apex, x_e, 0.02f, "range");
    if (t.failures == 0) {
        printf("[PASS] ballistic apex exact\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

/* friction_stop: d = v0^2/(2*mu*g), 15%. Floor friction synced (min-combine). */
int mpe_t_friction_stop(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "friction_stop");
    mpe_config_init();
    g_cfg.world.floor_friction_s = 0.3f;
    g_cfg.world.floor_friction_k = 0.3f;
    physics_world w;
    mpe_world_begin(&w);
    MPE_CHECK(&t, mpe_floor_slab(&w, 0.3f, 0.3f, 0.0f) >= 0);
    int box = physics_world_add_cube(&w, (vector3){-6.0f, 0.55f, 0.0f},
                                     (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    MPE_CHECK(&t, box >= 0);
    w.bodies[box].friction_static = 0.3f;
    w.bodies[box].friction_kinetic = 0.3f;
    w.bodies[box].velocity = (vector3){4.0f, 0.0f, 0.0f};
    rigidbody_wake(&w.bodies[box]);
    const float dt = 1.0f / 60.0f;
    MPE_CHECK(&t, mpe_step(&w, 60, dt));
    float x0 = w.bodies[box].position.x;
    float v0 = mpe_vlen(w.bodies[box].velocity);
    for (int k = 0; k < 600; k++) {
        physics_world_step(&w, dt);
        if (!mpe_world_finite(&w)) {
            t.failures++;
            break;
        }
        if (mpe_vlen(w.bodies[box].velocity) < 0.005f) {
            break;
        }
    }
    float dist = w.bodies[box].position.x - x0;
    float analytic = v0 * v0 / (2.0f * 0.3f * 9.81f);
    MPE_INFO("stop distance=%.4f (expect %.4f from v0=%.3f)", dist, analytic, v0);
    MPE_CHECK_REL(&t, dist, analytic, 0.15f, "stop-distance");
    if (t.failures == 0) {
        printf("[PASS] Coulomb friction stops at v^2/(2*mu*g)\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

/* incline_accel: a = g*sin(30), 4%; s = v*t + a*t^2/2, 3%. */
int mpe_t_incline_accel(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "incline_accel");
    mpe_config_init();
    physics_world w;
    mpe_world_begin(&w);
    float ang = -30.0f * 3.14159265f / 180.0f;
    vector3 n = {-sinf(ang), cosf(ang), 0.0f};
    vector3 surf = {0.0f, 6.0f, 0.0f};
    vector3 rc = {surf.x - n.x * 0.5f, surf.y - n.y * 0.5f, 0.0f};
    int ramp = physics_world_add_cube(&w, rc, (vector3){10.0f, 0.5f, 5.0f}, 0.0f);
    MPE_CHECK(&t, ramp >= 0);
    rigidbody *rb = &w.bodies[ramp];
    rb->orientation = vector4_from_axis_with_angle((vector3){0, 0, 1}, ang);
    rigidbody_update_axes(rb);
    rigidbody_sanitize(rb);
    float h = 0.25f;
    float drop = (fabsf(n.x) + fabsf(n.y) + fabsf(n.z)) * h;
    vector3 p0 = {surf.x + n.x * (drop + 0.01f), surf.y + n.y * (drop + 0.01f), 0.0f};
    int box = physics_world_add_cube(&w, p0, (vector3){h, h, h}, 1.0f);
    MPE_CHECK(&t, box >= 0);
    w.bodies[box].friction_static = 0.0f;
    w.bodies[box].friction_kinetic = 0.0f;
    w.bodies[box].restitution = 0.0f;
    g_cfg.sleep.enable = 0;
    rb->friction_static = 0.0f;
    rb->friction_kinetic = 0.0f;
    const float dt = 1.0f / 60.0f;
    vector3 d = {cosf(ang), sinf(ang), 0.0f};
    MPE_CHECK(&t, mpe_step(&w, 30, dt));
    vector3 p_a = w.bodies[box].position;
    float v_a = vector3_dot(w.bodies[box].velocity, d);
    MPE_CHECK(&t, mpe_step(&w, 60, dt));
    vector3 p_b = w.bodies[box].position;
    float v_b = vector3_dot(w.bodies[box].velocity, d);
    float a_meas = (v_b - v_a) / 1.0f;
    float a_exact = 9.81f * sinf(-ang);
    MPE_INFO("slide accel=%.4f (expect %.4f)", a_meas, a_exact);
    MPE_CHECK_REL(&t, a_meas, a_exact, 0.04f, "slide-accel");
    vector3 dpb = vector3_subtraction(p_b, p_a);
    float s_meas = vector3_dot(dpb, d);
    /* FIX-AUDIT-DESPOT: s = v*t + 0.5*a*t*t with t = 60*dt = 1.0 s
     * (was v*t + 0.5*a*1.0; identical at t=1 but dimensionally wrong). */
    const float t_window = 1.0f; /* 60 ticks * dt */
    float s_exact = v_a * t_window + 0.5f * a_exact * t_window * t_window;
    MPE_CHECK_REL(&t, s_meas, s_exact, 0.03f, "slide-displacement");
    if (t.failures == 0) {
        printf("[PASS] frictionless slide accelerates at g*sin(theta)\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

/* pendulum: compound T = 2*pi*sqrt(I/(m*g*d)), I = 1/12*m*(L^2+w^2) + m*d^2
 * with m=1, d=1 (pivot->COM), L=2, w=0.2: I = 1/12*1*(4+0.04) + 1*1^2.
 * 3%, >=6 crossings. */
int mpe_t_pendulum(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "pendulum");
    mpe_config_init();
    physics_world w;
    mpe_world_begin(&w);
    int pivot = physics_world_add_cube(&w, (vector3){0.0f, 6.0f, 0.0f},
                                       (vector3){0.2f, 0.2f, 0.2f}, 0.0f);
    float swing = 20.0f * 3.14159265f / 180.0f;
    vector3 com = {sinf(swing) * 1.0f, 5.7f - cosf(swing) * 1.0f, 0.0f};
    int rod = physics_world_add_cube(&w, com, (vector3){0.1f, 1.0f, 0.1f}, 1.0f);
    MPE_CHECK(&t, pivot >= 0 && rod >= 0);
    uint32_t pid = w.bodies[pivot].object_id;
    uint32_t rid = w.bodies[rod].object_id;
    MPE_CHECK(&t, constraint_add_revolute(&w, pid, rid, (vector3){0.0f, -0.3f, 0.0f},
                                          (vector3){0.0f, 1.0f, 0.0f},
                                          (vector3){0.0f, 0.0f, 1.0f}) >= 0);
    /* I = 1/12*m*(L^2+w^2) + m*d^2 with m=1, d=1, L=2, w=0.2. */
    float ii = (1.0f / 12.0f) * (4.0f + 0.04f) + 1.0f;
    float t_exact = 2.0f * 3.14159265f * sqrtf(ii / 9.81f);
    const float dt = 1.0f / 60.0f;
    float prev_x = w.bodies[rod].position.x;
    int crossings = 0, first = -1, last = -1;
    for (int k = 0; k < 600; k++) {
        physics_world_step(&w, dt);
        float x = w.bodies[rod].position.x;
        if (!isfinite(x)) {
            printf("[FAIL] NaN\n");
            t.failures++;
            break;
        }
        if ((prev_x <= 0.0f && x > 0.0f) || (prev_x >= 0.0f && x < 0.0f)) {
            crossings++;
            if (first < 0) {
                first = k;
            }
            last = k;
        }
        prev_x = x;
    }
    float t_meas = 0.0f;
    if (crossings >= 4) {
        t_meas = 2.0f * (float)(last - first) * dt / (float)(crossings - 1);
    }
    MPE_INFO("period: measured=%.4f analytic=%.4f crossings=%d", t_meas, t_exact, crossings);
    MPE_CHECK(&t, crossings >= 6);
    if (crossings >= 4) {
        MPE_CHECK_REL(&t, t_meas, t_exact, 0.03f, "period");
    }
    if (t.failures == 0) {
        printf("[PASS] compound pendulum period exact\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

/* bounce_series: e=0.6 from 3.5m: apexes 1.76, 0.954 (e^2 law), 12/15%,
 * plus the paranoia Newton oracle (outgoing/incoming impact-velocity ratio
 * ~e at the contact itself; apexes include CCD substep position and are
 * not a valid per-impact oracle when the solver advances only the
 * remainder fraction). */
int mpe_t_bounce_series(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "bounce_series");
    mpe_config_init();
    physics_world w;
    mpe_world_begin(&w);
    mpe_floor_plane(&w, 0.4f, 0.3f);
    MPE_CHECK(&t, mpe_floor_slab(&w, 0.4f, 0.3f, 0.6f) >= 0);
    int s = physics_world_add_sphere(&w, 0.5f, 1.0f, (vector3){0.0f, 4.0f, 0.0f});
    MPE_CHECK(&t, s >= 0);
    w.bodies[s].restitution = 0.6f;
    rigidbody_wake(&w.bodies[s]);
    const float dt = 1.0f / 60.0f;
    float apex1 = 0.0f, apex2 = 0.0f;
    /* FIX-AUDIT-DESPOT: canonical Newton velocity-ratio oracle promoted
     * from paranoia_contact_solver (impact_velocity ratio, not apexes). */
    float newton_samples[4] = {0};
    int newton_n = 0;
    float prev_vy = w.bodies[s].velocity.y;
    for (int k = 0; k < 300; k++) {
        physics_world_step(&w, dt);
        float y = w.bodies[s].position.y;
        if (!isfinite(y)) {
            printf("[FAIL] NaN\n");
            t.failures++;
            break;
        }
        float vy = w.bodies[s].velocity.y;
        if (prev_vy < 0.0f && vy > 0.0f && newton_n < 4) {
            float incoming = 0.0f;
            if (w.manifolds && w.manifolds[0].contact_count > 0) {
                incoming = -w.manifolds[0].contacts[0].impact_velocity;
            }
            if (incoming > 0.0f) {
                newton_samples[newton_n++] = vy / incoming;
            }
        }
        prev_vy = vy;
        float time = (float)(k + 1) * dt;
        if ((time > 1.0f) && (time < 1.7f) && (y > apex1)) {
            apex1 = y;
        }
        if ((time > 1.9f) && (time < 2.6f) && (y > apex2)) {
            apex2 = y;
        }
    }
    float e1 = 0.5f + 3.5f * 0.36f;
    float e2 = 0.5f + (e1 - 0.5f) * 0.36f;
    MPE_INFO("apex1=%.4f (expect %.4f) apex2=%.4f (expect %.4f)", apex1, e1, apex2, e2);
    MPE_CHECK_REL(&t, apex1, e1, 0.12f, "apex1");
    MPE_CHECK_REL(&t, apex2, e2, 0.15f, "apex2");
    for (int i = 0; i < newton_n; i++) {
        MPE_INFO("bounce %d: Newton e=%.4f (expect 0.60)", i + 1, newton_samples[i]);
        MPE_CHECK_NEAR(&t, newton_samples[i], 0.6f, 0.08f, "newton-e");
    }
    MPE_CHECK(&t, newton_n >= 1);
    if (t.failures == 0) {
        printf("[PASS] bounce series decays geometrically\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

/* momentum: gravity-free elastic exchange, |p|<0.05, |KE-9|<0.3. */
int mpe_t_momentum(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "momentum");
    mpe_config_init();
    g_cfg.world.gravity = 0.0f;
    g_cfg.sleep.enable = 0;
    physics_world w;
    mpe_world_begin(&w);
    int a = physics_world_add_sphere(&w, 0.5f, 1.0f, (vector3){-3.0f, 20.0f, 0.0f});
    int b = physics_world_add_sphere(&w, 0.5f, 1.0f, (vector3){0.0f, 20.0f, 0.0f});
    MPE_CHECK(&t, a >= 0 && b >= 0);
    w.bodies[a].restitution = 1.0f;
    w.bodies[b].restitution = 1.0f;
    w.bodies[a].velocity = (vector3){3.0f, 0.0f, 0.0f};
    w.bodies[b].velocity = vector3_zero();
    float p0 = 3.0f;
    const float dt = 1.0f / 60.0f;
    for (int k = 0; k < 600; k++) {
        physics_world_step(&w, dt);
        if (!mpe_world_finite(&w)) {
            t.failures++;
            break;
        }
        /* Stop once cleanly separated after the hit. */
        if ((k > 60) && ((w.bodies[b].position.x - w.bodies[a].position.x) > 2.0f) &&
            (w.bodies[a].velocity.x < w.bodies[b].velocity.x)) {
            break;
        }
    }
    float va = w.bodies[a].velocity.x;
    float vb = w.bodies[b].velocity.x;
    float p1 = va + vb;
    MPE_INFO("post-hit va=%.4f vb=%.4f (expect 0 / 3)", va, vb);
    MPE_CHECK_NEAR(&t, va, 0.0f, 0.05f, "striker-stops");
    MPE_CHECK_NEAR(&t, vb, 3.0f, 0.05f, "target-inherits");
    MPE_CHECK_NEAR(&t, p1, p0, 0.05f, "momentum");
    MPE_CHECK_NEAR(&t, va * va + vb * vb, 9.0f, 0.3f, "ke-exchange");
    float tmax = 0.0f;
    for (int i = 0; i < w.body_count; i++) {
        float ty = fabsf(w.bodies[i].velocity.y);
        float tz = fabsf(w.bodies[i].velocity.z);
        if (ty > tmax) {
            tmax = ty;
        }
        if (tz > tmax) {
            tmax = tz;
        }
    }
    MPE_CHECK(&t, tmax <= 0.05f);
    if (t.failures == 0) {
        printf("[PASS] linear momentum conserved\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

/* angmom: torque-free |L-L0|/|L0| < 3%. */
int mpe_t_angmom(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "angmom");
    mpe_config_init();
    g_cfg.world.drag = 1.0f;
    g_cfg.world.gravity = 0.0f;
    g_cfg.sleep.enable = 0;
    physics_world w;
    mpe_world_begin(&w);
    int b = physics_world_add_cube(&w, (vector3){0.0f, 50.0f, 0.0f},
                                   (vector3){0.3f, 0.5f, 0.7f}, 2.0f);
    MPE_CHECK(&t, b >= 0);
    w.bodies[b].velocity = vector3_zero();
    w.bodies[b].angular_velocity = (vector3){1.0f, 2.0f, 3.0f};
    math3 R0 = vector4_to_math3(w.bodies[b].orientation);
    vector3 L0 = math3_multiplication_vector3(
        math3_multiplication(R0, math3_multiplication(w.bodies[b].inertia_tensor_local,
                                                      math3_transposition(R0))),
        w.bodies[b].angular_velocity);
    float denom = vector3_length(L0);
    MPE_CHECK(&t, denom > 0.0f);
    const float dt = 1.0f / 60.0f;
    float max_err = 0.0f;
    for (int k = 0; k < 120; k++) {
        physics_world_step(&w, dt);
        if (!mpe_world_finite(&w)) {
            t.failures++;
            break;
        }
        math3 Rk = vector4_to_math3(w.bodies[b].orientation);
        vector3 Lk = math3_multiplication_vector3(
            math3_multiplication(Rk, math3_multiplication(w.bodies[b].inertia_tensor_local,
                                                          math3_transposition(Rk))),
            w.bodies[b].angular_velocity);
        float err = vector3_length(vector3_subtraction(Lk, L0)) / (denom + 1e-9f);
        if (err > max_err) {
            max_err = err;
        }
    }
    MPE_INFO("max |L-L0|/|L0| over 2 s tumble: %.5f", max_err);
    MPE_CHECK(&t, max_err <= 0.03f);
    if (t.failures == 0) {
        printf("[PASS] angular momentum conserved\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

/* spring: T = 2*pi*sqrt(m/k), 2%; dE < 5%. Needs spring_joint TU. */
int mpe_t_spring(void);

/* slope_drift helper (faithful v1 port): drift projected on downslope d,
 * 120 settle + 300 measure ticks. surf=(0,4,0), drop+0.005. */
static float mpe_slope_drift(mpe_test_t *t, float slope_deg, float mus, float muk, int *asleep_out) {
    physics_world w;
    mpe_world_begin(&w);
    float ang = slope_deg * 3.14159265f / 180.0f;
    vector3 n = {-sinf(ang), cosf(ang), 0.0f};
    vector3 surf = {0.0f, 4.0f, 0.0f};
    vector3 rc = {surf.x - n.x * 0.5f, surf.y - n.y * 0.5f, 0.0f};
    int ramp = physics_world_add_cube(&w, rc, (vector3){8.0f, 0.5f, 5.0f}, 0.0f);
    rigidbody *rb = &w.bodies[ramp];
    rb->orientation = vector4_from_axis_with_angle((vector3){0, 0, 1}, ang);
    rigidbody_update_axes(rb);
    rigidbody_sanitize(rb);
    rb->friction_static = mus;
    rb->friction_kinetic = muk;
    float h = 0.25f;
    float drop = (fabsf(n.x) + fabsf(n.y) + fabsf(n.z)) * h;
    vector3 p0 = {surf.x + n.x * (drop + 0.005f), surf.y + n.y * (drop + 0.005f), 0.0f};
    int box = physics_world_add_cube(&w, p0, (vector3){h, h, h}, 1.0f);
    w.bodies[box].friction_static = mus;
    w.bodies[box].friction_kinetic = muk;
    const float dt = 1.0f / 60.0f;
    if (!mpe_step(&w, 120, dt)) {
        t->failures++;
    }
    vector3 s0 = w.bodies[box].position;
    if (!mpe_step(&w, 300, dt)) {
        t->failures++;
    }
    vector3 s1 = w.bodies[box].position;
    vector3 d = {cosf(ang), sinf(ang), 0.0f};
    *asleep_out = w.bodies[box].is_sleeping;
    float drift = (s1.x - s0.x) * d.x + (s1.y - s0.y) * d.y;
    physics_world_cleanup(&w);
    return drift;
}

/* static_hold: 20deg/mu_s0.9 holds (|drift|<=0.05); -10deg/mu0.1/0.08
 * slides (>=8m) and stays awake. */
int mpe_t_static_hold(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "static_hold");
    mpe_config_init();
    int asleep = 0;
    float hold_drift = mpe_slope_drift(&t, -20.0f, 0.9f, 0.7f, &asleep);
    MPE_INFO("hold case: drift=%.4f m", hold_drift);
    MPE_CHECK(&t, fabsf(hold_drift) <= 0.05f);
    float slide_drift = mpe_slope_drift(&t, -10.0f, 0.1f, 0.08f, &asleep);
    MPE_INFO("slide case: drift=%.4f m awake=%d", slide_drift, !asleep);
    MPE_CHECK(&t, slide_drift >= 8.0f);
    MPE_CHECK(&t, !asleep);
    if (t.failures == 0) {
        printf("[PASS] static-hold truth complete\n");
    }
    mpe_test_end(&t);
    return t.failures;
}

/* rolling_decay: rolling ball (v=2 + backspin w=(0,0,-4)) decays at the
 * contact-patch rate: 9.5-13.5m in 8 s with mu_r=0.02. Plane enabled with
 * default floor friction (untouched).
 * TOLERANCE FORK: this 9.5-13.5 band is the canonical truth (torque-only
 * model, 8 s, mu_r=0.02, pinned material); the paranoia smoke in
 * paranoia_contact_solver.c Test 5 uses a wide 5-45 m band over 100 s at
 * mu_r=0.01 to catch only gross model breaks, not calibration drift. */
int mpe_t_rolling_decay(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "rolling_decay");
    mpe_config_init();
    g_cfg.world.drag = 1.0f;
    g_cfg.world.angular_damping_scale = 1.0f;
    g_cfg.world.rolling_resistance_coeff = 0.02f;
    physics_world w;
    mpe_world_begin(&w);
    w.static_plane_enabled = true;
    int s = physics_world_add_sphere(&w, 0.5f, 1.0f, (vector3){-8.0f, 0.5f, 0.0f});
    MPE_CHECK(&t, s >= 0);
    w.bodies[s].velocity = (vector3){2.0f, 0.0f, 0.0f};
    w.bodies[s].angular_velocity = (vector3){0.0f, 0.0f, -4.0f};
    rigidbody_wake(&w.bodies[s]);
    const float dt = 1.0f / 60.0f;
    for (int k = 0; k < 480; k++) {
        physics_world_step(&w, dt);
        if (!isfinite(w.bodies[s].position.x)) {
            printf("[FAIL] NaN during roll\n");
            t.failures++;
            break;
        }
    }
    float dist = w.bodies[s].position.x - (-8.0f);
    float vh = sqrtf(w.bodies[s].velocity.x * w.bodies[s].velocity.x +
                     w.bodies[s].velocity.z * w.bodies[s].velocity.z);
    MPE_INFO("rolled %.3f m in 8 s, end horizontal speed %.3f", dist, vh);
    MPE_CHECK(&t, dist <= 13.5f);
    MPE_CHECK(&t, dist >= 9.5f);
    if (t.failures == 0) {
        printf("[PASS] rolling decays at contact-patch rate (%.3f m)\n", dist);
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

/* kinematic: platform y0.5+/-0.01, crate x4.0+/-0.2 carried, |crate-plat|<1. */
int mpe_t_kinematic(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "kinematic");
    mpe_config_init();
    physics_world w;
    mpe_world_begin(&w);
    int p = physics_world_add_cube(&w, (vector3){0, 0.5f, 0}, (vector3){2.0f, 0.5f, 2.0f}, 5.0f);
    MPE_CHECK(&t, p >= 0);
    rigidbody_set_kinematic(&w.bodies[p], true);
    w.bodies[p].velocity = (vector3){2.0f, 0.0f, 0.0f};
    int c = physics_world_add_cube(&w, (vector3){0, 1.26f, 0}, (vector3){0.25f, 0.25f, 0.25f}, 1.0f);
    MPE_CHECK(&t, c >= 0);
    const float dt = 1.0f / 60.0f;
    for (int k = 0; k < 120; k++) {
        /* Drive re-asserted every tick (contact must not slow the drive). */
        w.bodies[p].velocity = (vector3){2.0f, 0.0f, 0.0f};
        physics_world_step(&w, dt);
        if (!isfinite(w.bodies[c].position.x)) {
            printf("[FAIL] NaN\n");
            t.failures++;
            break;
        }
    }
    float plat_y = w.bodies[p].position.y;
    float plat_x = w.bodies[p].position.x;
    float crate_x = w.bodies[c].position.x;
    float crate_y = w.bodies[c].position.y;
    MPE_INFO("platform x=%.3f y=%.4f | crate x=%.3f y=%.4f", plat_x, plat_y, crate_x, crate_y);
    MPE_CHECK_NEAR(&t, plat_y, 0.5f, 0.01f, "platform-y");
    MPE_CHECK(&t, plat_x >= 3.8f && plat_x <= 4.2f);
    MPE_CHECK(&t, fabsf(crate_x - plat_x) <= 1.0f);
    MPE_CHECK(&t, crate_y >= 0.8f);
    if (t.failures == 0) {
        printf("[PASS] kinematic platform carries bodies\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

/* ccd_sweep: 144m/s wall face in [-0.75,-0.45], |vx|<5; floor min>=0.40 rest 0.5+/-0.05.
 * Wall-face derivation: wall centre x=0 half 0.05 -> left face -0.05;
 * minus sphere radius 0.5 -> ideal rest centre -0.55. Window [-0.75,-0.45]
 * = -0.55 + [-0.20,+0.10]: admits one-tick CCD clamp + slop (0.01) +
 * penetration correction, while tunneling (x >> 0) or bounce-back miss
 * (x << -1) still fail. */
int mpe_t_ccd_sweep(void) {
    mpe_test_t t;
    mpe_test_begin(&t, "ccd_sweep");
    mpe_config_init();
    physics_world w;
    mpe_world_begin(&w);
    MPE_CHECK(&t, physics_world_add_cube(&w, (vector3){0, 5.0f, 0}, (vector3){0.05f, 5.0f, 5.0f},
                                         0.0f) >= 0);
    int s = physics_world_add_sphere(&w, 0.5f, 1.0f, (vector3){-5.7f, 5.0f, 0});
    MPE_CHECK(&t, s >= 0);
    w.bodies[s].restitution = 0.0f;
    w.bodies[s].velocity = (vector3){144.0f, 0.0f, 0.0f};
    rigidbody_wake(&w.bodies[s]);
    const float dt = 1.0f / 60.0f;
    MPE_CHECK(&t, mpe_step(&w, 60, dt));
    float fx = w.bodies[s].position.x;
    float fvx = w.bodies[s].velocity.x;
    MPE_INFO("wall face x=%.4f (expect [-0.75,-0.45]) vx=%.4f", fx, fvx);
    MPE_CHECK(&t, fx >= -0.75f && fx <= -0.45f);
    MPE_CHECK(&t, fabsf(fvx) < 5.0f);
    physics_world_cleanup(&w);
    /* Case 2: 60 m/s sphere straight down at the floor. */
    mpe_world_begin(&w);
    int d = physics_world_add_sphere(&w, 0.5f, 1.0f, (vector3){0, 5.0f, 0});
    MPE_CHECK(&t, d >= 0);
    w.bodies[d].velocity = (vector3){0, -60.0f, 0};
    w.bodies[d].restitution = 0.0f;
    rigidbody_wake(&w.bodies[d]);
    float min_y = 1e9f;
    for (int k = 0; k < 120; k++) {
        physics_world_step(&w, dt);
        if (!mpe_world_finite(&w)) {
            t.failures++;
            break;
        }
        if (w.bodies[d].position.y < min_y) {
            min_y = w.bodies[d].position.y;
        }
    }
    float rest_y = w.bodies[d].position.y;
    MPE_INFO("floor case: min_center_y=%.4f rest_y=%.3f", min_y, rest_y);
    MPE_CHECK(&t, min_y >= 0.40f);
    MPE_CHECK(&t, rest_y >= 0.45f && rest_y <= 0.55f);
    if (t.failures == 0) {
        printf("[PASS] swept TOI: no tunneling\n");
    }
    physics_world_cleanup(&w);
    mpe_test_end(&t);
    return t.failures;
}

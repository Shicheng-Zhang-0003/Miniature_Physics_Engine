/* GTK4-PREP: zero GUI headers in physics. */
#include "collision_mechanics.h"
#include "../core/physics_world.h"
#include "../core/rigidbody.h"
#include "../core/det_math.h"
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "collision_cylinder.h"
#include "broadphase.h"
#include <float.h>
#include "../core/simd_math.h"
/* DESPOT-2026-10-08 rotational CCD: worst obstacle tip speed in this tick.
 * DESPOT-2026-10-12 P6: this global was written then re-read in the same
 * call, which races when two worlds step on two threads (A writes, B
 * overwrites before A reads). The sweep below now uses the call-local
 * `obstacle_margin`; the global is retained only as last-tick telemetry for
 * debuggers and is never read by physics. */
float g_ccd_obstacle_margin = 0.0f;
static float ccd_support_depth (const rigidbody *body) {
    /* Lowest-point offset below the center along world -Y. */
    if (body -> type == object_sphere) { return body -> radius; }
    if (body -> type == object_custom) {
        /* The core's documented fallback geometry for custom shapes is a
         * bounding sphere until a CCD stage override is attached. */
        return body -> radius;
    }
    if (body -> type == object_cylinder) {
        float ay = body -> cached_axes [0].y;
        if (ay > 1.0f) { ay = 1.0f; }
        if (ay < -1.0f) { ay = -1.0f; }
        return body -> radius * sqrtf (fmaxf (0.0f, 1.0f - ay * ay)) + body -> cylinder_half_length * fabsf (ay);
    }
    vector3 down = {0.0f, -1.0f, 0.0f};
    return body -> half_extensions.x * fabsf (vector3_dot (body -> cached_axes [0], down)) +
           body -> half_extensions.y * fabsf (vector3_dot (body -> cached_axes [1], down)) +
           body -> half_extensions.z * fabsf (vector3_dot (body -> cached_axes [2], down));
}
static float ccd_min_thickness (const rigidbody *body) {
    if (body -> type == object_sphere) { return body -> radius; }
    if (body -> type == object_custom) { return body -> radius; }
    if (body -> type == object_cylinder) { return fminf (body -> radius, body -> cylinder_half_length); }
    return fminf (body -> half_extensions.x, fminf (body -> half_extensions.y, body -> half_extensions.z));
} /* Earliest positive time at which |dp + dv*t| reaches radius. Compute the
 * quadratic in double and use the cancellation-resistant q formulation; the
 * direct (-b-sqrt(D))/(2a) root loses the near root for distant/high-speed
 * pairs and can silently miss a real sweep. */
static float ccd_sphere_sweep_toi (vector3 dp, vector3 dv, float radius, float dt) {
    if (!isfinite (dp.x) || !isfinite (dp.y) || !isfinite (dp.z) || !isfinite (dv.x) || !isfinite (dv.y) ||
        !isfinite (dv.z) || !isfinite (radius) || !(radius > 0.0f) || !isfinite (dt) || !(dt > 0.0f)) {
        return -1.0f;
    } double dx = (double) dp.x, dy = (double) dp.y, dz = (double) dp.z;
    double vx = (double) dv.x, vy = (double) dv.y, vz = (double) dv.z;
    double r = (double) radius;
    double a = vx * vx + vy * vy + vz * vz;
    double c = dx * dx + dy * dy + dz * dz - r * r;
    if (!(a > 0.0) || !(c > 0.0)) {
        return -1.0f; /* no relative motion, or already overlapping */
    } double b = 2.0 * (dx * vx + dy * vy + dz * vz);
    /* DESPOT-2026-10-01: fma(b,b,-4*a*c) is a libm/hardware-FMA op, not an
     * IEEE-exact +,-,*,/,sqrt sequence. Cross-target contraction differs,
     * so the TOI clamp/no-clamp bifurcation desyncs. Plain double mults
     * are bit-identical under -ffp-contract=off (see det_math.h). */
    double discriminant = b * b - 4.0 * a * c;
    if (!(discriminant >= 0.0)) { return -1.0f; }
    double root = sqrt (discriminant);
    double q = -0.5 * (b + copysign (root, b));
    double t0, t1;
    if (q == 0.0) { t0 = t1 = -b / (2.0 * a); } else {
        t0 = q / a;
        t1 = c / q;
    } double toi = INFINITY;
    if (t0 > 0.0 && t0 < toi)
        toi = t0;
    if (t1 > 0.0 && t1 < toi)
        toi = t1;
    if (!(toi < (double) dt)) { return -1.0f; }
    return (float) toi;
}
/* Range-based CCD clamp: processes bodies [start, end). Assumes
 * obstacle_margin is pre-computed (max tip speed over ALL bodies) and
 * output arrays are allocated by the caller. Each body writes only to its
 * own position/velocity/orientation and its own output slots, so disjoint
 * ranges are thread-safe. Returns the number of bodies clamped. */
int collision_ccd_sweep_clamp_range (rigidbody *bodies, int body_count, int start, int end,
                                     float dt, float obstacle_margin, const mpe_config_t *C,
                                     bool record_remainder,
                                     float *time_remaining_out, float *best_tois_out,
                                     unsigned char *hit_flags_out) {
    if ((!bodies) || (body_count <= 0) || (start < 0) || (end > body_count) || (start >= end) ||
        (!(dt > 0.0f)) || !isfinite (dt) || !C) {
        return 0;
    }
    int clamped = 0;
    for (int i = start; i < end; i++) {
        best_tois_out [i] = dt;
        hit_flags_out [i] = 0;
        if (time_remaining_out) { time_remaining_out [i] = dt; }
        rigidbody *mover = &bodies [i];
        if ((mover -> static_state) || (mover -> is_sleeping) || (mover -> no_collide)) { continue; }
        float lin_speed = vector3_length (mover -> velocity);
        float ang_speed = vector3_length (mover -> angular_velocity);
        float bound_r = broadphase_bounding_radius (mover);
        if ((!isfinite (bound_r)) || (bound_r < 0.0f)) { bound_r = 0.0f; }
        float sweep_speed = lin_speed + ang_speed * bound_r;
        sweep_speed += obstacle_margin;
        if (sweep_speed <= 0.0001f) { continue; }
        float thickness = ccd_min_thickness (mover);
        if ((thickness <= 0.0f) || (!isfinite (thickness))) { continue; }
        float displacement = sweep_speed * dt;
        bool do_volumes = displacement > fminf (thickness, 0.2f);
        if ((displacement <= C -> solver.penetration_slop) && (!do_volumes)) {
            continue; /* discrete sampling suffices */
        } float best_toi = dt;
        bool hit = false;
        /* 1. Floor plane y = 0. */
        float r_lowest_y = ccd_support_depth (mover);
        float lowest = mover -> position.y - r_lowest_y;
        vector3 r_lowest = {0.0f, -r_lowest_y, 0.0f};
        vector3 w_cross_rlow = vector3_cross (mover -> angular_velocity, r_lowest);
        float v0 = mover -> velocity.y + w_cross_rlow.y;
        {
            float gravity = C -> world.gravity;
            float y0 = lowest;
            float a_lin = 0.5f * gravity;
            if (fabsf (a_lin) < 1e-9f) {
                if (fabsf (v0) > 1e-9f) {
                    float toi = y0 / -v0;
                    if ((toi > 0.0f) && (toi < best_toi)) {
                        best_toi = toi;
                        hit = true;
                    }
                }
            } else {
                double a = 0.5 * (double) gravity;
                double b = (double) v0;
                double c = (double) y0;
                double disc = b * b - 4.0 * a * c;
                if (disc >= 0.0) {
                    double sqrt_disc = sqrt (disc);
                    double denom = 2.0 * a;
                    if (fabs (denom) > 1e-18) {
                        double t1 = (-b - sqrt_disc) / denom;
                        double t2 = (-b + sqrt_disc) / denom;
                        double toi = 1e30;
                        if (t1 > 0.0) { toi = t1; }
                        if (t2 > 0.0 && t2 < toi) { toi = t2; }
                        if (toi > 0.0 && toi < (double) best_toi) {
                            best_toi = (float) toi;
                            hit = true;
                        }
                    }
                }
            }
        }
        /* 2. Volumes. */
        if (do_volumes) {
            for (int j = 0; j < body_count; j++) {
                if (j == i) { continue; }
                rigidbody *other = &bodies [j];
                if (other -> no_collide) { continue; }
                vector3 other_v = ((other -> static_state) || (other -> is_sleeping)) ? vector3_zero () : other -> velocity;
                if ((other -> type == object_sphere) || (other -> type == object_custom)) {
                    vector3 dp = simd_sub (other -> position, mover -> position);
                    vector3 dv = simd_sub (other_v, mover -> velocity);
                    float mover_radius =
                        (mover -> type == object_sphere) ? mover -> radius : broadphase_bounding_radius (mover);
                    float other_radius =
                        (other -> type == object_sphere) ? other -> radius : broadphase_bounding_radius (other);
                    float toi = ccd_sphere_sweep_toi (dp, dv, mover_radius + other_radius, dt);
                    if ((toi > 0.0f) && (toi < best_toi)) {
                        best_toi = toi;
                        hit = true;
                    }
                } else if (other -> type == object_cylinder) {
                    if (mover -> type == object_sphere) {
                        vector3 ax = other -> cached_axes [0];
                        float ax_len = vector3_length (ax);
                        if (ax_len < 1e-6f) {
                            ax = (vector3) {1.0f, 0.0f, 0.0f};
                        } else {
                            ax = simd_scale (ax, 1.0f / ax_len);
                        } float h = other -> cylinder_half_length;
                        float r_cyl = other -> radius;
                        float r_sph = mover -> radius;
                        float rr = r_cyl + r_sph;
                        vector3 ep1 = simd_add (other -> position, simd_scale (ax, -h));
                        vector3 ep2 = simd_add (other -> position, simd_scale (ax, h));
                        vector3 dp1 = simd_sub (ep1, mover -> position);
                        vector3 dp2 = simd_sub (ep2, mover -> position);
                        vector3 dv = simd_sub (other_v, mover -> velocity);
                        float best_cyl_toi = dt;
                        for (int ep = 0; ep < 2; ep++) {
                            vector3 dp = (ep == 0) ? dp1 : dp2;
                            float toi = ccd_sphere_sweep_toi (dp, dv, rr, dt);
                            if ((toi > 0.0f) && (toi < best_cyl_toi)) { best_cyl_toi = toi; }
                        }
                        float dv_ax = vector3_dot (dv, ax);
                        vector3 dv_perp = simd_sub (dv, simd_scale (ax, dv_ax));
                        float dv_perp_len_sq = vector3_length_squared (dv_perp);
                        if (dv_perp_len_sq > 1e-12f) {
                            vector3 dp_mid = simd_sub (other -> position, mover -> position);
                            float dp_ax = vector3_dot (dp_mid, ax);
                            vector3 dp_perp_local = simd_sub (dp_mid, simd_scale (ax, dp_ax));
                            {
                                float toi = ccd_sphere_sweep_toi (dp_perp_local, dv_perp, rr, dt);
                                if ((toi > 0.0f) && (toi < best_cyl_toi)) {
                                    vector3 rel_pos = simd_add (dp_mid, simd_scale (dv, toi));
                                    float rel_ax = vector3_dot (rel_pos, ax);
                                    if (fabsf (rel_ax) <= h) { best_cyl_toi = toi; }
                                }
                            }
                        }
                        if ((best_cyl_toi > 0.0f) && (best_cyl_toi < best_toi)) {
                            best_toi = best_cyl_toi;
                            hit = true;
                        }
                    } else {
                        /* Cylinder vs cylinder: bounding-sphere conservative. */
                        vector3 dp = simd_sub (other -> position, mover -> position);
                        vector3 dv = simd_sub (other_v, mover -> velocity);
                        float mover_radius = broadphase_bounding_radius (mover);
                        float other_radius = broadphase_bounding_radius (other);
                        float toi = ccd_sphere_sweep_toi (dp, dv, mover_radius + other_radius, dt);
                        if ((toi > 0.0f) && (toi < best_toi)) {
                            best_toi = toi;
                            hit = true;
                        }
                    }
                } else if (other -> type == object_cube) {
                    vector3 rel = simd_sub (mover -> position, other -> position);
                    vector3 rel_v = simd_sub (mover -> velocity, other_v);
                    vector3 ax0 = other -> cached_axes [0];
                    vector3 ax1 = other -> cached_axes [1];
                    vector3 ax2 = other -> cached_axes [2];
                    float sr = (mover -> type == object_sphere) ? mover -> radius : broadphase_bounding_radius (mover);
                    float pl [3] = {vector3_dot (rel, ax0), vector3_dot (rel, ax1), vector3_dot (rel, ax2)};
                    float vl [3] = {vector3_dot (rel_v, ax0), vector3_dot (rel_v, ax1), vector3_dot (rel_v, ax2)};
                    float ex [3] = {other -> half_extensions.x + sr, other -> half_extensions.y + sr,
                                   other -> half_extensions.z + sr};
                    float tmin = 0.0f, tmax = dt;
                    bool miss = false;
                    for (int a3 = 0; a3 < 3; a3++) {
                        float p = pl [a3], v = vl [a3], e = ex [a3];
                        if (fabsf (v) < 1e-9f) {
                            if ((p < -e) || (p > e)) { miss = true; break; }
                        } else {
                            float t1 = (-e - p) / v;
                            float t2 = (e - p) / v;
                            if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
                            if (t1 > tmin) { tmin = t1; }
                            if (t2 < tmax) { tmax = t2; }
                            if (tmin > tmax) { miss = true; break; }
                        }
                    }
                    if ((!miss) && (tmin > 0.0f) && (tmin < best_toi)) {
                        best_toi = tmin;
                        hit = true;
                    }
                }
            }
        } /* do_volumes */
        /* Phase 1: record only (no move yet — symmetric two-phase). */
        if (hit && best_toi < dt && best_toi > 0.0f) {
            best_tois_out [i] = best_toi;
            hit_flags_out [i] = 1;
        }
        /* Phase 2: apply clamp immediately (within-range, disjoint bodies). */
        if (record_remainder && hit_flags_out [i]) {
            float toi = best_tois_out [i];
            if (!(toi > 0.0f) || !(toi < dt)) { continue; }
            vector3 v_pre = mover -> velocity;
            if (!isfinite (v_pre.x) || !isfinite (v_pre.y) || !isfinite (v_pre.z)) { continue; }
            float drag_c = C -> world.drag;
            float grav_c = C -> world.gravity;
            if (!isfinite (drag_c) || drag_c <= 0.0f) { drag_c = 1.0f; }
            if (drag_c > 1.0f) { drag_c = 1.0f; }
            if (!isfinite (grav_c)) { grav_c = 0.0f; }
            double cdr = (drag_c >= 1.0f - 1e-6f) ? 0.0 : -det_ln_pos ((double) drag_c);
            double e_toi, a_pos, g_pos, v_toi_k;
            if (cdr == 0.0) {
                double t = (double) toi;
                e_toi = 1.0;
                a_pos = t;
                g_pos = 0.5 * t * t;
                v_toi_k = t;
            } else {
                double cdt = cdr * (double) toi;
                double e;
                if (cdt < 1e-4) { e = 1.0 - cdt + 0.5 * cdt * cdt - cdt * cdt * cdt / 6.0; } else {
                    e = det_exp_small (-cdt);
                } e_toi = e;
                a_pos = (1.0 - e) / cdr;
                g_pos = (double) toi / cdr - (1.0 - e) / (cdr * cdr);
                v_toi_k = (1.0 - e) / cdr;
            }
            vector3 grav_vec = {0.0f, grav_c, 0.0f};
            mover -> position =
                simd_add (mover -> position, simd_add (simd_scale (v_pre, (float) a_pos),
                                                                      simd_scale (grav_vec, (float) g_pos)));
            mover -> velocity =
                simd_add (simd_scale (v_pre, (float) e_toi), simd_scale (grav_vec, (float) v_toi_k));
            float spin = vector3_length (mover -> angular_velocity);
            if (spin > 1e-6f && isfinite (spin)) {
                double half = 0.5 * (double) spin * (double) toi;
                double s = det_sin (half);
                double c = det_cos (half);
                double inv = 1.0 / (double) spin;
                vector4 rotor = {(float) c, (float) (mover -> angular_velocity.x * inv * s),
                                 (float) (mover -> angular_velocity.y * inv * s),
                                 (float) (mover -> angular_velocity.z * inv * s)};
                mover -> orientation = vector4_normalisation (vector4_multiplication (rotor, mover -> orientation));
            } rigidbody_wake (mover);
            rigidbody_update_axes (mover);
            clamped++;
            float rem = dt - toi;
            time_remaining_out [i] = (rem > 0.0f) ? rem : 0.0f;
        }
    }
    return clamped;
}
int collision_ccd_sweep_clamp_full (rigidbody *bodies, int body_count, float dt, float *time_remaining_out,
                                    const mpe_config_t *cfg, float *best_tois_out, unsigned char *hit_flags_out) {
    if ((!bodies) || (body_count <= 0) || (!(dt > 0.0f)) || !isfinite (dt)) { return 0; }
    /* TRUTH: low-memory NULL path must NOT pre-move. Degraded mode: discrete only. */
    bool record_remainder = (time_remaining_out != NULL);
    if (time_remaining_out) {
        for (int k = 0; k < body_count; k++) { time_remaining_out [k] = dt; }
    }
    /* Obstacle margin: max tip speed over ALL bodies (computed serially,
     * then read-only during the per-body sweep). */
    float obstacle_margin = 0.0f;
    for (int k = 0; k < body_count; k++) {
        rigidbody *ob = &bodies [k];
        if ((ob -> static_state) || (ob -> is_sleeping) || (ob -> no_collide)) { continue; }
        float ow = vector3_length (ob -> angular_velocity);
        float orad = broadphase_bounding_radius (ob);
        if ((!isfinite (ow)) || (!isfinite (orad)) || (orad < 0.0f)) { continue; }
        float tip = ow * orad;
        if (isfinite (tip) && (tip > obstacle_margin)) { obstacle_margin = tip; }
    }
    if (obstacle_margin > 2.0f) { obstacle_margin = 2.0f; }
    g_ccd_obstacle_margin = obstacle_margin;
    /* Scratch arrays: caller-provided or heap. */
    bool use_heap = false;
    float *best_tois = best_tois_out;
    unsigned char *hit_flags = hit_flags_out;
    if (!best_tois || !hit_flags) {
        use_heap = true;
        best_tois = (float *) malloc ((size_t) body_count * sizeof (float));
        hit_flags = (unsigned char *) malloc ((size_t) body_count * sizeof (unsigned char));
        if (!best_tois || !hit_flags) {
            free (best_tois);
            free (hit_flags);
            return 0;
        }
    }
    /* Delegate the per-body sweep (phase 1 TOI + phase 2 clamp) to the
     * range function over [0, body_count). The range function initialises
     * best_tois/hit_flags internally and applies clamps with disjoint writes
     * per body. */
    int clamped = collision_ccd_sweep_clamp_range (bodies, body_count, 0, body_count,
                                                    dt, obstacle_margin, cfg, record_remainder,
                                                    time_remaining_out, best_tois, hit_flags);
    if (use_heap) {
        free (best_tois);
        free (hit_flags);
    } return clamped;
}
int collision_ccd_sweep_clamp (rigidbody *bodies, int body_count, float dt) {return collision_ccd_sweep_clamp_full (bodies, body_count, dt, NULL, NULL, NULL, NULL);}
int collision_ccd_sweep_clamp_world (struct physics_world *world, float dt) {
    if (!world || !world -> bodies || world -> body_count <= 0) { return 0; }
    const mpe_config_t *C = world -> cfg ? world -> cfg : &g_cfg;
    if (world -> ccd_time_remaining) {
        return collision_ccd_sweep_clamp_full (world -> bodies, world -> body_count, dt, world -> ccd_time_remaining, C,
                                               world -> ccd_best_tois, world -> ccd_hit_flags);
    } return collision_ccd_sweep_clamp_full (world -> bodies, world -> body_count, dt, NULL, C, NULL, NULL);
}

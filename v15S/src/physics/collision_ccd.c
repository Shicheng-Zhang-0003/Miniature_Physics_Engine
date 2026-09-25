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

static float ccd_support_depth(const rigidbody *body) {
    /* Lowest-point offset below the center along world -Y. */
    if (body->type == object_sphere) {
        return body->radius;
    }
    if (body->type == object_custom) {
        /* The core's documented fallback geometry for custom shapes is a
         * bounding sphere until a CCD stage override is attached. */
        return body->radius;
    }
    if (body->type == object_cylinder) {
        float ay = body->cached_axes[0].y;
        if (ay > 1.0f) {
            ay = 1.0f;
        }
        if (ay < -1.0f) {
            ay = -1.0f;
        }
        return body->radius * sqrtf(fmaxf(0.0f, 1.0f - ay * ay)) +
               body->cylinder_half_length * fabsf(ay);
    }
    vector3 down = {0.0f, -1.0f, 0.0f};
    return body->half_extensions.x * fabsf(vector3_dot(body->cached_axes[0], down)) +
           body->half_extensions.y * fabsf(vector3_dot(body->cached_axes[1], down)) +
           body->half_extensions.z * fabsf(vector3_dot(body->cached_axes[2], down));
}

static float ccd_min_thickness(const rigidbody *body) {
    if (body->type == object_sphere) {
        return body->radius;
    }
    if (body->type == object_custom) {
        return body->radius;
    }
    if (body->type == object_cylinder) {
        return fminf(body->radius, body->cylinder_half_length);
    }
    return fminf(body->half_extensions.x, fminf(body->half_extensions.y, body->half_extensions.z));
}

/* Earliest positive time at which |dp + dv*t| reaches radius. Compute the
 * quadratic in double and use the cancellation-resistant q formulation; the
 * direct (-b-sqrt(D))/(2a) root loses the near root for distant/high-speed
 * pairs and can silently miss a real sweep. */
static float ccd_sphere_sweep_toi(vector3 dp, vector3 dv, float radius, float dt) {
    if (!isfinite(dp.x) || !isfinite(dp.y) || !isfinite(dp.z) || !isfinite(dv.x) || !isfinite(dv.y) ||
        !isfinite(dv.z) || !isfinite(radius) || !(radius > 0.0f) || !isfinite(dt) || !(dt > 0.0f)) {
        return -1.0f;
    }
    double dx = (double) dp.x, dy = (double) dp.y, dz = (double) dp.z;
    double vx = (double) dv.x, vy = (double) dv.y, vz = (double) dv.z;
    double r = (double) radius;
    double a = vx * vx + vy * vy + vz * vz;
    double c = dx * dx + dy * dy + dz * dz - r * r;
    if (!(a > 0.0) || !(c > 0.0)) {
        return -1.0f; /* no relative motion, or already overlapping */
    }
    double b = 2.0 * (dx * vx + dy * vy + dz * vz);
    double discriminant = fma(b, b, -4.0 * a * c);
    if (!(discriminant >= 0.0)) {
        return -1.0f;
    }
    double root = sqrt(discriminant);
    double q = -0.5 * (b + copysign(root, b));
    double t0, t1;
    if (q == 0.0) {
        t0 = t1 = -b / (2.0 * a);
    } else {
        t0 = q / a;
        t1 = c / q;
    }
    double toi = INFINITY;
    if (t0 > 0.0 && t0 < toi) toi = t0;
    if (t1 > 0.0 && t1 < toi) toi = t1;
    if (!(toi < (double) dt)) {
        return -1.0f;
    }
    return (float) toi;
}

int collision_ccd_sweep_clamp_full(rigidbody *bodies, int body_count, float dt, float *time_remaining_out,
                                   const mpe_config_t *cfg, float *best_tois_out,
                                   unsigned char *hit_flags_out) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!bodies) || (body_count <= 0) || (!(dt > 0.0f)) || !isfinite(dt)) {
        return 0;
    }
    /* TRUTH: low-memory NULL path must NOT pre-move. Old code did
     * pos+=v*toi with no remainder recorded, then caller integrated full dt
     * => toi+dt double-count overshoot. Degraded mode: discrete only. */
    bool record_remainder = (time_remaining_out != NULL);
    if (time_remaining_out) {
        for (int k = 0; k < body_count; k++) {
            time_remaining_out[k] = dt;
        }
    }
    /* TRUTH: symmetric two-phase clamp. Old sequential per-body move vs
     * already-moved positions let B tunnel through A (A clamps to contact
     * vs B_old, moves; B sees c<=0 vs A_new, skips, then integrates full dt
     * through A). Phase 1 computes all TOIs vs OLD positions; phase 2 moves
     * all simultaneously. Order-independent. */
    float *best_tois = NULL;
    unsigned char *hit_flags = NULL;
    bool use_heap = false;
    bool use_caller_scratch = (best_tois_out != NULL) && (hit_flags_out != NULL);
    float stack_tois[64];
    unsigned char stack_hits[64];
    if (use_caller_scratch) {
        best_tois = best_tois_out;
        hit_flags = hit_flags_out;
    } else if (body_count > 64) {
        use_heap = true;
        best_tois = (float *) malloc((size_t) body_count * sizeof(float));
        hit_flags = (unsigned char *) malloc((size_t) body_count * sizeof(unsigned char));
        if (!best_tois || !hit_flags) {
            free(best_tois);
            free(hit_flags);
            return 0;
        }
    } else {
        best_tois = stack_tois;
        hit_flags = stack_hits;
    }
    for (int i = 0; i < body_count; i++) {
        best_tois[i] = dt;
        hit_flags[i] = 0;
    }
    for (int i = 0; i < body_count; i++) {
        rigidbody *mover = &bodies[i];
        if ((mover->static_state) || (mover->is_sleeping) || (mover->no_collide)) {
            continue;
        }
        /* TRUTH P1-19: angular sweep bound. Tip speed |w x r| <= |w|*R sweeps
         * a disc; linear-only bound tunnels for fast spinners. */
        float lin_speed = vector3_length(mover->velocity);
        float ang_speed = vector3_length(mover->angular_velocity);
        float bound_r = broadphase_bounding_radius(mover);
        if ((!isfinite(bound_r)) || (bound_r < 0.0f)) {
            bound_r = 0.0f;
        }
        float sweep_speed = lin_speed + ang_speed * bound_r;
        if (sweep_speed <= 0.0001f) {
            continue;
        }
        float thickness = ccd_min_thickness(mover);
        if ((thickness <= 0.0f) || (!isfinite(thickness))) {
            continue;
        }
        float displacement = sweep_speed * dt;
        /* Plane sweep is O(1) per body: run it whenever the tick motion
         * exceeds slop, so impacts never fall in the gap between slop-band
         * sampling and the safety nets (which would delete the bounce).
         * TRUTH: volume gate must consider obstacle thinness, not mover
         * alone. Large mover (1m) moving 0.5m would skip volumes and tunnel
         * a thin 0.1m wall. Gate on min(mover, 0.2m) so fast large bodies
         * still sweep. */
        bool do_volumes = displacement > fminf(thickness, 0.2f);
        if ((displacement <= C->solver.penetration_slop) && (!do_volumes)) {
            continue; /* discrete sampling suffices */
        }
        float best_toi = dt;
        bool hit = false;

                /* 1. Floor plane y = 0. Exact quadratic CCD under constant gravity.
         * Equation: 0.5*g*t^2 + v0*t + y0 = 0, with v0 the CENTER vertical
         * velocity and y0 the lowest-point height (center minus support).
         * Translational-only by design (see below): spin never triggers. */
        /* Lowest point offset from center (support depth preserves the
         * contact geometry for boxes/cylinders; the TOI itself is
         * translational — see below). */
        float r_lowest_y = ccd_support_depth(mover);
        float lowest = mover->position.y - r_lowest_y;

        /* Floor TOI uses the CENTER (translational) velocity, never the
         * lowest-point velocity v_center + omega x r. Rotation alone cannot
         * translate the center through the plane: a pure spinner reports a
         * diving lowest point (wheel at 127 rad/s: vy_low = -6.8 m/s while
         * stationary) and would clamp every tick — teleporting/rotating
         * rolling contact into levitation and spin pump (driven_wheel
         * disease). Dipping corners are bounded oscillation the discrete
         * solver re-seats; CCD owns translation tunneling only. */
        float v0 = mover->velocity.y;
        /* Exact quadratic CCD for floor plane under constant gravity.
         * Equation: 0.5*g*t^2 + v0*t + y0 = 0
         * where g = gravity (negative), y0 = lowest-point height above plane.
         * TRUTH: solve UNCONDITIONALLY (no v0 sign gate). Gravity curves
         * trajectories: a rising body (v0>0) still impacts within the tick
         * when y0 is small (v0=0.01,y0=1e-4,g=-9.81 -> toi=0.0056<dt); the
         * old v0<-eps gate tunneled those. Only toi in (0,dt) clamps, and
         * sleeping movers are skipped above, so resting sleepers never churn.
         * Micro-hop artifacts stay sub-tick scale. */
        {
            float gravity = C->world.gravity;  /* negative */
            float y0 = lowest;
            /* Linear fallback whenever |a| is degenerate: |g|<2e-9 gives
             * a<1e-9, where float disc rounds sqrt(b^2-4ac)->|b| and the
             * quadratic root collapses to 0 instead of y0/-v0. */
            float a_lin = 0.5f * gravity;
            if (fabsf(a_lin) < 1e-9f) {
                /* Linear case (no/near-zero gravity): y0 + v0*t = 0 */
                if (fabsf(v0) > 1e-9f) {
                    float toi = y0 / -v0;
                    if ((toi > 0.0f) && (toi < best_toi)) {
                        best_toi = toi;
                        hit = true;
                    }
                }
            } else {
                /* Quadratic: 0.5*g*t^2 + v0*t + y0 = 0.
                 * Fires under real gravity (the tunneling case CCD exists for).
                 * TRUTH: parabola ignores viscous drag (exact only for
                 * drag=1; drag<1 errs O(c*dt^2), ~1e-6m at 0.99/60Hz,
                 * second-order — one Newton step on the analytic residual
                 * would make it exact if ever needed).
                 * Computed in double to avoid float cancellation. */
                double a = 0.5 * (double)gravity;
                double b = (double)v0;
                double c = (double)y0;
                double disc = b * b - 4.0 * a * c;
                if (disc >= 0.0) {
                    double sqrt_disc = sqrt(disc);
                    double denom = 2.0 * a;
                    if (fabs(denom) > 1e-18) {
                        double t1 = (-b - sqrt_disc) / denom;
                        double t2 = (-b + sqrt_disc) / denom;
                        double toi = 1e30;
                        if (t1 > 0.0) toi = t1;
                        if (t2 > 0.0 && t2 < toi) toi = t2;
                        if (toi > 0.0 && toi < (double)best_toi) {
                            best_toi = (float)toi;
                            hit = true;
                        }
                    }
                }
            }
        }

        /* 2. Volumes: spheres, boxes (static AND dynamic via relative
         * velocity in obstacle frame), cylinders and custom shapes (using
         * conservative bounding spheres where exact sweep geometry is not
         * available). */
        if (do_volumes) {
        for (int j = 0; j < body_count; j++) {
            if (j == i) {
                continue;
            }
            rigidbody *other = &bodies[j];
            if (other->no_collide) {
                continue; /* render-only proxies: never obstacles */
            }
            vector3 other_v =
                ((other->static_state) || (other->is_sleeping)) ? vector3_zero() : other->velocity;
            if ((other->type == object_sphere) || (other->type == object_custom)) {
                vector3 dp = vector3_subtraction(other->position, mover->position);
                vector3 dv = vector3_subtraction(other_v, mover->velocity);
                float mover_radius = (mover->type == object_sphere) ? mover->radius
                                                                    : broadphase_bounding_radius(mover);
                float other_radius = (other->type == object_sphere) ? other->radius
                                                                    : broadphase_bounding_radius(other);
                float toi = ccd_sphere_sweep_toi(dp, dv, mover_radius + other_radius, dt);
                if ((toi > 0.0f) && (toi < best_toi)) {
                    best_toi = toi;
                    hit = true;
                }
            } else if (other->type == object_cylinder) {
                /* Exact segment-vs-sphere sweep for cylinder obstacles.
                 * Cylinder = segment (axle) + radius. Sweep the mover's bounding
                 * sphere against the cylinder's swept capsule.
                 *
                 * For a moving sphere (or sphere-bounded body) vs static cylinder:
                 *   - The cylinder's axle endpoints sweep spheres of radius r
                 *   - The barrel sweeps a capsule along the relative velocity
                 *   - We solve for the earliest TOI by checking segment-sphere
                 *     and capsule-sphere sweep.
                 *
                 * For a moving cylinder vs static cylinder: both segments sweep.
                 * Full segment-segment sweep is complex; fall back to bounding
                 * sphere for cylinder-vs-cylinder (conservative, never misses).
                 */
                if (mover->type == object_sphere) {
                    /* Sphere vs cylinder: exact segment-sphere sweep. */
                    vector3 ax = other->cached_axes[0];
                    float ax_len = vector3_length(ax);
                    if (ax_len < 1e-6f) {
                        ax = (vector3){1.0f, 0.0f, 0.0f};
                    } else {
                        ax = vector3_scaling(ax, 1.0f / ax_len);
                    }
                    float h = other->cylinder_half_length;
                    float r_cyl = other->radius;
                    float r_sph = mover->radius;
                    float rr = r_cyl + r_sph;

                    /* Cylinder endpoints in world space. */
                    vector3 ep1 = vector3_addition(other->position, vector3_scaling(ax, -h));
                    vector3 ep2 = vector3_addition(other->position, vector3_scaling(ax, h));

                    /* Relative motion. */
                    vector3 dp1 = vector3_subtraction(ep1, mover->position);
                    vector3 dp2 = vector3_subtraction(ep2, mover->position);
                    vector3 dv = vector3_subtraction(other_v, mover->velocity);

                    /* Sweep against both endpoint spheres. */
                    float best_cyl_toi = dt;
                    for (int ep = 0; ep < 2; ep++) {
                        vector3 dp = (ep == 0) ? dp1 : dp2;
                        float toi = ccd_sphere_sweep_toi(dp, dv, rr, dt);
                        if ((toi > 0.0f) && (toi < best_cyl_toi)) {
                            best_cyl_toi = toi;
                        }
                    }

                    /* Sweep against barrel (capsule segment).
                     * Project relative velocity onto plane perpendicular to axle. */
                    float dv_ax = vector3_dot(dv, ax);
                    vector3 dv_perp = vector3_subtraction(dv, vector3_scaling(ax, dv_ax));
                    float dv_perp_len_sq = vector3_length_squared(dv_perp);
                    if (dv_perp_len_sq > 1e-12f) {
                        /* Relative motion has perpendicular component - capsule sweep.
                         * The capsule is the segment extruded along dv_perp.
                         * Find closest approach of sphere to swept capsule. */
                        vector3 dp_mid = vector3_subtraction(other->position, mover->position);
                        float dp_ax = vector3_dot(dp_mid, ax);
                        vector3 dp_perp = vector3_subtraction(dp_mid, vector3_scaling(ax, dp_ax));

                        /* Quadratic for perpendicular distance == rr.
                         * |dp_perp + t*dv_perp|^2 = rr^2 */
                        {
                            float toi = ccd_sphere_sweep_toi(dp_perp, dv_perp, rr, dt);
                            if ((toi > 0.0f) && (toi < best_cyl_toi)) {
                                /* Check if contact point is within segment bounds at TOI.
                                 * TRUTH: strict |axial|<=h (barrel only). The old
                                 * h+rr double-covered the caps (already swept
                                 * exactly as endpoint spheres above) and reported
                                 * barrel hits rr beyond the segment end, where the
                                 * true distance sqrt(rr^2+(axial-h)^2)>rr:
                                 * early (wrong-side) clamps. */
                                vector3 rel_pos = vector3_addition(dp_mid, vector3_scaling(dv, toi));
                                float rel_ax = vector3_dot(rel_pos, ax);
                                if (fabsf(rel_ax) <= h) {
                                    best_cyl_toi = toi;
                                }
                            }
                        }
                    }

                    if ((best_cyl_toi > 0.0f) && (best_cyl_toi < best_toi)) {
                        best_toi = best_cyl_toi;
                        hit = true;
                    }
                } else {
                    /* Non-sphere mover vs cylinder: conservative bounding sphere sweep.
                     * (Exact segment-segment sweep for cylinder-vs-cylinder is complex;
                     * bounding sphere is conservative and never misses.) */
                    vector3 dp = vector3_subtraction(other->position, mover->position);
                    vector3 dv = vector3_subtraction(other_v, mover->velocity);
                    float rr = broadphase_bounding_radius(mover) + broadphase_bounding_radius(other);
                    float toi = ccd_sphere_sweep_toi(dp, dv, rr, dt);
                    if ((toi > 0.0f) && (toi < best_toi)) {
                        best_toi = toi;
                        hit = true;
                    }
                }
            } else if (other->type == object_cube) {
                /* Swept sphere-vs-OBB via slab test in box space, with
                 * RELATIVE velocity so dynamic boxes sweep correctly. */
                float sr = (mover->type == object_sphere) ? mover->radius
                                                          : broadphase_bounding_radius(mover);
                vector3 rel = vector3_subtraction(mover->position, other->position);
                vector3 rel_v = vector3_subtraction(mover->velocity, other_v);
                vector3 ax0 = other->cached_axes[0];
                vector3 ax1 = other->cached_axes[1];
                vector3 ax2 = other->cached_axes[2];
                float pl[3] = {vector3_dot(rel, ax0), vector3_dot(rel, ax1), vector3_dot(rel, ax2)};
                float vl[3] = {vector3_dot(rel_v, ax0), vector3_dot(rel_v, ax1),
                               vector3_dot(rel_v, ax2)};
                float ex[3] = {other->half_extensions.x + sr, other->half_extensions.y + sr,
                               other->half_extensions.z + sr};
                float tmin = 0.0f, tmax = dt;
                bool miss = false;
                for (int a3 = 0; a3 < 3; a3++) {
                    float p = pl[a3], v = vl[a3], e = ex[a3];
                    if (fabsf(v) < 1e-9f) {
                        if ((p < -e) || (p > e)) {
                            miss = true;
                            break;
                        }
                    } else {
                        float t1 = (-e - p) / v;
                        float t2 = (e - p) / v;
                        if (t1 > t2) {
                            float tmp = t1;
                            t1 = t2;
                            t2 = tmp;
                        }
                        if (t1 > tmin) {
                            tmin = t1;
                        }
                        if (t2 < tmax) {
                            tmax = t2;
                        }
                        if (tmin > tmax) {
                            miss = true;
                            break;
                        }
                    }
                }
                /* tmin > 0 with a hit means true entry (starting outside);
                 * starting inside gives tmin <= 0: discrete path owns it. */
                if ((!miss) && (tmin > 0.0f) && (tmin < best_toi)) {
                    best_toi = tmin;
                    hit = true;
                }
            }
        }
        } /* do_volumes */

        /* Phase 1: record only (no move yet — symmetric two-phase). */
        if (hit && best_toi < dt && best_toi > 0.0f) {
            best_tois[i] = best_toi;
            hit_flags[i] = 1;
        }
    }
    /* Phase 2: apply all clamps simultaneously vs OLD positions.
     * TRUTH: analytic pre-move + chained v(toi), not linear pre-move.
     * Linear pre-move (pos+=v*toi) plus remainder analytic-from-v_pre
     * violates the semigroup: total = v0*dt+0.5*g*rem^2 instead of
     * v0*dt+0.5*g*dt^2 (pos err 0.5*g*(toi^2+2*toi*rem), vel err g*toi;
     * dt=1/60,toi=dt/2: 1mm + 0.08m/s per tick). Instead advance the exact
     * gravity+drag flow to toi AND chain v_toi into velocity, so the
     * remainder analytic (from v_toi, via the tick_v0 snapshot taken after
     * CCD) composes to exactly analytic(dt) from v_pre. Rotation already
     * composes (w*toi + w*rem); this makes translation match. Volume TOIs
     * (derived linearly) overshoot by <=0.5*g*toi^2 < slop: absorbed. */
    int clamped = 0;
    if (record_remainder) {
        const mpe_config_t *CC = C;
        float drag_c = CC->world.drag;
        float grav_c = CC->world.gravity;
        if (!isfinite(drag_c) || drag_c <= 0.0f) drag_c = 1.0f;
        if (drag_c > 1.0f) drag_c = 1.0f;
        if (!isfinite(grav_c)) grav_c = 0.0f;
        double cdr = (drag_c >= 1.0f - 1e-6f) ? 0.0 : -det_ln_pos((double) drag_c);
        for (int i = 0; i < body_count; i++) {
            if (!hit_flags[i]) {
                continue;
            }
            rigidbody *mover = &bodies[i];
            if ((mover->static_state) || (mover->is_sleeping)) {
                continue;
            }
            float toi = best_tois[i];
            if (!(toi > 0.0f) || !(toi < dt)) {
                continue;
            }
            /* Exact flow to toi from v_pre (velocity untouched since tick
             * start: CCD runs before force integration). */
            vector3 v_pre = mover->velocity;
            if (!isfinite(v_pre.x) || !isfinite(v_pre.y) || !isfinite(v_pre.z)) {
                continue;
            }
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
                if (cdt < 1e-4) {
                    e = 1.0 - cdt + 0.5 * cdt * cdt - cdt * cdt * cdt / 6.0;
                } else {
                    e = det_exp_small(-cdt);
                }
                e_toi = e;
                a_pos = (1.0 - e) / cdr;
                g_pos = (double) toi / cdr - (1.0 - e) / (cdr * cdr);
                v_toi_k = (1.0 - e) / cdr;
            }
            vector3 grav_vec = {0.0f, grav_c, 0.0f};
            mover->position = vector3_addition(
                mover->position,
                vector3_addition(vector3_scaling(v_pre, (float) a_pos), vector3_scaling(grav_vec, (float) g_pos)));
            mover->velocity = vector3_addition(vector3_scaling(v_pre, (float) e_toi),
                                               vector3_scaling(grav_vec, (float) v_toi_k));
            float spin = vector3_length(mover->angular_velocity);
            if (spin > 1e-6f && isfinite(spin)) {
                /* TRUTH: full-range det sin/cos (bit-identical), never libm
                 * from_axis_with_angle in the tick path (|w|*toi routinely
                 * exceeds 0.5 for fast spinners, e.g. 60 rad/s). */
                double half = 0.5 * (double) spin * (double) toi;
                double s = det_sin(half);
                double c = det_cos(half);
                double inv = 1.0 / (double) spin;
                vector4 rotor = {(float) c, (float) (mover->angular_velocity.x * inv * s),
                                 (float) (mover->angular_velocity.y * inv * s),
                                 (float) (mover->angular_velocity.z * inv * s)};
                mover->orientation = vector4_normalisation(vector4_multiplication(rotor, mover->orientation));
            }
            rigidbody_wake(mover);
            rigidbody_update_axes(mover);
            clamped++;
            float rem = dt - toi;
            time_remaining_out[i] = (rem > 0.0f) ? rem : 0.0f;
        }
    } else {
        /* Degraded NULL mode: no pre-move (would double-count). Count only. */
        for (int i = 0; i < body_count; i++) {
            if (hit_flags[i]) {
                clamped++;
            }
        }
    }
    if (use_heap) {
        free(best_tois);
        free(hit_flags);
    }
    return clamped;
}

int collision_ccd_sweep_clamp(rigidbody *bodies, int body_count, float dt) {
    return collision_ccd_sweep_clamp_full(bodies, body_count, dt, NULL, NULL, NULL, NULL);
}

int collision_ccd_sweep_clamp_world(struct physics_world *world, float dt) {
    if (!world || !world->bodies || world->body_count <= 0) {
        return 0;
    }
    const mpe_config_t *C = world->cfg ? world->cfg : &g_cfg;
    if (world->ccd_time_remaining) {
        return collision_ccd_sweep_clamp_full(world->bodies, world->body_count, dt, world->ccd_time_remaining, C, world->ccd_best_tois, world->ccd_hit_flags);
    }
    return collision_ccd_sweep_clamp_full(world->bodies, world->body_count, dt, NULL, C, NULL, NULL);
}

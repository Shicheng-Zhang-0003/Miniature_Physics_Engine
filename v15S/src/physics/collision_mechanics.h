#ifndef collisions_h
#define collisions_h

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "../core/math3d.h"
#include "../core/rigidbody.h"
#include "../config/mpe_config.h"
/* Per-world config convention: narrowphase/solver/CCD functions take
 * `const mpe_config_t *cfg` with NULL meaning "global g_cfg". Step paths
 * pass their world's snapshot (mpe_world_cfg); direct unit callers pass
 * NULL. This keeps foreign per-world configs authoritative without
 * breaking the direct-call test surface. */
struct physics_world; /* MFS_131: forward decl for per-world cache */
typedef struct {
    vector3 position;
    float penetration;
    vector3 local_position_a;
    vector3 local_position_b;
    float accumulated_normal_impulse;
    float accumulated_tangent_impulse;
    float effective_mass_normal;
    float effective_mass_tangent;
    vector3 tangent_vector;
    /* Second Coulomb tangent (disc model): tangent2 = n x tangent1.
     * Solved jointly with a combined |Ft| <= mu*Fn clamp. */
    vector3 tangent2;
    float accumulated_tangent2_impulse;
    float effective_mass_tangent2;
    /* cached_tangent REMOVED (dead field, never read; tangent_vector +
     * tangent2 are the live Coulomb-disc frame). */
    /* Poisson restitution state: Newtonian velocity bias is wrong for
     * multi-contact (it pays bounce per iteration). Instead the compression
     * phase accumulates unbiased impulse; the restitution pass then pays
     * e * (this tick's compression impulse) once, where
     * compression_this_tick = accumulated_normal_impulse - base_normal_impulse.
     * impact_velocity gates the pass to fresh impacts. TRUTH: base INCLUDES
     * the adopted warm-start impulse (excluding it would re-pay last tick's
     * bounce every tick). */
    float impact_velocity;
    float base_normal_impulse;
    /* RESERVED (ABI): compression fields predate the normal-minus-base
     * formulation and are intentionally unwritten/unread. Kept so plugin
     * struct layouts don't shift. Do not use. */
    float base_compression_impulse;
    float accumulated_compression_impulse;
    /* warmed REMOVED (dead provenance flag; SOR damping deleted with the
     * strict warm-start match — all contacts solve full steps). */
    /* NOTE: there is deliberately NO velocity-level Baumgarte bias field.
     * Penetration is corrected positionally only (split impulse +
     * depenetration pass). A velocity bias would inject approach velocity
     * that the friction clamp and Poisson pass then treat as real impact. */
    vector3 ra;
    vector3 rb;
} contact_point_data;

/* ---- Anisotropic Coulomb cone --------------------------------------------
 *
 * Legacy friction clamps the COMBINED tangential impulse to a DISC of radius
 * mu*Fn, because Coulomb friction is isotropic. Some real contacts are not:
 * a mecanum roller, a slipper sole, or anisotropic tread all resist motion
 * differently depending on direction within the contact plane. Modelling that
 * honestly means an ELLIPSE instead of a disc:
 *
 *     (p.a)^2 / (mu_a*Fn)^2  +  (p.b)^2 / (mu_b*Fn)^2  <=  1
 *
 * where a is the body's roll axis projected into the contact plane and b is
 * the in-plane perpendicular.
 *
 * Two properties make this a safe extension rather than a rewrite:
 *   1. mu_a == mu_b reproduces the disc EXACTLY, so every isotropic body in
 *      the engine (and every existing test) is bit-for-bit unchanged.
 *   2. The ellipse is contained in the disc whenever mu_a, mu_b <= mu, so
 *      anisotropy is strictly subtractive: it can only remove grip in the
 *      chosen direction, never inject energy or destabilise a contact that
 *      was stable without it.
 *
 * `axis_world` is ignored unless it projects to a usable in-plane direction.
 * A degenerate projection falls back to isotropic using min(mu_a, mu_b) so the
 * fallback can never grant more grip than the caller asked for.
 */
static inline void a3_anisotropic_coulomb_clamp(vector3 normal, vector3 t1, vector3 t2, float p1, float p2,
                                                float fn, float mu_a, float mu_b, vector3 axis_world,
                                                float *out1, float *out2) {
    *out1 = p1;
    *out2 = p2;
    if (!(fn > 0.0f) || !isfinite(fn)) {
        *out1 = 0.0f;
        *out2 = 0.0f;
        return;
    }
    if (!isfinite(mu_a) || mu_a < 0.0f) {
        mu_a = 0.0f;
    }
    if (!isfinite(mu_b) || mu_b < 0.0f) {
        mu_b = 0.0f;
    }
    float radius_a = mu_a * fn;
    float radius_b = mu_b * fn;
    /* A ZERO semi-axis is a RAIL, not "no friction". A free roller transmits
     * force along its own axle and nothing across it, so mu_b = 0 must leave
     * the axial direction fully live and only annihilate the transverse one.
     * Treating either zero radius as "clamp everything to zero" is what made a
     * mecanum wheel lose ALL grip instead of gaining a direction, and it is
     * why every mu_across = 0 configuration measured dead. Only a contact
     * with no live axis at all is frictionless. */
    if (!(radius_a > 0.0f) && !(radius_b > 0.0f)) {
        *out1 = 0.0f;
        *out2 = 0.0f;
        return;
    }
    /* Isotropic fast path: identical arithmetic to the legacy disc clamp. */
    if (mu_a == mu_b) {
        float combo_sq = p1 * p1 + p2 * p2;
        if (combo_sq > radius_a * radius_a) {
            float scale = radius_a / sqrtf(combo_sq);
            *out1 = p1 * scale;
            *out2 = p2 * scale;
        }
        return;
    }
    /* Project the roll axis into the contact plane. */
    float axial = vector3_dot(axis_world, normal);
    vector3 a = vector3_subtraction(axis_world, vector3_scaling(normal, axial));
    float len_a_sq = vector3_length_squared(a);
    if (!isfinite(len_a_sq) || (len_a_sq < 1.0e-8f)) {
        /* Roll axis is (anti)parallel to the contact normal: it carries no
         * in-plane information. Fall back to isotropic, weakest axis only. */
        float mu_min = (mu_a < mu_b) ? mu_a : mu_b;
        a3_anisotropic_coulomb_clamp(normal, t1, t2, p1, p2, fn, mu_min, mu_min, axis_world, out1, out2);
        return;
    }
    a = vector3_scaling(a, 1.0f / sqrtf(len_a_sq));
    vector3 b = vector3_cross(normal, a);
    float b_len_sq = vector3_length_squared(b);
    if (!isfinite(b_len_sq) || (b_len_sq < 1.0e-8f)) {
        float mu_min = (mu_a < mu_b) ? mu_a : mu_b;
        a3_anisotropic_coulomb_clamp(normal, t1, t2, p1, p2, fn, mu_min, mu_min, axis_world, out1, out2);
        return;
    }
    b = vector3_scaling(b, 1.0f / sqrtf(b_len_sq));
    float comp_a = p1 * vector3_dot(a, t1) + p2 * vector3_dot(a, t2);
    float comp_b = p1 * vector3_dot(b, t1) + p2 * vector3_dot(b, t2);
    /* Degenerate ellipse = line segment. Project onto the surviving axis and
     * clamp to ITS radius; the dead axis contributes nothing. This is the
     * exact anisotropic Coulomb set for a coefficient of 0 (an equality
     * constraint on the transverse component), not an approximation of it,
     * and it still sits inside the isotropic disc of radius mu_iso*fn. */
    if (!(radius_b > 0.0f) || !(radius_a > 0.0f)) {
        const vector3 live = (radius_a > 0.0f) ? a : b;
        const float live_radius = (radius_a > 0.0f) ? radius_a : radius_b;
        float cl = (radius_a > 0.0f) ? comp_a : comp_b;
        if (fabsf(cl) > live_radius) {
            cl = (cl > 0.0f) ? live_radius : -live_radius;
        }
        const float at1 = vector3_dot(live, t1);
        const float at2 = vector3_dot(live, t2);
        *out1 = cl * at1;
        *out2 = cl * at2;
        return;
    }
    float u = comp_a / radius_a;
    float v = comp_b / radius_b;
    float q = u * u + v * v;
    if ((q > 1.0f) && isfinite(q) && (q > 0.0f)) {
        float scale = 1.0f / sqrtf(q);
        *out1 = p1 * scale;
        *out2 = p2 * scale;
    }
}

/* Combine two bodies into one contact cone. mu_iso is the isotropic
 * coefficient the legacy path would have used (already min-of-bodies and
 * already stick/slip selected), and is used directly when neither body is
 * anisotropic. An anisotropic body contributes its own two coefficients under
 * the same fmin convention the engine already uses, and donates its world
 * roll axis. With two anisotropic bodies the first one found sets the axis:
 * the engine's only such case is a wheel (anisotropic) on a floor
 * (isotropic), so there is no ambiguity to resolve. */
static inline void a3_contact_friction_cone(const rigidbody *body_a, const rigidbody *body_b, float mu_iso,
                                            float *out_roll, float *out_lateral, vector3 *out_axis) {
    *out_roll = mu_iso;
    *out_lateral = mu_iso;
    *out_axis = (vector3){0.0f, 0.0f, 1.0f};
    bool have_axis = false;
    for (int side = 0; side < 2; side++) {
        const rigidbody *rb = (side == 0) ? body_a : body_b;
        if (!rb || !rb->friction_anisotropic) {
            continue;
        }
        if (!have_axis) {
            *out_axis = vector4_rotate_to_vector3(rb->orientation, rb->friction_anisotropy_axis);
            have_axis = true;
        }
        if (rb->friction_along_axis < *out_roll) {
            *out_roll = rb->friction_along_axis;
        }
        if (rb->friction_across_axis < *out_lateral) {
            *out_lateral = rb->friction_across_axis;
        }
    }
}

/* ---- Shared warm-start stamp (single source of truth) --------------------
 * FIX-AUDIT-DESPOT: collision_solver.c and collision_cache.c each carried a
 * private copy of this stamp (a3_task05_body_property_stamp). The solver
 * comment even warned "must match contact_cache_save's stamp exactly" —
 * a divergence that silently mis-warm-starts (stale impulse injection)
 * with zero diagnostics. One static-inline definition here; both TUs
 * delegate (thin wrappers kept so call sites don't churn). Friction,
 * restitution, kinematic, sleep, cylinder length, custom id and quantized
 * orientation are all part of the stamp because each changes the solved
 * impulse or the lever geometry. NOTE: the match-role/adopt predicates
 * cannot live here: they take cached_contact (defined in physics_world.h,
 * which includes THIS header — circular), so they stay in
 * collision_solver.c next to their only caller. */
static inline uint32_t a3_contact_cache_mix_u32(uint32_t hash_value, uint32_t input_value) {
    hash_value ^= input_value + 0x9e3779b9u + (hash_value << 6) + (hash_value >> 2);
    return hash_value;
}

static inline uint32_t a3_contact_cache_float_bits(float value) {
    union {
        float float_value;
        uint32_t integer_value;
    } converter;
    converter.float_value = value;
    return converter.integer_value;
}

static inline uint32_t a3_contact_cache_body_stamp(const rigidbody *rigid_body) {
    if (!rigid_body) {
        return 0;
    }
    uint32_t stamp = 2166136261u;
    stamp = a3_contact_cache_mix_u32(stamp, (uint32_t) rigid_body->type);
    stamp = a3_contact_cache_mix_u32(stamp, rigid_body->static_state ? 1u : 0u);
    stamp = a3_contact_cache_mix_u32(stamp, a3_contact_cache_float_bits(rigid_body->mass));
    stamp = a3_contact_cache_mix_u32(stamp, a3_contact_cache_float_bits(rigid_body->inverse_mass));
    stamp = a3_contact_cache_mix_u32(stamp, a3_contact_cache_float_bits(rigid_body->radius));
    stamp = a3_contact_cache_mix_u32(stamp, a3_contact_cache_float_bits(rigid_body->half_extensions.x));
    stamp = a3_contact_cache_mix_u32(stamp, a3_contact_cache_float_bits(rigid_body->half_extensions.y));
    stamp = a3_contact_cache_mix_u32(stamp, a3_contact_cache_float_bits(rigid_body->half_extensions.z));
    stamp = a3_contact_cache_mix_u32(stamp, a3_contact_cache_float_bits(rigid_body->friction_static));
    stamp = a3_contact_cache_mix_u32(stamp, a3_contact_cache_float_bits(rigid_body->friction_kinetic));
    stamp = a3_contact_cache_mix_u32(stamp, a3_contact_cache_float_bits(rigid_body->restitution));
    stamp = a3_contact_cache_mix_u32(stamp, a3_contact_cache_float_bits(rigid_body->cylinder_half_length));
    stamp = a3_contact_cache_mix_u32(stamp, (uint32_t) rigid_body->custom_shape);
    stamp = a3_contact_cache_mix_u32(stamp, rigid_body->kinematic ? 2u : 0u);
    stamp = a3_contact_cache_mix_u32(stamp, rigid_body->is_sleeping ? 4u : 0u);
    stamp = a3_contact_cache_mix_u32(
        stamp, a3_contact_cache_float_bits(roundf(rigid_body->orientation.w * 1000.0f) / 1000.0f));
    stamp = a3_contact_cache_mix_u32(
        stamp, a3_contact_cache_float_bits(roundf(rigid_body->orientation.x * 1000.0f) / 1000.0f));
    stamp = a3_contact_cache_mix_u32(
        stamp, a3_contact_cache_float_bits(roundf(rigid_body->orientation.y * 1000.0f) / 1000.0f));
    stamp = a3_contact_cache_mix_u32(
        stamp, a3_contact_cache_float_bits(roundf(rigid_body->orientation.z * 1000.0f) / 1000.0f));
    return stamp;
}

typedef struct {
    rigidbody *object_a;
    rigidbody *object_b;
    vector3 normal_vector;
    contact_point_data contacts[4];
    int contact_count;
} collision_data;
bool collision_dual_sphere(rigidbody *rigidbody_object_a, rigidbody *rigidbody_object_b,
                           collision_data *collision_output_data, const mpe_config_t *cfg);
float project_obb(rigidbody *rigid_body, vector3 axis, vector3 axes[3]);
bool collision_sphere_cube(rigidbody *sphere, rigidbody *cube, collision_data *collision_output_data,
                           const mpe_config_t *cfg);
bool collision_dual_cube(rigidbody *cube_a, rigidbody *cube_b, collision_data *collision_output_data,
                         const mpe_config_t *cfg);
void collision_prepare_solver(struct physics_world *world, collision_data *source, collision_data *manifold_entry,
                              float dt);
/* Returns the largest impulse magnitude applied this visit (for convergence tests). */
float collision_resolve_iterative(collision_data *manifold_entry, float dt, bool friction_only, int start_index,
                                  const mpe_config_t *cfg);
/* Support-first manifold order: indices sorted by lowest contact height
 * (floor contacts first, then ascending pairs), ties broken by manifold
 * index for a deterministic total order. Sequential impulse propagates
 * support ~one contact level per sweep in arbitrary order; support-first
 * order carries floor support to the top of a stack in a single sweep, so
 * deep stacks converge in far fewer iterations. Order affects only the
 * sweep sequence (same equations); twin runs agree bit-for-bit. Keys live
 * in the world's scratch. */
void collision_manifold_solve_order(struct physics_world *world, collision_data *manifolds, int manifold_count,
                                    int *order_out);
/* Split impulse (Catto): positional penetration correction applied AFTER
 * the velocity iterations, directly to positions, mass-weighted. Carries
 * no velocity change, so it cannot inflate contact impulses or friction. */
/* Split impulse (Catto): positional penetration correction applied AFTER
 * the velocity iterations, directly to positions, mass-weighted. Carries
 * no velocity change, so it cannot inflate contact impulses or friction. */
void collision_apply_split_impulse(collision_data *manifolds, int manifold_count, float dt,
                                   const mpe_config_t *cfg);
/* Rolling + spin resistance, once per tick after the velocity solve. */
void collision_apply_rolling_resistance(collision_data *manifolds, int manifold_count, float dt,
                                        const mpe_config_t *cfg);
/* Poisson restitution: after compression converges, each fresh impact gets
 * e * (this tick's compression impulse) once, then a short relaxation lets
 * friction respond. Correct for multi-contact; Newtonian bias over-pays. */
void collision_apply_poisson_restitution(collision_data *manifolds, int manifold_count, const mpe_config_t *cfg);
/* TRUTH P0-2: refresh Poisson gate to post-force-integration velocities.
 * Prepare() runs before gravity/springs/motors are integrated, so the
 * recorded impact_velocity is stale by g*dt + spring/motor deltas.
 * Call after rb_integrate_velocity, before iterations: recomputes
 * vn = (vb+wb×rb − va−wa×ra)·n from current velocities (ra/rb/n unchanged,
 * positions not yet moved). Exact pre-solve approach speed. */
void collision_refresh_impact_velocities(collision_data *manifolds, int manifold_count);
/* CCD swept clamp: for bodies whose per-tick displacement exceeds their
 * contact thickness, time-of-impact against the floor plane, sphere/custom
 * bounding spheres, finite cylinders, and boxes (including moving boxes via
 * relative linear velocity). Clamps the body to the TOI configuration and
 * keeps velocity, so discrete narrowphase then sees penetration≈0 with the
 * true approach velocity (restitution/friction respond correctly).
 * TRUTH P0-3: fills time_remaining_out[i] = dt - toi (dt if unclamped).
 * Post-solve integration MUST advance only the remainder, else toi+dt
 * double-counts. TOI geometry uses linear translation and a conservative
 * angular speed gate; obstacle rotation during the tick is not swept. */
int collision_ccd_sweep_clamp(rigidbody *bodies, int body_count, float dt);
int collision_ccd_sweep_clamp_full(rigidbody *bodies, int body_count, float dt, float *time_remaining_out,
                                    const mpe_config_t *cfg, float *best_tois_out, unsigned char *hit_flags_out);
/* World-aware CCD entry: forwards the world's config snapshot. */
struct physics_world;
int collision_ccd_sweep_clamp_world(struct physics_world *world, float dt);
void contact_cache_save(struct physics_world *world, collision_data *manifolds, int count); /* MFS_131 */
void contact_cache_clear(struct physics_world *world); /* MFS_131 */

bool collision_static_plane_sphere(rigidbody *plane_body, rigidbody *sphere, float plane_y, collision_data *collision_output_data,
                                   const mpe_config_t *cfg);
bool collision_static_plane_cube(rigidbody *plane_body, rigidbody *cube, float plane_y, collision_data *collision_output_data,
                                 const mpe_config_t *cfg);
bool collision_static_plane_body(rigidbody *plane_body, rigidbody *body, float plane_y,
                                  collision_data *collision_output_data, const mpe_config_t *cfg);

void contact_cache_stats_reset(struct physics_world *world);
int contact_cache_get_hits(const struct physics_world *world);
int contact_cache_get_misses(const struct physics_world *world);
/* Pair-novelty probe: true if this id pair has any entry saved from a prior
 * tick (either order). Drives wake-on-first-touch (see implementation). */
bool contact_cache_has_pair(struct physics_world *world, uint32_t id_a, uint32_t id_b);
/* Cylinder-vs-object narrowphase. TRUE solid-cylinder geometry:
 * flat end-caps (SDF), rim circle, inside SDF branch; segment-OBB convex
 * exact for cube; coaxial face-gap + parallel 2-point for cyl-cyl.
 * Barrel contacts exact; see collision_cylinder.c. */
bool collision_cylinder_sphere(rigidbody *cyl, rigidbody *sph,
                               collision_data *out, const mpe_config_t *cfg);
bool collision_cylinder_cube(rigidbody *cyl, rigidbody *cube,
                             collision_data *out, const mpe_config_t *cfg);
bool collision_cylinder_cylinder(rigidbody *cyl_a, rigidbody *cyl_b,
                                 collision_data *out, const mpe_config_t *cfg);

#endif

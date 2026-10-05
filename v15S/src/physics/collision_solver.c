/* GTK4-PREP: zero GUI headers in physics. */
#include "collision_mechanics.h"
#ifndef MPE_SPLIT_NO_ANGULAR
#define MPE_SPLIT_NO_ANGULAR 0
#endif
#include "../core/physics_world.h"
#include "../core/rigidbody.h"
#include "../core/det_math.h"
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
/* World-space anisotropy (MATERIAL) axis for a contact, honouring each body's
 * optional reference frame (friction_anisotropy_frame).
 *
 * This is the frame the elliptical Coulomb cone is defined in, and therefore
 * the one the solver must build its tangent basis from. Resolving it in the
 * wrong frame is not cosmetic: t1 follows the SLIP direction, so an axis
 * carried by a spinning hub sweeps with the hub and the contact silently
 * loses the rail it was supposed to have.
 *
 * Lives here rather than in collision_mechanics.h because it needs the full
 * physics_world definition, and physics_world.h includes collision_mechanics.h.
 * `world` may be NULL, in which case a framed axis degrades to the owner's own
 * frame (the only defined choice when the frame body is unknown). */
static bool a3_contact_anisotropy_axis (const physics_world *world, const rigidbody *body_a, const rigidbody *body_b,
                                        vector3 *out_axis) {
    const rigidbody *owner = NULL;
    for (int side = 0; side < 2; side++) {
        const rigidbody *rb = (side == 0) ? body_a : body_b;
        if (rb && rb -> friction_anisotropic) {
            owner = rb;
            break;
        }
    }
    if (!owner) {
        return false;
    }
    const rigidbody *frame = owner;
    if (world && (owner -> friction_anisotropy_frame != 0)) {
        /* FIX-AUDIT-DESPOT: was an O(n) id scan per CONTACT (prepare runs
         * it per contact per tick: O(contacts*bodies)). Route through the
         * world's id->index cache (O(1) hit; the lookup verifies against
         * the live array and misses safely, so stale frames degrade to the
         * owner's own frame, never to a wrong body). */
        int frame_idx = physics_world_index_by_id ((physics_world *) world, owner -> friction_anisotropy_frame);
        if ((frame_idx >= 0) && (frame_idx < world -> body_count)) {
            frame = &world -> bodies [frame_idx];
        }
    }
    * out_axis = vector4_rotate_to_vector3 (frame -> orientation, owner -> friction_anisotropy_axis);
    return true;
}
/* Tangent basis for an anisotropic contact, built from the MATERIAL axis
 * rather than from the slip direction.
 *
 * Why: the elliptical cone is only the anisotropic Coulomb law when its
 * semi-axes are measured in the material frame. The solver's normal t1 tracks
 * the slip, so evaluating the ellipse in a slip-aligned basis collapses it to
 * "limit the total tangential impulse, slightly differently by direction",
 * which cannot express a rail at all. With mu_across = 0 every slip direction
 * has some component across the material axis, so that formulation zeroes the
 * WHOLE impulse and the wheel loses all grip instead of gaining a direction -
 * which is exactly what the mecanum sweep measured. In the material frame the
 * same mu_across = 0 pins only the second tangent and leaves the first free: a
 * true one-way rail, which is what a free roller physically transmits.
 *
 * The material frame also exists when there is no slip, which is precisely when
 * a rail contact most needs it. Returns false when there is no usable material
 * frame (not anisotropic, or axis parallel to the contact normal), in which
 * case the caller keeps its ordinary slip-aligned basis. */
static bool a3_anisotropic_tangent_frame (const physics_world *world, const rigidbody *body_a, const rigidbody *body_b,
                                          vector3 normal, vector3 *out_t1, vector3 *out_t2) {
    vector3 axis;
    if (!a3_contact_anisotropy_axis (world, body_a, body_b, &axis)) {
        return false;
    }
    if (!isfinite (axis.x) || !isfinite (axis.y) || !isfinite (axis.z) || (vector3_length_squared (axis) < 1.0e-8f)) {
        return false;
    }
    /* Project into the contact plane; an axis normal to the contact describes
     * no in-plane direction, so it is not a usable tangent frame. */
    vector3 t1 = vector3_subtraction (axis, vector3_scaling (normal, vector3_dot (axis, normal)));
    if (vector3_length_squared (t1) < 1.0e-6f) {
        return false;
    }
    t1 = vector3_normalisation (t1);
    vector3 t2 = vector3_cross (normal, t1);
    if (vector3_length_squared (t2) < 1.0e-6f) {
        return false;
    }
    * out_t1 = t1;
    *out_t2 = vector3_normalisation (t2);
    return true;
}
#define contact_hash_bits 12
#define contact_hash_size (1 << contact_hash_bits)
#define contact_hash_mask (contact_hash_size - 1)
static inline uint32_t contact_pair_key (uint32_t id_a, uint32_t id_b) {
    uint32_t lo = (id_a < id_b) ? id_a : id_b;
    uint32_t hi = (id_a < id_b) ? id_b : id_a;
    uint64_t key = ((uint64_t) lo << 32) | (uint64_t) hi;
    /* splitmix64 finalizer over the combined key. */
    key ^= key >> 30;
    key *= 0xbf58476d1ce4e5b9ULL;
    key ^= key >> 27;
    key *= 0x94d049bb133111ebULL;
    key ^= key >> 31;
    return (uint32_t) (key & contact_hash_mask);
}
static inline vector4 collision_inverse_orientation (vector4 orientation) {
    return (vector4) {orientation.w, -orientation.x, -orientation.y, -orientation.z};
}
static inline vector3 collision_world_offset_to_body_local (rigidbody *body, vector3 world_offset) {
    return vector4_rotate_to_vector3 (collision_inverse_orientation (body -> orientation), world_offset);
}
static inline vector3 collision_body_local_to_world_offset (rigidbody *body, vector3 local_offset) {
    return vector4_rotate_to_vector3 (body -> orientation, local_offset);
}
/* FIX-AUDIT-DESPOT: thin wrapper over the shared stamp in
 * collision_mechanics.h (single source of truth with contact_cache_save).
 * Kept under the legacy a3_task05_ name so call sites below don't churn;
 * bodies must match exactly or warm-start injects stale impulses. */
static uint32_t a3_task05_body_property_stamp (const rigidbody *rigid_body) {
    return a3_contact_cache_body_stamp (rigid_body);
}
static bool a3_task05_cached_impulses_are_usable (float normal_impulse, float tangent_impulse) {
    if ((!isfinite (normal_impulse)) || (!isfinite (tangent_impulse))) {
        return false;
    }
    if (normal_impulse < 0.0f) {
        return false;
    }
    if (fabsf (normal_impulse) > 1000000.0f) {
        return false;
    }
    if (fabsf (tangent_impulse) > 1000000.0f) {
        return false;
    }
    return true;
}
static int contact_cache_match_role (const cached_contact *cc, uint32_t id_a, uint32_t id_b, uint32_t stamp_a,
                                     uint32_t stamp_b, vector3 local_a, vector3 local_b, float match_dist_sq) {
    if ((!cc) || (id_a == 0) || (id_b == 0)) {
        return 0;
    }
    if ((cc -> object_id_a == id_a) && (cc -> object_id_b == id_b) && (cc -> property_stamp_a == stamp_a) &&
        (cc -> property_stamp_b == stamp_b)) {
        /* TRUTH: strict both-side matching. Old side-A-only aliased two B
         * bodies sharing an A anchor within 5cm (wrong impulse injection).
         * Both material points must coincide; resting contacts satisfy this
         * exactly (body-local storage survives rigid translation). */
        float dist_a_sq = vector3_length_squared (vector3_subtraction (cc -> local_position_a, local_a));
        float dist_b_sq = vector3_length_squared (vector3_subtraction (cc -> local_position_b, local_b));
        if ((dist_a_sq < match_dist_sq) && (dist_b_sq < match_dist_sq) &&
            (a3_task05_cached_impulses_are_usable (cc -> accumulated_normal_impulse, cc -> accumulated_tangent_impulse))) {
            return 1;
        }
        return 0;
    }
    if ((cc -> object_id_a == id_b) && (cc -> object_id_b == id_a) && (cc -> property_stamp_a == stamp_b) &&
        (cc -> property_stamp_b == stamp_a)) {
        float dist_sq_ab = vector3_length_squared (vector3_subtraction (cc -> local_position_a, local_b));
        float dist_sq_ba = vector3_length_squared (vector3_subtraction (cc -> local_position_b, local_a));
        if ((dist_sq_ab < match_dist_sq) && (dist_sq_ba < match_dist_sq) &&
            (a3_task05_cached_impulses_are_usable (cc -> accumulated_normal_impulse, cc -> accumulated_tangent_impulse))) {
            return 2;
        }
    }
    return 0;
}
static bool contact_cache_adoptable (const cached_contact *cc, uint32_t id_a, uint32_t id_b, uint32_t stamp_a,
                                     uint32_t stamp_b, vector3 local_a, vector3 local_b, float match_dist_sq) {
    if ((!cc) || (id_a == 0) || (id_b == 0)) {
        return false;
    }
    if (!((cc -> object_id_a == id_a) && (cc -> object_id_b == id_b) && (cc -> property_stamp_a == stamp_a) &&
          (cc -> property_stamp_b == stamp_b))) {
        return false;
    }
    /* TRUTH: require BOTH sides like match_role. Side-A-only matching let
     * two B bodies sharing one A (within 5cm) share tangent memory. */
    float dist_a_sq = vector3_length_squared (vector3_subtraction (cc -> local_position_a, local_a));
    if (dist_a_sq >= match_dist_sq) {
        return false;
    }
    float dist_b_sq = vector3_length_squared (vector3_subtraction (cc -> local_position_b, local_b));
    if (dist_b_sq >= match_dist_sq) {
        return false;
    }
    return vector3_length_squared (cc -> tangent_dir) > 0.0001f;
}
/* Forward: defined beside the other per-phase passes below; called from
 * prepare (pre-force tick-start selection). */
void collision_snapshot_friction_mu (collision_data *manifolds, int manifold_count, const mpe_config_t *cfg);
void collision_prepare_solver (struct physics_world *world, collision_data *source, collision_data *m, float dt) {
    *m = *source;
    if (dt <= 0.0f) {
        dt = 1.0f / 60.0f;
    }
    /* DESPOT-2026-10-01: refresh world inertia from CURRENT orientation.
     * inverse_inertia_system was last written by rb_integrate_velocity at the
     * START of the previous tick; orientation has since advanced, so k built
     * from the cache is 1 tick stale (error O(|w|dt·I_aniso), systematic for
     * tumblers). Recompute R·I⁻¹·Rᵀ here from exact mults (deterministic). */
    for (int bi = 0; bi < 2; bi++) {
        rigidbody *b = (bi == 0) ? m -> object_a : m -> object_b;
        if (b && !b -> static_state && !b -> kinematic && rigidbody_effective_inv_mass (b) > 0.0f) {
            math3 R = vector4_to_math3 (b -> orientation);
            math3 Rt = math3_transposition (R);
            b -> inverse_inertia_system =
                math3_multiplication (R, math3_multiplication (b -> inverse_inertia_tensor_local, Rt));
        }
    }
    /* Per-world cache; a missing cache degrades to all-miss (cold solve).
     * No global fallback remains. */
    cached_contact *cache_array = (world) ? world -> world_contact_cache : NULL;
    int cache_count = (world) ? world -> world_contact_cache_count : 0;
    int32_t *hash_head = (world) ? world -> contact_hash_head : NULL;
    if (!cache_array) {
        cache_count = 0;
    }
    for (int i = 0; i < m -> contact_count; i++) {
        contact_point_data *cp = &m -> contacts [i];
        cp -> ra = vector3_subtraction (cp -> position, m -> object_a -> position);
        cp -> rb = vector3_subtraction (cp -> position, m -> object_b -> position);
        cp -> local_position_a =
            collision_world_offset_to_body_local (m -> object_a, cp -> ra); /* A3_PATCH_19_BODY_LOCAL_WARM_START */
        cp -> local_position_b = collision_world_offset_to_body_local (m -> object_b, cp -> rb);
        cp -> accumulated_normal_impulse = 0.0f;
        cp -> accumulated_tangent_impulse = 0.0f;
        cp -> accumulated_tangent2_impulse = 0.0f;
        cp -> snap_friction_mu = -1.0f; /* unset until collision_snapshot_friction_mu runs */
        /* AUDIT: no velocity-level Baumgarte bias is computed here on
         * purpose (see header). g_cfg.solver.bias_factor drives the
         * positional split-impulse correction instead, where bias velocity
         * cannot leak into impulses. An earlier revision computed a
         * per-contact separation_bias that nothing read. */
        /* MPE_TASK_05_CACHE_MATCH_BEGIN */
        uint32_t cache_id_a = (m -> object_a) ? m -> object_a -> object_id : 0;
        uint32_t cache_id_b = (m -> object_b) ? m -> object_b -> object_id : 0;
        uint32_t cache_stamp_a = a3_task05_body_property_stamp (m -> object_a);
        uint32_t cache_stamp_b = a3_task05_body_property_stamp (m -> object_b);
        int cache_match_found = 0;
        /* Per-world warm-start match distance (was global). */
        const mpe_config_t *prep_cfg = world ? mpe_world_cfg (world) : &g_cfg;
        float prep_match_sq = prep_cfg -> solver.warm_start_match_dist_sq;
        /* Warm-start matching: strict both-side material-point coincidence
         * (see contact_cache_match_role). A hit adopts cached impulses at
         * full step — no damped SOR, no provenance flags. NOTE (truth):
         * only the normal + PRIMARY tangent are restored. The second disc
         * tangent is deliberately never cached: t2_new = n×t1_new can point
         * anywhere relative to a cached t2_old when frames rotate, and
         * restoring it injected sideways energy; cold t2 re-converges in
         * the relaxation sweeps. Normal is frame-independent: always warm. */
        if ((hash_head) && (cache_id_a != 0) && (cache_id_b != 0)) {
            /* Hash walk: visits the pair's entries in save order, i.e. the
             * same first-hit the legacy linear scan below would find. */
            uint32_t slot0 = contact_pair_key (cache_id_a, cache_id_b);
            for (int32_t slot = hash_head [slot0], guard = 0;
                 (slot >= 0) && (slot < cache_count) && (guard <= cache_count);
                 slot = cache_array [slot].hash_next, guard++) {
                cached_contact *cc = &cache_array [slot];
                int role = contact_cache_match_role (cc, cache_id_a, cache_id_b, cache_stamp_a, cache_stamp_b,
                                                     cp -> local_position_a, cp -> local_position_b, prep_match_sq);
                if (role == 1) {
                    cp -> accumulated_normal_impulse = fmaxf (cc -> accumulated_normal_impulse, 0.0f);
                    cp -> accumulated_tangent_impulse = cc -> accumulated_tangent_impulse;
                    cache_match_found = 1;
                    break;
                } else if (role == 2) {
                    /* Swapped body order: normal stays positive (manifold
                     * normal already points A->B); tangent reverses with
                     * the relative-velocity order. */
                    cp -> accumulated_normal_impulse = fmaxf (cc -> accumulated_normal_impulse, 0.0f);
                    cp -> accumulated_tangent_impulse = -cc -> accumulated_tangent_impulse;
                    cache_match_found = 1;
                    break;
                }
            }
        } else {
            /* Legacy linear fallback (no hash heads, e.g. malloc failure). */
            for (int c = 0; c < cache_count; c++) {
                cached_contact *cc = &cache_array [c];
                int role = contact_cache_match_role (cc, cache_id_a, cache_id_b, cache_stamp_a, cache_stamp_b,
                                                     cp -> local_position_a, cp -> local_position_b, prep_match_sq);
                if (role == 1) {
                    cp -> accumulated_normal_impulse = fmaxf (cc -> accumulated_normal_impulse, 0.0f);
                    cp -> accumulated_tangent_impulse = cc -> accumulated_tangent_impulse;
                    cache_match_found = 1;
                    break;
                } else if (role == 2) {
                    cp -> accumulated_normal_impulse = fmaxf (cc -> accumulated_normal_impulse, 0.0f);
                    cp -> accumulated_tangent_impulse = -cc -> accumulated_tangent_impulse;
                    cache_match_found = 1;
                    break;
                }
            }
        }
        /* MPE_TASK_05_CACHE_MATCH_END */
        if (world) {
            if (cache_match_found) {
                world -> contact_cache_hits++;
            } else {
                world -> contact_cache_misses++;
            }
        }
        /* PHYSICS-TRUTH (F10 10-stack): NORMAL warm-start is DISABLED here.
         *
         * This line unconditionally discards whatever the cache lookup above
         * restored. It is deliberate, and the reason is measured: restoring
         * last tick's normal as the iterations' seed ejected the 10-stack at
         * ~13 m/s across guard/cap/cone/tightness/adoption ablations, while
         * the restored values themselves stayed healthy (~0.5, bounded: no
         * save-bigger loop).
         *
         * DESPOT-2026-10-03: THE CLAIM THAT "TANGENT MEMORY STILL RESTORES"
         * WAS FALSE, AND PROVING IT CHANGED THE DOCUMENTATION RATHER THAN THE
         * CODE.
         *
         * The restored tangent impulse was projected onto the Coulomb cone at
         * application time, passing cp->accumulated_normal_impulse as the
         * cone's fn. The normal is cold-zeroed a few lines above, so fn is
         * exactly 0.0 on every contact, every tick -- and
         * a3_anisotropic_coulomb_clamp() opens with
         *     if (!(fn > 0.0f) || !isfinite(fn)) { *out1=0; *out2=0; return; }
         * Instrumented on a 4-cube stack taken to rest:
         *
         *     restored t1 = 0.154651 | fn passed to cone = 0.000000
         *     | AFTER clamp  t1 = 0.000000
         *
         * So the tangent warm start was deleted by the very clamp that was
         * added to protect it, and THIS ENGINE HAS NO WORKING WARM START ON
         * ANY ROW. That also explains why it needs 96-128 iterations for a
         * 10-cube stack -- 12-30x the published practitioner budget (Catto,
         * Solver2D 2024: "typically 4 to 8 iterations") -- since with no
         * cross-tick information carry the whole column is re-propagated from
         * the floor every tick.
         *
         * Restoring it the way the old comment described (seed unclamped, let
         * the sweep enforce the cone once it has a real lambda_n) was
         * implemented and MEASURED, and it is a clear REGRESSION, so the clamp
         * stays. Worst pairwise cube-cube overlap, 10-cube stack:
         *
         *     iterations         32       64       96      128
         *     gravity -9.81   0.1134   0.0145   0.0000   0.0000  (current)
         *     with warm start 0.1813   0.0763   0.0596   0.0000
         *     gravity -17.0   0.2084   0.3105   0.0024   0.0000  (current)
         *     with warm start 0.2274   0.0920   0.0558   0.0700
         *
         * Better at 64 iterations under extreme gravity, worse almost
         * everywhere else, and it turns the suite RED in all five regimes:
         * f10_long_run loses 23 of 27 bodies to sleep (4/27 asleep,
         * run_max > 2.0) and f11_torture's interpenetration gate fires at
         * 0.2791 m against a 0.05 m bound.
         *
         * CONCLUSION, recorded so it is not re-derived: the no-warm-start
         * behaviour is what this engine has actually been tuned around, and
         * the old comment described an aspiration rather than the
         * implementation. A tangential seed inconsistent with a cold normal
         * injects a spurious tangential velocity the solver must then remove,
         * which costs more than the seed saves. The tangent FRAME adoption
         * below IS live and does help; only the impulse seed is dead.
         * Re-enabling an effective warm start needs a formulation that seeds
         * the NORMAL too, which is separate work with a stability proof
         * attached, not a one-line un-clamp.
         *
         * The tangent frame still adopts. The normal therefore SOLVES FROM
         * ZERO EVERY TICK, converging at 64-128
         * iterations for a 10-high stack; low-iteration tall stacks may creep
         * — tune iterations, not seeds.
         *
         * HONESTY NOTE: readme.md and RELEASE_POLICY.md advertise
         * "warm-starting contact solver ... for stable stacking" and
         * "strict warm-start" as headline features. That is HALF TRUE. Only
         * the TANGENT impulses are warm-started; the normal — which carries
         * the large majority of the impulse — is cold every tick. The docs
         * have been corrected to say so. Re-enable normal warm-start only
         * with a stability proof on f10_long_run. */
        cp -> accumulated_normal_impulse = 0.0f;
        vector3 va = vector3_addition (m -> object_a -> velocity, vector3_cross (m -> object_a -> angular_velocity, cp -> ra));
        vector3 vb = vector3_addition (m -> object_b -> velocity, vector3_cross (m -> object_b -> angular_velocity, cp -> rb));
        vector3 rel_vel = vector3_subtraction (vb, va);
        float vn_initial = vector3_dot (rel_vel, m -> normal_vector);
        /* TRUTH: feed sleep gating. max_relative_speed_sq is reset each tick
         * by the step and MUST be written here (contact processing); without
         * writers the relative_calm gate is dead (always 0 < thresh) and
         * riders sleep on moving platforms. */
        {
            float rsq = vector3_length_squared (rel_vel);
            if (isfinite (rsq)) {
                if (rsq > m -> object_a -> max_relative_speed_sq)
                    m -> object_a -> max_relative_speed_sq = rsq;
                if (rsq > m -> object_b -> max_relative_speed_sq)
                    m -> object_b -> max_relative_speed_sq = rsq;
            }
        }
        /* Poisson gate input: pre-solve approach speed of this tick. */
        cp -> impact_velocity = vn_initial;
        vector3 ra_cross_n = vector3_cross (cp -> ra, m -> normal_vector);
        vector3 rb_cross_n = vector3_cross (cp -> rb, m -> normal_vector);
        vector3 ang_a = vector3_cross (
            math3_multiplication_vector3 (rigidbody_effective_inv_inertia (m -> object_a), ra_cross_n), cp -> ra);
        vector3 ang_b = vector3_cross (
            math3_multiplication_vector3 (rigidbody_effective_inv_inertia (m -> object_b), rb_cross_n), cp -> rb);
        float k_normal = rigidbody_effective_inv_mass (m -> object_a) + rigidbody_effective_inv_mass (m -> object_b) +
                         vector3_dot (vector3_addition (ang_a, ang_b), m -> normal_vector);
        cp -> effective_mass_normal = (k_normal > 0.0f) ? (1.0f / k_normal) : 0.0f;
        vector3 rel_vel_tangent = vector3_subtraction (rel_vel, vector3_scaling (m -> normal_vector, vn_initial));
        float tangent_speed = vector3_length (rel_vel_tangent);
        /* Coulomb tangent frame. Stick/slip select mirrors the sweep
         * (static below thresh, kinetic above): below thresh the slip
         * direction is micro-motion noise, so a resting contact must use
         * the remembered tangent or no frame at all — firing full warm
         * friction along a noise direction walks stacks sideways (F10
         * 10-stack ejects at 13 m/s with noise frames, stands with
         * adopted-or-zero). True sliding keeps the slip direction. */
        const mpe_config_t *frame_cfg = world ? mpe_world_cfg (world) : &g_cfg;
        float stick_thresh = frame_cfg -> solver.static_friction_thresh;
        if (!(stick_thresh > 0.0f) || !isfinite (stick_thresh)) {
            stick_thresh = 0.02f;
        }
        bool frame_sliding = (tangent_speed >= stick_thresh);
        vector3 adopted_tangent = vector3_zero ();
        if (!frame_sliding) {
            /* Same first-hit as the legacy full-array scan (see hash note
             * above): the bucket holds exactly the matchable entries in
             * save order. */
            if ((hash_head) && (cache_id_a != 0) && (cache_id_b != 0)) {
                uint32_t slot0 = contact_pair_key (cache_id_a, cache_id_b);
                for (int32_t slot = hash_head [slot0], guard = 0;
                     (slot >= 0) && (slot < cache_count) && (guard <= cache_count);
                     slot = cache_array [slot].hash_next, guard++) {
                    cached_contact *cc = &cache_array [slot];
                    if (contact_cache_adoptable (cc, cache_id_a, cache_id_b, cache_stamp_a, cache_stamp_b,
                                                 cp -> local_position_a, cp -> local_position_b, prep_match_sq)) {
                        adopted_tangent = cc -> tangent_dir;
                        break;
                    }
                }
            } else {
                for (int c = 0; c < cache_count; c++) {
                    cached_contact *cc = &cache_array [c];
                    if (contact_cache_adoptable (cc, cache_id_a, cache_id_b, cache_stamp_a, cache_stamp_b,
                                                 cp -> local_position_a, cp -> local_position_b, prep_match_sq)) {
                        adopted_tangent = cc -> tangent_dir;
                        break;
                    }
                }
            }
            if (vector3_length_squared (adopted_tangent) > 0.0001f) {
                /* Re-orthogonalize against the current normal. */
                adopted_tangent = vector3_subtraction (
                    adopted_tangent,
                    vector3_scaling (m -> normal_vector, vector3_dot (adopted_tangent, m -> normal_vector)));
                if (vector3_length_squared (adopted_tangent) > 0.0001f) {
                    adopted_tangent = vector3_normalisation (adopted_tangent);
                } else {
                    adopted_tangent = vector3_zero ();
                }
            }
        }
        /* Frame select: true sliding keeps the slip direction
         * (meaningful); sticking uses the remembered direction or, with
         * no memory yet, no frame (normal-only this tick — the sweep
         * cannot invent a hold direction from noise).
         *
         * Anisotropic contacts are the exception and take their basis from
         * the MATERIAL frame instead (see a3_anisotropic_tangent_frame): the
         * elliptical cone is only the anisotropic Coulomb law when measured
         * against the material axes, and a slip-aligned basis reduces it to a
         * magnitude limit that cannot express a rail. The material frame is
         * also available when there is no slip at all, which is exactly when a
         * rail contact most needs it. */
        vector3 aniso_t1 = vector3_zero ();
        vector3 aniso_t2 = vector3_zero ();
        const bool have_material_frame =
            a3_anisotropic_tangent_frame (world, m -> object_a, m -> object_b, m -> normal_vector, &aniso_t1, &aniso_t2);
        if (have_material_frame) {
            /* DESPOT-FIX: this branch was EMPTY — the material frame was
             * computed and discarded, so anisotropic contacts kept a stale
             * (or zero) tangent: friction froze at warm-start and rollers
             * could never develop rail force (measured strafe ~0.0005 m).
             * Assign the material basis; tangent2/effective-masses below
             * then derive from it consistently. */
            cp -> tangent_vector = aniso_t1;
        } else {
            if (tangent_speed > 0.0001f) {
                cp -> tangent_vector = vector3_scaling (rel_vel_tangent, -1.0f / tangent_speed);
            } else {
                cp -> tangent_vector = adopted_tangent;
            }
        }
        if (vector3_length_squared (cp -> tangent_vector) > 0.0001f) {
            /* Second tangent completes the Coulomb disc: t2 = n x t1. */
            cp -> tangent2 = vector3_cross (m -> normal_vector, cp -> tangent_vector);
            if (vector3_length_squared (cp -> tangent2) > 0.0001f) {
                cp -> tangent2 = vector3_normalisation (cp -> tangent2);
            } else {
                cp -> tangent2 = vector3_zero ();
            }
            vector3 ra_cross_t = vector3_cross (cp -> ra, cp -> tangent_vector);
            vector3 rb_cross_t = vector3_cross (cp -> rb, cp -> tangent_vector);
            vector3 ang_a_t = vector3_cross (
                math3_multiplication_vector3 (rigidbody_effective_inv_inertia (m -> object_a), ra_cross_t), cp -> ra);
            vector3 ang_b_t = vector3_cross (
                math3_multiplication_vector3 (rigidbody_effective_inv_inertia (m -> object_b), rb_cross_t), cp -> rb);
            float k_tangent = rigidbody_effective_inv_mass (m -> object_a) + rigidbody_effective_inv_mass (m -> object_b) +
                              vector3_dot (vector3_addition (ang_a_t, ang_b_t), cp -> tangent_vector);
            cp -> effective_mass_tangent = (k_tangent > 0.0f) ? (1.0f / k_tangent) : 0.0f;
            if (vector3_length_squared (cp -> tangent2) > 0.0001f) {
                vector3 ra_cross_t2 = vector3_cross (cp -> ra, cp -> tangent2);
                vector3 rb_cross_t2 = vector3_cross (cp -> rb, cp -> tangent2);
                vector3 ang_a_t2 = vector3_cross (
                    math3_multiplication_vector3 (rigidbody_effective_inv_inertia (m -> object_a), ra_cross_t2), cp -> ra);
                vector3 ang_b_t2 = vector3_cross (
                    math3_multiplication_vector3 (rigidbody_effective_inv_inertia (m -> object_b), rb_cross_t2), cp -> rb);
                float k_tangent2 = rigidbody_effective_inv_mass (m -> object_a) +
                                   rigidbody_effective_inv_mass (m -> object_b) +
                                   vector3_dot (vector3_addition (ang_a_t2, ang_b_t2), cp -> tangent2);
                cp -> effective_mass_tangent2 = (k_tangent2 > 0.0f) ? (1.0f / k_tangent2) : 0.0f;
            } else {
                cp -> effective_mass_tangent2 = 0.0f;
            }
        } else {
            /* No slip and no remembered direction: nothing to hold against. */
            cp -> tangent_vector = vector3_zero ();
            cp -> effective_mass_tangent = 0.0f;
            cp -> tangent2 = vector3_zero ();
            cp -> effective_mass_tangent2 = 0.0f;
            cp -> accumulated_tangent2_impulse = 0.0f;
        }
        /* NOTE (truth, measured): the primary tangent keeps its cached
         * magnitude even on fresh slip (original static-hold behavior). An
         * experiment zeroing it here fixed a 127 rad/s wheel singularity but
         * regressed the 6-cube stack (drift 0.27m vs 0.003m), so it was
         * reverted: extreme-spin contacts are handled by keeping the wheel
         * test in the resolvable regime (see driven_wheel_test), not by
         * weakening everyday friction. The second disc tangent is never
         * cached (cold every tick): t2_new = n×t1_new can point anywhere
         * relative to a cached t2_old when frames rotate. */
        if (cp -> accumulated_normal_impulse != 0.0f || cp -> accumulated_tangent_impulse != 0.0f ||
            cp -> accumulated_tangent2_impulse != 0.0f) {
            /* TRUTH: apply restored support only to an APPROACHING contact.
             * Last tick's impulse is meaningless when the pair is separating
             * this tick (whipping rim contact, liftoff): firing it anyway
             * injects approach that isn't there, and the cache loop (save
             * bigger, restore bigger) turns it exponential. A separating
             * contact starts cold; the sweeps below converge it. */
            vector3 va_now =
                vector3_addition (m -> object_a -> velocity, vector3_cross (m -> object_a -> angular_velocity, cp -> ra));
            vector3 vb_now =
                vector3_addition (m -> object_b -> velocity, vector3_cross (m -> object_b -> angular_velocity, cp -> rb));
            float vn_now = vector3_dot (vector3_subtraction (vb_now, va_now), m -> normal_vector);
            if (vn_now >= 0.0f) {
                cp -> accumulated_normal_impulse = 0.0f;
                cp -> accumulated_tangent_impulse = 0.0f;
                cp -> accumulated_tangent2_impulse = 0.0f;
            } else {
                /* TRUTH: normal starts cold (see above), so no magnitude cap is
             * needed: stale hits cannot bomb through a zero seed, and
             * capping a trusted guess against local need starves stacked
             * contacts (base of a 10-stack needs ~10x local need). Tangent
             * is projected onto the current Coulomb cone likewise (the
             * sweep loop does this every iteration; application must not
             * bypass). */
                {
                    float mus_a = m -> object_a ? m -> object_a -> friction_static : 0.0f;
                    float mus_b = m -> object_b ? m -> object_b -> friction_static : 0.0f;
                    float mu_cap = (mus_a < mus_b) ? mus_a : mus_b;
                    /* TRUTH: mirror the sweep's stick/slip select (static below
                 * thresh, kinetic above). Capping sliding restored friction
                 * at mu_s overestimates what the sweep allows (mu_k) and
                 * re-admits sideways energy through application. */
                    {
                        const mpe_config_t *mu_cfg = world ? mpe_world_cfg (world) : &g_cfg;
                        float mks_a = m -> object_a ? m -> object_a -> friction_kinetic : 0.0f;
                        float mks_b = m -> object_b ? m -> object_b -> friction_kinetic : 0.0f;
                        float mu_k = (mks_a < mks_b) ? mks_a : mks_b;
                        float sth = mu_cfg -> solver.static_friction_thresh;
                        if (!(sth > 0.0f) || !isfinite (sth))
                            sth = 0.02f;
                        if (tangent_speed >= sth)
                            mu_cap = mu_k;
                    }
                    if (!(mu_cap >= 0.0f) || !isfinite (mu_cap))
                        mu_cap = 0.0f;
                    float t1 = cp -> accumulated_tangent_impulse, t2 = cp -> accumulated_tangent2_impulse;
                    /* Same cone the sweep below uses, so warm start can never
                 * re-admit an impulse the sweep would immediately clamp out.
                 * Isotropic bodies resolve to mu_roll == mu_lateral, i.e. the
                 * legacy disc. */
                    float mu_roll, mu_lateral;
                    vector3 aniso_axis;
                    a3_contact_friction_cone (m -> object_a, m -> object_b, mu_cap, &mu_roll, &mu_lateral, &aniso_axis);
                    /* DESPOT-FIX: the ellipse axis IS the basis t1 by construction
                 * (prepare assigned the material frame to tangent_vector, so
                 * projecting it back is identity). Passing the cone's
                 * owner-frame axis instead disagrees whenever a reference
                 * frame is set (rail hubs: cone rotates the stored
                 * chassis-frame rail by the SPINNING hub; basis is the steady
                 * chassis frame) — the clamp then measured the ellipse in a
                 * sweeping frame and the rail force smeared to nothing. The
                 * separate frame-aware re-resolution is subsumed by this. */
                    (void) aniso_axis;
                    a3_anisotropic_coulomb_clamp (m -> normal_vector, cp -> tangent_vector, cp -> tangent2, t1, t2,
                                                  cp -> accumulated_normal_impulse, mu_roll, mu_lateral,
                                                  cp -> tangent_vector, &cp -> accumulated_tangent_impulse,
                                                  &cp -> accumulated_tangent2_impulse);
                }
                vector3 impulse = vector3_addition (
                    vector3_scaling (m -> normal_vector, cp -> accumulated_normal_impulse),
                    vector3_addition (vector3_scaling (cp -> tangent_vector, cp -> accumulated_tangent_impulse),
                                      vector3_scaling (cp -> tangent2, cp -> accumulated_tangent2_impulse)));
                if (rigidbody_effective_inv_mass (m -> object_a) > 0.0f) {
                    m -> object_a -> velocity = vector3_subtraction (
                        m -> object_a -> velocity, vector3_scaling (impulse, rigidbody_effective_inv_mass (m -> object_a)));
                    m -> object_a -> angular_velocity = vector3_subtraction (
                        m -> object_a -> angular_velocity,
                        math3_multiplication_vector3 (rigidbody_effective_inv_inertia (m -> object_a),
                                                      vector3_cross (cp -> ra, impulse)));
                }
                if (rigidbody_effective_inv_mass (m -> object_b) > 0.0f) {
                    m -> object_b -> velocity = vector3_addition (
                        m -> object_b -> velocity, vector3_scaling (impulse, rigidbody_effective_inv_mass (m -> object_b)));
                    m -> object_b -> angular_velocity =
                        vector3_addition (m -> object_b -> angular_velocity,
                                          math3_multiplication_vector3 (rigidbody_effective_inv_inertia (m -> object_b),
                                                                        vector3_cross (cp -> rb, impulse)));
                }
            }
        }
        /* Poisson base: compression impulse entering the iterations (warm
         * start included). The restitution pass pays e over the delta. */
        cp -> base_normal_impulse = cp -> accumulated_normal_impulse;
    }
    /* DESPOT-2026-10-04: tick-start friction selection, recorded HERE (not
     * at solve time). Prepare runs pre-force-integration, so these are the
     * tick's opening velocities — the true was-it-sticking state. A step
     * path that snapshots post-force instead sees the tick's own injected
     * F*dt (0.117 m/s at 7N) above the static gate and wrongly solves the
     * whole tick kinetic (measured: mu_s=0.9/1.5 broke at ~5.9N). */
    collision_snapshot_friction_mu (m, 1, world ? mpe_world_cfg (world) : &g_cfg);
}
static void collision_manifold_merge_sort (const float *keys, int *order, int *scratch, int n) {
    for (int i = 0; i < n; i++) {
        order [i] = i;
    }
    int *src = order;
    int *dst = scratch;
    for (int width = 1; width < n; width *= 2) {
        for (int lo = 0; lo < n; lo += 2 * width) {
            int mid = lo + width < n ? lo + width : n;
            int hi = lo + 2 * width < n ? lo + 2 * width : n;
            int a = lo, b = mid, o = lo;
            while (a < mid && b < hi) {
                float ka = keys [src [a]];
                float kb = keys [src [b]];
                bool take_a;
                if (ka < kb) {
                    take_a = true;
                } else if (ka > kb) {
                    take_a = false;
                } else {
                    take_a = src [a] < src [b];
                }
                dst [o++] = take_a ? src [a++] : src [b++];
            }
            while (a < mid) {
                dst [o++] = src [a++];
            }
            while (b < hi) {
                dst [o++] = src [b++];
            }
        }
        int *tmp = src;
        src = dst;
        dst = tmp;
    }
    if (src != order) {
        for (int i = 0; i < n; i++) {
            order [i] = src [i];
        }
    }
}
void collision_manifold_solve_order (struct physics_world *world, collision_data *manifolds, int manifold_count,
                                     int *order_out) {
    if ((!world) || (!world -> manifold_sort_keys) || (!manifolds) || (!order_out) || (manifold_count <= 0)) {
        return;
    }
    if (manifold_count > a3_max_manifolds) {
        manifold_count = a3_max_manifolds;
    }
    for (int m = 0; m < manifold_count; m++) {
        float lowest = 1000000.0f;
        for (int i = 0; i < manifolds [m].contact_count; i++) {
            float y = manifolds [m].contacts [i].position.y;
            if (y < lowest) {
                lowest = y;
            }
        }
        world -> manifold_sort_keys [m] = lowest;
        order_out [m] = m;
    }
    /* TRUTH: mergesort with thread-local scratch (no malloc, no globals).
     * Deterministic total order, race-free. */
    {
        static _Thread_local int merge_scratch [8192];
        if (manifold_count <= 8192) {
            collision_manifold_merge_sort (world -> manifold_sort_keys, order_out, merge_scratch, manifold_count);
            return;
        }
    }
    /* Tiny fallback: insertion sort (deterministic, no globals). */
    for (int i = 1; i < manifold_count; i++) {
        int key_idx = order_out [i];
        float key_val = world -> manifold_sort_keys [key_idx];
        int j = i - 1;
        while (j >= 0) {
            int cur_idx = order_out [j];
            float cur_val = world -> manifold_sort_keys [cur_idx];
            bool shift = (cur_val > key_val) || (cur_val == key_val && cur_idx > key_idx);
            if (!shift) {
                break;
            }
            order_out [j + 1] = order_out [j];
            j--;
        }
        order_out [j + 1] = key_idx;
    }
}
float collision_resolve_iterative (collision_data *m, float dt, bool friction_only, int start_index,
                                   const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if (dt <= 0.0f) {
        dt = 1.0f / 60.0f;
    }
    /* FIX-AUDIT-DESPOT cost TRUTH: callers visit every manifold TWICE per
     * iteration (see physics_world_step: resolve(...,iter) +
     * resolve(...,iter+1)), so the "64 iterations" knob is really 128
     * contact visits per manifold per tick (plus 2 friction-only relaxation
     * sweeps after Poisson). Convergent, not divergent — the second visit
     * settles coupled contacts within one sweep — but profiling must count
     * 2x the knob, and halving the knob halves 2x the work. */
    /* Returns the largest impulse magnitude applied this visit (normal +
     * friction deltas). Callers use it for local-convergence polishing. */
    float max_applied = 0.0f;
    /* AUDIT NOTE (reverted experiment): rotating the contact start index
     * per sweep was tried to symmetrize first-solver bias, but it broke
     * sliding kinetic friction badly (3x stopping distance: order cycling
     * interacts with the acc>=0 clamp, rectifying oscillation into net
     * drift). Fixed order + local double-visit (see callers) converges
     * without that interaction. start_index is accepted and ignored. */
    (void) start_index;
    for (int k = 0; k < m -> contact_count; k++) {
        int i = k;
        contact_point_data *cp = &m -> contacts [i];
        vector3 va = vector3_addition (m -> object_a -> velocity, vector3_cross (m -> object_a -> angular_velocity, cp -> ra));
        vector3 vb = vector3_addition (m -> object_b -> velocity, vector3_cross (m -> object_b -> angular_velocity, cp -> rb));
        vector3 rel_vel = vector3_subtraction (vb, va);
        float vn = vector3_dot (rel_vel, m -> normal_vector);
        /* Pure compression: no restitution bias here (Poisson pass later). */
        if (!friction_only) {
            float lambda_n = -vn * cp -> effective_mass_normal;
            float old_impulse = cp -> accumulated_normal_impulse;
            cp -> accumulated_normal_impulse = fmaxf (old_impulse + lambda_n, 0.0f);
            lambda_n = cp -> accumulated_normal_impulse - old_impulse;
            /* TRUTH: full step always. Old provenance-gated SOR (warm *=0.5)
             * halved steady-state corrections to mask cache aliasing
             * ping-pong; with strict both-side matching (above) the alias
             * source is gone, so dampening true warm starts only slows
             * convergence 2x. Fixed points unchanged either way.
             * (The old redundant re-assign acc=old+lambda after clamping is
             * deleted: lambda was already recomputed post-clamp, so the
             * re-assign was identity — dead write.) */
            if (lambda_n != 0.0f) {
                float applied_n = fabsf (lambda_n);
                if (applied_n > max_applied) {
                    max_applied = applied_n;
                }
                vector3 impulse = vector3_scaling (m -> normal_vector, lambda_n);
                if (rigidbody_effective_inv_mass (m -> object_a) > 0.0f) {
                    m -> object_a -> velocity = vector3_subtraction (
                        m -> object_a -> velocity, vector3_scaling (impulse, rigidbody_effective_inv_mass (m -> object_a)));
                    m -> object_a -> angular_velocity = vector3_subtraction (
                        m -> object_a -> angular_velocity,
                        math3_multiplication_vector3 (rigidbody_effective_inv_inertia (m -> object_a),
                                                      vector3_cross (cp -> ra, impulse)));
                }
                if (rigidbody_effective_inv_mass (m -> object_b) > 0.0f) {
                    m -> object_b -> velocity = vector3_addition (
                        m -> object_b -> velocity, vector3_scaling (impulse, rigidbody_effective_inv_mass (m -> object_b)));
                    m -> object_b -> angular_velocity =
                        vector3_addition (m -> object_b -> angular_velocity,
                                          math3_multiplication_vector3 (rigidbody_effective_inv_inertia (m -> object_b),
                                                                        vector3_cross (cp -> rb, impulse)));
                }
            }
        } /* !friction_only: normal solve skipped in relaxation so the
             Poisson bounce is never subtracted back out. */
        va = vector3_addition (m -> object_a -> velocity, vector3_cross (m -> object_a -> angular_velocity, cp -> ra));
        vb = vector3_addition (m -> object_b -> velocity, vector3_cross (m -> object_b -> angular_velocity, cp -> rb));
        rel_vel = vector3_subtraction (vb, va);
        /* Ensure a complete orthonormal tangent frame: rebuild from current
         * slip when the stored frame is missing, refresh t2 otherwise. */
        vector3 tangent = cp -> tangent_vector;
        if (vector3_length_squared (tangent) < 0.0001f) {
            vector3 rel_vel_tangent = vector3_subtraction (
                rel_vel, vector3_scaling (m -> normal_vector, vector3_dot (rel_vel, m -> normal_vector)));
            float tangent_length = vector3_length (rel_vel_tangent);
            if (tangent_length > 0.0001f) {
                tangent = vector3_scaling (rel_vel_tangent, -1.0f / tangent_length);
                cp -> tangent_vector = tangent;
                cp -> tangent2 = vector3_normalisation (vector3_cross (m -> normal_vector, tangent));
            }
        } else {
            vector3 t2_check = vector3_cross (m -> normal_vector, tangent);
            if (vector3_length_squared (t2_check) > 0.0001f) {
                cp -> tangent2 = vector3_normalisation (t2_check);
            }
        }
        if (vector3_length_squared (tangent) > 0.0001f) {
            vector3 tangent2 = cp -> tangent2;
            bool has_t2 = (vector3_length_squared (tangent2) > 0.0001f);
            float vt1 = vector3_dot (rel_vel, tangent);
            float vt2 = has_t2 ? vector3_dot (rel_vel, tangent2) : 0.0f;
            float slip_speed = sqrtf (vt1 * vt1 + vt2 * vt2);
            float eff1 = cp -> effective_mass_tangent;
            float eff2 = has_t2 ? cp -> effective_mass_tangent2 : 0.0f;
            if (eff1 <= 0.0f) {
                vector3 ra_c = vector3_cross (cp -> ra, tangent);
                vector3 rb_c = vector3_cross (cp -> rb, tangent);
                float k = rigidbody_effective_inv_mass (m -> object_a) + rigidbody_effective_inv_mass (m -> object_b) +
                          vector3_dot (
                              vector3_addition (vector3_cross (math3_multiplication_vector3 (
                                                                   rigidbody_effective_inv_inertia (m -> object_a), ra_c),
                                                               cp -> ra),
                                                vector3_cross (math3_multiplication_vector3 (
                                                                   rigidbody_effective_inv_inertia (m -> object_b), rb_c),
                                                               cp -> rb)),
                              tangent);
                eff1 = (k > 0.0f) ? (1.0f / k) : 0.0f;
                cp -> effective_mass_tangent = eff1;
            }
            if (has_t2 && (eff2 <= 0.0f)) {
                vector3 ra_c2 = vector3_cross (cp -> ra, tangent2);
                vector3 rb_c2 = vector3_cross (cp -> rb, tangent2);
                float k2 = rigidbody_effective_inv_mass (m -> object_a) + rigidbody_effective_inv_mass (m -> object_b) +
                           vector3_dot (vector3_addition (
                                            vector3_cross (math3_multiplication_vector3 (
                                                               rigidbody_effective_inv_inertia (m -> object_a), ra_c2),
                                                           cp -> ra),
                                            vector3_cross (math3_multiplication_vector3 (
                                                               rigidbody_effective_inv_inertia (m -> object_b), rb_c2),
                                                           cp -> rb)),
                                        tangent2);
                eff2 = (k2 > 0.0f) ? (1.0f / k2) : 0.0f;
                cp -> effective_mass_tangent2 = eff2;
            }
            /* Stick/slip select on combined slip speed. With a persistent
             * frame and an honest normal impulse, stick (full slip kill
             * inside the cone) genuinely holds; sliding clamps to mu_k.
             * DESPOT-2026-10-04: the selection below used to re-evaluate on
             * live per-iteration slip (solver transient). Prefer the
             * tick-start snapshot when a solve phase recorded one (see
             * collision_snapshot_friction_mu); unset (<0) keeps the legacy
             * live behaviour for direct resolve callers. */
            const float static_friction_threshold = C -> solver.static_friction_thresh; /* MPE_TASK_30 */
            float static_friction_coeff = fminf (m -> object_a -> friction_static, m -> object_b -> friction_static);
            float kinetic_friction_coeff = fminf (m -> object_a -> friction_kinetic, m -> object_b -> friction_kinetic);
            if (static_friction_coeff < kinetic_friction_coeff) {
                static_friction_coeff = kinetic_friction_coeff;
            }
            float friction_coeff;
            if (cp -> snap_friction_mu >= 0.0f) {
                friction_coeff = cp -> snap_friction_mu;
            } else {
                friction_coeff =
                    (slip_speed < static_friction_threshold) ? static_friction_coeff : kinetic_friction_coeff;
            }
            /* Coulomb cone: solve both tangents, clamp the COMBINED vector.
             * Isotropic bodies (the default, and every pre-existing body)
             * resolve mu_roll == mu_lateral, which is the legacy disc clamp
             * below, unchanged. Anisotropic bodies (mecanum rollers) get an
             * ellipse: grip along their roll axis, sliding across it. */
            float lambda_t1 = -vt1 * eff1;
            float lambda_t2 = has_t2 ? (-vt2 * eff2) : 0.0f;
            float new_acc1 = cp -> accumulated_tangent_impulse + lambda_t1;
            float new_acc2 = cp -> accumulated_tangent2_impulse + lambda_t2;
            {
                float mu_roll, mu_lateral;
                vector3 aniso_axis;
                a3_contact_friction_cone (m -> object_a, m -> object_b, friction_coeff, &mu_roll, &mu_lateral, &aniso_axis);
                /* DESPOT-FIX: axis IS the basis t1 (prepare stored the
                 * material frame in the contact tangents; projecting it back
                 * is identity). The cone's owner-frame axis sweeps with a
                 * spinning hub whenever a reference frame is set — see the
                 * warm-start site above. */
                (void) aniso_axis;
                a3_anisotropic_coulomb_clamp (m -> normal_vector, tangent, tangent2, new_acc1, new_acc2,
                                              cp -> accumulated_normal_impulse, mu_roll, mu_lateral, tangent, &new_acc1,
                                              &new_acc2);
            }
            /* TRUTH: full friction step (see normal solve: SOR deleted). */
            float step1 = new_acc1 - cp -> accumulated_tangent_impulse;
            float step2 = new_acc2 - cp -> accumulated_tangent2_impulse;
            cp -> accumulated_tangent_impulse += step1;
            cp -> accumulated_tangent2_impulse += step2;
            vector3 friction_delta = vector3_addition (vector3_scaling (tangent, step1),
                                                       has_t2 ? vector3_scaling (tangent2, step2) : vector3_zero ());
            {
                float applied_t = sqrtf (vector3_length_squared (friction_delta));
                if (applied_t > max_applied) {
                    max_applied = applied_t;
                }
            }
            if (vector3_length_squared (friction_delta) > 0.0f) {
                if (rigidbody_effective_inv_mass (m -> object_a) > 0.0f) {
                    m -> object_a -> velocity = vector3_subtraction (
                        m -> object_a -> velocity,
                        vector3_scaling (friction_delta, rigidbody_effective_inv_mass (m -> object_a)));
                    m -> object_a -> angular_velocity = vector3_subtraction (
                        m -> object_a -> angular_velocity,
                        math3_multiplication_vector3 (rigidbody_effective_inv_inertia (m -> object_a),
                                                      vector3_cross (cp -> ra, friction_delta)));
                }
                if (rigidbody_effective_inv_mass (m -> object_b) > 0.0f) {
                    m -> object_b -> velocity =
                        vector3_addition (m -> object_b -> velocity,
                                          vector3_scaling (friction_delta, rigidbody_effective_inv_mass (m -> object_b)));
                    m -> object_b -> angular_velocity =
                        vector3_addition (m -> object_b -> angular_velocity,
                                          math3_multiplication_vector3 (rigidbody_effective_inv_inertia (m -> object_b),
                                                                        vector3_cross (cp -> rb, friction_delta)));
                }
            }
        }
    }
    return max_applied;
}
void collision_refresh_impact_velocities (collision_data *manifolds, int manifold_count) {
    if ((!manifolds) || (manifold_count <= 0)) {
        return;
    }
    for (int m = 0; m < manifold_count; m++) {
        collision_data *man = &manifolds [m];
        if ((!man -> object_a) || (!man -> object_b)) {
            continue;
        }
        for (int i = 0; i < man -> contact_count; i++) {
            contact_point_data *cp = &man -> contacts [i];
            vector3 va =
                vector3_addition (man -> object_a -> velocity, vector3_cross (man -> object_a -> angular_velocity, cp -> ra));
            vector3 vb =
                vector3_addition (man -> object_b -> velocity, vector3_cross (man -> object_b -> angular_velocity, cp -> rb));
            cp -> impact_velocity = vector3_dot (vector3_subtraction (vb, va), man -> normal_vector);
        }
    }
}
void collision_snapshot_friction_mu (collision_data *manifolds, int manifold_count, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!manifolds) || (manifold_count <= 0)) {
        return;
    }
    float sth = C -> solver.static_friction_thresh;
    if (!(sth > 0.0f) || !isfinite (sth)) {
        sth = 0.02f;
    }
    for (int m = 0; m < manifold_count; m++) {
        collision_data *man = &manifolds [m];
        float mus = (man -> object_a) ? man -> object_a -> friction_static : 0.0f;
        float muk = (man -> object_a) ? man -> object_a -> friction_kinetic : 0.0f;
        if (man -> object_b) {
            if (man -> object_b -> friction_static < mus) {
                mus = man -> object_b -> friction_static;
            }
            if (man -> object_b -> friction_kinetic < muk) {
                muk = man -> object_b -> friction_kinetic;
            }
        }
        if (!(mus >= 0.0f) || !isfinite (mus)) {
            mus = 0.0f;
        }
        if (!(muk >= 0.0f) || !isfinite (muk)) {
            muk = 0.0f;
        }
        if (mus < muk) {
            mus = muk;
        }
        for (int i = 0; i < man -> contact_count; i++) {
            contact_point_data *cp = &man -> contacts [i];
            if ((!man -> object_a) || (!man -> object_b)) {
                cp -> snap_friction_mu = 0.0f;
                continue;
            }
            /* Frame-independent slip: full relative point velocity minus
             * the normal component. Identical to the sweep's
             * sqrt(vt1^2+vt2^2) once its frame exists, but valid before any
             * iteration has run (no frame needed). */
            vector3 va =
                vector3_addition (man -> object_a -> velocity, vector3_cross (man -> object_a -> angular_velocity, cp -> ra));
            vector3 vb =
                vector3_addition (man -> object_b -> velocity, vector3_cross (man -> object_b -> angular_velocity, cp -> rb));
            vector3 rel = vector3_subtraction (vb, va);
            float vn = vector3_dot (rel, man -> normal_vector);
            vector3 rel_t = vector3_subtraction (rel, vector3_scaling (man -> normal_vector, vn));
            float slip = vector3_length (rel_t);
            cp -> snap_friction_mu = (slip < sth) ? mus : muk;
        }
    }
}
void collision_apply_poisson_restitution (collision_data *manifolds, int manifold_count, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!manifolds) || (manifold_count <= 0)) {
        return;
    }
    for (int m = 0; m < manifold_count; m++) {
        collision_data *man = &manifolds [m];
        for (int i = 0; i < man -> contact_count; i++) {
            contact_point_data *cp = &man -> contacts [i];
            float e = fminf (man -> object_a -> restitution, man -> object_b -> restitution);
            if (e <= 0.0f) {
                continue;
            }
            /* Threshold is a speed (approach magnitude). Enforce negative
             * sign so a misconfigured +1.0 cannot make resting contacts
             * bounce: gate is impact_vn >= -|thresh] skip. */
            float rest_th = -fabsf (C -> solver.restitution_velocity_thresh);
            if (cp -> impact_velocity >= rest_th) {
                continue;
            }
            float compression = cp -> accumulated_normal_impulse - cp -> base_normal_impulse;
            if (compression <= 0.0f) {
                continue;
            }
            float lambda_r = e * compression;
            /* Newton bound: restitution reverses the RECORDED approach, not
             * the accumulated sum. Interleaved joint bias can re-inject
             * approach every iteration (joint pulls, contact re-stops), so
             * the accumulator exceeds true compression (measured 7x). The
             * Newtonian payment e*(-vn_impact)*m_eff is immune to that. */
            float approach = -cp -> impact_velocity;
            if (approach < 0.0f) {
                approach = 0.0f;
            }
            float newton_bound = e * approach * cp -> effective_mass_normal;
            if (lambda_r > newton_bound) {
                lambda_r = newton_bound;
            }
            /* TRUTH P0-9: NO artificial restitution cap. The Newton bound
             * above IS the physical bound (e reverses recorded approach).
             * Capping at max_restitution_bias*m_eff (default 20 m/s) deadens
             * fast bounce 7x (144 m/s e=1 pays 20). Param retained for
             * emergency NaN guard at 1e6 scale (never binds physically:
             * tightening it to ~20*m_eff would cap legitimate 144 m/s CCD
             * impacts and reintroduce the deadening — rejected with cause). */
            {
                float emergency_cap = 1.0e6f * cp -> effective_mass_normal;
                if (lambda_r > emergency_cap) {
                    lambda_r = emergency_cap;
                }
            }
            if (lambda_r <= 0.0f) {
                continue;
            }
            cp -> accumulated_normal_impulse += lambda_r;
            vector3 impulse = vector3_scaling (man -> normal_vector, lambda_r);
            if (rigidbody_effective_inv_mass (man -> object_a) > 0.0f) {
                man -> object_a -> velocity = vector3_subtraction (
                    man -> object_a -> velocity, vector3_scaling (impulse, rigidbody_effective_inv_mass (man -> object_a)));
                man -> object_a -> angular_velocity =
                    vector3_subtraction (man -> object_a -> angular_velocity,
                                         math3_multiplication_vector3 (rigidbody_effective_inv_inertia (man -> object_a),
                                                                       vector3_cross (cp -> ra, impulse)));
            }
            if (rigidbody_effective_inv_mass (man -> object_b) > 0.0f) {
                man -> object_b -> velocity = vector3_addition (
                    man -> object_b -> velocity, vector3_scaling (impulse, rigidbody_effective_inv_mass (man -> object_b)));
                man -> object_b -> angular_velocity =
                    vector3_addition (man -> object_b -> angular_velocity,
                                      math3_multiplication_vector3 (rigidbody_effective_inv_inertia (man -> object_b),
                                                                    vector3_cross (cp -> rb, impulse)));
            }
        }
    }
}
void collision_apply_rolling_resistance (collision_data *manifolds, int manifold_count, float dt,
                                         const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!manifolds) || (manifold_count <= 0) || (dt <= 0.0f)) {
        return;
    }
    float rolling_mu = C -> world.rolling_resistance_coeff;
    if (rolling_mu <= 0.0f) {
        return;
    }
    for (int m = 0; m < manifold_count; m++) {
        collision_data *man = &manifolds [m];
        /* Shared patch: split total dissipation across sides when BOTH bodies
         * are dynamic (floor/static bodies take the full single-sided rate).
         * TRUTH: fixed 0.5 was an admitted tune, not derived. Mass-weighted
         * (I-normalized) split: share_a = I_b/(I_a+I_b) ~= m_b/(m_a+m_b)
         * = inv_a/(inv_a+inv_b) for similar geometry; lighter body carries
         * the larger share, heavy slab ~0, equal masses recover 0.5 exactly.
         * Shares sum to 1.0 so total dynamic-dynamic dissipation is preserved
         * (rolling_decay band unchanged: that test is sphere-vs-static,
         * share=1.0 path). Dissipative clamps below (dw<=speed) untouched. */
        bool b_dynamic = (man -> object_b) && (!man -> object_b -> static_state) && (!man -> object_b -> is_sleeping);
        bool a_dynamic = (man -> object_a) && (!man -> object_a -> static_state) && (!man -> object_a -> is_sleeping);
        float share_a = 1.0f, share_b = 1.0f;
        if (a_dynamic && b_dynamic) {
            float inv_a = rigidbody_effective_inv_mass (man -> object_a);
            float inv_b = rigidbody_effective_inv_mass (man -> object_b);
            double sum = (double) inv_a + (double) inv_b;
            if (isfinite (sum) && sum > 1e-12) {
                share_a = (float) ((double) inv_a / sum);
                share_b = (float) ((double) inv_b / sum);
                if (!isfinite (share_a) || share_a < 0.0f)
                    share_a = 0.5f;
                if (!isfinite (share_b) || share_b < 0.0f)
                    share_b = 0.5f;
                if (share_a > 1.0f)
                    share_a = 1.0f;
                if (share_b > 1.0f)
                    share_b = 1.0f;
            } else {
                share_a = share_b = 0.5f;
            }
        }
        for (int i = 0; i < man -> contact_count; i++) {
            contact_point_data *cp = &man -> contacts [i];
            if (cp -> accumulated_normal_impulse <= 0.0f) {
                continue;
            }
            {
                float normal_force = cp -> accumulated_normal_impulse / dt;
                rigidbody *bodies [2] = {man -> object_a, man -> object_b};
                vector3 rlev [2] = {cp -> ra, cp -> rb};
                for (int bi = 0; bi < 2; bi++) {
                    rigidbody *bd = bodies [bi];
                    if ((!bd) || (bd -> static_state) || (bd -> is_sleeping) || (bd -> kinematic)) {
                        continue;
                    }
                    /* TRUTH: roll lever = |r| (contact radius, =R for spheres:
                     * M=mu*N*R, standard Coulomb rolling resistance). Spin
                     * lever = Hertz patch a=sqrt(R*pen_eff), capped 0.3R.
                     * An earlier revision used patch for roll too, which
                     * under-damped 28x (R=0.5,pen=0.5mm: R/patch~32) and
                     * failed rolling_decay (15.8m vs 4-14m). Roll and spin
                     * are different physics: roll resists translation via
                     * R, spin resists yaw via patch. Slop zero-depth gets
                     * pen_eff=0.5mm floor so resting spin still decays. */
                    float pen_raw = (cp -> penetration > 0.0f) ? cp -> penetration : 0.0f;
                    float pen_eff = (pen_raw > 0.0005f) ? pen_raw : 0.0005f;
                    float r_eff = sqrtf (vector3_length_squared (rlev [bi]));
                    if ((!isfinite (r_eff)) || (r_eff < 1e-6f)) {
                        continue;
                    }
                    float patch = sqrtf (fmaxf (r_eff * pen_eff, 0.0f));
                    float patch_cap = 0.3f * r_eff;
                    if (patch > patch_cap) {
                        patch = patch_cap;
                    }
                    /* Rolling part: oppose tangential-plane spin, lever=|r|
                     * (contact radius: M=mu*N*R, standard Coulomb rolling
                     * resistance). Applies to all shapes; boxes in face
                     * contact get tipping damping that settles stacks
                     * (verified: F10 10-stack calm, 6-cube holds). Spin
                     * below uses the Hertz patch. */
                    vector3 spin_n =
                        vector3_scaling (man -> normal_vector, vector3_dot (bd -> angular_velocity, man -> normal_vector));
                    vector3 roll_w = vector3_subtraction (bd -> angular_velocity, spin_n);
                    float roll_speed = vector3_length (roll_w);
                    if (roll_speed > 0.0001f) {
                        vector3 roll_axis = vector3_scaling (roll_w, 1.0f / roll_speed);
                        float inertia_axis =
                            1.0f / fmaxf (vector3_dot (roll_axis, math3_multiplication_vector3 (
                                                                      rigidbody_effective_inv_inertia (bd), roll_axis)),
                                          1e-9f);
                        if (!isfinite (inertia_axis) || inertia_axis <= 0.0f) {
                            continue;
                        }
                        float share = (bi == 0) ? share_a : share_b;
                        float dw = share * rolling_mu * normal_force * r_eff * dt / inertia_axis;
                        if (!isfinite (dw) || dw < 0.0f) {
                            continue;
                        }
                        if (dw > roll_speed) {
                            dw = roll_speed;
                        }
                        bd -> angular_velocity =
                            vector3_subtraction (bd -> angular_velocity, vector3_scaling (roll_axis, dw));
                    }
                    /* Spin part: same patch. */
                    float spin_speed = vector3_length (spin_n);
                    if (spin_speed > 0.0001f) {
                        vector3 spin_axis = vector3_scaling (spin_n, 1.0f / spin_speed);
                        float inertia_spin =
                            1.0f / fmaxf (vector3_dot (spin_axis, math3_multiplication_vector3 (
                                                                      rigidbody_effective_inv_inertia (bd), spin_axis)),
                                          1e-9f);
                        if (!isfinite (inertia_spin) || inertia_spin <= 0.0f) {
                            continue;
                        }
                        float share_sp = (bi == 0) ? share_a : share_b;
                        float dw_spin = share_sp * rolling_mu * normal_force * patch * dt / inertia_spin;
                        if (!isfinite (dw_spin) || dw_spin < 0.0f) {
                            continue;
                        }
                        if (dw_spin > spin_speed) {
                            dw_spin = spin_speed;
                        }
                        bd -> angular_velocity =
                            vector3_subtraction (bd -> angular_velocity, vector3_scaling (spin_axis, dw_spin));
                    }
                }
            }
        }
    }
}
void collision_apply_split_impulse (collision_data *manifolds, int manifold_count, float dt, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!manifolds) || (manifold_count <= 0) || (!(dt > 0.0f))) {
        return;
    }
    float slop = C -> solver.penetration_slop;
    float beta = C -> solver.bias_factor;
    float max_bias_vel = C -> solver.max_separation_bias;
    /* TRUTH: runtime clamps survive old config files with huge caps.
     * slop 0..5cm, beta 0..1, bias vel <=10 m/s. */
    if (!isfinite (slop) || slop < 0.0f) {
        slop = 0.01f;
    }
    if (slop > 0.05f) {
        slop = 0.05f;
    }
    if (!isfinite (beta) || beta < 0.0f) {
        beta = 0.0f;
    }
    if (beta > 1.0f) {
        beta = 1.0f;
    }
    if (!isfinite (max_bias_vel) || max_bias_vel < 0.0f) {
        max_bias_vel = 5.0f;
    }
    if (max_bias_vel > 10.0f) {
        max_bias_vel = 10.0f;
    }
    const float max_corr = max_bias_vel * dt;
    /* Shared wake depth (see the wake site below). Clamped to the same
     * [0, 0.1] the schema registers it with. */
    float wake_depth = C -> depenetration.wake_depth_thresh;
    if (!isfinite (wake_depth) || wake_depth < 0.0f) {
        wake_depth = 0.02f;
    }
    if (wake_depth > 0.1f) {
        wake_depth = 0.1f;
    }
    for (int m = 0; m < manifold_count; m++) {
        collision_data *man = &manifolds [m];
        rigidbody *body_a = man -> object_a;
        rigidbody *body_b = man -> object_b;
        if ((!body_a) || (!body_b)) {
            continue;
        }
        float inv_a = rigidbody_effective_inv_mass (body_a);
        float inv_b = rigidbody_effective_inv_mass (body_b);
        /* Sleeping bodies hold infinite mass (undisturbed rest) unless the
         * correction is significant, in which case wake first. */
        float inv_sum = inv_a + inv_b;
        if (inv_sum <= 0.0f) {
            /* Both sides locked: check whether the overlap is significant
             * enough to wake the dynamic sleepers. */
            float deepest_check = 0.0f;
            for (int i = 0; i < man -> contact_count; i++) {
                if (man -> contacts [i].penetration > deepest_check) {
                    deepest_check = man -> contacts [i].penetration;
                }
            }
            if (deepest_check > C -> depenetration.wake_depth_thresh) {
                if (!body_a -> static_state) {
                    rigidbody_wake (body_a);
                }
                if (!body_b -> static_state) {
                    rigidbody_wake (body_b);
                }
            }
            continue;
        }
        /* DESPOT-2026-10-03: THE PER-CONTACT EXPERIMENT WAS TRIED, MEASURED,
         * AND REVERTED. Kept here because the negative result is the useful
         * part and the reasoning is worth not repeating.
         *
         * The ORIGINAL code computed `deepest` -- the maximum penetration over
         * the manifold -- DISCARDED which contact it belonged to, and then
         * translated both bodies along the normal AT THE BODY CENTRE by
         * corr/(inv_a+inv_b). That is a purely translational projection, so it
         * cannot right a tilt: a box with one buried corner gets shoved
         * sideways instead of rotated flat. That is a real deficiency, and it
         * is why the per-contact form below was tried.
         *
         * Catto (GDC 2014 p.53) and Box2D/Bullet solve a per-contact
         * pseudo-impulse WITH lever arms (m v' = n*lambda, I w' = (r x n)*lambda)
         * so the r x n term supplies the angular correction. Implemented twice:
         *
         *  (a) ONE INDEPENDENT PASS per contact. Right idea, WRONG MAGNITUDE.
         *      Every contact applied its own FULL correction, so a 4-point
         *      manifold gave the body 4x the intended translation plus 4
         *      independent angular kicks at r = 0.5 m. Measured on the F10/F11
         *      10-high pile: worst pairwise overlap 0.0537 -> 0.2253 m (4.2x
         *      worse) and max|omega| 0.0000 -> 1.9217 rad/s. 0.2253 m between
         *      1.0 m cubes IS the "cubes phasing and folding into each other"
         *      symptom. This was a regression introduced by the audit, caught by
         *      measurement, and reverted.
         *
         *  (b) SHARED GAUSS-SEIDEL with an accumulated per-point impulse
         *      clamped at >= 0, which is what Box2D 2.x SolvePositionConstraints
         *      actually does -- the clamp is precisely what makes the contacts
         *      SHARE the correction instead of each taking the whole thing.
         *      Correct in principle, and it fixed (a)'s over-correction
         *      (0.2253 -> 0.0818 m, omega back to 0.0001). But across the F11
         *      torture seed sweep it was still a NET REGRESSION against the
         *      original whole-body translation:
         *
         *        seed              1      2      3      6      7      9    SUM
         *        original      0.440  0.808  0.220  0.240  0.349  0.363  2.420
         *        shared+angular 0.650  0.832  0.060  0.504  0.407  0.500  2.953
         *        shared-linear  0.766  0.642  0.832  0.728  0.648  0.417  4.033
         *
         *      Better on one seed, worse on five. In a buckling, chaotic pile
         *      the angular positional kick adds energy into the next tick's
         *      velocity solve, and the column is already near its stability
         *      boundary. The isolated tilted-box gain (0.354 -> 0.312 deg)
         *      does not pay for that.
         *
         * SO: the whole-body translation stays. The genuine, measured remedy
         * for deep overlap is elsewhere and is not a solver reformulation --
         * see docs/VALIDATION.md -> [F11-BUCKLE-INTERPENETRATION], where the
         * dominant lever is shown to be SOLVER ITERATION COUNT (0.31 m at 64,
         * 0.0024 m at 96, exactly 0 at 128, for the same scene).
         *
         * The ONE defect found here that WAS real and IS kept: the wake
         * threshold was a hard-coded 0.01 m, directly contradicting the config
         * schema's own measured note ("measured 0.01 re-admits the F10 runaway,
         * runmax 13.07 m/s ejection ... while 0.02 holds runmax 0.00"). A
         * source literal that the documentation records as a measured failure
         * mode is a bug in one of the two; it was the literal. */
        float deepest = 0.0f;
        for (int i = 0; i < man -> contact_count; i++) {
            if (man -> contacts [i].penetration > deepest) {
                deepest = man -> contacts [i].penetration;
            }
        }
        float corr = beta * fmaxf (deepest - slop, 0.0f);
        if (corr > max_corr) {
            corr = max_corr;
        }
        if (corr <= 0.0f) {
            continue;
        }
        vector3 shift = vector3_scaling (man -> normal_vector, corr / inv_sum);
        /* TRUTH: kinematic has stored inv!=0 but effective 0. Old
         * !static_state moved kinematics, corrupting prescribed motion.
         * Gate on effective inv (zero for kinematic/sleeping/static). */
        if (inv_a > 0.0f) {
            body_a -> position = vector3_subtraction (body_a -> position, vector3_scaling (shift, inv_a));
        }
        if (inv_b > 0.0f) {
            body_b -> position = vector3_addition (body_b -> position, vector3_scaling (shift, inv_b));
        }
        /* Wake depth now reads the config knob (see note above). */
        if (corr > wake_depth) {
            if (!body_a -> static_state) {
                rigidbody_wake (body_a);
            }
            if (!body_b -> static_state) {
                rigidbody_wake (body_b);
            }
        }
    }
}

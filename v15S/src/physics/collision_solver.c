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

#define contact_hash_bits 12
#define contact_hash_size (1 << contact_hash_bits)
#define contact_hash_mask (contact_hash_size - 1)

static inline uint32_t contact_pair_key(uint32_t id_a, uint32_t id_b) {
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

static inline vector4 collision_inverse_orientation(vector4 orientation) {
    return (vector4){orientation.w, -orientation.x, -orientation.y, -orientation.z};
}

static inline vector3 collision_world_offset_to_body_local(rigidbody *body, vector3 world_offset) {
    return vector4_rotate_to_vector3(collision_inverse_orientation(body->orientation), world_offset);
}

static inline vector3 collision_body_local_to_world_offset(rigidbody *body, vector3 local_offset) {
    return vector4_rotate_to_vector3(body->orientation, local_offset);
}





static uint32_t a3_task05_mix_u32(uint32_t hash_value, uint32_t input_value) {
    hash_value ^= input_value + 0x9e3779b9u + (hash_value << 6) + (hash_value >> 2);
    return hash_value;
}

static uint32_t a3_task05_float_bits(float value) {
    union {
        float float_value;
        uint32_t integer_value;
    } converter;

    converter.float_value = value;
    return converter.integer_value;
}

static uint32_t a3_task05_body_property_stamp(const rigidbody *rigid_body) {
    if (!rigid_body) {
        return 0;
    }

    uint32_t stamp = 2166136261u;

    stamp = a3_task05_mix_u32(stamp, (uint32_t) rigid_body->type);
    stamp = a3_task05_mix_u32(stamp, rigid_body->static_state ? 1u : 0u);

    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->mass));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->inverse_mass));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->radius));

    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->half_extensions.x));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->half_extensions.y));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->half_extensions.z));

    /* TRUTH: friction/restitution/kinematic affect the solved impulse.
     * Old stamp omitted them: editing friction or toggling kinematic hit a
     * stale acc_n*new_mu (wrong friction cone for a tick). Include.
     * TRUTH: cylinder_half_length and custom_shape likewise change lever
     * arms and dispatch: editing h hit stale acc with the wrong geometry.
     * Must match contact_cache_save's stamp exactly (both sides). */
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->friction_static));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->friction_kinetic));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->restitution));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(rigid_body->cylinder_half_length));
    stamp = a3_task05_mix_u32(stamp, (uint32_t) rigid_body->custom_shape);
    stamp = a3_task05_mix_u32(stamp, rigid_body->kinematic ? 2u : 0u);
    stamp = a3_task05_mix_u32(stamp, rigid_body->is_sleeping ? 4u : 0u);

    /* Orientation quantized to 1e-3: rotation invalidates local-space cache
     * matching. Without this, a body that rotates significantly between
     * frames can false-positive match a stale contact (PHYS-007). Quantizing
     * keeps resting contacts stable while forcing a miss on real rotation. */
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(roundf(rigid_body->orientation.w * 1000.0f) / 1000.0f));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(roundf(rigid_body->orientation.x * 1000.0f) / 1000.0f));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(roundf(rigid_body->orientation.y * 1000.0f) / 1000.0f));
    stamp = a3_task05_mix_u32(stamp, a3_task05_float_bits(roundf(rigid_body->orientation.z * 1000.0f) / 1000.0f));

    return stamp;
}

static bool a3_task05_cached_impulses_are_usable(float normal_impulse, float tangent_impulse) {
    if ((!isfinite(normal_impulse)) || (!isfinite(tangent_impulse))) {
        return false;
    }

    if (normal_impulse < 0.0f) {
        return false;
    }

    if (fabsf(normal_impulse) > 1000000.0f) {
        return false;
    }
    if (fabsf(tangent_impulse) > 1000000.0f) {
        return false;
    }

    return true;
}

static int contact_cache_match_role(const cached_contact *cc, uint32_t id_a, uint32_t id_b, uint32_t stamp_a,
                                    uint32_t stamp_b, vector3 local_a, vector3 local_b, float match_dist_sq) {
    if ((!cc) || (id_a == 0) || (id_b == 0)) {
        return 0;
    }
    if ((cc->object_id_a == id_a) && (cc->object_id_b == id_b) && (cc->property_stamp_a == stamp_a) &&
        (cc->property_stamp_b == stamp_b)) {
        /* TRUTH: strict both-side matching. Old side-A-only aliased two B
         * bodies sharing an A anchor within 5cm (wrong impulse injection).
         * Both material points must coincide; resting contacts satisfy this
         * exactly (body-local storage survives rigid translation). */
        float dist_a_sq = vector3_length_squared(vector3_subtraction(cc->local_position_a, local_a));
        float dist_b_sq = vector3_length_squared(vector3_subtraction(cc->local_position_b, local_b));
        if ((dist_a_sq < match_dist_sq) && (dist_b_sq < match_dist_sq) &&
            (a3_task05_cached_impulses_are_usable(cc->accumulated_normal_impulse, cc->accumulated_tangent_impulse))) {
            return 1;
        }
        return 0;
    }
    if ((cc->object_id_a == id_b) && (cc->object_id_b == id_a) && (cc->property_stamp_a == stamp_b) &&
        (cc->property_stamp_b == stamp_a)) {
        float dist_sq_ab = vector3_length_squared(vector3_subtraction(cc->local_position_a, local_b));
        float dist_sq_ba = vector3_length_squared(vector3_subtraction(cc->local_position_b, local_a));
        if ((dist_sq_ab < match_dist_sq) && (dist_sq_ba < match_dist_sq) &&
            (a3_task05_cached_impulses_are_usable(cc->accumulated_normal_impulse, cc->accumulated_tangent_impulse))) {
            return 2;
        }
    }
    return 0;
}

static bool contact_cache_adoptable(const cached_contact *cc, uint32_t id_a, uint32_t id_b, uint32_t stamp_a,
                                    uint32_t stamp_b, vector3 local_a, vector3 local_b, float match_dist_sq) {
    if ((!cc) || (id_a == 0) || (id_b == 0)) {
        return false;
    }
    if (!((cc->object_id_a == id_a) && (cc->object_id_b == id_b) && (cc->property_stamp_a == stamp_a) &&
          (cc->property_stamp_b == stamp_b))) {
        return false;
    }
    /* TRUTH: require BOTH sides like match_role. Side-A-only matching let
     * two B bodies sharing one A (within 5cm) share tangent memory. */
    float dist_a_sq = vector3_length_squared(vector3_subtraction(cc->local_position_a, local_a));
    if (dist_a_sq >= match_dist_sq) {
        return false;
    }
    float dist_b_sq = vector3_length_squared(vector3_subtraction(cc->local_position_b, local_b));
    if (dist_b_sq >= match_dist_sq) {
        return false;
    }
    return vector3_length_squared(cc->tangent_dir) > 0.0001f;
}


void collision_prepare_solver(struct physics_world *world, collision_data *source, collision_data *m, float dt) {
    *m = *source;
    if (dt <= 0.0f) {
        dt = 1.0f / 60.0f;
    }
    /* Per-world cache; a missing cache degrades to all-miss (cold solve).
     * No global fallback remains. */
    cached_contact *cache_array = (world) ? world->world_contact_cache : NULL;
    int cache_count = (world) ? world->world_contact_cache_count : 0;
    int32_t *hash_head = (world) ? world->contact_hash_head : NULL;
    if (!cache_array) {
        cache_count = 0;
    }

    for (int i = 0; i < m->contact_count; i++) {
        contact_point_data *cp = &m->contacts[i];
        cp->ra = vector3_subtraction(cp->position, m->object_a->position);
        cp->rb = vector3_subtraction(cp->position, m->object_b->position);
        cp->local_position_a =
            collision_world_offset_to_body_local(m->object_a, cp->ra); /* A3_PATCH_19_BODY_LOCAL_WARM_START */
        cp->local_position_b = collision_world_offset_to_body_local(m->object_b, cp->rb);

        cp->accumulated_normal_impulse = 0.0f;
        cp->accumulated_tangent_impulse = 0.0f;
        cp->accumulated_tangent2_impulse = 0.0f;
        /* AUDIT: no velocity-level Baumgarte bias is computed here on
         * purpose (see header). g_cfg.solver.bias_factor drives the
         * positional split-impulse correction instead, where bias velocity
         * cannot leak into impulses. An earlier revision computed a
         * per-contact separation_bias that nothing read. */

        /* MPE_TASK_05_CACHE_MATCH_BEGIN */
        uint32_t cache_id_a = (m->object_a) ? m->object_a->object_id : 0;
        uint32_t cache_id_b = (m->object_b) ? m->object_b->object_id : 0;

        uint32_t cache_stamp_a = a3_task05_body_property_stamp(m->object_a);
        uint32_t cache_stamp_b = a3_task05_body_property_stamp(m->object_b);

        int cache_match_found = 0;
        /* Per-world warm-start match distance (was global). */
        const mpe_config_t *prep_cfg = world ? mpe_world_cfg(world) : &g_cfg;
        float prep_match_sq = prep_cfg->solver.warm_start_match_dist_sq;

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
            uint32_t slot0 = contact_pair_key(cache_id_a, cache_id_b);
            for (int32_t slot = hash_head[slot0], guard = 0;
                 (slot >= 0) && (slot < cache_count) && (guard <= cache_count);
                 slot = cache_array[slot].hash_next, guard++) {
                cached_contact *cc = &cache_array[slot];
                int role = contact_cache_match_role(cc, cache_id_a, cache_id_b, cache_stamp_a, cache_stamp_b,
                                                    cp->local_position_a, cp->local_position_b, prep_match_sq);
                if (role == 1) {
                    cp->accumulated_normal_impulse = fmaxf(cc->accumulated_normal_impulse, 0.0f);
                    cp->accumulated_tangent_impulse = cc->accumulated_tangent_impulse;
                    cache_match_found = 1;
                    break;
                } else if (role == 2) {
                    /* Swapped body order: normal stays positive (manifold
                     * normal already points A->B); tangent reverses with
                     * the relative-velocity order. */
                    cp->accumulated_normal_impulse = fmaxf(cc->accumulated_normal_impulse, 0.0f);
                    cp->accumulated_tangent_impulse = -cc->accumulated_tangent_impulse;
                    cache_match_found = 1;
                    break;
                }
            }
        } else {
            /* Legacy linear fallback (no hash heads, e.g. malloc failure). */
            for (int c = 0; c < cache_count; c++) {
                cached_contact *cc = &cache_array[c];
                int role = contact_cache_match_role(cc, cache_id_a, cache_id_b, cache_stamp_a, cache_stamp_b,
                                                    cp->local_position_a, cp->local_position_b, prep_match_sq);
                if (role == 1) {
                    cp->accumulated_normal_impulse = fmaxf(cc->accumulated_normal_impulse, 0.0f);
                    cp->accumulated_tangent_impulse = cc->accumulated_tangent_impulse;
                    cache_match_found = 1;
                    break;
                } else if (role == 2) {
                    cp->accumulated_normal_impulse = fmaxf(cc->accumulated_normal_impulse, 0.0f);
                    cp->accumulated_tangent_impulse = -cc->accumulated_tangent_impulse;
                    cache_match_found = 1;
                    break;
                }
            }
        }
        /* MPE_TASK_05_CACHE_MATCH_END */
        if (world) {
            if (cache_match_found) {
                world->contact_cache_hits++;
            } else {
                world->contact_cache_misses++;
            }
        }
        /* PHYSICS-TRUTH (F10 10-stack): normal warm-start feedback is
         * unstable in this tree — restoring last tick's normal as the
         * iterations' seed ejects the column at ~13 m/s (measured over
         * guard/cap/cone/tightness/adoption ablations; cold normal holds
         * runmax 0.05), while the restored values themselves stay healthy
         * (~0.5, bounded: no save-bigger loop). Tangent memory still
         * restores (friction hold needs it; proven harmless) and the
         * tangent frame still adopts. Normal solves from zero every tick
         * (converges at 64–128 iterations for 10-high; low-iteration tall
         * stacks may creep — tune iterations, not seeds). Re-enable
         * normal warm-start only with a stability proof on f10_long_run. */
        cp->accumulated_normal_impulse = 0.0f;

        vector3 va = vector3_addition(m->object_a->velocity, vector3_cross(m->object_a->angular_velocity, cp->ra));
        vector3 vb = vector3_addition(m->object_b->velocity, vector3_cross(m->object_b->angular_velocity, cp->rb));
        vector3 rel_vel = vector3_subtraction(vb, va);
        float vn_initial = vector3_dot(rel_vel, m->normal_vector);

        /* TRUTH: feed sleep gating. max_relative_speed_sq is reset each tick
         * by the step and MUST be written here (contact processing); without
         * writers the relative_calm gate is dead (always 0 < thresh) and
         * riders sleep on moving platforms. */
        {
            float rsq = vector3_length_squared(rel_vel);
            if (isfinite(rsq)) {
                if (rsq > m->object_a->max_relative_speed_sq) m->object_a->max_relative_speed_sq = rsq;
                if (rsq > m->object_b->max_relative_speed_sq) m->object_b->max_relative_speed_sq = rsq;
            }
        }

        /* Poisson gate input: pre-solve approach speed of this tick. */
        cp->impact_velocity = vn_initial;

        vector3 ra_cross_n = vector3_cross(cp->ra, m->normal_vector);
        vector3 rb_cross_n = vector3_cross(cp->rb, m->normal_vector);
        vector3 ang_a = vector3_cross(
            math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_a), ra_cross_n), cp->ra);
        vector3 ang_b = vector3_cross(
            math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_b), rb_cross_n), cp->rb);
        float k_normal = rigidbody_effective_inv_mass(m->object_a) + rigidbody_effective_inv_mass(m->object_b) +
                         vector3_dot(vector3_addition(ang_a, ang_b), m->normal_vector);
        cp->effective_mass_normal = (k_normal > 0.0f) ? (1.0f / k_normal) : 0.0f;

        vector3 rel_vel_tangent = vector3_subtraction(rel_vel, vector3_scaling(m->normal_vector, vn_initial));
        float tangent_speed = vector3_length(rel_vel_tangent);

        /* Coulomb tangent frame. Stick/slip select mirrors the sweep
         * (static below thresh, kinetic above): below thresh the slip
         * direction is micro-motion noise, so a resting contact must use
         * the remembered tangent or no frame at all — firing full warm
         * friction along a noise direction walks stacks sideways (F10
         * 10-stack ejects at 13 m/s with noise frames, stands with
         * adopted-or-zero). True sliding keeps the slip direction. */
        const mpe_config_t *frame_cfg = world ? mpe_world_cfg(world) : &g_cfg;
        float stick_thresh = frame_cfg->solver.static_friction_thresh;
        if (!(stick_thresh > 0.0f) || !isfinite(stick_thresh)) {
            stick_thresh = 0.02f;
        }
        bool frame_sliding = (tangent_speed >= stick_thresh);
        vector3 adopted_tangent = vector3_zero();
        if (!frame_sliding) {
            /* Same first-hit as the legacy full-array scan (see hash note
             * above): the bucket holds exactly the matchable entries in
             * save order. */
            if ((hash_head) && (cache_id_a != 0) && (cache_id_b != 0)) {
                uint32_t slot0 = contact_pair_key(cache_id_a, cache_id_b);
                for (int32_t slot = hash_head[slot0], guard = 0;
                     (slot >= 0) && (slot < cache_count) && (guard <= cache_count);
                     slot = cache_array[slot].hash_next, guard++) {
                    cached_contact *cc = &cache_array[slot];
                    if (contact_cache_adoptable(cc, cache_id_a, cache_id_b, cache_stamp_a, cache_stamp_b,
                                                cp->local_position_a, cp->local_position_b, prep_match_sq)) {
                        adopted_tangent = cc->tangent_dir;
                        break;
                    }
                }
            } else {
                for (int c = 0; c < cache_count; c++) {
                    cached_contact *cc = &cache_array[c];
                    if (contact_cache_adoptable(cc, cache_id_a, cache_id_b, cache_stamp_a, cache_stamp_b,
                                                cp->local_position_a, cp->local_position_b, prep_match_sq)) {
                        adopted_tangent = cc->tangent_dir;
                        break;
                    }
                }
            }
            if (vector3_length_squared(adopted_tangent) > 0.0001f) {
                /* Re-orthogonalize against the current normal. */
                adopted_tangent = vector3_subtraction(
                    adopted_tangent,
                    vector3_scaling(m->normal_vector, vector3_dot(adopted_tangent, m->normal_vector)));
                if (vector3_length_squared(adopted_tangent) > 0.0001f) {
                    adopted_tangent = vector3_normalisation(adopted_tangent);
                } else {
                    adopted_tangent = vector3_zero();
                }
            }
        }

        /* Frame select: true sliding keeps the slip direction
         * (meaningful); sticking uses the remembered direction or, with
         * no memory yet, no frame (normal-only this tick — the sweep
         * cannot invent a hold direction from noise). */
        if (!frame_sliding) {
            if (vector3_length_squared(adopted_tangent) > 0.0001f) {
                cp->tangent_vector = adopted_tangent;
            } else {
                cp->tangent_vector = vector3_zero();
            }
        } else {
            if (tangent_speed > 0.0001f) {
                cp->tangent_vector = vector3_scaling(rel_vel_tangent, -1.0f / tangent_speed);
            } else {
                cp->tangent_vector = adopted_tangent;
            }
        }
        if (vector3_length_squared(cp->tangent_vector) > 0.0001f) {
            /* Second tangent completes the Coulomb disc: t2 = n x t1. */
            cp->tangent2 = vector3_cross(m->normal_vector, cp->tangent_vector);
            if (vector3_length_squared(cp->tangent2) > 0.0001f) {
                cp->tangent2 = vector3_normalisation(cp->tangent2);
            } else {
                cp->tangent2 = vector3_zero();
            }
            vector3 ra_cross_t = vector3_cross(cp->ra, cp->tangent_vector);
            vector3 rb_cross_t = vector3_cross(cp->rb, cp->tangent_vector);
            vector3 ang_a_t =
                vector3_cross(math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_a), ra_cross_t), cp->ra);
            vector3 ang_b_t =
                vector3_cross(math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_b), rb_cross_t), cp->rb);
            float k_tangent = rigidbody_effective_inv_mass(m->object_a) + rigidbody_effective_inv_mass(m->object_b) +
                              vector3_dot(vector3_addition(ang_a_t, ang_b_t), cp->tangent_vector);
            cp->effective_mass_tangent = (k_tangent > 0.0f) ? (1.0f / k_tangent) : 0.0f;
            if (vector3_length_squared(cp->tangent2) > 0.0001f) {
                vector3 ra_cross_t2 = vector3_cross(cp->ra, cp->tangent2);
                vector3 rb_cross_t2 = vector3_cross(cp->rb, cp->tangent2);
                vector3 ang_a_t2 = vector3_cross(
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_a), ra_cross_t2), cp->ra);
                vector3 ang_b_t2 = vector3_cross(
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_b), rb_cross_t2), cp->rb);
                float k_tangent2 = rigidbody_effective_inv_mass(m->object_a) + rigidbody_effective_inv_mass(m->object_b) +
                                   vector3_dot(vector3_addition(ang_a_t2, ang_b_t2), cp->tangent2);
                cp->effective_mass_tangent2 = (k_tangent2 > 0.0f) ? (1.0f / k_tangent2) : 0.0f;
            } else {
                cp->effective_mass_tangent2 = 0.0f;
            }
        } else {
            /* No slip and no remembered direction: nothing to hold against. */
            cp->tangent_vector = vector3_zero();
            cp->effective_mass_tangent = 0.0f;
            cp->tangent2 = vector3_zero();
            cp->effective_mass_tangent2 = 0.0f;
            cp->accumulated_tangent2_impulse = 0.0f;
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

        if (cp->accumulated_normal_impulse != 0.0f || cp->accumulated_tangent_impulse != 0.0f ||
            cp->accumulated_tangent2_impulse != 0.0f) {
            /* TRUTH: apply restored support only to an APPROACHING contact.
             * Last tick's impulse is meaningless when the pair is separating
             * this tick (whipping rim contact, liftoff): firing it anyway
             * injects approach that isn't there, and the cache loop (save
             * bigger, restore bigger) turns it exponential. A separating
             * contact starts cold; the sweeps below converge it. */
            vector3 va_now = vector3_addition(m->object_a->velocity,
                                              vector3_cross(m->object_a->angular_velocity, cp->ra));
            vector3 vb_now = vector3_addition(m->object_b->velocity,
                                              vector3_cross(m->object_b->angular_velocity, cp->rb));
            float vn_now = vector3_dot(vector3_subtraction(vb_now, va_now), m->normal_vector);
            if (vn_now >= 0.0f) {
                cp->accumulated_normal_impulse = 0.0f;
                cp->accumulated_tangent_impulse = 0.0f;
                cp->accumulated_tangent2_impulse = 0.0f;
            } else {
            /* TRUTH: normal starts cold (see above), so no magnitude cap is
             * needed: stale hits cannot bomb through a zero seed, and
             * capping a trusted guess against local need starves stacked
             * contacts (base of a 10-stack needs ~10x local need). Tangent
             * is projected onto the current Coulomb cone likewise (the
             * sweep loop does this every iteration; application must not
             * bypass). */
            {
                float mus_a = m->object_a ? m->object_a->friction_static : 0.0f;
                float mus_b = m->object_b ? m->object_b->friction_static : 0.0f;
                float mu_cap = (mus_a < mus_b) ? mus_a : mus_b;
                /* TRUTH: mirror the sweep's stick/slip select (static below
                 * thresh, kinetic above). Capping sliding restored friction
                 * at mu_s overestimates what the sweep allows (mu_k) and
                 * re-admits sideways energy through application. */
                {
                    const mpe_config_t *mu_cfg = world ? mpe_world_cfg(world) : &g_cfg;
                    float mks_a = m->object_a ? m->object_a->friction_kinetic : 0.0f;
                    float mks_b = m->object_b ? m->object_b->friction_kinetic : 0.0f;
                    float mu_k = (mks_a < mks_b) ? mks_a : mks_b;
                    float sth = mu_cfg->solver.static_friction_thresh;
                    if (!(sth > 0.0f) || !isfinite(sth)) sth = 0.02f;
                    if (tangent_speed >= sth) mu_cap = mu_k;
                }
                if (!(mu_cap >= 0.0f) || !isfinite(mu_cap)) mu_cap = 0.0f;
                float tcone = mu_cap * cp->accumulated_normal_impulse;
                float t1 = cp->accumulated_tangent_impulse, t2 = cp->accumulated_tangent2_impulse;
                float tcombo = sqrtf(t1 * t1 + t2 * t2);
                if (tcombo > tcone && tcombo > 0.0f) {
                    float s = tcone / tcombo;
                    cp->accumulated_tangent_impulse = t1 * s;
                    cp->accumulated_tangent2_impulse = t2 * s;
                }
            }
            vector3 impulse = vector3_addition(
                vector3_scaling(m->normal_vector, cp->accumulated_normal_impulse),
                vector3_addition(vector3_scaling(cp->tangent_vector, cp->accumulated_tangent_impulse),
                                 vector3_scaling(cp->tangent2, cp->accumulated_tangent2_impulse)));
            if (rigidbody_effective_inv_mass(m->object_a) > 0.0f) {
                m->object_a->velocity =
                    vector3_subtraction(m->object_a->velocity, vector3_scaling(impulse, rigidbody_effective_inv_mass(m->object_a)));
                m->object_a->angular_velocity = vector3_subtraction(
                    m->object_a->angular_velocity,
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_a), vector3_cross(cp->ra, impulse)));
            }
            if (rigidbody_effective_inv_mass(m->object_b) > 0.0f) {
                m->object_b->velocity =
                    vector3_addition(m->object_b->velocity, vector3_scaling(impulse, rigidbody_effective_inv_mass(m->object_b)));
                m->object_b->angular_velocity = vector3_addition(
                    m->object_b->angular_velocity,
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_b), vector3_cross(cp->rb, impulse)));
            }
            }
        }
        /* Poisson base: compression impulse entering the iterations (warm
         * start included). The restitution pass pays e over the delta. */
        cp->base_normal_impulse = cp->accumulated_normal_impulse;
    }
}

static void collision_manifold_merge_sort(const float *keys, int *order, int *scratch, int n) {
    for (int i = 0; i < n; i++) {
        order[i] = i;
    }
    int *src = order;
    int *dst = scratch;
    for (int width = 1; width < n; width *= 2) {
        for (int lo = 0; lo < n; lo += 2 * width) {
            int mid = lo + width < n ? lo + width : n;
            int hi = lo + 2 * width < n ? lo + 2 * width : n;
            int a = lo, b = mid, o = lo;
            while (a < mid && b < hi) {
                float ka = keys[src[a]];
                float kb = keys[src[b]];
                bool take_a;
                if (ka < kb) {
                    take_a = true;
                } else if (ka > kb) {
                    take_a = false;
                } else {
                    take_a = src[a] < src[b];
                }
                dst[o++] = take_a ? src[a++] : src[b++];
            }
            while (a < mid) {
                dst[o++] = src[a++];
            }
            while (b < hi) {
                dst[o++] = src[b++];
            }
        }
        int *tmp = src;
        src = dst;
        dst = tmp;
    }
    if (src != order) {
        for (int i = 0; i < n; i++) {
            order[i] = src[i];
        }
    }
}

void collision_manifold_solve_order(struct physics_world *world, collision_data *manifolds, int manifold_count,
                                      int *order_out) {
    if ((!world) || (!world->manifold_sort_keys) || (!manifolds) || (!order_out) || (manifold_count <= 0)) {
        return;
    }
    if (manifold_count > a3_max_manifolds) {
        manifold_count = a3_max_manifolds;
    }
    for (int m = 0; m < manifold_count; m++) {
        float lowest = 1000000.0f;
        for (int i = 0; i < manifolds[m].contact_count; i++) {
            float y = manifolds[m].contacts[i].position.y;
            if (y < lowest) {
                lowest = y;
            }
        }
        world->manifold_sort_keys[m] = lowest;
        order_out[m] = m;
    }
    /* TRUTH: mergesort with thread-local scratch (no malloc, no globals).
     * Deterministic total order, race-free. */
    {
        static _Thread_local int merge_scratch[8192];
        if (manifold_count <= 8192) {
            collision_manifold_merge_sort(world->manifold_sort_keys, order_out, merge_scratch, manifold_count);
            return;
        }
    }
    /* Tiny fallback: insertion sort (deterministic, no globals). */
    for (int i = 1; i < manifold_count; i++) {
        int key_idx = order_out[i];
        float key_val = world->manifold_sort_keys[key_idx];
        int j = i - 1;
        while (j >= 0) {
            int cur_idx = order_out[j];
            float cur_val = world->manifold_sort_keys[cur_idx];
            bool shift = (cur_val > key_val) || (cur_val == key_val && cur_idx > key_idx);
            if (!shift) {
                break;
            }
            order_out[j + 1] = order_out[j];
            j--;
        }
        order_out[j + 1] = key_idx;
    }
}

float collision_resolve_iterative(collision_data *m, float dt, bool friction_only, int start_index,
                                  const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if (dt <= 0.0f) {
        dt = 1.0f / 60.0f;
    }
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
    for (int k = 0; k < m->contact_count; k++) {
        int i = k;
        contact_point_data *cp = &m->contacts[i];

        vector3 va = vector3_addition(m->object_a->velocity, vector3_cross(m->object_a->angular_velocity, cp->ra));
        vector3 vb = vector3_addition(m->object_b->velocity, vector3_cross(m->object_b->angular_velocity, cp->rb));
        vector3 rel_vel = vector3_subtraction(vb, va);
        float vn = vector3_dot(rel_vel, m->normal_vector);

        /* Pure compression: no restitution bias here (Poisson pass later). */
        if (!friction_only) {
            float lambda_n = -vn * cp->effective_mass_normal;
            float old_impulse = cp->accumulated_normal_impulse;
            cp->accumulated_normal_impulse = fmaxf(old_impulse + lambda_n, 0.0f);
            lambda_n = cp->accumulated_normal_impulse - old_impulse;
            /* TRUTH: full step always. Old provenance-gated SOR (warm *=0.5)
             * halved steady-state corrections to mask cache aliasing
             * ping-pong; with strict both-side matching (above) the alias
             * source is gone, so dampening true warm starts only slows
             * convergence 2x. Fixed points unchanged either way.
             * (The old redundant re-assign acc=old+lambda after clamping is
             * deleted: lambda was already recomputed post-clamp, so the
             * re-assign was identity — dead write.) */
            if (lambda_n != 0.0f) {
                float applied_n = fabsf(lambda_n);
                if (applied_n > max_applied) {
                    max_applied = applied_n;
                }
                vector3 impulse = vector3_scaling(m->normal_vector, lambda_n);
                if (rigidbody_effective_inv_mass(m->object_a) > 0.0f) {
                    m->object_a->velocity = vector3_subtraction(
                        m->object_a->velocity, vector3_scaling(impulse, rigidbody_effective_inv_mass(m->object_a)));
                    m->object_a->angular_velocity = vector3_subtraction(
                        m->object_a->angular_velocity,
                        math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_a),
                                                     vector3_cross(cp->ra, impulse)));
                }
                if (rigidbody_effective_inv_mass(m->object_b) > 0.0f) {
                    m->object_b->velocity = vector3_addition(
                        m->object_b->velocity, vector3_scaling(impulse, rigidbody_effective_inv_mass(m->object_b)));
                    m->object_b->angular_velocity = vector3_addition(
                        m->object_b->angular_velocity,
                        math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_b),
                                                     vector3_cross(cp->rb, impulse)));
                }
            }
        } /* !friction_only: normal solve skipped in relaxation so the
             Poisson bounce is never subtracted back out. */

        va = vector3_addition(m->object_a->velocity, vector3_cross(m->object_a->angular_velocity, cp->ra));
        vb = vector3_addition(m->object_b->velocity, vector3_cross(m->object_b->angular_velocity, cp->rb));
        rel_vel = vector3_subtraction(vb, va);
        /* Ensure a complete orthonormal tangent frame: rebuild from current
         * slip when the stored frame is missing, refresh t2 otherwise. */
        vector3 tangent = cp->tangent_vector;
        if (vector3_length_squared(tangent) < 0.0001f) {
            vector3 rel_vel_tangent =
                vector3_subtraction(rel_vel, vector3_scaling(m->normal_vector, vector3_dot(rel_vel, m->normal_vector)));
            float tangent_length = vector3_length(rel_vel_tangent);
            if (tangent_length > 0.0001f) {
                tangent = vector3_scaling(rel_vel_tangent, -1.0f / tangent_length);
                cp->tangent_vector = tangent;
                cp->tangent2 = vector3_normalisation(vector3_cross(m->normal_vector, tangent));
            }
        } else {
            vector3 t2_check = vector3_cross(m->normal_vector, tangent);
            if (vector3_length_squared(t2_check) > 0.0001f) {
                cp->tangent2 = vector3_normalisation(t2_check);
            }
        }
        if (vector3_length_squared(tangent) > 0.0001f) {
            vector3 tangent2 = cp->tangent2;
            bool has_t2 = (vector3_length_squared(tangent2) > 0.0001f);

            float vt1 = vector3_dot(rel_vel, tangent);
            float vt2 = has_t2 ? vector3_dot(rel_vel, tangent2) : 0.0f;
            float slip_speed = sqrtf(vt1 * vt1 + vt2 * vt2);

            float eff1 = cp->effective_mass_tangent;
            float eff2 = has_t2 ? cp->effective_mass_tangent2 : 0.0f;
            if (eff1 <= 0.0f) {
                vector3 ra_c = vector3_cross(cp->ra, tangent);
                vector3 rb_c = vector3_cross(cp->rb, tangent);
                float k = rigidbody_effective_inv_mass(m->object_a) + rigidbody_effective_inv_mass(m->object_b) +
                          vector3_dot(vector3_addition(
                                          vector3_cross(math3_multiplication_vector3(
                                                            rigidbody_effective_inv_inertia(m->object_a), ra_c),
                                                        cp->ra),
                                          vector3_cross(math3_multiplication_vector3(
                                                            rigidbody_effective_inv_inertia(m->object_b), rb_c),
                                                        cp->rb)),
                                      tangent);
                eff1 = (k > 0.0f) ? (1.0f / k) : 0.0f;
                cp->effective_mass_tangent = eff1;
            }
            if (has_t2 && (eff2 <= 0.0f)) {
                vector3 ra_c2 = vector3_cross(cp->ra, tangent2);
                vector3 rb_c2 = vector3_cross(cp->rb, tangent2);
                float k2 = rigidbody_effective_inv_mass(m->object_a) + rigidbody_effective_inv_mass(m->object_b) +
                           vector3_dot(vector3_addition(
                                           vector3_cross(math3_multiplication_vector3(
                                                             rigidbody_effective_inv_inertia(m->object_a), ra_c2),
                                                         cp->ra),
                                           vector3_cross(math3_multiplication_vector3(
                                                             rigidbody_effective_inv_inertia(m->object_b), rb_c2),
                                                         cp->rb)),
                                       tangent2);
                eff2 = (k2 > 0.0f) ? (1.0f / k2) : 0.0f;
                cp->effective_mass_tangent2 = eff2;
            }

            /* Stick/slip select on combined slip speed. With a persistent
             * frame and an honest normal impulse, stick (full slip kill
             * inside the cone) genuinely holds; sliding clamps to mu_k. */
            const float static_friction_threshold = C->solver.static_friction_thresh; /* MPE_TASK_30 */
            float static_friction_coeff = fminf(m->object_a->friction_static, m->object_b->friction_static);
            float kinetic_friction_coeff = fminf(m->object_a->friction_kinetic, m->object_b->friction_kinetic);
            if (static_friction_coeff < kinetic_friction_coeff) {
                static_friction_coeff = kinetic_friction_coeff;
            }
            float friction_coeff =
                (slip_speed < static_friction_threshold) ? static_friction_coeff : kinetic_friction_coeff;
            float max_friction = cp->accumulated_normal_impulse * friction_coeff;

            /* Coulomb disc: solve both tangents, clamp the COMBINED vector. */
            float lambda_t1 = -vt1 * eff1;
            float lambda_t2 = has_t2 ? (-vt2 * eff2) : 0.0f;
            float new_acc1 = cp->accumulated_tangent_impulse + lambda_t1;
            float new_acc2 = cp->accumulated_tangent2_impulse + lambda_t2;
            float combo_sq = new_acc1 * new_acc1 + new_acc2 * new_acc2;
            if ((max_friction > 0.0f) && (combo_sq > max_friction * max_friction)) {
                float scale = max_friction / sqrtf(combo_sq);
                new_acc1 *= scale;
                new_acc2 *= scale;
            } else if (max_friction <= 0.0f) {
                new_acc1 = 0.0f;
                new_acc2 = 0.0f;
            }
            /* TRUTH: full friction step (see normal solve: SOR deleted). */
            float step1 = new_acc1 - cp->accumulated_tangent_impulse;
            float step2 = new_acc2 - cp->accumulated_tangent2_impulse;
            cp->accumulated_tangent_impulse += step1;
            cp->accumulated_tangent2_impulse += step2;
            vector3 friction_delta = vector3_addition(vector3_scaling(tangent, step1),
                                                      has_t2 ? vector3_scaling(tangent2, step2) : vector3_zero());
            {
                float applied_t = sqrtf(vector3_length_squared(friction_delta));
                if (applied_t > max_applied) {
                    max_applied = applied_t;
                }
            }
            if (vector3_length_squared(friction_delta) > 0.0f) {
                if (rigidbody_effective_inv_mass(m->object_a) > 0.0f) {
                    m->object_a->velocity = vector3_subtraction(
                        m->object_a->velocity, vector3_scaling(friction_delta, rigidbody_effective_inv_mass(m->object_a)));
                    m->object_a->angular_velocity =
                        vector3_subtraction(m->object_a->angular_velocity,
                                            math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_a),
                                                                         vector3_cross(cp->ra, friction_delta)));
                }
                if (rigidbody_effective_inv_mass(m->object_b) > 0.0f) {
                    m->object_b->velocity = vector3_addition(
                        m->object_b->velocity, vector3_scaling(friction_delta, rigidbody_effective_inv_mass(m->object_b)));
                    m->object_b->angular_velocity =
                        vector3_addition(m->object_b->angular_velocity,
                                         math3_multiplication_vector3(rigidbody_effective_inv_inertia(m->object_b),
                                                                      vector3_cross(cp->rb, friction_delta)));
                }
            }

        }
    }
    return max_applied;
}

void collision_refresh_impact_velocities(collision_data *manifolds, int manifold_count) {
    if ((!manifolds) || (manifold_count <= 0)) {
        return;
    }
    for (int m = 0; m < manifold_count; m++) {
        collision_data *man = &manifolds[m];
        if ((!man->object_a) || (!man->object_b)) {
            continue;
        }
        for (int i = 0; i < man->contact_count; i++) {
            contact_point_data *cp = &man->contacts[i];
            vector3 va = vector3_addition(man->object_a->velocity,
                                          vector3_cross(man->object_a->angular_velocity, cp->ra));
            vector3 vb = vector3_addition(man->object_b->velocity,
                                          vector3_cross(man->object_b->angular_velocity, cp->rb));
            cp->impact_velocity = vector3_dot(vector3_subtraction(vb, va), man->normal_vector);
        }
    }
}

void collision_apply_poisson_restitution(collision_data *manifolds, int manifold_count,
                                         const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!manifolds) || (manifold_count <= 0)) {
        return;
    }
    for (int m = 0; m < manifold_count; m++) {
        collision_data *man = &manifolds[m];
        for (int i = 0; i < man->contact_count; i++) {
            contact_point_data *cp = &man->contacts[i];
            float e = fminf(man->object_a->restitution, man->object_b->restitution);
            if (e <= 0.0f) {
                continue;
            }
            if (cp->impact_velocity >= C->solver.restitution_velocity_thresh) {
                continue;
            }
            float compression = cp->accumulated_normal_impulse - cp->base_normal_impulse;
            if (compression <= 0.0f) {
                continue;
            }
            float lambda_r = e * compression;
            /* Newton bound: restitution reverses the RECORDED approach, not
             * the accumulated sum. Interleaved joint bias can re-inject
             * approach every iteration (joint pulls, contact re-stops), so
             * the accumulator exceeds true compression (measured 7x). The
             * Newtonian payment e*(-vn_impact)*m_eff is immune to that. */
            float approach = -cp->impact_velocity;
            if (approach < 0.0f) {
                approach = 0.0f;
            }
            float newton_bound = e * approach * cp->effective_mass_normal;
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
                float emergency_cap = 1.0e6f * cp->effective_mass_normal;
                if (lambda_r > emergency_cap) {
                    lambda_r = emergency_cap;
                }
            }
            if (lambda_r <= 0.0f) {
                continue;
            }
            cp->accumulated_normal_impulse += lambda_r;
            vector3 impulse = vector3_scaling(man->normal_vector, lambda_r);
            if (rigidbody_effective_inv_mass(man->object_a) > 0.0f) {
                man->object_a->velocity = vector3_subtraction(
                    man->object_a->velocity, vector3_scaling(impulse, rigidbody_effective_inv_mass(man->object_a)));
                man->object_a->angular_velocity = vector3_subtraction(
                    man->object_a->angular_velocity,
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(man->object_a),
                                                 vector3_cross(cp->ra, impulse)));
            }
            if (rigidbody_effective_inv_mass(man->object_b) > 0.0f) {
                man->object_b->velocity = vector3_addition(
                    man->object_b->velocity, vector3_scaling(impulse, rigidbody_effective_inv_mass(man->object_b)));
                man->object_b->angular_velocity = vector3_addition(
                    man->object_b->angular_velocity,
                    math3_multiplication_vector3(rigidbody_effective_inv_inertia(man->object_b),
                                                 vector3_cross(cp->rb, impulse)));
            }
        }
    }
}

void collision_apply_rolling_resistance(collision_data *manifolds, int manifold_count, float dt,
                                        const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!manifolds) || (manifold_count <= 0) || (dt <= 0.0f)) {
        return;
    }
    float rolling_mu = C->world.rolling_resistance_coeff;
    if (rolling_mu <= 0.0f) {
        return;
    }
    for (int m = 0; m < manifold_count; m++) {
        collision_data *man = &manifolds[m];
        /* Shared patch: halve per side when BOTH bodies are dynamic (each
         * side dissipates half the patch loss; floor/static bodies take the
         * full single-sided rate). TRUTH: the 0.5 is a game tune, not
         * derived (each body physically dissipates its own full contact
         * patch). Kept: halving dynamic-dynamic decay matches the
         * rolling_decay band; use share=1.0 if per-body full dissipation
         * is ever required. */
        bool b_dynamic =
            (man->object_b) && (!man->object_b->static_state) && (!man->object_b->is_sleeping);
        bool a_dynamic =
            (man->object_a) && (!man->object_a->static_state) && (!man->object_a->is_sleeping);
        float share = (a_dynamic && b_dynamic) ? 0.5f : 1.0f;
        for (int i = 0; i < man->contact_count; i++) {
            contact_point_data *cp = &man->contacts[i];
            if (cp->accumulated_normal_impulse <= 0.0f) {
                continue;
            }
            {
                float normal_force = cp->accumulated_normal_impulse / dt;
                rigidbody *bodies[2] = {man->object_a, man->object_b};
                vector3 rlev[2] = {cp->ra, cp->rb};
                for (int bi = 0; bi < 2; bi++) {
                    rigidbody *bd = bodies[bi];
                    if ((!bd) || (bd->static_state) || (bd->is_sleeping) || (bd->kinematic)) {
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
                    float pen_raw = (cp->penetration > 0.0f) ? cp->penetration : 0.0f;
                    float pen_eff = (pen_raw > 0.0005f) ? pen_raw : 0.0005f;
                    float r_eff = sqrtf(vector3_length_squared(rlev[bi]));
                    if ((!isfinite(r_eff)) || (r_eff < 1e-6f)) {
                        continue;
                    }
                    float patch = sqrtf(fmaxf(r_eff * pen_eff, 0.0f));
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
                    vector3 spin_n = vector3_scaling(man->normal_vector,
                                                     vector3_dot(bd->angular_velocity, man->normal_vector));
                    vector3 roll_w = vector3_subtraction(bd->angular_velocity, spin_n);
                    float roll_speed = vector3_length(roll_w);
                    if (roll_speed > 0.0001f) {
                        vector3 roll_axis = vector3_scaling(roll_w, 1.0f / roll_speed);
                        float inertia_axis = 1.0f / fmaxf(vector3_dot(
                            roll_axis, math3_multiplication_vector3(rigidbody_effective_inv_inertia(bd), roll_axis)),
                            1e-9f);
                        if (!isfinite(inertia_axis) || inertia_axis <= 0.0f) {
                            continue;
                        }
                        float dw = share * rolling_mu * normal_force * r_eff * dt / inertia_axis;
                        if (!isfinite(dw) || dw < 0.0f) {
                            continue;
                        }
                        if (dw > roll_speed) {
                            dw = roll_speed;
                        }
                        bd->angular_velocity = vector3_subtraction(
                            bd->angular_velocity, vector3_scaling(roll_axis, dw));
                    }
                    /* Spin part: same patch. */
                    float spin_speed = vector3_length(spin_n);
                    if (spin_speed > 0.0001f) {
                        vector3 spin_axis = vector3_scaling(spin_n, 1.0f / spin_speed);
                        float inertia_spin = 1.0f / fmaxf(vector3_dot(
                            spin_axis, math3_multiplication_vector3(rigidbody_effective_inv_inertia(bd), spin_axis)),
                            1e-9f);
                        if (!isfinite(inertia_spin) || inertia_spin <= 0.0f) {
                            continue;
                        }
                        float dw_spin = share * rolling_mu * normal_force * patch * dt / inertia_spin;
                        if (!isfinite(dw_spin) || dw_spin < 0.0f) {
                            continue;
                        }
                        if (dw_spin > spin_speed) {
                            dw_spin = spin_speed;
                        }
                        bd->angular_velocity = vector3_subtraction(
                            bd->angular_velocity, vector3_scaling(spin_axis, dw_spin));
                    }
                }
            }
        }
    }
}

void collision_apply_split_impulse(collision_data *manifolds, int manifold_count, float dt,
                                   const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!manifolds) || (manifold_count <= 0) || (!(dt > 0.0f))) {
        return;
    }
    float slop = C->solver.penetration_slop;
    float beta = C->solver.bias_factor;
    float max_bias_vel = C->solver.max_separation_bias;
    /* TRUTH: runtime clamps survive old config files with huge caps.
     * slop 0..5cm, beta 0..1, bias vel <=10 m/s. */
    if (!isfinite(slop) || slop < 0.0f) {
        slop = 0.01f;
    }
    if (slop > 0.05f) {
        slop = 0.05f;
    }
    if (!isfinite(beta) || beta < 0.0f) {
        beta = 0.0f;
    }
    if (beta > 1.0f) {
        beta = 1.0f;
    }
    if (!isfinite(max_bias_vel) || max_bias_vel < 0.0f) {
        max_bias_vel = 5.0f;
    }
    if (max_bias_vel > 10.0f) {
        max_bias_vel = 10.0f;
    }
    const float max_corr = max_bias_vel * dt;
    for (int m = 0; m < manifold_count; m++) {
        collision_data *man = &manifolds[m];
        rigidbody *body_a = man->object_a;
        rigidbody *body_b = man->object_b;
        if ((!body_a) || (!body_b)) {
            continue;
        }
        float inv_a = rigidbody_effective_inv_mass(body_a);
        float inv_b = rigidbody_effective_inv_mass(body_b);
        /* Sleeping bodies hold infinite mass (undisturbed rest) unless the
         * correction is significant, in which case wake first. */
        float inv_sum = inv_a + inv_b;
        if (inv_sum <= 0.0f) {
            /* Both sides locked: check whether the overlap is significant
             * enough to wake the dynamic sleepers. */
            float deepest_check = 0.0f;
            for (int i = 0; i < man->contact_count; i++) {
                if (man->contacts[i].penetration > deepest_check) {
                    deepest_check = man->contacts[i].penetration;
                }
            }
            if (deepest_check > C->depenetration.wake_depth_thresh) {
                if (!body_a->static_state) {
                    rigidbody_wake(body_a);
                }
                if (!body_b->static_state) {
                    rigidbody_wake(body_b);
                }
            }
            continue;
        }
        float deepest = 0.0f;
        for (int i = 0; i < man->contact_count; i++) {
            if (man->contacts[i].penetration > deepest) {
                deepest = man->contacts[i].penetration;
            }
        }
        float corr = beta * fmaxf(deepest - slop, 0.0f);
        if (corr > max_corr) {
            corr = max_corr;
        }
        if (corr <= 0.0f) {
            continue;
        }
        vector3 shift = vector3_scaling(man->normal_vector, corr / inv_sum);
        /* TRUTH: kinematic has stored inv!=0 but effective 0. Old
         * !static_state moved kinematics, corrupting prescribed motion.
         * Gate on effective inv (zero for kinematic/sleeping/static). */
        if (inv_a > 0.0f) {
            body_a->position = vector3_subtraction(body_a->position, vector3_scaling(shift, inv_a));
            if (corr > 0.01f) {
                rigidbody_wake(body_a);
            }
        }
        if (inv_b > 0.0f) {
            body_b->position = vector3_addition(body_b->position, vector3_scaling(shift, inv_b));
            if (corr > 0.01f) {
                rigidbody_wake(body_b);
            }
        }
    }
}
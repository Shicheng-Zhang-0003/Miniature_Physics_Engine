/* GTK4-PREP: zero GUI headers in scene. */
#include "scene_load.h"
#include "scene_init.h"
#include "scene_id_remap.h" /* MPE_FTC_058 */
#include "scene_crc.h"
#include "../core/physics_world.h"
#include "../core/rigidbody.h"
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include "../physics/spring_joint_types.h"
#include "../physics/constraint.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
static int read_float(FILE *f, float *v) {
    return fread(v, sizeof(float), 1, f) == 1;
}
static int read_int(FILE *f, int32_t *v) {
    return fread(v, sizeof(int32_t), 1, f) == 1;
}
static int read_vec3(FILE *f, vector3 *v) {
    return fread(v, sizeof(vector3), 1, f) == 1;
}
static int read_vec4(FILE *f, vector4 *v) {
    return fread(v, sizeof(vector4), 1, f) == 1;
}

/* R3-02: Staged scene load.
 *
 * The previous implementation called scene_clear() before reading the
 * bodies. A truncated or corrupt file would destroy the live scene and
 * replace it with a partial one.
 *
 * This implementation reads everything into a staging buffer first.
 * Only if the entire file reads successfully does it clear the scene
 * and commit. On any failure the live scene is untouched.
 */

/* Staged joint data read from file before committing. */
typedef struct {
    uint32_t id_a;
    uint32_t id_b;
    float eq;
    float k;
    float c;
} staged_joint;

/* Staged v200 revolute data (mirrors (physics_world_get_primary()->revolute_constraints) entries). */
typedef struct {
    uint32_t type;
    uint32_t id_a;
    uint32_t id_b;
    vector3 anchor_a;
    vector3 anchor_b;
    vector3 axis_a;
    vector3 axis_b;
    uint32_t motor_enabled;
    float motor_target;
    float motor_max_torque;
    uint32_t limits_enabled;
    float limit_min;
    float limit_max;
} staged_revolute;

/* Staged fixed constraint data. */
typedef struct {
    uint32_t type;
    uint32_t id_a;
    uint32_t id_b;
    vector3 anchor_a;
    vector3 anchor_b;
} staged_fixed;

/* Staged distance constraint data. */
typedef struct {
    uint32_t type;
    uint32_t id_a;
    uint32_t id_b;
    vector3 anchor_a;
    vector3 anchor_b;
    float rest_length;
} staged_distance;

/* Staged prismatic constraint data. */
typedef struct {
    uint32_t type;
    uint32_t id_a;
    uint32_t id_b;
    vector3 anchor_a;
    vector3 anchor_b;
    vector3 axis_a;
    vector3 axis_b;
    uint32_t motor_enabled;
    float motor_target_speed;
    float motor_max_force;
    uint32_t limits_enabled;
    float limit_min;
    float limit_max;
} staged_prismatic;

/* Staged rope constraint data. */
typedef struct {
    uint32_t type;
    uint32_t id_a;
    uint32_t id_b;
    vector3 anchor_a;
    vector3 anchor_b;
    float rest_length;
} staged_rope;

static int scene_load_vec3(FILE *f, uint32_t *crc, vector3 *v) {
    return scene_rfloat(f, crc, &v->x) && scene_rfloat(f, crc, &v->y) && scene_rfloat(f, crc, &v->z);
}

static int scene_load_quat(FILE *f, uint32_t *crc, vector4 *q) {
    return scene_rfloat(f, crc, &q->w) && scene_rfloat(f, crc, &q->x) && scene_rfloat(f, crc, &q->y) &&
           scene_rfloat(f, crc, &q->z);
}

static bool scene_id_in_staged(const int32_t *staged_ids, int staged_count, uint32_t id) {
    if (id == 0) {
        return false;
    }
    for (int i = 0; i < staged_count; i++) {
        if ((uint32_t) staged_ids[i] == id) {
            return true;
        }
    }
    return false;
}

/* Scene format v200 reader. The caller has consumed magic+version; the file
 * position is at body_count. Same staged discipline as the legacy path:
 * everything validates (including the CRC32 footer) before the live scene
 * is touched. IDs are preserved verbatim (no remap); the allocator is
 * advanced past them. Returns 1 on commit, 0 leaving the scene untouched. */
static int scene_loading_v200(FILE *f, uint32_t header_crc) {
    uint32_t crc = header_crc;
    uint32_t body_count_u = 0;
    if (!scene_r32(f, &crc, &body_count_u)) {
        return 0;
    }
    if ((body_count_u > (uint32_t) mpe_max_bodies)) {
        return 0;
    }
    int count = (int) body_count_u;
    if (count == 0) {
        /* Empty scene round-trips: valid header + zero bodies + joints + CRC. */
        /* Fall through to section parsing with zero bodies. */
    }
    if (!scene_ensure_pool_capacity(count)) {
        return 0;
    }
    /* count is now guaranteed <= body_capacity after ensure_pool_capacity. */
    if (count > (physics_world_get_primary()->body_capacity)) {
        return 0;
    }

    /* Allocate at least 1 element even for count==0 (malloc(0) may
     * return NULL on some platforms, which would abort the load). */
    int alloc_count = count > 0 ? count : 1;
    rigidbody *staged_bodies = (rigidbody *) malloc((size_t) alloc_count * sizeof(rigidbody));
    int32_t *staged_ids = (int32_t *) malloc((size_t) alloc_count * sizeof(int32_t));
    if ((!staged_bodies) || (!staged_ids)) {
        free(staged_bodies);
        free(staged_ids);
        return 0;
    }
    for (int i = 0; i < count; i++) {
        staged_ids[i] = 0;
    }

    int staged_body_count = 0;
    int body_ok = 1;
    for (int i = 0; (i < count) && body_ok; i++) {
        uint32_t type_u = 0, static_u = 0, id_u = 0, nice_u = 0, sleep_u = 0, kin_u = 0, gen_u = 0;
        float mass = 0.0f, radius = 0.0f, half_len = 0.0f;
        float sleep_timer_f = 0.0f;
        vector3 half_ext = {0.0f, 0.0f, 0.0f};
        vector3 pos = {0.0f, 0.0f, 0.0f}, vel = {0.0f, 0.0f, 0.0f}, ang = {0.0f, 0.0f, 0.0f};
        vector4 orient = {1.0f, 0.0f, 0.0f, 0.0f};
        vector3 colour = {0.0f, 0.0f, 0.0f};
        float rest = 0.0f, fs = 0.0f, fk = 0.0f;
        body_ok = body_ok && scene_r32(f, &crc, &type_u);
        body_ok = body_ok && scene_rfloat(f, &crc, &mass);
        body_ok = body_ok && scene_rfloat(f, &crc, &radius);
        body_ok = body_ok && scene_rfloat(f, &crc, &half_len);
        body_ok = body_ok && scene_load_vec3(f, &crc, &half_ext);
        body_ok = body_ok && scene_load_vec3(f, &crc, &pos);
        body_ok = body_ok && scene_load_vec3(f, &crc, &vel);
        body_ok = body_ok && scene_load_vec3(f, &crc, &ang);
        body_ok = body_ok && scene_load_quat(f, &crc, &orient);
        body_ok = body_ok && scene_load_vec3(f, &crc, &colour);
        body_ok = body_ok && scene_rfloat(f, &crc, &rest);
        body_ok = body_ok && scene_rfloat(f, &crc, &fs);
        body_ok = body_ok && scene_rfloat(f, &crc, &fk);
        body_ok = body_ok && scene_r32(f, &crc, &static_u);
        body_ok = body_ok && scene_r32(f, &crc, &id_u);
        body_ok = body_ok && scene_r32(f, &crc, &nice_u);
        body_ok = body_ok && scene_r32(f, &crc, &sleep_u);
        body_ok = body_ok && scene_rfloat(f, &crc, &sleep_timer_f);
        body_ok = body_ok && scene_r32(f, &crc, &kin_u);
        body_ok = body_ok && scene_r32(f, &crc, &gen_u);
        if (!body_ok) {
            break;
        }
        /* Structural validation (CRC gates corruption; this gates logic).
         * object_custom persists as bare type tag (no plugin blob yet):
         * accept and init as sphere placeholder so one custom body never
         * vetoes the whole file. Full blob persistence is future work. */
        if ((type_u > (uint32_t) object_custom) || (id_u == 0) || (!isfinite(mass)) || (mass < 0.0f)) {
            body_ok = 0;
            break;
        }
        /* Full float validation: NaN/Inf pos/vel/ang/orient/colour/material
         * must veto the file, never poison the live world. Zero-quat vetoed
         * (normalisation would yield identity, hiding corruption). */
        {
            float vals[] = {pos.x, pos.y, pos.z, vel.x, vel.y, vel.z,
                            ang.x, ang.y, ang.z, orient.w, orient.x, orient.y, orient.z,
                            colour.x, colour.y, colour.z, rest, fs, fk, sleep_timer_f,
                            radius, half_len, half_ext.x, half_ext.y, half_ext.z};
            bool all_fin = true;
            for (size_t vi = 0; vi < sizeof(vals) / sizeof(vals[0]); vi++) {
                if (!isfinite(vals[vi])) {
                    all_fin = false;
                    break;
                }
            }
            double q2 = (double)orient.w * orient.w + (double)orient.x * orient.x +
                        (double)orient.y * orient.y + (double)orient.z * orient.z;
            if (!all_fin || !(q2 > 1e-12) || fabsf(pos.x) > 1e6f || fabsf(pos.y) > 1e6f ||
                fabsf(pos.z) > 1e6f) {
                body_ok = 0;
                break;
            }
        }
        /* Duplicate-ID veto: staged file with cloned IDs would alias the
         * id_cache and mis-wire joints. */
        for (int di = 0; di < i; di++) {
            if (staged_ids[di] == (int32_t)id_u) {
                body_ok = 0;
                break;
            }
        }
        if (!body_ok) {
            break;
        }
        object_type type = (object_type) type_u;
        if (type == object_cube) {
            rigidbody_initialisation_cube(&staged_bodies[i], pos, half_ext, mass);
        } else if (type == object_cylinder) {
            rigidbody_initialisation_cylinder(&staged_bodies[i], radius, half_len, mass, pos);
        } else if (type == object_custom) {
            /* v200 has no custom blob: restore as custom id 100 (capsule)
             * with persisted radius so dispatch still finds the handler
             * instead of degrading to sphere. */
            memset(&staged_bodies[i], 0, sizeof(staged_bodies[i]));
            staged_bodies[i].type = object_custom;
            staged_bodies[i].custom_shape = 100;
            staged_bodies[i].radius = (isfinite(radius) && radius > 0.0f) ? radius : 0.5f;
            staged_bodies[i].mass = mass;
            staged_bodies[i].inverse_mass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
            staged_bodies[i].position = pos;
            staged_bodies[i].orientation = vector4_identity();
            staged_bodies[i].half_extensions = (vector3){staged_bodies[i].radius,
                                                         staged_bodies[i].radius,
                                                         staged_bodies[i].radius};
        } else {
            rigidbody_initialisation_sphere(&staged_bodies[i], radius, mass, pos);
        }
        staged_bodies[i].velocity = vel;
        staged_bodies[i].angular_velocity = ang;
        staged_bodies[i].orientation = vector4_normalisation(orient);
        staged_bodies[i].colour = colour;
        staged_bodies[i].restitution = rest;
        staged_bodies[i].friction_static = fs;
        staged_bodies[i].friction_kinetic = fk;
        staged_bodies[i].nice_value = (int) nice_u;
        staged_bodies[i].object_id = id_u;
        staged_bodies[i].object_generation = gen_u;
        if (static_u != 0) {
            rigidbody_set_static(&staged_bodies[i], true);
        } else {
            rigidbody_set_static(&staged_bodies[i], false);
        }
        if (kin_u != 0) {
            rigidbody_set_kinematic(&staged_bodies[i], true);
        }
        if ((sleep_u != 0) && (!staged_bodies[i].static_state) && (!staged_bodies[i].kinematic)) {
            staged_bodies[i].is_sleeping = true;
            staged_bodies[i].sleep_timer = sleep_timer_f;
            staged_bodies[i].velocity = vector3_zero();
            staged_bodies[i].angular_velocity = vector3_zero();
        }
        rigidbody_sanitize(&staged_bodies[i]);
        /* Sanitize repairs out-of-range material values but must not
         * renumber a preserved identity. */
        staged_bodies[i].object_id = id_u;
        staged_bodies[i].object_generation = gen_u;
        staged_ids[i] = (int32_t) id_u;
        staged_body_count++;
    }

    /* Springs (validated against staged IDs; dangling entries dropped). */
    uint32_t spring_count_u = 0;
    staged_joint *staged_springs = NULL;
    int staged_spring_count = 0;
    int springs_ok = body_ok;
    if (springs_ok) {
        springs_ok = springs_ok && scene_r32(f, &crc, &spring_count_u);
    }
    if (springs_ok && (spring_count_u > 0) && (spring_count_u <= (uint32_t) mpe_max_joints)) {
        staged_springs = (staged_joint *) malloc((size_t) spring_count_u * sizeof(staged_joint));
        if (!staged_springs) {
            springs_ok = 0;
        }
    } else if (springs_ok && (spring_count_u > (uint32_t) mpe_max_joints)) {
        springs_ok = 0;
    }
    for (uint32_t j = 0; springs_ok && (j < spring_count_u); j++) {
        uint32_t id_a = 0, id_b = 0;
        float eq = 0.0f, k = 0.0f, c = 0.0f;
        springs_ok = springs_ok && scene_r32(f, &crc, &id_a);
        springs_ok = springs_ok && scene_r32(f, &crc, &id_b);
        springs_ok = springs_ok && scene_rfloat(f, &crc, &eq);
        springs_ok = springs_ok && scene_rfloat(f, &crc, &k);
        springs_ok = springs_ok && scene_rfloat(f, &crc, &c);
        if (!springs_ok) {
            break;
        }
        if ((id_a == 0) || (id_a == id_b) || (!scene_id_in_staged(staged_ids, staged_body_count, id_a)) ||
            (!scene_id_in_staged(staged_ids, staged_body_count, id_b)) || (!isfinite(eq)) || (!isfinite(k)) ||
            (!isfinite(c)) || (k < 0.0f) || (c < 0.0f)) {
            continue;
        }
        staged_springs[staged_spring_count].id_a = id_a;
        staged_springs[staged_spring_count].id_b = id_b;
        staged_springs[staged_spring_count].eq = (eq < 0.0f) ? 0.0f : eq;
        staged_springs[staged_spring_count].k = k;
        staged_springs[staged_spring_count].c = c;
        staged_spring_count++;
    }

    /* Fixed constraints (v200+). */
    uint32_t fixed_count_u = 0;
    staged_fixed *staged_fixeds = NULL;
    int staged_fixed_count = 0;
    int fixed_ok = springs_ok;
    if (fixed_ok) {
        fixed_ok = fixed_ok && scene_r32(f, &crc, &fixed_count_u);
    }
    if (fixed_ok && (fixed_count_u > 0) && (fixed_count_u <= (uint32_t) mpe_max_joints)) {
        staged_fixeds = (staged_fixed *) malloc((size_t) fixed_count_u * sizeof(staged_fixed));
        if (!staged_fixeds) {
            fixed_ok = 0;
        }
    } else if (fixed_ok && (fixed_count_u > (uint32_t) mpe_max_joints)) {
        fixed_ok = 0;
    }
    for (uint32_t j = 0; fixed_ok && (j < fixed_count_u); j++) {
        staged_fixed fc;
        fixed_ok = fixed_ok && scene_r32(f, &crc, &fc.type);
        fixed_ok = fixed_ok && scene_r32(f, &crc, &fc.id_a);
        fixed_ok = fixed_ok && scene_r32(f, &crc, &fc.id_b);
        fixed_ok = fixed_ok && scene_load_vec3(f, &crc, &fc.anchor_a);
        fixed_ok = fixed_ok && scene_load_vec3(f, &crc, &fc.anchor_b);
        if (!fixed_ok) break;
        if ((fc.type != (uint32_t) constraint_fixed) || (fc.id_a == 0) || (fc.id_a == fc.id_b) ||
            (!scene_id_in_staged(staged_ids, staged_body_count, fc.id_a)) ||
            (!scene_id_in_staged(staged_ids, staged_body_count, fc.id_b))) {
            continue;
        }
        staged_fixeds[staged_fixed_count++] = fc;
    }

    /* Distance constraints (v200+). */
    uint32_t dist_count_u = 0;
    staged_distance *staged_dists = NULL;
    int staged_dist_count = 0;
    int dist_ok = fixed_ok;
    if (dist_ok) {
        dist_ok = dist_ok && scene_r32(f, &crc, &dist_count_u);
    }
    if (dist_ok && (dist_count_u > 0) && (dist_count_u <= (uint32_t) mpe_max_joints)) {
        staged_dists = (staged_distance *) malloc((size_t) dist_count_u * sizeof(staged_distance));
        if (!staged_dists) {
            dist_ok = 0;
        }
    } else if (dist_ok && (dist_count_u > (uint32_t) mpe_max_joints)) {
        dist_ok = 0;
    }
    for (uint32_t j = 0; dist_ok && (j < dist_count_u); j++) {
        staged_distance dc;
        dist_ok = dist_ok && scene_r32(f, &crc, &dc.type);
        dist_ok = dist_ok && scene_r32(f, &crc, &dc.id_a);
        dist_ok = dist_ok && scene_r32(f, &crc, &dc.id_b);
        dist_ok = dist_ok && scene_load_vec3(f, &crc, &dc.anchor_a);
        dist_ok = dist_ok && scene_load_vec3(f, &crc, &dc.anchor_b);
        dist_ok = dist_ok && scene_rfloat(f, &crc, &dc.rest_length);
        if (!dist_ok) break;
        if ((dc.type != (uint32_t) constraint_distance) || (dc.id_a == 0) || (dc.id_a == dc.id_b) ||
            (!scene_id_in_staged(staged_ids, staged_body_count, dc.id_a)) ||
            (!scene_id_in_staged(staged_ids, staged_body_count, dc.id_b)) ||
            (!isfinite(dc.rest_length)) || (dc.rest_length < 0.0f)) {
            continue;
        }
        staged_dists[staged_dist_count++] = dc;
    }

    /* Prismatic constraints (v200+). */
    uint32_t prism_count_u = 0;
    staged_prismatic *staged_prisms = NULL;
    int staged_prism_count = 0;
    int prism_ok = dist_ok;
    if (prism_ok) {
        prism_ok = prism_ok && scene_r32(f, &crc, &prism_count_u);
    }
    if (prism_ok && (prism_count_u > 0) && (prism_count_u <= (uint32_t) mpe_max_joints)) {
        staged_prisms = (staged_prismatic *) malloc((size_t) prism_count_u * sizeof(staged_prismatic));
        if (!staged_prisms) {
            prism_ok = 0;
        }
    } else if (prism_ok && (prism_count_u > (uint32_t) mpe_max_joints)) {
        prism_ok = 0;
    }
    for (uint32_t j = 0; prism_ok && (j < prism_count_u); j++) {
        staged_prismatic pc;
        uint32_t motor_u = 0, limits_u = 0;
        prism_ok = prism_ok && scene_r32(f, &crc, &pc.type);
        prism_ok = prism_ok && scene_r32(f, &crc, &pc.id_a);
        prism_ok = prism_ok && scene_r32(f, &crc, &pc.id_b);
        prism_ok = prism_ok && scene_load_vec3(f, &crc, &pc.anchor_a);
        prism_ok = prism_ok && scene_load_vec3(f, &crc, &pc.anchor_b);
        prism_ok = prism_ok && scene_load_vec3(f, &crc, &pc.axis_a);
        prism_ok = prism_ok && scene_load_vec3(f, &crc, &pc.axis_b);
        prism_ok = prism_ok && scene_r32(f, &crc, &motor_u);
        prism_ok = prism_ok && scene_rfloat(f, &crc, &pc.motor_target_speed);
        prism_ok = prism_ok && scene_rfloat(f, &crc, &pc.motor_max_force);
        prism_ok = prism_ok && scene_r32(f, &crc, &limits_u);
        prism_ok = prism_ok && scene_rfloat(f, &crc, &pc.limit_min);
        prism_ok = prism_ok && scene_rfloat(f, &crc, &pc.limit_max);
        if (!prism_ok) break;
        pc.motor_enabled = motor_u;
        pc.limits_enabled = limits_u;
        if ((pc.type != (uint32_t) constraint_prismatic) || (pc.id_a == 0) || (pc.id_a == pc.id_b) ||
            (!scene_id_in_staged(staged_ids, staged_body_count, pc.id_a)) ||
            (!scene_id_in_staged(staged_ids, staged_body_count, pc.id_b)) ||
            (!isfinite(pc.motor_target_speed)) || (!isfinite(pc.motor_max_force)) || (pc.motor_max_force < 0.0f) ||
            (!isfinite(pc.limit_min)) || (!isfinite(pc.limit_max))) {
            continue;
        }
        staged_prisms[staged_prism_count++] = pc;
    }

    /* Rope constraints (v200+). */
    uint32_t rope_count_u = 0;
    staged_rope *staged_ropes = NULL;
    int staged_rope_count = 0;
    int rope_ok = prism_ok;
    if (rope_ok) {
        rope_ok = rope_ok && scene_r32(f, &crc, &rope_count_u);
    }
    if (rope_ok && (rope_count_u > 0) && (rope_count_u <= (uint32_t) mpe_max_joints)) {
        staged_ropes = (staged_rope *) malloc((size_t) rope_count_u * sizeof(staged_rope));
        if (!staged_ropes) {
            rope_ok = 0;
        }
    } else if (rope_ok && (rope_count_u > (uint32_t) mpe_max_joints)) {
        rope_ok = 0;
    }
    for (uint32_t j = 0; rope_ok && (j < rope_count_u); j++) {
        staged_rope rc;
        rope_ok = rope_ok && scene_r32(f, &crc, &rc.type);
        rope_ok = rope_ok && scene_r32(f, &crc, &rc.id_a);
        rope_ok = rope_ok && scene_r32(f, &crc, &rc.id_b);
        rope_ok = rope_ok && scene_load_vec3(f, &crc, &rc.anchor_a);
        rope_ok = rope_ok && scene_load_vec3(f, &crc, &rc.anchor_b);
        rope_ok = rope_ok && scene_rfloat(f, &crc, &rc.rest_length);
        if (!rope_ok) break;
        if ((rc.type != (uint32_t) constraint_rope) || (rc.id_a == 0) || (rc.id_a == rc.id_b) ||
            (!scene_id_in_staged(staged_ids, staged_body_count, rc.id_a)) ||
            (!scene_id_in_staged(staged_ids, staged_body_count, rc.id_b)) ||
            (!isfinite(rc.rest_length)) || (rc.rest_length < 0.0f)) {
            continue;
        }
        staged_ropes[staged_rope_count++] = rc;
    }

    /* Revolute constraints (v200+, kept for backward compatibility). */
    uint32_t rev_count_u = 0;
    staged_revolute *staged_revs = NULL;
    int staged_rev_count = 0;
    int revs_ok = rope_ok;
    if (revs_ok) {
        revs_ok = revs_ok && scene_r32(f, &crc, &rev_count_u);
    }
    if (revs_ok && (rev_count_u > 0) && (rev_count_u <= (uint32_t) mpe_max_joints)) {
        staged_revs = (staged_revolute *) malloc((size_t) rev_count_u * sizeof(staged_revolute));
        if (!staged_revs) {
            revs_ok = 0;
        }
    } else if (revs_ok && (rev_count_u > (uint32_t) mpe_max_joints)) {
        revs_ok = 0;
    }
    for (uint32_t j = 0; revs_ok && (j < rev_count_u); j++) {
        staged_revolute r;
        uint32_t motor_u = 0, limits_u = 0;
        revs_ok = revs_ok && scene_r32(f, &crc, &r.type);
        revs_ok = revs_ok && scene_r32(f, &crc, &r.id_a);
        revs_ok = revs_ok && scene_r32(f, &crc, &r.id_b);
        revs_ok = revs_ok && scene_load_vec3(f, &crc, &r.anchor_a);
        revs_ok = revs_ok && scene_load_vec3(f, &crc, &r.anchor_b);
        revs_ok = revs_ok && scene_load_vec3(f, &crc, &r.axis_a);
        revs_ok = revs_ok && scene_load_vec3(f, &crc, &r.axis_b);
        revs_ok = revs_ok && scene_r32(f, &crc, &motor_u);
        revs_ok = revs_ok && scene_rfloat(f, &crc, &r.motor_target);
        revs_ok = revs_ok && scene_rfloat(f, &crc, &r.motor_max_torque);
        revs_ok = revs_ok && scene_r32(f, &crc, &limits_u);
        revs_ok = revs_ok && scene_rfloat(f, &crc, &r.limit_min);
        revs_ok = revs_ok && scene_rfloat(f, &crc, &r.limit_max);
        if (!revs_ok) {
            break;
        }
        r.motor_enabled = motor_u;
        r.limits_enabled = limits_u;
        if ((r.type != (uint32_t) constraint_revolute) || (r.id_a == 0) || (r.id_a == r.id_b) ||
            (!scene_id_in_staged(staged_ids, staged_body_count, r.id_a)) ||
            (!scene_id_in_staged(staged_ids, staged_body_count, r.id_b)) ||
            (!isfinite(r.motor_target)) || (!isfinite(r.motor_max_torque)) || (r.motor_max_torque < 0.0f) ||
            (!isfinite(r.limit_min)) || (!isfinite(r.limit_max))) {
            continue;
        }
        staged_revs[staged_rev_count++] = r;
    }

    /* Footer CRC over every preceding byte (read raw, then compare). */
    int crc_ok = revs_ok;
    unsigned char footer[4] = {0, 0, 0, 0};
    if (crc_ok) {
        crc_ok = (fread(footer, 1, 4, f) == 4);
    }
    if (crc_ok) {
        uint32_t stored = ((uint32_t) footer[0]) | (((uint32_t) footer[1]) << 8) |
                          (((uint32_t) footer[2]) << 16) | (((uint32_t) footer[3]) << 24);
        crc_ok = (stored == (crc ^ 0xFFFFFFFFu));
    }
    if (!crc_ok) {
        fprintf(stderr, "Error LDF04: CRC mismatch (corrupt scene file)\n");
        free(staged_bodies);
        free(staged_ids);
        free(staged_springs);
        free(staged_revs);
        free(staged_fixeds);
        free(staged_dists);
        free(staged_prisms);
        free(staged_ropes);
        return 0;
    }

    /* --- Commit: clear scene AND joint pools, install staged data --- */
    scene_clear();
    constraint_pool_init(physics_world_get_primary());
    scene_id_remap_reset();
    contact_cache_clear(physics_world_get_primary());
    joint_init_pool(physics_world_get_primary());

    (physics_world_get_primary()->body_count) = staged_body_count;
    for (int i = 0; i < staged_body_count; i++) {
        int staged_cs = staged_bodies[i].custom_shape;
        (physics_world_get_primary()->bodies)[i] = staged_bodies[i];
        (physics_world_get_primary()->bodies)[i].body_index = i;
        /* Preserve foreign custom identity (staged); non-custom bodies
         * keep custom_shape=-1. Clobbering customs to -1 broke dispatch
         * (100/-1 mismatch rejects the capsule handler). */
        if ((physics_world_get_primary()->bodies)[i].type == object_custom) {
            if (staged_cs < 100) {
                staged_cs = 100;
            }
            (physics_world_get_primary()->bodies)[i].custom_shape = staged_cs;
        } else {
            (physics_world_get_primary()->bodies)[i].custom_shape = -1;
        }
        (physics_world_get_primary()->bodies)[i].max_relative_speed_sq = 0.0f;
        rigidbody_update_axes(&(physics_world_get_primary()->bodies)[i]);
        scene_note_loaded_id((physics_world_get_primary()->bodies)[i].object_id);
    }
    physics_world_bump_revision(physics_world_get_primary());
    for (int j = 0; j < staged_spring_count; j++) {
        add_joint_by_ids(physics_world_get_primary(), staged_springs[j].id_a, staged_springs[j].id_b, staged_springs[j].eq, staged_springs[j].k,
                         staged_springs[j].c);
    }
    for (int j = 0; j < staged_rev_count; j++) {
        staged_revolute *r = &staged_revs[j];
        int index = constraint_add_revolute(physics_world_get_primary(), r->id_a, r->id_b, r->anchor_a, r->anchor_b, r->axis_a);
        if (index >= 0) {
            constraint_set_revolute_axes(physics_world_get_primary(), index, r->axis_a, r->axis_b);
            constraint_set_revolute_motor(physics_world_get_primary(), index, r->motor_enabled != 0, r->motor_target, r->motor_max_torque);
            constraint_set_revolute_limits(physics_world_get_primary(), index, r->limits_enabled != 0, r->limit_min, r->limit_max);
        }
    }

    /* Install fixed constraints. */
    for (int j = 0; j < staged_fixed_count; j++) {
        staged_fixed *fc = &staged_fixeds[j];
        constraint_add_fixed(physics_world_get_primary(),
            scene_id_remap_resolve(fc->id_a),
            scene_id_remap_resolve(fc->id_b),
            fc->anchor_a, fc->anchor_b);
    }

    /* Install distance constraints. */
    for (int j = 0; j < staged_dist_count; j++) {
        staged_distance *dc = &staged_dists[j];
        constraint_add_distance(physics_world_get_primary(),
            scene_id_remap_resolve(dc->id_a),
            scene_id_remap_resolve(dc->id_b),
            dc->anchor_a, dc->anchor_b, dc->rest_length);
    }

    /* Install prismatic constraints. */
    for (int j = 0; j < staged_prism_count; j++) {
        staged_prismatic *pc = &staged_prisms[j];
        int index = constraint_add_prismatic(physics_world_get_primary(),
            scene_id_remap_resolve(pc->id_a),
            scene_id_remap_resolve(pc->id_b),
            pc->anchor_a, pc->anchor_b, pc->axis_a);
        if (index >= 0) {
            constraint_set_prismatic_axes(physics_world_get_primary(), index, pc->axis_a, pc->axis_b);
            constraint_set_prismatic_motor(physics_world_get_primary(), index, pc->motor_enabled != 0, pc->motor_target_speed, pc->motor_max_force);
            constraint_set_prismatic_limits(physics_world_get_primary(), index, pc->limits_enabled != 0, pc->limit_min, pc->limit_max);
        }
    }

    /* Install rope constraints. */
    for (int j = 0; j < staged_rope_count; j++) {
        staged_rope *rc = &staged_ropes[j];
        constraint_add_rope(physics_world_get_primary(),
            scene_id_remap_resolve(rc->id_a),
            scene_id_remap_resolve(rc->id_b),
            rc->anchor_a, rc->anchor_b, rc->rest_length);
    }

    free(staged_bodies);
    free(staged_ids);
    free(staged_springs);
    free(staged_revs);
    free(staged_fixeds);
    free(staged_dists);
    free(staged_prisms);
    free(staged_ropes);
    return 1;
}

int scene_loading(const char *file_source_path)
{
    FILE *f = fopen(file_source_path, "rb");
    if (!f) {
        fprintf(stderr, "Error LDF01: Could not open %s\n", file_source_path);
        return 0;
    }

    /* --- Read and validate header --- */
    /* Magic and version decode as little-endian: v1 files wrote native
     * order (LE bytes on every supported machine) and v200 writes LE
     * explicitly, so one decoder accepts both. Past this point v200
     * continues in scene_loading_v200; older versions use the native
     * legacy reader below (byte-identical on LE). */
    /* The v200 CRC covers every byte before the footer, magic included:
     * feed the header through the running checksum here and hand it to
     * the v200 reader. (Legacy versions have no checksum.) */
    uint32_t header_crc = 0xFFFFFFFFu;
    uint32_t magic_le = 0, version_le = 0;
    if ((!scene_r32(f, &header_crc, &magic_le)) || (magic_le != (uint32_t) mpe_magic)) {
        fprintf(stderr, "Error LDF02: Invalid magic number\n");
        fclose(f);
        return 0;
    }
    if (!scene_r32(f, &header_crc, &version_le)) {
        fprintf(stderr, "Error LDF03: Version mismatch\n");
        fclose(f);
        return 0;
    }
    int32_t version = (int32_t) version_le;
    int32_t count = 0;

    if (version == 200) {
        int v200_result = scene_loading_v200(f, header_crc);
        fclose(f);
        return v200_result;
    }

    /* Legacy native-order reader (v130..v153). */
    if ((version != 130) && (version != 140) && (version != 150) && (version != 151) && (version != 152) &&
        (version != 153)) {
        fprintf(stderr, "Error LDF03: Version mismatch\n");
        fclose(f);
        return 0;
    }

    if ((!read_int(f, &count)) || (count < 0)) {
        fclose(f);
        return 0;
    }

    if (count == 0) {
        /* Empty legacy scene is valid: clear and succeed (v200 already
         * handles count==0; legacy malloc(0) could return NULL and
         * misreport OOM). */
        scene_clear();
        fclose(f);
        return 1;
    }

    if (count > mpe_max_bodies) {
        count = mpe_max_bodies;
    }

    if (!scene_ensure_pool_capacity(count)) {
        fclose(f);
        return 0;
    }

    if (count > (physics_world_get_primary()->body_capacity)) {
        count = (physics_world_get_primary()->body_capacity);
    }

    /* --- Allocate staging buffers --- */
    rigidbody *staged_bodies = (rigidbody *)malloc((size_t)count * sizeof(rigidbody));
    if (!staged_bodies) {
        fclose(f);
        return 0;
    }

    /* Read joint count to size the joint staging buffer.
     * We need to seek past the bodies to find it, so we read
     * bodies first, then read joints. */
    int32_t staged_joint_count = 0;
    staged_joint *staged_joints = NULL;
    /* Saved object IDs staged separately (v150+). Previously nice_value
     * was abused as a temp holder, which destroyed the persisted value. */
    int32_t *staged_ids = NULL;
    if (count > 0) {
        staged_ids = (int32_t *)malloc((size_t)count * sizeof(int32_t));
        if (!staged_ids) {
            free(staged_bodies);
            fclose(f);
            return 0;
        }
        for (int i = 0; i < count; i++) staged_ids[i] = 0;
    }

    /* --- Stage all bodies --- */
    int staged_body_count = 0;

    for (int i = 0; i < count; i++) {
        rigidbody temp;
        int32_t type_int, static_int, saved_object_id = 0;
        int32_t saved_nice = 0, saved_sleep = 0, saved_kinematic = 0;

        /* Field order must match save_scene: type, mass, radius,
         * half_length (v151+), half extents, position, velocity,
         * angular velocity, orientation, colour, restitution,
         * friction x2, static flag, object id (v150+),
         * nice_value + sleep state (v152+). */
        if (!read_int(f, &type_int))               break;
        if (!read_float(f, &temp.mass))            break;
        if (!read_float(f, &temp.radius))          break;
        /* R3-04: Read cylinder_half_length. Present in version >= 151.
         * For older versions, default to radius/2. */
        if (version >= 151) {
            if (!read_float(f, &temp.cylinder_half_length)) break;
        } else {
            temp.cylinder_half_length = temp.radius * 0.5f;
        }
        if (!read_vec3(f, &temp.half_extensions))  break;
        if (!read_vec3(f, &temp.position))         break;
        if (!read_vec3(f, &temp.velocity))         break;
        if (!read_vec3(f, &temp.angular_velocity)) break;
        if (!read_vec4(f, &temp.orientation))      break;
        if (!read_vec3(f, &temp.colour))           break;
        if (!read_float(f, &temp.restitution))     break;
        if (!read_float(f, &temp.friction_static)) break;
        if (!read_float(f, &temp.friction_kinetic)) break;
        if (!read_int(f, &static_int))             break;

        saved_object_id = 0;
        if (version >= 150) {
            if (!read_int(f, &saved_object_id)) break;
        }
        if (version >= 152) {
            if (!read_int(f, &saved_nice)) break;
            if (!read_int(f, &saved_sleep)) break;
        }
        if (version >= 153) {
            if (!read_int(f, &saved_kinematic)) break;
        }

        temp.type = (object_type)type_int;
        temp.static_state = (static_int != 0);

        /* Initialise the staged body */
        if (temp.type == object_cube) {
            rigidbody_initialisation_cube(&staged_bodies[i],
                temp.position, temp.half_extensions, temp.mass);
        } else if (temp.type == object_cylinder) {
            /* R3-04: Cylinder branch. Previously cylinders were silently
             * re-initialised as spheres, corrupting their geometry. */
            rigidbody_initialisation_cylinder(&staged_bodies[i],
                temp.radius, temp.cylinder_half_length, temp.mass, temp.position);
        } else {
            rigidbody_initialisation_sphere(&staged_bodies[i],
                temp.radius, temp.mass, temp.position);
        }

        staged_bodies[i].velocity          = temp.velocity;
        staged_bodies[i].angular_velocity  = temp.angular_velocity;
        staged_bodies[i].orientation       = vector4_normalisation(temp.orientation);
        staged_bodies[i].colour            = temp.colour;
        staged_bodies[i].restitution       = temp.restitution;
        staged_bodies[i].friction_static   = temp.friction_static;
        staged_bodies[i].friction_kinetic  = temp.friction_kinetic;
        staged_bodies[i].static_state      = temp.static_state;

        if (staged_bodies[i].static_state) {
            rigidbody_set_static(&staged_bodies[i], true);
        } else {
            rigidbody_set_static(&staged_bodies[i], false);
        }

        /* Single sanitization point: set_static already refreshed inertia
         * and axes; sanitize once after sleep/kinematic assignment below.
         * (Old code sanitized here AND after update_axes AND inside
         * set_kinematic — triple work per body on every load.) */

        /* nice_value and sleep state persist (v152+); older files load
         * awake with default damping. Saved IDs go to staged_ids. */
        staged_bodies[i].nice_value = saved_nice;
        if (saved_kinematic != 0) {
            rigidbody_set_kinematic(&staged_bodies[i], true);
        }
        if ((saved_sleep != 0) && (!staged_bodies[i].static_state) && (!staged_bodies[i].kinematic)) {
            staged_bodies[i].is_sleeping = true;
            staged_bodies[i].sleep_timer = 0.0f;
            staged_bodies[i].velocity = vector3_zero();
            staged_bodies[i].angular_velocity = vector3_zero();
        }
        /* Single sanitize covers the whole legacy path (set_kinematic already
         * sanitizes, but non-kinematic bodies need it here). */
        if (saved_kinematic == 0) {
            rigidbody_sanitize(&staged_bodies[i]);
        }
        staged_ids[i] = saved_object_id;

        staged_body_count++;
    }

    /* --- Stage all joints --- */
    if (read_int(f, &staged_joint_count) && (staged_joint_count > 0)) {
        staged_joints = (staged_joint *)malloc(
            (size_t)staged_joint_count * sizeof(staged_joint));
        if (!staged_joints) {
            free(staged_bodies);
            free(staged_ids);
            fclose(f);
            return 0;
        }

        for (int j = 0; j < staged_joint_count; j++) {
            int32_t id_a, id_b;
            float eq, k, c;

            if (!read_int(f, &id_a))   { staged_joint_count = j; break; }
            if (!read_int(f, &id_b))   { staged_joint_count = j; break; }
            if (!read_float(f, &eq))   { staged_joint_count = j; break; }
            if (!read_float(f, &k))    { staged_joint_count = j; break; }
            if (!read_float(f, &c))    { staged_joint_count = j; break; }

            staged_joints[j].id_a = (uint32_t)id_a;
            staged_joints[j].id_b = (uint32_t)id_b;
            staged_joints[j].eq   = eq;
            staged_joints[j].k    = k;
            staged_joints[j].c    = c;
        }
    } else {
        staged_joint_count = 0;
    }

    fclose(f);

    /* --- Validate staged data before committing --- */
    if (staged_body_count == 0 || staged_body_count < count) {
        /* Nothing to commit or partial read (truncated file). Do not
         * clear the live scene — a partial load must leave the scene
         * untouched. */
        free(staged_bodies);
        if (staged_ids) free(staged_ids);
        if (staged_joints) free(staged_joints);
        return 0;
    }

    /* --- Commit: clear scene and install staged data --- */
    scene_clear();
    /* FIX-AUDIT: stale revolute joints survived every load (only the
     * spring pool was reset), constraining dead IDs. v1 files carry no
     * revolute data, so clearing is strictly more correct there too. */
    constraint_pool_init(physics_world_get_primary());
    scene_id_remap_reset();
    contact_cache_clear(physics_world_get_primary());
    joint_init_pool(physics_world_get_primary());

    (physics_world_get_primary()->body_count) = staged_body_count;

    for (int i = 0; i < staged_body_count; i++) {
        (physics_world_get_primary()->bodies)[i] = staged_bodies[i];

        /* Recover the saved object ID from staging (v150+). */
        int32_t saved_id = staged_ids[i];

        (physics_world_get_primary()->bodies)[i].object_id = scene_allocate_object_id();

        if ((version >= 150) && (saved_id > 0)) {
            scene_id_remap_add((uint32_t)saved_id, (physics_world_get_primary()->bodies)[i].object_id);
        }

        (physics_world_get_primary()->bodies)[i].object_generation = 1;
    }

    /* Install staged joints */
    for (int j = 0; j < staged_joint_count; j++) {
        add_joint_by_ids(physics_world_get_primary(), 
            scene_id_remap_resolve(staged_joints[j].id_a),
            scene_id_remap_resolve(staged_joints[j].id_b),
            staged_joints[j].eq,
            staged_joints[j].k,
            staged_joints[j].c);
    }

    free(staged_bodies);
    if (staged_ids) free(staged_ids);
    if (staged_joints) free(staged_joints);

    return 1;
}

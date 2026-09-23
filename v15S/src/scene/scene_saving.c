/* GTK4-PREP: zero GUI headers in scene. */
#include "scene_saving.h"
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
#include <unistd.h>
#include <sys/stat.h>

/* Scene format v200: explicit little-endian fields + sectioned joints +
 * CRC32 footer. Layout (all integers/floats LE):
 *   u32 magic ("MPE3"), u32 version (200), u32 body_count,
 *   bodies[]: u32 type, f32 mass, radius, half_length,
 *     half_extents xyz, position xyz, velocity xyz, angular_velocity xyz,
 *     orientation wxyz, colour xyz, restitution, fric_s, fric_k,
 *     u32 static, u32 object_id, i32 nice_value, u32 sleeping, u32 kinematic,
 *     u32 generation,
 *   u32 spring_count, springs[]: u32 id_a, id_b, f32 eq, k, c,
 *   u32 revolute_count, revolutes[]: u32 type, id_a, id_b,
 *     anchor_a xyz, anchor_b xyz, axis_a xyz, axis_b xyz,
 *     u32 motor_enabled, f32 target, max_torque,
 *     u32 limits_enabled, f32 limit_min, limit_max,
 *   u32 fixed_count, fixeds[]: u32 type, id_a, id_b,
 *     anchor_a xyz, anchor_b xyz,
 *   u32 distance_count, distances[]: u32 type, id_a, id_b,
 *     anchor_a xyz, anchor_b xyz, f32 rest_length,
 *   u32 prismatic_count, prismatics[]: u32 type, id_a, id_b,
 *     anchor_a xyz, anchor_b xyz, axis_a xyz, axis_b xyz,
 *     u32 motor_enabled, f32 target_speed, max_force,
 *     u32 limits_enabled, f32 limit_min, limit_max,
 *   u32 rope_count, ropes[]: u32 type, id_a, id_b,
 *     anchor_a xyz, anchor_b xyz, f32 rest_length,
 *   u32 crc32 (IEEE, over every preceding byte).
 * Older versions (<=153) keep their native-order legacy reader in
 * scene_load.c; the saver only ever writes v200. */

static int save_vec3(FILE *f, uint32_t *crc, vector3 v) {
    return scene_wfloat(f, crc, v.x) && scene_wfloat(f, crc, v.y) && scene_wfloat(f, crc, v.z);
}

static int save_quat(FILE *f, uint32_t *crc, vector4 q) {
    return scene_wfloat(f, crc, q.w) && scene_wfloat(f, crc, q.x) && scene_wfloat(f, crc, q.y) &&
           scene_wfloat(f, crc, q.z);
}

int save_scene(const char *file_destination_path) {
    /* R3-03: Atomic write. mkstemp + 0600 + fsync + rename: no symlink
     * hijack, no partial file on crash. */
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "v200 scene format is little-endian; big-endian hosts need byteswap (unimplemented)"
#endif
    if (!file_destination_path || !*file_destination_path) {
        fprintf(stderr, "Error SVF01: null/empty path\n");
        return 0;
    }
    char tmp_template[520];
    if (snprintf(tmp_template, sizeof(tmp_template), "%s.XXXXXX", file_destination_path) >= (int)sizeof(tmp_template)) {
        fprintf(stderr, "Error SVF01: path too long\n");
        return 0;
    }

    int tmp_fd = mkstemp(tmp_template);
    if (tmp_fd < 0) {
        fprintf(stderr, "Error SVF01: Could not create temp file\n");
        return 0;
    }
    fchmod(tmp_fd, 0600);
    FILE *f = fdopen(tmp_fd, "wb");
    if (!f) {
        close(tmp_fd);
        remove(tmp_template);
        fprintf(stderr, "Error SVF01: Could not open temp file\n");
        return 0;
    }
    uint32_t crc = 0xFFFFFFFFu;
    int ok = 1;
    ok = ok && scene_w32(f, &crc, (uint32_t) mpe_magic);
    ok = ok && scene_w32(f, &crc, (uint32_t) mpe_version);
    ok = ok && scene_w32(f, &crc, (uint32_t) (physics_world_get_primary()->body_count));
    for (int i = 0; ok && (i < (physics_world_get_primary()->body_count)); i++) {
        rigidbody *rb = &(physics_world_get_primary()->bodies)[i];
        ok = ok && scene_w32(f, &crc, (uint32_t) rb->type);
        ok = ok && scene_wfloat(f, &crc, rb->mass);
        ok = ok && scene_wfloat(f, &crc, rb->radius);
        ok = ok && scene_wfloat(f, &crc, rb->cylinder_half_length);
        ok = ok && save_vec3(f, &crc, rb->half_extensions);
        ok = ok && save_vec3(f, &crc, rb->position);
        ok = ok && save_vec3(f, &crc, rb->velocity);
        ok = ok && save_vec3(f, &crc, rb->angular_velocity);
        ok = ok && save_quat(f, &crc, rb->orientation);
        ok = ok && save_vec3(f, &crc, rb->colour);
        ok = ok && scene_wfloat(f, &crc, rb->restitution);
        ok = ok && scene_wfloat(f, &crc, rb->friction_static);
        ok = ok && scene_wfloat(f, &crc, rb->friction_kinetic);
        ok = ok && scene_w32(f, &crc, rb->static_state ? 1u : 0u);
        ok = ok && scene_w32(f, &crc, rb->object_id); /* MPE_FTC_058 */
        ok = ok && scene_w32(f, &crc, (uint32_t) rb->nice_value);
        ok = ok && scene_w32(f, &crc, rb->is_sleeping ? 1u : 0u);
        ok = ok && scene_wfloat(f, &crc, rb->sleep_timer);
        ok = ok && scene_w32(f, &crc, rb->kinematic ? 1u : 0u);
        ok = ok && scene_w32(f, &crc, rb->object_generation);
    }
    /* FIX-AUDIT: scan the FULL pool, not 0..(physics_world_get_primary()->spring_joint_count). Removal
     * leaves holes (active joints above a removed index), which the old
     * bound silently dropped from saves. */
    int active_springs = 0;
    for (int j = 0; j < mpe_max_joints; j++) {
        if ((physics_world_get_primary()->spring_joints)[j].is_active) {
            active_springs++;
        }
    }
    ok = ok && scene_w32(f, &crc, (uint32_t) active_springs);
    for (int j = 0; ok && (j < mpe_max_joints); j++) {
        if (!(physics_world_get_primary()->spring_joints)[j].is_active) {
            continue;
        }
        ok = ok && scene_w32(f, &crc, (physics_world_get_primary()->spring_joints)[j].object_id_a);
        ok = ok && scene_w32(f, &crc, (physics_world_get_primary()->spring_joints)[j].object_id_b);
        ok = ok && scene_wfloat(f, &crc, (physics_world_get_primary()->spring_joints)[j].equilibrium_length);
        ok = ok && scene_wfloat(f, &crc, (physics_world_get_primary()->spring_joints)[j].spring_constant);
        ok = ok && scene_wfloat(f, &crc, (physics_world_get_primary()->spring_joints)[j].damping_coefficient);
    }

    /* Save all constraint types from the unified constraint pool. */
    int constraint_counts[5] = {0}; /* fixed, distance, prismatic, rope (revolute handled separately) */
    for (int j = 0; j < constraint_pool_capacity(); j++) {
        const constraint *c = constraint_pool_at(physics_world_get_primary(), j);
        if ((c) && (c->type != constraint_revolute) && (c->type != constraint_spring)) {
            if (c->type < constraint_fixed || c->type > constraint_rope) {
                continue;
            }
            constraint_counts[c->type - constraint_fixed]++;
        }
    }

    /* Fixed constraints */
    ok = ok && scene_w32(f, &crc, (uint32_t) constraint_counts[constraint_fixed - constraint_fixed]);
    for (int j = 0; ok && (j < constraint_pool_capacity()); j++) {
        const constraint *c = constraint_pool_at(physics_world_get_primary(), j);
        if ((!c) || (c->type != constraint_fixed)) continue;
        ok = ok && scene_w32(f, &crc, (uint32_t) c->type);
        ok = ok && scene_w32(f, &crc, c->body_id_a);
        ok = ok && scene_w32(f, &crc, c->body_id_b);
        ok = ok && save_vec3(f, &crc, c->p.fixed.anchor_a);
        ok = ok && save_vec3(f, &crc, c->p.fixed.anchor_b);
    }

    /* Distance constraints */
    ok = ok && scene_w32(f, &crc, (uint32_t) constraint_counts[constraint_distance - constraint_fixed]);
    for (int j = 0; ok && (j < constraint_pool_capacity()); j++) {
        const constraint *c = constraint_pool_at(physics_world_get_primary(), j);
        if ((!c) || (c->type != constraint_distance)) continue;
        ok = ok && scene_w32(f, &crc, (uint32_t) c->type);
        ok = ok && scene_w32(f, &crc, c->body_id_a);
        ok = ok && scene_w32(f, &crc, c->body_id_b);
        ok = ok && save_vec3(f, &crc, c->p.distance.anchor_a);
        ok = ok && save_vec3(f, &crc, c->p.distance.anchor_b);
        ok = ok && scene_wfloat(f, &crc, c->p.distance.rest_length);
    }

    /* Prismatic constraints */
    ok = ok && scene_w32(f, &crc, (uint32_t) constraint_counts[constraint_prismatic - constraint_fixed]);
    for (int j = 0; ok && (j < constraint_pool_capacity()); j++) {
        const constraint *c = constraint_pool_at(physics_world_get_primary(), j);
        if ((!c) || (c->type != constraint_prismatic)) continue;
        ok = ok && scene_w32(f, &crc, (uint32_t) c->type);
        ok = ok && scene_w32(f, &crc, c->body_id_a);
        ok = ok && scene_w32(f, &crc, c->body_id_b);
        ok = ok && save_vec3(f, &crc, c->p.prismatic.anchor_a);
        ok = ok && save_vec3(f, &crc, c->p.prismatic.anchor_b);
        ok = ok && save_vec3(f, &crc, c->p.prismatic.axis_a);
        ok = ok && save_vec3(f, &crc, c->p.prismatic.axis_b);
        ok = ok && scene_w32(f, &crc, c->p.prismatic.motor_enabled ? 1u : 0u);
        ok = ok && scene_wfloat(f, &crc, c->p.prismatic.motor_target_speed);
        ok = ok && scene_wfloat(f, &crc, c->p.prismatic.motor_max_force);
        ok = ok && scene_w32(f, &crc, c->p.prismatic.limits_enabled ? 1u : 0u);
        ok = ok && scene_wfloat(f, &crc, c->p.prismatic.limit_min);
        ok = ok && scene_wfloat(f, &crc, c->p.prismatic.limit_max);
    }

    /* Rope constraints */
    ok = ok && scene_w32(f, &crc, (uint32_t) constraint_counts[constraint_rope - constraint_fixed]);
    for (int j = 0; ok && (j < constraint_pool_capacity()); j++) {
        const constraint *c = constraint_pool_at(physics_world_get_primary(), j);
        if ((!c) || (c->type != constraint_rope)) continue;
        ok = ok && scene_w32(f, &crc, (uint32_t) c->type);
        ok = ok && scene_w32(f, &crc, c->body_id_a);
        ok = ok && scene_w32(f, &crc, c->body_id_b);
        ok = ok && save_vec3(f, &crc, c->p.rope.anchor_a);
        ok = ok && save_vec3(f, &crc, c->p.rope.anchor_b);
        ok = ok && scene_wfloat(f, &crc, c->p.rope.rest_length);
    }

    /* Revolute constraints (kept for backward compatibility) */
    int active_revolutes = 0;
    for (int j = 0; j < constraint_pool_capacity(); j++) {
        const constraint *c = constraint_pool_at(physics_world_get_primary(), j);
        if ((c) && (c->type == constraint_revolute)) {
            active_revolutes++;
        }
    }
    ok = ok && scene_w32(f, &crc, (uint32_t) active_revolutes);
    for (int j = 0; ok && (j < constraint_pool_capacity()); j++) {
        const constraint *c = constraint_pool_at(physics_world_get_primary(), j);
        if ((!c) || (c->type != constraint_revolute)) {
            continue;
        }
        ok = ok && scene_w32(f, &crc, (uint32_t) c->type);
        ok = ok && scene_w32(f, &crc, c->body_id_a);
        ok = ok && scene_w32(f, &crc, c->body_id_b);
        ok = ok && save_vec3(f, &crc, c->p.revolute.anchor_a);
        ok = ok && save_vec3(f, &crc, c->p.revolute.anchor_b);
        ok = ok && save_vec3(f, &crc, c->p.revolute.axis_a);
        ok = ok && save_vec3(f, &crc, c->p.revolute.axis_b);
        ok = ok && scene_w32(f, &crc, c->p.revolute.motor_enabled ? 1u : 0u);
        ok = ok && scene_wfloat(f, &crc, c->p.revolute.motor_target_speed);
        ok = ok && scene_wfloat(f, &crc, c->p.revolute.motor_max_torque);
        ok = ok && scene_w32(f, &crc, c->p.revolute.limits_enabled ? 1u : 0u);
        ok = ok && scene_wfloat(f, &crc, c->p.revolute.limit_min_rad);
        ok = ok && scene_wfloat(f, &crc, c->p.revolute.limit_max_rad);
    }

    /* Footer CRC over every preceding byte (finalize + raw LE append). */
    uint32_t final_crc = crc ^ 0xFFFFFFFFu;
    if (ok) {
        unsigned char footer[4] = {(unsigned char) (final_crc & 0xFFu),
                                   (unsigned char) ((final_crc >> 8) & 0xFFu),
                                   (unsigned char) ((final_crc >> 16) & 0xFFu),
                                   (unsigned char) ((final_crc >> 24) & 0xFFu)};
        ok = (fwrite(footer, 1, 4, f) == 4);
    }
    /* Fail the save if any buffered write errored. */
    if ((!ok) || ferror(f)) {
        fprintf(stderr, "Error SVF03: Write failure\n");
        fclose(f);
        remove(tmp_template);
        return 0;
    }
    fflush(f);
    int fsync_fd = fileno(f);
    if (fsync_fd >= 0) fsync(fsync_fd);
    fclose(f);
    /* R3-03: Atomic rename over the target */
    if (rename(tmp_template, file_destination_path) != 0) {
        fprintf(stderr, "Error SVF02: Could not rename temp file\n");
        remove(tmp_template);
        return 0;
    }
    return 1;
}
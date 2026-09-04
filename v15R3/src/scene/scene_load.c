#include "../mpe_engine.h"
#include "scene_load.h"
#include "scene_init.h"
#include "scene_id_remap.h" /* MPE_FTC_058 */
#include "../physics/spring_joint.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
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
int scene_loading(const char *file_source_path) {
    FILE *f = fopen(file_source_path, "rb");
    if (!f) {
        fprintf(stderr, "Error LDF01: Could not open %s\n", file_source_path);
        return 0;
    }
    int32_t magic, version, count;
    if ((!read_int(f, &magic)) || (magic != mpe_magic)) {
        fprintf(stderr, "Error LDF02: Invalid magic number\n");
        fclose(f);
        return 0;
    }
    if ((!read_int(f, &version)) || (version != mpe_version && version != 130 && version != 140)) {
        fprintf(stderr, "Error LDF03: Version mismatch\n");
        fclose(f);
        return 0;
    }
    if ((!read_int(f, &count)) || (count < 0)) {
        fclose(f);
        return 0;
    }
    scene_clear();
    scene_id_remap_reset(); /* MPE_FTC_058 */
    contact_cache_clear(NULL); /* A3_PATCH_22_SCENE_LOAD_RESET */
contact_cache_clear(physics_world_get_primary()); /* MFS_131 */
    joint_init_pool();
    if (count > mpe_max_bodies) {
        count = mpe_max_bodies;
    }

    if (!scene_ensure_pool_capacity(count)) {
        fclose(f);
        return 0;
    }

    if (count > object_capacity) {
        count = object_capacity;
    }
    int loaded_count = 0; /* A3_PATCH_42_CRITICAL_LIFECYCLE */
    for (int i = 0; i < count; i++) {
        rigidbody temp;
        int32_t type_int, static_int, saved_object_id = 0; /* MPE_FTC_058 */
        if (!read_int(f, &type_int))
            break;
        if (!read_float(f, &temp.mass))
            break;
        if (!read_float(f, &temp.radius))
            break;
        if (!read_vec3(f, &temp.half_extensions))
            break;
        if (!read_vec3(f, &temp.position))
            break;
        if (!read_vec3(f, &temp.velocity))
            break;
        if (!read_vec3(f, &temp.angular_velocity))
            break;
        if (!read_vec4(f, &temp.orientation))
            break;
        if (!read_vec3(f, &temp.colour))
            break;
        if (!read_float(f, &temp.restitution))
            break;
        if (!read_float(f, &temp.friction_static))
            break;
        if (!read_float(f, &temp.friction_kinetic))
            break;
        if (!read_int(f, &static_int))
            break;
        saved_object_id = 0;
        if (version >= 150) {
            if (!read_int(f, &saved_object_id)) {
                break;
            }
        } /* MPE_FTC_058 */
        temp.type = (object_type) type_int;
        temp.static_state = (static_int != 0);
        if (temp.type == object_cube) {
            rigidbody_initialisation_cube(&obj_per_scene[i], temp.position, temp.half_extensions, temp.mass);
        } else {
            rigidbody_initialisation_sphere(&obj_per_scene[i], temp.radius, temp.mass, temp.position);
        }
        obj_per_scene[i].velocity = temp.velocity;
        obj_per_scene[i].angular_velocity = temp.angular_velocity;
        obj_per_scene[i].orientation = vector4_normalisation(temp.orientation);
        obj_per_scene[i].colour = temp.colour;
        obj_per_scene[i].restitution = temp.restitution;
        obj_per_scene[i].friction_static = temp.friction_static;
        obj_per_scene[i].friction_kinetic = temp.friction_kinetic;
        obj_per_scene[i].static_state = temp.static_state;
        if (obj_per_scene[i].static_state) {
            rigidbody_set_static(&obj_per_scene[i], true);
        } else {
            rigidbody_set_static(&obj_per_scene[i], false);
        }
        rigidbody_sanitize(&obj_per_scene[i]);
        rigidbody_update_axes(&obj_per_scene[i]);
        rigidbody_sanitize(&obj_per_scene[i]); /* A3_PATCH_47_NAN_SANITIZATION */
        obj_per_scene[i].object_id = scene_allocate_object_id();
        if ((version >= 150) && (saved_object_id > 0)) {
            scene_id_remap_add((uint32_t) saved_object_id, obj_per_scene[i].object_id);
        } /* MPE_FTC_058 */
        obj_per_scene[i].object_generation = 1;
        loaded_count++;
    }
    object_count = loaded_count; /* A3_PATCH_42_CRITICAL_LIFECYCLE */
    int32_t active_joints = 0;
    if (read_int(f, &active_joints) && (active_joints > 0)) {
        for (int j = 0; j < active_joints; j++) {
            int32_t id_a, id_b;
            float eq, k, c;
            if (!read_int(f, &id_a))
                break;
            if (!read_int(f, &id_b))
                break;
            if (!read_float(f, &eq))
                break;
            if (!read_float(f, &k))
                break;
            if (!read_float(f, &c))
                break;
            add_joint_by_ids(scene_id_remap_resolve((uint32_t) id_a), scene_id_remap_resolve((uint32_t) id_b), eq, k,
                             c); /* MPE_FTC_058 */
        }
    }
    fclose(f);
    return 1;
}

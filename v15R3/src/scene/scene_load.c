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

int scene_loading(const char *file_source_path)
{
    FILE *f = fopen(file_source_path, "rb");
    if (!f) {
        fprintf(stderr, "Error LDF01: Could not open %s\n", file_source_path);
        return 0;
    }

    /* --- Read and validate header --- */
    int32_t magic, version, count;

    if ((!read_int(f, &magic)) || (magic != mpe_magic)) {
        fprintf(stderr, "Error LDF02: Invalid magic number\n");
        fclose(f);
        return 0;
    }

    if ((!read_int(f, &version)) ||
        (version != mpe_version && version != 130 && version != 140)) {
        fprintf(stderr, "Error LDF03: Version mismatch\n");
        fclose(f);
        return 0;
    }

    if ((!read_int(f, &count)) || (count < 0)) {
        fclose(f);
        return 0;
    }

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

    /* --- Stage all bodies --- */
    int staged_body_count = 0;

    for (int i = 0; i < count; i++) {
        rigidbody temp;
        int32_t type_int, static_int, saved_object_id = 0;

        if (!read_int(f, &type_int))               break;
                if (!read_float(f, &temp.radius))          break;
        /* R3-04: Read cylinder_half_length. Present in version >= 151.
         * For older versions, default to radius/2. */
        if (version >= 151) {
            if (!read_float(f, &temp.cylinder_half_length)) break;
        } else {
            temp.cylinder_half_length = temp.radius * 0.5f;
        }
        if (!read_vec3(f, &temp.half_extensions))  break;
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

        rigidbody_sanitize(&staged_bodies[i]);
        rigidbody_update_axes(&staged_bodies[i]);

        /* Store the saved object ID in a temporary field.
         * We use the nice_value field as a temporary holder since
         * it is not used during staging. We restore it after commit. */
        staged_bodies[i].nice_value = saved_object_id;

        staged_body_count++;
    }

    /* --- Stage all joints --- */
    if (read_int(f, &staged_joint_count) && (staged_joint_count > 0)) {
        staged_joints = (staged_joint *)malloc(
            (size_t)staged_joint_count * sizeof(staged_joint));
        if (!staged_joints) {
            free(staged_bodies);
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
    if (staged_body_count == 0) {
        /* Nothing to commit. Do not clear the scene. */
        free(staged_bodies);
        if (staged_joints) free(staged_joints);
        return 0;
    }

    /* --- Commit: clear scene and install staged data --- */
    scene_clear();
    scene_id_remap_reset();
    contact_cache_clear(NULL);
    contact_cache_clear(physics_world_get_primary());
    joint_init_pool();

    object_count = staged_body_count;

    for (int i = 0; i < staged_body_count; i++) {
        obj_per_scene[i] = staged_bodies[i];

        /* Recover the saved object ID from the temporary holder */
        int32_t saved_id = obj_per_scene[i].nice_value;
        obj_per_scene[i].nice_value = 0; /* Restore nice_value to default */

        obj_per_scene[i].object_id = scene_allocate_object_id();

        if ((version >= 150) && (saved_id > 0)) {
            scene_id_remap_add((uint32_t)saved_id, obj_per_scene[i].object_id);
        }

        obj_per_scene[i].object_generation = 1;
    }

    /* Install staged joints */
    for (int j = 0; j < staged_joint_count; j++) {
        add_joint_by_ids(
            scene_id_remap_resolve(staged_joints[j].id_a),
            scene_id_remap_resolve(staged_joints[j].id_b),
            staged_joints[j].eq,
            staged_joints[j].k,
            staged_joints[j].c);
    }

    free(staged_bodies);
    if (staged_joints) free(staged_joints);

    return 1;
}

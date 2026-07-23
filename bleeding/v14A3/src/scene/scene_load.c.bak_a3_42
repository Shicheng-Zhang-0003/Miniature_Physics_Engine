#include "../mpe_engine.h"
#include "scene_load.h"
#include "scene_init.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#define MPE_MAGIC 0x4D504533
#define MPE_VERSION 140
static int read_float (FILE *f, float *v) { return fread (v, sizeof (float), 1, f) == 1; }
static int read_int (FILE *f, int32_t *v) { return fread (v, sizeof (int32_t), 1, f) == 1; }
static int read_vec3 (FILE *f, vector3 *v) { return fread (v, sizeof (vector3), 1, f) == 1; }
static int read_vec4 (FILE *f, vector4 *v) { return fread (v, sizeof (vector4), 1, f) == 1; }
int scene_loading (const char *file_source_path) {
    FILE *f = fopen (file_source_path, "rb");
    if (!f) { fprintf (stderr, "Error LDF01: Could not open %s\n", file_source_path); return 0; }
    int32_t magic, version, count;
    if ((!read_int (f, &magic)) || (magic != MPE_MAGIC)) { fprintf (stderr, "Error LDF02: Invalid magic number\n"); fclose (f); return 0; }
    if ((!read_int (f, &version)) || (version != MPE_VERSION && version != 130)) { fprintf (stderr, "Error LDF03: Version mismatch\n"); fclose (f); return 0; }
    if ((!read_int (f, &count)) || (count < 0)) { fclose (f); return 0; }
    scene_clear ();
    contact_cache_clear (); /* A3_PATCH_22_SCENE_LOAD_RESET */
    /* A3_PATCH_23_POOL_CONSISTENCY */
if (count > MPE_MAX_BODIES) {count = MPE_MAX_BODIES;}

if (!scene_ensure_pool_capacity (count)) {
    fclose (f);
    return 0;
}

if (count > object_capacity) {count = object_capacity;} for (int i = 0; i < count; i++) {
        rigidbody temp;
        int32_t type_int, static_int;
        if (!read_int (f, &type_int)) break;
        if (!read_float (f, &temp.mass)) break;
        if (!read_float (f, &temp.radius)) break;
        if (!read_vec3 (f, &temp.half_extensions)) break;
        if (!read_vec3 (f, &temp.position)) break;
        if (!read_vec3 (f, &temp.velocity)) break;
        if (!read_vec3 (f, &temp.angular_velocity)) break;
        if (!read_vec4 (f, &temp.orientation)) break;
        if (!read_vec3 (f, &temp.colour)) break;
        if (!read_float (f, &temp.restitution)) break;
        if (!read_float (f, &temp.friction_static)) break;
        if (!read_float (f, &temp.friction_kinetic)) break;
        if (!read_int (f, &static_int)) break;
        temp.type = (object_type) type_int;
        temp.static_state = (static_int != 0);
        if (temp.type == object_cube) {rigidbody_initialisation_cube (&obj_per_scene [i], temp.position, temp.half_extensions, temp.mass);}
        else {rigidbody_initialisation_sphere (&obj_per_scene [i], temp.radius, temp.mass, temp.position);}
        obj_per_scene [i].velocity = temp.velocity;
        obj_per_scene [i].angular_velocity = temp.angular_velocity;
        obj_per_scene [i].orientation = vector4_normalisation (temp.orientation);
        obj_per_scene [i].colour = temp.colour;
        obj_per_scene [i].restitution = temp.restitution;
        obj_per_scene [i].friction_static = temp.friction_static;
        obj_per_scene [i].friction_kinetic = temp.friction_kinetic;
        obj_per_scene [i].static_state = temp.static_state;
        if (obj_per_scene [i].static_state) { obj_per_scene [i].inverse_mass = 0.0f; }
        rigidbody_update_axes (&obj_per_scene [i]);
        /* A3_PATCH_06_STABLE_IDS_LOAD */
        obj_per_scene [i].object_id = scene_allocate_object_id ();
        obj_per_scene [i].object_generation = 1;
    } object_count = count;
    fclose (f);
    return 1;
}

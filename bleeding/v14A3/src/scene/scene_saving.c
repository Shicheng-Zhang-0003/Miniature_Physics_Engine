#include "scene_saving.h"
#include <stdio.h>
#include <stdint.h>
#define MPE_MAGIC 0x4D504533
#define MPE_VERSION 140
static void write_float (FILE *f, float v) { fwrite (&v, sizeof (float), 1, f); }
static void write_int (FILE *f, int32_t v) { fwrite (&v, sizeof (int32_t), 1, f); }
static void write_vec3 (FILE *f, vector3 v) { fwrite (&v, sizeof (vector3), 1, f); }
static void write_vec4 (FILE *f, vector4 v) { fwrite (&v, sizeof (vector4), 1, f); }
int save_scene (const char *file_destination_path) {
    FILE *f = fopen (file_destination_path, "wb");
    if (!f) { fprintf (stderr, "Error SVF01: Could not open %s\n", file_destination_path); return 0; }
    write_int (f, MPE_MAGIC);
    write_int (f, MPE_VERSION);
    write_int (f, object_count);
    for (int i = 0; i < object_count; i++) {
        rigidbody *rb = &obj_per_scene [i];
        write_int (f, (int32_t) rb -> type);
        write_float (f, rb -> mass);
        write_float (f, rb -> radius);
        write_vec3 (f, rb -> half_extensions);
        write_vec3 (f, rb -> position);
        write_vec3 (f, rb -> velocity);
        write_vec3 (f, rb -> angular_velocity);
        write_vec4 (f, rb -> orientation);
        write_vec3 (f, rb -> colour);
        write_float (f, rb -> restitution);
        write_float (f, rb -> friction_static);
        write_float (f, rb -> friction_kinetic);
        write_int (f, rb -> static_state ? 1 : 0);
    } fclose (f);
    return 1;
}

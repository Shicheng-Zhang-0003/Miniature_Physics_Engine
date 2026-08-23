#include "../mpe_engine.h"
#include "scene_saving.h"
#include "../physics/spring_joint.h"
#include <stdio.h>
#include <stdint.h>
static void write_float (FILE *f, float v) { fwrite (&v, sizeof (float), 1, f); }
static void write_int (FILE *f, int32_t v) { fwrite (&v, sizeof (int32_t), 1, f); }
static void write_vec3 (FILE *f, vector3 v) { fwrite (&v, sizeof (vector3), 1, f); }
static void write_vec4 (FILE *f, vector4 v) { fwrite (&v, sizeof (vector4), 1, f); }
int save_scene (const char *file_destination_path) {
    FILE *f = fopen (file_destination_path, "wb");
    if (!f) { fprintf (stderr, "Error SVF01: Could not open %s\n", file_destination_path); return 0; }
    write_int (f, mpe_magic);
    write_int (f, mpe_version);
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
    }
    int active_joints = 0;
    for (int j = 0; j < current_joint_count; j++) {
        if (joint_pool [j].is_active) { active_joints++; }
    }
    write_int (f, active_joints);
    for (int j = 0; j < current_joint_count; j++) {
        if (joint_pool [j].is_active) {
            write_int (f, (int32_t) joint_pool [j].object_id_a);
            write_int (f, (int32_t) joint_pool [j].object_id_b);
            write_float (f, joint_pool [j].equilibrium_length);
            write_float (f, joint_pool [j].spring_constant);
            write_float (f, joint_pool [j].damping_coefficient);
        }
    }
    fclose (f);
    return 1;
}

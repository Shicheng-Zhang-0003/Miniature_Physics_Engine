#include "scene_load.h"
#include "../../stage3/scene/scene_init.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define MPE_BINARY_MAGIC 0x4D504532
#define MPE_BINARY_VERSION 120

typedef struct {
    uint32_t magic;
    uint32_t version;
    int32_t object_count;
} scene_header;

int scene_loading (const char *file_source_path) {
    FILE *file_source = fopen (file_source_path, "rb");
    if (!file_source) {fprintf (stderr, "Error LDF01: Loading File Error %s\n", file_source_path); return 0;}

    scene_header header;
    if (fread (&header, sizeof (scene_header), 1, file_source) != 1) {fclose (file_source); return 0;}

    if (header.magic != MPE_BINARY_MAGIC || header.version != MPE_BINARY_VERSION) {
        fprintf (stderr, "Error LDF02: Invalid binary format or version\n");
        fclose (file_source);
        return 0;
    }

    int scene_object_count = header.object_count;
    if (scene_object_count < 0) {fclose (file_source); return 0;}

    rigidbody *loaded_scene_objects = NULL;
    if (scene_object_count > 0) {
        loaded_scene_objects = malloc (scene_object_count * sizeof (rigidbody));
        if (!loaded_scene_objects) {fclose (file_source); return 0;}
        if (fread (loaded_scene_objects, sizeof (rigidbody), scene_object_count, file_source) != (size_t) scene_object_count) {
            free (loaded_scene_objects);
            fclose (file_source);
            return 0;
        }
    }
    fclose (file_source);

    scene_clear ();
    if (scene_object_count > object_capacity) {
        rigidbody *new_object_array = realloc (obj_per_scene, scene_object_count * sizeof (rigidbody));
        if (!new_object_array) {free (loaded_scene_objects); return 0;}
        obj_per_scene = new_object_array;
        object_capacity = scene_object_count;
    }

    for (int object_index = 0; object_index < scene_object_count; object_index++) {
        obj_per_scene [object_index] = loaded_scene_objects [object_index];
        obj_per_scene [object_index].orientation = vector4_normalisation (obj_per_scene [object_index].orientation);
        rigidbody_update_axes (&obj_per_scene [object_index]);
    }
    object_count = scene_object_count;
    free (loaded_scene_objects);
    return 1;
}

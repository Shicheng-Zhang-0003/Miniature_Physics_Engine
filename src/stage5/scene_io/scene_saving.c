#include "scene_saving.h"
#include <stdio.h>
#include <stdint.h>
#define MPE_BINARY_MAGIC 0x4D504532
#define MPE_BINARY_VERSION 120
typedef struct {
    uint32_t magic;
    uint32_t version;
    int32_t object_count;
} scene_header;
int save_scene (const char *file_destination_path) {
    FILE *file_destination = fopen (file_destination_path, "wb");
    if (!file_destination) {fprintf (stderr, "Error SVF01, could not open %s\n", file_destination_path); return 0;}
    scene_header header;
    header.magic = MPE_BINARY_MAGIC;
    header.version = MPE_BINARY_VERSION;
    header.object_count = object_count;
    fwrite (&header, sizeof (scene_header), 1, file_destination);
    for (int object_index = 0; object_index < object_count; object_index++) {
        rigidbody *rb = &obj_per_scene [object_index];
        fwrite (rb, sizeof (rigidbody), 1, file_destination);
    } fclose (file_destination);
    return 1;
}

#ifndef scene_saving_h
#define scene_saving_h
#include "../core/math3d.h"
#include "../core/rigidbody.h"

/* Returns 1 on full success, 2 on success with a parent-directory-sync
 * warning (FIX-AUDIT-DESPOT: scene bytes durable, dir entry not — callers
 * must treat nonzero as saved), 0 on failure (live scene untouched, no
 * file published). */
int save_scene(const char *file_destination_path);
#endif

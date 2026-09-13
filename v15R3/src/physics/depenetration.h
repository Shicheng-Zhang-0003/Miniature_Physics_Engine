#ifndef mfs_depenetration_h
#define mfs_depenetration_h

#include <stdbool.h>
#include "broadphase.h"
#include "collision_mechanics.h"

/* MFS_PHASE_A: positional depenetration, extracted from simulation.c.
 * Shared helpers declared here for physics_world path. */

bool a3_depenetration_dispatch(rigidbody *rigid_body_a, rigidbody *rigid_body_b,
                               collision_data *collision_output);
void a3_positional_depenetrate_manifold(collision_data *manifold);
/* Legacy-path entry point, moved here from simulation.c so headless
 * harnesses can link the legacy step without the GTK application TU.
 * Takes the body array explicitly (no globals) so this TU stays
 * link-clean for world-only binaries. */
void a3_positional_depenetration_pass(rigidbody *bodies, int body_count, broadphase_pair *pair_buffer,
                                      int *pair_count_pointer, bool rebuild_broadphase);

#endif

#ifndef mpe_depenetration_world_h
#define mpe_depenetration_world_h

#include "../core/physics_world.h"
#include "broadphase.h"

/* MPE_PHASE1_DEPENETRATION_WORLD:
 * Pure physics_world depenetration pass.
 * Operates entirely on the provided physics_world and broadphase pairs.
 * Eliminates the hardcoded y=0 floor plane pass (the floor is just a static body).
 */
void physics_world_depenetration_pass(physics_world *world, broadphase_pair *pairs, int pair_count);

#endif /* mpe_depenetration_world_h */

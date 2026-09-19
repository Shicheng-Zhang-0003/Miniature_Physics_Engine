/* Application-owned primary world (old-series kernel global, relocated).
 *
 * The kernel (physics_world.c) holds NO simulation state: every step
 * takes an explicit physics_world*. The single primary instance a GUI /
 * editor process needs lives HERE, in an app-support TU — the same
 * role as UE's GWorld. physics_world_get_primary() is the compatibility
 * accessor for the scene/UI/render layers; headless code should declare
 * its own worlds instead.
 *
 * This TU is part of MPE_TEST_CORE so scene/spring test binaries (which
 * exercise the primary-backed scene layer) keep linking. */
#include "physics_world.h"

static physics_world mpe_primary_world = {.bodies = NULL,
                                          .body_count = 0,
                                          .body_capacity = 0,
                                          .next_object_id = 1};

physics_world *physics_world_get_primary(void) {
    return &mpe_primary_world;
}

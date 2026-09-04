#include "../mpe_engine.h"
#include "define_forces.h"
/* MPE_TASK_042: Legacy force applicants removed.
 * gravity     -> rb_apply_forces_perfect() in simulation.c
 * friction    -> impulse solver in collision_mechanics.c
 * springs     -> spring_joint.c
 * All other force models (universal gravity, rolling friction,
 * string tension, vertical anchor) were never called. */

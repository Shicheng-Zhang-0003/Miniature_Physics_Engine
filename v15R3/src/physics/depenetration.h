#ifndef mfs_depenetration_h
#define mfs_depenetration_h

#include <stdbool.h>
#include "broadphase.h"

/* MFS_PHASE_A: positional depenetration, extracted from simulation.c. */

void a3_positional_depenetration_pass(broadphase_pair *pair_buffer, int *pair_count_pointer, bool rebuild_broadphase);

#endif

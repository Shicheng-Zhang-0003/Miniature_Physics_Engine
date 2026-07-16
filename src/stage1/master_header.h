#ifndef master_h
#define master_h

#define MPE_MAX_BODIES 16384
#define MPE_MAX_JOINTS 1024

//master header file for calling all library files
#include "math/math3D.h" //Math
#include "math/math4_special.h" //Math
#include "math_phys_buffer/buffer.h" //Buffer
#include "physics/collision_mechanics/collision_mechanics.h" //Collision Mechanics
#include "physics/forces/defines_majority_forces/define_forces.h" //Physics (Forces and Energy)

#endif

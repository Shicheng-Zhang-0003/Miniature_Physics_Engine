#ifndef physics_world_h
#define physics_world_h

#include "../core/math3D.h"
#include "../core/math4_special.h"
#include "../core/buffer.h"
#include "collision_mechanics.h"
#include "broadphase.h"
#include "spring_joint.h"

typedef struct {
    float gravity_y;
    float drag_coefficient;
    float friction_static;
    float friction_kinetic;
    int solver_iterations;
    float fixed_dt;
    float time_accumulator;
    int max_substeps;
} physics_world;

extern physics_world main_physics_world;

void physics_world_init (physics_world *world);

#endif // physics_world_h

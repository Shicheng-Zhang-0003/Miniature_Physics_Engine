#include "physics_world.h"

physics_world main_physics_world;

void physics_world_init (physics_world *world) {
    world -> gravity_y = -9.81f;
    world -> drag_coefficient = 0.99f;
    world -> friction_static = 0.2f;
    world -> friction_kinetic = 0.1f;
    world -> solver_iterations = 16;
    world -> fixed_dt = 1.0f / 60.0f;
    world -> time_accumulator = 0.0f;
    world -> max_substeps = 5;
}

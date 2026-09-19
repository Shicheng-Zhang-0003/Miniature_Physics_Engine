#ifndef spring_joint_h
#define spring_joint_h
#include "../core/math3d.h"
#include "../core/math4_special.h"
#include "../core/rigidbody.h"
#include "../core/physics_world.h"
#include "spring_joint_types.h"

#include <stdint.h>
#include <epoxy/gl.h>

/* Spring pool lives in physics_world (per-world state). The functions below
 * take the owning world explicitly; legacy single-world callers pass the
 * primary world. No file-scope pool remains. */
void joint_init_pool(physics_world *world);
int spring_joint_count(const physics_world *world);
int add_joint_by_ids(physics_world *world, uint32_t object_id_a, uint32_t object_id_b, float equilibrium_length,
                     float spring_constant, float damping_coefficient);
int add_joint(physics_world *world, int object_index_a, int object_index_b, float equilibrium_length,
              float spring_constant, float damping_coefficient);
void remove_joint(physics_world *world, int joint_pool_index);
void remove_joints_from_object(physics_world *world, int object_index);
void remove_joints_from_object_id(physics_world *world, uint32_t object_id);
void apply_force_all_joints(physics_world *world);
void apply_force_all_joints_dt(physics_world *world, float dt);
/* World-aware spring pass over an explicit body array (headless + world
 * step path). Reads the pool of the given world. */
void apply_spring_forces_world(physics_world *world, rigidbody *bodies, int body_count);
void apply_spring_forces_world_dt(physics_world *world, rigidbody *bodies, int body_count, float dt);
/* Canonical per-tick spring entry (see spring_joint_types.h). */
void mpe_springs_apply(physics_world *world, float dt);
void spring_joint_render(GLuint shader_program, math4 view_matrix, math4 projection_matrix);
#endif

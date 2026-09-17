/* Spring-joint storage types (GL-free).
 * Split from spring_joint.h so physics_world.h can own the joint pool
 * without pulling epoxy/OpenGL headers into every translation unit.
 * Behaviour and layout unchanged. */
#ifndef spring_joint_types_h
#define spring_joint_types_h

#include <stdint.h>
#include <stdbool.h>
#include "../core/rigidbody.h" /* clean (math only); gives rigidbody. physics_world stays forward-declared. */

#ifndef mpe_max_joints
#define mpe_max_joints 1024
#endif

typedef struct {
    uint32_t object_id_a, object_id_b; /* A3_PATCH_09_JOINT_IDS */
    float equilibrium_length;
    float spring_constant;
    float damping_coefficient;
    bool is_active;
} spring_joint;

/* GTK4-PREP: spring pool entry points live here (GL-free) so core, scene,
 * tests, and the TUI can use springs without epoxy/OpenGL headers.
 * spring_joint.h keeps only the GL render declaration. */
typedef struct physics_world physics_world;
void joint_init_pool(physics_world *world);
int spring_joint_count(const physics_world *world);
int add_joint_by_ids(physics_world *world, uint32_t object_id_a, uint32_t object_id_b,
                     float equilibrium_length, float spring_constant, float damping_coefficient);
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

#endif /* spring_joint_types_h */

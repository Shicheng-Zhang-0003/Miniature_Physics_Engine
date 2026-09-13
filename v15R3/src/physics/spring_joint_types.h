/* Spring-joint storage types (GL-free).
 * Split from spring_joint.h so physics_world.h can own the joint pool
 * without pulling epoxy/OpenGL headers into every translation unit.
 * Behaviour and layout unchanged. */
#ifndef spring_joint_types_h
#define spring_joint_types_h

#include <stdint.h>
#include <stdbool.h>

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

#endif /* spring_joint_types_h */

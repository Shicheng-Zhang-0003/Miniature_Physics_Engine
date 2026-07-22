#ifndef spring_joint_h
#define spring_joint_h
#include "../core/math3D.h"
#include "../core/math4_special.h"
#include "../core/buffer.h"
#include <epoxy/gl.h>

typedef struct {
    int object_index_a, object_index_b;
    float equilibrium_length;
    float spring_constant;
    float damping_coefficient;
    bool is_active;
} spring_joint;

#define maximum_joint_count 32
extern spring_joint joint_pool [maximum_joint_count];
extern int current_joint_count;

int add_joint (int object_index_a, int object_index_b, float equilibrium_length, float spring_constant, float damping_coefficient);
void remove_joint (int joint_pool_index);
void apply_force_all_joints (void);
void remove_joints_from_object (int object_index);
void adjust_joints_after_deletion (int deleted_object_index);
void spring_joint_render (GLuint shader_program, math4 view_matrix, math4 projection_matrix);
void joint_init_pool (void);
#endif

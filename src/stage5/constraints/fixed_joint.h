#ifndef fixed_joint_h
#define fixed_joint_h
#include "../../stage1/master_header.h"
#include "../../stage2/master_header_2.h"
typedef struct {
    int object_index_a, object_index_b;
    vector3 anchor_point;
    bool is_active;
} fixed_joint;
#define maximum_fixed_joint_count 32
extern fixed_joint fixed_joint_pool [maximum_fixed_joint_count];
extern int current_fixed_joint_count;
int add_fixed_joint (int object_index_a, int object_index_b, vector3 anchor);
void remove_fixed_joint (int joint_pool_index);
void solve_all_fixed_joints (float delta_time);
void remove_fixed_joints_from_object (int object_index);
void fixed_joint_init_pool (void);
#endif

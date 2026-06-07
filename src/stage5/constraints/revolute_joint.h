#ifndef revolute_joint_h
#define revolute_joint_h
#include "../../stage1/master_header.h"
#include "../../stage2/master_header_2.h"
typedef struct {
    int object_index_a, object_index_b;
    vector3 anchor_point;
    vector3 axis;
    float current_angle;
    float lower_limit, upper_limit;
    float motor_speed;
    float max_motor_torque;
    bool use_motor;
    bool use_limits;
    bool is_active;
} revolute_joint;
#define maximum_revolute_joint_count 32
extern revolute_joint revolute_joint_pool [maximum_revolute_joint_count];
extern int current_revolute_joint_count;
int add_revolute_joint (int object_index_a, int object_index_b, vector3 anchor, vector3 axis);
void remove_revolute_joint (int joint_pool_index);
void solve_all_revolute_joints (float delta_time);
void remove_revolute_joints_from_object (int object_index);
void revolute_joint_init_pool (void);
#endif

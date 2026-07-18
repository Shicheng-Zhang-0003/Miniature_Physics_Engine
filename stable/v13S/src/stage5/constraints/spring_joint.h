#ifndef spring_joint_h
#define spring_joint_h
#include "../../stage1/master_header.h"
#include "../../stage2/master_header_2.h"
typedef struct {
    int object_index_a, object_index_b; //index of the objects to be connected and analysed
    float equilibrium_length; //length of the spring at rest
    float spring_constant; //Fs = -kx, k constant
    float damping_coefficient; //Loss of energy, no infinite oscillations
    bool is_active; //In use? Being acted by a foreign force?
} spring_joint;
#define maximum_joint_count 32
//Pool of joint values
extern spring_joint joint_pool [maximum_joint_count];
extern int current_joint_count;
//Add a particular joint between two objects, returns either the actual joint index of -1 is none is found
int add_joint (int object_index_a, int object_index_b, float equilibrium_length, float spring_constant, float damping_coefficient);
//Remove a particular joint
void remove_joint (int joint_pool_index);
//Apply the spring force for active joints connected
void apply_force_all_joints (void);
//Remove all joints connected to a particular object
void remove_joints_from_object (int object_index);
//Initialise the actual pool of objects to be processed
void joint_init_pool (void);
#endif

#ifndef spring_joint_h
#define spring_joint_h
#include "../../stage1/master_header.h"
#include "../../stage2/master_header_2.h"
typedef struct {
    int index_av, index_bv; //index of the objects to be connected and analysed
    float spring_length_rest; //length of the spring at rest
    float k_constant; //Fs = -kx, k constant
    float dampening; //Loss of energy, no infinite oscillations
    bool activation; //In use? Being acted by a foreign force?
} spring_joint_mechanism;
#define max_joint_count 32
//Pool of joint values
extern spring_joint_mechanism joint_pool [max_joint_count];
extern int joint_count;
//Add a particular joint between two objects, returns either the actual joint index of -1 is none is found
int add_joint (int index_av, int index_bv, float spring_length_rest, float k_constant, float dampening);
//Remove a particular joint
void remove_joint (int joint_index);
//Apply the spring force for active joints connected
void apply_force_all_joints (void);
//Remove all joints connected to a particular object
void remove_joints_from_object (int object_index);
//Initialise the actual pool of objects to be processed
void joint_init_pool (void);
#endif

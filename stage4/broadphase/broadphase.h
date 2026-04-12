#ifndef broadphase_h
#define broadphase_h
#include "../../stage1/master_header.h"
#include "../../stage2/master_header_2.h"
//Object Indices that has passed braodphase
typedef struct {
    int a, b;
} broadphase_pair;
//This eventually fills output_pairs, spheres whose boudnaries are close enough, which triggers the collision check in stage1 definition
int broadphase_generate_pairing (broadphase_pair *output_pairs, int maximum_pairs);
#endif

#ifndef constraint_interface_h
#define constraint_interface_h

#include "../../stage1/master_header.h"

typedef enum {
    CONSTRAINT_SPRING,
    CONSTRAINT_DISTANCE,
    CONSTRAINT_HINGE
} ConstraintType;

typedef struct {
    ConstraintType type;
    int body_a_index;
    int body_b_index;
    bool is_active;

    // Union allows us to store specific parameters without bloating memory
    union {
        struct { float eq_length; float k; float damping; } spring;
        struct { float fixed_length; } distance;
    } data;
} Constraint;

extern Constraint g_constraint_pool [MPE_MAX_JOINTS];
extern int g_active_constraint_count;

// The solver will iterate this single array, regardless of joint type
void constraints_apply_forces (void);

#endif

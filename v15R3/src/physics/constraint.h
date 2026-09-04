/* MPE_FTC_060 header, updated MPE_FTC_063 */
#ifndef constraint_h
#define constraint_h
#include "../core/rigidbody.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CONSTRAINT_SPRING,
    CONSTRAINT_REVOLUTE,
    CONSTRAINT_FIXED,
    CONSTRAINT_PRISMATIC,
    CONSTRAINT_DISTANCE
} constraint_type;

typedef struct {
    vector3 anchor_a;
    vector3 anchor_b;
    vector3 axis_a;
    float motor_target_speed;
    float motor_max_torque;
    float limit_min_rad;
    float limit_max_rad;
    bool motor_enabled;
    bool limits_enabled;
} revolute_params;

typedef struct {
    constraint_type type;
    uint32_t body_id_a;
    uint32_t body_id_b;
    bool is_active;
    union {
        revolute_params revolute;
    } p;
} constraint;

void constraint_pool_init (void);
int  constraint_add_revolute (uint32_t id_a, uint32_t id_b, vector3 anchor_a, vector3 anchor_b, vector3 axis_a);
void constraint_remove (int index);
int  constraint_get_count (void);
void constraint_set_revolute_motor (int index, bool enabled, float target_speed, float max_torque);
/* Positional/axis solve — call once per tick. */
void constraint_solve_all (rigidbody *bodies, int body_count, float dt);
/* Motor drive — call once per tick before velocity integration. */
void constraint_apply_motors (rigidbody *bodies, int body_count, float dt);
#endif

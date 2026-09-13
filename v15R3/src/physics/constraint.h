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
    vector3 axis_b; /* FIX-AUDIT: per-body hinge axis. Zero = fall back to axis_a (legacy callers). */
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
/* Scene persistence support (v200): per-body hinge axis, joint limits,
 * and read-only pool access. Setters validate index/type and no-op
 * otherwise, so loading corrupt entries cannot destabilize live joints. */
void constraint_set_revolute_axes (int index, vector3 axis_a, vector3 axis_b);
void constraint_set_revolute_limits (int index, bool enabled, float limit_min_rad, float limit_max_rad);
int constraint_pool_capacity (void);
const constraint *constraint_pool_at (int index);
/* Solve joints — call once per solver iteration inside solver loop. */
void constraint_solve_all (rigidbody *bodies, int body_count, float dt);
/* Active joint endpoint ids (for island building). Returns count. */
int constraint_get_active_ids (uint32_t *ids_a, uint32_t *ids_b, int capacity);
/* Axis-drift correction — call exactly ONCE per tick AFTER the solver loop.
 * Never inside it: the positional error is iteration-invariant, so inner
 * calls multiply the correction by the iteration count (energy pump). */
void constraint_correct_axis_drift_all (rigidbody *bodies, int body_count, float dt);
/* Motor drive — call once per tick before velocity integration. */
void constraint_apply_motors (rigidbody *bodies, int body_count, float dt);
#endif

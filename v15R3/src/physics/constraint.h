/* MPE_FTC_060 header, updated MPE_FTC_063.
 * Per-world pools: every function takes the owning world explicitly (or,
 * for solvers, derives bodies from it). No file-scope pool remains; the
 * legacy global pool is retired. */
#ifndef constraint_h
#define constraint_h
#include "../core/rigidbody.h"
#include <stdint.h>
#include <stdbool.h>
struct physics_world;

typedef enum {
    constraint_spring,
    constraint_revolute,
    constraint_fixed,
    constraint_prismatic,
    constraint_distance
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

struct physics_world;
void constraint_pool_init (struct physics_world *world);
int  constraint_add_revolute (struct physics_world *world, uint32_t id_a, uint32_t id_b, vector3 anchor_a,
                              vector3 anchor_b, vector3 axis_a);
void constraint_remove (struct physics_world *world, int index);
int  constraint_get_count (const struct physics_world *world);
void constraint_set_revolute_motor (struct physics_world *world, int index, bool enabled, float target_speed,
                                    float max_torque);
/* Scene persistence support (v200): per-body hinge axis, joint limits,
 * and read-only pool access. Setters validate index/type and no-op
 * otherwise, so loading corrupt entries cannot destabilize live joints. */
void constraint_set_revolute_axes (struct physics_world *world, int index, vector3 axis_a, vector3 axis_b);
void constraint_set_revolute_limits (struct physics_world *world, int index, bool enabled, float limit_min_rad,
                                     float limit_max_rad);
int constraint_pool_capacity (void);
const constraint *constraint_pool_at (const struct physics_world *world, int index);
/* Solve joints — call once per solver iteration inside solver loop. */
void constraint_solve_all (struct physics_world *world, float dt);
/* Active joint endpoint ids (for island building). Returns count. */
int constraint_get_active_ids (const struct physics_world *world, uint32_t *ids_a, uint32_t *ids_b, int capacity);
/* Motor drive — call once per tick before velocity integration. */
void constraint_apply_motors (struct physics_world *world, float dt);
/* Axis-drift correction — call exactly ONCE per tick AFTER the solver loop.
 * Never inside it: the positional error is iteration-invariant, so inner
 * calls multiply the correction by the iteration count (energy pump). */
void constraint_correct_axis_drift_all (struct physics_world *world, float dt);
#endif

#ifndef rigidbody_h
#define rigidbody_h

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "math3d.h"
#include "math4_special.h"
#include "../config/mpe_config.h"
typedef enum { object_sphere, object_cube, object_cylinder, object_custom } object_type; /* MPE_FTC_090 + modular custom */
enum { OBJECT_SPHERE = object_sphere, OBJECT_CUBE = object_cube, OBJECT_CYLINDER = object_cylinder, OBJECT_CUSTOM = object_custom };
typedef struct {
    //Linear Kinematics
    vector3 position, velocity, acceleration;
    //Rotational Motion
    vector4 orientation;
    vector3 angular_velocity, angular_acceleration;
    //Dynamics (Properties)
    float mass, inverse_mass, restitution;
    //Inertial Tensor
    math3 inertia_tensor_local, inverse_inertia_tensor_local, inverse_inertia_system;
    //Force and Torque accumulation
    vector3 force_accumulator, torque_accumulator;
    //Dimensions
    float radius;
    float cylinder_half_length; /* MPE_FTC_090: half-length along axle (X) */
    bool static_state;
    float friction_static, friction_kinetic;
    vector3 colour;
    object_type type;
    //Cube Specific Variables
    vector3 half_extensions;
    vector3 cached_axes[3];
    /* MPE_TASK_15_AXIS_CACHE_FIELD_BEGIN */
    vector4 cached_axes_orientation;
    /* MPE_TASK_15_AXIS_CACHE_FIELD_END */
    //v1.2 Sleeping Bodies
    bool is_sleeping;
    float sleep_timer;
    /* Kinematic bodies: infinite mass like static, but move with a
     * user-prescribed velocity (conveyors, platforms). Forces never alter
     * them; contacts treat them as immovable with velocity. */
    bool kinematic;
    /* MPE_TASK_V15R2_NICE_VALUE_BEGIN */
    int nice_value; /* NON-PHYSICAL settle tool (0=truth, off). Positive = extra
                     * numerical damping for rapid game settling, NOT fluid
                     * mechanics. Curve: factor=(1-0.002*nice)^(dt*60).
                     * nice=19 halves every ~18 ticks: intentionally strong for
                     * gameplay; use 0 for physics truth. Negative clamps to 0
                     * (no anti-damping/energy injection, ever). */
    /* MPE_TASK_V15R2_NICE_VALUE_END */
    uint32_t object_id;
    uint32_t object_generation;
    /* Modular custom shape: valid only when type==object_custom.
     * Foreign plugins claim an id (>=100) via mpe_register_pair_handler;
     * core treats the body as a bounding-sphere for broadphase/CCD
     * until the plugin overrides those stages. */
    int custom_shape;
    /* Index in physics_world->bodies array (set on add, updated on remove).
     * Eliminates pointer arithmetic for O(1) body lookup in island building. */
    int body_index;
    /* Maximum relative velocity squared at contacts (for sleep check).
     * Updated during contact processing; used for relative-velocity sleep gating. */
    float max_relative_speed_sq;
    /* Render-only / proxy bodies: excluded from broadphase pairing,
     * narrowphase dispatch, floor contact and CCD (render + dumps still
     * see them). Append-only ABI extension: old plugins ignore it.
     * Default false; set by visual-proxy layers (gui_robot_registry). */
    bool no_collide;
} rigidbody;
void rigidbody_update_axes(rigidbody *rigid_body);
void rigidbody_initialisation_sphere(rigidbody *rigid_body, float radius, float mass, vector3 position_input);
void rigidbody_update_inertia_sphere(rigidbody *rigid_body);
void rigidbody_update_inertia_cube(rigidbody *rigid_body);
void rigidbody_initialisation_cylinder(rigidbody *rigid_body, float radius, float half_length, float mass, vector3 position_input); /* MPE_FTC_090 */
void rigidbody_update_inertia_cylinder(rigidbody *rigid_body); /* MPE_FTC_090 */
void rb_apply_forces_perfect(rigidbody *rigid_body, vector3 force_applied);
void rb_apply_forces_localised(rigidbody *rigid_body, vector3 force_applied, vector3 locale_impact);
float rb_get_kinetic_energy(rigidbody *rigid_body);
/* make_half_extents REMOVED (trivial helper, zero callers). */
void rigidbody_initialisation_cube(rigidbody *rigid_body, vector3 position_input, vector3 half_extensions, float mass);
void rigidbody_wake(rigidbody *rigid_body);

void rigidbody_sanitize(rigidbody *rigid_body);
void rigidbody_set_static(rigidbody *rigid_body, bool make_static);
void rigidbody_set_kinematic(rigidbody *rigid_body, bool make_kinematic);

void rb_integrate_velocity(rigidbody *rigid_body, float delta_time, float linear_damping, float angular_damping);
void rb_integrate_position(rigidbody *rigid_body, float delta_time);
void rb_integrate_position_exact(rigidbody *rigid_body, float delta_time, const mpe_config_t *cfg, bool free_flight);
/* rb_integrate_position_free_flight_original REMOVED (was dead, invited double-counts). */

/* Effective-mass helpers: sleeping/static/kinematic bodies behave as infinite
 * mass WITHOUT mutating stored inverse_mass/inertia. Use these in the solver,
 * split impulse, and depenetration instead of zeroing the stored fields
 * (the old staticize hack corrupted observable state mid-tick and blocked
 * multithreading). */
static inline float rigidbody_effective_inv_mass(const rigidbody *rb) {
    if (!rb) {
        return 0.0f;
    }
    if (rb->static_state || rb->is_sleeping || rb->kinematic) {
        return 0.0f;
    }
    return rb->inverse_mass;
}

static inline bool rigidbody_is_awake_for_solver(const rigidbody *rb) {
    if (!rb) {
        return false;
    }
    if (rb->static_state) {
        return false;
    }
    /* TRUTH: kinematic has eff_inv==0 (infinite mass) but still moves with
     * prescribed velocity. It is "awake" for island purposes (floor stacks
     * on moving platforms must stay awake) while solver treats it as
     * immovable via effective_* helpers. Sleeping is the only solver-skip. */
    return !rb->is_sleeping;
}

/* Effective inverse inertia: zero matrix for infinite-mass bodies, otherwise
 * the stored world-space inverse. Avoids mutating the stored field. */
static inline math3 rigidbody_effective_inv_inertia(const rigidbody *rb) {
    math3 zero = {{{0.0f}}};
    if (!rb) {
        return zero;
    }
    if (rb->static_state || rb->is_sleeping || rb->kinematic) {
        return zero;
    }
    return rb->inverse_inertia_system;
}
#endif
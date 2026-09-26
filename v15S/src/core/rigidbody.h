#ifndef rigidbody_h
#define rigidbody_h

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "math3d.h"
#include "math4_special.h"
#include "../config/mpe_config.h"
/* FIX-AUDIT-DESPOT float-precision TRUTH: position/velocity/mass/dimensions
 * are float (23-bit mantissa, ~1e-7 relative). Consequences, documented so
 * callers stop filing them as bugs:
 * - Far-field ulp: at |x|=250m one ulp is ~3e-5m (30um); at 500m ~6e-5m.
 *   Resting-contact slop (default 1cm) still dwarfs this inside the ±250m
 *   playable volume, but sub-mm stacking truth beyond ~250m is unrepresentable.
 * - double accumulation is used internally where cancellation bites
 *   (orientation length, cell-size averages); the struct boundary stays
 *   float by design (deterministic layout, cache footprint). If far-field
 *   accuracy is ever required, promote STORAGE to double — no local cast
 *   can recover bits the struct never held.
 * Inertia formulas (verified, do not "fix"): sphere (2/5)mr² solid;
 * box (m/12)(h²+d²) per axis; cylinder axle-X ½mr² axial,
 * (m/12)(3r²+l²) transverse. */
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
    /* ---- Anisotropic friction (append-only; see collision_mechanics.h) ----
     * Default false => the contact cone is the legacy isotropic DISC and
     * these fields are ignored entirely, so every pre-existing body is
     * bit-for-bit unaffected.
     *
     * When true, the contact's Coulomb cone becomes an ELLIPSE with
     * semi-axis friction_along_axis along friction_anisotropy_axis, and
     * friction_across_axis perpendicular to it (in the contact plane).
     *
     * Mecanum wheels are the motivating case, and the direction is easy to
     * get backwards, so: the rollers let the wheel translate ALONG the
     * roller-spin direction (perpendicular to the roller axis) with the
     * rollers spinning, while the contact can only push ALONG the roller
     * axis. So the roller axis is the GRIPPED direction and its in-plane
     * perpendicular is the FREE one. Point friction_anisotropy_axis at the
     * free direction and give it the LOW coefficient; the gripped roller
     * axis then inherits friction_across_axis (the HIGH one). With the
     * mirrored roller pattern across the four wheels, chassis translation
     * emerges from the contacts instead of being injected at the CoM.
     *
     * friction_anisotropy_axis is BODY-LOCAL and fixed in the body. It does
     * NOT spin with the wheel about its own axle, because the roller axis is
     * perpendicular to that axle. Sanitised to unit length.
     *
     * Anisotropy is strictly SUBTRACTIVE: the ellipse lies inside the disc
     * when both coefficients are <= the isotropic mu, so enabling it can
     * only reduce grip in the chosen direction. It cannot add energy or
     * destabilise a body that was stable without it. */
    bool friction_anisotropic;
    float friction_along_axis;     /* Coulomb coefficient along friction_anisotropy_axis */
    float friction_across_axis;    /* Coulomb coefficient perpendicular to it, in-plane */
    vector3 friction_anisotropy_axis; /* body-local unit axis (the FREE / roller-spin direction) */
    /* Object id of the body whose frame interprets friction_anisotropy_axis.
     * 0 means "this body" (the default, and what a tyre wants: its material
     * frame spins with the tyre).
     *
     * A driven wheel is the case where the two differ. A mecanum/omni wheel's
     * roller axes are fixed to the WHEEL MOUNT, not to the spinning hub, so a
     * hub-local axis would sweep around as the wheel turns and the contact
     * would have no fixed rail at all. Pointing the frame at the chassis keeps
     * the rail where the hardware keeps it. */
    uint32_t friction_anisotropy_frame;
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
/* Enable/disable anisotropic friction. axis_local is body-local and is
 * normalised here (a zero-length axis is rejected and leaves the body
 * isotropic). Coeffs are clamped to [0, 5] like friction_static/_kinetic;
 * non-finite input is rejected. Disabling restores the isotropic disc and
 * zeroes the coefficients so no stale ellipse can be read back. */
void rigidbody_set_friction_anisotropic(rigidbody *rigid_body, vector3 axis_local, float mu_roll, float mu_lateral);
/* Same, but friction_anisotropy_axis is interpreted in `frame_id`'s frame
 * instead of this body's. Pass 0 to keep the body-local default. */
void rigidbody_set_friction_anisotropic_in_frame(rigidbody *rigid_body, uint32_t frame_id,
                                                  vector3 axis_local, float mu_roll, float mu_lateral);
void rigidbody_clear_friction_anisotropic(rigidbody *rigid_body);

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
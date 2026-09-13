#include "../mpe_engine.h"
#include "rigidbody.h"
#include "det_math.h" /* deterministic damping rotor factors */
// Helper to update axes from orientation
static bool a3_vector3_is_finite(vector3 v) {
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static bool a3_vector4_is_finite(vector4 v) {
    return isfinite(v.w) && isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

static bool a3_math3_is_finite(math3 m) {
    for (int row_index = 0; row_index < 3; row_index++) {
        for (int column_index = 0; column_index < 3; column_index++) {
            if (!isfinite(m.matrix[row_index][column_index])) {
                return false;
            }
        }
    }
    return true;
}

static bool a3_math3_is_zero(math3 m) {
    for (int row_index = 0; row_index < 3; row_index++) {
        for (int column_index = 0; column_index < 3; column_index++) {
            if (m.matrix[row_index][column_index] != 0.0f) {
                return false;
            }
        }
    }
    return true;
}

void rigidbody_sanitize(rigidbody *rigid_body) {
    bool needs_inertia_recalc = false;
    math3 zero_matrix = {{{0.0f}}};

    if (!a3_vector3_is_finite(rigid_body->position)) {
        rigid_body->position = (vector3){0.0f, 5.0f, 0.0f};
        rigid_body->velocity = vector3_zero();
        rigid_body->angular_velocity = vector3_zero();
        rigid_body->orientation = vector4_identity();
        rigid_body->is_sleeping = false;
        rigid_body->sleep_timer = 0.0f;
        needs_inertia_recalc = true;
    }

    if (!a3_vector3_is_finite(rigid_body->velocity)) {
        rigid_body->velocity = vector3_zero();
    }

    if (!a3_vector3_is_finite(rigid_body->angular_velocity)) {
        rigid_body->angular_velocity = vector3_zero();
    }

    /* FIX-AUDIT: sanitize force/torque accumulators + accelerations. A NaN
     * here otherwise survives velocity zeroing and re-injects NaN via
     * accel=F*invM on the next integrate. */
    if (!a3_vector3_is_finite(rigid_body->force_accumulator)) {
        rigid_body->force_accumulator = vector3_zero();
    }
    if (!a3_vector3_is_finite(rigid_body->torque_accumulator)) {
        rigid_body->torque_accumulator = vector3_zero();
    }
    if (!a3_vector3_is_finite(rigid_body->acceleration)) {
        rigid_body->acceleration = vector3_zero();
    }
    if (!a3_vector3_is_finite(rigid_body->angular_acceleration)) {
        rigid_body->angular_acceleration = vector3_zero();
    }

    if (!a3_vector4_is_finite(rigid_body->orientation)) {
        rigid_body->orientation = vector4_identity();
        needs_inertia_recalc = true;
    } else {
        float orientation_length_squared = rigid_body->orientation.w * rigid_body->orientation.w +
                                           rigid_body->orientation.x * rigid_body->orientation.x +
                                           rigid_body->orientation.y * rigid_body->orientation.y +
                                           rigid_body->orientation.z * rigid_body->orientation.z;

        /* FIX-AUDIT: vector4_to_math3 assumes a unit quaternion. The old
         * [0.25,4.0] deadband let |q| up to 2.0 through, scaling R by |q|^2
         * and I_world by |q|^4. Normalize on any meaningful drift. */
        if (!isfinite(orientation_length_squared) || (fabsf(orientation_length_squared - 1.0f) > 1e-6f)) {
            if (orientation_length_squared > 1e-12f) {
                rigid_body->orientation = vector4_normalisation(rigid_body->orientation);
            } else {
                rigid_body->orientation = vector4_identity();
            }
            needs_inertia_recalc = true;
        }
    }

    if (!isfinite(rigid_body->mass) || (rigid_body->mass < 0.0f)) {
        rigid_body->mass = rigid_body->static_state ? 0.0f : 1.0f;
        needs_inertia_recalc = true;
    }

    if (rigid_body->type == object_sphere) {
        if (!isfinite(rigid_body->radius) || (rigid_body->radius <= 0.0f)) {
            rigid_body->radius = 0.01f;
            needs_inertia_recalc = true;
        }
        /* AUDIT: keep bounding half-extents synced (see initialisation). */
        rigid_body->half_extensions =
            (vector3){rigid_body->radius, rigid_body->radius, rigid_body->radius};
    } else if (rigid_body->type == object_cylinder) { /* R3-001 */
        if (!isfinite(rigid_body->radius) || (rigid_body->radius <= 0.0f)) {
            rigid_body->radius = 0.01f;
            needs_inertia_recalc = true;
        }
        if (!isfinite(rigid_body->cylinder_half_length) || (rigid_body->cylinder_half_length <= 0.0f)) {
            rigid_body->cylinder_half_length = 0.01f;
            needs_inertia_recalc = true;
        }
        /* AUDIT: axle is local X (see initialisation). */
        rigid_body->half_extensions = (vector3){rigid_body->cylinder_half_length, rigid_body->radius,
                                               rigid_body->radius};
    } else {
        if (!isfinite(rigid_body->half_extensions.x) || (rigid_body->half_extensions.x <= 0.0f)) {
            rigid_body->half_extensions.x = 0.01f;
            needs_inertia_recalc = true;
        }
        if (!isfinite(rigid_body->half_extensions.y) || (rigid_body->half_extensions.y <= 0.0f)) {
            rigid_body->half_extensions.y = 0.01f;
            needs_inertia_recalc = true;
        }
        if (!isfinite(rigid_body->half_extensions.z) || (rigid_body->half_extensions.z <= 0.0f)) {
            rigid_body->half_extensions.z = 0.01f;
            needs_inertia_recalc = true;
        }
    }

    if (!isfinite(rigid_body->friction_static) || (rigid_body->friction_static < 0.0f)) {
        rigid_body->friction_static = 0.3f;
    }

    if (!isfinite(rigid_body->friction_kinetic) || (rigid_body->friction_kinetic < 0.0f)) {
        rigid_body->friction_kinetic = 0.2f;
    }

    if (!isfinite(rigid_body->restitution) || (rigid_body->restitution < 0.0f)) {
        rigid_body->restitution = 0.0f;
    }

    if (rigid_body->restitution > 1.0f) {
        rigid_body->restitution = 1.0f;
    }

    if (rigid_body->static_state) {
        rigid_body->kinematic = false;
        rigid_body->inverse_mass = 0.0f;
        rigid_body->inverse_inertia_tensor_local = zero_matrix;
        rigid_body->inverse_inertia_system = zero_matrix;
        rigid_body->velocity = vector3_zero();
        rigid_body->angular_velocity = vector3_zero();
        rigid_body->is_sleeping = false;
        rigid_body->sleep_timer = 0.0f;
    } else if (rigid_body->kinematic) {
        /* Kinematic: infinite mass, but velocity is prescribed and kept.
         * Never sleeps; forces never integrate. */
        rigid_body->inverse_mass = 0.0f;
        rigid_body->inverse_inertia_tensor_local = zero_matrix;
        rigid_body->inverse_inertia_system = zero_matrix;
        rigid_body->is_sleeping = false;
        rigid_body->sleep_timer = 0.0f;
        if (!a3_vector3_is_finite(rigid_body->velocity)) {
            rigid_body->velocity = vector3_zero();
        }
        if (!a3_vector3_is_finite(rigid_body->angular_velocity)) {
            rigid_body->angular_velocity = vector3_zero();
        }
    } else {
        if ((rigid_body->mass <= 0.0f) || (!isfinite(rigid_body->mass))) {
            rigid_body->mass = 1.0f;
            needs_inertia_recalc = true;
        }

        rigid_body->inverse_mass = 1.0f / rigid_body->mass;

        if (needs_inertia_recalc || (!a3_math3_is_finite(rigid_body->inverse_inertia_tensor_local)) ||
            (!a3_math3_is_finite(rigid_body->inverse_inertia_system)) ||
            (a3_math3_is_zero(rigid_body->inverse_inertia_system))) {
            if (rigid_body->type == object_sphere) {
                rigidbody_update_inertia_sphere(rigid_body);
            } else if (rigid_body->type == object_cylinder) { /* MPE_FTC_093d */
                rigidbody_update_inertia_cylinder(rigid_body);
            } else {
                rigidbody_update_inertia_cube(rigid_body);
            }
        }
    }

    /* MPE_TASK_15_SANITIZE_AXIS_CACHE_BEGIN */
    if ((!a3_vector3_is_finite(rigid_body->cached_axes[0])) || (!a3_vector3_is_finite(rigid_body->cached_axes[1])) ||
        (!a3_vector3_is_finite(rigid_body->cached_axes[2])) ||
        (!a3_vector4_is_finite(rigid_body->cached_axes_orientation)) ||
        (fabsf(rigid_body->cached_axes_orientation.w - rigid_body->orientation.w) > 0.000001f) ||
        (fabsf(rigid_body->cached_axes_orientation.x - rigid_body->orientation.x) > 0.000001f) ||
        (fabsf(rigid_body->cached_axes_orientation.y - rigid_body->orientation.y) > 0.000001f) ||
        (fabsf(rigid_body->cached_axes_orientation.z - rigid_body->orientation.z) > 0.000001f)) {
        rigidbody_update_axes(rigid_body);
    }
    /* MPE_TASK_15_SANITIZE_AXIS_CACHE_END */
}
void rigidbody_update_axes(rigidbody *rigid_body) {
    math3 rotation_matrix = vector4_to_math3(rigid_body->orientation);
    rigid_body->cached_axes[0] =
        (vector3){rotation_matrix.matrix[0][0], rotation_matrix.matrix[1][0], rotation_matrix.matrix[2][0]};
    rigid_body->cached_axes[1] =
        (vector3){rotation_matrix.matrix[0][1], rotation_matrix.matrix[1][1], rotation_matrix.matrix[2][1]};
    rigid_body->cached_axes[2] =
        (vector3){rotation_matrix.matrix[0][2], rotation_matrix.matrix[1][2], rotation_matrix.matrix[2][2]};
    /* MPE_TASK_15_AXIS_STAMP_BEGIN */
    rigid_body->cached_axes_orientation = rigid_body->orientation;
    /* MPE_TASK_15_AXIS_STAMP_END */
} //Init
void rigidbody_initialisation_sphere(rigidbody *rigid_body, float radius, float mass, vector3 position_input) {
    //Kinematic
    rigid_body->position = position_input;
    rigid_body->velocity = vector3_zero();
    rigid_body->acceleration = vector3_zero();
    rigid_body->orientation = vector4_identity();
    rigid_body->angular_velocity = vector3_zero();
    rigid_body->angular_acceleration = vector3_zero();
    rigid_body->colour = (vector3){0.2f, 0.6f, 1.0f};
    rigid_body->type = object_sphere;
    rigidbody_update_axes(rigid_body);
    //Dynamic
    rigid_body->mass = mass;
    if (mass > 0) {
        rigid_body->inverse_mass = 1.0f / mass;
    } else {
        rigid_body->inverse_mass = 0.0f;
    }
    rigid_body->radius = radius;
    /* AUDIT: half_extensions must never be indeterminate (malloc'd bodies).
     * Spheres carry their radius on all axes so OBB-style readers
     * (boundary box, save/load) see the true bounding box. */
    rigid_body->half_extensions = (vector3){radius, radius, radius};
    rigid_body->restitution = g_cfg.body_defaults.sphere_restitution; /* MPE_TASK_32 */
    rigid_body->static_state = (mass == 0);
    rigid_body->is_sleeping = false;
    rigid_body->sleep_timer = 0.0f; //Static Objects
    rigid_body->nice_value = 0; /* MPE_TASK_V15R2_NICE_INIT */
    rigid_body->kinematic = false;
    /* FIX-AUDIT: use config body defaults (were hardcoded, registry dead). */
    rigid_body->friction_static = g_cfg.body_defaults.sphere_fric_s;
    rigid_body->friction_kinetic = g_cfg.body_defaults.sphere_fric_k;
    //Inertial Tensors
    //I = 0.4fmr ^ 2
    float inertia_coefficient_sphere = (0.4f) * mass * radius * radius;
    rigid_body->inertia_tensor_local = (math3){{{0}}};
    rigid_body->inertia_tensor_local.matrix[0][0] = inertia_coefficient_sphere;
    rigid_body->inertia_tensor_local.matrix[1][1] = inertia_coefficient_sphere;
    rigid_body->inertia_tensor_local.matrix[2][2] = inertia_coefficient_sphere;
    //Initialize Inverse Inertia System
    if (mass > 0) {
        rigid_body->inverse_inertia_tensor_local = math3_inverse(rigid_body->inertia_tensor_local);
        rigid_body->inverse_inertia_system = rigid_body->inverse_inertia_tensor_local;
    } else {
        rigid_body->inverse_inertia_tensor_local = (math3){{{0}}};
        rigid_body->inverse_inertia_system = (math3){{{0}}};
    } //Total Force and Torque accumulation
    rigid_body->force_accumulator = vector3_zero();
    rigid_body->torque_accumulator = vector3_zero();
} // Helper to update inertia tensor after mass/radius change
void rigidbody_update_inertia_sphere(rigidbody *rigid_body) {
    float inertia_coefficient_sphere = (0.4f) * rigid_body->mass * rigid_body->radius * rigid_body->radius;
    rigid_body->inertia_tensor_local = (math3){{{0}}};
    rigid_body->inertia_tensor_local.matrix[0][0] = inertia_coefficient_sphere;
    rigid_body->inertia_tensor_local.matrix[1][1] = inertia_coefficient_sphere;
    rigid_body->inertia_tensor_local.matrix[2][2] = inertia_coefficient_sphere;
    if (rigid_body->mass > 0) {
        rigid_body->inverse_inertia_tensor_local = math3_inverse(rigid_body->inertia_tensor_local);
        rigid_body->inverse_inertia_system = rigid_body->inverse_inertia_tensor_local;
    } else {
        rigid_body->inverse_inertia_tensor_local = (math3){{{0}}};
        rigid_body->inverse_inertia_system = (math3){{{0}}};
    }
} // Helper to update inertia tensor after mass/radius change
void rigidbody_update_inertia_cube(rigidbody *rigid_body) {
    float width = rigid_body->half_extensions.x * 2.0f;
    float height = rigid_body->half_extensions.y * 2.0f;
    float depth = rigid_body->half_extensions.z * 2.0f;
    float mass = rigid_body->mass;
    rigid_body->inertia_tensor_local = (math3){{{0}}};
    rigid_body->inertia_tensor_local.matrix[0][0] = (mass / 12.0f) * (height * height + depth * depth);
    rigid_body->inertia_tensor_local.matrix[1][1] = (mass / 12.0f) * (width * width + depth * depth);
    rigid_body->inertia_tensor_local.matrix[2][2] = (mass / 12.0f) * (width * width + height * height);
    if (mass > 0) {
        rigid_body->inverse_inertia_tensor_local = math3_inverse(rigid_body->inertia_tensor_local);
        rigid_body->inverse_inertia_system = rigid_body->inverse_inertia_tensor_local;
    } else {
        rigid_body->inverse_inertia_tensor_local = (math3){{{0}}};
        rigid_body->inverse_inertia_system = (math3){{{0}}};
    }
} //Force application and Torque Dynamics
//Apply a force at a centre of mass (perfect collision movement, linear movement only defined)
void rb_apply_forces_perfect(rigidbody *rigid_body, vector3 force_applied) {
    if (rigid_body->static_state) {
        return;
    }
    /* MPE_TASK_13_2_FORCE_SLEEP_FIX_BEGIN */
    if ((rigid_body->is_sleeping) && (vector3_length_squared(force_applied) > 0.000001f)) {
        rigid_body->is_sleeping = false;
        rigid_body->sleep_timer = 0.0f;
    }
    /* MPE_TASK_13_2_FORCE_SLEEP_FIX_END */
    rigid_body->force_accumulator =
        vector3_addition(rigid_body->force_accumulator, force_applied); //Force applied to torque and circular momentum
} //Apply force at a point not the centre of mass (which generates rotational motion and torque)
//locale_impact = impact point on object identified
void rb_apply_forces_localised(rigidbody *rigid_body, vector3 force_applied, vector3 locale_impact) {
    if (rigid_body->static_state) {
        return;
    }
    /* MPE_TASK_13_2_LOCALIZED_SLEEP_FIX_BEGIN */
    if ((rigid_body->is_sleeping) && (vector3_length_squared(force_applied) > 0.000001f)) {
        rigid_body->is_sleeping = false;
        rigid_body->sleep_timer = 0.0f;
    }
    /* MPE_TASK_13_2_LOCALIZED_SLEEP_FIX_END */
    rb_apply_forces_perfect(rigid_body, force_applied);
    //Torque = r * F (r = vector from Centre of Mass to the point of actual contact between objects)
    vector3 relative_contact_vector = vector3_subtraction(locale_impact, rigid_body->position);
    vector3 torque_generated = vector3_cross(relative_contact_vector, force_applied);
    rigid_body->torque_accumulator = vector3_addition(rigid_body->torque_accumulator, torque_generated);
} //Energy Computation
float rb_get_kinetic_energy(rigidbody *rigid_body) {
    //EK normal = 0.5fmv ^ 2
    float linear_kinetic_energy = 0.5f * rigid_body->mass * vector3_length_squared(rigid_body->velocity);
    //EK rotational = 0.5fwIw
    /* MFS_166_INERTIA_FIX: use inertia_tensor_local rotated to world space
* instead of inverting the already-inverted inverse_inertia_system */
math3 a3_ke_rotation = vector4_to_math3(rigid_body->orientation);
math3 a3_ke_rotation_t = math3_transposition(a3_ke_rotation);
math3 a3_ke_world_inertia = math3_multiplication(a3_ke_rotation,
math3_multiplication(rigid_body->inertia_tensor_local, a3_ke_rotation_t));
vector3 angular_momemtum =
math3_multiplication_vector3(a3_ke_world_inertia, rigid_body->angular_velocity);
    float rotational_kinetic_energy = 0.5f * vector3_dot(rigid_body->angular_velocity, angular_momemtum);
    return linear_kinetic_energy + rotational_kinetic_energy;
} //Integration Segmentation (Movement Compute)
void rb_integrate_velocity(rigidbody *rigid_body, float delta_time, float linear_damping, float angular_damping) {
    /* FIX-AUDIT: old early return banked force/torque into static/sleeping
     * accumulators (-> inf/NaN, kick on wake). Always drain first. */
    if ((rigid_body->static_state) || (delta_time <= 0.0f) || (rigid_body->is_sleeping)) {
        rigid_body->force_accumulator = vector3_zero();
        rigid_body->torque_accumulator = vector3_zero();
        return;
    }
    if (rigid_body->kinematic) {
        /* Prescribed velocity: forces drain unapplied, velocity untouched. */
        rigid_body->force_accumulator = vector3_zero();
        rigid_body->torque_accumulator = vector3_zero();
        return;
    }

    math3 rotation_matrix_current = vector4_to_math3(rigid_body->orientation);
    math3 rotation_matrix_transposed = math3_transposition(rotation_matrix_current);
    rigid_body->inverse_inertia_system =
        math3_multiplication(rotation_matrix_current, math3_multiplication(rigid_body->inverse_inertia_tensor_local,
                                                                           rotation_matrix_transposed));

    rigid_body->acceleration = vector3_scaling(rigid_body->force_accumulator, rigid_body->inverse_mass);
    rigid_body->velocity =
        vector3_addition(rigid_body->velocity, vector3_scaling(rigid_body->acceleration, delta_time));
    rigid_body->velocity = vector3_scaling(rigid_body->velocity, linear_damping);
/* FIX-AUDIT: nice damping was per-tick (frame-rate dependent). Make it
      * per-second-anchored: factor semantics preserved at 60Hz via
      * det_pow_retention(base, dt*60) (bit-deterministic, see det_math.h).
      * Nice value >= 0 only: positive = extra damping, negative clamped to 0.
      * No anti-damping (energy injection) allowed. */
    if (rigid_body->nice_value > 0) {
        float nice_base = 1.0f - 0.002f * (float) rigid_body->nice_value;
        if (nice_base < 0.9f) {
            nice_base = 0.9f;
        }
        if (nice_base > 1.0f) {
            nice_base = 1.0f;
        }
        float nice_factor =
            (float) det_pow_retention((double) nice_base, (double) delta_time * 60.0);
        rigid_body->velocity = vector3_scaling(rigid_body->velocity, nice_factor);
    }
    /* MPE_TASK_V15R2_NICE_DAMPING_END */

    /* AUDIT: no snap-to-zero dead zone here. Zeroing sub-threshold
     * velocities is unphysical (it destroys slow creep, micro-pendulum
     * motion, and rolling decay tails) and redundant: the sleep system
     * owns macroscopic rest, Coulomb friction owns stick. Any jitter that
     * this snap used to hide is a solver-truth issue to fix at the
     * source, not to mask with a velocity guillotine. */

    rigid_body->angular_acceleration = math3_multiplication_vector3(rigid_body->inverse_inertia_system, rigid_body->torque_accumulator);

    /* LIST4 NEW-16: Gyroscopic torque.
 *
 * Without this, torque-free bodies conserve world-space angular velocity
 * instead of angular momentum. That suppresses gyroscopic precession and
 * allows rotational kinetic energy to drift for tumbling non-spherical bodies.
 *
 * Euler's equation requires:
 *
 *   tau_gyro = -omega x (I_world * omega)
 *
 * We compute it in local space because inertia_tensor_local is constant
 * and usually diagonal for primitive shapes.
 * The max_angular_speed clamp later in this function bounds extreme cases.
 */
    {
        math3 list4_gyro_rotation = vector4_to_math3(rigid_body->orientation);
        math3 list4_gyro_rotation_t = math3_transposition(list4_gyro_rotation);

        vector3 list4_gyro_omega_local =
        math3_multiplication_vector3(list4_gyro_rotation_t, rigid_body->angular_velocity);

        vector3 list4_gyro_angular_momentum_local =
        math3_multiplication_vector3(rigid_body->inertia_tensor_local, list4_gyro_omega_local);

        vector3 list4_gyro_torque_local =
        vector3_cross(list4_gyro_omega_local, list4_gyro_angular_momentum_local);

        list4_gyro_torque_local = vector3_scaling(list4_gyro_torque_local, -1.0f);

        vector3 list4_gyro_torque_world =
        math3_multiplication_vector3(list4_gyro_rotation, list4_gyro_torque_local);

        vector3 list4_gyro_alpha =
        math3_multiplication_vector3(rigid_body->inverse_inertia_system, list4_gyro_torque_world);

        rigid_body->angular_acceleration =
            vector3_addition(rigid_body->angular_acceleration, list4_gyro_alpha);
    }

    rigid_body->angular_velocity =
        vector3_addition(rigid_body->angular_velocity, vector3_scaling(rigid_body->angular_acceleration, delta_time));
    rigid_body->angular_velocity = vector3_scaling(rigid_body->angular_velocity, angular_damping);

    /* AUDIT: no angular snap either (see above). */

    float max_linear_speed = g_cfg.timestep.max_linear_speed; /* MPE_TASK_30 */
    float current_speed_sq = vector3_length_squared(rigid_body->velocity);

    if (current_speed_sq > max_linear_speed * max_linear_speed) {
        rigid_body->velocity = vector3_scaling(vector3_normalisation(rigid_body->velocity), max_linear_speed);
    }

    float max_angular_speed = g_cfg.timestep.max_angular_speed; /* MPE_TASK_30 */
    float current_angular_speed_sq = vector3_length_squared(rigid_body->angular_velocity);

    if (current_angular_speed_sq > max_angular_speed * max_angular_speed) {
        rigid_body->angular_velocity =
            vector3_scaling(vector3_normalisation(rigid_body->angular_velocity), max_angular_speed);
    }

    rigid_body->force_accumulator = vector3_zero();
    rigid_body->torque_accumulator = vector3_zero();
}

void rb_integrate_position(rigidbody *rigid_body, float delta_time) {
    if ((rigid_body->static_state) || (delta_time <= 0.0f)) {
        return;
    }
    if (rigid_body->is_sleeping) {
        return;
    }

    rigid_body->position = vector3_addition(rigid_body->position, vector3_scaling(rigid_body->velocity, delta_time));

    /* Exact exponential-map rotation: q' = normalize(dq(w,|w|dt) * q).
     * First-order Euler (q += 0.5*dt*w*q) accumulates orientation phase
     * error for fast spinners; the closed-form rotor is exact for constant
     * w over the tick and unconditionally stable. Frame order (world-frame
     * left multiplication) matches the previous scheme. */
    float spin_rate = vector3_length(rigid_body->angular_velocity);
    if (spin_rate > 0.000001f) {
        /* Deterministic rotor: fixed-coefficient trig (see det_math.h).
         * Falls back to libm only for absurd rate*dt products. */
        double half_angle = 0.5 * (double) spin_rate * (double) delta_time;
        vector4 spin_rotor;
        if (half_angle > -0.5 && half_angle < 0.5) {
            double s = det_sin_small(half_angle);
            double c = det_cos_small(half_angle);
            double inv = 1.0 / (double) spin_rate;
            spin_rotor = (vector4){(float) c, (float) (rigid_body->angular_velocity.x * inv * s),
                                  (float) (rigid_body->angular_velocity.y * inv * s),
                                  (float) (rigid_body->angular_velocity.z * inv * s)};
        } else {
            spin_rotor = vector4_from_axis_with_angle(
                vector3_scaling(rigid_body->angular_velocity, 1.0f / spin_rate), spin_rate * delta_time);
        }
        rigid_body->orientation = vector4_normalisation(vector4_multiplication(spin_rotor, rigid_body->orientation));
    } else {
        rigid_body->orientation = vector4_normalisation(rigid_body->orientation);
    }
    rigidbody_update_axes(rigid_body);

    float speed_sq = vector3_length_squared(rigid_body->velocity);
    float angular_speed_sq = vector3_length_squared(rigid_body->angular_velocity);

    if (rigid_body->kinematic) {
        rigid_body->sleep_timer = 0.0f;
    } else if ((speed_sq < g_cfg.sleep.linear_thresh_sq) && (angular_speed_sq < g_cfg.sleep.angular_thresh_sq)) {
        rigid_body->sleep_timer += delta_time;
        if (rigid_body->sleep_timer > g_cfg.sleep.timer_duration) {
            rigid_body->is_sleeping = true;
        }
    } else {
        rigid_body->sleep_timer = 0.0f;
    }
}

vector3 make_half_extents(float width, float height, float depth) {
    return (vector3){width * 0.5f, height * 0.5f, depth * 0.5f};
}
// Initialize a cube: Box, OBB
void rigidbody_initialisation_cube(rigidbody *rigid_body, vector3 position_input, vector3 half_extensions, float mass) {
    //Kinematic
    rigid_body->position = position_input;
    rigid_body->velocity = vector3_zero();
    rigid_body->acceleration = vector3_zero();
    rigid_body->orientation = vector4_identity();
    rigid_body->angular_velocity = vector3_zero();
    rigid_body->angular_acceleration = vector3_zero();
    rigid_body->colour = (vector3){1.0f, 0.4f, 0.2f}; // Orange-ish for cubes
    rigid_body->type = object_cube;
    rigidbody_update_axes(rigid_body);
    //Dynamic
    rigid_body->mass = mass;
    if (mass > 0) {
        rigid_body->inverse_mass = 1.0f / mass;
    } else {
        rigid_body->inverse_mass = 0.0f;
    }
    rigid_body->half_extensions = half_extensions;
    rigid_body->radius = vector3_length(half_extensions); // Bounding radius for broadphase
    rigid_body->restitution = g_cfg.body_defaults.cube_restitution; /* MPE_TASK_32 */
    rigid_body->static_state = (mass == 0);
    rigid_body->is_sleeping = false;
    rigid_body->sleep_timer = 0.0f;
    rigid_body->nice_value = 0; /* MPE_TASK_V15R2_NICE_INIT */
    rigid_body->kinematic = false;
    /* FIX-AUDIT: use config body defaults (were hardcoded). */
    rigid_body->friction_static = g_cfg.body_defaults.cube_fric_s;
    rigid_body->friction_kinetic = g_cfg.body_defaults.cube_fric_k;
    //Inertia Tensor for rectangular box: I = (m/12) * (h² + d², w² + d², w² + h²), nominal extension only for boxes
    float width = half_extensions.x * 2.0f; //full width
    float height = half_extensions.y * 2.0f; //full height
    float depth = half_extensions.z * 2.0f; //full depth
    rigid_body->inertia_tensor_local = (math3){{{0}}};
    rigid_body->inertia_tensor_local.matrix[0][0] = (mass / 12.0f) * (height * height + depth * depth);
    rigid_body->inertia_tensor_local.matrix[1][1] = (mass / 12.0f) * (width * width + depth * depth);
    rigid_body->inertia_tensor_local.matrix[2][2] = (mass / 12.0f) * (width * width + height * height);
    if (mass > 0) {
        rigid_body->inverse_inertia_tensor_local = math3_inverse(rigid_body->inertia_tensor_local);
        rigid_body->inverse_inertia_system = rigid_body->inverse_inertia_tensor_local;
    } else {
        rigid_body->inverse_inertia_tensor_local = (math3){{{0}}};
        rigid_body->inverse_inertia_system = (math3){{{0}}};
    } // Force & Torque accumulators
    rigid_body->force_accumulator = vector3_zero();
    rigid_body->torque_accumulator = vector3_zero();
}
void rigidbody_wake(rigidbody *rigid_body) {
    if (rigid_body->is_sleeping) {
        rigid_body->is_sleeping = false;
        rigid_body->sleep_timer = 0.0f;
    }
    /* FIX-AUDIT: legacy sleep staticizes inverses to 0. A body woken
     * mid-narrowphase would otherwise solve one tick with infinite mass.
     * Restore real inverses on wake (no-op for static bodies). */
    if (!rigid_body->static_state && rigid_body->mass > 0.0f && isfinite(rigid_body->mass)) {
        rigid_body->inverse_mass = 1.0f / rigid_body->mass;
        math3 rot = vector4_to_math3(rigid_body->orientation);
        math3 rot_t = math3_transposition(rot);
        if (a3_math3_is_finite(rigid_body->inverse_inertia_tensor_local) &&
            !a3_math3_is_zero(rigid_body->inverse_inertia_tensor_local)) {
            rigid_body->inverse_inertia_system = math3_multiplication(
                rot, math3_multiplication(rigid_body->inverse_inertia_tensor_local, rot_t));
        }
    }
}

void rigidbody_set_static(rigidbody *rigid_body, bool make_static) {
    math3 zero_matrix = {{{0.0f}}};

    rigid_body->static_state = make_static;
    /* MPE_TASK_07_ACCUMULATOR_CLEAR_BEGIN */
    rigid_body->force_accumulator = vector3_zero();
    rigid_body->torque_accumulator = vector3_zero();
    /* MPE_TASK_07_ACCUMULATOR_CLEAR_END */

    if (make_static) {
        rigid_body->inverse_mass = 0.0f;
        rigid_body->inverse_inertia_tensor_local = zero_matrix;
        rigid_body->inverse_inertia_system = zero_matrix;
        rigid_body->velocity = vector3_zero();
        rigid_body->angular_velocity = vector3_zero();
        rigid_body->is_sleeping = false;
        rigid_body->sleep_timer = 0.0f;
    } else {
        if ((rigid_body->mass <= 0.0f) || (!isfinite(rigid_body->mass))) {
            rigid_body->mass = 1.0f;
        }

        if (rigid_body->mass < 0.0001f) {
            rigid_body->mass = 0.0001f;
        }

        rigid_body->inverse_mass = 1.0f / rigid_body->mass;

        if (rigid_body->type == object_sphere) {
            rigidbody_update_inertia_sphere(rigid_body);
    } else if (rigid_body->type == object_cylinder) { /* MPE_FTC_091 */
        rigidbody_update_inertia_cylinder(rigid_body); /* R3-001 dedupe */
    } else {
            rigidbody_update_inertia_cube(rigid_body);
        }

        rigidbody_wake(rigid_body);
    }

    rigidbody_update_axes(rigid_body);
}

void rigidbody_set_kinematic(rigidbody *rigid_body, bool make_kinematic) {
    if (!rigid_body) {
        return;
    }
    if (make_kinematic) {
        rigid_body->static_state = false;
        rigid_body->kinematic = true;
        rigid_body->is_sleeping = false;
        rigid_body->sleep_timer = 0.0f;
        rigid_body->force_accumulator = vector3_zero();
        rigid_body->torque_accumulator = vector3_zero();
    } else {
        rigid_body->kinematic = false;
        rigidbody_set_static(rigid_body, false);
        return;
    }
    rigidbody_sanitize(rigid_body);
    rigidbody_update_axes(rigid_body);
}

/* MPE_FTC_090: Cylinder inertia and initialization */
void rigidbody_update_inertia_cylinder(rigidbody *rigid_body) {
    float r = rigid_body->radius;
    float h = rigid_body->cylinder_half_length;
    float mass = rigid_body->mass;
    float L = 2.0f * h;

    rigid_body->inertia_tensor_local = (math3){{{0}}};
    /* Axle is along X axis */
    rigid_body->inertia_tensor_local.matrix[0][0] = 0.5f * mass * r * r;
    rigid_body->inertia_tensor_local.matrix[1][1] = (mass / 12.0f) * (3.0f * r * r + L * L);
    rigid_body->inertia_tensor_local.matrix[2][2] = (mass / 12.0f) * (3.0f * r * r + L * L);

    if (mass > 0) {
        rigid_body->inverse_inertia_tensor_local = math3_inverse(rigid_body->inertia_tensor_local);
        rigid_body->inverse_inertia_system = rigid_body->inverse_inertia_tensor_local;
    } else {
        rigid_body->inverse_inertia_tensor_local = (math3){{{0}}};
        rigid_body->inverse_inertia_system = (math3){{{0}}};
    }
}

void rigidbody_initialisation_cylinder(rigidbody *rigid_body, float radius, float half_length, float mass, vector3 position_input) {
    rigid_body->position = position_input;
    rigid_body->velocity = vector3_zero();
    rigid_body->acceleration = vector3_zero();
    rigid_body->orientation = vector4_identity();
    rigid_body->angular_velocity = vector3_zero();
    rigid_body->angular_acceleration = vector3_zero();
    rigid_body->colour = (vector3){0.6f, 0.6f, 0.6f}; /* Grey for cylinders */
    rigid_body->type = object_cylinder;
    rigidbody_update_axes(rigid_body);

    rigid_body->mass = mass;
    if (mass > 0) {
        rigid_body->inverse_mass = 1.0f / mass;
    } else {
        rigid_body->inverse_mass = 0.0f;
    }
    rigid_body->radius = radius;
    rigid_body->cylinder_half_length = half_length;
    /* AUDIT: axle is local X, so the bounding half-extents are
     * (half_length, radius, radius). Keeps OBB-style readers honest. */
    rigid_body->half_extensions = (vector3){half_length, radius, radius};
    rigid_body->restitution = g_cfg.body_defaults.cylinder_restitution; /* MFS_165_CYL_RESTITUTION */
    rigid_body->static_state = (mass == 0);
    rigid_body->is_sleeping = false;
    rigid_body->sleep_timer = 0.0f;
    rigid_body->nice_value = 0;
    rigid_body->kinematic = false;
    /* FIX-AUDIT: use config cylinder defaults (new registry entries). */
    rigid_body->friction_static = g_cfg.body_defaults.cylinder_fric_s;
    rigid_body->friction_kinetic = g_cfg.body_defaults.cylinder_fric_k;

    rigidbody_update_inertia_cylinder(rigid_body);

    rigid_body->force_accumulator = vector3_zero();
    rigid_body->torque_accumulator = vector3_zero();
}

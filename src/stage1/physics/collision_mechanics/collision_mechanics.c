#include "collision_mechanics.h"
bool collision_dual_sphere (rigidbody *rigidbody_object_a, rigidbody *rigidbody_object_b, collision_data *collision_output_data) {
    vector3 relative_position_vector = vector3_subtraction (rigidbody_object_b -> position, rigidbody_object_a -> position);
    float distance_between_centres_squared = vector3_length_squared (relative_position_vector);
    float total_combined_radius = rigidbody_object_a -> radius + rigidbody_object_b -> radius;
    if (distance_between_centres_squared >= total_combined_radius * total_combined_radius) {return false;}
    float distance_between_centres = sqrtf (distance_between_centres_squared);
    collision_output_data -> object_a = rigidbody_object_a;
    collision_output_data -> object_b = rigidbody_object_b;
    const float minimum_distance_threshold_epsilon = 0.0001f;
    if (distance_between_centres > minimum_distance_threshold_epsilon) {collision_output_data -> normal_vector = vector3_scaling (relative_position_vector, 1.0f / distance_between_centres);}
    else {collision_output_data -> normal_vector = (vector3) {0.0f, 1.0f, 0.0f};}
    collision_output_data -> penetration_contact = total_combined_radius - distance_between_centres;
    collision_output_data -> contact_point = vector3_addition (rigidbody_object_a -> position, vector3_scaling (collision_output_data -> normal_vector, rigidbody_object_a -> radius));
    return true;
} float project_obb (rigidbody *rigid_body, vector3 axis, vector3 axes [3]) {return rigid_body -> half_extensions.x * fabsf (vector3_dot (axes [0], axis)) + rigid_body -> half_extensions.y * fabsf (vector3_dot (axes [1], axis)) + rigid_body -> half_extensions.z * fabsf (vector3_dot (axes [2], axis));}
bool collision_sphere_cube (rigidbody *sphere, rigidbody *cube, collision_data *collision_output_data) {
    vector3 *axes_cube = cube -> cached_axes;
    vector3 relative_position = vector3_subtraction (sphere -> position, cube -> position);
    vector3 closest_point = cube -> position;
    bool inside = true;
    float minimum_distance = 1000000.0f;
    int nearest_face_axis = 0;
    float nearest_face_sign = 1.0f;
    for (int axis_index = 0; axis_index < 3; axis_index++) {
        float distance = vector3_dot (relative_position, axes_cube [axis_index]);
        float extent = (axis_index == 0) ? cube -> half_extensions.x : (axis_index == 1) ? cube -> half_extensions.y : cube -> half_extensions.z;
        if (distance > extent) { distance = extent; inside = false; }
        else if (distance < -extent) { distance = -extent; inside = false; }
        else {
            float d_pos = extent - distance; float d_neg = extent + distance;
            if (d_pos < minimum_distance) {minimum_distance = d_pos; nearest_face_axis = axis_index; nearest_face_sign = 1.0f;}
            if (d_neg < minimum_distance) {minimum_distance = d_neg; nearest_face_axis = axis_index; nearest_face_sign = -1.0f;}
        } closest_point = vector3_addition (closest_point, vector3_scaling (axes_cube [axis_index], distance));
    } vector3 difference = vector3_subtraction (sphere -> position, closest_point);
    float distance_sq = vector3_length_squared (difference);
    if (!inside && distance_sq > sphere -> radius * sphere -> radius) return false;
    collision_output_data -> object_a = sphere; collision_output_data -> object_b = cube;
    if (inside) {
        collision_output_data -> normal_vector = vector3_scaling (axes_cube [nearest_face_axis], nearest_face_sign);
        collision_output_data -> penetration_contact = sphere -> radius + minimum_distance;
        collision_output_data -> contact_point = closest_point;
    } else {
        float distance = sqrtf (distance_sq);
        if (distance > 0.0001f) {collision_output_data -> normal_vector = vector3_scaling (difference, -1.0f / distance);}
        else {collision_output_data -> normal_vector = (vector3) {0.0f, -1.0f, 0.0f};}
        collision_output_data -> penetration_contact = sphere -> radius - distance;
        collision_output_data -> contact_point = closest_point;
    } return true;
} bool collision_dual_cube (rigidbody *cube_a, rigidbody *cube_b, collision_data *collision_output_data) {
    vector3 *axes_a = cube_a -> cached_axes; vector3 *axes_b = cube_b -> cached_axes;
    vector3 relative_position = vector3_subtraction (cube_b -> position, cube_a -> position);
    float minimum_overlap = 1000000.0f; vector3 best_axis = {0,0,0};
    for (int axis_index = 0; axis_index < 6; axis_index++) {
        vector3 axis = (axis_index < 3) ? axes_a [axis_index] : axes_b [axis_index - 3];
        float projection_a = project_obb (cube_a, axis, axes_a); float projection_b = project_obb (cube_b, axis, axes_b);
        float distance = fabsf (vector3_dot (relative_position, axis)); float overlap = projection_a + projection_b - distance;
        if (overlap < 0.0f) {return false;}
        if (overlap < minimum_overlap) {minimum_overlap = overlap; best_axis = axis;}
    } for (int axis_index_a = 0; axis_index_a < 3; axis_index_a++) {
        for (int axis_index_b = 0; axis_index_b < 3; axis_index_b++) {
            vector3 axis = vector3_cross (axes_a [axis_index_a], axes_b [axis_index_b]);
            float length_squared = vector3_length_squared (axis);
            if (length_squared < 0.0001f) continue;
            axis = vector3_scaling (axis, 1.0f / sqrtf (length_squared));
            float projection_a = project_obb (cube_a, axis, axes_a); float projection_b = project_obb (cube_b, axis, axes_b);
            float distance = fabsf (vector3_dot (relative_position, axis)); float overlap = projection_a + projection_b - distance;
            if (overlap < 0.0f) {return false;}
            if (overlap < minimum_overlap) {minimum_overlap = overlap; best_axis = axis;}
        }
    } if (vector3_dot (relative_position, best_axis) < 0) {best_axis = vector3_scaling (best_axis, -1.0f);}
    collision_output_data -> object_a = cube_a; collision_output_data -> object_b = cube_b;
    collision_output_data -> normal_vector = best_axis; collision_output_data -> penetration_contact = minimum_overlap;
    vector3 contact_point = cube_b -> position;
    for (int axis_index = 0; axis_index < 3; axis_index++) {
        float extent = (axis_index == 0) ? cube_b -> half_extensions.x : (axis_index == 1) ? cube_b -> half_extensions.y : cube_b -> half_extensions.z;
        vector3 offset = vector3_scaling (axes_b [axis_index], extent);
        if (vector3_dot (offset, best_axis) > 0) {contact_point = vector3_subtraction (contact_point, offset);}
        else {contact_point = vector3_addition (contact_point, offset);}
    } collision_output_data -> contact_point = contact_point;
    return true;
} void collision_prepare_solver (collision_data *source, collision_data *m) {
    *m = *source;
    m -> accumulated_normal_impulse = 0.0f;
    m -> accumulated_tangent_impulse = 0.0f;
    m -> ra = vector3_subtraction (m -> contact_point, m -> object_a -> position);
    m -> rb = vector3_subtraction (m -> contact_point, m -> object_b -> position);
    vector3 va = vector3_addition (m -> object_a -> velocity, vector3_cross (m -> object_a -> angular_velocity, m -> ra));
    vector3 vb = vector3_addition (m -> object_b -> velocity, vector3_cross (m -> object_b -> angular_velocity, m -> rb));
    vector3 rel_vel = vector3_subtraction (vb, va);
    float vn_initial = vector3_dot (rel_vel, m -> normal_vector);
    // FIX: Calculate Restitution Bias ONCE (Erin Catto Box2D method)
    // Only apply bounce if impact velocity is significant (> 1.0 m/s) to prevent micro-jitter
    float restitution = fminf (m -> object_a -> restitution, m -> object_b -> restitution);
    if (vn_initial < -1.0f) {m -> restitution_bias = -restitution * vn_initial;
    } else {m -> restitution_bias = 0.0f;}
    vector3 ra_cross_n = vector3_cross (m -> ra, m -> normal_vector);
    vector3 rb_cross_n = vector3_cross (m -> rb, m -> normal_vector);
    vector3 ang_a = vector3_cross (math3_multiplication_vector3 (m -> object_a -> inverse_inertia_system, ra_cross_n), m -> ra);
    vector3 ang_b = vector3_cross (math3_multiplication_vector3 (m -> object_b -> inverse_inertia_system, rb_cross_n), m -> rb);
    float k_normal = m -> object_a -> inverse_mass + m -> object_b -> inverse_mass + vector3_dot (vector3_addition (ang_a, ang_b), m -> normal_vector);
    m -> effective_mass_normal = (k_normal > 0.0f) ? (1.0f / k_normal) : 0.0f;
    // Precompute initial tangent for effective mass (will be recalculated in loop for accuracy)
    vector3 rel_vel_tangent = vector3_subtraction (rel_vel, vector3_scaling (m -> normal_vector, vn_initial));
    float tangent_speed = vector3_length (rel_vel_tangent);
    if (tangent_speed > 0.0001f) {
        m -> tangent_vector = vector3_scaling (rel_vel_tangent, -1.0f / tangent_speed);
        vector3 ra_cross_t = vector3_cross (m -> ra, m -> tangent_vector);
        vector3 rb_cross_t = vector3_cross (m -> rb, m -> tangent_vector);
        vector3 ang_a_t = vector3_cross (math3_multiplication_vector3 (m -> object_a -> inverse_inertia_system, ra_cross_t), m -> ra);
        vector3 ang_b_t = vector3_cross (math3_multiplication_vector3 (m -> object_b -> inverse_inertia_system, rb_cross_t), m -> rb);
        float k_tangent = m -> object_a -> inverse_mass + m -> object_b -> inverse_mass + vector3_dot (vector3_addition (ang_a_t, ang_b_t), m -> tangent_vector);
        m -> effective_mass_tangent = (k_tangent > 0.0f) ? (1.0f / k_tangent) : 0.0f;
    } else {
        m -> tangent_vector = vector3_zero ();
        m -> effective_mass_tangent = 0.0f;
    }
} void collision_resolve_iterative (collision_data *m) {
    // FIX 1: Recalculate CURRENT relative velocity at contact point inside the loop
    vector3 va = vector3_addition (m -> object_a -> velocity, vector3_cross (m -> object_a -> angular_velocity, m -> ra));
    vector3 vb = vector3_addition (m -> object_b -> velocity, vector3_cross (m -> object_b -> angular_velocity, m -> rb));
    vector3 rel_vel = vector3_subtraction (vb, va);
    float vn = vector3_dot (rel_vel, m -> normal_vector);
    // FIX 2: Use the safe restitution bias instead of multiplying (1+e) every iteration
    float lambda_n = (-vn + m -> restitution_bias) * m -> effective_mass_normal;
    float old_impulse = m -> accumulated_normal_impulse;
    m -> accumulated_normal_impulse = fmaxf (old_impulse + lambda_n, 0.0f);
    lambda_n = m -> accumulated_normal_impulse - old_impulse;
    if (lambda_n != 0.0f) {
        vector3 impulse = vector3_scaling (m -> normal_vector, lambda_n);
        if (!m -> object_a -> static_state) {
            m -> object_a -> velocity = vector3_subtraction (m -> object_a -> velocity, vector3_scaling (impulse, m -> object_a -> inverse_mass));
            m -> object_a -> angular_velocity = vector3_subtraction (m -> object_a -> angular_velocity, math3_multiplication_vector3 (m -> object_a -> inverse_inertia_system, vector3_cross (m -> ra, impulse)));
        } if (!m -> object_b -> static_state) {
            m -> object_b -> velocity = vector3_addition (m -> object_b -> velocity, vector3_scaling (impulse, m -> object_b -> inverse_mass));
            m -> object_b -> angular_velocity = vector3_addition (m -> object_b -> angular_velocity, math3_multiplication_vector3 (m -> object_b -> inverse_inertia_system, vector3_cross (m -> rb, impulse)));
        }
    } // Friction Impulse
    va = vector3_addition (m -> object_a -> velocity, vector3_cross (m -> object_a -> angular_velocity, m -> ra));
    vb = vector3_addition (m -> object_b -> velocity, vector3_cross (m -> object_b -> angular_velocity, m -> rb));
    rel_vel = vector3_subtraction (vb, va);
    vector3 tangent = vector3_subtraction (rel_vel, vector3_scaling (m -> normal_vector, vector3_dot (rel_vel, m -> normal_vector)));
    float tangent_length = vector3_length (tangent);
    if (tangent_length > 0.0001f) {
        tangent = vector3_scaling (tangent, -1.0f / tangent_length);
        float vt = vector3_dot (rel_vel, tangent);
        // Recompute effective mass for this specific tangent to ensure stability
        vector3 ra_cross_t = vector3_cross (m -> ra, tangent);
        vector3 rb_cross_t = vector3_cross (m -> rb, tangent);
        vector3 ang_a_t = vector3_cross (math3_multiplication_vector3 (m -> object_a -> inverse_inertia_system, ra_cross_t), m -> ra);
        vector3 ang_b_t = vector3_cross (math3_multiplication_vector3 (m -> object_b -> inverse_inertia_system, rb_cross_t), m -> rb);
        float k_tangent = m -> object_a -> inverse_mass + m -> object_b -> inverse_mass + vector3_dot (vector3_addition (ang_a_t, ang_b_t), tangent);
        float eff_mass_t = (k_tangent > 0.0f) ? (1.0f / k_tangent) : 0.0f;
        float lambda_t = -vt * eff_mass_t;
        float friction_coeff = fminf (m -> object_a -> friction_kinetic, m -> object_b -> friction_kinetic);
        float max_friction = m -> accumulated_normal_impulse * friction_coeff;
        float old_tangent_impulse = m -> accumulated_tangent_impulse;
        m -> accumulated_tangent_impulse = fmaxf (-max_friction, fminf (old_tangent_impulse + lambda_t, max_friction));
        lambda_t = m -> accumulated_tangent_impulse - old_tangent_impulse;
        if (lambda_t != 0.0f) {
            vector3 friction_impulse = vector3_scaling (tangent, lambda_t);
            if (!m -> object_a -> static_state) {
                m -> object_a -> velocity = vector3_subtraction (m -> object_a -> velocity, vector3_scaling (friction_impulse, m -> object_a -> inverse_mass));
                m -> object_a -> angular_velocity = vector3_subtraction (m -> object_a -> angular_velocity, math3_multiplication_vector3 (m -> object_a -> inverse_inertia_system, vector3_cross (m -> ra, friction_impulse)));
            } if (!m -> object_b -> static_state) {
                m -> object_b -> velocity = vector3_addition (m -> object_b -> velocity, vector3_scaling (friction_impulse, m -> object_b -> inverse_mass));
                m -> object_b -> angular_velocity = vector3_addition (m -> object_b -> angular_velocity, math3_multiplication_vector3 (m -> object_b -> inverse_inertia_system, vector3_cross (m -> rb, friction_impulse)));
            }
        }
    }
} void collision_resolve (collision_data *collision) {(void) collision;}

#include <stdio.h>
#include <math.h>
#include "../../math/math3D.h"
#include "../../math_phys_buffer/buffer.h"
#include "../forces/defines_majority_forces/define_forces.h"
#ifndef collisions_h
#define collisions_h
typedef struct {
    rigidbody *object_a;
    rigidbody *object_b;
    vector3 normal_vector; //Normal direction from obj A to obj B
    vector3 contact_point; //Position on both objects of contact
    float penetration_contact; //Overlap amount
} collision_data;
//Detection of actual collision (Sphere to Sphere contact)
static bool collision_dual_sphere (rigidbody *rigidbody_object_a, rigidbody *rigidbody_object_b, collision_data *collision_output_data) {
    vector3 relative_position_vector = vector3_subtraction (rigidbody_object_b -> position, rigidbody_object_a -> position);
    float distance_between_centres_squared = vector3_length_squared (relative_position_vector);
    float total_combined_radius = rigidbody_object_a -> radius + rigidbody_object_b -> radius;
    //Check Radial discrepancy
    if (distance_between_centres_squared >= total_combined_radius * total_combined_radius) {return false;}
    float distance_between_centres = sqrt (distance_between_centres_squared);
    collision_output_data -> object_a = rigidbody_object_a;
    collision_output_data -> object_b = rigidbody_object_b;
    //Normal = Vector from A to B
    //Safety Check (n / 0)
    const float minimum_distance_threshold_epsilon = 0.0001f;
    if (distance_between_centres > minimum_distance_threshold_epsilon) {collision_output_data -> normal_vector = vector3_scaling (relative_position_vector, 1.0f / distance_between_centres);}
    else {
        //Fallback Normal (Overlapping)
        collision_output_data -> normal_vector = (vector3) {0.0f, 1.0f, 0.0f};
    } collision_output_data -> penetration_contact = total_combined_radius - distance_between_centres;
    //Contact point between objects
    collision_output_data -> contact_point = vector3_addition (rigidbody_object_a -> position, vector3_scaling (collision_output_data -> normal_vector, rigidbody_object_a -> radius));
    return true;
} // Helper: Get OBB axes
static void get_obb_axes (rigidbody *rigid_body, vector3 axes [3]) {
    math3 rotation_matrix = vector4_to_math3 (rigid_body -> orientation);
    axes [0] = (vector3) {rotation_matrix.matrix [0][0], rotation_matrix.matrix [1][0], rotation_matrix.matrix [2][0]};
    axes [1] = (vector3) {rotation_matrix.matrix [0][1], rotation_matrix.matrix [1][1], rotation_matrix.matrix [2][1]};
    axes [2] = (vector3) {rotation_matrix.matrix [0][2], rotation_matrix.matrix [1][2], rotation_matrix.matrix [2][2]};
} //Project OBB onto axis renders
static float project_obb (rigidbody *rigid_body, vector3 axis, vector3 axes [3]) {
    return rigid_body -> half_extensions.x * fabsf (vector3_dot (axes [0], axis)) + rigid_body -> half_extensions.y * fabsf (vector3_dot (axes [1], axis)) + rigid_body -> half_extensions.z * fabsf (vector3_dot (axes [2], axis));
} //Detection of Sphere to Cube collisions, OBB
static bool collision_sphere_cube (rigidbody *sphere, rigidbody *cube, collision_data *collision_output_data) {
    vector3 *axes_cube = cube -> cached_axes;
    vector3 relative_position = vector3_subtraction (sphere -> position, cube -> position);
    vector3 closest_point = cube -> position;
    bool inside = true;
    float minimum_distance = 1000000.0f;
    int nearest_face_axis = 0;
    float nearest_face_sign = 1.0f;
    for (int axis_index = 0; axis_index < 3; axis_index++) {
        float distance = vector3_dot (relative_position, axes_cube [axis_index]);
        float extent;
        if (axis_index == 0) {extent = cube -> half_extensions.x;}
        else if (axis_index == 1) {extent = cube -> half_extensions.y;}
        else {extent = cube -> half_extensions.z;}
        if (distance > extent) {
            distance = extent;
            inside = false;
        } else if (distance < -extent) {
            distance = -extent;
            inside = false;
        } else {
            // If inside, track distance to nearest face
            float d_pos = extent - distance;
            float d_neg = extent + distance;
            if (d_pos < minimum_distance) {minimum_distance = d_pos; nearest_face_axis = axis_index; nearest_face_sign = 1.0f;}
            if (d_neg < minimum_distance) {minimum_distance = d_neg; nearest_face_axis = axis_index; nearest_face_sign = -1.0f;}
        } closest_point = vector3_addition (closest_point, vector3_scaling (axes_cube [axis_index], distance));
    } vector3 difference = vector3_subtraction (sphere -> position, closest_point);
    float distance_sq = vector3_length_squared (difference);
    if (!inside && distance_sq > sphere -> radius * sphere -> radius) return false;
    collision_output_data -> object_a = sphere;
    collision_output_data -> object_b = cube;
    if (inside) {
        // Normal points from sphere center to nearest face (Sphere -> Cube)
        collision_output_data -> normal_vector = vector3_scaling (axes_cube [nearest_face_axis], nearest_face_sign);
        collision_output_data -> penetration_contact = sphere -> radius + minimum_distance;
        collision_output_data -> contact_point = closest_point; // closest point is on the face
    } else {
        float distance = sqrtf (distance_sq);
        //Normal must point from Sphere to Cube
        //Difference is at sphere_ctpt - cube_clpt
        //We need negative difference to account for vector from spheres to cubes
        if (distance > 0.0001f) {collision_output_data -> normal_vector = vector3_scaling (difference, -1.0f / distance);}
        else {collision_output_data -> normal_vector = (vector3) {0.0f, -1.0f, 0.0f};}
        collision_output_data -> penetration_contact = sphere -> radius - distance;
        collision_output_data -> contact_point = closest_point;
    } return true;
} //Cube to Cube OBB-OBB collision detection using SAT
static bool collision_dual_cube (rigidbody *cube_a, rigidbody *cube_b, collision_data *collision_output_data) {
    vector3 *axes_a = cube_a -> cached_axes;
    vector3 *axes_b = cube_b -> cached_axes;
    vector3 relative_position = vector3_subtraction (cube_b -> position, cube_a -> position);
    float minimum_overlap = 1000000.0f;
    vector3 best_axis = {0,0,0};
    //Check Face Axes, 6 of them
    for (int axis_index = 0; axis_index < 6; axis_index++) {
        vector3 axis;
        if (axis_index < 3) {axis = axes_a [axis_index];}
        else {axis = axes_b [axis_index - 3];}
        float projection_a = project_obb (cube_a, axis, axes_a);
        float projection_b = project_obb (cube_b, axis, axes_b);
        float distance = fabsf (vector3_dot (relative_position, axis));
        float overlap = projection_a + projection_b - distance;
        if (overlap < 0.0f) {return false;}
        if (overlap < minimum_overlap) {
            minimum_overlap = overlap;
            best_axis = axis;
        }
    } //Check Cross Product Axes, 9 of them
    for (int axis_index_a = 0; axis_index_a < 3; axis_index_a++) {
        for (int axis_index_b = 0; axis_index_b < 3; axis_index_b++) {
            vector3 axis = vector3_cross (axes_a [axis_index_a], axes_b [axis_index_b]);
            float length_squared = vector3_length_squared (axis);
            if (length_squared < 0.0001f) continue;
            axis = vector3_scaling (axis, 1.0f / sqrtf (length_squared));
            float projection_a = project_obb (cube_a, axis, axes_a);
            float projection_b = project_obb (cube_b, axis, axes_b);
            float distance = fabsf (vector3_dot (relative_position, axis));
            float overlap = projection_a + projection_b - distance;
            if (overlap < 0.0f) {return false;}
            if (overlap < minimum_overlap) {
                minimum_overlap = overlap;
                best_axis = axis;
            }
        }
    } // Ensure normal points from A to B
    if (vector3_dot (relative_position, best_axis) < 0) {best_axis = vector3_scaling (best_axis, -1.0f);}
    collision_output_data -> object_a = cube_a;
    collision_output_data -> object_b = cube_b;
    collision_output_data -> normal_vector = best_axis;
    collision_output_data -> penetration_contact = minimum_overlap;
    // Improved contact point: Find vertex of Cube B furthest in direction of -normal
    vector3 contact_point = cube_b -> position;
    for (int axis_index = 0; axis_index < 3; axis_index++) {
        float extent;
        if (axis_index == 0) {extent = cube_b -> half_extensions.x;}
        else if (axis_index == 1) {extent = cube_b -> half_extensions.y;}
        else {extent = cube_b -> half_extensions.z;}
        vector3 offset = vector3_scaling (axes_b [axis_index], extent);
        if (vector3_dot (offset, best_axis) > 0) {contact_point = vector3_subtraction (contact_point, offset);}
        else {contact_point = vector3_addition (contact_point, offset);}
    } collision_output_data -> contact_point = contact_point;
    return true;
} //Resolution (Impulse-Momemtum) (x, y, z directional vectoring for impulse related calculations)
static void collision_resolve (collision_data *collision) {
    rigidbody *object_a = collision -> object_a, *object_b = collision -> object_b;
    //Calculate the vectors from the centre of mass to the contact point between objects (rad_a, and rad_b)]
    vector3 relative_contact_vector_a = vector3_subtraction (collision -> contact_point, object_a -> position);
    vector3 relative_contact_vector_b = vector3_subtraction (collision -> contact_point, object_b -> position);
    //Relative Velocity of the system at the point of contact
    vector3 velocity_a_at_contact = vector3_addition (object_a -> velocity, vector3_cross (object_a -> angular_velocity, relative_contact_vector_a)); //Compute Same change of velocity stemming from angualr velocity change for object A
    vector3 velocity_b_at_contact = vector3_addition (object_b -> velocity, vector3_cross (object_b -> angular_velocity, relative_contact_vector_b)); //Compute change of angular velocity change of the object sphere for object B and combine with already existing velocity
    vector3 relative_velocity = vector3_subtraction (velocity_b_at_contact, velocity_a_at_contact); //Compute Net Velocity of system (both objects)
    //Velocity along the normal line of vector connecting objects to contact point
    float relative_velocity_dot_normal = vector3_dot (relative_velocity, collision -> normal_vector);
    //If objects are moving apart/away, no need for implement
    if (relative_velocity_dot_normal > 0) {return;}
    //Calculate Impulse on Scalar (delta P)
    //Delta P = (-1(1 + e) * vrdn) / ( ((mass_a) ^ -1) + ((mass_b) ^ -1) + (n * (((impulse_a) ^ -1 * ra * normal_vector * ra) + ((impulse_b) ^ -1 * rb * normal_vector * rb))))
    float restitution_coefficient = fminf (object_a -> restitution, object_b -> restitution);
    //Linear Components
    float inverse_mass_sum = object_a -> inverse_mass + object_b -> inverse_mass;
    //Rotational Components (Inertial resistance to rotational momemtum experiences by getting hit by other object)
    vector3 ra_cross_normal = vector3_cross (relative_contact_vector_a, collision -> normal_vector);
    vector3 rb_cross_normal = vector3_cross (relative_contact_vector_b, collision -> normal_vector);
    vector3 angular_motion_component_a = vector3_cross (math3_multiplication_vector3 (object_a -> inverse_inertia_system, ra_cross_normal), relative_contact_vector_a);
    vector3 angular_motion_component_b = vector3_cross (math3_multiplication_vector3 (object_b -> inverse_inertia_system, rb_cross_normal), relative_contact_vector_b);
    float rotational_resistance_sum = vector3_dot (vector3_addition (angular_motion_component_a, angular_motion_component_b), collision -> normal_vector);
    float impulse_denominator = inverse_mass_sum + rotational_resistance_sum;
    if (impulse_denominator <= 0.0f) {return;}
    float impulse_scalar = (-(1.0f + restitution_coefficient) * relative_velocity_dot_normal) / impulse_denominator;
    //Apply Impulse to objects
    vector3 impulse_vector = vector3_scaling (collision -> normal_vector, impulse_scalar);
    //Linear Velocity Changes: (delta v = impulse * mass ^ -1)
    if (!object_a -> static_state) {
        object_a -> velocity = vector3_subtraction (object_a -> velocity, vector3_scaling (impulse_vector, object_a -> inverse_mass));
        vector3 a_angular_impulse = vector3_cross (relative_contact_vector_a, vector3_scaling (impulse_vector, -1.0f));
        object_a -> angular_velocity = vector3_addition (object_a -> angular_velocity, math3_multiplication_vector3 (object_a -> inverse_inertia_system, a_angular_impulse));
    } if (!object_b -> static_state) {
        object_b -> velocity = vector3_addition (object_b -> velocity, vector3_scaling (impulse_vector, object_b -> inverse_mass));
        vector3 b_angular_impulse = vector3_cross (relative_contact_vector_b, impulse_vector);
        object_b -> angular_velocity = vector3_addition (object_b -> angular_velocity, math3_multiplication_vector3 (object_b -> inverse_inertia_system, b_angular_impulse));
    } //Correct Position Change Values (FPU error, results in sinking of objects into each other)
    const float error_correction_percent = 0.15f; // Reduced from 20% to 15% for stability
    const float penetration_allowance_slop = 0.02f; // Increased allowance to reduce 'kick' jitter
    //Correction: Push back proportions for dropping beneath mesh
    if (inverse_mass_sum <= 0.0001f) {return;}
    float correction_magnitude = fmaxf (collision -> penetration_contact - penetration_allowance_slop, 0.0f);
    if (correction_magnitude > 2.0f) correction_magnitude = 2.0f; // Increased cap for deeper penetration recovery
    // Improved correction: Only scale by inverse mass if both are dynamic.
    // If one is static, the other must take the full displacement.
    vector3 position_correction;
    if (object_a -> static_state || object_b -> static_state) {
        // One object is static, dynamic object takes 100% of correction
        position_correction = vector3_scaling (collision -> normal_vector, correction_magnitude * error_correction_percent);
    } else {
        // Both dynamic, distribute by inverse mass
        position_correction = vector3_scaling (collision -> normal_vector, (correction_magnitude / inverse_mass_sum) * error_correction_percent);
    } object_a -> position = vector3_subtraction (object_a -> position, vector3_scaling (position_correction, object_a -> inverse_mass));
    object_b -> position = vector3_addition (object_b -> position, vector3_scaling (position_correction, object_b -> inverse_mass));
}
#endif

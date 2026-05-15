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
    //Safety Check (Div zero)
    const float minimum_distance_threshold_epsilon = 0.0001f;
    if (distance_between_centres > minimum_distance_threshold_epsilon) {
        collision_output_data -> normal_vector = vector3_scaling (relative_position_vector, 1.0f / distance_between_centres);
    } else {
        //Fallback Normal (Overlapping)
        collision_output_data -> normal_vector = (vector3) {0.0f, 1.0f, 0.0f};
    } collision_output_data -> penetration_contact = total_combined_radius - distance_between_centres;
    //Contact point between objects
    collision_output_data -> contact_point = vector3_addition (rigidbody_object_a -> position, vector3_scaling (collision_output_data -> normal_vector, rigidbody_object_a -> radius));
    return true;
}

// Helper: Get OBB axes
static void get_obb_axes (rigidbody *rb, vector3 axes [3]) {
    math3 rot = vector4_to_math3 (rb -> orientation);
    axes [0] = (vector3) {rot.matrix [0][0], rot.matrix [1][0], rot.matrix [2][0]};
    axes [1] = (vector3) {rot.matrix [0][1], rot.matrix [1][1], rot.matrix [2][1]};
    axes [2] = (vector3) {rot.matrix [0][2], rot.matrix [1][2], rot.matrix [2][2]};
}

// Helper: Project OBB onto axis
static float project_obb (rigidbody *rb, vector3 axis, vector3 axes [3]) {
    return rb -> half_extensions.x * fabsf (vector3_dot (axes [0], axis)) +
           rb -> half_extensions.y * fabsf (vector3_dot (axes [1], axis)) +
           rb -> half_extensions.z * fabsf (vector3_dot (axes [2], axis));
}

// Detection: Sphere to Cube (OBB)
static bool collision_sphere_cube (rigidbody *sphere, rigidbody *cube, collision_data *collision_output_data) {
    vector3 axes_cube [3];
    get_obb_axes (cube, axes_cube);

    vector3 relative_pos = vector3_subtraction (sphere -> position, cube -> position);
    vector3 closest_point = cube -> position;
    bool inside = true;
    float min_dist = 1000000.0f;
    int nearest_face_axis = 0;
    float nearest_face_sign = 1.0f;

    for (int i = 0; i < 3; i++) {
        float dist = vector3_dot (relative_pos, axes_cube [i]);
        float extent = (i == 0) ? cube -> half_extensions.x : (i == 1 ? cube -> half_extensions.y : cube -> half_extensions.z);
        
        if (dist > extent) {
            dist = extent;
            inside = false;
        } else if (dist < -extent) {
            dist = -extent;
            inside = false;
        } else {
            // If inside, track distance to nearest face
            float d_pos = extent - dist;
            float d_neg = extent + dist;
            if (d_pos < min_dist) { min_dist = d_pos; nearest_face_axis = i; nearest_face_sign = 1.0f; }
            if (d_neg < min_dist) { min_dist = d_neg; nearest_face_axis = i; nearest_face_sign = -1.0f; }
        }
        closest_point = vector3_addition (closest_point, vector3_scaling (axes_cube [i], dist));
    }

    vector3 diff = vector3_subtraction (sphere -> position, closest_point);
    float distance_sq = vector3_length_squared (diff);

    if (!inside && distance_sq > sphere -> radius * sphere -> radius) return false;

    collision_output_data -> object_a = sphere;
    collision_output_data -> object_b = cube;

    if (inside) {
        // Normal points from sphere center to nearest face (Sphere -> Cube)
        collision_output_data -> normal_vector = vector3_scaling (axes_cube [nearest_face_axis], nearest_face_sign);
        collision_output_data -> penetration_contact = sphere -> radius + min_dist;
        collision_output_data -> contact_point = closest_point; // closest point is on the face
    } else {
        float distance = sqrtf (distance_sq);
        // Normal must point from Sphere (A) to Cube (B). 
        // diff is (Sphere - closestPoint), which is (A - B_surface).
        // So we need -diff.
        if (distance > 0.0001f) {
            collision_output_data -> normal_vector = vector3_scaling (diff, -1.0f / distance);
        } else {
            collision_output_data -> normal_vector = (vector3) {0.0f, -1.0f, 0.0f};
        }
        collision_output_data -> penetration_contact = sphere -> radius - distance;
        collision_output_data -> contact_point = closest_point;
    }

    return true;
}

// Detection: Cube to Cube (OBB-OBB) using SAT
static bool collision_dual_cube (rigidbody *cube_a, rigidbody *cube_b, collision_data *collision_output_data) {
    vector3 axes_a [3], axes_b [3];
    get_obb_axes (cube_a, axes_a);
    get_obb_axes (cube_b, axes_b);

    vector3 relative_pos = vector3_subtraction (cube_b -> position, cube_a -> position);
    
    vector3 test_axes [15];
    for (int i = 0; i < 3; i++) test_axes [i] = axes_a [i];
    for (int i = 0; i < 3; i++) test_axes [i + 3] = axes_b [i];
    
    int axis_idx = 6;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            test_axes [axis_idx] = vector3_cross (axes_a [i], axes_b [j]);
            if (vector3_length_squared (test_axes [axis_idx]) < 0.0001f) {
                test_axes [axis_idx] = (vector3) {0,0,0}; // Skip degenerate
            } else {
                test_axes [axis_idx] = vector3_normalisation (test_axes [axis_idx]);
            }
            axis_idx++;
        }
    }

    float min_overlap = 1000000.0f;
    vector3 best_axis = {0,0,0};

    for (int i = 0; i < 15; i++) {
        vector3 axis = test_axes [i];
        if (axis.x == 0 && axis.y == 0 && axis.z == 0) continue;

        float projection_a = project_obb (cube_a, axis, axes_a);
        float projection_b = project_obb (cube_b, axis, axes_b);
        float distance = fabsf (vector3_dot (relative_pos, axis));

        float overlap = projection_a + projection_b - distance;
        if (overlap < 0) return false;

        if (overlap < min_overlap) {
            min_overlap = overlap;
            best_axis = axis;
        }
    }

    // Ensure normal points from A to B
    if (vector3_dot (relative_pos, best_axis) < 0) {
        best_axis = vector3_scaling (best_axis, -1.0f);
    }

    collision_output_data -> object_a = cube_a;
    collision_output_data -> object_b = cube_b;
    collision_output_data -> normal_vector = best_axis;
    collision_output_data -> penetration_contact = min_overlap;
    
    // Improved contact point: Find vertex of Cube B furthest in direction of -normal
    vector3 contact_point = cube_b -> position;
    for (int i = 0; i < 3; i++) {
        float extent = (i == 0) ? cube_b -> half_extensions.x : (i == 1 ? cube_b -> half_extensions.y : cube_b -> half_extensions.z);
        vector3 offset = vector3_scaling (axes_b [i], extent);
        if (vector3_dot (offset, best_axis) > 0) {
            contact_point = vector3_subtraction (contact_point, offset);
        } else {
            contact_point = vector3_addition (contact_point, offset);
        }
    }
    collision_output_data -> contact_point = contact_point;

    return true;
}

//Resolution (Impulse-Momemtum) (x, y, z directional vectoring for impulse related calculations)
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
    object_a -> velocity = vector3_subtraction (object_a -> velocity, vector3_scaling (impulse_vector, object_a -> inverse_mass)); //Add current Velocity to delta v (impulse * mass ^ -1) (Object A)
    object_b -> velocity = vector3_addition (object_b -> velocity, vector3_scaling (impulse_vector, object_b -> inverse_mass)); //Add current Velocity to delta v (impulse * mass ^ -1) (Object B)
    //Changes to angular velocity: (delta angular_v = I ^ -1 * (r * impulse_scalar))
    //I ^ -1: inverse of the moment of inertia (tensor)
    vector3 a_angular_impulse = vector3_cross (relative_contact_vector_a, vector3_scaling (impulse_vector, -1.0f)); //Impulse scalar for object a
    vector3 b_angular_impulse = vector3_cross (relative_contact_vector_b, impulse_vector); //impulse_scalar for object b
    object_a -> angular_velocity = vector3_addition (object_a -> angular_velocity, math3_multiplication_vector3 (object_a -> inverse_inertia_system, a_angular_impulse)); //Delta angular_v + current angular_v = final angular_v (Object A)
    object_b -> angular_velocity = vector3_addition (object_b -> angular_velocity, math3_multiplication_vector3 (object_b -> inverse_inertia_system, b_angular_impulse)); //Delta angular_v + current_angualr_v = final angular_v (Object B)
    //Correct Position Change Values (FPU error, results in sinking of objects into each other)
    const float error_correction_percent = 0.2f; //20% correction per frame and motion calculated
    const float penetration_allowance_slop = 0.01f; //Allowance for object overlap (penetration, sinking)
    //Correction: Push back proportions by 20% for dropping beneath mesh
    if (inverse_mass_sum <= 0.0f) {return;}
    float correction_magnitude = fmaxf (collision -> penetration_contact - penetration_allowance_slop, 0.0f);
    if (correction_magnitude > 0.5f) correction_magnitude = 0.5f; // Safety cap to prevent warping
    vector3 position_correction = vector3_scaling (collision -> normal_vector, correction_magnitude / inverse_mass_sum * error_correction_percent);
    object_a -> position = vector3_subtraction (object_a -> position, vector3_scaling (position_correction, object_a -> inverse_mass));
    object_b -> position = vector3_addition (object_b -> position, vector3_scaling (position_correction, object_b -> inverse_mass));
}
#endif

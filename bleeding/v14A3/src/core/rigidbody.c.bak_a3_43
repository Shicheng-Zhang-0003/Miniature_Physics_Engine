#include "../mpe_engine.h"
#include "rigidbody.h"
// Helper to update axes from orientation
void rigidbody_update_axes (rigidbody *rigid_body) {
    math3 rotation_matrix = vector4_to_math3 (rigid_body -> orientation);
    rigid_body -> cached_axes [0] = (vector3) {rotation_matrix.matrix [0][0], rotation_matrix.matrix [1][0], rotation_matrix.matrix [2][0]};
    rigid_body -> cached_axes [1] = (vector3) {rotation_matrix.matrix [0][1], rotation_matrix.matrix [1][1], rotation_matrix.matrix [2][1]};
    rigid_body -> cached_axes [2] = (vector3) {rotation_matrix.matrix [0][2], rotation_matrix.matrix [1][2], rotation_matrix.matrix [2][2]};
} //Init
void rigidbody_initialisation_sphere (rigidbody *rigid_body, float radius, float mass, vector3 position_input) {
    //Kinematic
    rigid_body -> position = position_input;
    rigid_body -> velocity = vector3_zero ();
    rigid_body -> acceleration = vector3_zero ();
    rigid_body -> orientation = vector4_identity ();
    rigid_body -> angular_velocity = vector3_zero ();
    rigid_body -> angular_acceleration = vector3_zero ();
    rigid_body -> colour = (vector3) {0.2f, 0.6f, 1.0f};
    rigid_body -> type = object_sphere;
    rigidbody_update_axes (rigid_body);
    //Dynamic
    rigid_body -> mass = mass;
    if (mass > 0) {rigid_body -> inverse_mass = 1.0f / mass;}
    else {rigid_body -> inverse_mass = 0.0f;}
    rigid_body -> radius = radius;
    rigid_body -> restitution = 0.5f; //Default Bounce Energy Return
    rigid_body -> static_state = (mass == 0);
    rigid_body -> is_sleeping = false;
    rigid_body -> sleep_timer = 0.0f; //Static Objects
    rigid_body -> friction_static = 0.3f;
    rigid_body -> friction_kinetic = 0.2f;
    //Inertial Tensors
    //I = 0.4fmr ^ 2
    float inertia_coefficient_sphere = (0.4f) * mass * radius * radius;
    rigid_body -> inertia_tensor_local = (math3) {{{0}}};
    rigid_body -> inertia_tensor_local.matrix [0][0] = inertia_coefficient_sphere;
    rigid_body -> inertia_tensor_local.matrix [1][1] = inertia_coefficient_sphere;
    rigid_body -> inertia_tensor_local.matrix [2][2] = inertia_coefficient_sphere;
    //Initialize Inverse Inertia System
    if (mass > 0) {
        rigid_body -> inverse_inertia_tensor_local = math3_inverse (rigid_body -> inertia_tensor_local);
        rigid_body -> inverse_inertia_system = rigid_body -> inverse_inertia_tensor_local;
    } else {
        rigid_body -> inverse_inertia_tensor_local = (math3) {{{0}}};
        rigid_body -> inverse_inertia_system = (math3) {{{0}}};
    } //Total Force and Torque accumulation
    rigid_body -> force_accumulator = vector3_zero ();
    rigid_body -> torque_accumulator = vector3_zero ();
} // Helper to update inertia tensor after mass/radius change
void rigidbody_update_inertia_sphere (rigidbody *rigid_body) {
    float inertia_coefficient_sphere = (0.4f) * rigid_body -> mass * rigid_body -> radius * rigid_body -> radius;
    rigid_body -> inertia_tensor_local = (math3) {{{0}}};
    rigid_body -> inertia_tensor_local.matrix [0][0] = inertia_coefficient_sphere;
    rigid_body -> inertia_tensor_local.matrix [1][1] = inertia_coefficient_sphere;
    rigid_body -> inertia_tensor_local.matrix [2][2] = inertia_coefficient_sphere;
    if (rigid_body -> mass > 0) {
        rigid_body -> inverse_inertia_tensor_local = math3_inverse (rigid_body -> inertia_tensor_local);
        rigid_body -> inverse_inertia_system = rigid_body -> inverse_inertia_tensor_local;
    } else {
        rigid_body -> inverse_inertia_tensor_local = (math3) {{{0}}};
        rigid_body -> inverse_inertia_system = (math3) {{{0}}};
    }
} // Helper to update inertia tensor after mass/radius change
void rigidbody_update_inertia_cube (rigidbody *rigid_body) {
    float width = rigid_body -> half_extensions.x * 2.0f;
    float height = rigid_body -> half_extensions.y * 2.0f;
    float depth = rigid_body -> half_extensions.z * 2.0f;
    float mass = rigid_body -> mass;
    rigid_body -> inertia_tensor_local = (math3) {{{0}}};
    rigid_body -> inertia_tensor_local.matrix [0][0] = (mass / 12.0f) * (height * height + depth * depth);
    rigid_body -> inertia_tensor_local.matrix [1][1] = (mass / 12.0f) * (width * width + depth * depth);
    rigid_body -> inertia_tensor_local.matrix [2][2] = (mass / 12.0f) * (width * width + height * height);
    if (mass > 0) {
        rigid_body -> inverse_inertia_tensor_local = math3_inverse (rigid_body -> inertia_tensor_local);
        rigid_body -> inverse_inertia_system = rigid_body -> inverse_inertia_tensor_local;
    } else {
        rigid_body -> inverse_inertia_tensor_local = (math3) {{{0}}};
        rigid_body -> inverse_inertia_system = (math3) {{{0}}};
    }
} //Force application and Torque Dynamics
//Apply a force at a centre of mass (perfect collision movement, linear movement only defined)
void rb_apply_forces_perfect (rigidbody *rigid_body, vector3 force_applied) {
    if (rigid_body -> static_state) {return;}
    rigid_body -> is_sleeping = false;
    rigid_body -> sleep_timer = 0.0f;
    rigid_body -> force_accumulator = vector3_addition (rigid_body -> force_accumulator, force_applied); //Force applied to torque and circular momentum
} //Apply force at a point not the centre of mass (which generates rotational motion and torque)
//locale_impact = impact point on object identified
void rb_apply_forces_localised (rigidbody *rigid_body, vector3 force_applied, vector3 locale_impact) {
    if (rigid_body -> static_state) {return;}
    rigid_body -> is_sleeping = false;
    rigid_body -> sleep_timer = 0.0f;
    rb_apply_forces_perfect (rigid_body, force_applied);
    //Torque = r * F (r = vector from Centre of Mass to the point of actual contact between objects)
    vector3 relative_contact_vector = vector3_subtraction (locale_impact, rigid_body -> position);
    vector3 torque_generated = vector3_cross (relative_contact_vector, force_applied);
    rigid_body -> torque_accumulator = vector3_addition (rigid_body -> torque_accumulator, torque_generated);
} //Energy Computation
float rb_get_kinetic_energy (rigidbody *rigid_body) {
    //EK normal = 0.5fmv ^ 2
    float linear_kinetic_energy = 0.5f * rigid_body -> mass * vector3_length_squared (rigid_body -> velocity);
    //EK rotational = 0.5fwIw
    vector3 angular_momemtum = math3_multiplication_vector3 (math3_inverse (rigid_body -> inverse_inertia_system), rigid_body -> angular_velocity);
    float rotational_kinetic_energy = 0.5f * vector3_dot (rigid_body -> angular_velocity, angular_momemtum);
    return linear_kinetic_energy + rotational_kinetic_energy;
} //Integration Segmentation (Movement Compute)
void rb_integrate (rigidbody *rigid_body, float delta_time, float linear_damping, float angular_damping) {
    if ((rigid_body -> static_state) || (delta_time <= 0.0f)) {return;}
    if (rigid_body -> is_sleeping) {return;}

    // Safety Clamp: Prevent infinite energy explosions in deep piles
    float max_linear_speed = 150.0f;
    float current_speed_sq = vector3_length_squared (rigid_body -> velocity);
    if (current_speed_sq > max_linear_speed * max_linear_speed) {
        rigid_body -> velocity = vector3_scaling (vector3_normalisation (rigid_body -> velocity), max_linear_speed);
    }
    float max_angular_speed = 30.0f;
    float current_ang_speed_sq = vector3_length_squared (rigid_body -> angular_velocity);
    if (current_ang_speed_sq > max_angular_speed * max_angular_speed) {
        rigid_body -> angular_velocity = vector3_scaling (vector3_normalisation (rigid_body -> angular_velocity), max_angular_speed);
    }

    float speed_sq = vector3_length_squared (rigid_body -> velocity);
    float ang_speed_sq = vector3_length_squared (rigid_body -> angular_velocity);
    if (speed_sq < 0.0025f && ang_speed_sq < 0.0001f) {
        rigid_body -> sleep_timer += delta_time;
        if (rigid_body -> sleep_timer > 1.0f) {rigid_body -> is_sleeping = true;}
    } else {rigid_body -> sleep_timer = 0.0f;}
    //Update inverse inertia tensor based on current orientation before using it
    //inverse_inertia_system = rotational * inverse_inertia_local * transposed value in 4D rotational axis
    math3 rotation_matrix_current = vector4_to_math3 (rigid_body -> orientation); //W axis orientation of rotation
    math3 rotation_matrix_transposed = math3_transposition (rotation_matrix_current);
    rigid_body -> inverse_inertia_system = math3_multiplication (rotation_matrix_current, math3_multiplication (rigid_body -> inverse_inertia_tensor_local, rotation_matrix_transposed));
    //Calculate Linear Acceleration (F = ma, a = Fm ^ -1)
    rigid_body -> acceleration = vector3_scaling (rigid_body -> force_accumulator, rigid_body -> inverse_mass); //Multiply Force by inverse of mass
    //Calculate Instantaneous Velocity
    rigid_body -> velocity = vector3_addition (rigid_body -> velocity, vector3_scaling (rigid_body -> acceleration, delta_time)); //Add currenty velocity to delta v
    //Air resistance increased slightly for stability
    rigid_body -> velocity = vector3_scaling (rigid_body -> velocity, linear_damping);
    // Sleep Threshold, eliminates tiny jitters
    if (vector3_length_squared (rigid_body -> velocity) < 0.00005f) {rigid_body -> velocity = vector3_zero ();}
    //Calculate Position Standard
    rigid_body -> position = vector3_addition (rigid_body -> position, vector3_scaling (rigid_body -> velocity, delta_time)); //Add current position to delta d
    //Update Angular Acceleration Standard
    rigid_body -> angular_acceleration = math3_multiplication_vector3 (rigid_body -> inverse_inertia_system, rigid_body -> torque_accumulator);
    //Update Standard Angular Velocity
    rigid_body -> angular_velocity = vector3_addition (rigid_body -> angular_velocity, vector3_scaling (rigid_body -> angular_acceleration, delta_time)); //Sum of current angular velocity by delta angular velocity
    //Angular Damping - High damping helps objects settle
    rigid_body -> angular_velocity = vector3_scaling (rigid_body -> angular_velocity, angular_damping);
    if (vector3_length_squared (rigid_body -> angular_velocity) < 0.0001f) {rigid_body -> angular_velocity = vector3_zero ();}
    //Update General Orientation (4D)
    //delta_q = [0, w-axis_values] * q * 0.5f * dt
    vector4 angular_velocity_quaternion = {0, rigid_body -> angular_velocity.x, rigid_body -> angular_velocity.y, rigid_body -> angular_velocity.z}; //Start with no w axis definition
    vector4 orientation_change_delta = vector4_multiplication (angular_velocity_quaternion, rigid_body -> orientation); //Orientation = W-Axis value
    //Set Orientation individually
    rigid_body -> orientation.w += orientation_change_delta.w * 0.5f * delta_time;
    rigid_body -> orientation.x += orientation_change_delta.x * 0.5f * delta_time;
    rigid_body -> orientation.y += orientation_change_delta.y * 0.5f * delta_time;
    rigid_body -> orientation.z += orientation_change_delta.z * 0.5f * delta_time;
    rigid_body -> orientation = vector4_normalisation (rigid_body -> orientation);
    //Clear accumulators of force and torque for next implementation
    rigid_body -> force_accumulator = vector3_zero ();
    rigid_body -> torque_accumulator = vector3_zero ();
    rigidbody_update_axes (rigid_body);
} vector3 make_half_extents (float width, float height, float depth) {return (vector3){width * 0.5f, height * 0.5f, depth * 0.5f};}
// Initialize a cube: Box, OBB
void rigidbody_initialisation_cube (rigidbody *rigid_body, vector3 position_input, vector3 half_extensions, float mass) {
    //Kinematic
    rigid_body -> position = position_input;
    rigid_body -> velocity = vector3_zero ();
    rigid_body -> acceleration = vector3_zero ();
    rigid_body -> orientation = vector4_identity ();
    rigid_body -> angular_velocity = vector3_zero ();
    rigid_body -> angular_acceleration = vector3_zero ();
    rigid_body -> colour = (vector3) {1.0f, 0.4f, 0.2f};  // Orange-ish for cubes
    rigid_body -> type = object_cube;
    rigidbody_update_axes (rigid_body);
    //Dynamic
    rigid_body -> mass = mass;
    if (mass > 0) {rigid_body -> inverse_mass = 1.0f / mass;}
    else {rigid_body -> inverse_mass = 0.0f;}
    rigid_body -> half_extensions = half_extensions;
    rigid_body -> radius = vector3_length (half_extensions); // Bounding radius for broadphase
    rigid_body -> restitution = 0.5f;
    rigid_body -> static_state = (mass == 0);
    rigid_body -> is_sleeping = false;
    rigid_body -> sleep_timer = 0.0f;
    rigid_body -> friction_static = 0.4f;
    rigid_body -> friction_kinetic = 0.3f;
    //Inertia Tensor for rectangular box: I = (m/12) * (h² + d², w² + d², w² + h²), nominal extension only for boxes
    float width = half_extensions.x * 2.0f;  //full width
    float height = half_extensions.y * 2.0f;  //full height
    float depth = half_extensions.z * 2.0f;  //full depth
    rigid_body -> inertia_tensor_local = (math3) {{{0}}};
    rigid_body -> inertia_tensor_local.matrix [0][0] = (mass / 12.0f) * (height * height + depth * depth);
    rigid_body -> inertia_tensor_local.matrix [1][1] = (mass / 12.0f) * (width * width + depth * depth);
    rigid_body -> inertia_tensor_local.matrix [2][2] = (mass / 12.0f) * (width * width + height * height);
    if (mass > 0) {
        rigid_body -> inverse_inertia_tensor_local = math3_inverse (rigid_body -> inertia_tensor_local);
        rigid_body -> inverse_inertia_system = rigid_body -> inverse_inertia_tensor_local;
    } else {
        rigid_body -> inverse_inertia_tensor_local = (math3) {{{0}}};
        rigid_body -> inverse_inertia_system = (math3) {{{0}}};
    } // Force & Torque accumulators
    rigid_body -> force_accumulator = vector3_zero ();
    rigid_body -> torque_accumulator = vector3_zero ();
} void rigidbody_wake (rigidbody *rigid_body) {
    if (rigid_body -> is_sleeping) {
        rigid_body -> is_sleeping = false;
        rigid_body -> sleep_timer = 0.0f;
    }
}

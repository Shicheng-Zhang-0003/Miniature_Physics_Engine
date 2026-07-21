#include "spring_joint.h"
#include <stdio.h>
#include <stdlib.h>
#include <epoxy/gl.h>

spring_joint joint_pool [maximum_joint_count];
int current_joint_count = 0;
int add_joint (int object_index_a, int object_index_b, float equilibrium_length, float spring_constant, float damping_coefficient) {
    //Add a new object in a empty row of buffer
    for (int joint_index = 0; joint_index < maximum_joint_count; joint_index++) {
        if (!joint_pool [joint_index].is_active) {
            joint_pool [joint_index].object_index_a = object_index_a;
            joint_pool [joint_index].object_index_b = object_index_b;
            joint_pool [joint_index].equilibrium_length = equilibrium_length;
            joint_pool [joint_index].spring_constant = spring_constant;
            joint_pool [joint_index].damping_coefficient = damping_coefficient;
            joint_pool [joint_index].is_active = true;
            current_joint_count += 1;
            return joint_index;
        }
    } fprintf (stderr, "Error SJA001: No Remaining Space in Buffer\n");
    return -1;
} void remove_joint (int joint_pool_index) {
    if ((joint_pool_index < 0) || (joint_pool_index >= maximum_joint_count)) {return;}
    if (!joint_pool [joint_pool_index].is_active) {return;}
    joint_pool [joint_pool_index].is_active = false;
    current_joint_count -= 1;
} void apply_force_all_joints (void) {
    for (int joint_index = 0; joint_index < maximum_joint_count; joint_index++) {
        if (!joint_pool [joint_index].is_active) {continue;}
        spring_joint *current_spring_joint = &joint_pool [joint_index];
        //Validate Indices
        if ((current_spring_joint -> object_index_a >= object_count) || (current_spring_joint -> object_index_b >= object_count)) {
            remove_joint (joint_index);
            continue;
        } rigidbody *rigid_body_a = &obj_per_scene [current_spring_joint -> object_index_a];
        rigidbody *rigid_body_b = &obj_per_scene [current_spring_joint -> object_index_b];
        //Vector: A to B
        vector3 displacement_vector = vector3_subtraction (rigid_body_b -> position, rigid_body_a -> position);
        float current_separation_distance = vector3_length (displacement_vector);
        if (current_separation_distance < math_epsilon) {continue;}
        float spring_extension = current_separation_distance - current_spring_joint -> equilibrium_length;
        vector3 spring_axis_direction = vector3_normalisation (displacement_vector);
        //Spring Force (Fs = -kx)
        vector3 restoration_force = vector3_scaling (spring_axis_direction, current_spring_joint -> spring_constant * spring_extension);
        vector3 relative_velocity = vector3_subtraction (rigid_body_b -> velocity, rigid_body_a -> velocity);
        float velocity_along_spring_axis = vector3_dot (relative_velocity, spring_axis_direction);
        //Spring Damping (F = -cv)
        vector3 damping_force = vector3_scaling (spring_axis_direction, current_spring_joint -> damping_coefficient * velocity_along_spring_axis);
        //Net Resultant Force
        vector3 net_joint_force = vector3_addition (restoration_force, damping_force);
        //Apply Equal and Opposite forces
        rb_apply_forces_perfect (rigid_body_a, net_joint_force);
        rb_apply_forces_perfect (rigid_body_b, vector3_scaling (net_joint_force, -1.0f));
    }
} void remove_joints_from_object (int object_index) {
    for (int joint_index = 0; joint_index < maximum_joint_count; joint_index++) {
        if (!joint_pool [joint_index].is_active) {continue;}
        if ((joint_pool [joint_index].object_index_a == object_index) || (joint_pool [joint_index].object_index_b == object_index)) {remove_joint (joint_index);}
    }
} void adjust_joints_after_deletion (int deleted_object_index) {
    for (int joint_index = 0; joint_index < maximum_joint_count; joint_index++) {
        if (!joint_pool [joint_index].is_active) {continue;}
        if (joint_pool [joint_index].object_index_a > deleted_object_index) {
            joint_pool [joint_index].object_index_a -= 1;
        }
        if (joint_pool [joint_index].object_index_b > deleted_object_index) {
            joint_pool [joint_index].object_index_b -= 1;
        }
    }
} void joint_init_pool (void) {
    for (int joint_index = 0; joint_index < maximum_joint_count; joint_index++) {joint_pool [joint_index].is_active = false;}
    current_joint_count = 0;
}

static GLuint joint_vao = 0;
static GLuint joint_vbo = 0;

void spring_joint_render (GLuint shader_program, math4 view_matrix, math4 projection_matrix) {
    int active_count = 0;
    for (int i = 0; i < maximum_joint_count; i++) {
        if (joint_pool [i].is_active) {
            if (joint_pool [i].object_index_a < object_count && joint_pool [i].object_index_b < object_count) {
                active_count++;
            }
        }
    }
    if (active_count == 0) {return;}

    float *vertices = malloc (active_count * 2 * 3 * sizeof (float));
    if (!vertices) {return;}
    int v_idx = 0;
    for (int i = 0; i < maximum_joint_count; i++) {
        if (joint_pool [i].is_active) {
            if (joint_pool [i].object_index_a < object_count && joint_pool [i].object_index_b < object_count) {
                vector3 pos_a = obj_per_scene [joint_pool [i].object_index_a].position;
                vector3 pos_b = obj_per_scene [joint_pool [i].object_index_b].position;
                vertices [v_idx++] = pos_a.x;
                vertices [v_idx++] = pos_a.y;
                vertices [v_idx++] = pos_a.z;
                vertices [v_idx++] = pos_b.x;
                vertices [v_idx++] = pos_b.y;
                vertices [v_idx++] = pos_b.z;
            }
        }
    }

    if (joint_vao == 0) {
        glGenVertexArrays (1, &joint_vao);
        glGenBuffers (1, &joint_vbo);
        glBindVertexArray (joint_vao);
        glBindBuffer (GL_ARRAY_BUFFER, joint_vbo);
        glBufferData (GL_ARRAY_BUFFER, maximum_joint_count * 2 * 3 * sizeof (float), NULL, GL_DYNAMIC_DRAW);
        glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof (float), (void*) 0);
        glEnableVertexAttribArray (0);
        glBindVertexArray (0);
    }

    glBindBuffer (GL_ARRAY_BUFFER, joint_vbo);
    glBufferSubData (GL_ARRAY_BUFFER, 0, active_count * 2 * 3 * sizeof (float), vertices);

    glUseProgram (shader_program);
    float view_matrix_flat_array [16], projection_matrix_flat_array [16];
    math4_to_flat_array (view_matrix, view_matrix_flat_array);
    math4_to_flat_array (projection_matrix, projection_matrix_flat_array);
    glUniformMatrix4fv (glGetUniformLocation (shader_program, "viewframe"), 1, GL_FALSE, view_matrix_flat_array);
    glUniformMatrix4fv (glGetUniformLocation (shader_program, "projection"), 1, GL_FALSE, projection_matrix_flat_array);

    math4 model_matrix = math4_identity ();
    float model_matrix_flat_array [16];
    math4_to_flat_array (model_matrix, model_matrix_flat_array);
    glUniformMatrix4fv (glGetUniformLocation (shader_program, "model"), 1, GL_FALSE, model_matrix_flat_array);

    math3 identity_normal_matrix = math3_identity ();
    float normal_matrix_flat_array [9];
    for (int row_index = 0; row_index < 3; row_index++) {
        for (int column_index = 0; column_index < 3; column_index++) {
            normal_matrix_flat_array [row_index * 3 + column_index] = identity_normal_matrix.matrix [row_index][column_index];
        }
    }
    glUniformMatrix3fv (glGetUniformLocation (shader_program, "normal_matrix"), 1, GL_FALSE, normal_matrix_flat_array);

    // Glowing pink/magenta line for joints
    glUniform3f (glGetUniformLocation (shader_program, "object_colour"), 1.0f, 0.0f, 1.0f);
    glVertexAttrib3f (1, 0.0f, 1.0f, 0.0f);

    glBindVertexArray (joint_vao);
    glDrawArrays (GL_LINES, 0, active_count * 2);
    glBindVertexArray (0);

    free (vertices);
}

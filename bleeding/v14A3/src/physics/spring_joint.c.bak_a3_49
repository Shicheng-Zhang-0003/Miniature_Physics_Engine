#include "../mpe_engine.h"
#include "spring_joint.h"
#include <stdio.h>
#include <stdlib.h>
#include <epoxy/gl.h>

spring_joint joint_pool [MPE_MAX_JOINTS];
int current_joint_count = 0;
int add_joint_by_ids (uint32_t object_id_a, uint32_t object_id_b, float equilibrium_length, float spring_constant, float damping_coefficient) {
    /* A3_PATCH_09_JOINT_IDS */
    if ((object_id_a == 0) || (object_id_b == 0) || (object_id_a == object_id_b)) {return -1;}

    for (int joint_index = 0; joint_index < MPE_MAX_JOINTS; joint_index++) {
        if (!joint_pool [joint_index].is_active) {
            joint_pool [joint_index].object_id_a = object_id_a;
            joint_pool [joint_index].object_id_b = object_id_b;
            joint_pool [joint_index].equilibrium_length = equilibrium_length;
            joint_pool [joint_index].spring_constant = spring_constant;
            joint_pool [joint_index].damping_coefficient = damping_coefficient;
            joint_pool [joint_index].is_active = true;
            current_joint_count += 1;
            return joint_index;
        }
    }

    fprintf (stderr, "Error SJA001: No Remaining Space in Buffer\n");
    return -1;
}

int add_joint (int object_index_a, int object_index_b, float equilibrium_length, float spring_constant, float damping_coefficient) {
    if ((object_index_a < 0) || (object_index_a >= object_count) || (object_index_b < 0) || (object_index_b >= object_count)) {
        fprintf (stderr, "Error SJA002: Invalid joint object index\n");
        return -1;
    }

    uint32_t object_id_a = obj_per_scene [object_index_a].object_id;
    uint32_t object_id_b = obj_per_scene [object_index_b].object_id;

    return add_joint_by_ids (object_id_a, object_id_b, equilibrium_length, spring_constant, damping_coefficient);
}

void remove_joint (int joint_pool_index) {
    if ((joint_pool_index < 0) || (joint_pool_index >= MPE_MAX_JOINTS)) {return;}
    if (!joint_pool [joint_pool_index].is_active) {return;}
    joint_pool [joint_pool_index].is_active = false;
    current_joint_count -= 1;
} void apply_force_all_joints (void) {
    /* A3_PATCH_09_JOINT_IDS */
    for (int joint_index = 0; joint_index < MPE_MAX_JOINTS; joint_index++) {
        if (!joint_pool [joint_index].is_active) {continue;}

        spring_joint *current_spring_joint = &joint_pool [joint_index];

        int object_index_a = scene_find_object_index_by_id (current_spring_joint -> object_id_a);
        int object_index_b = scene_find_object_index_by_id (current_spring_joint -> object_id_b);

        if ((object_index_a < 0) || (object_index_b < 0)) {
            remove_joint (joint_index);
            continue;
        }

        rigidbody *rigid_body_a = &obj_per_scene [object_index_a];
        rigidbody *rigid_body_b = &obj_per_scene [object_index_b];

        vector3 displacement_vector = vector3_subtraction (rigid_body_b -> position, rigid_body_a -> position);
        float current_separation_distance = vector3_length (displacement_vector);

        if (current_separation_distance < math_epsilon) {continue;}

        float spring_extension = current_separation_distance - current_spring_joint -> equilibrium_length;
        vector3 spring_axis_direction = vector3_normalisation (displacement_vector);

        vector3 restoration_force = vector3_scaling (spring_axis_direction, current_spring_joint -> spring_constant * spring_extension);

        vector3 relative_velocity = vector3_subtraction (rigid_body_b -> velocity, rigid_body_a -> velocity);
        float velocity_along_spring_axis = vector3_dot (relative_velocity, spring_axis_direction);

        vector3 damping_force = vector3_scaling (spring_axis_direction, current_spring_joint -> damping_coefficient * velocity_along_spring_axis);
        vector3 net_joint_force = vector3_addition (restoration_force, damping_force);

        rb_apply_forces_perfect (rigid_body_a, net_joint_force);
        rb_apply_forces_perfect (rigid_body_b, vector3_scaling (net_joint_force, -1.0f));
    }
}

void remove_joints_from_object_id (uint32_t object_id) {
    /* A3_PATCH_09_JOINT_IDS */
    if (object_id == 0) {return;}

    for (int joint_index = 0; joint_index < MPE_MAX_JOINTS; joint_index++) {
        if (!joint_pool [joint_index].is_active) {continue;}

        if ((joint_pool [joint_index].object_id_a == object_id) || (joint_pool [joint_index].object_id_b == object_id)) {
            remove_joint (joint_index);
        }
    }
}

void remove_joints_from_object (int object_index) {
    if ((object_index < 0) || (object_index >= object_count)) {return;}
    remove_joints_from_object_id (obj_per_scene [object_index].object_id);
}

void adjust_joints_after_deletion (int deleted_object_index) {
    /* A3_PATCH_09_JOINT_IDS */
    (void) deleted_object_index;
    /* Joints now use stable object IDs, so index repair is unnecessary. */
}

void joint_init_pool (void) {
    for (int joint_index = 0; joint_index < MPE_MAX_JOINTS; joint_index++) {joint_pool [joint_index].is_active = false;}
    current_joint_count = 0;
}

static GLuint joint_vao = 0;
static GLuint joint_vbo = 0;

/* A3_PATCH_27_UNIFORM_CACHE */
static GLuint a3_spring_cached_program = 0;
static GLint a3_spring_uniform_viewframe = -1;
static GLint a3_spring_uniform_projection = -1;
static GLint a3_spring_uniform_model = -1;
static GLint a3_spring_uniform_normal_matrix = -1;
static GLint a3_spring_uniform_object_colour = -1;

static void a3_spring_cache_uniforms (GLuint shader_program) {
    if (shader_program == a3_spring_cached_program) {return;}

    a3_spring_cached_program = shader_program;
    a3_spring_uniform_viewframe = glGetUniformLocation (shader_program, "viewframe");
    a3_spring_uniform_projection = glGetUniformLocation (shader_program, "projection");
    a3_spring_uniform_model = glGetUniformLocation (shader_program, "model");
    a3_spring_uniform_normal_matrix = glGetUniformLocation (shader_program, "normal_matrix");
    a3_spring_uniform_object_colour = glGetUniformLocation (shader_program, "object_colour");
}

/* A3_PATCH_30_MISSING_UNIFORMS */
static GLint a3_spring_uniform_camera_position = -1;
static GLint a3_spring_uniform_light_position = -1;

static void a3_spring_cache_missing_uniforms (GLuint shader_program) {
    static GLuint a3_spring_missing_cached_program = 0;

    if (shader_program == a3_spring_missing_cached_program) {return;}

    a3_spring_missing_cached_program = shader_program;
    a3_spring_uniform_camera_position = glGetUniformLocation (shader_program, "camera_position");
    a3_spring_uniform_light_position = glGetUniformLocation (shader_program, "light_position");
}

void spring_joint_render (GLuint shader_program, math4 view_matrix, math4 projection_matrix) {
    /* A3_PATCH_09_JOINT_IDS */
    int active_count = 0;

    for (int i = 0; i < MPE_MAX_JOINTS; i++) {
        if (!joint_pool [i].is_active) {continue;}

        rigidbody *rb_a = scene_resolve_object_by_id (joint_pool [i].object_id_a);
        rigidbody *rb_b = scene_resolve_object_by_id (joint_pool [i].object_id_b);

        if ((rb_a) && (rb_b)) {active_count++;}
    }

    if (active_count == 0) {return;}
    if (active_count > MPE_MAX_JOINTS) {active_count = MPE_MAX_JOINTS;} /* A3_PATCH_28_PERSISTENT_JOINT_BUFFER */

    static float vertices [MPE_MAX_JOINTS * 2 * 3]; /* A3_PATCH_28_PERSISTENT_JOINT_BUFFER */

    int v_idx = 0;

    for (int i = 0; i < MPE_MAX_JOINTS; i++) {
        if (!joint_pool [i].is_active) {continue;}

        rigidbody *rb_a = scene_resolve_object_by_id (joint_pool [i].object_id_a);
        rigidbody *rb_b = scene_resolve_object_by_id (joint_pool [i].object_id_b);

        if ((rb_a) && (rb_b)) {
            vertices [v_idx++] = rb_a -> position.x;
            vertices [v_idx++] = rb_a -> position.y;
            vertices [v_idx++] = rb_a -> position.z;
            vertices [v_idx++] = rb_b -> position.x;
            vertices [v_idx++] = rb_b -> position.y;
            vertices [v_idx++] = rb_b -> position.z;
        }
    }

    if (joint_vao == 0) {
        glGenVertexArrays (1, &joint_vao);
        glGenBuffers (1, &joint_vbo);
        glBindVertexArray (joint_vao);
        glBindBuffer (GL_ARRAY_BUFFER, joint_vbo);
        glBufferData (GL_ARRAY_BUFFER, MPE_MAX_JOINTS * 2 * 3 * sizeof (float), NULL, GL_DYNAMIC_DRAW);
        glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof (float), (void*) 0);
        glEnableVertexAttribArray (0);
        glBindVertexArray (0);
    }

    glBindBuffer (GL_ARRAY_BUFFER, joint_vbo);
    glBufferSubData (GL_ARRAY_BUFFER, 0, active_count * 2 * 3 * sizeof (float), vertices);

    glUseProgram (shader_program);

    a3_spring_cache_missing_uniforms (shader_program);
    a3_spring_cache_uniforms (shader_program);
    float view_matrix_flat_array [16], projection_matrix_flat_array [16];
    math4_to_flat_array (view_matrix, view_matrix_flat_array);
    math4_to_flat_array (projection_matrix, projection_matrix_flat_array);

    glUniformMatrix4fv (a3_spring_uniform_viewframe, 1, GL_FALSE, view_matrix_flat_array);
    glUniformMatrix4fv (a3_spring_uniform_projection, 1, GL_FALSE, projection_matrix_flat_array);

    math4 model_matrix = math4_identity ();
    float model_matrix_flat_array [16];
    math4_to_flat_array (model_matrix, model_matrix_flat_array);
    glUniformMatrix4fv (a3_spring_uniform_model, 1, GL_FALSE, model_matrix_flat_array);

    math3 identity_normal_matrix = math3_identity ();
    float normal_matrix_flat_array [9];

    for (int row_index = 0; row_index < 3; row_index++) {
        for (int column_index = 0; column_index < 3; column_index++) {
            normal_matrix_flat_array [row_index * 3 + column_index] = identity_normal_matrix.matrix [row_index][column_index];
        }
    }

    glUniformMatrix3fv (a3_spring_uniform_normal_matrix, 1, GL_FALSE, normal_matrix_flat_array);
    glUniform3f (a3_spring_uniform_object_colour, 1.0f, 0.0f, 1.0f);
    /* A3_PATCH_30_MISSING_UNIFORMS */
    glUniform3f (a3_spring_uniform_camera_position, main_camera_fov.position.x, main_camera_fov.position.y, main_camera_fov.position.z);
    glUniform3f (a3_spring_uniform_light_position, 20.0f, 40.0f, 20.0f);
    glVertexAttrib3f (1, 0.0f, 1.0f, 0.0f);

    glBindVertexArray (joint_vao);
    glDrawArrays (GL_LINES, 0, active_count * 2);
    glBindVertexArray (0);
}

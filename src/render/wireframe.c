#include "../mpe_engine.h"
#include "wireframe.h"
#include "sphere_meshing.h"
#include "cube_meshing.h"
#include <epoxy/gl_generated.h>
extern mesh sphere_mesh;
extern mesh cube_mesh;
/* A3_PATCH_27_UNIFORM_CACHE */
static GLuint a3_wire_cached_program = 0;
static GLint a3_wire_uniform_viewframe = -1;
static GLint a3_wire_uniform_projection = -1;
static GLint a3_wire_uniform_model = -1;
static GLint a3_wire_uniform_object_colour = -1;

static void a3_wire_cache_uniforms (GLuint shader_program) {
    if (shader_program == a3_wire_cached_program) {return;}

    a3_wire_cached_program = shader_program;
    a3_wire_uniform_viewframe = glGetUniformLocation (shader_program, "viewframe");
    a3_wire_uniform_projection = glGetUniformLocation (shader_program, "projection");
    a3_wire_uniform_model = glGetUniformLocation (shader_program, "model");
    a3_wire_uniform_object_colour = glGetUniformLocation (shader_program, "object_colour");
}

/* A3_PATCH_30_MISSING_UNIFORMS */
static GLint a3_wire_uniform_normal_matrix = -1;
static GLint a3_wire_uniform_camera_position = -1;
static GLint a3_wire_uniform_light_position = -1;

static void a3_wire_cache_missing_uniforms (GLuint shader_program) {
    static GLuint a3_wire_missing_cached_program = 0;

    if (shader_program == a3_wire_missing_cached_program) {return;}

    a3_wire_missing_cached_program = shader_program;
    a3_wire_uniform_normal_matrix = glGetUniformLocation (shader_program, "normal_matrix");
    a3_wire_uniform_camera_position = glGetUniformLocation (shader_program, "camera_position");
    a3_wire_uniform_light_position = glGetUniformLocation (shader_program, "light_position");
}

void wireframe_render_object (GLuint shader_program, math4 view_matrix, math4 projection_matrix, rigidbody *rigid_body, vector3 wireframe_colour) {
    glUseProgram (shader_program);
    a3_wire_cache_missing_uniforms (shader_program);
    a3_wire_cache_uniforms (shader_program);
    float view_matrix_flat_array [16];
    float projection_matrix_flat_array [16];
    math4_to_flat_array (view_matrix, view_matrix_flat_array);
    math4_to_flat_array (projection_matrix, projection_matrix_flat_array);
    glUniformMatrix4fv (a3_wire_uniform_viewframe,  1, GL_FALSE, view_matrix_flat_array);
    glUniformMatrix4fv (a3_wire_uniform_projection, 1, GL_FALSE, projection_matrix_flat_array);
    math4 translation_matrix = math4_translation (rigid_body -> position);
    math4 rotation_matrix = vector4_to_math4 (rigid_body -> orientation);
    math4 scale_matrix;
    if (rigid_body -> type == object_sphere) {
        float s = rigid_body -> radius * 1.01f;
        scale_matrix = math4_scaling ((vector3) {s, s, s});
    } else {
        scale_matrix = math4_scaling ((vector3) {
            rigid_body -> half_extensions.x * 1.01f,
            rigid_body -> half_extensions.y * 1.01f,
            rigid_body -> half_extensions.z * 1.01f
        });
    } math4 model_matrix = math4_multiplication (translation_matrix, math4_multiplication (rotation_matrix, scale_matrix));
    float model_matrix_flat_array [16];
    math4_to_flat_array (model_matrix, model_matrix_flat_array);
    glUniform3f (a3_wire_uniform_object_colour, wireframe_colour.x, wireframe_colour.y, wireframe_colour.z);
    glUniformMatrix4fv (a3_wire_uniform_model, 1, GL_FALSE, model_matrix_flat_array);
    /* A3_PATCH_30_MISSING_UNIFORMS */
    math3 a3_wire_normal_matrix = math3_identity ();
    float a3_wire_normal_matrix_flat [9];

    for (int row_index = 0; row_index < 3; row_index++) {
        for (int column_index = 0; column_index < 3; column_index++) {
            a3_wire_normal_matrix_flat [row_index * 3 + column_index] = a3_wire_normal_matrix.matrix [row_index][column_index];
        }
    }

    glUniformMatrix3fv (a3_wire_uniform_normal_matrix, 1, GL_FALSE, a3_wire_normal_matrix_flat);
    glUniform3f (a3_wire_uniform_camera_position, main_camera_fov.position.x, main_camera_fov.position.y, main_camera_fov.position.z);
    glUniform3f (a3_wire_uniform_light_position, 20.0f, 40.0f, 20.0f);
    if (rigid_body -> type == object_sphere) {
        glBindVertexArray (sphere_mesh.vertex_array_object);
        glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, sphere_mesh.wireframe_element_buffer_object);
        glDrawElements (GL_LINES, sphere_mesh.wireframe_index_count, GL_UNSIGNED_INT, 0);
        glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, sphere_mesh.element_buffer_object);
    } else {
        glBindVertexArray (cube_mesh.vertex_array_object);
        glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, cube_mesh.wireframe_element_buffer_object);
        glDrawElements (GL_LINES, cube_mesh.wireframe_index_count, GL_UNSIGNED_INT, 0);
        glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, cube_mesh.element_buffer_object);
    } glBindVertexArray (0);
} void wireframe_render_selected_object (GLuint shader_program, math4 view_matrix, math4 projection_matrix) {
    if ((selected_object < 0) || (selected_object >= object_count)) {return;}
    //Yellow outline (Selected Object Visibility)
    wireframe_render_object (shader_program, view_matrix, projection_matrix, &obj_per_scene [selected_object], (vector3) {1.0f, 1.0f, 0.0f});
}

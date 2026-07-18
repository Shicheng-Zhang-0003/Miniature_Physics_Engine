#include "wireframe.h"
#include "../../stage2/interface/sphere_object/meshing/sphere_meshing.h"
#include "../../stage2/interface/cube_object/meshing/cube_meshing.h"
#include <epoxy/gl_generated.h>
extern mesh sphere_mesh;
extern mesh cube_mesh;
void wireframe_render_object (GLuint shader_program, math4 view_matrix, math4 projection_matrix, rigidbody *rigid_body, vector3 wireframe_colour) {
    glUseProgram (shader_program);
    float view_matrix_flat_array [16];
    float projection_matrix_flat_array [16];
    math4_to_flat_array (view_matrix, view_matrix_flat_array);
    math4_to_flat_array (projection_matrix, projection_matrix_flat_array);
    glUniformMatrix4fv (glGetUniformLocation (shader_program, "viewframe"),  1, GL_FALSE, view_matrix_flat_array);
    glUniformMatrix4fv (glGetUniformLocation (shader_program, "projection"), 1, GL_FALSE, projection_matrix_flat_array);
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
    glUniform3f (glGetUniformLocation (shader_program, "object_colour"), wireframe_colour.x, wireframe_colour.y, wireframe_colour.z);
    glUniformMatrix4fv (glGetUniformLocation (shader_program, "model"), 1, GL_FALSE, model_matrix_flat_array);
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

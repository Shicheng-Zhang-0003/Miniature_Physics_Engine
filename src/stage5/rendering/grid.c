#include "grid.h"
#include <epoxy/gl_generated.h>
#include <stdlib.h>
void grid_init (grid_mesh *grid_mesh_object, int half_extent, int cell_spacing) {
    //Count the lines: one iteration per X axis, one along Z axis
    int grid_line_steps = (half_extent * 2) / cell_spacing + 1;
    //Each line (2 vertices, 3 floats)
    int vertex_float_count = grid_line_steps * 4 * 3; //(* 2 * 2 iteration per axis)
    float *vertex_data = malloc (vertex_float_count * sizeof (float));
    int vertex_index = 0;
    for (int step_coordinate = -half_extent; step_coordinate <= half_extent; step_coordinate += cell_spacing) {
        //Line along the Z axis at X = step_coordinate;
        vertex_data [vertex_index++] = (float) step_coordinate;
        vertex_data [vertex_index++] = 0.0;
        vertex_data [vertex_index++] = (float) -half_extent;
        vertex_data [vertex_index++] = (float) step_coordinate;
        vertex_data [vertex_index++] = 0.0;
        vertex_data [vertex_index++] = (float) half_extent;
        //Line along X axis at position of Z = step_coordinate
        vertex_data [vertex_index++] = (float) -half_extent;
        vertex_data [vertex_index++] = 0.0;
        vertex_data [vertex_index++] = (float) step_coordinate;
        vertex_data [vertex_index++] = (float) half_extent;
        vertex_data [vertex_index++] = 0.0;
        vertex_data [vertex_index++] = (float) step_coordinate;
    } grid_mesh_object -> line_vertex_count = vertex_index / 3;
    glGenVertexArrays (1, &grid_mesh_object -> vertex_array_object);
    glGenBuffers (1, &grid_mesh_object -> vertex_buffer_object);
    glBindVertexArray (grid_mesh_object -> vertex_array_object);
    glBindBuffer (GL_ARRAY_BUFFER, grid_mesh_object -> vertex_buffer_object);
    glBufferData (GL_ARRAY_BUFFER, vertex_index * sizeof (float), vertex_data, GL_STATIC_DRAW);
    glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof (float), (void*) 0);
    glEnableVertexAttribArray (0);
    glBindVertexArray (0);
    free (vertex_data);
} void grid_render (grid_mesh *grid_mesh_object, GLuint shader_program, math4 view_matrix, math4 projection_matrix) {
    glUseProgram (shader_program);
    float view_matrix_flat_array [16], projection_matrix_flat_array [16];
    math4_to_flat_array (view_matrix, view_matrix_flat_array);
    math4_to_flat_array (projection_matrix, projection_matrix_flat_array);
    glUniformMatrix4fv (glGetUniformLocation (shader_program, "viewframe"), 1, GL_FALSE, view_matrix_flat_array);
    glUniformMatrix4fv (glGetUniformLocation (shader_program, "projection"), 1, GL_FALSE, projection_matrix_flat_array);
    //Identity Model Matrix (sits at the (0, 0, 0, 0w))
    math4 model_matrix = math4_identity ();
    float model_matrix_flat_array [16];
    math4_to_flat_array (model_matrix, model_matrix_flat_array);
    glUniformMatrix4fv (glGetUniformLocation (shader_program, "model"), 1, GL_FALSE, model_matrix_flat_array);
    //Normal Matrix (Identity for the Static Floor)
    math3 identity_normal_matrix = math3_identity ();
    float normal_matrix_flat_array [9];
    for (int row_index = 0; row_index < 3; row_index++) {
        for (int column_index = 0; column_index < 3; column_index++) {normal_matrix_flat_array [row_index * 3 + column_index] = identity_normal_matrix.matrix [row_index][column_index];}
    } glUniformMatrix3fv (glGetUniformLocation (shader_program, "normal_matrix"), 1, GL_FALSE, normal_matrix_flat_array);
    glUniform3f (glGetUniformLocation (shader_program, "object_colour"), 0.3, 0.3, 0.3);
    const float grid_surface_normal_x = 0.0;
    const float grid_surface_normal_y = 1.0;
    const float grid_surface_normal_z = 0.0;
    glVertexAttrib3f (1, grid_surface_normal_x, grid_surface_normal_y, grid_surface_normal_z); // Constant normal pointing up for the grid
    glBindVertexArray (grid_mesh_object -> vertex_array_object);
    glDrawArrays (GL_LINES, 0, grid_mesh_object -> line_vertex_count);
    glBindVertexArray (0);
}

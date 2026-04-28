#include "grid.h"
#include <epoxy/gl_generated.h>
#include <stdlib.h>
void grid_init (grid_mesh *gm, int half_buffer, int spacing) {
    //Count the lines: one iteration per X axis, one along Z axis
    int steps = (half_buffer * 2) / spacing + 1;
    //Each line (2 vertices, 3 floats)
    int total_float_count = steps * 4 * 3; //(* 2 * 2 iteration per axis)
    float *vertices = malloc (total_float_count * sizeof (float));
    int idx = 0;
    for (int step = -half_buffer; step <= half_buffer; step += spacing) {
        //Line along the Z axis at X = step;
        vertices [idx++] = (float) step;
        vertices [idx++] = 0.0;
        vertices [idx++] = (float) -half_buffer;
        vertices [idx++] = (float) step;
        vertices [idx++] = 0.0;
        vertices [idx++] = (float) half_buffer;
        //Line along X axis at position of Z = step
        vertices [idx++] = (float) -half_buffer;
        vertices [idx++] = 0.0;
        vertices [idx++] = (float) step;
        vertices [idx++] = (float) half_buffer;
        vertices [idx++] = 0.0;
        vertices [idx++] = (float) step;
    } gm -> line_count = idx / 3;
    glGenVertexArrays (1, &gm -> vao);
    glGenBuffers (1, &gm -> vbo);
    glBindVertexArray (gm -> vao);
    glBindBuffer (GL_ARRAY_BUFFER, gm -> vbo);
    glBufferData (GL_ARRAY_BUFFER, idx * sizeof (float), vertices, GL_STATIC_DRAW);
    glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof (float), (void*) 0);
    glEnableVertexAttribArray (0);
    glBindVertexArray (0);
    free (vertices);
} void grid_render (grid_mesh *gm, GLuint shader_program_call, math4 view, math4 projection) {
    glUseProgram (shader_program_call);
    float view_array [16], projection_array [16];
    math4_to_flat_array (view, view_array);
    math4_to_flat_array (projection, projection_array);
    glUniformMatrix4fv (glGetUniformLocation (shader_program_call, "viewframe"), 1, GL_FALSE, view_array);
    glUniformMatrix4fv (glGetUniformLocation (shader_program_call, "projection"), 1, GL_FALSE, projection_array);
    //Identity Model Matrix (sits at the (0, 0, 0, 0w))
    math4 model = math4_identity ();
    float model_array [16];
    math4_to_flat_array (model, model_array);
    glUniformMatrix4fv (glGetUniformLocation (shader_program_call, "model"), 1, GL_FALSE, model_array);
    glUniform3f (glGetUniformLocation (shader_program_call, "object_colour"), 0.3, 0.3, 0.3);
    const float grid_surface_normal_x = 0.0;
    const float grid_surface_normal_y = 1.0;
    const float grid_surface_normal_z = 0.0;
    glVertexAttrib3f (1, grid_surface_normal_x, grid_surface_normal_y, grid_surface_normal_z); // Constant normal pointing up for the grid
    glBindVertexArray (gm -> vao);
    glDrawArrays (GL_LINES, 0, gm -> line_count);
    glBindVertexArray (0);
}

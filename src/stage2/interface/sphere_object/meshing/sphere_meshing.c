#include "sphere_meshing.h"
#include <complex.h>
#include <epoxy/gl_generated.h>
#include <stdlib.h>
#include <math.h>
void init_sm_system (mesh *mesh_object, int horizontal_sections, int vertical_stacks) {
    //Calculate the number of vertices we require for rendering
    // (vertical_stacks + 1)(horizontal_sections + 1) total amount fo vertices required
    int vertex_count = (vertical_stacks + 1) * (horizontal_sections + 1);
    float *vertex_data = malloc (vertex_count * 6 * sizeof (float));
    //Generate Vertex COordinates for rendering
    int vertex_index = 0; //Assembly notation idx bitshifting
    for (int stack_step = 0; stack_step <= vertical_stacks; stack_step++) {
        //Phi value goes from .5pi to -.5pi (Max to Min)
        float phi_angle = math_pi / 2 - stack_step * math_pi / vertical_stacks;
        for (int section_step = 0; section_step <= horizontal_sections; section_step++) {
            //Theta goes in radians all around circulat object
            float theta_angle = section_step * 2 * math_pi / horizontal_sections;
            //Transpose Spherical objects to a cartesian plan (r = 1 as example)
            float position_x = cosf (phi_angle) * cosf (theta_angle);
            float position_y = sinf (phi_angle);
            float position_z = cosf (phi_angle) * sinf (theta_angle);
            //Positional Values
            vertex_data [vertex_index++] = position_x;
            vertex_data [vertex_index++] = position_y;
            vertex_data [vertex_index++] = position_z;
            //Normal Rendering (Unit Sphere, x ^ 2 + y ^ 2 + z ^ 2 = 1)
            vertex_data [vertex_index++] = position_x;
            vertex_data [vertex_index++] = position_y;
            vertex_data [vertex_index++] = position_z;
        }
    } //Generate Indices
    //Calculate indices for 3 orthogonal equators (Wireframe)
    int wireframe_indices_size = (horizontal_sections + 2 * vertical_stacks + 2 * vertical_stacks) * 2;
    unsigned int *wireframe_indices = malloc (wireframe_indices_size * sizeof (unsigned int));
    int wireframe_index = 0;
    //Horizontal (XZ plane)
    int middle_stack_index = vertical_stacks / 2;
    for (int section_index = 0; section_index < horizontal_sections; section_index++) {
        wireframe_indices [wireframe_index++] = middle_stack_index * (horizontal_sections + 1) + section_index;
        wireframe_indices [wireframe_index++] = middle_stack_index * (horizontal_sections + 1) + section_index + 1;
    }
    //Vertical 1 (XY plane)
    int theta_0_index = 0;
    int theta_pi_index = horizontal_sections / 2;
    for (int stack_index = 0; stack_index < vertical_stacks; stack_index++) {
        wireframe_indices [wireframe_index++] = stack_index * (horizontal_sections + 1) + theta_0_index;
        wireframe_indices [wireframe_index++] = (stack_index + 1) * (horizontal_sections + 1) + theta_0_index;
        wireframe_indices [wireframe_index++] = stack_index * (horizontal_sections + 1) + theta_pi_index;
        wireframe_indices [wireframe_index++] = (stack_index + 1) * (horizontal_sections + 1) + theta_pi_index;
    }
    //Vertical 2 (YZ plane)
    int theta_pi_half_index = horizontal_sections / 4;
    int theta_three_pi_half_index = 3 * horizontal_sections / 4;
    for (int stack_index = 0; stack_index < vertical_stacks; stack_index++) {
        wireframe_indices [wireframe_index++] = stack_index * (horizontal_sections + 1) + theta_pi_half_index;
        wireframe_indices [wireframe_index++] = (stack_index + 1) * (horizontal_sections + 1) + theta_pi_half_index;
        wireframe_indices [wireframe_index++] = stack_index * (horizontal_sections + 1) + theta_three_pi_half_index;
        wireframe_indices [wireframe_index++] = (stack_index + 1) * (horizontal_sections + 1) + theta_three_pi_half_index;
    }
    mesh_object -> wireframe_index_count = wireframe_index;
    //Quads in each grid is split into dual triangle configurations
    mesh_object -> index_count = vertical_stacks * horizontal_sections * 6;
    unsigned int *element_indices = malloc (mesh_object -> index_count * sizeof (unsigned int));
    int element_index = 0;
    for (int stack_index = 0; stack_index < vertical_stacks; stack_index++) {
        int current_row_start = stack_index * (horizontal_sections + 1);
        int next_row_start = current_row_start + horizontal_sections + 1;
        for (int section_index = 0; section_index < horizontal_sections; section_index++, current_row_start++, next_row_start++) {
            //First Triangle Drawn
            element_indices [element_index++] = current_row_start;
            element_indices [element_index++] = next_row_start;
            element_indices [element_index++] = current_row_start + 1;
            //Second Triangle
            element_indices [element_index++] = current_row_start + 1;
            element_indices [element_index++] = next_row_start;
            element_indices [element_index++] = next_row_start + 1;
        }
    } //Upload to GPU rendering
    glGenVertexArrays (1, &mesh_object -> vertex_array_object);
    glGenBuffers (1, &mesh_object -> vertex_buffer_object);
    glGenBuffers (1, &mesh_object -> element_buffer_object);
    glBindVertexArray (mesh_object -> vertex_array_object);
    //Vertice Upload Count
    glBindBuffer (GL_ARRAY_BUFFER, mesh_object -> vertex_buffer_object);
    glBufferData (GL_ARRAY_BUFFER, vertex_count * 6 * sizeof (float), vertex_data, GL_STATIC_DRAW);
    //Indice Upload Count
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, mesh_object -> element_buffer_object);
    glBufferData (GL_ELEMENT_ARRAY_BUFFER, mesh_object -> index_count * sizeof (unsigned int), element_indices, GL_STATIC_DRAW);
    //OpenGL attribute set --> affix certain attributes to 3D position statements
    glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof (float), (void*) 0);
    glEnableVertexAttribArray (0);
    //Normal attribute
    glVertexAttribPointer (1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof (float), (void*) (3 * sizeof (float)));
    glEnableVertexAttribArray (1);
    //Upload Wireframe EBO
    glGenBuffers (1, &mesh_object -> wireframe_element_buffer_object);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, mesh_object -> wireframe_element_buffer_object);
    glBufferData (GL_ELEMENT_ARRAY_BUFFER, mesh_object -> wireframe_index_count * sizeof (unsigned int), wireframe_indices, GL_STATIC_DRAW);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, mesh_object -> element_buffer_object);
    glBindVertexArray (0);
    //Other attribute set
    //Free Malloced Memory Addresses
    free (wireframe_indices);
    free (element_indices);
    free (vertex_data);
}  void render_sphere_object (mesh *mesh_object, rigidbody *rigid_body) {
    (void) rigid_body;
    //Sphere Preloaded GPU data
    glBindVertexArray (mesh_object -> vertex_array_object);
    //Draw Triangles
    //GL_TRIANGLES for GPU to run indices 3 at a time
    glDrawElements (GL_TRIANGLES, mesh_object -> index_count, GL_UNSIGNED_INT, 0);
    glBindVertexArray (0);
}

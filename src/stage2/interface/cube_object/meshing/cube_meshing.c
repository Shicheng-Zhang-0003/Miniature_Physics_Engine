#include "stage2/interface/sphere_object/meshing/sphere_meshing.h"
#include "cube_meshing.h"
#include <epoxy/gl.h>
#include <epoxy/gl_generated.h>
mesh cube_mesh;
void cube_meshing_init (void) {
    // 24 face vertices (position + normal, 6 floats each) — unchanged
    // + 12 cross-section ring vertices (positions only, normals zeroed)
    // = 36 vertices total in the VBO
    float vertices [] = {
        // Face vertices (24)
        -1,-1,-1,  0, 0,-1,   1,-1,-1,  0,0,-1,   1,1,-1,  0,0,-1,  -1,1,-1, 0,0,-1,
        -1,-1, 1,  0,0, 1,   1,-1, 1,  0,0, 1,   1,1, 1,  0,0, 1,  -1,1, 1, 0,0, 1,
        -1,-1,-1, -1,0,0,   -1,1,-1, -1,0,0,   -1,1,1, -1,0,0,  -1,-1,1,-1,0,0,
         1,-1,-1,  1,0,0,    1,1,-1,  1,0,0,    1,1,1,  1,0,0,   1,-1,1, 1,0,0,
        -1,-1,-1,  0,-1,0,  -1,-1,1,  0,-1,0,   1,-1,1,  0,-1,0,  1,-1,-1,0,-1,0,
        -1, 1,-1,  0, 1,0,  -1,1,1,   0,1,0,    1,1,1,   0,1,0,   1,1,-1, 0,1,0,
        // XZ ring (y = 0) — indices 24-27
        -1,0,-1, 0,0,0,   1,0,-1, 0,0,0,   1,0,1, 0,0,0,  -1,0,1, 0,0,0,
        // XY ring (z = 0) — indices 28-31
        -1,-1,0, 0,0,0,   1,-1,0, 0,0,0,   1,1,0, 0,0,0,  -1,1,0, 0,0,0,
        // YZ ring (x = 0) — indices 32-35
         0,-1,-1, 0,0,0,   0,1,-1, 0,0,0,   0,1,1, 0,0,0,  0,-1,1, 0,0,0
    }; unsigned int indices [] = {
        0,1,2, 2,3,0,
        4,5,6, 6,7,4,
        8,9,10,10,11,8,
        12,13,14,14,15,12,
        16,17,18,18,19,16,
        20,21,22,22,23,20
    }; glGenVertexArrays (1, &cube_mesh.vertex_array_object);
    glGenBuffers (1, &cube_mesh.vertex_buffer_object);
    glGenBuffers (1, &cube_mesh.element_buffer_object);
    glBindVertexArray (cube_mesh.vertex_array_object);
    glBindBuffer (GL_ARRAY_BUFFER, cube_mesh.vertex_buffer_object);
    glBufferData (GL_ARRAY_BUFFER, sizeof (vertices), vertices, GL_STATIC_DRAW);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, cube_mesh.element_buffer_object);
    glBufferData (GL_ELEMENT_ARRAY_BUFFER, sizeof (indices), indices, GL_STATIC_DRAW);
    glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof (float), (void*) 0);
    glEnableVertexAttribArray (0);
    glVertexAttribPointer (1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof (float), (void*) (3 * sizeof (float)));
    glEnableVertexAttribArray (1);
    // Wireframe EBO: 3 axis rings, 4 edges each, 2 indices per edge = 24 indices
    // Each ring sits at the midplane of its axis pair — shows rotation around that axis
    unsigned int wireframe_indices [] = {
        24,25, 25,26, 26,27, 27,24,   // XZ ring — rotate to see Y-axis spin
        28,29, 29,30, 30,31, 31,28,   // XY ring — rotate to see Z-axis spin
        32,33, 33,34, 34,35, 35,32    // YZ ring — rotate to see X-axis spin
    }; glGenBuffers (1, &cube_mesh.wireframe_element_buffer_object);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, cube_mesh.wireframe_element_buffer_object);
    glBufferData (GL_ELEMENT_ARRAY_BUFFER, sizeof (wireframe_indices), wireframe_indices, GL_STATIC_DRAW);
    // Restore solid EBO before unbinding
    glBindBuffer  (GL_ELEMENT_ARRAY_BUFFER, cube_mesh.element_buffer_object);
    glBindVertexArray (0);
    cube_mesh.index_count = 36;
    cube_mesh.wireframe_index_count = 24;
} void render_cube_object (mesh *mesh_object, rigidbody *rigid_body) {
    (void) rigid_body;
    glBindVertexArray (mesh_object -> vertex_array_object);
    glDrawElements (GL_TRIANGLES, mesh_object -> index_count, GL_UNSIGNED_INT, 0);
    glBindVertexArray (0);
}

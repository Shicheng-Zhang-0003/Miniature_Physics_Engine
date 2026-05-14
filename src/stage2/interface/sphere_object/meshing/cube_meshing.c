#include "cube_meshing.h"
#include <epoxy/gl.h>
#include <epoxy/gl_generated.h>
mesh cube_mesh;
void cube_meshing_init (void) {
    // Cube vertices: position, 3 fold + normal, 3 fold
    float vertices [] = {
        -1,-1,-1,  0, 0,-1,   1,-1,-1,  0,0,-1,   1,1,-1,  0,0,-1,  -1,1,-1, 0,0,-1,
        -1,-1, 1,  0,0, 1,   1,-1, 1,  0,0, 1,   1,1, 1,  0,0, 1,  -1,1, 1, 0,0, 1,
        -1,-1,-1, -1,0,0,   -1,1,-1, -1,0,0,   -1,1,1, -1,0,0,  -1,-1,1,-1,0,0,
         1,-1,-1,  1,0,0,    1,1,-1,  1,0,0,    1,1,1,  1,0,0,   1,-1,1, 1,0,0,
        -1,-1,-1,  0,-1,0,  -1,-1,1,  0,-1,0,   1,-1,1,  0,-1,0,  1,-1,-1,0,-1,0,
        -1, 1,-1,  0, 1,0,  -1,1,1,   0,1,0,    1,1,1,   0,1,0,   1,1,-1, 0,1,0
    }; unsigned int indices [] = {
        0,1,2, 2,3,0,
        4,5,6, 6,7,4,
        8,9,10,10,11,8,
        12,13,14,14,15,12,
        16,17,18,18,19,16,
        20,21,22,22,23,20
    }; glGenVertexArrays(1, &cube_mesh.vertex_array_object);
    glGenBuffers(1, &cube_mesh.vertex_buffer_object);
    glGenBuffers(1, &cube_mesh.element_buffer_object);
    glBindVertexArray(cube_mesh.vertex_array_object);
    glBindBuffer(GL_ARRAY_BUFFER, cube_mesh.vertex_buffer_object);
    glBufferData(GL_ARRAY_BUFFER, sizeof (vertices), vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cube_mesh.element_buffer_object);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof (indices), indices, GL_STATIC_DRAW);
    glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 6*sizeof (float), (void*)0);
    glEnableVertexAttribArray (0);
    glVertexAttribPointer (1, 3, GL_FLOAT, GL_FALSE, 6*sizeof (float), (void*) (3*sizeof (float)));
    glEnableVertexAttribArray (1);
    glBindVertexArray (0);
    cube_mesh.index_count = 36;
} void render_cube_object (mesh *mesh_object, rigidbody *rigid_body) {
    (void) rigid_body;
    glBindVertexArray (mesh_object -> vertex_array_object);
    glDrawElements (GL_TRIANGLES, mesh_object -> index_count, GL_UNSIGNED_INT, 0);
    glBindVertexArray (0);
}

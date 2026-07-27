#include "../mpe_engine.h"
#include "cube_meshing.h"
#include <epoxy/gl.h>
#include <epoxy/gl_generated.h>
mesh cube_mesh;
void cube_meshing_init (void) {
    float vertices [] = {
        -1,-1,-1,  0, 0,-1,   1,-1,-1,  0,0,-1,   1,1,-1,  0,0,-1,  -1,1,-1, 0,0,-1,
        -1,-1, 1,  0,0, 1,   1,-1, 1,  0,0, 1,   1,1, 1,  0,0, 1,  -1,1, 1, 0,0, 1,
        -1,-1,-1, -1,0,0,   -1,1,-1, -1,0,0,   -1,1,1, -1,0,0,  -1,-1,1,-1,0,0,
         1,-1,-1,  1,0,0,    1,1,-1,  1,0,0,    1,1,1,  1,0,0,   1,-1,1, 1,0,0,
        -1,-1,-1,  0,-1,0,  -1,-1,1,  0,-1,0,   1,-1,1,  0,-1,0,  1,-1,-1,0,-1,0,
        -1, 1,-1,  0, 1,0,  -1,1,1,   0,1,0,    1,1,1,   0,1,0,   1,1,-1, 0,1,0,
        -1,0,-1, 0,0,0,   1,0,-1, 0,0,0,   1,0,1, 0,0,0,  -1,0,1, 0,0,0,
        -1,-1,0, 0,0,0,   1,-1,0, 0,0,0,   1,1,0, 0,0,0,  -1,1,0, 0,0,0,
         0,-1,-1, 0,0,0,   0,1,-1, 0,0,0,   0,1,1, 0,0,0,  0,-1,1, 0,0,0
    }; unsigned int indices [] = {
        0,1,2, 2,3,0, 4,5,6, 6,7,4, 8,9,10,10,11,8, 12,13,14,14,15,12, 16,17,18,18,19,16, 20,21,22,22,23,20
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
    unsigned int wireframe_indices [] = {
        24,25, 25,26, 26,27, 27,24, 28,29, 29,30, 30,31, 31,28, 32,33, 33,34, 34,35, 35,32
    }; glGenBuffers (1, &cube_mesh.wireframe_element_buffer_object);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, cube_mesh.wireframe_element_buffer_object);
    glBufferData (GL_ELEMENT_ARRAY_BUFFER, sizeof (wireframe_indices), wireframe_indices, GL_STATIC_DRAW);
    glGenBuffers (1, &cube_mesh.instance_vbo);
    glBindBuffer (GL_ARRAY_BUFFER, cube_mesh.instance_vbo);
    glBufferData (GL_ARRAY_BUFFER, MPE_MAX_BODIES * 19 * sizeof (float), NULL, GL_DYNAMIC_DRAW);
    for (int i = 0; i < 4; i++) {
        glVertexAttribPointer (2 + i, 4, GL_FLOAT, GL_FALSE, 19 * sizeof (float), (void*) (i * 4 * sizeof (float)));
        glEnableVertexAttribArray (2 + i);
        glVertexAttribDivisor (2 + i, 1);
    } glVertexAttribPointer (6, 3, GL_FLOAT, GL_FALSE, 19 * sizeof (float), (void*) (16 * sizeof (float)));
    glEnableVertexAttribArray (6);
    glVertexAttribDivisor (6, 1);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, cube_mesh.element_buffer_object);
    glBindVertexArray (0);
    cube_mesh.index_count = 36;
    cube_mesh.wireframe_index_count = 24;
} 
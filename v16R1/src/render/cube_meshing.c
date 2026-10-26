/* GTK4-PREP: zero GUI headers in render. */
#include "cube_meshing.h"
#include "../config/mpe_constants.h"
#include <epoxy/gl.h>
#include <epoxy/gl_generated.h>
mesh cube_mesh;
void cube_meshing_init (void) {
    float vertices [] = {-1, -1, -1, 0, 0, -1, 1, -1, -1, 0, 0, -1, 1, 1, -1, 0, 0, -1, -1, 1, -1, 0, 0, -1,
                        -1, -1, 1, 0, 0, 1, 1, -1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, -1, 1, 1, 0, 0, 1,
                        -1, -1, -1, -1, 0, 0, -1, 1, -1, -1, 0, 0, -1, 1, 1, -1, 0, 0, -1, -1, 1, -1, 0, 0,
                        1, -1, -1, 1, 0, 0, 1, 1, -1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, -1, 1, 1, 0, 0,
                        -1, -1, -1, 0, -1, 0, -1, -1, 1, 0, -1, 0, 1, -1, 1, 0, -1, 0, 1, -1, -1, 0, -1, 0,
                        -1, 1, -1, 0, 1, 0, -1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, -1, 0, 1, 0,
                        -1, 0, -1, 0, 0, 0, 1, 0, -1, 0, 0, 0, 1, 0, 1, 0, 0, 0, -1, 0, 1, 0, 0, 0,
                        -1, -1, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, -1, 1, 0, 0, 0, 0,
                        0, -1, -1, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, -1, 1, 0, 0, 0};
    unsigned int indices [] = {0, 1, 2, 2, 3, 0, 4, 5, 6, 6, 7, 4, 8, 9, 10, 10, 11, 8,
                              12, 13, 14, 14, 15, 12, 16, 17, 18, 18, 19, 16, 20, 21, 22, 22, 23, 20};
    glGenVertexArrays (1, &cube_mesh.vertex_array_object);
    glGenBuffers (1, &cube_mesh.vertex_buffer_object);
    glGenBuffers (1, &cube_mesh.element_buffer_object);
    glBindVertexArray (cube_mesh.vertex_array_object);
    glBindBuffer (GL_ARRAY_BUFFER, cube_mesh.vertex_buffer_object);
    glBufferData (GL_ARRAY_BUFFER, sizeof (vertices), vertices, GL_STATIC_DRAW);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, cube_mesh.element_buffer_object);
    glBufferData (GL_ELEMENT_ARRAY_BUFFER, sizeof (indices), indices, GL_STATIC_DRAW);
    glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof (float), (void *) 0);
    glEnableVertexAttribArray (0);
    glVertexAttribPointer (1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof (float), (void *) (3 * sizeof (float)));
    glEnableVertexAttribArray (1);
    /* DESPOT-2026-10-20 (WIREFRAME): indices 24..35 are the three INTERIOR
     * cross-section quads of the vertex INFILL rows (y=0 per row G, z=0
     * row H, x=0 row I — normals 0,0,0 plane slices), NOT the corners. The
     * real 12 edges must run through the axis-face corner vertices:
     * bottom face y-1 = 16,17,18,19, top face y+1 = 20,21,22,23 (row E/F).
     * 8 + 4 verticals = 24 indices. Old drawing wrapped the mid-height
     * planes, so the wireframe outline never touched the corners it is
     * meant to trace. */
    unsigned int wireframe_indices [] = {16, 17, 17, 18, 18, 19, 19, 16, 20, 21, 21, 22,
                                        22, 23, 23, 20, 16, 20, 17, 21, 18, 22, 19, 23};
    glGenBuffers (1, &cube_mesh.wireframe_element_buffer_object);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, cube_mesh.wireframe_element_buffer_object);
    glBufferData (GL_ELEMENT_ARRAY_BUFFER, sizeof (wireframe_indices), wireframe_indices, GL_STATIC_DRAW);
    glGenBuffers (1, &cube_mesh.instance_vbo);
    glBindBuffer (GL_ARRAY_BUFFER, cube_mesh.instance_vbo);
    glBufferData (GL_ARRAY_BUFFER, mpe_max_bodies * 19 * sizeof (float), NULL, GL_DYNAMIC_DRAW);
    for (int i = 0; i < 4; i++) {
        glVertexAttribPointer (2 + i, 4, GL_FLOAT, GL_FALSE, 19 * sizeof (float), (void *) (i * 4 * sizeof (float)));
        glEnableVertexAttribArray (2 + i);
        glVertexAttribDivisor (2 + i, 1);
    } glVertexAttribPointer (6, 3, GL_FLOAT, GL_FALSE, 19 * sizeof (float), (void *) (16 * sizeof (float)));
    glEnableVertexAttribArray (6);
    glVertexAttribDivisor (6, 1);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, cube_mesh.element_buffer_object);
    glBindVertexArray (0);
    cube_mesh.index_count = 36;
    cube_mesh.wireframe_index_count = 24;
}

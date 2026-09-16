#include "../mpe_engine.h"
#include "cylinder_meshing.h"
#include <epoxy/gl_generated.h>
#include <stdlib.h>
#include <math.h>

void init_cylinder_system(mesh *mesh_object, int radial_segments) {
    if (radial_segments < 8) {
        radial_segments = 8;
    }
    int ring_verts = radial_segments + 1;
    /* Rings: x=-1 barrel, x=+1 barrel, x=-1 cap disc, x=+1 cap disc, + 2 cap centers. */
    int vertex_count = ring_verts * 4 + 2;
    float *vertex_data = malloc((size_t) vertex_count * 6 * sizeof(float));
    int vi = 0;
    for (int ring = 0; ring < 4; ring++) {
        float x = (ring == 0 || ring == 2) ? -1.0f : 1.0f;
        for (int s = 0; s <= radial_segments; s++) {
            float a = (float) s * 2.0f * math_pi / (float) radial_segments;
            float y = cosf(a);
            float z = sinf(a);
            vertex_data[vi++] = x;
            vertex_data[vi++] = y;
            vertex_data[vi++] = z;
            if (ring < 2) {
                /* Barrel normal: radial. */
                vertex_data[vi++] = 0.0f;
                vertex_data[vi++] = y;
                vertex_data[vi++] = z;
            } else {
                /* Cap normal: axial. */
                vertex_data[vi++] = x;
                vertex_data[vi++] = 0.0f;
                vertex_data[vi++] = 0.0f;
            }
        }
    }
    /* Cap centers (flat shading anchors for fans). */
    vertex_data[vi++] = -1.0f;
    vertex_data[vi++] = 0.0f;
    vertex_data[vi++] = 0.0f;
    vertex_data[vi++] = -1.0f;
    vertex_data[vi++] = 0.0f;
    vertex_data[vi++] = 0.0f;
    vertex_data[vi++] = 1.0f;
    vertex_data[vi++] = 0.0f;
    vertex_data[vi++] = 0.0f;
    vertex_data[vi++] = 1.0f;
    vertex_data[vi++] = 0.0f;
    vertex_data[vi++] = 0.0f;

    int ring0 = 0, ring1 = ring_verts, ring2 = ring_verts * 2, ring3 = ring_verts * 3;
    int center_neg = ring_verts * 4, center_pos = ring_verts * 4 + 1;
    /* Side quads + two cap fans. */
    mesh_object->index_count = radial_segments * 6 + radial_segments * 3 * 2;
    unsigned int *element_indices = malloc((size_t) mesh_object->index_count * sizeof(unsigned int));
    int ei = 0;
    for (int s = 0; s < radial_segments; s++) {
        element_indices[ei++] = ring0 + s;
        element_indices[ei++] = ring1 + s;
        element_indices[ei++] = ring0 + s + 1;
        element_indices[ei++] = ring0 + s + 1;
        element_indices[ei++] = ring1 + s;
        element_indices[ei++] = ring1 + s + 1;
    }
    for (int s = 0; s < radial_segments; s++) {
        /* -X cap (winding faces -X). */
        element_indices[ei++] = center_neg;
        element_indices[ei++] = ring2 + s + 1;
        element_indices[ei++] = ring2 + s;
        /* +X cap. */
        element_indices[ei++] = center_pos;
        element_indices[ei++] = ring3 + s;
        element_indices[ei++] = ring3 + s + 1;
    }
    /* Wireframe: two rim loops + 4 axial rails. */
    mesh_object->wireframe_index_count = radial_segments * 2 * 2 + 4 * 2;
    unsigned int *wireframe_indices = malloc((size_t) mesh_object->wireframe_index_count * sizeof(unsigned int));
    int wi = 0;
    for (int s = 0; s < radial_segments; s++) {
        wireframe_indices[wi++] = ring0 + s;
        wireframe_indices[wi++] = ring0 + s + 1;
        wireframe_indices[wi++] = ring1 + s;
        wireframe_indices[wi++] = ring1 + s + 1;
    }
    for (int k = 0; k < 4; k++) {
        int s = k * radial_segments / 4;
        wireframe_indices[wi++] = ring0 + s;
        wireframe_indices[wi++] = ring1 + s;
    }

    glGenVertexArrays(1, &mesh_object->vertex_array_object);
    glGenBuffers(1, &mesh_object->vertex_buffer_object);
    glGenBuffers(1, &mesh_object->element_buffer_object);
    glBindVertexArray(mesh_object->vertex_array_object);
    glBindBuffer(GL_ARRAY_BUFFER, mesh_object->vertex_buffer_object);
    glBufferData(GL_ARRAY_BUFFER, vertex_count * 6 * sizeof(float), vertex_data, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh_object->element_buffer_object);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh_object->index_count * sizeof(unsigned int), element_indices,
                 GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) 0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) (3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glGenBuffers(1, &mesh_object->wireframe_element_buffer_object);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh_object->wireframe_element_buffer_object);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh_object->wireframe_index_count * sizeof(unsigned int),
                 wireframe_indices, GL_STATIC_DRAW);
    glGenBuffers(1, &mesh_object->instance_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, mesh_object->instance_vbo);
    glBufferData(GL_ARRAY_BUFFER, mpe_max_bodies * 19 * sizeof(float), NULL, GL_DYNAMIC_DRAW);
    for (int i = 0; i < 4; i++) {
        glVertexAttribPointer(2 + i, 4, GL_FLOAT, GL_FALSE, 19 * sizeof(float), (void *) (i * 4 * sizeof(float)));
        glEnableVertexAttribArray(2 + i);
        glVertexAttribDivisor(2 + i, 1);
    }
    glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, 19 * sizeof(float), (void *) (16 * sizeof(float)));
    glEnableVertexAttribArray(6);
    glVertexAttribDivisor(6, 1);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh_object->element_buffer_object);
    glBindVertexArray(0);
    free(wireframe_indices);
    free(element_indices);
    free(vertex_data);
}

#include "sphere_meshing.h"
#include <epoxy/gl_generated.h>
#include <stdlib.h>
#include <math.h>
void init_sm_system (mesh *mesh_s, int sections, int stack) {
    //Calculate the number of vertices we require for rendering
    // (stack + 1)(sections + 1) total amount fo vertices required
    int vertice_count = (stack + 1) * (section + 1);
    float *vertices = malloc (vertice_count * 3 * sizeof (float));
    //Generate Vertex COordinates for rendering
    int v_idx = 0; //Assembly notation idx bitshifting
    for (int step = 0; step < stack; step++) {
        //Phi value goes from .5pi to -.5pi (Max to Min)
        float phi_variable = math_pi / 2 - step * math_pi / stack;
        for (int step2 = 0; step2 <= sections; step2++) {
            //Theta goes in radians all around circulat object
            float theta_variable = step2 * 2 * math_pi / sections;
            //Transpose Spherical objects to a cartesian plan (r = 1 as example)
            vertices [v_idx++] = cosf (phi) * cosf (theta);
            vertices [v_idx++] = sinf (phi);
            vertices [v_idx++] cosf (phi) * sinf (theta);
        }
    } //Generate Indices
    //Quads in each grid is split into dual triangle configurations
    mesh_s->index = stack * sections * 6;
    unsigned int *indice_list = malloc (mesh->index * sizeof (unsigned int));
    int i_idx = 0;
    for (int step3 = 0; step3 < stack; step3++) {
        int current_row = step3 * (sections + 1);
        int next_row = current_row + sections + 1;
        for (int step4 = 0; step4 < sections; step4++, current_row++; next_row++) {
            //First Triangle Drawn
            indice_list [i_idx++] = k1;
            indice_list [i_idx++] = k2;
            indice_list [i_idx++] = k1 + 1;
            //Second Triangle
            indice_list [i_idx++] = k1 + 1;
            indice_list [i_idx++] = k2;
            indice_list [i_idx++] = k2 + 1;
        }
    } //Upload to GPU rendering
    glGenVertexArrays (1, &mesh_s->vao);
    glGenBuffers (1, &mesh_s->vbo);
    glGenBuffers (1, &mesh_s->ebo);
    glBindVertexArray (mesh_s->vao);
    //Vertice Upload Count
    glBindBuffer (GL_ARRAY_BUFFER, mesh_s->vbo);
    glBufferData (GL_ARRAY_BUFFER, vertice_count * 3 * sizeof (float), vertices, GL_STATIC_DRAW);
    //Indice Upload Count
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, mesh_s->ebo);
    glBufferData (GL_ELEMENT_ARRAY_BUFFER, mesh_index * sizeof (unsigned int), indice_list, GL_STATIC_DRAW);
    //OpenGL attribute set --> affix certain attributes to 3D position statements
    glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof (float), (void*) 0);
    glEnableVertexAttribArray (0);
    //Free Malloced Memory Addrs
    free (indice_list);
    free (vertices);
}  void render_sphere (mesh *mesh_s, rigidbody *rb, math3 projection, math3 viewpoint) {
    //Sphere Preloaded GPU data
    glBindVertexArray (mesh_s->vao);
    //Model Matrix (Scalable by Radius of Sphere, then translate the sphere to apt position)
    //Math3D functions
    //Scale by rb->radius (mesh template set to 1)
    //Translation to rb->position
    //For now, use fixed function setup to test efficacy and accuracy
    glPushMatrix ();
    //Apply Physics Math to Actual Rendered Object
    glTranslatef (rb->position.x, rb->position.y, rb->position.z);
    glScalef (rb->radius, rb->radius, rb->radius);
    //Draw Triangles 
    //GL_TRIANGLES for GPU to run indices 3 at a time
    glDrawElements (GL_TRIANGLES, mesh_s->index, GL_UNSIGNED_INT, 0);
    glPopMatrix ();
}

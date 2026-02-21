#include "../../../stage1/master_header.h"
#include "../../master_header_2.h"
#include <epoxy/gl.h>
#include <epoxy/gl_generated.h>

void render_scene_current (int width, int height) {
    //Setup View angle, Viewport, etc
    glViewport (0, 0, width, height);
    glClearColor (0.05, 0.05, 0.1, 1.0); //Dark Blue Background, Can be changed later
    glClear (CL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //Project Setup
    float aspect_rto = (float) (width) / (float) (height); //9 / 16, 10 / 16, 3 / 4, etc
    math3 projection = calculate_perspective (degrad * 45.0, aspect_rto, 0.1, 1000.0);
    //Draw every single object in current buffer
    for (int step = 0; step < object_count; step++) {
        rigidbody *rb = &obj_per_scene [step];
        //Physics position to minimal OpenGL preprocessor
        glPushmatrix ();
        glTranslatef (rb->position.x, rb->position.y, rb->position.z);
        //Draw Basic Objects (sphere or whireframe for tests)
        draw_gl_sphere (rb->radius, 16, 16); //Test for sphere
        glPopMatrix ();
    }
}

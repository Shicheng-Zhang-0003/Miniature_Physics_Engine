//#include "../../../stage1/master_header.h"
//#include "../../master_header_2.h"
//#include <epoxy/gl.h>
//#include <epoxy/gl_generated.h>
/* extern rigidbody obj_per_scene [10];
extern int object_count;
void render_scene_current (int widget_width, int widget_height) {
    //Setup View angle, Viewport, etc
    glViewport (0, 0, widget_width, widget_height);
    glClearColor (0.05f, 0.05f, 0.1f, 1.0f); //Dark Blue Background, Can be changed later
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //Project Setup
    float aspect_ratio = (float) (widget_width) / (float) (widget_height); //9 / 16, 10 / 16, 3 / 4, etc
    math3 projection_matrix = calculate_perspective (degrad * 45.0f, aspect_ratio, 0.1f, 1000.0f);
    //Draw every single object in current buffer
    for (int object_index = 0; object_index < object_count; object_index++) {
        rigidbody *rigid_body = &obj_per_scene [object_index];
        //Physics position to minimal OpenGL preprocessor
        glPushMatrix ();
        glTranslatef (rigid_body -> position.x, rigid_body -> position.y, rigid_body -> position.z);
        //Draw Basic Objects (sphere or whireframe for tests)
        //render_sphere (&sphere_mesh, rigid_body, projection_matrix, view_matrix);
        glPopMatrix ();
    }
} */

//Incorporate Camera FOV
// Inside rendering loop
//float view_matrix_array [16];
//camera_get_view_matrix (&main_camera_fov, view_matrix_array);

// Send it to shadering unit in OpenGL
//GLuint view_location = glGetUniformLocation (shader_program, "view");
//glUniformMatrix4fv (view_location, 1, GL_FALSE, view_matrix_array);

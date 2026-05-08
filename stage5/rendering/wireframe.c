#include "wireframe.h"
#include "../../stage2/interface/sphere_object/meshing/sphere_meshing.h"
#include <epoxy/gl_generated.h>
extern mesh sphere_mesh;
void wireframe_render_object (GLuint shader_program, math4 view, math4 projection, rigidbody *rb, vector3 wire_colour) {
    glUseProgram (shader_program);
    //Pass the viewpoint and the projection data
    float view_array [16];
    float projection_array [16];
    math4_to_flat_array (view, view_array);
    math4_to_flat_array (projection, projection_array);
    glUniformMatrix4fv (glGetUniformLocation (shader_program, "viewframe"), 1, GL_FALSE, view_array);
    glUniformMatrix4fv (glGetUniformLocation (shader_program, "projection"), 1, GL_FALSE, projection_array);
    //Scale the mesh frame larger than the actual sphere by a small amount
    float outline_scale = rb -> radius * 1.01;
    math4 translation = math4_translation (rb -> position);
    math4 rotation = vector4_to_math4 (rb -> orientation);
    math4 scale = math4_scaling ((vector3) {outline_scale, outline_scale, outline_scale});
    math4 model = math4_multiplication (translation, math4_multiplication(rotation, scale));
    float model_array [16];
    math4_to_flat_array (model, model_array);
    //Set the object colour to the wireframe colour
    glUniform3f (glGetUniformLocation (shader_program, "object_colour"), wire_colour.x, wire_colour.y, wire_colour.z);
    glUniformMatrix4fv (glGetUniformLocation (shader_program, "model"), 1, GL_FALSE, model_array);
    //Draw in wireframes
    glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
    glBindVertexArray (sphere_mesh.vao);
    glDrawElements (GL_TRIANGLES, sphere_mesh.index, GL_UNSIGNED_INT, 0);
    glBindVertexArray (0);
    //Restore normal operation later --> line mesh mode to fill mode to prevent something else breaking
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
} void wireframe_render_selected_object (GLuint shader_program, math4 view, math4 projection) {
    if ((selected_object < 0) || (selected_object >= object_count)) {return;}
    //A yellow colour stands out from the background
    wireframe_render_object (shader_program, view, projection, &obj_per_scene [selected_object], (vector3) {1.0, 1.0, 0.0});
}

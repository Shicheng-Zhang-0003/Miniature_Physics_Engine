#include "../mpe_engine.h"
#include <complex.h>
#include <epoxy/gl.h>
#include <epoxy/gl_generated.h>
#include <sys/types.h>
#include <stdlib.h>
extern camera main_camera_fov;
extern rigidbody *obj_per_scene;
extern int object_count;
extern mesh cube_mesh;
static GLuint instanced_shader_program = 0;
static GLuint utility_shader_program = 0;
static struct {
    GLint projection_matrix_location;
    GLint view_matrix_location;
    GLint camera_position_location;
    GLint light_position_location;
} instanced_uniforms;
static struct {
    GLint projection_matrix_location;
    GLint view_matrix_location;
    GLint model_matrix_location;
    GLint normal_matrix_location;
    GLint object_colour_location;
    GLint camera_position_location;
    GLint light_position_location;
} utility_uniforms;
mesh sphere_mesh;
static int render_init_status = 0;
static grid_mesh main_grid;
static float *sphere_instances = NULL;
static float *cube_instances = NULL;
void render_init () {
    if (render_init_status) {return;}
    instanced_shader_program = create_shader_program ("render/shaders/vertex_shader.glsl", "render/shaders/fragment_shader.glsl");
    utility_shader_program = create_shader_program ("render/shaders/utility_vertex.glsl", "render/shaders/utility_fragment.glsl");
    instanced_uniforms.projection_matrix_location = glGetUniformLocation (instanced_shader_program, "projection");
    instanced_uniforms.view_matrix_location = glGetUniformLocation (instanced_shader_program, "viewframe");
    instanced_uniforms.camera_position_location = glGetUniformLocation (instanced_shader_program, "camera_position");
    instanced_uniforms.light_position_location = glGetUniformLocation (instanced_shader_program, "light_position");
    utility_uniforms.projection_matrix_location = glGetUniformLocation (utility_shader_program, "projection");
    utility_uniforms.view_matrix_location = glGetUniformLocation (utility_shader_program, "viewframe");
    utility_uniforms.model_matrix_location = glGetUniformLocation (utility_shader_program, "model");
    utility_uniforms.normal_matrix_location = glGetUniformLocation (utility_shader_program, "normal_matrix");
    utility_uniforms.object_colour_location = glGetUniformLocation (utility_shader_program, "object_colour");
    utility_uniforms.camera_position_location = glGetUniformLocation (utility_shader_program, "camera_position");
    utility_uniforms.light_position_location = glGetUniformLocation (utility_shader_program, "light_position");
    grid_init (&main_grid, 250, 5);
    init_sm_system (&sphere_mesh, 32, 32);
    cube_meshing_init ();
    sphere_instances = malloc (MPE_MAX_BODIES * 19 * sizeof (float));
    cube_instances = malloc (MPE_MAX_BODIES * 19 * sizeof (float));
    render_init_status = 1;
} void render_scene_current (int widget_width, int widget_height) {
    render_init ();
    glViewport (0, 0, widget_width, widget_height);
    glClearColor (0.05f, 0.05f, 0.1f, 1.0f);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    float window_aspect_ratio = (float) (widget_width) / (float) (widget_height);
    math4 projection_matrix = math4_perspective_fov (degrad * 45.0f, window_aspect_ratio, 0.1f, 1000.0f);
    float projection_matrix_flat_array [16];
    math4_to_flat_array (projection_matrix, projection_matrix_flat_array);
    math4 view_matrix = math4_look_view (main_camera_fov.position, main_camera_fov.forward_vector, main_camera_fov.vertical_vector);
    float view_matrix_flat_array [16];
    math4_to_flat_array (view_matrix, view_matrix_flat_array);
    grid_render (&main_grid, utility_shader_program, view_matrix, projection_matrix);
    int sphere_inst_count = 0;
    int cube_inst_count = 0;
    for (int object_index = 0; object_index < object_count; object_index++) {
        rigidbody *rigid_body = &obj_per_scene [object_index];
        math4 translation_matrix = math4_translation (rigid_body -> position);
        math4 rotation_matrix = vector4_to_math4 (rigid_body -> orientation);
        math4 scale_matrix;
        if (rigid_body -> type == object_sphere) {scale_matrix = math4_scaling ((vector3) {rigid_body -> radius, rigid_body -> radius, rigid_body -> radius});}
        else {scale_matrix = math4_scaling (rigid_body -> half_extensions);}
        math4 model_matrix = math4_multiplication (translation_matrix, math4_multiplication (rotation_matrix, scale_matrix));
        float *target_array;
        int *target_count;
        if (rigid_body -> type == object_sphere) {target_array = sphere_instances; target_count = &sphere_inst_count;}
        else {target_array = cube_instances; target_count = &cube_inst_count;}
        if ((*target_count) < MPE_MAX_BODIES) {
            int idx = (*target_count) * 19;
            math4_to_flat_array (model_matrix, &target_array [idx]);
            target_array [idx + 16] = rigid_body -> colour.x;
            target_array [idx + 17] = rigid_body -> colour.y;
            target_array [idx + 18] = rigid_body -> colour.z;
            (*target_count)++;
        }
    } glUseProgram (instanced_shader_program);
    glUniformMatrix4fv (instanced_uniforms.projection_matrix_location, 1, GL_FALSE, projection_matrix_flat_array);
    glUniformMatrix4fv (instanced_uniforms.view_matrix_location, 1, GL_FALSE, view_matrix_flat_array);
    glUniform3f (instanced_uniforms.camera_position_location, main_camera_fov.position.x, main_camera_fov.position.y, main_camera_fov.position.z);
    glUniform3f (instanced_uniforms.light_position_location, 20.0f, 40.0f, 20.0f);
    if (sphere_inst_count > 0) {
        glBindBuffer (GL_ARRAY_BUFFER, sphere_mesh.instance_vbo);
        glBufferSubData (GL_ARRAY_BUFFER, 0, sphere_inst_count * 19 * sizeof (float), sphere_instances);
        glBindVertexArray (sphere_mesh.vertex_array_object);
        glDrawElementsInstanced (GL_TRIANGLES, sphere_mesh.index_count, GL_UNSIGNED_INT, 0, sphere_inst_count);
    } if (cube_inst_count > 0) {
        glBindBuffer (GL_ARRAY_BUFFER, cube_mesh.instance_vbo);
        glBufferSubData (GL_ARRAY_BUFFER, 0, cube_inst_count * 19 * sizeof (float), cube_instances);
        glBindVertexArray (cube_mesh.vertex_array_object);
        glDrawElementsInstanced (GL_TRIANGLES, cube_mesh.index_count, GL_UNSIGNED_INT, 0, cube_inst_count);
    } glBindVertexArray (0);
    spring_joint_render (utility_shader_program, view_matrix, projection_matrix);
    wireframe_render_selected_object (utility_shader_program, view_matrix, projection_matrix);
}

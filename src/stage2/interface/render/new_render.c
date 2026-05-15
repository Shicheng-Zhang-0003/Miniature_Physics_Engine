#include "../../../stage1/master_header.h"
#include "../../master_header_2.h"
#include "../../../stage5/rendering/grid.h"
#include "../../../stage5/rendering/wireframe.h"
#include "../../../stage5/constraints/spring_joint.h"
#include "../../interface/sphere_object/meshing/sphere_meshing.h"
#include "../../interface/sphere_object/meshing/cube_meshing.h"
#include "shader_loading.h"
#include <complex.h>
#include <epoxy/gl.h>
#include <epoxy/gl_generated.h>
#include <sys/types.h>
//Global Values for shading and meshing
extern camera main_camera_fov;
extern rigidbody *obj_per_scene;
extern int object_count;
extern mesh cube_mesh;
static GLuint shaders_program_total = 0;
static struct {
    GLint projection_matrix_location;
    GLint view_matrix_location;
    GLint model_matrix_location;
    GLint normal_matrix_location;
    GLint object_colour_location;
    GLint camera_position_location;
    GLint light_position_location;
} shader_uniform_location_registry;
mesh sphere_mesh;
static int render_init_status = 0;
static grid_mesh main_grid;
//Render all axes of rotation
/* static GLuint axis_shader_program_total = 0;
static struct {
    GLint projection_matrix_location;
    GLint view_matrix_location;
    GLint axis_colour_location;
} axis_uniform_location_registry;
static GLuint axis_vertex_array_object = 0;
static GLuint axis_vertex_buffer_object = 0;
static void axis_renderer_init () {
    if (axis_shader_program_total) {return;}
    axis_shader_program_total = create_shader_program ("stage2/interface/render/shaders/axis_vertex.glsl", "stage2/interface/render/shaders/axis_fragment.glsl");
    axis_uniform_location_registry.projection_matrix_location = glGetUniformLocation (axis_shader_program_total, "projection");
    axis_uniform_location_registry.view_matrix_location = glGetUniformLocation (axis_shader_program_total, "viewframe");
    axis_uniform_location_registry.axis_colour_location = glGetUniformLocation (axis_shader_program_total, "axis_colour");
    glGenVertexArrays (1, &axis_vertex_array_object);
    glGenBuffers (1, &axis_vertex_buffer_object);
    glBindVertexArray (axis_vertex_array_object);
    glBindBuffer (GL_ARRAY_BUFFER, axis_vertex_buffer_object);
    glBufferData (GL_ARRAY_BUFFER, 6 * sizeof (float), NULL, GL_DYNAMIC_DRAW);
    glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray (0);
    glBindVertexArray (0);
} static void draw_axis_segment (vector3 axis_segment_origin_point, vector3 axis_segment_tip_point, float colour_red_component, float colour_green_component, float colour_blue_component) {
    glUniform3f (axis_uniform_location_registry.axis_colour_location, colour_red_component, colour_green_component, colour_blue_component);
    glBindVertexArray (axis_vertex_array_object);
    glBindBuffer (GL_ARRAY_BUFFER, axis_vertex_buffer_object);
    float axis_line_vertex_position_data [6] = {axis_segment_origin_point.x, axis_segment_origin_point.y, axis_segment_origin_point.z,axis_segment_tip_point.x, axis_segment_tip_point.y, axis_segment_tip_point.z};
    glBufferSubData (GL_ARRAY_BUFFER, 0, sizeof (axis_line_vertex_position_data), axis_line_vertex_position_data);
    glDrawArrays (GL_LINES, 0, 2);
    float axis_tip_point_vertex_data [3] = {axis_segment_tip_point.x, axis_segment_tip_point.y, axis_segment_tip_point.z};
    glBufferSubData (GL_ARRAY_BUFFER, 0, sizeof (axis_tip_point_vertex_data), axis_tip_point_vertex_data);
    glDrawArrays (GL_POINTS, 0, 1);
    glBindVertexArray (0);
} static void draw_sphere_axes (rigidbody *rigid_body, float *projection_matrix_flat_array, float *view_matrix_flat_array) {
    axis_renderer_init ();
    glUseProgram (axis_shader_program_total);
    glUniformMatrix4fv (axis_uniform_location_registry.projection_matrix_location, 1, GL_FALSE, projection_matrix_flat_array);
    glUniformMatrix4fv (axis_uniform_location_registry.view_matrix_location, 1, GL_FALSE, view_matrix_flat_array);
    glEnable (GL_PROGRAM_POINT_SIZE);
    glLineWidth (2.0f);
    math3 sphere_orientation_rotation_matrix = vector4_to_math3 (rigid_body -> orientation);
    float axis_visual_scale_factor = rigid_body -> radius * 1.5f;
    vector3 local_x_axis_world_direction = {sphere_orientation_rotation_matrix.matrix [0][0], sphere_orientation_rotation_matrix.matrix [1][0], sphere_orientation_rotation_matrix.matrix [2][0]};
    vector3 local_y_axis_world_direction = {sphere_orientation_rotation_matrix.matrix [0][1], sphere_orientation_rotation_matrix.matrix [1][1], sphere_orientation_rotation_matrix.matrix [2][1]};
    vector3 local_z_axis_world_direction = {sphere_orientation_rotation_matrix.matrix [0][2], sphere_orientation_rotation_matrix.matrix [1][2], sphere_orientation_rotation_matrix.matrix [2][2]};
    vector3 x_axis_tip_world_position = {rigid_body -> position.x + local_x_axis_world_direction.x * axis_visual_scale_factor, rigid_body -> position.y + local_x_axis_world_direction.y * axis_visual_scale_factor, rigid_body -> position.z + local_x_axis_world_direction.z * axis_visual_scale_factor};
    vector3 y_axis_tip_world_position = {rigid_body -> position.x + local_y_axis_world_direction.x * axis_visual_scale_factor, rigid_body -> position.y + local_y_axis_world_direction.y * axis_visual_scale_factor, rigid_body -> position.z + local_y_axis_world_direction.z * axis_visual_scale_factor};
    vector3 z_axis_tip_world_position = {rigid_body -> position.x + local_z_axis_world_direction.x * axis_visual_scale_factor, rigid_body -> position.y + local_z_axis_world_direction.y * axis_visual_scale_factor, rigid_body -> position.z + local_z_axis_world_direction.z * axis_visual_scale_factor};
    draw_axis_segment (rigid_body -> position, x_axis_tip_world_position, 1.0f, 0.0f, 0.0f);
    draw_axis_segment (rigid_body -> position, y_axis_tip_world_position, 0.0f, 1.0f, 0.0f);
    draw_axis_segment (rigid_body -> position, z_axis_tip_world_position, 0.0f, 0.0f, 1.0f);
    glDisable (GL_PROGRAM_POINT_SIZE);
} */
void render_init () {
    if (render_init_status) {return;}
    //Load/Compilation of Shaders
    shaders_program_total = create_shader_program ("stage2/interface/sphere_object/shaders/vertex_shader.glsl", "stage2/interface/sphere_object/shaders/fragment_shader.glsl");
    //Initialise (32 sec, 32 stack)
    //Cache Locations
    shader_uniform_location_registry.projection_matrix_location = glGetUniformLocation (shaders_program_total, "projection");
    shader_uniform_location_registry.view_matrix_location = glGetUniformLocation (shaders_program_total, "viewframe");
    shader_uniform_location_registry.model_matrix_location = glGetUniformLocation (shaders_program_total, "model");
    shader_uniform_location_registry.normal_matrix_location = glGetUniformLocation (shaders_program_total, "normal_matrix");
    shader_uniform_location_registry.object_colour_location = glGetUniformLocation (shaders_program_total, "object_colour");
    shader_uniform_location_registry.camera_position_location = glGetUniformLocation (shaders_program_total, "camera_position");
    shader_uniform_location_registry.light_position_location = glGetUniformLocation (shaders_program_total, "light_position");
    grid_init (&main_grid, 250, 5);
    init_sm_system (&sphere_mesh, 32, 32);
    cube_meshing_init ();
    render_init_status = 1;
} void render_scene_current (int widget_width, int widget_height) {
    render_init ();
    //View Status, Clear SCN
    glViewport (0, 0, widget_width, widget_height);
    glClearColor (0.05f, 0.05f, 0.1f, 1.0f);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram (shaders_program_total);
    //Projection Matrix, 4D Matrix
    float window_aspect_ratio = (float) (widget_width) / (float) (widget_height);
    math4 projection_matrix = math4_perspective_fov (degrad * 45.0f, window_aspect_ratio, 0.1f, 1000.0f);
    float projection_matrix_flat_array [16];
    math4_to_flat_array (projection_matrix, projection_matrix_flat_array);
    glUniformMatrix4fv (shader_uniform_location_registry.projection_matrix_location, 1, GL_FALSE, projection_matrix_flat_array);
    //View Matrix From Camera Point of View
    math4 view_matrix = math4_look_view (main_camera_fov.position, main_camera_fov.forward_vector, main_camera_fov.vertical_vector);
    float view_matrix_flat_array [16];
    math4_to_flat_array (view_matrix, view_matrix_flat_array);
    glUniformMatrix4fv (shader_uniform_location_registry.view_matrix_location, 1, GL_FALSE, view_matrix_flat_array);
    glUniform3f (shader_uniform_location_registry.camera_position_location, main_camera_fov.position.x, main_camera_fov.position.y, main_camera_fov.position.z);
    //Light Position (Stronger overhead lighting)
    glUniform3f (shader_uniform_location_registry.light_position_location, 20.0f, 40.0f, 20.0f);
    //Draw Each Object in Question
    grid_render (&main_grid, shaders_program_total, view_matrix, projection_matrix);
    glUseProgram (shaders_program_total); // Ensure we are back to our main program after grid_render
    for (int object_index = 0; object_index < object_count; object_index++) {
        rigidbody *rigid_body = &obj_per_scene [object_index];
        //Model Matrix --> Position + Orientation
        math4 translation_matrix = math4_translation (rigid_body -> position);
        math4 rotation_matrix = vector4_to_math4 (rigid_body -> orientation);
        math4 scale_matrix;
        if (rigid_body -> type == object_sphere) {scale_matrix = math4_scaling ((vector3) {rigid_body -> radius, rigid_body -> radius, rigid_body -> radius});}
        else {scale_matrix = math4_scaling (rigid_body -> half_extensions);}
        math4 model_matrix = math4_multiplication (translation_matrix, math4_multiplication (rotation_matrix, scale_matrix));
        float model_matrix_flat_array [16];
        math4_to_flat_array (model_matrix, model_matrix_flat_array);
        //Colour Uniform of the Objects
        glUniform3f (shader_uniform_location_registry.object_colour_location, rigid_body -> colour.x, rigid_body -> colour.y, rigid_body -> colour.z);
        glUniformMatrix4fv (shader_uniform_location_registry.model_matrix_location, 1, GL_FALSE, model_matrix_flat_array);
        math3 normal_calculation_matrix_3x3;
        for (int row_index = 0; row_index < 3; row_index++) {
            for (int column_index = 0; column_index < 3; column_index++) {normal_calculation_matrix_3x3.matrix [row_index][column_index] = model_matrix.matrix [row_index][column_index];}
        } math3 normal_matrix = math3_transposition (math3_inverse (normal_calculation_matrix_3x3));
        float normal_matrix_flat_array [9];
        for (int row_index = 0; row_index < 3; row_index++) {
            for (int column_index = 0; column_index < 3; column_index++) {normal_matrix_flat_array [row_index * 3 + column_index] = normal_matrix.matrix [row_index][column_index];}
        } glUniformMatrix3fv (shader_uniform_location_registry.normal_matrix_location, 1, GL_FALSE, normal_matrix_flat_array);
        //Render Objects - Choose sphere or cube based on type
        if (rigid_body -> type == object_sphere) {render_sphere_object (&sphere_mesh, rigid_body);}
        else if (rigid_body -> type == object_cube) {render_cube_object (&cube_mesh, rigid_body);}
        //Draw Mesh Latitude and Longitudinal Lines (Rotational Visibility)
        /* Draw rotation axes for each object */ /* Makes the game far too laggy
        for (int step_axes = 0; step_axes < object_count; step_axes++) {
            draw_sphere_axes (&obj_per_scene [step_axes], projection_matrix_flat_array, view_matrix_flat_array);
        } */
        glUseProgram (shaders_program_total);
    } wireframe_render_selected_object (shaders_program_total, view_matrix, projection_matrix);
}

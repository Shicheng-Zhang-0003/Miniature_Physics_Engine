#include "../../../stage1/master_header.h"
#include "../../master_header_2.h"
#include "../../../stage5/rendering/grid.h"
#include "../../../stage5/rendering/wireframe.h"
#include "../../../stage5/constraints/spring_joint.h"
#include "../../interface/sphere_object/meshing/sphere_meshing.h"
#include "shader_loading.h"
#include <epoxy/gl.h>
#include <epoxy/gl_generated.h>
#include <sys/types.h>
//Global Values for shading and meshing
extern camera main_camera_fov;
extern rigidbody *obj_per_scene;
extern int object_count;
static GLuint shaders_program_total = 0;
mesh sphere_mesh;
static int render_init_status = 0;
static grid_mesh main_grid;
void render_init () {
    if (render_init_status) {return;}
    //Load/Compilation of Shaders
    shaders_program_total = create_shader_program (
        "stage2/interface/sphere_object/shaders/vertex_shader.glsl",
        "stage2/interface/sphere_object/shaders/fragment_shader.glsl"
    ); //Initialise (32 sec, 32 stack)
    grid_init (&main_grid, 100, 5);
    init_sm_system (&sphere_mesh, 32, 32);
    render_init_status = 1;
} void render_scene_current (int width, int height) {
    render_init ();
    //View Status, Clear SCN
    glViewport (0, 0, width, height);
    glClearColor (0.05, 0.05, 0.1, 1.0);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram (shaders_program_total);
    //Projection Matrix, 4D Matrix
    float aspect_rto = (float) (width) / (float) (height);
    math4 projection = math4_perspective_fov (degrad * 45.0, aspect_rto, 0.1, 1000.0);
    float projection_array [16];
    math4_to_flat_array (projection, projection_array);
    glUniformMatrix4fv (glGetUniformLocation (shaders_program_total, "projection"), 1, GL_FALSE, projection_array);
    //View Matrix From Camera Point of View
    math4 viewpoint = math4_look_view (main_camera_fov.position, main_camera_fov.front, main_camera_fov.vertical);
    float viewing_array [16];
    math4_to_flat_array (viewpoint, viewing_array);
    glUniformMatrix4fv (glGetUniformLocation (shaders_program_total, "viewframe"), 1, GL_FALSE, viewing_array);
    glUniform3f (glGetUniformLocation (shaders_program_total, "camera_position"), main_camera_fov.position.x, main_camera_fov.position.y, main_camera_fov.position.z);
    //Light Position
    glUniform3f (glGetUniformLocation (shaders_program_total, "light_position"), 10.0, 20.0, 10.0);
    //Draw Each Object in Question
    grid_render (&main_grid, shaders_program_total, viewpoint, projection);
    apply_force_all_joints ();
    for (int step = 0; step < object_count; step++) {
        rigidbody *rb = &obj_per_scene [step];
        //Model Matrix --> Position + Orientation
        math4 translation = math4_translation (rb->position);
        math4 rotation = vector4_to_math4 (rb->orientation);
        math4 scale_render = math4_scaling ((vector3) {rb->radius, rb->radius, rb->radius});
        math4 model = math4_multiplication (translation, math4_multiplication (rotation, scale_render));
        float model_array [16];
        math4_to_flat_array (model, model_array);
        //Colour Uniform of the Objects
        glUniform3f (glGetUniformLocation (shaders_program_total, "object_colour"), rb->colour.x, rb->colour.y, rb->colour.z);
        glUniformMatrix4fv (glGetUniformLocation (shaders_program_total, "model"), 1, GL_FALSE, model_array);
        math3 model3;
        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {model3.matrix [row][column] = model.matrix [row][column];}
        } math3 normal_mat = math3_transposition (math3_inverse (model3));
        float normal_array [9];
        for (int row2 = 0; row2 < 3; row2++) {
            for (int column2 = 0; column2 < 3; column2++) {normal_array [row2 * 3 + column2] = normal_mat.matrix [row2][column2];}
        } glUniformMatrix3fv (glGetUniformLocation (shaders_program_total, "normal_matrix"), 1, GL_FALSE, normal_array);
        //Render Objects
        render_sphere_object (&sphere_mesh, rb);
    } wireframe_render_selected_object (shaders_program_total, viewpoint, projection);
}

/*
 * Add In Future:
 *
 * math3 normal_mat = math3_transposition (math3_inverse ( upper-left 3x3 of model ));
 *
 * */

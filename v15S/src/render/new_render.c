/* GTK4-PREP: zero GUI headers in render (GL only). */
#include <epoxy/gl.h>
#include <epoxy/gl_generated.h>
#include "../core/physics_world.h"
#include "../core/rigidbody.h"
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include "../physics/broadphase.h"
#include "../ui_input/camera.h"
#include "shader_loading.h"
#include "sphere_meshing.h"
#include "cube_meshing.h"
#include "cylinder_meshing.h"
#include "grid.h"
#include "wireframe.h"
#include "../physics/spring_joint.h"
#include <sys/types.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

extern camera main_camera_fov;
extern mesh cube_mesh;
static GLuint instanced_shader_program = 0;
static GLuint utility_shader_program = 0;
static struct {
    GLint projection_matrix_location;
    GLint view_matrix_location;
    GLint camera_position_location;
    GLint light_position_location;
    GLint ambient_strength_location;
    GLint specular_coeff_location;
    GLint specular_exponent_location;
} instanced_uniforms;
static struct {
    GLint projection_matrix_location;
    GLint view_matrix_location;
    GLint model_matrix_location;
    GLint normal_matrix_location;
    GLint object_colour_location;
    GLint camera_position_location;
    GLint light_position_location;
    GLint ambient_strength_location;
    GLint specular_coeff_location;
    GLint specular_exponent_location;
} utility_uniforms;
mesh sphere_mesh;
mesh cylinder_mesh;
typedef enum { render_uninitialized = 0, render_ok = 1, render_failed = -1 } render_status;
static render_status render_init_status = render_uninitialized;
static grid_mesh main_grid;
static float *sphere_instances = NULL;
static float *cube_instances = NULL;
static float *cylinder_instances = NULL;
void render_init() {
    if (render_init_status != render_uninitialized) {
        return;
    }
    const char *shader_dir = getenv("MPE_SHADER_DIR");
    char vs_path[512], fs_path[512], uvs_path[512], ufs_path[512];
    if (shader_dir && shader_dir[0]) {
        snprintf(vs_path, sizeof(vs_path), "%s/vertex_shader.glsl", shader_dir);
        snprintf(fs_path, sizeof(fs_path), "%s/fragment_shader.glsl", shader_dir);
        snprintf(uvs_path, sizeof(uvs_path), "%s/utility_vertex.glsl", shader_dir);
        snprintf(ufs_path, sizeof(ufs_path), "%s/utility_fragment.glsl", shader_dir);
    } else {
        snprintf(vs_path, sizeof(vs_path), "render/shaders/vertex_shader.glsl");
        snprintf(fs_path, sizeof(fs_path), "render/shaders/fragment_shader.glsl");
        snprintf(uvs_path, sizeof(uvs_path), "render/shaders/utility_vertex.glsl");
        snprintf(ufs_path, sizeof(ufs_path), "render/shaders/utility_fragment.glsl");
    }
    /* Installed fallback: <prefix>/share/mpe/shaders (see make install). */
    instanced_shader_program = create_shader_program(vs_path, fs_path);
    if (instanced_shader_program == 0) {
        char alt_vs[512], alt_fs[512];
        const char *home = getenv("HOME");
        if (home) {
            snprintf(alt_vs, sizeof(alt_vs), "%s/.local/share/mpe/shaders/vertex_shader.glsl", home);
            snprintf(alt_fs, sizeof(alt_fs), "%s/.local/share/mpe/shaders/fragment_shader.glsl", home);
            instanced_shader_program = create_shader_program(alt_vs, alt_fs);
        }
    }
    utility_shader_program = create_shader_program(uvs_path, ufs_path);
    if ((instanced_shader_program == 0) || (utility_shader_program == 0)) {
        fprintf(stderr, "RENDER INIT FAILED: shader program creation failed (instanced=%u, utility=%u)\n",
                instanced_shader_program, utility_shader_program);
        render_init_status = render_failed;
        return;
    }
    instanced_uniforms.projection_matrix_location = glGetUniformLocation(instanced_shader_program, "projection");
    instanced_uniforms.view_matrix_location = glGetUniformLocation(instanced_shader_program, "viewframe");
    instanced_uniforms.camera_position_location = glGetUniformLocation(instanced_shader_program, "camera_position");
    instanced_uniforms.light_position_location = glGetUniformLocation(instanced_shader_program, "light_position");
    instanced_uniforms.ambient_strength_location = glGetUniformLocation(instanced_shader_program, "u_ambient_strength");
    instanced_uniforms.specular_coeff_location = glGetUniformLocation(instanced_shader_program, "u_specular_coeff");
    instanced_uniforms.specular_exponent_location =
        glGetUniformLocation(instanced_shader_program, "u_specular_exponent");
    utility_uniforms.projection_matrix_location = glGetUniformLocation(utility_shader_program, "projection");
    utility_uniforms.view_matrix_location = glGetUniformLocation(utility_shader_program, "viewframe");
    utility_uniforms.model_matrix_location = glGetUniformLocation(utility_shader_program, "model");
    utility_uniforms.normal_matrix_location = glGetUniformLocation(utility_shader_program, "normal_matrix");
    utility_uniforms.object_colour_location = glGetUniformLocation(utility_shader_program, "object_colour");
    utility_uniforms.camera_position_location = glGetUniformLocation(utility_shader_program, "camera_position");
    utility_uniforms.light_position_location = glGetUniformLocation(utility_shader_program, "light_position");
    grid_init(&main_grid, 250, 5);
    init_sm_system(&sphere_mesh, 32, 32);
    cube_meshing_init();
    init_cylinder_system(&cylinder_mesh, 24);
    sphere_instances = malloc(mpe_max_bodies * 19 * sizeof(float));
    cube_instances = malloc(mpe_max_bodies * 19 * sizeof(float));
    cylinder_instances = malloc(mpe_max_bodies * 19 * sizeof(float));
    if (!sphere_instances || !cube_instances || !cylinder_instances) {
        fprintf(stderr, "RENDER INIT FAILED: instance buffer OOM\n");
        free(sphere_instances); sphere_instances = NULL;
        free(cube_instances); cube_instances = NULL;
        free(cylinder_instances); cylinder_instances = NULL;
        render_init_status = render_failed;
        return;
    }
    render_init_status = render_ok;
}
void render_cleanup(void) {
    if (sphere_instances) {
        free(sphere_instances);
        sphere_instances = NULL;
    }
    if (cube_instances) {
        free(cube_instances);
        cube_instances = NULL;
    }
    if (cylinder_instances) {
        free(cylinder_instances);
        cylinder_instances = NULL;
    }
    render_init_status = render_uninitialized;
}
/* Primary-world-only renderer: draws physics_world_get_primary().
 * TODO(world-param): take an explicit physics_world* so headless/secondary
 * worlds can render without relying on the app-owned primary. */
void render_scene_current(int widget_width, int widget_height) {
    if (widget_width <= 0 || widget_height <= 0) {
        return;
    }
    if (render_init_status == render_failed) {
        glViewport(0, 0, widget_width, widget_height);
        glClearColor(0.5f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        return;
    }
    glViewport(0, 0, widget_width, widget_height);
    glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    float window_aspect_ratio = (float) (widget_width) / (float) (widget_height);
    math4 projection_matrix = math4_perspective_fov(degrad * 45.0f, window_aspect_ratio, 0.1f, 1000.0f);
    float projection_matrix_flat_array[16];
    math4_to_flat_array(projection_matrix, projection_matrix_flat_array);
    math4 view_matrix =
        math4_look_view(main_camera_fov.position, main_camera_fov.forward_vector, main_camera_fov.vertical_vector);
    float view_matrix_flat_array[16];
    math4_to_flat_array(view_matrix, view_matrix_flat_array);
    grid_render(&main_grid, utility_shader_program, view_matrix, projection_matrix);
    /* Frustum culling: extract the six inward-facing planes from the
     * column-major view-projection matrix (Gribb/Hartmann) and skip
     * packing/uploading/drawing fully-outside bodies. Bounding spheres
     * come from broadphase_bounding_radius (rotation-invariant, never
     * wrongly excludes). Zero persistent memory: planes live on stack. */
    math4 view_projection = math4_multiplication(projection_matrix, view_matrix);
    vector4 frustum_planes[6];
    {
        float row0[4] = {view_projection.matrix[0][0], view_projection.matrix[1][0], view_projection.matrix[2][0],
                         view_projection.matrix[3][0]};
        float row1[4] = {view_projection.matrix[0][1], view_projection.matrix[1][1], view_projection.matrix[2][1],
                         view_projection.matrix[3][1]};
        float row2[4] = {view_projection.matrix[0][2], view_projection.matrix[1][2], view_projection.matrix[2][2],
                         view_projection.matrix[3][2]};
        float row3[4] = {view_projection.matrix[0][3], view_projection.matrix[1][3], view_projection.matrix[2][3],
                         view_projection.matrix[3][3]};
        float combos[6][4];
        for (int k = 0; k < 4; k++) {
            combos[0][k] = row3[k] + row0[k];
            combos[1][k] = row3[k] - row0[k];
            combos[2][k] = row3[k] + row1[k];
            combos[3][k] = row3[k] - row1[k];
            combos[4][k] = row3[k] + row2[k];
            combos[5][k] = row3[k] - row2[k];
        }
        for (int p = 0; p < 6; p++) {
            float len =
                sqrtf(combos[p][0] * combos[p][0] + combos[p][1] * combos[p][1] + combos[p][2] * combos[p][2]);
            if (len < 0.000001f) {
                len = 1.0f;
            }
            /* vector4 packs {w,x,y,z}: store (d,a,b,c) so .x/.y/.z/.w
             * read as the (a,b,c,d) plane coefficients below. */
            frustum_planes[p] =
                (vector4){combos[p][3] / len, combos[p][0] / len, combos[p][1] / len, combos[p][2] / len};
        }
    }
    int sphere_inst_count = 0;
    int cube_inst_count = 0;
    int cylinder_inst_count = 0;
    for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
        rigidbody *rigid_body = &(physics_world_get_primary()->bodies)[object_index];
        /* Sphere-vs-frustum: outside if signed distance < -radius on any plane. */
        {
            float bound = broadphase_bounding_radius(rigid_body);
            bool culled = false;
            for (int p = 0; p < 6; p++) {
                float dist = frustum_planes[p].x * rigid_body->position.x +
                             frustum_planes[p].y * rigid_body->position.y +
                             frustum_planes[p].z * rigid_body->position.z + frustum_planes[p].w;
                if (dist < -bound) {
                    culled = true;
                    break;
                }
            }
            if (culled) {
                continue;
            }
        }
        /* Direct T*R*S composition (~20 flops) instead of two full 4x4
         * multiplies (~128): M[c][r] = R[c][r]*s[c], M[c][3] = 0,
         * M[3][r] = t[r], M[3][3] = 1. Render-only path (no physics
         * impact); also why hand-SIMD stops here — the solver's
         * bit-determinism discipline (-ffp-contract=off, fixed op order)
         * outranks single-digit-% CPU gains. See ENABLE_NATIVE for the
         * opt-in auto-vectorized build. */
        math4 rotation_matrix = vector4_to_math4(rigid_body->orientation);
        vector3 model_scale;
        if (rigid_body->type == object_sphere) {
            model_scale = (vector3){rigid_body->radius, rigid_body->radius, rigid_body->radius};
        } else if (rigid_body->type == object_cylinder) {
            /* Unit mesh: axle X half-length 1, radius 1 (see
             * cylinder_meshing.h): scale columns to (h, r, r). */
            model_scale =
                (vector3){rigid_body->cylinder_half_length, rigid_body->radius, rigid_body->radius};
        } else {
            model_scale = rigid_body->half_extensions;
        }
        float scale_comp[3] = {model_scale.x, model_scale.y, model_scale.z};
        math4 model_matrix = {{{0}}};
        for (int mc = 0; mc < 3; mc++) {
            for (int mr = 0; mr < 3; mr++) {
                model_matrix.matrix[mc][mr] = rotation_matrix.matrix[mc][mr] * scale_comp[mc];
            }
            model_matrix.matrix[mc][3] = 0.0f;
            model_matrix.matrix[3][mc] = (mc == 0) ? rigid_body->position.x
                                        : (mc == 1) ? rigid_body->position.y
                                                    : rigid_body->position.z;
        }
        model_matrix.matrix[3][3] = 1.0f;
        float *target_array;
        int *target_count;
        if (rigid_body->type == object_sphere) {
            target_array = sphere_instances;
            target_count = &sphere_inst_count;
        } else if (rigid_body->type == object_cylinder) {
            target_array = cylinder_instances;
            target_count = &cylinder_inst_count;
        } else {
            target_array = cube_instances;
            target_count = &cube_inst_count;
        }
        if ((*target_count) < mpe_max_bodies) {
            int idx = (*target_count) * 19;
            math4_to_flat_array(model_matrix, &target_array[idx]);
            target_array[idx + 16] = rigid_body->colour.x;
            target_array[idx + 17] = rigid_body->colour.y;
            target_array[idx + 18] = rigid_body->colour.z;
            (*target_count)++;
        }
    }
    glUseProgram(instanced_shader_program);
    glUniformMatrix4fv(instanced_uniforms.projection_matrix_location, 1, GL_FALSE, projection_matrix_flat_array);
    glUniformMatrix4fv(instanced_uniforms.view_matrix_location, 1, GL_FALSE, view_matrix_flat_array);
    glUniform3f(instanced_uniforms.camera_position_location, main_camera_fov.position.x, main_camera_fov.position.y,
                main_camera_fov.position.z);
    glUniform3f(instanced_uniforms.light_position_location, g_cfg.render.light_x, g_cfg.render.light_y,
                g_cfg.render.light_z);
    glUniform1f(instanced_uniforms.ambient_strength_location, g_cfg.render.ambient_strength);
    glUniform1f(instanced_uniforms.specular_coeff_location, g_cfg.render.specular_coeff);
    glUniform1f(instanced_uniforms.specular_exponent_location, g_cfg.render.specular_exponent);
    if (sphere_inst_count > 0) {
        glBindBuffer(GL_ARRAY_BUFFER, sphere_mesh.instance_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sphere_inst_count * 19 * sizeof(float), sphere_instances);
        glBindVertexArray(sphere_mesh.vertex_array_object);
        glDrawElementsInstanced(GL_TRIANGLES, sphere_mesh.index_count, GL_UNSIGNED_INT, 0, sphere_inst_count);
    }
    if (cube_inst_count > 0) {
        glBindBuffer(GL_ARRAY_BUFFER, cube_mesh.instance_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, cube_inst_count * 19 * sizeof(float), cube_instances);
        glBindVertexArray(cube_mesh.vertex_array_object);
        glDrawElementsInstanced(GL_TRIANGLES, cube_mesh.index_count, GL_UNSIGNED_INT, 0, cube_inst_count);
    }
    if (cylinder_inst_count > 0) {
        glBindBuffer(GL_ARRAY_BUFFER, cylinder_mesh.instance_vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, cylinder_inst_count * 19 * sizeof(float), cylinder_instances);
        glBindVertexArray(cylinder_mesh.vertex_array_object);
        glDrawElementsInstanced(GL_TRIANGLES, cylinder_mesh.index_count, GL_UNSIGNED_INT, 0, cylinder_inst_count);
    }
    glBindVertexArray(0);
    spring_joint_render(utility_shader_program, view_matrix, projection_matrix);
    wireframe_render_selected_object(utility_shader_program, view_matrix, projection_matrix);
}

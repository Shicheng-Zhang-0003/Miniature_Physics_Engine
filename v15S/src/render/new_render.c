/* GTK4-PREP: zero GUI headers in render (GL only). */
#include <epoxy/gl.h>
#include "../core/mpe_platform.h"
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
#include <stdio.h>
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
static void render_delete_gl_objects (void);
void render_cleanup (void);
void render_init () {
    /* Re-init safe: initialising twice used to either leak (re-running
     * creation) or wedge on stale state (early return). Tear down the
     * previous GL + CPU state first so re-init is a clean rebuild. A
     * previous failure simply retries. */
    if (render_init_status == render_ok) {
        render_cleanup ();
    }
    render_init_status = render_uninitialized;
    const char *shader_dir = getenv ("MPE_SHADER_DIR");
    char vs_path[512], fs_path[512], uvs_path[512], ufs_path[512];
    if (shader_dir && shader_dir[0]) {
        snprintf (vs_path, sizeof (vs_path), "%s/vertex_shader.glsl", shader_dir);
        snprintf (fs_path, sizeof (fs_path), "%s/fragment_shader.glsl", shader_dir);
        snprintf (uvs_path, sizeof (uvs_path), "%s/utility_vertex.glsl", shader_dir);
        snprintf (ufs_path, sizeof (ufs_path), "%s/utility_fragment.glsl", shader_dir);
    } else {
        snprintf (vs_path, sizeof (vs_path), "render/shaders/vertex_shader.glsl");
        snprintf (fs_path, sizeof (fs_path), "render/shaders/fragment_shader.glsl");
        snprintf (uvs_path, sizeof (uvs_path), "render/shaders/utility_vertex.glsl");
        snprintf (ufs_path, sizeof (ufs_path), "render/shaders/utility_fragment.glsl");
    }
    /* Installed fallback: <prefix>/share/mpe/shaders (see make install). */
    instanced_shader_program = create_shader_program (vs_path, fs_path);
    if (instanced_shader_program == 0) {
        char alt_vs[512], alt_fs[512];
        const char *home = mpe_home_dir ();
        if (home) {
            snprintf (alt_vs, sizeof (alt_vs), "%s/.local/share/mpe/shaders/vertex_shader.glsl", home);
            snprintf (alt_fs, sizeof (alt_fs), "%s/.local/share/mpe/shaders/fragment_shader.glsl", home);
            instanced_shader_program = create_shader_program (alt_vs, alt_fs);
        }
    }
    utility_shader_program = create_shader_program (uvs_path, ufs_path);
    if ((utility_shader_program == 0) && (instanced_shader_program != 0)) {
        /* DESPOT-2026-10-04: the utility shader had no ~/.local fallback
         * while the instanced one did — a missing utility file red-screened
         * the whole renderer even with a good instanced program. Retry it. */
        const char *home_u = mpe_home_dir ();
        if (home_u) {
            char alt_uvs[512], alt_ufs[512];
            snprintf (alt_uvs, sizeof (alt_uvs), "%s/.local/share/mpe/shaders/utility_vertex.glsl", home_u);
            snprintf (alt_ufs, sizeof (alt_ufs), "%s/.local/share/mpe/shaders/utility_fragment.glsl", home_u);
            utility_shader_program = create_shader_program (alt_uvs, alt_ufs);
        }
    }
    if ((instanced_shader_program == 0) || (utility_shader_program == 0)) {
        fprintf (stderr, "RENDER INIT FAILED: shader program creation failed (instanced=%u, utility=%u)\n",
                 instanced_shader_program, utility_shader_program);
        render_delete_gl_objects ();
        render_init_status = render_failed;
        return;
    }
    /* DESPOT-2026-10-04: glGetUniformLocation == -1 (renamed/missing GLSL
     * uniform) used to fail silently — every later glUniform is a
     * spec-defined no-op with wrong lighting and no diagnostic. Name the
     * missing uniform loudly; keep rendering (locations stay -1 = no-op). */
#define MPE_CHECK_UNIFORM(field, prog, name)                                                                           \
    do {                                                                                                               \
        (field) = glGetUniformLocation ((prog), (name));                                                               \
        if ((field) == -1) {                                                                                           \
            fprintf (stderr, "[render] WARNING: uniform '%s' not found (program %u)\n", (name), (unsigned) (prog));    \
        }                                                                                                              \
    } while (0)
    MPE_CHECK_UNIFORM (instanced_uniforms.projection_matrix_location, instanced_shader_program, "projection");
    MPE_CHECK_UNIFORM (instanced_uniforms.view_matrix_location, instanced_shader_program, "viewframe");
    MPE_CHECK_UNIFORM (instanced_uniforms.camera_position_location, instanced_shader_program, "camera_position");
    MPE_CHECK_UNIFORM (instanced_uniforms.light_position_location, instanced_shader_program, "light_position");
    MPE_CHECK_UNIFORM (instanced_uniforms.ambient_strength_location, instanced_shader_program, "u_ambient_strength");
    MPE_CHECK_UNIFORM (instanced_uniforms.specular_coeff_location, instanced_shader_program, "u_specular_coeff");
    MPE_CHECK_UNIFORM (instanced_uniforms.specular_exponent_location, instanced_shader_program, "u_specular_exponent");
    MPE_CHECK_UNIFORM (utility_uniforms.projection_matrix_location, utility_shader_program, "projection");
    MPE_CHECK_UNIFORM (utility_uniforms.view_matrix_location, utility_shader_program, "viewframe");
    MPE_CHECK_UNIFORM (utility_uniforms.model_matrix_location, utility_shader_program, "model");
    MPE_CHECK_UNIFORM (utility_uniforms.normal_matrix_location, utility_shader_program, "normal_matrix");
    MPE_CHECK_UNIFORM (utility_uniforms.object_colour_location, utility_shader_program, "object_colour");
    MPE_CHECK_UNIFORM (utility_uniforms.camera_position_location, utility_shader_program, "camera_position");
    MPE_CHECK_UNIFORM (utility_uniforms.light_position_location, utility_shader_program, "light_position");
#undef MPE_CHECK_UNIFORM
    grid_init (&main_grid, 250, 5);
    init_sm_system (&sphere_mesh, 32, 32);
    cube_meshing_init ();
    init_cylinder_system (&cylinder_mesh, 24);
    sphere_instances = malloc (mpe_max_bodies * 19 * sizeof (float));
    cube_instances = malloc (mpe_max_bodies * 19 * sizeof (float));
    cylinder_instances = malloc (mpe_max_bodies * 19 * sizeof (float));
    if (!sphere_instances || !cube_instances || !cylinder_instances) {
        fprintf (stderr, "RENDER INIT FAILED: instance buffer OOM\n");
        free (sphere_instances);
        sphere_instances = NULL;
        free (cube_instances);
        cube_instances = NULL;
        free (cylinder_instances);
        cylinder_instances = NULL;
        render_delete_gl_objects ();
        render_init_status = render_failed;
        return;
    }
    render_init_status = render_ok;
}
/* Delete one instanced mesh's GL objects (all ids zero-guarded so a
 * context-less or repeated cleanup is a safe no-op). */
static void render_delete_mesh (mesh *mesh_object) {
    if (!mesh_object)
        return;
    if (mesh_object->vertex_array_object) {
        glDeleteVertexArrays (1, &mesh_object->vertex_array_object);
        mesh_object->vertex_array_object = 0;
    }
    if (mesh_object->vertex_buffer_object) {
        glDeleteBuffers (1, &mesh_object->vertex_buffer_object);
        mesh_object->vertex_buffer_object = 0;
    }
    if (mesh_object->element_buffer_object) {
        glDeleteBuffers (1, &mesh_object->element_buffer_object);
        mesh_object->element_buffer_object = 0;
    }
    if (mesh_object->wireframe_element_buffer_object) {
        glDeleteBuffers (1, &mesh_object->wireframe_element_buffer_object);
        mesh_object->wireframe_element_buffer_object = 0;
    }
    if (mesh_object->instance_vbo) {
        glDeleteBuffers (1, &mesh_object->instance_vbo);
        mesh_object->instance_vbo = 0;
    }
    mesh_object->index_count = 0;
    mesh_object->wireframe_index_count = 0;
    mesh_object->instance_capacity = 0;
}
/* Unconditional GL teardown shared by render_cleanup and the mid-init
 * failure paths (which run with a current context but a not-yet-ok
 * status). All ids are zero-guarded. */
static void render_delete_gl_objects (void) {
    if (instanced_shader_program) {
        glDeleteProgram (instanced_shader_program);
        instanced_shader_program = 0;
    }
    if (utility_shader_program) {
        glDeleteProgram (utility_shader_program);
        utility_shader_program = 0;
    }
    render_delete_mesh (&sphere_mesh);
    render_delete_mesh (&cube_mesh);
    render_delete_mesh (&cylinder_mesh);
    if (main_grid.vertex_array_object) {
        glDeleteVertexArrays (1, &main_grid.vertex_array_object);
        main_grid.vertex_array_object = 0;
    }
    if (main_grid.vertex_buffer_object) {
        glDeleteBuffers (1, &main_grid.vertex_buffer_object);
        main_grid.vertex_buffer_object = 0;
    }
    main_grid.line_vertex_count = 0;
    wireframe_invalidate_cache ();
    grid_invalidate_cache ();
}
void render_cleanup (void) {
    /* GL objects: the old cleanup freed only the CPU instance buffers and
     * leaked every program/VAO/VBO on each init/cleanup cycle. Delete them
     * here (callers run on the GL thread with the context current — see
     * root_gtk destroy paths). Gated on render_ok so a context-less or
     * repeated cleanup never issues GL calls for objects that were never
     * created; every id is additionally zero-guarded. Uniform caches in
     * wireframe/grid are invalidated because GL reuses deleted ids. */
    if (render_init_status == render_ok) {
        render_delete_gl_objects ();
    }
    if (sphere_instances) {
        free (sphere_instances);
        sphere_instances = NULL;
    }
    if (cube_instances) {
        free (cube_instances);
        cube_instances = NULL;
    }
    if (cylinder_instances) {
        free (cylinder_instances);
        cylinder_instances = NULL;
    }
    render_init_status = render_uninitialized;
}
/* Primary-world-only renderer: draws physics_world_get_primary().
 * TODO(world-param): take an explicit physics_world* so headless/secondary
 * worlds can render without relying on the app-owned primary. */
void render_scene_current (int widget_width, int widget_height) {
    if (widget_width <= 0 || widget_height <= 0) {
        return;
    }
    /* DESPOT-2026-09-29: this guarded only render_failed. render_uninitialized
     * fell through, and the three instance buffers are NULL until render_init()
     * mallocs them, so the body loop wrote through a null pointer:
     *   target_array = sphere_instances;   -> NULL
     *   math4_to_flat_array(m, &target_array[idx]);  -> write to address 0
     * Reachable: root_gtk.c connects the window "destroy" handler to
     * render_cleanup (which frees the buffers and resets the status to
     * render_uninitialized) independently of the "render" signal, so a frame
     * delivered during teardown with body_count > 0 writes through 0. */
    if (render_init_status == render_uninitialized) {
        glViewport (0, 0, widget_width, widget_height);
        glClearColor (0.05f, 0.05f, 0.1f, 1.0f);
        glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        return;
    }
    if (render_init_status == render_failed) {
        glViewport (0, 0, widget_width, widget_height);
        glClearColor (0.5f, 0.0f, 0.0f, 1.0f);
        glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        return;
    }
    glViewport (0, 0, widget_width, widget_height);
    glClearColor (0.05f, 0.05f, 0.1f, 1.0f);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    float window_aspect_ratio = (float) (widget_width) / (float) (widget_height);
    math4 projection_matrix = math4_perspective_fov (degrad * 45.0f, window_aspect_ratio, 0.1f, 1000.0f);
    float projection_matrix_flat_array[16];
    math4_to_flat_array (projection_matrix, projection_matrix_flat_array);
    math4 view_matrix =
        math4_look_view (main_camera_fov.position, main_camera_fov.forward_vector, main_camera_fov.vertical_vector);
    float view_matrix_flat_array[16];
    math4_to_flat_array (view_matrix, view_matrix_flat_array);
    grid_render (&main_grid, utility_shader_program, view_matrix, projection_matrix);
    /* Frustum culling: extract the six inward-facing planes from the
     * column-major view-projection matrix (Gribb/Hartmann) and skip
     * packing/uploading/drawing fully-outside bodies. Bounding spheres
     * come from broadphase_bounding_radius (rotation-invariant, never
     * wrongly excludes). Zero persistent memory: planes live on stack. */
    math4 view_projection = math4_multiplication (projection_matrix, view_matrix);
    /* DESPOT-2026-09-29: the plane extraction and the sphere test now live in
     * math4_special.h so the shipped culler is reachable from a headless test
     * (mpe_t_frustum_culler). Previously it was inline here, in a GL function
     * nothing could call, so no gate ever executed the real culler. */
    float frustum_planes[6][4];
    math4_frustum_planes (view_projection, frustum_planes);
    int sphere_inst_count = 0;
    int cube_inst_count = 0;
    int cylinder_inst_count = 0;
    for (int object_index = 0; object_index < (physics_world_get_primary ()->body_count); object_index++) {
        rigidbody *rigid_body = &(physics_world_get_primary ()->bodies)[object_index];
        /* DESPOT-2026-10-04: a NaN body used to poison the instance buffer
         * (NaN model matrix → NaN vertices → driver-dependent garbage or
         * worse). Skip non-finite bodies loudly; the physics side already
         * counts them and the save path refuses to persist them. */
        if ((!isfinite (rigid_body->position.x)) || (!isfinite (rigid_body->position.y)) ||
            (!isfinite (rigid_body->position.z)) || (!isfinite (rigid_body->orientation.w)) ||
            (!isfinite (rigid_body->orientation.x)) || (!isfinite (rigid_body->orientation.y)) ||
            (!isfinite (rigid_body->orientation.z))) {
            fprintf (stderr, "[render] WARNING: skipping non-finite body %d\n", object_index);
            continue;
        }
        /* Sphere-vs-frustum: culled only when fully outside one plane. */
        {
            float bound = broadphase_bounding_radius (rigid_body);
            if (!math4_frustum_sphere_visible (frustum_planes, rigid_body->position.x, rigid_body->position.y,
                                               rigid_body->position.z, bound)) {
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
        math4 rotation_matrix = vector4_to_math4 (rigid_body->orientation);
        vector3 model_scale;
        if (rigid_body->type == object_sphere) {
            model_scale = (vector3){rigid_body->radius, rigid_body->radius, rigid_body->radius};
        } else if (rigid_body->type == object_cylinder) {
            /* Unit mesh: axle X half-length 1, radius 1 (see
             * cylinder_meshing.h): scale columns to (h, r, r). */
            model_scale = (vector3){rigid_body->cylinder_half_length, rigid_body->radius, rigid_body->radius};
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
            model_matrix.matrix[3][mc] = (mc == 0)   ? rigid_body->position.x
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
            math4_to_flat_array (model_matrix, &target_array[idx]);
            target_array[idx + 16] = rigid_body->colour.x;
            target_array[idx + 17] = rigid_body->colour.y;
            target_array[idx + 18] = rigid_body->colour.z;
            (*target_count)++;
        }
    }
    glUseProgram (instanced_shader_program);
    glUniformMatrix4fv (instanced_uniforms.projection_matrix_location, 1, GL_FALSE, projection_matrix_flat_array);
    glUniformMatrix4fv (instanced_uniforms.view_matrix_location, 1, GL_FALSE, view_matrix_flat_array);
    glUniform3f (instanced_uniforms.camera_position_location, main_camera_fov.position.x, main_camera_fov.position.y,
                 main_camera_fov.position.z);
    glUniform3f (instanced_uniforms.light_position_location, g_cfg.render.light_x, g_cfg.render.light_y,
                 g_cfg.render.light_z);
    glUniform1f (instanced_uniforms.ambient_strength_location, g_cfg.render.ambient_strength);
    glUniform1f (instanced_uniforms.specular_coeff_location, g_cfg.render.specular_coeff);
    glUniform1f (instanced_uniforms.specular_exponent_location, g_cfg.render.specular_exponent);
    if (sphere_inst_count > 0) {
        glBindBuffer (GL_ARRAY_BUFFER, sphere_mesh.instance_vbo);
        glBufferSubData (GL_ARRAY_BUFFER, 0, sphere_inst_count * 19 * sizeof (float), sphere_instances);
        glBindVertexArray (sphere_mesh.vertex_array_object);
        glDrawElementsInstanced (GL_TRIANGLES, sphere_mesh.index_count, GL_UNSIGNED_INT, 0, sphere_inst_count);
    }
    if (cube_inst_count > 0) {
        glBindBuffer (GL_ARRAY_BUFFER, cube_mesh.instance_vbo);
        glBufferSubData (GL_ARRAY_BUFFER, 0, cube_inst_count * 19 * sizeof (float), cube_instances);
        glBindVertexArray (cube_mesh.vertex_array_object);
        glDrawElementsInstanced (GL_TRIANGLES, cube_mesh.index_count, GL_UNSIGNED_INT, 0, cube_inst_count);
    }
    if (cylinder_inst_count > 0) {
        glBindBuffer (GL_ARRAY_BUFFER, cylinder_mesh.instance_vbo);
        glBufferSubData (GL_ARRAY_BUFFER, 0, cylinder_inst_count * 19 * sizeof (float), cylinder_instances);
        glBindVertexArray (cylinder_mesh.vertex_array_object);
        glDrawElementsInstanced (GL_TRIANGLES, cylinder_mesh.index_count, GL_UNSIGNED_INT, 0, cylinder_inst_count);
    }
    glBindVertexArray (0);
    spring_joint_render (utility_shader_program, view_matrix, projection_matrix);
    wireframe_render_selected_object (utility_shader_program, view_matrix, projection_matrix);
    /* DESPOT-2026-10-04: zero glGetError coverage meant every GL misuse
     * above (zero VBO/VAO, bad enum, torn context) failed silently. Drain
     * the error queue once per frame and name it — one bounded stderr line
     * per code, not per call. */
    {
        GLenum render_err = glGetError ();
        if (render_err != GL_NO_ERROR) {
            fprintf (stderr, "[render] WARNING: glGetError=0x%x in render_scene_current\n", (unsigned) render_err);
            while (glGetError () != GL_NO_ERROR) {
            }
        }
    }
}

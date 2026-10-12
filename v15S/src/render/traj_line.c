/* Purple predicted-trajectory polyline (see traj_line.h).
 * GTK4-PREP: zero GUI headers in render (GL only). Uses the same utility
 * shader and per-draw object_colour uniform as the grid, so the line lights
 * consistently with the floor and needs no new shader. The vertex buffer is
 * small and re-uploaded each frame with glBufferData (the predictor already
 * throttles how often it recomputes, so this is a few hundred floats). */
#include "traj_line.h"
#include "../core/physics_world.h"
#include "../config/mpe_config.h"
#include "../ui_input/camera.h"
#include <epoxy/gl_generated.h>
#include <math.h>
#include <stdlib.h>
extern camera main_camera_fov;
/* Purple, a touch brighter than pure so it reads against the dark clear. */
#define TRAJ_R 0.62f
#define TRAJ_G 0.13f
#define TRAJ_B 0.86f
void traj_line_init (traj_line_mesh *m) {
    if (!m)
        return;
    m -> vertex_count = 0;
    m -> cpu = NULL;
    m -> cpu_cap = 0;
    glGenVertexArrays (1, &m -> vertex_array_object);
    glGenBuffers (1, &m -> vertex_buffer_object);
    glBindVertexArray (m -> vertex_array_object);
    glBindBuffer (GL_ARRAY_BUFFER, m -> vertex_buffer_object);
    glBufferData (GL_ARRAY_BUFFER, 0, NULL, GL_DYNAMIC_DRAW);
    glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof (float), (void *) 0);
    glEnableVertexAttribArray (0);
    glBindVertexArray (0);
}
void traj_line_update (traj_line_mesh *m, const vector3 *pts, int n) {
    if (!m)
        return;
    /* n points -> (n-1) segments -> (n-1)*2 vertices -> (n-1)*6 floats. */
    int segs = (pts && n >= 2) ? (n - 1) : 0;
    int floats = segs * 6;
    if (floats <= 0) {
        m -> vertex_count = 0;
        return;
    }
    if (m -> cpu_cap < floats) {
        float *nc = (float *) realloc (m -> cpu, (size_t) floats * sizeof (float));
        if (!nc) {
            m -> vertex_count = 0;
            return;
        }
        m -> cpu = nc;
        m -> cpu_cap = floats;
    }
    int k = 0;
    for (int i = 0; i < segs; i++) {
        const vector3 *a = &pts [i];
        const vector3 *b = &pts [i + 1];
        m -> cpu [k++] = a -> x;
        m -> cpu [k++] = a -> y;
        m -> cpu [k++] = a -> z;
        m -> cpu [k++] = b -> x;
        m -> cpu [k++] = b -> y;
        m -> cpu [k++] = b -> z;
    }
    m -> vertex_count = floats;
    /* Upload now so render() stays a pure draw (GL context is current here;
     * the caller updates during the frame before the draw pass). */
    glBindBuffer (GL_ARRAY_BUFFER, m -> vertex_buffer_object);
    glBufferData (GL_ARRAY_BUFFER, (size_t) floats * sizeof (float), m -> cpu, GL_DYNAMIC_DRAW);
    glBindBuffer (GL_ARRAY_BUFFER, 0);
}
/* Cached uniform locations (grid does the same; GL ids are reused after a
 * context rebuild, so the cache is dropped by traj_line_cleanup). */
static GLuint s_cached_program = 0;
static GLint s_u_viewframe = -1, s_u_projection = -1, s_u_model = -1, s_u_normal_matrix = -1, s_u_object_colour = -1,
              s_u_camera_position = -1, s_u_light_position = -1, s_u_ambient = -1, s_u_spec_coeff = -1, s_u_spec_exp = -1;
static void traj_cache_uniforms (GLuint prog) {
    if (prog == s_cached_program)
        return;
    s_cached_program = prog;
    s_u_viewframe = glGetUniformLocation (prog, "viewframe");
    s_u_projection = glGetUniformLocation (prog, "projection");
    s_u_model = glGetUniformLocation (prog, "model");
    s_u_normal_matrix = glGetUniformLocation (prog, "normal_matrix");
    s_u_object_colour = glGetUniformLocation (prog, "object_colour");
    s_u_camera_position = glGetUniformLocation (prog, "camera_position");
    s_u_light_position = glGetUniformLocation (prog, "light_position");
    s_u_ambient = glGetUniformLocation (prog, "u_ambient_strength");
    s_u_spec_coeff = glGetUniformLocation (prog, "u_specular_coeff");
    s_u_spec_exp = glGetUniformLocation (prog, "u_specular_exponent");
}
void traj_line_render (traj_line_mesh *m, GLuint shader_program, math4 view_matrix, math4 projection_matrix) {
    if (!m || m -> vertex_count <= 0)
        return;
    glUseProgram (shader_program);
    traj_cache_uniforms (shader_program);
    float viewf [16], projf [16];
    math4_to_flat_array (view_matrix, viewf);
    math4_to_flat_array (projection_matrix, projf);
    glUniformMatrix4fv (s_u_viewframe, 1, GL_FALSE, viewf);
    glUniformMatrix4fv (s_u_projection, 1, GL_FALSE, projf);
    math4 model = math4_identity ();
    float modelf [16];
    math4_to_flat_array (model, modelf);
    glUniformMatrix4fv (s_u_model, 1, GL_FALSE, modelf);
    math3 nrm = math3_identity ();
    float nrmf [9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            nrmf [i * 3 + j] = nrm.matrix [i][j];
    glUniformMatrix3fv (s_u_normal_matrix, 1, GL_FALSE, nrmf);
    glUniform3f (s_u_camera_position, main_camera_fov.position.x, main_camera_fov.position.y,
                 main_camera_fov.position.z);
    glUniform3f (s_u_light_position, g_cfg.render.light_x, g_cfg.render.light_y, g_cfg.render.light_z);
    glUniform1f (s_u_ambient, g_cfg.render.ambient_strength);
    glUniform1f (s_u_spec_coeff, g_cfg.render.specular_coeff);
    glUniform1f (s_u_spec_exp, g_cfg.render.specular_exponent);
    if (s_u_object_colour >= 0)
        glUniform3f (s_u_object_colour, TRAJ_R, TRAJ_G, TRAJ_B);
    glBindVertexArray (m -> vertex_array_object);
    /* Constant surface normal for the (unlit-meaningful) line, same ordering
     * rule as grid: bind the VAO FIRST, then set the generic attribute. */
    glVertexAttrib3f (1, 0.0f, 1.0f, 0.0f);
    glDrawArrays (GL_LINES, 0, m -> vertex_count / 3);
    glBindVertexArray (0);
}
void traj_line_cleanup (traj_line_mesh *m) {
    if (m) {
        if (m -> vertex_array_object) {
            glDeleteVertexArrays (1, &m -> vertex_array_object);
            m -> vertex_array_object = 0;
        }
        if (m -> vertex_buffer_object) {
            glDeleteBuffers (1, &m -> vertex_buffer_object);
            m -> vertex_buffer_object = 0;
        }
        free (m -> cpu);
        m -> cpu = NULL;
        m -> cpu_cap = 0;
        m -> vertex_count = 0;
    }
    s_cached_program = 0;
    s_u_viewframe = -1;
    s_u_projection = -1;
    s_u_model = -1;
    s_u_normal_matrix = -1;
    s_u_object_colour = -1;
    s_u_camera_position = -1;
    s_u_light_position = -1;
    s_u_ambient = -1;
    s_u_spec_coeff = -1;
    s_u_spec_exp = -1;
}
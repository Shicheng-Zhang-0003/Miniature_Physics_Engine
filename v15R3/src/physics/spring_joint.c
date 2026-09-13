#include "../mpe_engine.h"
#include "spring_joint.h"
#include "../core/physics_world.h"
#include <stdio.h>
#include <stdlib.h>
#include <epoxy/gl.h>

/* Per-world spring pool. No file-scope pool remains. */

static rigidbody *spring_find_body(rigidbody *bodies, int body_count, uint32_t object_id) {
    if ((!bodies) || (object_id == 0)) {
        return NULL;
    }
    for (int i = 0; i < body_count; i++) {
        if (bodies[i].object_id == object_id) {
            return &bodies[i];
        }
    }
    return NULL;
}

void joint_init_pool(physics_world *world) {
    if (!world) {
        return;
    }
    for (int joint_index = 0; joint_index < mpe_max_joints; joint_index++) {
        world->spring_joints[joint_index].is_active = false;
    }
    world->spring_joint_count = 0;
}

int spring_joint_count(const physics_world *world) {
    if (!world) {
        return 0;
    }
    return world->spring_joint_count;
}

int add_joint_by_ids(physics_world *world, uint32_t object_id_a, uint32_t object_id_b, float equilibrium_length,
                     float spring_constant, float damping_coefficient) {
    if (!world) {
        return -1;
    }
    if ((object_id_a == 0) || (object_id_b == 0) || (object_id_a == object_id_b)) {
        return -1;
    }

    for (int joint_index = 0; joint_index < mpe_max_joints; joint_index++) {
        if (!world->spring_joints[joint_index].is_active) {
            world->spring_joints[joint_index].object_id_a = object_id_a;
            world->spring_joints[joint_index].object_id_b = object_id_b;
            world->spring_joints[joint_index].equilibrium_length = equilibrium_length;
            world->spring_joints[joint_index].spring_constant = spring_constant;
            world->spring_joints[joint_index].damping_coefficient = damping_coefficient;
            world->spring_joints[joint_index].is_active = true;
            world->spring_joint_count += 1;
            return joint_index;
        }
    }

    fprintf(stderr, "Error SJA001: No Remaining Space in Buffer\n");
    return -1;
}

int add_joint(physics_world *world, int object_index_a, int object_index_b, float equilibrium_length,
              float spring_constant, float damping_coefficient) {
    if (!world) {
        return -1;
    }
    if ((object_index_a < 0) || (object_index_a >= world->body_count) || (object_index_b < 0) ||
        (object_index_b >= world->body_count)) {
        fprintf(stderr, "Error SJA002: Invalid joint object index\n");
        return -1;
    }

    uint32_t object_id_a = world->bodies[object_index_a].object_id;
    uint32_t object_id_b = world->bodies[object_index_b].object_id;

    return add_joint_by_ids(world, object_id_a, object_id_b, equilibrium_length, spring_constant,
                            damping_coefficient);
}

void remove_joint(physics_world *world, int joint_pool_index) {
    if (!world) {
        return;
    }
    if ((joint_pool_index < 0) || (joint_pool_index >= mpe_max_joints)) {
        return;
    }
    if (!world->spring_joints[joint_pool_index].is_active) {
        return;
    }
    world->spring_joints[joint_pool_index].is_active = false;
    world->spring_joint_count -= 1;
}

/* Shared Hooke+damping+limit core over an explicit body array. Both step
 * paths funnel through here (legacy passes the primary world's bodies). */
static void spring_apply_core(physics_world *world, rigidbody *bodies, int body_count) {
    for (int joint_index = 0; joint_index < mpe_max_joints; joint_index++) {
        if (!world->spring_joints[joint_index].is_active) {
            continue;
        }

        spring_joint *current_spring_joint = &world->spring_joints[joint_index];

        rigidbody *rigid_body_a = spring_find_body(bodies, body_count, current_spring_joint->object_id_a);
        rigidbody *rigid_body_b = spring_find_body(bodies, body_count, current_spring_joint->object_id_b);

        if ((!rigid_body_a) || (!rigid_body_b)) {
            remove_joint(world, joint_index);
            continue;
        }

        vector3 displacement_vector = vector3_subtraction(rigid_body_b->position, rigid_body_a->position);
        float current_separation_distance = vector3_length(displacement_vector);

        /* FIX-AUDIT: coincident bodies with L0>0 need maximal repulsion,
         * not skip. Pick an arbitrary axis. */
        vector3 spring_axis_direction;
        if (current_separation_distance < math_epsilon) {
            spring_axis_direction = (vector3){1.0f, 0.0f, 0.0f};
            current_separation_distance = 0.0f;
        } else {
            spring_axis_direction = vector3_scaling(displacement_vector, 1.0f / current_separation_distance);
        }

        float spring_extension = current_separation_distance - current_spring_joint->equilibrium_length;

        vector3 restoration_force =
            vector3_scaling(spring_axis_direction, current_spring_joint->spring_constant * spring_extension);

        vector3 relative_velocity = vector3_subtraction(rigid_body_b->velocity, rigid_body_a->velocity);
        float velocity_along_spring_axis = vector3_dot(relative_velocity, spring_axis_direction);

        vector3 damping_force = vector3_scaling(spring_axis_direction,
                                                current_spring_joint->damping_coefficient * velocity_along_spring_axis);
        vector3 net_joint_force = vector3_addition(restoration_force, damping_force);

        float a3_inverse_mass_sum = rigid_body_a->inverse_mass + rigid_body_b->inverse_mass;
        if (a3_inverse_mass_sum > 0.0f) {
            /* FIX-AUDIT: explicit-Euler springs go unstable for
             * w*dt = sqrt(k/m)*dt > 2 (e.g. k=5000,m=0.01 -> 11.8).
             * The Fmax limiter below keeps it bounded but silently breaks
             * Hooke's law; that tradeoff is retained and documented. */
            float a3_reduced_mass = 1.0f / a3_inverse_mass_sum;
            float a3_max_joint_force = a3_reduced_mass * g_cfg.joints.max_acceleration;
            float a3_force_length = vector3_length(net_joint_force);
            if ((a3_force_length > a3_max_joint_force) && (a3_force_length > math_epsilon)) {
                net_joint_force = vector3_scaling(net_joint_force, a3_max_joint_force / a3_force_length);
            }
        }
        rb_apply_forces_perfect(rigid_body_a, net_joint_force);
        rb_apply_forces_perfect(rigid_body_b, vector3_scaling(net_joint_force, -1.0f));
    }
}

/* Legacy entry point: applies the owning world's pool to its own bodies. */
void apply_force_all_joints(physics_world *world) {
    if ((!world) || (!world->bodies) || (world->body_count <= 0)) {
        return;
    }
    spring_apply_core(world, world->bodies, world->body_count);
}

/* World-aware spring pass over an explicit body array (headless + world
 * step path). Pool comes from the given world. */
void apply_spring_forces_world(physics_world *world, rigidbody *bodies, int body_count) {
    if ((!world) || (!bodies) || (body_count <= 0)) {
        return;
    }
    spring_apply_core(world, bodies, body_count);
}

void remove_joints_from_object_id(physics_world *world, uint32_t object_id) {
    if ((!world) || (object_id == 0)) {
        return;
    }

    for (int joint_index = 0; joint_index < mpe_max_joints; joint_index++) {
        if (!world->spring_joints[joint_index].is_active) {
            continue;
        }

        if ((world->spring_joints[joint_index].object_id_a == object_id) ||
            (world->spring_joints[joint_index].object_id_b == object_id)) {
            remove_joint(world, joint_index);
        }
    }
}

void remove_joints_from_object(physics_world *world, int object_index) {
    if ((!world) || (object_index < 0) || (object_index >= world->body_count)) {
        return;
    }
    remove_joints_from_object_id(world, world->bodies[object_index].object_id);
}

static GLuint joint_vao = 0;
static GLuint joint_vbo = 0;

static GLuint a3_spring_cached_program = 0;
static GLint a3_spring_uniform_viewframe = -1;
static GLint a3_spring_uniform_projection = -1;
static GLint a3_spring_uniform_model = -1;
static GLint a3_spring_uniform_normal_matrix = -1;
static GLint a3_spring_uniform_object_colour = -1;

static void a3_spring_cache_uniforms(GLuint shader_program) {
    if (shader_program == a3_spring_cached_program) {
        return;
    }

    a3_spring_cached_program = shader_program;
    a3_spring_uniform_viewframe = glGetUniformLocation(shader_program, "viewframe");
    a3_spring_uniform_projection = glGetUniformLocation(shader_program, "projection");
    a3_spring_uniform_model = glGetUniformLocation(shader_program, "model");
    a3_spring_uniform_normal_matrix = glGetUniformLocation(shader_program, "normal_matrix");
    a3_spring_uniform_object_colour = glGetUniformLocation(shader_program, "object_colour");
}

static GLint a3_spring_uniform_camera_position = -1;
static GLint a3_spring_uniform_light_position = -1;
static GLint a3_spring_uniform_ambient = -1;
static GLint a3_spring_uniform_specular_coeff = -1;
static GLint a3_spring_uniform_specular_exp = -1;

static void a3_spring_cache_missing_uniforms(GLuint shader_program) {
    static GLuint a3_spring_missing_cached_program = 0;

    if (shader_program == a3_spring_missing_cached_program) {
        return;
    }

    a3_spring_missing_cached_program = shader_program;
    a3_spring_uniform_camera_position = glGetUniformLocation(shader_program, "camera_position");
    a3_spring_uniform_light_position = glGetUniformLocation(shader_program, "light_position");
    a3_spring_uniform_ambient = glGetUniformLocation(shader_program, "u_ambient_strength");
    a3_spring_uniform_specular_coeff = glGetUniformLocation(shader_program, "u_specular_coeff");
    a3_spring_uniform_specular_exp = glGetUniformLocation(shader_program, "u_specular_exponent");
}

void spring_joint_render(GLuint shader_program, math4 view_matrix, math4 projection_matrix) {
    /* Single-viewport renderer: draws the primary world's joints. */
    physics_world *world = physics_world_get_primary();
    if (!world) {
        return;
    }
    int active_count = 0;

    for (int i = 0; i < mpe_max_joints; i++) {
        if (!world->spring_joints[i].is_active) {
            continue;
        }

        rigidbody *rb_a = scene_resolve_object_by_id(world->spring_joints[i].object_id_a);
        rigidbody *rb_b = scene_resolve_object_by_id(world->spring_joints[i].object_id_b);

        if ((rb_a) && (rb_b)) {
            active_count++;
        }
    }

    if (active_count == 0) {
        return;
    }
    if (active_count > mpe_max_joints) {
        active_count = mpe_max_joints;
    } /* A3_PATCH_28_PERSISTENT_JOINT_BUFFER */

    static float vertices[mpe_max_joints * 2 * 3]; /* A3_PATCH_28_PERSISTENT_JOINT_BUFFER */

    int v_idx = 0;

    for (int i = 0; i < mpe_max_joints; i++) {
        if (!world->spring_joints[i].is_active) {
            continue;
        }

        rigidbody *rb_a = scene_resolve_object_by_id(world->spring_joints[i].object_id_a);
        rigidbody *rb_b = scene_resolve_object_by_id(world->spring_joints[i].object_id_b);

        if ((rb_a) && (rb_b)) {
            vertices[v_idx++] = rb_a->position.x;
            vertices[v_idx++] = rb_a->position.y;
            vertices[v_idx++] = rb_a->position.z;
            vertices[v_idx++] = rb_b->position.x;
            vertices[v_idx++] = rb_b->position.y;
            vertices[v_idx++] = rb_b->position.z;
        }
    }

    if (joint_vao == 0) {
        glGenVertexArrays(1, &joint_vao);
        glGenBuffers(1, &joint_vbo);
        glBindVertexArray(joint_vao);
        glBindBuffer(GL_ARRAY_BUFFER, joint_vbo);
        glBufferData(GL_ARRAY_BUFFER, mpe_max_joints * 2 * 3 * sizeof(float), NULL, GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *) 0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }

    glBindBuffer(GL_ARRAY_BUFFER, joint_vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, active_count * 2 * 3 * sizeof(float), vertices);

    glUseProgram(shader_program);

    a3_spring_cache_missing_uniforms(shader_program);
    a3_spring_cache_uniforms(shader_program);
    float view_matrix_flat_array[16], projection_matrix_flat_array[16];
    math4_to_flat_array(view_matrix, view_matrix_flat_array);
    math4_to_flat_array(projection_matrix, projection_matrix_flat_array);

    glUniformMatrix4fv(a3_spring_uniform_viewframe, 1, GL_FALSE, view_matrix_flat_array);
    glUniformMatrix4fv(a3_spring_uniform_projection, 1, GL_FALSE, projection_matrix_flat_array);

    math4 model_matrix = math4_identity();
    float model_matrix_flat_array[16];
    math4_to_flat_array(model_matrix, model_matrix_flat_array);
    glUniformMatrix4fv(a3_spring_uniform_model, 1, GL_FALSE, model_matrix_flat_array);

    math3 identity_normal_matrix = math3_identity();
    float normal_matrix_flat_array[9];

    for (int row_index = 0; row_index < 3; row_index++) {
        for (int column_index = 0; column_index < 3; column_index++) {
            normal_matrix_flat_array[row_index * 3 + column_index] =
                identity_normal_matrix.matrix[row_index][column_index];
        }
    }

    glUniformMatrix3fv(a3_spring_uniform_normal_matrix, 1, GL_FALSE, normal_matrix_flat_array);
    glUniform3f(a3_spring_uniform_object_colour, 1.0f, 0.0f, 1.0f);
    glUniform3f(a3_spring_uniform_camera_position, main_camera_fov.position.x, main_camera_fov.position.y,
                main_camera_fov.position.z);
    glUniform3f(a3_spring_uniform_light_position, g_cfg.render.light_x, g_cfg.render.light_y, g_cfg.render.light_z);
    glUniform1f(a3_spring_uniform_ambient, g_cfg.render.ambient_strength);
    glUniform1f(a3_spring_uniform_specular_coeff, g_cfg.render.specular_coeff);
    glUniform1f(a3_spring_uniform_specular_exp, g_cfg.render.specular_exponent);
    glVertexAttrib3f(1, 0.0f, 1.0f, 0.0f);

    glBindVertexArray(joint_vao);
    glDrawArrays(GL_LINES, 0, active_count * 2);
    glBindVertexArray(0);
}

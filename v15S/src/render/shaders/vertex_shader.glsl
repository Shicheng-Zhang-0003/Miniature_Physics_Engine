#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in mat4 instance_model;
layout (location = 6) in vec3 instance_color;
uniform mat4 viewframe;
uniform mat4 projection;
out vec3 normal;
out vec3 fragment_position;
out vec3 local_position;
out vec3 out_color;
void main () {
    mat4 model = instance_model;
    gl_Position = projection * viewframe * model * vec4 (aPos, 1.0f);
    fragment_position = vec3 (model * vec4 (aPos, 1.0f));
    local_position = aPos;
    /* A3_PATCH_29_SHADER_NORMALS */
mat3 a3_rotation_scale = mat3 (model);
vec3 a3_scale = vec3 (
    max (length (a3_rotation_scale [0]), 0.0001),
    max (length (a3_rotation_scale [1]), 0.0001),
    max (length (a3_rotation_scale [2]), 0.0001)
);
mat3 a3_rotation = mat3 (
    a3_rotation_scale [0] / a3_scale.x,
    a3_rotation_scale [1] / a3_scale.y,
    a3_rotation_scale [2] / a3_scale.z
);
normal = normalize (a3_rotation * (aNormal / a3_scale));
    out_color = instance_color;
}

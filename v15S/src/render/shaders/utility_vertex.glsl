#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
uniform mat4 model;
uniform mat4 viewframe;
uniform mat4 projection;
uniform mat3 normal_matrix;
out vec3 normal;
out vec3 fragment_position;
out vec3 local_position;
void main () {
    gl_Position = projection * viewframe * model * vec4 (aPos, 1.0f);
    fragment_position = vec3 (model * vec4 (aPos, 1.0f));
    local_position = aPos;
    normal = normal_matrix * aNormal;
}

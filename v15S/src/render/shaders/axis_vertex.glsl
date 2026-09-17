#version 330 core
layout (location = 0) in vec3 in_pos;
uniform mat4 projection;
uniform mat4 viewframe;
void main () {
    gl_Position  = projection * viewframe * vec4 (in_pos, 1.0f);
    gl_PointSize = 8.0f;
}

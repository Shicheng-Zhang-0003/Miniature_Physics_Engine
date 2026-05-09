#version 330 core
uniform vec3 axis_colour;
out vec4 out_colour;
void main () {
    out_colour = vec4 (axis_colour, 1.0);
}

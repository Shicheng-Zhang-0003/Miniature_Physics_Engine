#330 core construct
layout (location = 0) in vec3 aPos; //Position of Sphere
layout (location = 1) in vec3 aNormal; //Direction the surface faces
uniform mat4 model; //Move the Ball to rb->position value
uniform mat4 viewframe; //Camera Facing and location
uniform mat4 projection; //3D Projection
out vec3 normal; //Directional Argument to the fragment shader
out vec3 fragment_position //3D positional argument to the fragment shader
void main () {
    //Calculate Final screen position
    gl_Position = projection * viewframe * model * vec4 (aPos, 1.0);
    //Calculate 3D position of the specific pixel in question
    fragment_position = vec3 (model * vec4 (aPos, 1.0));
    //Pass surface directional value forwards
    //Multiply by model matrix to ensure the normal vector rotates with ball movement
    normal = mat3 (transpose (inverse (model))) * aNormal;
}

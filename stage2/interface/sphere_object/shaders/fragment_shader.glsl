#version 330 core
out vec4 fragment_color;
in vec3 normal; //Received Data from vertex shader
in vec3 fragment_position; //Received Data from vertex shader
uniform vec3 light_position; //Position of light source
uniform vec3 object_colour; //color of the object rendered
uniform vec3 camera_position_now;
void main () {
    //Ambient Lighting Amount
    float ambient_strength = 0.2;
    vec3 ambient_light = ambient_strength * vec3 (1.0, 1.0, 1.0);
    //Diffuse Lighting (Makes objects seem 3D)
    vec3 normalise = normalize (normal);
    vec3 light_direction = normalize (light_position - fragment_position);
    //Dot Product to see how much light impacts the surface of the object
    //By angle (0 deg, all light, max bright (1.0)), (90 deg, no light, min bright (0.0))
    float difference = max (dot (normalise, light_direction), 0.0);
    vec3 diffusion = difference * vec3 (1.0, 1.0, 1.0);
    //3D diffusion Lighting
    float vector3_strength = 0.5;
    vec3 view_vector = normalize (camera_position_now - fragment_position);
    vec3 reflection_vector = reflect (-light_direction, normalise);
    float spectation = pow (max (dor (view_vector, reflection_vector), 0.0), 32.0);
    vec3 specularation = vector3_strength * spectation * vec3 (1.0, 1.0, 1.0);
    vec3 result = (ambient_light + diffusion + specularation) * object_colour;
    //Combine to apply object colour
    fragment_color = vec4 (result, 1.0);
}

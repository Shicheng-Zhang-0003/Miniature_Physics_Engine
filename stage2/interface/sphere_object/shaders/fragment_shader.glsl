#version 330 core
out vec4 fragment_color;
in vec3 normal; //Received Data from vertex shader
in vec3 fragment_position; //Received Data from vertex shader
uniform vec3 light_position; //Position of light source
uniform vec3 object_colour; //color of the object rendered
uniform vec3 camera_position;
void main () {
    //Ambient Lighting Amount (Increased for visibility)
    float ambient_strength = 0.4;
    vec3 ambient_light = ambient_strength * vec3 (1.0, 1.0, 1.0);
    //Diffuse Lighting (Makes objects seem 3D)
    vec3 normalise = normalize (normal);
    vec3 light_direction = normalize (light_position - fragment_position);
    //Dot Product to see how much light impacts the surface of the object
    //By angle (0 deg, all light, max bright (1.0)), (90 deg, no light, min bright (0.0))
    float difference = max (dot (normalise, light_direction), 0.0);
    vec3 diffusion = difference * vec3 (1.0, 1.0, 1.0);
    //3D diffusion Lighting (Increased Specular)
    float specular_lighting_intensity_coefficient = 0.8;
    vec3 camera_to_fragment_view_direction_vector = normalize (camera_position - fragment_position);
    vec3 light_reflection_direction_vector = reflect (-light_direction, normalise);
    float specular_exponent_factor = pow (max (dot (camera_to_fragment_view_direction_vector, light_reflection_direction_vector), 0.0), 32.0);
    vec3 specular_lighting_result_colour = specular_lighting_intensity_coefficient * specular_exponent_factor * vec3 (1.0, 1.0, 1.0);
    vec3 final_calculated_pixel_colour = (ambient_light + diffusion + specular_lighting_result_colour) * object_colour;
    //Combine to apply object colour
    fragment_color = vec4 (final_calculated_pixel_colour, 1.0);
}

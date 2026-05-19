#version 330 core
out vec4 fragment_color;
in vec3 normal; //Received Data from vertex shader
in vec3 fragment_position; //Received Data from vertex shader
in vec3 local_position; //Local Position of Object
uniform vec3 light_position; //Position of light source
uniform vec3 object_colour; //color of the object rendered
uniform vec3 camera_position;
void main () {
    //Ambient Lighting Amount (Increased for visibility)
    float ambient_strength = 0.6f;
    vec3 ambient_light = ambient_strength * vec3 (1.0f, 1.0f, 1.0f);
    //Diffuse Lighting (Makes objects seem 3D)
    vec3 normalise = normalize (normal);
    vec3 light_direction = normalize (light_position - fragment_position);
    //Dot Product to see how much light impacts the surface of the object
    //By angle (0 deg, all light, max bright (1.0f)), (90 deg, no light, min bright (0.0f))
    float difference = max (dot (normalise, light_direction), 0.0f);
    vec3 diffusion = difference * vec3 (1.0f, 1.0f, 1.0f);
    //3D diffusion Lighting (Increased Specular)
    float specular_lighting_intensity_coefficient = 0.8f;
    vec3 camera_to_fragment_view_direction_vector = normalize (camera_position - fragment_position);
    vec3 light_reflection_direction_vector = reflect (-light_direction, normalise);
    float specular_exponent_factor = pow (max (dot (camera_to_fragment_view_direction_vector, light_reflection_direction_vector), 0.0f), 32.0f);
    vec3 specular_lighting_result_colour = specular_lighting_intensity_coefficient * specular_exponent_factor * vec3 (1.0f, 1.0f, 1.0f);
    vec3 final_calculated_pixel_colour = (ambient_light + diffusion + specular_lighting_result_colour) * object_colour;
    // Axis rings — painted onto local midplanes so they rotate with the object
    // X ring (YZ plane, x=0): red — tells you X-axis rotation
    // Y ring (XZ plane, y=0): green — tells you Y-axis rotation
    // Z ring (XY plane, z=0): blue — tells you Z-axis rotation
    float stripe_width = 0.06;
    float x_ring = 1.0 - smoothstep (0.0, stripe_width, abs (local_position.x));
    float y_ring = 1.0 - smoothstep (0.0, stripe_width, abs (local_position.y));
    float z_ring = 1.0 - smoothstep (0.0, stripe_width, abs (local_position.z));
    vec3 ring_colour = vec3 (0.0);
    ring_colour += x_ring * vec3 (0.9, 0.15, 0.15);
    ring_colour += y_ring * vec3 (0.15, 0.9, 0.15);
    ring_colour += z_ring * vec3 (0.15, 0.35, 1.0);
    float ring_mask = clamp (x_ring + y_ring + z_ring, 0.0, 1.0);
    final_calculated_pixel_colour = mix (final_calculated_pixel_colour, ring_colour, ring_mask * 0.85);
    fragment_color = vec4 (final_calculated_pixel_colour, 1.0f);
}

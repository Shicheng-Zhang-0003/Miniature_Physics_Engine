#version 330 core
out vec4 fragment_color;
in vec3 normal;
in vec3 fragment_position;
in vec3 local_position;
in vec3 out_color;
uniform vec3 light_position;
uniform vec3 camera_position;
void main () {
    float ambient_strength = 0.6f;
    vec3 ambient_light = ambient_strength * vec3 (1.0f, 1.0f, 1.0f);
    vec3 normalise = normalize (normal);
    vec3 light_direction = normalize (light_position - fragment_position);
    float difference = max (dot (normalise, light_direction), 0.0f);
    vec3 diffusion = difference * vec3 (1.0f, 1.0f, 1.0f);
    float specular_lighting_intensity_coefficient = 0.8f;
    vec3 camera_to_fragment_view_direction_vector = normalize (camera_position - fragment_position);
    vec3 light_reflection_direction_vector = reflect (-light_direction, normalise);
    float specular_exponent_factor = pow (max (dot (camera_to_fragment_view_direction_vector, light_reflection_direction_vector), 0.0f), 32.0f);
    vec3 specular_lighting_result_colour = specular_lighting_intensity_coefficient * specular_exponent_factor * vec3 (1.0f, 1.0f, 1.0f);
    vec3 final_calculated_pixel_colour = (ambient_light + diffusion + specular_lighting_result_colour) * out_color;
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

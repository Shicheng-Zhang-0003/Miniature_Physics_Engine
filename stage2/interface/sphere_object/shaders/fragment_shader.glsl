#v330 core

out vec4 fragment_color;
in vec3 normal; //Received Data from vertex shader
in vec3 fragment_position //Received Data from vertex shader

uniform vec3 light_position //Position of light source
uniform vec3 object_colour //color of the object rendered

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
    //Combine to apply object colour
    vec3 result = (ambient + diffuse) * object_colour;
    fragment_color = vec4 (result, 1.0);
}

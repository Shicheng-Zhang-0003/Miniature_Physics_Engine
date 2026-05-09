#include <epoxy/gl.h>
#include <stdio.h>
#include <stdlib.h>
GLuint compile_shader (const char *shader_source, GLenum shader_type) {
    GLuint shader_object = glCreateShader (shader_type);
    glShaderSource (shader_object, 1, &shader_source, NULL);
    glCompileShader (shader_object);
    //Error Check
    int compilation_success;
    char information_log [512];
    glGetShaderiv (shader_object, GL_COMPILE_STATUS, &compilation_success);
    if (!compilation_success) {
        glGetShaderInfoLog (shader_object, 512, NULL, information_log);
        fprintf (stderr, "Shader compile failed: \n%s\n", information_log);
    } return shader_object;
} GLuint create_shader_program (const char *vertex_shader_path, const char *fragment_shader_path) {
    // Vertex shaders source code file loading
    FILE *vertex_shader_file = fopen (vertex_shader_path, "r");
    FILE *fragment_shader_file = fopen (fragment_shader_path, "r");
    if (!vertex_shader_file || !fragment_shader_file) {
        if (vertex_shader_file) fclose (vertex_shader_file);
        if (fragment_shader_file) fclose (fragment_shader_file);
        fprintf (stderr, "Shader file opening error \n");
        return 0;
    } fseek (vertex_shader_file, 0, SEEK_END); long vertex_file_size = ftell (vertex_shader_file); rewind (vertex_shader_file);
    fseek (fragment_shader_file, 0, SEEK_END); long fragment_file_size = ftell (fragment_shader_file); rewind (fragment_shader_file);
    char *vertex_shader_source = malloc (vertex_file_size + 1);
    char *fragment_shader_source = malloc (fragment_file_size + 1);
    if (!vertex_shader_source || !fragment_shader_source) {
        if (vertex_shader_source) free (vertex_shader_source);
        if (fragment_shader_source) free (fragment_shader_source);
        fclose (vertex_shader_file); fclose (fragment_shader_file);
        fprintf (stderr, "Memory allocation error for shader source\n");
        return 0;
    } if (fread (vertex_shader_source, 1, vertex_file_size, vertex_shader_file) != (size_t) vertex_file_size) {
        fprintf (stderr, "Error reading vertex shader\n");
    } vertex_shader_source [vertex_file_size] = '\0';
    if (fread (fragment_shader_source, 1, fragment_file_size, fragment_shader_file) != (size_t) fragment_file_size) {
        fprintf (stderr, "Error reading fragment shader\n");
    } fragment_shader_source [fragment_file_size] = '\0';
    fclose (vertex_shader_file); fclose (fragment_shader_file);
    // Compilation Process
    GLuint vertex_shader = compile_shader (vertex_shader_source, GL_VERTEX_SHADER);
    GLuint fragment_shader = compile_shader (fragment_shader_source, GL_FRAGMENT_SHADER);
    free (vertex_shader_source);
    free (fragment_shader_source);
    if (!vertex_shader || !fragment_shader) {
        if (vertex_shader) glDeleteShader (vertex_shader);
        if (fragment_shader) glDeleteShader (fragment_shader);
        return 0;
    }
    // Linkage and .o elf
    GLuint shader_program = glCreateProgram ();
    glAttachShader (shader_program, vertex_shader);
    glAttachShader (shader_program, fragment_shader);
    glLinkProgram (shader_program);
    // Linkage error checking
    int linkage_success;
    char information_log [512];
    glGetProgramiv (shader_program, GL_LINK_STATUS, &linkage_success);
    if (!linkage_success) {
        glGetProgramInfoLog (shader_program, 512, NULL, information_log);
        fprintf (stderr, "Shader linkage error: %s\n", information_log);
        glDeleteProgram (shader_program);
        shader_program = 0;
    } glDeleteShader (vertex_shader);
    glDeleteShader (fragment_shader);
    return shader_program;
}

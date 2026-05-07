#include <epoxy/gl.h>
#include <stdio.h>
#include <stdlib.h>
GLuint compiler_shader (const char *source, GLenum type) {
    GLuint shader = glCreateShader (type);
    glShaderSource (shader, 1, &source, NULL);
    glCompileShader (shader);
    //Error Check
    int success;
    char log [512];
    glGetShaderiv (shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog (shader, 512, NULL, log);
        fprintf (stderr, "Shader compile failed: \n%s\n", log);
    } return shader;
} GLuint create_shader_program (const char *vertex_path, const char *fragment_path) {
    // Vertex shaders source code file loading
    FILE *vf = fopen (vertex_path, "r");
    FILE *ff = fopen (fragment_path, "r");
    if (!vf || !ff) {
        if (vf) fclose (vf);
        if (ff) fclose (ff);
        fprintf (stderr, "Shader file opening error \n");
        return 0;
    } fseek (vf, 0, SEEK_END); long vsize = ftell (vf); rewind (vf);
    fseek (ff, 0, SEEK_END); long fsize = ftell (ff); rewind (ff);
    char *vsource = malloc (vsize + 1);
    char *fsource = malloc (fsize + 1);
    if (!vsource || !fsource) {
        if (vsource) free (vsource);
        if (fsource) free (fsource);
        fclose (vf); fclose (ff);
        fprintf (stderr, "Memory allocation error for shader source\n");
        return 0;
    } if (fread (vsource, 1, vsize, vf) != (size_t) vsize) {
        fprintf (stderr, "Error reading vertex shader\n");
    } vsource [vsize] = '\0';
    if (fread (fsource, 1, fsize, ff) != (size_t) fsize) {
        fprintf (stderr, "Error reading fragment shader\n");
    } fsource [fsize] = '\0';
    fclose (vf); fclose (ff);
    // Compilation Process
    GLuint vertex = compiler_shader (vsource, GL_VERTEX_SHADER);
    GLuint fragment = compiler_shader (fsource, GL_FRAGMENT_SHADER);
    free (vsource);
    free (fsource);
    if (!vertex || !fragment) {
        if (vertex) glDeleteShader (vertex);
        if (fragment) glDeleteShader (fragment);
        return 0;
    }
    // Linkage and .o elf
    GLuint program = glCreateProgram ();
    glAttachShader (program, vertex);
    glAttachShader (program, fragment);
    glLinkProgram (program);
    // Linkage error checking
    int success;
    char log [512];
    glGetProgramiv (program, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog (program, 512, NULL, log);
        fprintf (stderr, "Shader linkage error: %s\n", log);
        glDeleteProgram (program);
        program = 0;
    } glDeleteShader (vertex);
    glDeleteShader (fragment);
    return program;
}

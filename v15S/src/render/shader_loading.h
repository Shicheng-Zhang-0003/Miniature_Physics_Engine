#ifndef shader_loading_h
#define shader_loading_h
#include <epoxy/gl.h>
GLuint compile_shader(const char *source, GLenum type);
GLuint create_shader_program(const char *vertex_path, const char *fragment_path);
#endif

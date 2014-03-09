#ifndef TEXTURE_H
#define TEXTURE_H

#include <GL/glew.h>

GLuint loadBMP_custom(const char * imagepath);
// load a .DDS file using GLFW's own loader
GLuint loadDDS(const char* imagepath);

#endif // TEXTURE_H

#ifndef SHADER_H
#define SHADER_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <string>

using namespace std;

static string readFile(const char* filename);
GLuint loadShader(const char* path_vs, const char* path_fs);

#endif
#ifndef CURSOR_H
#define CURSOR_H

#include "obj3d.h"
#include <GLFW/glfw3.h>

using namespace glm;

class cursor : public obj3d
{
private:
	mat4 inc;
	float width, height;
	float deltaTime;

public:
	cursor(GLuint program, const char* filename_obj, const char* filename_texture, const bool isDDS);
	bool init(GLFWwindow* window, float width, float height, const char* name_vp);
	void computeCursorPos(GLFWwindow* window);
	void computeCursorRot();
	bool setUniformMat4Cursor(const char* name);
};

#endif

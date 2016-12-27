#ifndef CAMERA_H
#define CAMERA_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "glm/glm.hpp"


#ifndef PI
#define PI			3.14159265358f
#endif

#ifndef PI_OVER_TWO
#define PI_OVER_TWO 1.57079632679f
#endif

using namespace glm;

class camera{
private:
	mat4 view;
	mat4 proj;
	vec2 center;
	vec3 position;
	float horizontalAngle;
	float verticalAngle;
	float FOV;
	float keySpeed;
	float mouseSpeed;
	int mouseDirUp, mouseDirRight;

public:
	camera(unsigned int x, unsigned int y);
	camera(vec3 pos, unsigned int x, unsigned int y);

	void computeFirstPersonVP(GLFWwindow* window, bool isMouse);

	bool setUniformMat4VP(GLuint program, const char* name_VP);
	bool setUniformMat4V(GLuint program, const char* name_V);
	bool setUniformMat4P(GLuint program, const char* name_P);

	mat4 getView();
	void setView(mat4 v);

	mat4 getProj();
	void setProj(mat4 p);

	void setCenter(unsigned int x, unsigned int y);
	void setPosition(vec3 pos);
	void setDirection(float angle_h, float angle_v);
	void setFOV(float fov);
	void setKeySpeed(float speed);
	void setMouseSpeed(float speed);
	void setMouseDir(int up, int right);
};

#endif

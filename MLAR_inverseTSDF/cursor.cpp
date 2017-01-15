#include "cursor.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/euler_angles.hpp"

using namespace glm;


/*=====================================
Constructor
=====================================*/
cursor::cursor(GLuint program, const char* filename_obj, const char* filename_texture, const bool isDDS) : obj3d(program, filename_obj, filename_texture, isDDS)
{
	inc = eulerAngleYXZ(0.f, 3.14f, 0.7f);
	scal = scale(mat4(1.f), vec3(0.1f, 0.1f, 0.1f));
	deltaTime = 0;
}


/*=====================================
Cursor initialization
=====================================*/
bool cursor::init(GLFWwindow* window, float width, float height, const char* name_vp)
{
	this->width = width;
	this->height = height;
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	computeCursorPos(window);

	mat4 view = lookAt(vec3(0, 0, 1.f), vec3(), vec3(0, 1, 0));
	mat4 proj = ortho(0.f, width/100, height/100, 0.f, 0.1f, 100.f);

	//Set uniform matrix vp
	glUseProgram(program);
	GLint loc = glGetUniformLocation(program, name_vp);
	if(loc == -1) return false;

	glUniformMatrix4fv(loc, 1, GL_FALSE, value_ptr(proj * view));
	return true;
}


/*======================================
Compute cursor position
======================================*/
void cursor::computeCursorPos(GLFWwindow* window)
{
	double x, y;
	glfwGetCursorPos(window, &x, &y);

	if(x < 0) x = 0;
	else if(x > width) x = width;

	if(y < 0) y = 0;
	else if(y > height) y = height;

	glfwSetCursorPos(window, x, y);

	trans = translate(mat4(1.f), vec3((float)x/100, (float)y/100, 0));
}


/*======================================
Compute cursor rotation
======================================*/
void cursor::computeCursorRot()
{
	static double lastTime = glfwGetTime();

	double curTime = glfwGetTime();
	deltaTime = float(curTime - lastTime);

	rot = eulerAngleYXZ(-deltaTime, 0.f, 0.f);
}


/*======================================
Set uniform matrix (pos * int * rot)
======================================*/
bool cursor::setUniformMat4Cursor(const char* name)
{
	glUseProgram(this->program);
	GLint loc = glGetUniformLocation(this->program, name);
	if(loc == -1) return false;

	glUniformMatrix4fv(loc, 1, GL_FALSE, value_ptr(trans * inc * rot * scal));

	return true;
}

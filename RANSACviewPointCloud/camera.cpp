#include "camera.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

using namespace glm;


/*=======================================
Constructor
=======================================*/
camera::camera(unsigned int x, unsigned int y)
{
	center = vec2(x, y);
	position = vec3(0, 0, 5);
	horizontalAngle = PI;
	verticalAngle = 0.f;
	FOV = 45.f;
	keySpeed = 10.f;
	mouseSpeed = 0.003f;
	mouseDirUp = mouseDirRight = 1;
}

camera::camera(vec3 pos, unsigned int x, unsigned int y)
{
	center = vec2(x, y);
	position = pos;
	horizontalAngle = PI;
	verticalAngle = 0.f;
	FOV = 45.f;
	keySpeed = 10.f;
	mouseSpeed = 0.003f;
	mouseDirUp = mouseDirRight = 1;
}


/*=======================================
Compute 1st person VP matrix
=======================================*/
void camera::computeFirstPersonVP(GLFWwindow* window)
{
	static double lastTime = glfwGetTime();
	
	double curTime = glfwGetTime();
	float deltaTime = float(curTime - lastTime);
	
	//Get mouse position
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);
	
	//Reset mouse position for next frame
	glfwSetCursorPos(window, center.x, center.y);
	
	//Comput new orientation
	horizontalAngle += mouseDirRight * mouseSpeed * float(center.x - xpos);
	verticalAngle 	+= mouseDirUp * mouseSpeed * float(center.y - ypos);
	
	//Direction: spherical coordinates to Cartesian coordinates conversion
	vec3 direction(
		cos(verticalAngle) * sin(horizontalAngle),
		sin(verticalAngle),
		cos(verticalAngle) * cos(horizontalAngle)
	);
	
	//Right vector
	vec3 right = vec3(
		sin(horizontalAngle - PI_OVER_TWO),
		0,
		cos(horizontalAngle - PI_OVER_TWO)
	);
	
	//Up vector
	vec3 up = cross(right, direction);
	
	//Move forward
	if(glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		position += direction * deltaTime * keySpeed;
	
	//Move backward
	if(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		position -= direction * deltaTime * keySpeed;
	
	//Strafe right
	if(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		position += right * deltaTime * keySpeed;
	
	//Strafe left
	if(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		position -= right * deltaTime * keySpeed;
	
	//Move upward
	if(glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
		position += vec3(0, 1, 0) * deltaTime * keySpeed;
	
	//Move downward
	if(glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
		position -= vec3(0, 1, 0) * deltaTime * keySpeed;
	
	//Projection matrix
	proj = perspective(FOV, 4.f/3.f, 0.1f, 10000.f);
	
	//Camera matrix
	view = lookAt(position, position+direction, up);
				
	//For the next frame
	lastTime = curTime;
}


/*=======================================
Set uniform matrix of VP in the shader
=======================================*/
bool camera::setUniformMat4VP(GLuint program, const char* name_VP)
{
	glUseProgram(program);
	GLint loc = glGetUniformLocation(program, name_VP);
	if(loc == -1) return false;
	
	mat4 vp = this->proj * this->view;
	glUniformMatrix4fv(loc, 1, GL_FALSE, value_ptr(vp));
	
	return true;
}


/*=======================================
Set uniform matrix of view in the shader
=======================================*/
bool camera::setUniformMat4V(GLuint program, const char* name_V)
{
	glUseProgram(program);
	GLint loc = glGetUniformLocation(program, name_V);
	if(loc == -1) return false;
	
	glUniformMatrix4fv(loc, 1, GL_FALSE, value_ptr(this->view));
	
	return true;
}


/*=======================================
Set uniform matrix of projection in the shader
=======================================*/
bool camera::setUniformMat4P(GLuint program, const char* name_P)
{
	glUseProgram(program);
	GLint loc = glGetUniformLocation(program, name_P);
	if(loc == -1) return false;
	
	glUniformMatrix4fv(loc, 1, GL_FALSE, value_ptr(this->proj));
	
	return true;
}


/*=======================================
Set window center
=======================================*/
void camera::setCenter(unsigned int x, unsigned int y)
{
	this->center = vec2(x, y);
}


/*=======================================
Access view matrix
=======================================*/
mat4 camera::getView()
{
	return this->view;
}


void camera::setView(mat4 v)
{
	this->view = v;
}


/*=======================================
Access projection matrix
=======================================*/
mat4 camera::getProj()
{
	return this->proj;
}


void camera::setProj(mat4 p)
{
	this->proj = p;
}


/*=======================================
Set position
=======================================*/
void camera::setPosition(vec3 pos)
{
	this->position = pos;
}


/*=======================================
Set direction
=======================================*/
void camera::setDirection(float angle_h, float angle_v)
{
	this->horizontalAngle = angle_h;
	this->verticalAngle = angle_v;
}


/*=======================================
Set field of veiw
=======================================*/
void camera::setFOV(float fov)
{
	this->FOV = fov;
}


/*=======================================
Set key speed
=======================================*/
void camera::setKeySpeed(float speed)
{
	this->keySpeed = speed;
}


/*=======================================
Set mouse speed
=======================================*/
void camera::setMouseSpeed(float speed)
{
	this->mouseSpeed = speed;
}


/*=======================================
Set mouse direction
=======================================*/
void camera::setMouseDir(int up, int right)
{
	if(up >= 0) mouseDirUp = 1;
	else mouseDirUp = -1;
	
	if(right >= 0) mouseDirRight = 1;
	else mouseDirRight = -1;
}

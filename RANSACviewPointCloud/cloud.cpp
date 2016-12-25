#include "cloud.h"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include <vector>

using namespace glm;

cloud::cloud(GLuint program)
{
	glGenVertexArrays(1, &(this->vao));
	glGenBuffers(4, this->vbo);
	this->program = program;
	model = mat4(1.f);
	pointNum = 0;
}


cloud::cloud(GLuint program, std::vector<vec3> &points)
{
	glGenVertexArrays(1, &(this->vao));
	glGenBuffers(4, this->vbo);
	glBindVertexArray(this->vao);
	uploadPos(points, 0);
	this->program = program;
	model = mat4(1.f);
	pointNum = points.size();
}


cloud::~cloud()
{
	releaseBuffer();
}


void cloud::uploadPoints(std::vector<vec3> &points)
{
	glBindVertexArray(this->vao);
	uploadPos(points, 0);
	pointNum = points.size();
}
	
	
bool cloud::setUniformMat4Model(const char* name)
{
	glUseProgram(this->program);
	GLint loc = glGetUniformLocation(this->program, name);
	if(loc == -1) return false;
	
	glUniformMatrix4fv(loc, 1, GL_FALSE, value_ptr(this->model));
	
	return true;
}
	
	
void cloud::setModel(mat4 mat)
{
	model = mat;
}

mat4 cloud::getModel()
{
	return model;
}


void cloud::setPointNum(unsigned int num)
{
	pointNum = num;
}


unsigned int cloud::getPointNum()
{
	return pointNum;
}
	
	
void cloud::render()
{
	glUseProgram(program);
	glBindVertexArray(vao);
	glDrawArrays(GL_POINTS, 0, pointNum);
}


void cloud::releaseBuffer()
{
	glDeleteVertexArrays(1, &vao);
	glDeleteBuffers(4, vbo);
}

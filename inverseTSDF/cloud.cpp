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
	pointNum = 0;
}


cloud::cloud(GLuint program, std::vector<vec3> &points)
{
	glGenVertexArrays(1, &(this->vao));
	glGenBuffers(4, this->vbo);
	glBindVertexArray(this->vao);
	uploadPos(points, 0);
	this->program = program;
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

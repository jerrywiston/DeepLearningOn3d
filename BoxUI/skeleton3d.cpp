#include "skeleton3d.h"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"

using namespace glm;


skeleton3d::skeleton3d(GLuint program)
{
	this->color = vec3(1.0f, 1.0f, 1.0f);
	this->program = program;
	this->vertexNum = 0;

	glGenVertexArrays(1, &(this->vao));
	glGenBuffers(4, this->vbo);
}


skeleton3d::skeleton3d(GLuint program, std::vector<vec3> &vertex)
{
	this->program = program;

	glGenVertexArrays(1, &(this->vao));
	glGenBuffers(4, this->vbo);
	glBindVertexArray(this->vao);

	if(vertex.size() > 0){
		uploadPos(vertex, 0);
		this->vertexNum = vertex.size();
	}
	else this->vertexNum = 0;
}


skeleton3d::~skeleton3d()
{
	releaseBuffer();
}


void skeleton3d::setVertex(std::vector<vec3> &vertex)
{
	glBindVertexArray(this->vao);
	if(vertex.size() > 0){
		uploadPos(vertex, 0);
		this->vertexNum = vertex.size();
	}
}


void skeleton3d::render()
{
	glUseProgram(program);
	glBindVertexArray(vao);
	glDrawArrays(GL_LINES, 0, vertexNum);
}


void skeleton3d::releaseBuffer()
{
	glDeleteVertexArrays(1, &vao);
	glDeleteBuffers(4, vbo);
}

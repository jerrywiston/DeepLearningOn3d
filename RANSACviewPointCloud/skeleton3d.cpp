#include "skeleton3d.h"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"

using namespace glm;


skeleton3d::skeleton3d(GLuint program)
{
	this->program = program;
	this->model = mat4(1.f);
	this->vertexNum = 0;
	
	glGenVertexArrays(1, &(this->vao));
	glGenBuffers(4, this->vbo);
}


skeleton3d::skeleton3d(GLuint program, std::vector<vec3> &vertex)
{
	this->program = program;
	this->model = mat4(1.f);
	
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
	

bool skeleton3d::setUniformMat4Model(const char* name)
{
	glUseProgram(this->program);
	GLint loc = glGetUniformLocation(this->program, name);
	if(loc == -1) return false;
	
	glUniformMatrix4fv(loc, 1, GL_FALSE, value_ptr(this->model));
	
	return true;
}
	

void skeleton3d::setModel(mat4 mat)
{
	this->model = mat;
}


mat4 skeleton3d::getModel()
{
	return this->model;
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

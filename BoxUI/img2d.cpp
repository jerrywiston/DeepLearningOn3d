#include "img2d.h"
#include <vector>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

using namespace glm;


img2d::img2d(GLuint program, const char* filename, bool isDDS)
{
	this->program = program;
	loadText(filename, isDDS);
}


img2d::~img2d()
{
	releaseBuffer();
}


void img2d::loadText(const char* filename, bool isDDS)
{
	//Create VAO/VBO
	glGenVertexArrays(1, &(this->vao));
	glGenBuffers(4, this->vbo);
	texture = (isDDS) ? loadDDS(filename) : loadBMP(filename);

	isLoaded = (texture > 0) ? true : false;
}


void img2d::show(int x, int y, int size)
{
	//Fill buffers
	std::vector<vec2> vertex;
	std::vector<vec2> uv;

	//Calculate vertex
	vec2 vertex_up_left    = vec2(x		   , y + size);
	vec2 vertex_up_right   = vec2(x + size , y + size);
	vec2 vertex_down_right = vec2(x + size , y);
	vec2 vertex_down_left  = vec2(x 	   , y);

	vertex.push_back(vertex_up_left);
	vertex.push_back(vertex_down_left);
	vertex.push_back(vertex_up_right);

	vertex.push_back(vertex_down_right);
	vertex.push_back(vertex_up_right);
	vertex.push_back(vertex_down_left);

	//Calculate uv
	vec2 uv_up_left 	= vec2(0, 1);
	vec2 uv_up_right 	= vec2(1, 1);
	vec2 uv_down_right 	= vec2(1, 0);
	vec2 uv_down_left 	= vec2(0, 0);

	uv.push_back(uv_up_left);
	uv.push_back(uv_down_left);
	uv.push_back(uv_up_right);

	uv.push_back(uv_down_right);
	uv.push_back(uv_up_right);
	uv.push_back(uv_down_left);

	//Upload buffer data
	glBindVertexArray(this->vao);

	uploadPos(vertex, 0);
	uploadUV(uv, 1);

	glBindVertexArray(0);
}


bool img2d::is_loaded()
{
	return isLoaded;
}


void img2d::render()
{
	glUseProgram(program);
	glBindVertexArray(vao);
	glBindTexture(GL_TEXTURE_2D, texture);

	//Enable alpha blending
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//Draw triangles
	glDrawArrays(GL_TRIANGLES, 0, 6);

	glDisable(GL_BLEND);
}


void img2d::releaseBuffer()
{
	glDeleteVertexArrays(1, &vao);
	glDeleteTextures(1, &texture);
	glDeleteBuffers(4, vbo);
}

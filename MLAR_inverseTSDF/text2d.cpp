#include "text2d.h"
#include <cstring>
#include <vector>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

using namespace glm;


text2d::text2d(GLuint program, const char* filename, bool isDDS)
{
	this->program = program;
	loadText(filename, isDDS);
	vertexSize = 0;
}


text2d::~text2d()
{
	releaseBuffer();
}


void text2d::loadText(const char* filename, bool isDDS)
{
	//Create VAO/VBO
	glGenVertexArrays(1, &(this->vao));
	glGenBuffers(4, this->vbo);
	texture = (isDDS) ? loadDDS(filename) : loadBMP(filename);

	isLoaded = (texture > 0) ? true : false;
}


void text2d::print(const char* text, int x, int y, int size)
{
	unsigned int length = strlen(text);

	//Fill buffers
	std::vector<vec2> vertex;
	std::vector<vec2> uv;

	for(unsigned int i = 0; i < length; i++){
		//Calculate vertex
		vec2 vertex_up_left    = vec2(x + i*size		, y + size);
		vec2 vertex_up_right   = vec2(x + i*size + size , y + size);
		vec2 vertex_down_right = vec2(x + i*size + size , y);
		vec2 vertex_down_left  = vec2(x + i*size		, y);

		vertex.push_back(vertex_up_left);
		vertex.push_back(vertex_down_left);
		vertex.push_back(vertex_up_right);

		vertex.push_back(vertex_down_right);
		vertex.push_back(vertex_up_right);
		vertex.push_back(vertex_down_left);

		//Calculate uv
		char character = text[i];
		float uv_x = (character%16) / 16.f;
		float uv_y = (character/16) / 16.f;

		vec2 uv_up_left 	= vec2(uv_x, uv_y);
		vec2 uv_up_right 	= vec2(uv_x + 1.f/16.f, uv_y);
		vec2 uv_down_right 	= vec2(uv_x + 1.f/16.f, uv_y + 1.f/16.f);
		vec2 uv_down_left 	= vec2(uv_x, uv_y + 1.f/16.f);

		uv.push_back(uv_up_left);
		uv.push_back(uv_down_left);
		uv.push_back(uv_up_right);

		uv.push_back(uv_down_right);
		uv.push_back(uv_up_right);
		uv.push_back(uv_down_left);
	}

	//Upload buffer data
	glBindVertexArray(this->vao);

	uploadPos(vertex, 0);
	uploadUV(uv, 1);

	glBindVertexArray(0);
	vertexSize = vertex.size();
}


bool text2d::is_loaded()
{
	return isLoaded;
}


void text2d::render()
{
	glUseProgram(program);
	glBindVertexArray(vao);
	glBindTexture(GL_TEXTURE_2D, texture);

	//Enable alpha blending
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//Draw triangles
	glDrawArrays(GL_TRIANGLES, 0, vertexSize);

	glDisable(GL_BLEND);
}


void text2d::releaseBuffer()
{
	glDeleteVertexArrays(1, &vao);
	glDeleteTextures(1, &texture);
	glDeleteBuffers(4, vbo);
}

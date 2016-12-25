#ifndef VISIBLEOBJ_H
#define VISIBLEOBJ_H

#include <GL/glew.h>
#include "glm/glm.hpp"
#include <vector>

#define FOURCC_DXT1 0x31545844	//"DXT1" in ASCII
#define FOURCC_DXT3	0x33545844	//"DXT3" in ASCII
#define FOURCC_DXT5 0x35545844	//"DXT5" in ASCII

using namespace glm;

class visibleObj
{	
protected:
	GLuint program;
	GLuint vao;
	GLuint vbo[4];
	GLuint texture;
	unsigned int indexCount;
	
public:	
	static GLuint loadBMP(const char* filename);
	static GLuint loadDDS(const char* filename);

	void uploadPos(const std::vector<vec3> &vertex, const unsigned int attribNum);
	void uploadPos(const std::vector<vec2> &vertex, const unsigned int attribNum);
	void uploadUV(const std::vector<vec2> &uv, const unsigned int attribNum);
	void uploadNormal(const std::vector<vec3> &normal, const unsigned int attribNum);
	void uploadVBOIndex(const std::vector<unsigned int> &index);

	void setProgram(GLuint program);
	GLuint getProgram();
	
	virtual void render() = 0;
	virtual void releaseBuffer() = 0;
};

#endif

#ifndef VISIBLEOBJ_H
#define VISIBLEOBJ_H

#include <GL/glew.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
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

	vec3 color;
	mat4 trans;
	mat4 rot;
	mat4 scal;

public:
	static GLuint loadBMP(const char* filename);
	static GLuint loadDDS(const char* filename);

	visibleObj();
	virtual ~visibleObj();

	void uploadPos(const std::vector<vec3> &vertex, const unsigned int attribNum);
	void uploadPos(const std::vector<vec2> &vertex, const unsigned int attribNum);
	void uploadUV(const std::vector<vec2> &uv, const unsigned int attribNum);
	void uploadNormal(const std::vector<vec3> &normal, const unsigned int attribNum);
	void uploadVBOIndex(const std::vector<unsigned int> &index);

	bool setUniformMat4Model(const char* name);
	bool setUniformVec3Color(const char* name);
	void setColor(vec3 color);
	mat4 getModel();

	void setProgram(GLuint program);
	GLuint getProgram();

	void setTrans(mat4 trans);
	void setTrans(vec3 pos);
	void setTrans(float x, float y, float z);
	mat4 getTrans();
	vec3 getTransXYZ();

	void setRot(mat4 rot);
	mat4 getRot();

	void setScal(mat4 scal);
	void setScal(vec3 scal);
	void setScal(float x, float y, float z);
	mat4 getScal();
	vec3 getScalXYZ();

	virtual void render() = 0;
	virtual void releaseBuffer() = 0;
};

#endif

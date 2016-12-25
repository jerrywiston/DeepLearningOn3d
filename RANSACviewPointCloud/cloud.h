#ifndef CLOUD_H
#define CLOUD_H

#include "visibleObj.h"
#include <vector>

using namespace glm;

class cloud : public visibleObj
{
private:
	mat4 model;
	unsigned int pointNum;

public:
	cloud(GLuint program);
	cloud(GLuint program, std::vector<vec3> &points);
	virtual ~cloud();
	
	void uploadPoints(std::vector<vec3> &points);
	bool setUniformMat4Model(const char* name);
	
	void setModel(mat4 model);
	mat4 getModel();
	
	void setPointNum(unsigned int num);
	unsigned int getPointNum();
	
	virtual void render();
	virtual void releaseBuffer();
};

#endif
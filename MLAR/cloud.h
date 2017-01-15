#ifndef CLOUD_H
#define CLOUD_H

#include "visibleObj.h"
#include <vector>

using namespace glm;

class cloud : public visibleObj
{
private:
	unsigned int pointNum;

public:
	cloud(GLuint program);
	cloud(GLuint program, std::vector<vec3> &points);
	virtual ~cloud();

	void uploadPoints(std::vector<vec3> &points);
	void setPointNum(unsigned int num);
	unsigned int getPointNum();

	virtual void render();
	virtual void releaseBuffer();
};

#endif

#ifndef SKELETON3D_H
#define SKELETON3D_H

#include "visibleObj.h"

using namespace glm;


class skeleton3d : public visibleObj
{
private:
	unsigned int vertexNum;

public:
	skeleton3d(GLuint program);
	skeleton3d(GLuint program, std::vector<vec3> &vertex);
	virtual ~skeleton3d();

	void setVertex(std::vector<vec3> &vertex);

	virtual void render();
	virtual void releaseBuffer();
};

#endif

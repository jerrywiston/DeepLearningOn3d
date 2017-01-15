#ifndef OBJ3D_H
#define OBJ3D_H

#include "visibleObj.h"
#include <cstring>
#include <map>

using namespace glm;

struct packedVertex{
	vec3 pos;
	vec2 uv;
	vec3 normal;
	bool operator < (const packedVertex that) const{
		return memcmp((void*)this, (void*)&that, sizeof(packedVertex)) > 0;
	};
};

class obj3d : public visibleObj
{
protected:
	bool isLoaded;

public:
	obj3d();
	obj3d(GLuint program, const char* filename_obj, const char* filename_texture, const bool isDDS);
	virtual ~obj3d();

	bool loadObj(const char* filename_obj, const char* filename_texture, const bool isDDS);
	bool is_near(float v1, float v2);
	bool getSimilarVertexIndex(packedVertex &packed, std::map<packedVertex, unsigned int> &vertexToOutIndex, unsigned int &result);
	void indexVBO(
		std::vector<vec3> &in_vertex,
		std::vector<vec2> &in_uv,
		std::vector<vec3> &in_normal,

		std::vector<unsigned int> &out_index,
		std::vector<vec3> &out_vertex,
		std::vector<vec2> &out_uv,
		std::vector<vec3> &out_normal
	);

	bool is_loaded();

	virtual void render();
	virtual void releaseBuffer();
};

#endif

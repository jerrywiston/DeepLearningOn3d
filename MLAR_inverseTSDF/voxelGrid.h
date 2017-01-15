#ifndef VOXEL_GRID_H
#define VOXEL_GRID_H

#include <vector>
#include <iostream>
#include "config.hpp"


//Vector 3x1
typedef struct{
	float x;
	float y;
	float z;
}vec;

//Matrix 3x3
typedef struct{
	float x1;
	float x2;
	float x3;
	float y1;
	float y2;
	float y3;
	float z1;
	float z2;
	float z3;
}mat;

//voxel type
typedef struct{
	vec pos;
	float tsdf;
	unsigned short weight;
	bool isActive;
}voxel;


class voxelGrid
{
private:
	//Host data
	voxel *vox;
	vec *pc;
	vec center;
	vec dir;
	vec trans;
	vec bound1, bound2;
	vec offset;
	mat rot;
	unsigned int pc_count;

	//GPU data pointer
	voxel *d_vox;
	vec *d_pc;
	vec *d_vertexMap;
	vec *d_vertexMap_fused;
	vec *d_center;
	vec *d_trans;
	vec *d_bound1, *d_bound2;
	mat *d_rot;
	vec *d_offset;
	unsigned int *d_pc_count;

public:
	voxelGrid();
	virtual ~voxelGrid();

	void fusion(vec *vertexMap);
	void rayCast(vec *vertexMap_fused);
	void rayCastAll(std::vector<vec> &pcData);
	void rayCastAll_approx(std::vector<vec> &pcData);

	void setTrans(float x, float y, float z);
	void setTrans(vec t);
	void setRot(float x1, float x2, float x3,
				float y1, float y2, float y3,
				float z1, float z2, float z3);
	void setRot(mat r);
	void computeOffset();
	vec getOffsetTrans();

	void inverseTSDF(std::vector<vec> &pc);
	static vec depthToWorld(int u, int v, int z);
};

#endif

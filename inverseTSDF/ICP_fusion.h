#ifndef _ICP_FUSION_HPP
#define _ICP_FUSION_HPP

#include <iostream>
#include <math.h>
#include <vector>
#include "Eigen/Eigen"
#include "voxelGrid.h"

using namespace Eigen;
using namespace std;

class ICP_fusion{
public:
	ICP_fusion(int a_samp, int m_samp, int w_size, int iter);
	virtual ~ICP_fusion();

	void init_fusion(MatrixXf &pc_origin);
	void init_cond(Matrix3f R, Vector3f T);

	float error_count(MatrixXf &pc_now, vec vertexMap[], float dist);
	float get_err_dist();
	int update(MatrixXf &pc_now, MatrixXf &normal_now);

	Matrix3f get_camera_R();
	Vector3f get_camera_T();
	MatrixXf get_point_cloud_rec();
	vec getVoxOffset();
	std::vector<vec> getPointCloud();
	voxel* getVox();
	void savePointCloud(const char* fileName);
	void inverseTSDF(std::vector<vec> &pc);

private:
	Matrix3f camera_R;
	Vector3f camera_T;
	MatrixXf point_cloud_rec;
	MatrixXf pc_now_rec;
	MatrixXf pc;

	//model
	VectorXf UpdateIndex;

	bool collapse;
	int loss_count;
	MatrixXf pc_start;

	//icp parameter
	int align_samp;
	int model_samp;
	int window_size;
	int iterate;

	float err_dist;

	//volumetric data
	voxelGrid *vg;
	vec *vertexMap;
	vec *vertexMap_fused;
	//vec vertexMap[IMG_HEIGHT*IMG_WIDTH];
	//vec vertexMap_fused[IMG_HEIGHT*IMG_WIDTH];
};

#endif //_ICP_FUSION_HPP

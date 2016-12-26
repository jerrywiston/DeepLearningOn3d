#include <iostream>
#include <fstream>
#include <math.h>
#include <omp.h>

#include "Eigen/Eigen"
#include "ICP_fusion.h"
#include "ICP.h"
#include "ICP_kdtree.hpp"

using namespace Eigen;

/*================================
Constructor
================================*/
ICP_fusion::ICP_fusion(int a_samp, int m_samp, int w_size, int iter){
	//Initial pose
	camera_R << 1, 0, 0,
				0, 1, 0,
				0, 0, 1;

	camera_T << MAX_LEN / 2.0f, MAX_LEN / 2.0f, MAX_LEN / 4.0f;

	//Set parameters
	align_samp = a_samp;
	model_samp = m_samp;
	window_size = w_size;
	iterate = iter;
	collapse = false;
	loss_count = 0;

	//Allocate memory
	vg = new voxelGrid();
	vertexMap = new vec[IMG_HEIGHT*IMG_WIDTH];
	vertexMap_fused = new vec[IMG_HEIGHT*IMG_WIDTH];
}


/*================================
Destructor
================================*/
ICP_fusion::~ICP_fusion()
{
	delete vg;
	//delete [] vertexMap;
	delete [] vertexMap_fused;
}


/*================================
Set initial condition
================================*/
void ICP_fusion::init_cond(Matrix3f R, Vector3f T){
	camera_R = R;
	camera_T = T;
}


/*================================
Initial fusion
================================*/
void ICP_fusion::init_fusion(MatrixXf &pc_origin){
	point_cloud_rec = pc_origin;

	//Fusion
	vertexMap = (vec *)pc_origin.data();
	pc_start = pc_origin;

	vg->setTrans(camera_T(0), camera_T(1), camera_T(2));
	vg->setRot(camera_R(0, 0), camera_R(0, 1), camera_R(0, 2),
			   camera_R(1, 0), camera_R(1, 1), camera_R(1, 2),
			   camera_R(2, 0), camera_R(2, 1), camera_R(2, 2));
	vg->computeOffset();
	vg->fusion(vertexMap);

	std::cout << "Initialization Success !!" << std::endl;
}

/*================================
Update TSDF & camera pose
================================*/
int ICP_fusion::update(MatrixXf &pc_now, MatrixXf &normal_now)
{
	int status = 0;
	if(collapse == false){
		/****************************
		//Normal Phase
		****************************/
		//Perform the ICP algorithm
		icp::InitData(point_cloud_rec, pc_now, normal_now);
		if (icp::PointToPlaneAlign(align_samp, window_size, iterate) < 0){
			status = -1;
			return status;
		}

		//Update camera pose
		camera_R = camera_R * icp::Rtot;
		camera_T = camera_R * icp::Ttot + camera_T;
		vg->setTrans(camera_T(0), camera_T(1), camera_T(2));
		vg->setRot(camera_R(0, 0), camera_R(0, 1), camera_R(0, 2),
					camera_R(1, 0), camera_R(1, 1), camera_R(1, 2),
					camera_R(2, 0), camera_R(2, 1), camera_R(2, 2));

		//Error distance
		vg->rayCast(vertexMap_fused);
		err_dist = error_count(pc_now, vertexMap_fused, 3000);

		//Ray-casting & Fusion
		if(err_dist < 10000){
			loss_count = 0;
			vertexMap = (vec *)pc_now.data();
			vg->computeOffset();
			vg->fusion(vertexMap);
			vg->rayCast(vertexMap_fused);
			point_cloud_rec = Map<MatrixXf>((float *)vertexMap_fused, 3, IMG_WIDTH*IMG_HEIGHT);
		}
		else{
			//error correction
			status = 1;
			//cout << "<RayCast Fail> Use frame to frame ! " << err_dist << endl;
			++loss_count;
			icp::InitData(pc_now_rec, pc_now, normal_now);
			if (icp::PointToPlaneAlign(align_samp, window_size, iterate) < 0){
				status = -1;
				return status;
			}
			camera_R = camera_R * icp::Rtot;
			camera_T = camera_R * icp::Ttot + camera_T;
			point_cloud_rec = pc_now;

			if (loss_count > 30){
 				camera_R = Matrix3f::Identity();
				camera_T << MAX_LEN / 2.0f, MAX_LEN / 2.0f, MAX_LEN / 4.0f;
				collapse = true;
			}
		}

		pc_now_rec = pc_now;
	}
	else{
		/****************************
		//Collapse Phase
		****************************/
		status = 2;
		icp_kdtree::InitData(pc_start, pc_now, normal_now);
		int pair = icp_kdtree::PointToPlaneAlign(8, 200, 30); //4,200,45

		MatrixXf camera_R_temp = camera_R * icp_kdtree::Rtot;
		MatrixXf camera_T_temp = camera_R * icp_kdtree::Ttot + camera_T;

		vg->setTrans(camera_T_temp(0), camera_T_temp(1), camera_T_temp(2));
		vg->setRot(camera_R_temp(0, 0), camera_R_temp(0, 1), camera_R_temp(0, 2),
			camera_R_temp(1, 0), camera_R_temp(1, 1), camera_R_temp(1, 2),
			camera_R_temp(2, 0), camera_R_temp(2, 1), camera_R_temp(2, 2));

		vg->rayCast(vertexMap_fused);

		err_dist = error_count(pc_now, vertexMap_fused, 3000);
		//cout << "<Collapse> Use kd-tree ICP ! " << err_dist << endl;

		if (pair != -1 && err_dist < 7000){
			//cout << "Back Success !!" << endl;
			point_cloud_rec = Map<MatrixXf>((float *)vertexMap_fused, 3, IMG_WIDTH*IMG_HEIGHT);
			icp::InitData(point_cloud_rec, pc_now, normal_now);
			icp::PointToPlaneAlign(align_samp, window_size, 10);
			camera_R = camera_R_temp * icp::Rtot;
			camera_T = camera_R_temp * icp::Ttot + camera_T_temp;

			loss_count = 0;
			collapse = false;
		}
	}
	return status;
}

float ICP_fusion::error_count(MatrixXf &pc_now, vec *vertexMap, float dist){
	int count = 0;
	float err_dist = 0;

	#pragma omp parallel for
	for (int i = 0; i < pc_now.cols(); ++i){
		if (pc_now(2, i) != 0 && vertexMap[i].z != 0 && pc_now(2, i) < dist && vertexMap[i].z < dist){
			err_dist += pow(pc_now(2, i) - vertexMap[i].z, 2);
			++count;
		}
	}
	err_dist /= count;
	return err_dist;
}

float ICP_fusion::get_err_dist(){
	return err_dist;
}

/*================================
Access properties
================================*/
Matrix3f ICP_fusion::get_camera_R(){
	return camera_R.transpose();
}


Vector3f ICP_fusion::get_camera_T(){
	return camera_T;
}


MatrixXf ICP_fusion::get_point_cloud_rec(){
	return point_cloud_rec;
}


std::vector<vec> ICP_fusion::getPointCloud()
{
	std::vector<vec> pc;
	//vg->rayCastAll_approx(pc);
	vg->rayCastAll(pc);

	return pc;
}

vec ICP_fusion::getVoxOffset()
{
	return vg->getOffsetTrans();
}


/*================================
Save point cloud set
================================*/
void ICP_fusion::savePointCloud(const char* fileName)
{
	std::vector<vec> pc;
	vg->rayCastAll(pc);

	std::fstream file;
	file.open(fileName, std::ios::out);

	if (file){
		for (unsigned int i = 0; i < pc.size(); ++i)
			file << (pc[i].x - MAX_LEN / 2.0f) / MAX_LEN << "\t" << (pc[i].y - MAX_LEN / 1.75f) / MAX_LEN << "\t" << (pc[i].z - 200) / MAX_LEN << std::endl;

		file.close();
	}
}

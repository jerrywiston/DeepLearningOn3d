#include "kdtree.h"
#include "Eigen/Eigen"
#include <iostream>
using namespace std;
using namespace Eigen;


void Eigen_kdtree_build(kdtree* &kd, MatrixXf point_set){
	int i,j;
	float x,y,z;
	for(i=0; i<point_set.cols(); i++){
		x = point_set(0,i);
		y = point_set(1,i);
		z = point_set(2,i);
		kd_insert3f(kd, x, y, z, 0);
	}
}

Vector3f Eigen_kdnns(kdtree* &kd, Vector3f point){
	struct kdres *presult;
	float pos[3];

	presult = kd_nearest3f(kd, point(0), point(1), point(2));

	kd_res_itemf( presult, pos );
	Vector3f ans;
	ans << pos[0], pos[1], pos[2];
	kd_res_free( presult );
	return ans;
}

Vector3f Eigen_kdnns_range(kdtree* &kd, Vector3f point, float dist){
	struct kdres *presult;
	float pos[3];
	presult = kd_nearest_range3f(kd, point(0), point(1), point(2), dist);
	kd_res_itemf( presult, pos );
	
	Vector3f ans;
	ans << pos[0], pos[1], pos[2];
	kd_res_free( presult );
	return ans;
}
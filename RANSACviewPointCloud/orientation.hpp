#include <iostream>
#include <cmath>
#include <vector>
#include "Eigen/Eigen"
#include "ransac.hpp"

using namespace std;
using namespace Eigen;

Matrix3f AxisAngle2Matrix(float CosTheta, Vector3f u)
{
    float SinTheta = sqrt(1 - CosTheta*CosTheta);
    Matrix3f rot;
    rot(0,0) = CosTheta + u(0)*u(0)*(1-CosTheta);
    rot(0,1) = u(0)*u(1)*(1-CosTheta) - u(2)*SinTheta;
    rot(0,2) = u(0)*u(2)*(1-CosTheta) + u(1)*SinTheta;

    rot(1,0) = u(1)*u(0)*(1-CosTheta) + u(2)*SinTheta;
    rot(1,1) = CosTheta + u(1)*u(1)*(1-CosTheta);
    rot(1,2) = u(1)*u(2)*(1-CosTheta) - u(0)*SinTheta;

    rot(2,0) = u(2)*u(0)*(1-CosTheta) - u(1)*SinTheta;
    rot(2,1) = u(2)*u(1)*(1-CosTheta) + u(0)*SinTheta;
    rot(2,2) = CosTheta + u(2)*u(2)*(1-CosTheta);

    return rot;
}

MatrixXf OrientationWall(int arg_plane, vector <MatrixXf> &PointCloudG){
    Vector3f plane_nm[arg_plane];
	for(int i=0; i<arg_plane; ++i){
		plane_nm[i] = RANSAC::plane_normal(PointCloudG[i], 500);
		//cout << "( " << plane_nm[i].transpose() << " )" << endl;
	}

    Vector3f nm_gravity;
    nm_gravity << 0,1,0;
    Vector3f nm_wall;
    nm_wall << 0,0,-1;
    Vector3f nm_temp;
    float MinDist = 2;
    float id;

    for(int i=0; i<arg_plane; ++i){
        float angle = abs(plane_nm[i].dot(nm_gravity));
        float score = angle / PointCloudG[i].cols();
        if(score < MinDist){
            MinDist = score;
            nm_temp = plane_nm[i];
            id = i;
        }
    }
    cout << "<Wall : " << id << ">" << endl;

    float CosTheta = nm_temp.dot(nm_wall);
    Vector3f u = nm_temp.cross(nm_wall).normalized();
    if(CosTheta < 0){
        CosTheta = -CosTheta;
        u = -u;
    }
    Matrix3f rot = AxisAngle2Matrix(CosTheta, u);

    return rot;
}

MatrixXf OrientationFloor(int arg_plane, vector<MatrixXf> &PointCloudG){
    Vector3f plane_nm[arg_plane];
	for(int i=0; i<arg_plane; ++i){
		plane_nm[i] = RANSAC::plane_normal(PointCloudG[i], 500);
		//cout << "( " << plane_nm[i].transpose() << " )" << endl;
	}

    Vector3f nm_gravity;
    nm_gravity << 0,1,0;
    Vector3f nm_temp;
    float MaxDist = 2;
    float id;

    for(int i=0; i<arg_plane; ++i){
        float angle = abs(plane_nm[i].dot(nm_gravity));
        float score = angle * PointCloudG[i].cols();
        if(score > MaxDist){
            MaxDist = score;
            nm_temp = plane_nm[i];
            id = i;
        }
    }
    cout << "<Floor : " << id << ">" << endl;

    float CosTheta = nm_temp.dot(nm_gravity);
    Vector3f u = nm_temp.cross(nm_gravity).normalized();
    if(CosTheta < 0){
        CosTheta = -CosTheta;
        u = -u;
    }
    Matrix3f rot = AxisAngle2Matrix(CosTheta, u);

    return rot;
}

void OrientationCorrect( int arg_random, int arg_plane, int arg_dist,
                        MatrixXf &PointCloud, vector<MatrixXf> &PointCloudG){

    PointCloudG.resize(arg_plane + 1);
	//MatrixXf PointCloudG[arg_plane + 1];

	int count = RANSAC::plane_group(arg_plane, arg_random, arg_dist, PointCloud, PointCloudG);
	cout << "Total : " << PointCloud.cols() << endl;
	//rotation
	MatrixXf rotf = OrientationFloor(arg_plane, PointCloudG);
	for(int i=0; i<arg_plane+1; ++i)
		PointCloudG[i] = rotf * PointCloudG[i];
	MatrixXf rotw = OrientationWall(arg_plane, PointCloudG);
	for(int i=0; i<arg_plane+1; ++i)
		PointCloudG[i] = rotw * PointCloudG[i];


	//translation to origin
	Vector3f PointCloudAve = Vector3f::Zero();
	for(int i=0; i<arg_plane+1; ++i){
		PointCloudAve(0) += PointCloudG[i].row(0).sum();
		PointCloudAve(1) += PointCloudG[i].row(1).sum();
		PointCloudAve(2) += PointCloudG[i].row(2).sum();
	}

	PointCloudAve /= PointCloud.cols();
	for(int i=0; i<arg_plane+1; ++i)
		for(int j=0; j<PointCloudG[i].cols(); ++j)
			PointCloudG[i].col(j) -= PointCloudAve;
}

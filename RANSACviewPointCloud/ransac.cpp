#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <vector>
#include <ctime>
#include "Eigen/Eigen"
#include "ransac.hpp"

using namespace Eigen;
using namespace std;

typedef struct{
	Vector3f point;
	Vector3f normal;
	vector <int> id_set;
}PlaneGroup;

MatrixXf arr_to_mat(vector <Vector3f> v_pc){
	int size = v_pc.size();
	MatrixXf pc_mat(3,size);
	int i;
	for(i=0; i<size; i++)
		pc_mat.col(i) = v_pc[i];
	return pc_mat;
}

MatrixXf RANSAC::read_pc(const char *fn){
	fstream file;
	file.open(fn,ios::in);
	Vector3f mid;
	vector <Vector3f> pc_arr;

	float x,y,z;
	while(file){
		file >> x >> y >> z;
		mid << x, y, z;
		pc_arr.push_back(mid);
	}
	file.close();
	return arr_to_mat(pc_arr);
}

float point_to_plane(Vector3f p0, Vector3f p, Vector3f n){
	Vector3f v = p0 - p;
	float dist = abs(v.dot(n));
	return dist;
}

Vector3f find_normal(Vector3f p1, Vector3f p2, Vector3f p3){
	Vector3f v1;
	Vector3f v2;
	Vector3f n1;
	v1 = p2-p1;
	v2 = p3-p1;
	n1 = v1.cross(v2);
	n1 = n1/n1.norm();

	return n1;
}

bool same_plane(PlaneGroup plane1, PlaneGroup plane2, float arg_dist, float arg_normal){
	float dist = abs((plane1.point - plane2.point).dot(plane1.normal));
	float angle = abs((plane1.normal).dot(plane2.normal));
	if(dist < arg_dist && angle > arg_normal)
		return true;
	else
		return false;
}

int compare(const void *a, const void *b){	//qsort
	int c = *(int *)a;
	int d = *(int *)b;
	if(c > d)
		return -1;
	else if(c == d)
		return 0;
	else return 1;
}

int RANSAC::find_plane(MatrixXf PointCloud, MatrixXf NormalMap, int total, float arg_dist,
MatrixXf &PointCloudGroup, MatrixXf &NormalMapGroup, MatrixXf &PointCloudOther, MatrixXf &NormalMapOther){

	//Check input
	if(PointCloud.cols() != NormalMap.cols()){
		cout << "Wrong PointCloud & NormapMap size !!" << endl;
		return -1;
	}

	//Random to get plane
	srand(time(0));
	PlaneGroup group[total];
	int ra, rb, rc;
	bool record;
	if(PointCloud.cols() == 0)
		return -1;
	for(int i=0; i<total; i++){
		//get random number
		ra = rand() % PointCloud.cols();
		do{
			rb = rand() % PointCloud.cols();
		}while(rb == ra);
		do{
			rc = rand() % PointCloud.cols();
		}while(rc == ra || rc == rb);

		//calculate plane
		PlaneGroup insert;
		insert.point = PointCloud.col(ra);
		insert.normal = find_normal(PointCloud.col(ra), PointCloud.col(rb), PointCloud.col(rc));
		group[i] = insert;
	}

	//scan all point
	float dist;
	int i,j;
	for(int i=0; i<PointCloud.cols(); ++i){
		for(int j=0; j<total; ++j){
			dist = point_to_plane(PointCloud.col(i), group[j].point, group[j].normal);
			if(dist < arg_dist)
				group[j].id_set.push_back(i);
		}
	}

	//find best plane
	int best_id;
	int best_record = 0;
	for(int i=0; i<total; ++i){
		if(group[i].id_set.size() > best_record){
			best_id = i;
			best_record = group[i].id_set.size();
		}
	}

	//Construct plane group vector
	int cut_total = group[best_id].id_set.size();
	PointCloudGroup.resize(3, cut_total);
	NormalMapGroup.resize(3, cut_total);
	PointCloudOther.resize(3, PointCloud.cols() - cut_total);
	NormalMapOther.resize(3, PointCloud.cols() - cut_total);
	int point_arr[PointCloud.cols()];
	memset(point_arr, 0, PointCloud.cols()*sizeof(int) );

	for(int i=0; i<cut_total; ++i){
		PointCloudGroup.col(i) = PointCloud.col(group[best_id].id_set[i]);
		NormalMapGroup.col(i) = NormalMap.col(group[best_id].id_set[i]);
		point_arr[group[best_id].id_set[i]] = 1;
	}

	//Construct other PointCloud
	int count = 0;
	for(int i=0; i<PointCloud.cols(); ++i){
		if(point_arr[i] == 0){
			PointCloudOther.col(count) = PointCloud.col(i);
			++count;
		}
	}

	return cut_total;
}

void random_swap(MatrixXf &PointCloud, MatrixXf &NormalMap){
	int total = PointCloud.cols();
	Vector3f midp;
	Vector3f midn;
	for(int i=0; i<total/2; ++i){
		int ra = rand() * 3 % total;
		int rb = rand() * 3 % total;
		midp = PointCloud.col(ra);
		midn = NormalMap.col(ra);
		PointCloud.col(ra) = PointCloud.col(rb);
		PointCloud.col(rb) = midp;
		NormalMap.col(ra) = NormalMap.col(rb);
		NormalMap.col(rb) = midn;
	}
}

int RANSAC::plane_group(int arg_plane, int arg_random, float arg_dist, MatrixXf PointCloud, MatrixXf NormalMap, MatrixXf PointCloudG_mid[], MatrixXf NormalMapG_mid[]){
	MatrixXf PointCloud_mid = PointCloud;
	MatrixXf NormalMap_mid = NormalMap;

	MatrixXf PointCloudGroup;
	MatrixXf NormalMapGroup;
	MatrixXf PointCloudOther;
	MatrixXf NormalMapOther;

	int count = 0;

	for(int i=0; i<arg_plane; ++i){
		if(RANSAC::find_plane(PointCloud_mid, NormalMap_mid, arg_random, arg_dist, PointCloudGroup, NormalMapGroup, PointCloudOther, NormalMapOther) == 0)
			break;

		PointCloud_mid = PointCloudOther;
		NormalMap_mid = NormalMapOther;

		cout << "Group " << i << " total : " << PointCloudGroup.cols() << endl;
		count = count + PointCloudGroup.cols();

		PointCloudG_mid[i] = PointCloudGroup;
		NormalMapG_mid[i] = NormalMapGroup;
	}

	PointCloudG_mid[arg_plane] = PointCloudOther;
	NormalMapG_mid[arg_plane] = NormalMapOther;

	return count;
}

int RANSAC::plane_group(int arg_plane, int arg_random, float arg_dist, MatrixXf PointCloud, MatrixXf PointCloudG_mid[]){
	MatrixXf nm_empty = MatrixXf::Zero(PointCloud.rows(), PointCloud.cols());
	MatrixXf nm_emptyG_mid[arg_plane + 1];
	return RANSAC::plane_group(arg_plane, arg_random, arg_dist, PointCloud, nm_empty, PointCloudG_mid, nm_emptyG_mid);
}

void down_sample(int size, MatrixXf PointCloudGroup[], MatrixXf NormalMapGroup[], MatrixXf &PointCloudDown, MatrixXf &NormalMapDown, int down){
	srand(time(0));
	int total = 0;
	for(int i=0; i<size; i++)
		total += (PointCloudGroup[i].cols()-1)/down;

	PointCloudDown.resize(3,total);
	NormalMapDown.resize(3,total);
	int count = 0;
	cout << size << " " << total << endl;
	for(int i=0; i<size; i++){
		for(int j=0; j<(PointCloudGroup[i].cols()-1)/down; j++){
			random_swap(PointCloudGroup[i], NormalMapGroup[i]);
			PointCloudDown.col(count) = PointCloudGroup[i].col(j*down);
			NormalMapDown.col(count) = NormalMapGroup[i].col(j*down);
			++count;
		}
	}

}

Vector3f RANSAC::plane_normal(MatrixXf PointCloud, int num){
	srand(time(0));
	int size = PointCloud.cols();
	Vector3f ave_nm = Vector3f::Zero();
	int ra, rb, rc;
	for(int i=0; i<num; i++){
		//get random number
		ra = rand() % PointCloud.cols();
		do{
			rb = rand() % PointCloud.cols();
		}while(rb == ra);
		do{
			rc = rand() % PointCloud.cols();
		}while(rc == ra || rc == rb);

		//calculate plane
		Vector3f nm = find_normal(PointCloud.col(ra), PointCloud.col(rb), PointCloud.col(rc));
		//cout << nm.transpose() << endl;
		if(i > 0){
			if(nm.dot(ave_nm) < 0)
				nm = -nm;
		}
		ave_nm = ave_nm + nm;
	}

	return ave_nm.normalized();
}

/*
int main(int argc, char *argv[]){
	//50 20 20
	int arg_random = atoi(argv[1]);
	float arg_dist = atof(argv[2]);
	float arg_down = atoi(argv[3]);

	int arg_plane = 6;

	MatrixXf PointCloud = read_pc("PointCloud_1.txt");
	MatrixXf NormalMap = read_pc("NormalMap_1.txt");



	cout << "Total point : " << PointCloud.cols() << endl;
	cout << "Start plane group ... " << endl;

	float start = clock();
	MatrixXf PointCloudG[arg_plane + 1];
	MatrixXf NormalMapG[arg_plane + 1];
	int count = plane_group(arg_plane, arg_random, arg_dist, PointCloud , NormalMap, PointCloudG, NormalMapG);
	float end = clock();
	cout << (end - start)/1000 << " sec" << endl;
	cout << (float)count/(float)PointCloud.cols()*100 << "%" << endl;

	//print PointCloudGroup & NormalMapGroup
	cout << "Save PlaneGroup ..." << endl;

	char str[256];
	fstream file;

	for(int i=0; i<arg_plane; ++i){
		sprintf(str, "Group/PointCloudGroup_%d.npts", i);
		file.open(str, ios::out);
		file << PointCloudG[i].transpose();
		file.close();

		sprintf(str, "Group/NormalMapGroup_%d.npts", i);
		file.open(str, ios::out);
		file << NormalMapG[i].transpose();
		file.close();
	}

	file.open("Group/PointCloudOther.npts", ios::out);
	file << PointCloudG[arg_plane].transpose();
	file.close();

	file.open("Group/NormalMapOther.npts", ios::out);
	file << NormalMapG[arg_plane].transpose();
	file.close();
	//

	MatrixXf PointCloudDown;
	MatrixXf NormalMapDown;
	down_sample(arg_plane+1, PointCloudG, NormalMapG, PointCloudDown, NormalMapDown, arg_down);

	file.open("Group/PointCloudDown.npts", ios::out);
	file << PointCloudDown.transpose();
	file.close();
	file.open("Group/NormalMapDown.npts", ios::out);
	file << NormalMapDown.transpose();
	file.close();

	return 0;
}
*/

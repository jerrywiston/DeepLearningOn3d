#ifndef ICP_KDTREE_HPP
#define ICP_KDTREE_HPP

#include <iostream>
#include <ctime>
#include <cmath>
#include "Eigen/Eigen"
#include <cstdlib>
//#include <omp.h>
#include "Eigen_kdtree.hpp"

using namespace std;
using namespace Eigen;

namespace icp_kdtree{
	//Original Pointcloud Data
	MatrixXf pc_origin;
	MatrixXf pc_match;
	MatrixXf normal_match;

	//kdtree
	kdtree *root;

	//Down Sampling Data
	MatrixXf pc_match_down;
	MatrixXf normal_match_down;
	MatrixXf pc_origin_down;

	//Point Match Data
	MatrixXf Xc;
	MatrixXf Pc;
	MatrixXf Pn;

	//Rigid Transform
	Matrix3f Rtot;
	Vector3f Ttot;

	//Build kdtree from pc_origin_down
	void BuildKdtree(){
		root = kd_create(3);
		Eigen_kdtree_build(root, pc_origin_down);
	}

	void InitData(MatrixXf &s_pc_origin, MatrixXf &s_pc_match, MatrixXf &s_normal_match){
		Rtot << 1, 0, 0,
				0, 1, 0,
				0, 0, 1;

		Ttot << 0, 0, 0;

		pc_origin = s_pc_origin;
		pc_match = s_pc_match;
		normal_match = s_normal_match;
	}

	//Set initial condition
	void InitCond(Matrix3f R, Vector3f T){
		Rtot = R;
		Ttot = T;
	}

	//Uniform sampling
	void UniformSampling(int samp_o, int samp_m){
		int total = pc_match.cols();
		int count_m = 0;
		int count_o = 0;
		MatrixXf pc_match_down_arr(3, total / samp_m);
		MatrixXf normal_match_down_arr(3, total / samp_m);
		MatrixXf pc_origin_down_arr(3, total / samp_o);

		for (int i = 0; i<total / samp_o; i++){
			if (pc_origin(2, samp_o*i) != 0 && pc_origin(2, samp_o*i)<3000){
				pc_origin_down_arr.col(count_o) = pc_origin.col(samp_o * i);
				++count_o;
			}
		}

		for (int i = 0; i<total / samp_m; i++){
			if (pc_match(2, samp_m*i) != 0 && pc_match(2, samp_m*i)<3000){
				pc_match_down_arr.col(count_m) = pc_match.col(samp_m * i);
				normal_match_down_arr.col(count_m) = normal_match.col(samp_m * i);
				++count_m;
			}
		}

		pc_match_down = pc_match_down_arr.block(0, 0, 3, count_m);
		normal_match_down = normal_match_down_arr.block(0, 0, 3, count_m);
		pc_origin_down = pc_origin_down_arr.block(0, 0, 3, count_o);
	}

	//Point Match
	int compare(const void *a, const void *b){
		float c = *(float *)a;
		float d = *(float *)b;
		if(c < d)
			return -1;
		else if(c == d)
			return 0;
		else return 1;
	}

	void PointMatch(MatrixXf &pcm, MatrixXf &nm, float dist){
		const int max_size = pcm.cols();
		MatrixXf Xc_v(3, max_size);
		MatrixXf Pc_v(3, max_size);
		MatrixXf Pn_v(3, max_size);

		Vector3f mid;
		//float dist_arr[max_size][2];
		float *dist_arr = (float *)malloc(max_size * 2 * sizeof(float));
		float **dist_arr_2d = (float **)malloc(max_size * sizeof(float *));
		//float dist_arr_2d[max_size][2];
		for (int i = 0; i < max_size; ++i)
			dist_arr_2d[i] = &(dist_arr[2 * i]);

		#pragma omp parallel for
		for(int i=0; i<max_size; ++i){
			if(dist < 0)
				mid = Eigen_kdnns(root, pcm.col(i));
			else
				mid = Eigen_kdnns_range(root, pcm.col(i), dist);
			Xc_v.col(i) = mid;
			Pc_v.col(i) = pcm.col(i);
			Pn_v.col(i) = nm.col(i);

			dist_arr_2d[i][0] = (mid - pcm.col(i)).norm();
			dist_arr_2d[i][1] = (float)i;
		}

		qsort((void *)dist_arr, max_size, 2*sizeof(float), compare);

		int down = max_size*2/10;
		int up = max_size*8/10;

		Xc.resize(3, up-down);
		Pc.resize(3, up-down);
		Pn.resize(3, up-down);

		int count = 0;
		for(int i=down; i<up; i++){
			Xc.col(count) = Xc_v.col((int)(dist_arr_2d[i][1]));
			Pc.col(count) = Pc_v.col((int)(dist_arr_2d[i][1]));
			Pn.col(count) = Pn_v.col((int)(dist_arr_2d[i][1]));
			count++;
		}
	}

	float PointToPlaneAlign(int samp_o, int samp_m, int iterate){
		if (pc_match.cols() < 10 || pc_origin.cols() < 10){
			cout << "ICP fail !!" << endl;
			return -1;
		}

		Matrix3f R;
		Vector3f T;
		MatrixXf pcm;
		MatrixXf nm;
		UniformSampling(samp_o, samp_m);
		BuildKdtree();

		//iterative to align
		int i, j, k;
		for(i=0; i<iterate; ++i){
			//transform the match point cloud
			pcm = Rtot * pc_match_down;
			for (j = 0; j<pcm.cols(); j++)
				pcm.col(j) += Ttot;
			nm = Rtot * normal_match_down;

			//point match
			PointMatch(pcm, nm, -1);

			//system("pause");
			if (Pc.cols() < 100)
				return -1;

			//calculate A and b
			MatrixXf A(Pc.cols(), 6);
			VectorXf b(Pc.cols());

			//#pragma omp parallel for
			for (k = 0; k<Pc.cols(); k++){
				A(k, 0) = Pn(2, k)*Pc(1, k) - Pn(1, k)*Pc(2, k);
				A(k, 1) = Pn(0, k)*Pc(2, k) - Pn(2, k)*Pc(0, k);
				A(k, 2) = Pn(1, k)*Pc(0, k) - Pn(0, k)*Pc(1, k);
				A(k, 3) = Pn(0, k);
				A(k, 4) = Pn(1, k);
				A(k, 5) = Pn(2, k);

				b(k) = (((Xc.col(k) - Pc.col(k)).transpose()) * Pn.col(k))(0);
			}


			//SVD decomposition
			JacobiSVD<MatrixXf> svd(A, ComputeThinU | ComputeThinV);

			//Sigma matrix
			int svalue_total = svd.singularValues().size();
			MatrixXf sigma = MatrixXf::Zero(svalue_total, svalue_total);
			for (j = 0; j<svalue_total; j++)
				sigma(j, j) = (svd.singularValues())(j);

			//cout << sigma << endl; system("pause");
			VectorXf x = svd.matrixV() * sigma.inverse() * svd.matrixU().transpose() * b;

			//calculate transform matrix
			float alpha = x(0);
			float beta = x(1);
			float gamma = x(2);

			float sin_a = sinf(alpha);
			float sin_b = sinf(beta);
			float sin_g = sinf(gamma);
			float cos_a = cosf(alpha);
			float cos_b = cosf(beta);
			float cos_g = cosf(gamma);

			R(0, 0) = cos_g*cos_b;
			R(0, 1) = -sin_g*cos_a + cos_g*sin_b*sin_a;
			R(0, 2) = sin_g*sin_a + cos_g*sin_b*cos_a;
			R(1, 0) = sin_g * cos_b;
			R(1, 1) = cos_g*cos_a + sin_g*sin_b*sin_a;
			R(1, 2) = -cos_g*sin_a + sin_g*sin_b*cos_a;
			R(2, 0) = -sin_b;
			R(2, 1) = cos_b*sin_a;
			R(2, 2) = cos_b*cos_a;

			T(0) = x(3);
			T(1) = x(4);
			T(2) = x(5);

			//total transform
			Rtot = R * Rtot;
			Ttot = T + R*Ttot;
		}

		kd_free( root );
		return 0;
	}
}

#endif //ICP_KDTREE_HPP

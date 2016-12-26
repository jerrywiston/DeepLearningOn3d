#ifndef ICP_HPP
#define ICP_HPP

#include <iostream>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include "Eigen/Eigen"
#include <omp.h>
#include "config.hpp"

using namespace Eigen;

typedef struct{
	int index;
	float dist;
}form;


namespace icp{
	//Original Pointcloud Data
	MatrixXf pc_origin;
	MatrixXf pc_match;
	MatrixXf normal_match;

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
	MatrixXf pc_update;


	//Initialize pointcloud data (pc_origin, pc_match, normal_match)
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
	void InitCond(Matrix3f &R, Vector3f &T){
		Rtot = R;
		Ttot = T;
	}


	//Uniform sampling
	void UniformSampling(int samp){
		int total = pc_match.cols();
		int count = 0;
		MatrixXf pc_match_down_arr(3, total / samp);
		MatrixXf normal_match_down_arr(3, total / samp);

		for (int i = 0; i < total / samp; i++){
			if (pc_match(2, samp*i) != 0 && pc_match(2, samp*i) < 1500){
				pc_match_down_arr.col(count) = pc_match.col(samp * i);
				normal_match_down_arr.col(count) = normal_match.col(samp * i);
				++count;
			}
		}

		pc_match_down = pc_match_down_arr.block(0, 0, 3, count);
		normal_match_down = normal_match_down_arr.block(0, 0, 3, count);
	}


	//Transform world coordinate to depth coordinate.
	void WorldToDepth(float wx, float wy, float wz, int &cx, int &cy){
		cx = (int)round(wx * FOCAL_LEN / wz + IMG_WIDTH_OVER_TWO);
		cy = (int)round(wy * -FOCAL_LEN / wz + IMG_HEIGHT_OVER_TWO);
	}


	//Search the window to find the nearest point
	int SearchPoint(VectorXf &point, MatrixXf &pco, int wsize, int bound){
		int cx, cy;
		int wx, wy;

		int index_rec = -1;
		int index;
		float dist_rec = 999999;
		float dist;

		WorldToDepth(point(0), point(1), point(2), cx, cy);

		for (int j = -wsize; j <= wsize; j++){
			wy = cy + j;
			if (wy >= IMG_HEIGHT - bound || wy < bound)
				continue;
			for (int i = -wsize; i <= wsize; i++){
				wx = cx + i;
				if (wx >= IMG_WIDTH - bound || wx < bound)
					continue;

				index = INDEX(wx, wy);
				dist = (pco.col(index) - point).norm();
				if (dist < dist_rec){
					dist_rec = dist;
					index_rec = index;
				}
			}
		}

		if (index_rec < 0 || index_rec>IMG_WIDTH*IMG_HEIGHT){
			return -1;
		}
		else if (pco(2, index_rec) == 0){
			return -1;
		}
		else if (dist_rec > 100){
			return -1;
		}
		else
			return index_rec;
	}


	//Use reverse calibration to find point match relation. (Pc, Pn, Xn)
	void PointMatch(MatrixXf &pco, MatrixXf &pcm, MatrixXf &nm, int wsize){
		int total = pcm.cols();
		//MatrixXf pcm_arr(3, total);
		//MatrixXf nm_arr(3, total);
		MatrixXf pco_arr(3, total);

		#pragma omp parallel for
		for (int i = 0; i < total; ++i){
			VectorXf point = pcm.col(i);
			int index = SearchPoint(point, pco, wsize, 0);
			if (index < 0){
				pco_arr.col(i) = Vector3f(0, 0, -1);
			}
			else{
				pco_arr.col(i) = pc_origin.col(index);
			}
		}
		int count = 0;
		for (int i = 0; i < total; ++i){
			if (pco_arr(2, i) > 10)
				++count;
		}

		Pc = MatrixXf(3, count);
		Pn = MatrixXf(3, count);
		Xc = MatrixXf(3, count);

		int count2 = 0;
		for (int i = 0; i < total; ++i){
			if (count2 >= count)
				break;
			if (pco_arr(2, i) < 0)
				continue;
			Pc.col(count2) = pcm.col(i);
			Pn.col(count2) = nm.col(i);
			Xc.col(count2) = pco_arr.col(i);
			++count2;
		}
	}


	//Get non-overlapping point cloud (pc_update)
	void PointUpdate(int samp){
		UniformSampling(samp);

		MatrixXf pc_match_down_t = Rtot*pc_match_down;
		int total = pc_match_down_t.cols();
		for (int i = 0; i < total; ++i)
			pc_match_down_t.col(i) += Ttot;

		int count = 0;
		int index;
		MatrixXf pcu_arr(3, total);

		#pragma omp parallel for
		for (int i = 0; i < total; ++i){
			VectorXf point = pc_match_down_t.col(i);
			index = SearchPoint(point, pc_origin, 0, 5);
			if (index < 0){
				pcu_arr.col(count) = point;
				++count;
			}
		}

		pc_update = pcu_arr.block(0, 0, 3, count);
	}


	void PointUpdateIndex(VectorXf &UpdateIndex){
		UpdateIndex = VectorXf::Zero(pc_match.cols());
		MatrixXf pc_match_t = Rtot*pc_match;
		int total = pc_match_t.cols();
		for (int i = 0; i<total; ++i)
			pc_match_t.col(i) += Ttot;

		int count = 0;
		//int index;
		for (int i = 0; i<total; ++i){
			VectorXf point = pc_match_t.col(i);
			int index = SearchPoint(point, pc_origin, 0, 10);
			if (index < 0)
				UpdateIndex(i) = 1;
		}
	}


	//Point to plane error matric
	float PointToPlaneAlign(int samp, int wsize, int iterate){
		if (pc_match.cols() < 10 || pc_origin.cols() < 10){
			//std::cout << "ICP fail !!" << std::endl;
			return -1;
		}

		float sigma_max, sigma_min;
		MatrixXf sigma;
		Matrix3f R;	//rotation matrix
		Vector3f T;	//translation vector
		MatrixXf pcm;
		MatrixXf nm;
		UniformSampling(samp);

		//iterative to align
		int i, j, k;
		for (i = 0; i<iterate; i++){
			//transform the match point cloud
			pcm = Rtot*pc_match_down;
			nm = Rtot*normal_match_down;
			for (j = 0; j<pcm.cols(); j++)
				pcm.col(j) += Ttot;

			//point match
			PointMatch(pc_origin, pcm, nm, wsize);
			if (Pc.cols() < 10)
				return -1;
			//calculate A and b
			MatrixXf A(Pc.cols(), 6);
			VectorXf b(Pc.cols());

			#pragma omp parallel for
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
			sigma = MatrixXf::Zero(svalue_total, svalue_total);
			for (j = 0; j<svalue_total; j++)
				sigma(j, j) = (svd.singularValues())(j);

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

		sigma_max = 0;
		sigma_min = 99999;
		for (int i = 0; i < sigma.cols(); ++i){
			if (sigma(i, i) > sigma_max)
				sigma_max = sigma(i, i);
			if (sigma(i, i) < sigma_min)
				sigma_min = sigma(i, i);
		}
		return sigma_max/sigma_min;
	}
}

#endif //ICP_HPP

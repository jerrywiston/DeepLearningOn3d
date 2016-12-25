#ifndef _EIGEN_KDTREE_HPP
#define _EIGEN_KDTREE_HPP

#include "kdtree.h"
#include "Eigen/Eigen"

using namespace Eigen;

void Eigen_kdtree_build(kdtree* &kd, MatrixXf point_set);

Vector3f Eigen_kdnns(kdtree* &kd, Vector3f point);

Vector3f Eigen_kdnns_range(kdtree* &kd, Vector3f point, float dist);

#endif // _EIGEN_KDTREE_HPP
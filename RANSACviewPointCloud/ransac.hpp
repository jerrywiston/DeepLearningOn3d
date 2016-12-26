#include "Eigen/Eigen"
#include <iostream>

using namespace std;
using namespace Eigen;

namespace RANSAC{
    //Read point cloud data
    MatrixXf read_pc(const char *fn);

    //Find one major plane
    int find_plane( MatrixXf PointCloud,
                    int total, //(arg_random)
                    float arg_dist,
                    MatrixXf &PointCloudGroup,
                    MatrixXf &PointCloudOther   );

    //Find the major planes
    int plane_group(int arg_plane,
                    int arg_random,
                    float arg_dist,
                    MatrixXf &PointCloud,
                    vector<MatrixXf> &PointCloudG_mid);


    Vector3f plane_normal(MatrixXf &PointCloud, int num);
};

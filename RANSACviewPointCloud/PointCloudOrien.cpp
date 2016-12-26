#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string>
#include <vector>
#include <cstdlib>
#include "Eigen/Eigen"
#include "ransac.hpp"
#include "orientation.hpp"

using namespace std;


int getdir(string dir, vector<string> &files);
MatrixXf MergePointCloud(vector<MatrixXf> &PointCloudG);

int main(int argc, char *argv[]){
    //Set Parameter
    MatrixXf PointCloudIn, PointCloudOut;
    int arg_random = 5000;
	int arg_plane = 5;
	float arg_dist = 10.0f;
	float scale = 2048.0f;

    string dir = string("./input");
    vector<string> files = vector<string>();
    getdir(dir, files);

    for(int i=0; i<files.size(); ++i){
        cout << "Read file : " << files[i] << endl;
        PointCloudIn = scale * RANSAC::read_pc(("input/" + files[i]).c_str());
        vector<MatrixXf> PointCloudG;
    	OrientationCorrect(arg_random, arg_plane, arg_dist, PointCloudIn, PointCloudG);
        PointCloudOut = MergePointCloud(PointCloudG);

        //write file
        fstream file;
        char str[50];
        sprintf(str, "output/%s", files[i].c_str());
        file.open(str, ios::out);
        file << PointCloudOut.transpose();
        cout << "Done !!" << endl << endl;
    }

    return 0;
}

int getdir(string dir, vector<string> &files){
    DIR *dp;
    struct dirent *dirp;
    if((dp = opendir(dir.c_str())) == NULL)
        cout << "Error" << endl;

    while((dirp = readdir(dp)) != NULL){
        string filename = string(dirp->d_name);
        if(filename.compare(".") != 0 && filename.compare("..") != 0)
            files.push_back(filename);
    }
    closedir(dp);
    return 0;
}

MatrixXf MergePointCloud(vector<MatrixXf> &PointCloudG){
    int ArrLen = 0;
    for(int i=0; i<PointCloudG.size(); ++i)
        ArrLen += PointCloudG[i].cols();

    MatrixXf PointCloud(3, ArrLen);
    int count = 0;
    for(int i=0; i<PointCloudG.size(); ++i)
        for(int j=0; j<PointCloudG[i].cols(); ++j){
            PointCloud.col(count) = PointCloudG[i].col(j);
            ++count;
        }

    return PointCloud;
}

#include <iostream>
#include <fstream>
#include <cstdio>
#include <vector>

#define IMG_WIDTH 320
#define IMG_HEIGHT 240

using namespace std;

void DepthToWorld(int cx, int cy, unsigned short int cz, float &wx, float &wy, float &wz)
MatrixXf Pixel2Matrix(unsigned short *pDepth);

int main(int argc, char *argv[]){
    unsigned short pDepth[IMG_HEIGHT*IMG_WIDTH];
    FILE *fp;
    int speed = 15;
    if(argc>1)
        speed = atoi(argv[1]);
    if(argc==3)
        fp = fopen(argv[2], "rb");
    else
        fp = fopen("test.dvs", "rb");

    int FrameCount = 0;
    MatrixXf pc;
    while(1){
        if(fp && FrameCount%speed == 0){
            fread(pDepth, sizeof(unsigned short), IMG_WIDTH*IMG_HEIGHT, fp);
            FrameCount = 0;
        }
        pc = Pixel2Matrix(pDepth);
        //===========================


        //===========================
        ++FrameCount;
    }

    return 0;
}

/*====================================
Convert depth to world
====================================*/
void DepthToWorld(int cx, int cy, unsigned short int cz, float &wx, float &wy, float &wz){
	if (cz == 0){
		wx = 0;
		wy = 0;
		wz = 0;
		return;
	}

	wx = (cx - IMG_WIDTH/2) * cz / FOCAL_LEN;
	wy = (IMG_HEIGHT/2 - cy) * cz / FOCAL_LEN;
	wz = (float)cz;
}

/*====================================
Get point cloud form the depth image
====================================*/
MatrixXf Pixel2Matrix(unsigned short *pDepth){
    MatrixXf pc(3, IMG_WIDTH*IMG_HEIGHT)
    int count = 0;
    //get depth pixel
	for (int j = 0; j < IMG_HEIGHT; j++){
		for (int i = 0; i < IMG_WIDTH; i++){
			DepthToWorld(i, j, pDepth[j*IMG_WIDTH + i], wx, wy, wz);
			pc(0, count) = wx;
			pc(1, count) = wy;
			pc(2, count) = wz;
			++count;
		}
	}
    return pc;
}

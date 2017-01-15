#include <iostream>
#include <fstream>
#include <cstdio>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <vector>

#define IMG_WIDTH 320
#define IMG_HEIGHT 240

using namespace std;
using namespace cv;

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

    //OpenCV Initial Window
    cv::namedWindow("Depth Image", CV_WINDOW_AUTOSIZE);
    int FrameCount = 0;
    while(1){
        if(fp && FrameCount%speed == 0){
            fread(pDepth, sizeof(unsigned short), IMG_WIDTH*IMG_HEIGHT, fp);
            FrameCount = 0;
        }

        const cv::Mat mImageDepth(IMG_HEIGHT, IMG_WIDTH, CV_16UC1, (void *)pDepth);
        cv::Mat mScaledDepth;
        mImageDepth.convertTo( mScaledDepth, CV_8U, 3*255.0 / 10000 );
		cv::flip(mScaledDepth,mScaledDepth,1);
		cv::imshow( "Depth Image", mScaledDepth );
		if( cv::waitKey( 1 ) == 'q' )
            break;
        ++FrameCount;
    }

    return 0;
}

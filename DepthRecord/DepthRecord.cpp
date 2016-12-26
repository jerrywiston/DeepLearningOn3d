#include <iostream>
#include <fstream>
#include <cstdio>
#include <openni2/OpenNI.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <vector>

#define IMG_WIDTH 320
#define IMG_HEIGHT 240

using namespace std;
using namespace openni;
using namespace cv;

typedef vector< vector< DepthPixel > > DepthVideo;

static DepthPixel* get_depth_pixel(VideoStream &streamDepth, VideoFrameRef &frameDepth);
static void WriteFrame(DepthVideo &buffer, DepthPixel *pDepth);
static void WriteDVS(DepthVideo &buffer, string filename);

int main(int argc, char *argv[])
{
    //OpenNI Initial
	OpenNI::initialize();
	Device mDevice;

	if (mDevice.open(openni::ANY_DEVICE) != openni::STATUS_OK){
		std::cout << "ERROR: cannot open the device, please check your device connected" << endl;
		exit(EXIT_FAILURE);
	}

	//Create depth stream
	VideoStream mDepthStream;
	mDepthStream.create(mDevice, SENSOR_DEPTH);

	//set video mode
	VideoMode mMode;
	mMode.setResolution(IMG_WIDTH, IMG_HEIGHT);
	mMode.setFps(30);
	mMode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);

	//openNI start
	VideoFrameRef mDepthFrame;
	mDepthStream.start();
	int iMaxDepth = mDepthStream.getMaxPixelValue();

	//OpenCV Initial Window
    cv::namedWindow("Depth Image", CV_WINDOW_AUTOSIZE);

	//Depth Video buffer
	DepthVideo buffer;

	//Main Loop
    bool isRecord = false;
    while(1){
		DepthPixel *pDepth = get_depth_pixel(mDepthStream, mDepthFrame);
        if(isRecord)
            WriteFrame(buffer, pDepth);
		const cv::Mat mImageDepth(mDepthFrame.getHeight(), mDepthFrame.getWidth(), CV_16UC1, (void *)pDepth);
        cv::Mat mScaledDepth;
        mImageDepth.convertTo( mScaledDepth, CV_8U, 3*255.0 / iMaxDepth );
		cv::flip(mScaledDepth,mScaledDepth,1);
		cv::imshow( "Depth Image", mScaledDepth );

        char input = cv::waitKey(1);
        if( input == 'r' && isRecord == false){
            cout << "Start Recording ..." << endl;
            isRecord = true;
        }
        else if(input == 'q')
            break;
    }

	cout << "Total frame : " << buffer.size() << endl;
    if(argc>1)
        WriteDVS(buffer, argv[1]);
    else
        WriteDVS(buffer, "test.dvs");

	//Free memory
    mDepthStream.destroy();
    mDevice.close();
    OpenNI::shutdown();
    return 0;
}

/*=================================
Get depth map pixel
=================================*/
static DepthPixel* get_depth_pixel(VideoStream &streamDepth, VideoFrameRef &frameDepth)
{
	streamDepth.readFrame(&frameDepth);
	DepthPixel* pDepth = (DepthPixel*)frameDepth.getData();
	return pDepth;
}

static void WriteFrame(DepthVideo &buffer, DepthPixel *pDepth){
	vector <DepthPixel> pDepthSave;
	pDepthSave.resize(IMG_WIDTH*IMG_HEIGHT);
	for(int i=0; i<IMG_WIDTH*IMG_HEIGHT; ++i)
		pDepthSave[i] = pDepth[i];
	buffer.push_back(pDepthSave);
}

static void WriteDVS(DepthVideo &buffer, string filename){
	cout << "Save " << filename << " ..." << endl;
	int FrameCount = buffer.size();
	FILE *fp;
	fp = fopen(filename.c_str(), "wb");
	for(int i=0; i<FrameCount; ++i)
		fwrite(&(buffer[i][0]), sizeof(DepthPixel), IMG_WIDTH*IMG_HEIGHT, fp);

	fclose(fp);
	cout << "Done !!" << endl;
}

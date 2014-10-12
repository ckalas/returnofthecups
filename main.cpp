
#include "cups.h"
#include "device.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>


using namespace cv;
using namespace std;

int main(int argc, char **argv) {
	// Load classifier
	CascadeClassifier rectCup;
	if (!rectCup.load("rectCup.xml")) {
		cout << "Error loading classifier" << endl;
	}

	bool die(false);

	Mat depthMat(Size(640,480),CV_16UC1);
	Mat depthf (Size(640,480),CV_8UC1);
	Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
	Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));
	
	Freenect::Freenect freenect;
	MyFreenectDevice &device = freenect.createDevice<MyFreenectDevice>(0);
	
	namedWindow("rgb",CV_WINDOW_AUTOSIZE);
	namedWindow("depth",CV_WINDOW_AUTOSIZE);
	device.startVideo();
	device.startDepth();

	while (!die) {
		device.getVideo(rgbMat);
		device.getDepth(depthMat);
		detect_cups(&rgbMat, rectCup);
		cv::imshow("rgb", rgbMat);
		depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
		cv::imshow("depth",depthf); 
		char k = cvWaitKey(5);

		if( k == 27 ){
			cvDestroyWindow("rgb");
			cvDestroyWindow("depth");
			break;
		}

	}

	device.stopVideo();
	device.stopDepth();
	return 0;
}
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

    // Variables to determine FPS
    long int e1, e2;
    char k;
    double t;
    int fps;

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

    Mat cameraMatrix, dist, cameraInv;
	
    Freenect::Freenect freenect;
    MyFreenectDevice &device = freenect.createDevice<MyFreenectDevice>(0);
	
    namedWindow("rgb",CV_WINDOW_AUTOSIZE);
    namedWindow("depth",CV_WINDOW_AUTOSIZE);
    device.startVideo();
    device.startDepth();
    device.getCameraParams(&cameraMatrix,&dist,&cameraInv);
    while (!die) {
	// Check the clock tick
	//1 = 

	// Get new frames
	device.getVideo(rgbMat);
	device.getDepth(depthMat);

	//detect_cups(&rgbMat, rectCup);

	cv::imshow("rgb", rgbMat);
	cv::moveWindow("rgb", 0, 0);

	depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
	// Detect and locate cup/s
	detect_cups(&rgbMat, &depthMat, rectCup, &cameraInv);

	cv::imshow("rgb", rgbMat);
	cv::moveWindow("rgb", 0, 0);

	cv::imshow("depth",depthf); 
	cv::moveWindow("depth", 650, 0);

	char k = cvWaitKey(1);

	if( k == 'q' || k == 'Q' ){
	    cvDestroyWindow("rgb");
	    cvDestroyWindow("depth");
	    break;
	}
    }

    device.stopVideo();
    device.stopDepth();
    return 0;
}

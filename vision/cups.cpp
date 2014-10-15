#include <iostream>
#include <stdio.h>
#include <highgui.h>
#include "cups.h"

using namespace cv;
using namespace std;

void detect_cups(Mat *rgbMat, Mat *depthMat, CascadeClassifier cascade, Mat *inverseCamera) {

    std::vector<cv::Rect> matches;
    Mat gray, cameraCoords; // make this a vector of them ultimately
    uint8_t *depthPtr, z = 0;
    cvtColor(*rgbMat, gray, CV_BGR2GRAY);
    equalizeHist(gray,gray);

    cascade.detectMultiScale(gray, matches, 1.3, 3,0|CV_HAAR_SCALE_IMAGE, Size(20, 30));
	
    for (size_t i = 0; i < matches.size(); i++) {
	// Draw rectangle
	Point tl (matches[i].x, matches[i].y);
	Point br (matches[i].x+matches[i].width, matches[i].y+matches[i].height);
	rectangle(*rgbMat, tl ,br, Scalar( 0, 255, 255 ), +2, 4);
	// Get depth of cup
	Mat roi(*depthMat, matches[i]);
	imshow("test", roi);
	// depthPtr =  (uint8_t*)roi.col(matches[i].width / 2).data;
	depthPtr = roi.col(matches[i].width / 2).data;
	int fix = 0;
	/*	
	z = 0;


	while (!z) {

	    // cout << depthPtr[matches[i].height/2] << endl;
	    z = 1;
	    /*
	    z = (uint8_t)depthPtr[matches[i].height/2 + fix] ;
	    fix += 1;
	    cout << z << endl;

	}
	cout << z << endl;
	*/
	// Calculate distance from camera
	//Mat imageCoords = (Mat_<double>(1,3) << x*, -1, 0, -1, 5, -1, 0, -1, 0);
	//cameraCoords = *inverseCamera * 

	
    }
}

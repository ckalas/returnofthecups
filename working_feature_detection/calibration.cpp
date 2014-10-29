#include "calibration.h"
#include <iostream>

using namespace std;

int MyCalibration::addChessboardPoints(const vector<string>& filelist, Size& boardSize)
{

    vector<string>::const_iterator itImg;
    vector<Point2f> imageCorners;
    vector<Point3f> objectCorners;
    // initialize the chess board corners in the chessboard reference frame 3d scene points
    for(int i = 0; i<boardSize.height; i++) {
	for(int j = 0; j<boardSize.width; j++) {
	    objectCorners.push_back( Point3f(float(i)*squareLength, float(j)*squareLength, 0.0f));
	}
    }
    // 2D image points:
    Mat image, image2; // to contain chessboard image
    int successes = 0;
    for(itImg=filelist.begin(); itImg!=filelist.end(); itImg++) {
	image = imread(*itImg, 0);
	//cvtColor(image2, image, CV_BGR2GRAY);
	bool found = findChessboardCorners(image, boardSize, imageCorners);
	/*
	drawChessboardCorners(image, boardSize, imageCorners, found);
	imshow("Chess", image);
	waitKey(100);
	*/
	cornerSubPix(image, imageCorners, Size(5,5), Size(-1, -1),
		     TermCriteria(TermCriteria::MAX_ITER+TermCriteria::EPS, 30, 0.1));
	// if we have a good board, add it to our data
	if (uint(imageCorners.size()) == uint(boardSize.area())) {
	    addPoints(imageCorners, objectCorners);
	    successes++;
	}
    }
    
    return successes;				     
}

void MyCalibration::addPoints(const vector<Point2f> &imageCorners, const vector<Point3f> &objectCorners)
{
    // 2D image point from one view
    imagePoints.push_back(imageCorners);
    // correspoinding 3D scene points
    objectPoints.push_back(objectCorners);
}

double MyCalibration::calibrate(Size &imageSize)
{
    mustInitUndistort = true;
    vector<Mat> rvecs, tvecs;
    double temp =  calibrateCamera(objectPoints, // the 3D points
			   imagePoints,
			   imageSize,
			   cameraMatrix, // output camera matrix
			   distCoeffs,
			   rvecs,
			   tvecs,
			   flags);

    cout << cameraMatrix << endl;
    cout << distCoeffs << endl;

    return temp;

}

void MyCalibration::remap(const Mat &image, Mat &undistorted) 
{
    cout << cameraMatrix;
    if(mustInitUndistort) { // called once per calibration
	initUndistortRectifyMap(cameraMatrix,
				distCoeffs,
				Mat(),
				cameraMatrix,
				image.size(),
				CV_32FC1,
				map1, 
				map2);
	mustInitUndistort = false;
    }
    // apply mapping function
    //remap(image, undistorted, map1, map2, cv::INTER_LINEAR);
    // ERROR HERE
}
				

#include "detect_markers.h"
#include "device.h"
#include "calibration.h"
#include <iostream>
#include <highgui.h>
#include <string>
#include <cv.h>

using namespace std;
using namespace cv;

string dispenser = "sift/auto_dispenser.png";
string fiducial = "sift/id7.png";

int main (int argc, char **argv) {
    // Setting up the interfacing with Kinect
    Freenect::Freenect freenect;
    MyFreenectDevice &device = freenect.createDevice<MyFreenectDevice>(0);

    // Variable to storage video and depth
    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf(Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat undistortMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat grayMat(Size(640,480),CV_8UC1);
    Mat map1, map2;
    char k;

    // Variable to get the FPS
    long int e1, e2;
    double t;
    int fps;

    // Variables to save images incrementally
    stringstream sstm;
    string result;
    int counter = 0;

    // Setup variables and list to read the calibration
    Size boardSize(8,6);

    // Setup window to display the video and depth
    namedWindow("rgb", CV_WINDOW_AUTOSIZE);
    moveWindow("rgb", 0, 0);
    namedWindow("depth", CV_WINDOW_AUTOSIZE);
    moveWindow("depth", 650, 0);
    namedWindow("undistort", CV_WINDOW_AUTOSIZE);
    moveWindow("undistort", 0, 490);
    device.startVideo();
    device.startDepth();

    // import yml data
    string filename = "data.yml";
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    // read camera matrix and dist_coeff
    Mat intrinsics, distortion;
    fs["Camera_Matrix"] >> intrinsics;
    fs["Dist_Coeff"] >> distortion;
    // close the input file
    fs.release();

    cout << "intrinsics: " << intrinsics << endl;
    cout << "distortion: " << distortion << endl;

    Mat rvec = Mat(Size(3,1), CV_64F);
    Mat tvec = Mat(Size(3,1), CV_64F);

    vector<Point2d> imageFramePoints, imageOrigin;
    vector<Point3f> boardPoints, framePoints;
    vector<Point2f> imagePoints;

    // BEWARE, MUST BE CORRECT OTHERWISE EXCEPTION IS THROWN
    for(int i=0; i<8; i++) {
	for(int j=0; j<6; j++) {
	    boardPoints.push_back( Point3f( float(i), float(j), 0.0) );
	}
    }

    // generate points in the reference frame
    framePoints.push_back( Point3f( -3.0, 3.0, 0.0 ) );
    framePoints.push_back( Point3f( 0.0, 3.0, 0.0 ) );
    framePoints.push_back( Point3f( 0.0, 0.0, -3.0 ) );

    //waitKey(1000);

    while (1) {
	e1 = getTickCount();

	// Read from Kinect to variables
	device.getVideo(rgbMat);
	device.getDepth(depthMat);
	depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
	// Display video and depth
	cvtColor(rgbMat, grayMat, CV_BGR2GRAY);
	//imshow("rgb", rgbMat);
	imshow("depth", depthf);
	// undistort image
	Mat newMat = getOptimalNewCameraMatrix(intrinsics, distortion, rgbMat.size(), 1.0);

	initUndistortRectifyMap( intrinsics, distortion, Mat_<double>::eye(3,3), newMat,
				 rgbMat.size(), map1.type(), map1, map2);
        remap( rgbMat, undistortMat, map1, map2, INTER_LINEAR, BORDER_CONSTANT );
	//undistort(rgbMat, undistortMat, intrinsics, distortion);

	// Read keyboard inpu
	k = waitKey(1);

	// 3d checkboard pose
	//bool found = findChessboardCorners(grayMat, boardSize, imagePoints);
	/*
	if (found) {
	    Mat temp = undistortMat.clone();
	    drawChessboardCorners( temp, boardSize, imagePoints, found);
	    imshow("checker", temp);
	    solvePnP( Mat(boardPoints), Mat(imagePoints), intrinsics, distortion, rvec, tvec, false);

	    projectPoints(framePoints, rvec, tvec, intrinsics, distortion, 
			  imagePoints );

	    circle( undistortMat, imagePoints[0], 4, CV_RGB(255,0,0) );
	    //cout << imagePoints[0] << "|" << imagePoints[1] << endl;
	    line(undistortMat, imagePoints[0], imagePoints[1], CV_RGB(255,0,0), 2);
	    line(undistortMat, imagePoints[0], imagePoints[2], CV_RGB(0,255,0), 2);
	    line(undistortMat, imagePoints[0], imagePoints[3], CV_RGB(0,0,255), 2);
	    //imshow("rgb", rgbMat);

	    cout << fixed << setprecision(2) << "rvec = ["
		 << rvec.at<double>(0,0) << ", "
		 << rvec.at<double>(1,0) << ", "
		 << rvec.at<double>(2,0) << "] \t tvec = ["     
		 << tvec.at<double>(0,0) << ", "                // horizontal
		 << tvec.at<double>(1,0) << ", "                
		 << tvec.at<double>(2,0) << "]" << endl;        // depth
	}
	*/
	

	//checkSIFT(undistortMat, fiducial, intrinsics, distortion);

	if(k == 'q'){
	    break;
	}
	else if (k == 's') {
	    sstm.str(string());
	    sstm << "calibration/img" << counter << ".png";
	    result = sstm.str();
	    imwrite(result, grayMat);
	    counter++;
	}
	else if (k == 'm') {
	    checkSIFT(undistortMat, fiducial, intrinsics, distortion);
	}
	
	// display the video and depth in window
	imshow("undistort", undistortMat);
	imshow("rgb", rgbMat);

	e2 = getTickCount();
	t = double(e2 - e1) / getTickFrequency();
	fps = int( 1 / t );
	//cout << "FPS: " << fps << endl;
    }    

    device.stopVideo();

    return 0;
}






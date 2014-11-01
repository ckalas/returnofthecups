#include "detect_markers.h"
#include "device.h"
#include "calibration.h"
#include <iostream>
#include <highgui.h>
#include <string>
#include <cv.h>

#define WIDTH 640
#define HEIGHT 480

using namespace std;
using namespace cv;

string dispenser = "sift/auto_dispenser.png";
string fiducial = "sift/id7.png";
string sift_mark = "sift/sift_marker.png";
string surf_mark = "sift/surf_marker.png";

void on_trackbar( int, void*) { }

int main (int argc, char **argv) {
    // Setting up the interfacing with Kinect
    Freenect::Freenect freenect;
    MyFreenectDevice &device = freenect.createDevice<MyFreenectDevice>(0);
    //device.setVideoFormat(FREENECT_VIDEO_RGB, FREENECT_RESOLUTION_HIGH);

    // Variable to storage video and depth
    /*
    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf(Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat undistortMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat grayMat(Size(640,480),CV_8UC1);
    */
    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf(Size(640,480),CV_8UC1);
    Mat rgbMat(Size(WIDTH, HEIGHT),CV_8UC3,Scalar(0));
    Mat undistortMat(Size(WIDTH, HEIGHT),CV_8UC3,Scalar(0));
    Mat grayMat(Size(WIDTH,HEIGHT),CV_8UC1);


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
    //namedWindow("undistort", CV_WINDOW_AUTOSIZE);
    //moveWindow("undistort", 0, 490);
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

    // create trackbar for prototype
    string s1 = "min feature", s2 = "min distance", s3 = "multiplier";
    const int feat_slider_max = 10000, dist_slider_max = 10000, multi_slider_max = 100;
    int feat_slider = 500, dist_slider = 750, multi_slider = 5;

    createTrackbar(s1, "rgb", &feat_slider, feat_slider_max, on_trackbar);
    createTrackbar(s2, "rgb", &dist_slider, dist_slider_max, on_trackbar);
    createTrackbar(s3, "rgb", &multi_slider, multi_slider_max, on_trackbar);

    
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
	undistort(rgbMat, undistortMat, intrinsics, distortion);

	// Read keyboard input
	k = waitKey(1);

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
	else if (k == 'f') {
	    checkSIFT(undistortMat, fiducial, intrinsics, distortion,
		      500, 750, 3);
	    // note 4.863 seems like a good multiplier
	    //feat_slider, dist_slider, multi_slider);
	}
	else if (k == 'n') {
	    checkSIFT(rgbMat, sift_mark, intrinsics, distortion,
		      100, 10000, 2);
		      //feat_slider, dist_slider, multi_slider);

	}
	else if (k == 'm') {
	    checkSIFT(rgbMat, surf_mark, intrinsics, distortion,
		      feat_slider, dist_slider, multi_slider);
	}
	else if(k == 'c') {
	    checkSIFT(rgbMat, dispenser, intrinsics, distortion,
		      feat_slider, dist_slider, multi_slider);
	}
	
	
	// display the video and depth in window
	//imshow("undistort", undistortMat);
	imshow("rgb", rgbMat);

	e2 = getTickCount();
	t = double(e2 - e1) / getTickFrequency();
	fps = int( 1 / t );
	//cout << "FPS: " << fps << endl;
    }    

    device.stopVideo();

    return 0;
}






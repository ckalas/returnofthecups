//please include id7.png in the directory the program is executed

#ifndef FIDUCIAL_H
#define FIDUCIAL_H

#include <cstdio>
#include <iostream>
#include <cmath>

#include <highgui.h>
#include <cv.h>

using namespace std;
using namespace cv;

bool find_fid(Mat *rgbMat, Mat *depthMat, Mat *inverseCamera);
bool siftFeature(Mat *rgbMat, Mat *cameraMat, Mat *distCoeffs);
Mat reconfigure_reference(Mat rvec, Mat tvec);

#define MIN_MATCH_COUTN = 10
#define FID_WIDTH = 4.6 //[cm]

#endif //FIDUCIAL_H

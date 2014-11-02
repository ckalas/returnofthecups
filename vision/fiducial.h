//please include id7.png in the directory the program is executed

#ifndef FIDUCIAL_H
#define FIDUCIAL_H

#include <cstdio>
#include <iostream>
#include <cmath>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include <highgui.h>
#include <cv.h>

#define MIN_MATCH_COUNT 10
#define FID_WIDTH 4.6 // cm

using namespace std;
using namespace cv;

bool find_fid(Mat *rgbMat, Mat *depthMat, Mat *inverseCamera, Mat *HT);
bool sift_feature(Mat *rgbMat, Mat *cameraMat, Mat *distCoeffs, Mat *HT);
bool check_sift(Mat src, string objectString, Mat intrinsics, Mat distortion, int minFeat, int minDist, int multi);
void reconfigure_reference(Mat *rvec, Mat *tvec, Mat *HT);


#endif

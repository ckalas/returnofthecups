#ifndef FIDUCIAL_H
#define FIDUCIAL_H

#include <cstdio>
#include <iostream>
#include <math.h>
#include <cmath>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <highgui.h>
#include <cv.h>

#define MIN_MATCH_COUNT 10
#define FID_SIZE 8.0 // The size of the fiducial marker - VERY important to be right
#define FID_PIX 10 // An offset to find marker in depthMat, a bit hacky.

using namespace std;
using namespace cv;

bool check_sift(Mat src, Mat depthMat, string objectString, Mat intrinsics, Mat distortion, int minFeat, int minDist, int multi,  Mat &HT, FILE *output);
Mat reconfigure_reference(Mat rvec, Mat tvec);


#endif

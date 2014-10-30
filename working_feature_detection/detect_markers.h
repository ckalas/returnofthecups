#ifndef CHECK_MARKERS_H
#define CHECK_MARKERS_H

#include <cv.h>

using namespace cv;

void cameraPostFromHomography(const Mat& H, Mat& pose);
void checkSIFT(Mat src, string objectString, Mat intrinsics, Mat distortion);

#endif

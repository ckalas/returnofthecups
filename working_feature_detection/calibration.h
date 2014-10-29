#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <string>

using namespace cv;

class MyCalibration{
 private:
    vector<vector<Point3f> > objectPoints;
    vector<vector<Point2f> > imagePoints;
    // Square length
    float squareLength;
    // Output matrices
    Mat cameraMatrix; //intrinsic
    Mat distCoeffs;
    // flag to specify how calibration is done
    int flags;
    // used in image undistortion
    Mat map1, map2;
    bool mustInitUndistort;

 public:
    int addChessboardPoints(const vector<string>& filelist, Size& boardSize);
    void addPoints(const vector<Point2f>& imageCorners, const vector<Point3f>& objectCorners);
    double calibrate(Size &imageSize);
    void remap(const Mat &image, Mat &undistorted);
};

#endif

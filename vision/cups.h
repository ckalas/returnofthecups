#include <cv.h>

void detect_cups(cv::Mat *rgbMat, cv::Mat *depthMat, cv::CascadeClassifier cascade, cv::Mat *inverseCamera);
void show_fps(cv::Mat *rgbMat, int fps);
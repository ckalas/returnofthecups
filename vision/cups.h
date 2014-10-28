#include <cv.h>

using namespace std;
using namespace cv;

void detect_cups(Mat *rgbMat, Mat depthMat, CascadeClassifier cascade, Mat inverseCamera);
void find_cups(Mat *gray, CascadeClassifier cascade, vector<Point2f>  *points);
void show_fps(Mat *rgbMat, int fps);
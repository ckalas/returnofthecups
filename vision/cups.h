#include <cv.h>

using namespace std;
using namespace cv;

#define OFF_X 180
#define OFF_Y 220

#define FID_DIM 5.8

void accumlate_cups(Mat *rgbMat, CascadeClassifier cascade, vector<Point2f> *points);
void average_cups(vector<Point2f> *points) ;
void draw_cups(Mat *rgbMat, vector<Point2f> points) ;
void detect_cups(Mat *rgbMat, Mat depthMat, CascadeClassifier cascade, Mat inverseCamera, Mat HT);
void show_fps(Mat *rgbMat, int fps);
void print_mat3(Mat points, string label);
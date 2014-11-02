#include <cv.h>

using namespace std;
using namespace cv;

void accumlate_cups(Mat *rgbMat, CascadeClassifier cascade, vector<Point2f> *points);
void average_cups(vector<Point2f> *points) ;
void draw_cups(Mat *rgbMat, vector<Point2f> points) ;
void detect_cups(Mat *rgbMat, Mat depthMat, CascadeClassifier cascade, Mat inverseCamera, Mat HT);
void show_fps(Mat *rgbMat, int fps);
void print_point3(Mat points, string label);
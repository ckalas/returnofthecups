#ifndef CUPS_H
#define CUPS_H
#include <cv.h>

using namespace std;
using namespace cv;

#define OFF_X 180
#define OFF_Y 220

#define FID_DIM 8

enum cup_size {LARGE, MEDIUM};
typedef struct cup_t {
    int size;
    bool sorted;
    double timestamp, depth;
    Point2f pixelLocation;
    Point3f worldLocation;
} Cup;

void accumlate_cups(Mat *rgbMat, Mat depthMat, CascadeClassifier cascade, vector<Cup> *cups,Mat inverseCamera, Mat HT);
void average_cups(vector<Cup> *cups);
void draw_cups(Mat *rgbMat, vector<Cup> cup);
void transpose_cup(Cup cup);

double get_time(void) ;
double elapsed_time(double previous);
void show_fps(Mat *rgbMat, int fps);
void print_mat3(Mat points, string label);
#endif
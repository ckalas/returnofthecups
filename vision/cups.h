#ifndef CUPS_H
#define CUPS_H
#include <cv.h>
#include <stdint.h>

using namespace std;
using namespace cv;

#define OFF_X 250 //200 //270 at 70 cm //180 original
#define OFF_Y 220 //200 //150 at 90 cm //220 original

#define FID_DIM 8

enum cup_size {LARGE, MEDIUM};
typedef struct cup_t {
    int size;
    double depth;
    time_t timestamp;
    Point2f pixelLocation;
    Point3f worldLocation;
} Cup;

void accumlate_cups(Mat *rgbMat, Mat depthMat, CascadeClassifier cascade, vector<Cup> *cups,Mat inverseCamera, Mat HT);
void average_cups(vector<Cup> *cups);
void draw_cups(Mat *rgbMat, vector<Cup> cup);
void cup_info(vector<Cup> cups);
Point2f cup_prediction(float t, Point2f p_0);
int cup_classify(Mat depth,Point2f centre);
vector<uint8_t> take_order(void);
bool print_next_order(int cupsize, vector<uint8_t> *orders);

time_t get_time(void) ;
double elapsed_time(time_t previous);
void show_fps(Mat *rgbMat, int fps);
void print_mat3(Mat points, string label);
#endif

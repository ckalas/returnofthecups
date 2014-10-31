#include "cups.h"
#include "device.h"
#include "fiducial.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>


using namespace cv;
using namespace std;

int main(int argc, char **argv) {

    // Variables to determine FPS
    long int e1, e2;
    double t;
    int fps;

    // Load classifier
    CascadeClassifier rectCup;
    if (!rectCup.load("rectCup.xml")) {
	cout << "Error loading classifier" << endl;
    }
	
    bool die(false), showFrames(false), showDepth(false), showFinder(false), showTarget(true);
	
    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat HT;

    Mat cameraMatrix, dist, cameraInv;
	
    Freenect::Freenect freenect;
    MyFreenectDevice &device = freenect.createDevice<MyFreenectDevice>(0);
	
    device.startVideo();
    device.startDepth();

    device.getCameraParams(&cameraMatrix,&dist,&cameraInv);

    vector<Point2f> points;

    // Find the cups across 50 frames
    for(int i = 0; i < 50; i++) {
        device.getVideo(rgbMat);
        accumlate_cups(&rgbMat, rectCup, &points);
    }
    // Decide which cups are valid
    device.getVideo(rgbMat);
    points = average_cups(points);
    draw_cups(&rgbMat, points);
    imshow("rgb", rgbMat);
    waitKey(0);

    // Fiducial locating of origin
    /*
    do {
        device.getVideo(rgbMat);
    }
    while(!find_fid(&rgbMat, &cameraMatrix, &dist, &HT)) ; */

    while (!die) {
       // Check the clock tick
        e1 = cv::getTickCount();

        device.getVideo(rgbMat);
        device.getDepth(depthMat);

        if (showFinder) {
            detect_cups(&rgbMat, depthMat, rectCup, cameraInv);
        }
        if (showFrames) {
            // Calculate the fps and finding the time diff executing the code
            e2 = cv::getTickCount();
            t = double((e2 - e1) / cv::getTickFrequency());
            fps = int( 1 / t );
            show_fps(&rgbMat, fps);
        }

        if (showDepth) {
            depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
            imshow("depth", depthf);
        }

        if (showTarget) {
            rectangle(rgbMat, Point(180,250), Point(500,450), Scalar(255,0,0));
        }

        imshow("rgb", rgbMat);

        char c = (char)waitKey(10);

        if(c == 27) {
            break;
        }

        switch(c) {
            case 'f':
                showFrames = showFrames? false: true;
                break;
            case 's':
                showFinder = showFinder? false: true;
                break;
            case 'd':
                showDepth = showDepth? false : true;
                if (!showDepth){
                    destroyWindow("depth");
                }
                else {
                    namedWindow("depth", CV_WINDOW_AUTOSIZE);
                    moveWindow("depth", 650, 0);
                }
                break;
            case 'r':
                showTarget = showTarget ? false : true;
            }

    }

    device.stopVideo();
    device.stopDepth();
    return 0;
}

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

    RNG rng(12345);
    
    // Load classifier
    CascadeClassifier rectCup;
    if (!rectCup.load("rectCup.xml")) {
	cout << "Error loading classifier" << endl;
    }
	
    bool die(false), showFrames(false), showDepth(false), showFinder(false), showContours(false);
	
    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat HT;

    Mat cameraMatrix, dist, cameraInv;
	
    Freenect::Freenect freenect;
    MyFreenectDevice &device = freenect.createDevice<MyFreenectDevice>(0);
	
    namedWindow("rgb",CV_WINDOW_AUTOSIZE);
    moveWindow("rgb", 0, 0);
    device.startVideo();
    device.startDepth();

    device.getCameraParams(&cameraMatrix,&dist,&cameraInv);


    // Variables for object tracking
    int newCups = 0;
    bool addRemovePt = false;
    Point2f point;
    vector<Point2f> points[2];
    Mat gray, prevGray, image;
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    Size subPixWinSize(10,10), winSize(31,31);

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

        
        cvtColor(rgbMat, gray, COLOR_BGR2GRAY);

        if (showContours) {
             cout << "hi" << endl;
            /// Reduce the noise so we avoid false circle detection
            GaussianBlur( gray, gray, Size(9, 9), 2, 2 );

            vector<Vec3f> circles;

            /// Apply the Hough Transform to find the circles
            HoughCircles( gray, circles, CV_HOUGH_GRADIENT, 1, gray.rows/8, 200, 100, 0, 0 );

            /// Draw the circles detected
            for( size_t i = 0; i < circles.size(); i++ ) {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);
                // circle center
                circle( rgbMat, center, 3, Scalar(0,255,0), -1, 8, 0 );
                // circle outline
                circle( rgbMat, center, radius, Scalar(0,0,255), 3, 8, 0 );
            }
        }

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
            case 'c':
                showContours = showContours ? false : true;
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
                points[0].clear();
                points[1].clear();
                break;
            }

    }

    device.stopVideo();
    device.stopDepth();
    return 0;
}

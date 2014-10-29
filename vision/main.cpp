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
	
    bool die(false), showFrames(false), showDepth(false);
	
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

        newCups = find_cups(&gray, rectCup, &points[1]);
        cout << "0: " << points[0] << endl << "1: " << points[1] << endl;

        if (!points[0].empty()) {
            // Detect and locate cup/s
            //detect_cups(&rgbMat, depthMat, rectCup, cameraInv);
            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty()) gray.copyTo(prevGray);

            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,3, termcrit, 0, 0.001);
            size_t i, k;
            for( i = k = 0; i < points[1].size(); i++ ) {

                if( !status[i] ) continue;

                points[1][k++] = points[1][i];
                circle(rgbMat, points[1][i], 3, Scalar(0,255,255), -1, 8);
                points[1].resize(k);
            }
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
                showFrames ^= showFrames;
                break;
            case 'd':
                showDepth ^= showDepth;
                if (!showDepth){
                    destroyWindow("depth");
                }
                else {
                    namedWindow("depth", CV_WINDOW_AUTOSIZE);
                    moveWindow("depth", 650, 0);
                }
                break;

            case 'c':
                points[0].clear();
                points[1].clear();
                break;
            }

        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);

    }

    device.stopVideo();
    device.stopDepth();
    return 0;
}

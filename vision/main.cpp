#include <unistd.h>
#include <stdlib.h>
#include <fstream>
#include "cups.h"
#include "device.h"
#include "fiducial.h"

#define DEBUG 0

using namespace cv;
using namespace std;

int main(int argc, char **argv) {


    int toParent[2], fromParent[2];
    pipe(toParent);
    pipe(fromParent);

    if (fork()) { // parent
        close(toParent[1]);
        close(fromParent[0]);
    }
    else {
        close(toParent[0]);
        close(fromParent[1]);
        dup2(toParent[1], 1);
        dup2(fromParent[0], 0);
        close(toParent[1]);
        close(fromParent[0]);
        execl("robot", "robot", (char *) NULL);
    }

    FILE * output = fdopen(fromParent[1], "w");
    FILE * input = fdopen(toParent[0], "r");

    string robot = "id7.png";
    string autoFill = "id11.png";
    string classifier = "rectCup.xml";
    // Load Haar classifier
    CascadeClassifier rectCup;
    if(!rectCup.load(classifier)) {
        cout << "Error loading classifier" << endl;
        return 1;
    }
    // Variables to determine FPS
    long int e1, e2;
    double t;
    int fps;

    // Flags for output options	
    bool finished(false), showFrames(false), showDepth(false), showTarget(true), showTracker(true);

    // Storage for RGB and DEPTH frames
    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));

    // Import camera data from yml file
    Mat cameraMatrix, dist, cameraInv, HT;
    string filename = "data.yml";
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Dist_Coeff"] >> dist;
    cameraInv = cameraMatrix.inv();
    fs.release();

    // Storage for cup locations (aggregate)
    vector<Cup> cups;
    vector<double> tvec_r1 (3);

    // Setup the kinect device
    Freenect::Freenect freenect;
    MyFreenectDevice &device = freenect.createDevice<MyFreenectDevice>(0);

    // Start streaming frames
    device.startVideo();
    device.startDepth();

    cout << "Locating robot" << endl;
    // Locate fiducial at robot base
    do {
        device.getVideo(rgbMat);
        device.getDepth(depthMat);
    }
    while(!check_sift(rgbMat, depthMat, robot, cameraMatrix, dist, 500, 750, 3, HT, &tvec_r1));
    HT.convertTo(HT, CV_64F);
    cout << "Located robot" << endl;

    cout << "Locating auto fill" << endl;

    // Locate fiducial at auto fill
    do {
        device.getVideo(rgbMat);
        device.getDepth(depthMat);
    }
    while(!check_sift(rgbMat, depthMat, autoFill, cameraMatrix, dist, 500, 750, 3, HT, &tvec_r1));
    cout << "Located auto fill" << endl;

    cout << "Transform to auto fill "<< endl << tvec_r1[0] << ", " << endl  << tvec_r1[1] << ", "  << tvec_r1[2] << ", " <<endl;;

    // Send XYZ of autofill to child
    fprintf(output, "%f\n%f\n0\n", tvec_r1[0],tvec_r1[1]);
    fflush(output);

    #if DEBUG
    device.getVideo(rgbMat);
    draw_cups(&rgbMat, cups);
    imshow("rgb", rgbMat);
    waitKey(0);
    #endif
    cout << "Commencing main loop" << endl;
    // Main loop
    while (!finished) {
       // Check the clock tick
        e1 = cv::getTickCount();
        device.getVideo(rgbMat);
        device.getDepth(depthMat);


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

        if (showTracker) {
            accumlate_cups(&rgbMat, depthMat, rectCup, &cups, cameraInv, HT);
            average_cups(&cups);
            draw_cups(&rgbMat, cups);
            if (cups.size() > 0) {
                    double x = cups[0].worldLocation.x;
                    double z = cups[0].worldLocation.z;
                    double xt = -(x-18);
                    double yt = -(z+6)
                    fprintf(output, "%f\n%f\n0\n", xt,yt);
                    fflush(output);
            }
            //usleep(1000);
        }


        if (showTarget) {
            rectangle(rgbMat, Point(180,220), Point(500,430), Scalar(255,0,0), 3);
        }

        imshow("rgb", rgbMat);

        char c = (char)waitKey(10);

        switch(c) {
            case 27:
                finished = true;
                break;
            case 'f':
                showFrames = showFrames? false: true;
                break;
            case 't':
                showTracker = showTracker ? false : true;
                cout << "Tracking : " << showTracker << endl;
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
                cout << "Depth : " << showDepth << endl;
                break;
            case 'i':
                if (cups.size() > 0) {
                    cup_info(cups);
                }
                break;
            case 'n':
                cout << "Cleared cups" << endl;
                cups.clear();
                break;
            case 'r':
                showTarget = showTarget ? false : true;
                cout << "Display ROI: " << showTarget << endl;
            }

    }

    device.stopVideo();
    device.stopDepth();
    return 0;
}

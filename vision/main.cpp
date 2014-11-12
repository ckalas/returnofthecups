#include <unistd.h>
#include <stdlib.h>
#include <fstream>
#include "cups.h"
#include "device.h"
#include "fiducial.h"

#define DEBUG 0
#define ROBOT 1

using namespace cv;
using namespace std;

int main(int argc, char **argv) {

    vector<uint8_t> orders = take_order();

    // Pipe, fork, exec (to run robot as child)
 
    int toParent[2], fromParent[2];
    pipe(toParent);
    pipe(fromParent);

    #if ROBOT
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
        execl("../robot/robot", "../robot/robot", (char *) NULL);
    }
    #endif

    FILE * output = fdopen(fromParent[1], "w");
    FILE * input = fdopen(toParent[0], "r");

    // Set up for select() read of input pipe
    fd_set set;
    struct timeval timeout;

    /* Initialize the file descriptor set. */
    FD_ZERO(&set);
    FD_SET(toParent[0], &set);

    /* Initialize the timeout data structure. */
    timeout.tv_sec = 0;
    timeout.tv_usec = 10;
   

    string robot = "id7.png";
    string autoFill = "id11.png";
    string coaster = "id12.png";
    string classifier = "rectCup.xml";
    // Load Haar classifier
    CascadeClassifier rectCup;
    if(!rectCup.load(classifier)) {
        cout << "Error loading classifier" << endl;
        return 1;
    }

    // Flags for output options 
    bool finished(false), showTarget(true), ready(true);

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
    // Location of autofill fiducial
    vector<double> tvec_r1 (3);

    // Setup the kinect device
    Freenect::Freenect freenect;
    MyFreenectDevice &device = freenect.createDevice<MyFreenectDevice>(0);

    // Start streaming frames
    device.startVideo();
    device.startDepth();


    // Locate fiducial at robot base
    cout << "Locating robot" << endl;
    do {
        device.getVideo(rgbMat);
        device.getDepth(depthMat);
	//usleep(10000);
    }
    while(!check_sift(rgbMat, depthMat, robot, cameraMatrix, dist, 500, 750, 3, HT, output));
    HT.convertTo(HT, CV_64F);
    cout << "Located robot" << endl;

    // Locate fiducial at auto fill
    cout << "Locating auto fill..." << endl;
    do {
        device.getVideo(rgbMat);
        device.getDepth(depthMat);
    }
    while(!check_sift(rgbMat, depthMat, autoFill, cameraMatrix, dist, 500, 750, 3, HT, output));
    cout << "Located auto fill" << endl;

    // Locate fiducial at auto fill
    cout << "Locating coaster..." << endl;
    do {
        device.getVideo(rgbMat);
        device.getDepth(depthMat);
    }
    while(!check_sift(rgbMat, depthMat, coaster, cameraMatrix, dist, 500, 750, 3, HT, output));
    cout << "Located coaster" << endl;

    cout << "Commencing main loop" << endl;
    double cupOffset;
    // Main loop
    while (!finished) {

        device.getVideo(rgbMat);
        device.getDepth(depthMat);

        accumlate_cups(&rgbMat, depthMat, rectCup, &cups, cameraInv, HT);
        average_cups(&cups);
        draw_cups(&rgbMat, cups);
        if (cups.size() > 0) {
            cupOffset = cups[0].size ? 10-5.5 : 6-5.5;
            Point2f prediction =  Point2f(-(cups[0].worldLocation.x-18), -(cups[0].worldLocation.z+cupOffset));
            if (ready) {
                if(orders.size() > 0 && print_next_order(cups[0].size, &orders)) {
                    cup_info(cups);
                    fprintf(output, "%f\n%f\n0\n", prediction.x, prediction.y);
                    fflush(output);
                    ready = false;
                }
            }
            // Non-blocking read of pipe
            if (select(toParent[0]+1, &set, NULL, NULL, &timeout) > 0) {
		fgetc(input);
                ready = true;
            }
	    FD_ZERO(&set);
	    FD_SET(toParent[0],&set);

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
            case 'n':
                cout << "Cleared cups" << endl;
                cups.clear();
                break;
            case 'r':
                showTarget = showTarget ? false : true;
                cout << "Display ROI: " << showTarget << endl;
		break;
	}
    }

    device.stopVideo();
    device.stopDepth();
    return 0;
}

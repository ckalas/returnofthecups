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


    // Pipe, fork, exec (to run robot as child)

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
        execl("../robot/robot", "../robot/robot", (char *) NULL);
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

    // Flags for output options 
    bool finished(false), showTarget(true);

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

    cout << "Transform to auto fill "<< endl << tvec_r1[0] << ", " << endl  << tvec_r1[1] << ", "  << tvec_r1[2] << ", " <<endl;

    // Send XYZ of autofill to child
    fprintf(output, "%f\n%f\n0\n", tvec_r1[0],tvec_r1[1]);
    fflush(output);

    cout << "Commencing main loop" << endl;
    // Main loop
    while (!finished) {

	device.getVideo(rgbMat);
	device.getDepth(depthMat);

        accumlate_cups(&rgbMat, depthMat, rectCup, &cups, cameraInv, HT);
        average_cups(&cups);
        draw_cups(&rgbMat, cups);
        if (cups.size() > 0) {
            char temp;
            double x = cups[0].worldLocation.x;
            double z = cups[0].worldLocation.z;
            double xt = -(x - 18);
            double yt = -(z + 6);
            cup_info(cups);
            fprintf(output, "%f\n%f\n0\n", xt, yt);
            fflush(output);
            cout << "blocking in vision " << endl;
            temp = fgetc(input);
            cout << "Temp: " << temp << endl;

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

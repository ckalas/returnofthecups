#include "cups.h"
#include "device.h"
#include "fiducial.h"

#define DEBUG 1

using namespace cv;
using namespace std;

string robot = "id7.png";
string autoFill = "id11.png";
string classifier = "rectCup.xml";

int main(int argc, char **argv) {

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
    bool finished(false), showFrames(false), showDepth(false), showFinder(false), showTarget(true),
             showTracker(false);

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

    // Setup the kinect device
    Freenect::Freenect freenect;
    MyFreenectDevice &device = freenect.createDevice<MyFreenectDevice>(0);

    // Start streaming frames
    device.startVideo();
    device.startDepth();

    #if DEBUG
    cout << "Locating robot" << endl;
    #endif
    // Locate fiducial at robot base
    do {
        device.getVideo(rgbMat);
        device.getDepth(depthMat);
    }
    while(!check_sift(rgbMat, depthMat, robot, cameraMatrix, dist, 500, 750, 3, HT));
    #if DEBUG
    cout << "Locating auto fill" << endl;
    #endif
    // Locate fiducial at auto fill
    do {
        device.getVideo(rgbMat);
        device.getDepth(depthMat);
    }
    while(!check_sift(rgbMat, depthMat, autoFill, cameraMatrix, dist, 500, 750, 3, HT));

    #if DEBUG 
    cout << "Homogenous Transform to Fiducial "<< endl << HT << endl;
    #endif
    HT.convertTo(HT, CV_64F);

    // Find the cups across 50 frames
    for(int i = 0; i < 100; i++) {
        device.getVideo(rgbMat);
        accumlate_cups(&rgbMat, depthMat, rectCup, &cups,cameraInv, HT);
        waitKey(100);
    }
    // Decide which cups are valid
    average_cups(&cups);
    cout << "GOTO" << endl;
    cout << -(cups[0].worldLocation.x-25) << " "<< -(cups[0].worldLocation.z) << " "<< cups[0].worldLocation.y << endl;

    #if DEBUG
    device.getVideo(rgbMat);
    draw_cups(&rgbMat, cups);
    imshow("rgb", rgbMat);
    waitKey(0);
    #endif
    // Main loop
    while (!finished) {
       // Check the clock tick
        e1 = cv::getTickCount();
        device.getVideo(rgbMat);
        device.getDepth(depthMat);

        if (showFinder) {
            detect_cups(&rgbMat, depthMat, rectCup, cameraInv, HT);
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

        if (showTracker) {
            accumlate_cups(&rgbMat, depthMat, rectCup, &cups, cameraInv, HT);
            average_cups(&cups);
            draw_cups(&rgbMat, cups);
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
            case 's':
                showFinder = showFinder? false: true;
                cout << "Cup detection : " << showFinder << endl;
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
            case 'n':
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

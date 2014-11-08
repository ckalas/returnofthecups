#include <iostream>
#include <highgui.h>
#include <ctime>
#include "cups.h"

#define DEBUG 1

Rect roi = Rect(Point(OFF_X,OFF_Y), Point(500,430));

/**
 * Stores the pixel locations of all the cups that are present in the frame.
 *
 * @param   rgbMat       the rotation vector (radians) (rotx,roty,rotz)
 * @param   cascade      the translation vector (pixels) (x,y,z) where z is read from depthMat 
 * @param   points       a pointer to a vector containing the location of all cups detected
 * @returns void
 */

// TODO: Take out the unnecessary pointer and pass the reference instead 
void accumlate_cups(Mat *rgbMat,  Mat depthMat, CascadeClassifier cascade, vector<Cup> *cups,
                                        Mat inverseCamera, Mat HT) {
    // Only consider a smaller region of the picture - increase speed and reduce false positives
    Mat slice = (*rgbMat)(roi).clone();
    Mat cameraCoords, fidCoords;
    std::vector<cv::Rect> matches;
    Mat gray;
    Point2f centre;
    cvtColor(slice, gray, CV_BGR2GRAY);
    equalizeHist(gray,gray);

    // Locate all the cups in the scene
    cascade.detectMultiScale(gray, matches, 1.3, 3,0|CV_HAAR_SCALE_IMAGE, Size(20, 30));

    // Add all the cups found across 10 frames
    for (size_t i = 0; i < matches.size(); i++) {
        // Create new cup with time stamp and pixel location
        Cup newCup;
        newCup.timestamp = get_time();
        // Take point at centre of cup region
        newCup.pixelLocation = Point2f((float)matches[i].x+matches[i].width/2 + OFF_X,
                         (float)matches[i].y + matches[i].height/2+OFF_Y);
        // Get depth
        newCup.depth = depthMat.at<unsigned short>(matches[i].y + matches[i].height/2+OFF_Y, 
                                                   matches[i].x+matches[i].width/2+OFF_X )/10.0;
        if (newCup.depth < 40 || newCup.depth > 150) {
            continue;
        }

        // Calculate distance from camera
        Mat imageCoords =(Mat_<double>(3,1)<<(OFF_X+matches[i].x+matches[i].width/2)*newCup.depth,
                                                 (OFF_Y+matches[i].y+matches[i].height/2)*newCup.depth,
                                                 newCup.depth);
        cameraCoords = inverseCamera * imageCoords;
        // Homogeneous transform to base fiducial
        Mat add = Mat::ones(1,1, CV_64F);
        cameraCoords.push_back(add);
        fidCoords = HT*cameraCoords;
        newCup.worldLocation = Point3f(fidCoords.at<double>(0),fidCoords.at<double>(1),
                                       fidCoords.at<double>(2));

        newCup.sorted = false;

        cups->push_back(newCup);
    }

}

/**
 * Reduces the vector containing cup points based on euclidean distance between points.
 *
 * @param   points  a pointer to a vector containing the location of all cups detected
 * @returns void
 */

void average_cups(vector<Cup> *cups) {

    for (auto it = cups->begin(); it != cups->end(); ++it) {
        for (auto jt = std::next(it); jt != cups->end(); ) {
            if (norm((*it).pixelLocation-(*jt).pixelLocation) <= 25) {
                jt = cups->erase(jt);
            }
            else {
                ++jt;
            }
        }
    }
}

void transpose_cup(Cup cup) {
    double x,xt,y,yt,z,zt;
    x = cup.worldLocation.x;
    y = cup.worldLocation.y;
    z = cup.worldLocation.z;
    cout << "Fiducial Coords" << endl;
    cout << "x: " << x << ", y: " << y << ", z: " << z << endl;
    xt = -(x-18);
    yt = -z;
    zt = y;
    cout << "Transposed Coords" << endl;
    cout << "x: " << xt << ", y: " << yt << ", z: " << zt << endl;
}

/**
 * Draws circles at the location of cup points
 *
 * @param rgbMat    the RGB frame to draw on
 * @param points    the locations of the cups
 * @returns  void
 */

void draw_cups(Mat *rgbMat, vector<Cup> cups) {
    for (size_t i = 0; i < cups.size(); i++) {
        circle(*rgbMat, cups[i].pixelLocation, 3, Scalar(0,0,255), 2);
    }
}

double get_time(void) {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    return now->tm_sec;
}

double elapsed_time(double previous) {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    return now->tm_sec-previous;
}


/**
 * Prints the FPS calculation on RGB frame
 *
 * @param   rgbMat  the current RGB frame
 * @param   fps     the current depth frame
 * @returns void
 */

void show_fps(Mat *rgbMat, int fps) {
    string text = string(std::to_string(fps) +" FPS");
    int fontFace = FONT_HERSHEY_DUPLEX;
    double fontScale = 1;
    int thickness = 2;
    putText(*rgbMat, text, Point(10,40), fontFace, fontScale, Scalar::all(255), thickness, 5);
}


/**
 * Prints the first three entries in a Mat
 *
 * @param   points  vector of points
 * @param   label   the label for print
 * @returns void
 */
void print_mat3(Mat points, string label) {
    cout << label << endl << points.at<double>(0)<< ", " << points.at<double>(1)<< ", " << points.at<double>(2)<< endl;
}
#include <iostream>
#include <highgui.h>
#include "cups.h"

#define DEBUG 0

Rect roi = Rect(Point(OFF_X,OFF_Y), Point(500,430));

/**
 * Stores the pixel locations of all the cups that are present in the frame.
 *
 * @param   rgbMat       the rotation vector (radians) (rotx,roty,rotz)
 * @param   cascade    the translation vector (pixels) (x,y,z) where z is read from depthMat 
 * @param    points        a pointer to a vector containing the location of all cups detected
 * @returns  void
 */

// TODO: Take out the unnecessary pointer and pass the reference instead 
void accumlate_cups(Mat *rgbMat, CascadeClassifier cascade, vector<Point2f> *points) {
    // Only consider a smaller region of the picture - increase speed and reduce false positives
    Mat slice = (*rgbMat)(roi).clone();
    std::vector<cv::Rect> matches;
    Mat gray;
    Point2f centre;
    cvtColor(slice, gray, CV_BGR2GRAY);
    equalizeHist(gray,gray);

    // Locate all the cups in the scene
    cascade.detectMultiScale(gray, matches, 1.3, 3,0|CV_HAAR_SCALE_IMAGE, Size(20, 30));

    // Add all the cups found across 10 frames
    for (size_t i = 0; i < matches.size(); i++) {
        // Take point at centre of cup region
        centre = Point2f((float)matches[i].x+matches[i].width/2 + OFF_X, (float)matches[i].y + matches[i].height/2+OFF_Y);
        points->push_back(centre);
    }

}

/**
 * Reduces the vector containing cup points based on euclidean distance between points.
 *
 * @param   points  a pointer to a vector containing the location of all cups detected
 * @returns  void
 */

void average_cups(vector<Point2f> *points) {

    for (auto it = points->begin(); it != points->end(); ++it) {
        for (auto jt = std::next(it); jt != points->end(); ) {
            if (norm(*it-*jt) <= 25) {
                jt = points->erase(jt);
            }
            else {
                ++jt;
            }
        }
    }
    cout << *points << endl;
}

/**
 * Draws circles at the location of cup points
 *
 * @param rgbMat    the RGB frame to draw on
 * @param points      the locations of the cups
 * @returns  void
 */

void draw_cups(Mat *rgbMat, vector<Point2f> points) {
    for (size_t i = 0; i < points.size(); i++) {
        circle(*rgbMat, points[i], 3, Scalar(0,0,255), 2);
    }
}

/**
 * Detect and LOCATE the cup in camera space and fiducial space
 *
 * @param   rgbMat                  the current RGB frame
 * @param   depthMat              the current depth frame
 * @param   cascade               the cascade classifier of the cup
 * @param   inverseCamera   the inverted intrinsics matrix
 * @param   HT                         the homogeneous transform matrix
 * @returns  void
 */

void detect_cups(Mat *rgbMat, Mat depthMat, CascadeClassifier cascade, Mat inverseCamera, Mat HT) {
    Mat slice = (*rgbMat)(roi).clone();
    std::vector<cv::Rect> matches;
    Mat gray, cameraCoords, fidCoords, cameraCoordsHomogeneous; // make this a vector of them ultimately pointer
    cvtColor(slice, gray, CV_BGR2GRAY);
    equalizeHist(gray,gray);

    cascade.detectMultiScale(gray, matches, 1.3, 3,0|CV_HAAR_SCALE_IMAGE, Size(20, 30));

    for (size_t i = 0; i < matches.size(); i++) {

        // Draw rectangle
        Point tl (matches[i].x+OFF_X, matches[i].y+OFF_Y);
        Point br (matches[i].x+matches[i].width+OFF_X, matches[i].y+matches[i].height+OFF_Y);
        rectangle(*rgbMat, tl ,br, Scalar( 0, 255, 255 ), +2, 4);
        // Get depth of cup  -- depth(Y,X)
        double depth = depthMat.at<unsigned short>(matches[i].y + matches[i].height/2+OFF_Y, 
                                                   matches[i].x+matches[i].width/2+OFF_X )/10.0;
        if (depth > 40 and depth < 150) {

            // Calculate distance from camera
            Mat imageCoords =(Mat_<double>(3,1)<<(OFF_X+matches[i].x+matches[i].width/2)*depth,
                                                 (OFF_Y+matches[i].y+matches[i].height/2)*depth,
                                                 depth);
            cameraCoords = inverseCamera * imageCoords;
            #if DEBUG
            cout << "Depth at cup " << i << " : " << depth << endl;
            cout << "Coords of cup " << i << " : " << OFF_X+matches[i].x+matches[i].width/2 <<
                                          "," <<OFF_Y+matches[i].y + matches[i].height/2<< endl;
            cout << "img" << endl << imageCoords << endl << "inv" << endl << inverseCamera << endl;
            #endif
            print_mat3(cameraCoords, "Camera Coords");
            Mat add = Mat::ones(1,1, CV_64F);
            cameraCoords.push_back(add);
            fidCoords = HT*cameraCoords;
            // Subtract offset for fiducial size AND an x offset (25)
            fidCoords.at<double>(0) = fidCoords.at<double>(0) - FID_DIM;
            fidCoords.at<double>(1) = fidCoords.at<double>(1) - FID_DIM;
            print_mat3(fidCoords, "FiducialCoords");
        }

    }
}

/**
 * Prints the FPS calculation on RGB frame
 *
 * @param   rgbMat  the current RGB frame
 * @param   fps         the current depth frame
 * @returns  void
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
 * @param   label    the label for print
 * @returns  void
 */
void print_mat3(Mat points, string label) {
    cout << label << endl << points.at<double>(0)<< ", " << points.at<double>(1)<< ", " << points.at<double>(2)<< endl;
}
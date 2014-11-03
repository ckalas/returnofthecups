#include <iostream>
#include <highgui.h>
#include "cups.h"

#define DEBUG 0

Rect roi = Rect(Point(OFF_X,OFF_Y), Point(500,430));

void accumlate_cups(Mat *rgbMat, CascadeClassifier cascade, vector<Point2f> *points) {
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

void draw_cups(Mat *rgbMat, vector<Point2f> points) {
    for (size_t i = 0; i < points.size(); i++) {
        circle(*rgbMat, points[i], 3, Scalar(0,0,255), 2);
    }
}

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

void show_fps(Mat *rgbMat, int fps) {
    string text = string(std::to_string(fps) +" FPS");
    int fontFace = FONT_HERSHEY_DUPLEX;
    double fontScale = 1;
    int thickness = 2;
    putText(*rgbMat, text, Point(10,40), fontFace, fontScale, Scalar::all(255), thickness, 5);
}

void print_mat3(Mat points, string label) {
    cout << label << endl << points.at<double>(0)<< ", " << points.at<double>(1)<< ", " << points.at<double>(2)<< endl;
}
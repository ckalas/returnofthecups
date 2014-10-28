#include <iostream>
#include <stdio.h>

#include <highgui.h>
#include "cups.h"

using namespace cv;
using namespace std;

void detect_cups(Mat *rgbMat, Mat depthMat, CascadeClassifier cascade, Mat inverseCamera) {

    std::vector<cv::Rect> matches;
    Mat gray, cameraCoords; // make this a vector of them ultimately pointer
    cvtColor(*rgbMat, gray, CV_BGR2GRAY);
    equalizeHist(gray,gray);

    cascade.detectMultiScale(gray, matches, 1.3, 3,0|CV_HAAR_SCALE_IMAGE, Size(20, 30));

    for (size_t i = 0; i < matches.size(); i++) {

        // Draw rectangle
        Point tl (matches[i].x, matches[i].y);
        Point br (matches[i].x+matches[i].width, matches[i].y+matches[i].height);
        rectangle(*rgbMat, tl ,br, Scalar( 0, 255, 255 ), +2, 4);
        // Get depth of cup  -- Depth(Y,X)
        double depth = depthMat.at<unsigned short>(matches[i].y + matches[i].height/2, matches[i].x+matches[i].width/2 )/10.0 ;

        if (depth > 40 and depth < 150) {
            cout << depth << " cm" << endl;
            // Calculate distance from camera
            Mat imageCoords = (Mat_<double>(3,1) << matches[i].x*depth, matches[i].y*depth, depth);
            cameraCoords = inverseCamera * imageCoords;
            cout << "Cup location: " << cameraCoords << endl;
        }

    }
}

int find_cups(Mat *gray, CascadeClassifier cascade, vector<Point2f>  *points) {
    
    std::vector<cv::Rect> matches;
    int newCups = 0;
    cascade.detectMultiScale(*gray, matches, 1.3, 3,0|CV_HAAR_SCALE_IMAGE, Size(20, 30));

    for (size_t i = 0; i < matches.size(); i++) {

        Point centre = Point(matches[i].x+matches[i].width/2,matches[i].y + matches[i].height/2);
        if  (points->size() > 0 && norm (centre-(*points)[i]) <= 5 ) continue;
        newCups++;
        points->push_back(centre);
    }

    return newCups;

/*                  
                    if( norm(point - points[1][i]) <= 5 ) {
                        addRemovePt = false;
                        continue;
                    }
                    if( addRemovePt && points[1].size() < (size_t)MAX_COUNT ) {
                    vector<Point2f> tmp;
                    tmp.push_back(point);
                    cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
                    points[1].push_back(tmp[0]);
                    addRemovePt = false;
                    }*/


}

void show_fps(Mat *rgbMat, int fps) {
    string text = string(std::to_string(fps) +" FPS");
    int fontFace = FONT_HERSHEY_DUPLEX;
    double fontScale = 1;
    int thickness = 2;
    putText(*rgbMat, text, Point(10,40), fontFace, fontScale, Scalar::all(255), thickness, 5);
}

#include <iostream>
#include <highgui.h>
#include <ctime>
#include "cups.h"

#define DEBUG 1



//Rect roi = Rect(Point(OFF_X, OFF_Y), Point(420, 350));

Rect roi = Rect(Point(OFF_X,OFF_Y), Point(500,430)); // original
//Rect roi = Rect(Point(OFF_X, OFF_Y), Point(420, 250)); // at 90 cm
//Rect roi = Rect(Point(OFF_X, OFF_Y), Point(420, 350));


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
        newCup.worldLocation = Point3f(fidCoords.at<double>(0)-8,fidCoords.at<double>(1),
                                       fidCoords.at<double>(2));
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

    for (auto it = cups->begin(); it != cups->end(); ) {
        for (auto jt = std::next(it); jt != cups->end(); ) {
            if (norm((*it).pixelLocation-(*jt).pixelLocation) <= 25) {
                std::swap(*it,*jt);
                jt = cups->erase(jt);
            }
            else {
                ++jt;
            }

        }

        if (elapsed_time((*it).timestamp) > 0) {
            it = cups->erase(it);
        }
        else {
            ++it;
        }
    }
}

void cup_info(vector<Cup> cups) {
    double x,xt,y,yt,z,zt;

    for (size_t i = 0; i < cups.size() ; i++) {   
        Cup cup = cups[i];
        x = cup.worldLocation.x;
        y = cup.worldLocation.y;
        z = cup.worldLocation.z;
        cout << "------------------------------------------" <<  endl << "Cup " << i << endl;
        cout << "Last updated " << elapsed_time(cup.timestamp) << " seconds ago" << endl;
        cout << "Fiducial Coords" << endl;
        cout << "x: " << x << ", y: " << y << ", z: " << z << endl;
        xt = -(x-18);
        yt = -(z+6);
        zt = y;
        cout << "Transposed Coords" << endl;
        cout << "x: " << xt << ", y: " << yt << ", z: " << zt << endl;
        cout << "-----------------------------------------" << endl;
    }
}


//x_init - initial x distance from base to cup  
//y_init - initial y distance from base to cup
Point2f cup_prediction(float t, Point2f p_0) {
    //float y_t = 0; //distance between arm and centre of table
    float RPM = 2;

    //offset the coord with table at the center
    Point2f offSet; //offset from fiducial to center of turn table
    offSet.x = 16.5;  //14;
    offSet.y = 16; //18.5;
    
    Point2f pointOnTable;
    pointOnTable.x = p_0.x + offSet.x;
    pointOnTable.y = p_0.y + offSet.y;

    Point2f moveArm;

    //theta = 0 corresponds to the +x_axis in rectangular coords
    moveArm.x = (sqrt(pow(pointOnTable.x, 2) + pow(pointOnTable.y, 2)) * 
        cos(atan(pointOnTable.y / pointOnTable.x) + (M_PI / 30) * t * RPM)) - offSet.x;
    moveArm.y = sqrt(pow(pointOnTable.x, 2) + pow(pointOnTable.y, 2)) * 
        sin(atan(pointOnTable.y / pointOnTable.x) + (M_PI / 30) * t * RPM) - offSet.y;
    return moveArm;
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

time_t get_time(void) {
    time_t t = time(0);   // get time now
    //struct tm * now = localtime( & t );
    return t;
}

double elapsed_time(time_t previous) {
    time_t t = time(0);   // get time now
    //struct tm * now = localtime( & t );
    return difftime(t, previous);
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

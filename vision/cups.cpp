#include <iostream>
#include <fstream>
#include <highgui.h>
#include <ctime>
#include "cups.h"

#define DEBUG 1



Rect roi = Rect(Point(OFF_X,OFF_Y), Point(450,430)); // original

/**
 * Stores the pixel locations of all the cups that are present in the frame.
 *
 * @param   rgbMat       the rotation vector (radians) (rotx,roty,rotz)
 * @param   cascade      the translation vector (pixels) (x,y,z) where z is read from depthMat 
 * @param   points       a pointer to a vector containing the location of all cups detected
 * @returns void
 */

// TODO: Take out the unnecessary pointer and pass the reference instead 
void accumlate_cups(Mat &rgbMat,  Mat depthMat, CascadeClassifier cascade, vector<Cup> *cups,
                                        Mat inverseCamera, Mat HT) {
    // Only consider a smaller region of the picture - increase speed and reduce false positives
    Mat slice = (rgbMat)(roi).clone();
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
        int size = cup_classify(depthMat, newCup.pixelLocation);

        // Threshold cup size values
        if (size <= 60 || size > 80 || (size >= 67 && size <= 72)) {
            continue;
        }

        // 1 -> large, 0 -> medium
        newCup.size = size > 72 ? 1 : 0;
        //cout << "cup type..." << newCup.size << "," << size << endl;
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

    // NOTE: Works best with motion of cups
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

        // Remove old cups
        if (elapsed_time((*it).timestamp) > 0) {
            it = cups->erase(it);
        }
        else {
            ++it;
        }
    }
}

/**
 * Reduces the vector containing cup points based on euclidean distance between points.
 *
 * @param   cups  a vector containing the information of current cups
 * @returns void
 */
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


/**
 * Predicts where the cup will be based on clockwise motion of a turntable.
 * Doesn't really work.
 *
 * @param   t    the time in the future to predict location for (seconds)
 * @param   p_0  current cup location
 * @returns void
 */
Point2f cup_prediction(float t, Point2f p_0) {
    
    //float y_t = 0; //distance between arm and centre of table
    float RPM = 2;

    Point2f offSet = Point2f(16.0, (20-4.5)); //offset from fiducial to center of turn table
    
    Point2f pointOnTable = Point2f(p_0.x - offSet.x, p_0.y - offSet.y);

    Point2f moveArm = p_0;

    
    //theta = 0 corresponds to the +x_axis in rectangular coords
    moveArm.x = sqrt(pow(pointOnTable.x, 2) + pow(pointOnTable.y, 2))* 
	(cos(atan(pointOnTable.y / pointOnTable.x) + cos((M_PI / 30) * t * RPM))) + offSet.x;
    
    moveArm.y = sqrt(pow(pointOnTable.x, 2) + pow(pointOnTable.y, 2)) * 
	(sin(atan(pointOnTable.y / pointOnTable.x) + sin((M_PI / 30) * t * RPM))) + offSet.y;

	#if DEBUG
	cerr << "radius " << moveArm << endl
	 << "point on table: " << pointOnTable << endl
	 << "original cup position: " << p_0 << endl;
     #endif
	
    return moveArm;
}

/**
 * Draws circles at the location of cup points
 *
 * @param rgbMat    the RGB frame to draw on
 * @param points    the locations of the cups
 * @returns  void
 */

void draw_cups(Mat &rgbMat, vector<Cup> cups) {
    for (size_t i = 0; i < cups.size(); i++) {
        circle(rgbMat, cups[i].pixelLocation, 3, Scalar(0,0,255), 2);
    }
}

time_t get_time(void) {
    time_t t = time(0);   // get time now
    return t;
}

double elapsed_time(time_t previous) {
    time_t t = time(0);   // get time now
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

/**
 * Classifies a cup as either large or medium based on changing depth. The results are
 * subject to thresholding in the function accumulate_cups().
 *
 * @param   depth  the current depth image
 * @param   centre the centre of the current cup
 * @returns void
 */
int cup_classify(Mat depth, Point2f centre) {
    int img_midh = centre.y; //y
    int img_midw = centre.x; //x
    int diff;
    bool edge = false;
    int i = 0;

    Mat depthf (Size(depth.cols,depth.rows),CV_8UC1);
    depth.convertTo(depthf, CV_8UC1, 255.0/2048.0);
    //init the next_depth
   int next_depth = depth.at<unsigned short>(img_midh, img_midw), cur_depth;

    while (!edge) {	
        cur_depth = next_depth;
        next_depth = depth.at<unsigned short>(img_midh + i, img_midw);

        if ((diff = abs(cur_depth - next_depth)) >= 3.5) {
            edge = true;
        }
        //if at the top of the image and haven't found edge
        if ((img_midh + i) == (depth.rows - 1)) {
            return 0;
        }
        i++;
    }

    int bottom = img_midh + i;
    i = 0;
    //init the next_depth
    edge = false;
    next_depth = depth.at<unsigned short>(img_midh, img_midw);

    while (!edge) {	
        cur_depth = next_depth;
        next_depth = depth.at<unsigned short>(img_midh + i, img_midw);
        if ((diff = abs(cur_depth - next_depth)) >= 10) {
            edge = true;
        }
        //if at the bottom of the image and haven't found edge
        if ((img_midh + i) == 0) {
            return 0;
        }
        i--;
    }
    int top = img_midh + i;
    
    int height = abs(top - bottom);

    return height;
}

/**
 * Reads the order file and stores the data away with some bit shifting.
 *
 * Each order is condensed into a single byte as follows:
 * MSB -> LSB
 * (num_sugar << 6 ) | (num_tea << 4) | (num_coffee << 2) | size
 *       2                2                  2                1     (bits)
 *
 * @param   points  vector of points
 * @param   label   the label for print
 * @returns void
 */
vector<uint8_t> take_order(void) {
    ifstream infile("orders.txt");
    int numOrders, cs, nc, nt, ns, blank;

     // First line contains the number of orders
    infile >> numOrders;
    vector<uint8_t> orders;

    // Read lines cup size, ncoffee, ntea, nsugar, - , -
    while (infile >> cs >> nc >> nt >> ns >> blank >> blank) {
	cout << "Cup size:" << cs << "," << "C/T/S: " << nc << nt << ns << endl; 
        orders.push_back((ns << 6) | (nt << 4) | (nc << 2) | (cs-1));
    }

    cout << "Orders taken: " << orders.size() << endl;
    return orders;

}

/**
 * Checks if there the current order is for the cup passed in - based on size.
 *
 * Does simple decoding by shifting bits and bit masking.
 *
 * @param   cupsize 1 for large, 0 for medium
 * @param   orders  a vector containing the order bytes
 * @returns void
 */
bool print_next_order(int cupsize, vector<uint8_t> *orders) {
        // Su Su T T C C S
        int data = orders->at(0);
        if (((data)&1) == cupsize) {
            string size = cupsize == 1 ? "Large" : "Medium";
            int nc = (data >> 2) & 3;
            int nt = (data >> 4) & 3;
            int ns = (data >> 6) & 3;
            cout << "-------------------------" << endl << size << " Cup Order" << endl << "Coffee: " << nc << endl 
                    << "Tea: " << nt <<endl << "Sugar: " << ns << endl << "-----------------------" << endl;

            orders->erase(orders->begin());
            cout << "Orders left: " << orders->size() << endl;
            return true;
        }
        return false;

}


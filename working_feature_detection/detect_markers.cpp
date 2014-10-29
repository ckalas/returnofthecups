#include "detect_markers.h"
#include <iostream>
#include <opencv2/opencv.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

using namespace std;
using namespace cv;

Mat img_dispenser = imread("sift/auto_dispenser.png", IMREAD_GRAYSCALE);
Mat img_fiducial = imread("sift/id7.png", IMREAD_GRAYSCALE);
Mat img_scene;

void checkFiducial()
{

}

void checkSIFT(Mat src)
{
    Mat img_object = img_dispenser;
    img_scene = src;


    // -- Step 1: Detect the keypoints using SURF Detector
    int minFeatures = 100;

    SurfFeatureDetector detector( minFeatures);

    vector<KeyPoint> keypoints_object, keypoints_scene;

    detector.detect(img_object, keypoints_object);
    detector.detect(img_scene, keypoints_scene);

    // -- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    Mat descriptors_object, descriptors_scene;

    extractor.compute( img_object, keypoints_object, descriptors_object );
    extractor.compute( img_scene, keypoints_scene, descriptors_scene);

    // -- Step 3: Matching descriptor vector using FLANN matcher
    FlannBasedMatcher matcher;
    vector<DMatch> matches;
    matcher.match( descriptors_object, descriptors_scene, matches);

    double max_dist = 0; double min_dist = 100;

    // Quick calculation of max and min distance between keypoints
    for(int i=0; i< descriptors_object.rows; i++) {
	double dist = matches[i].distance;
	if(dist < min_dist) min_dist = dist;
	if(dist > max_dist) max_dist = dist;
    }
    /*
    cout << "-- Max dist : " << max_dist << endl;
    cout << "-- Min dist : " << min_dist << endl;
    */
    // -- Draw only "good" matches (i.e. whose distance is less than 3*min_dist
    vector<DMatch> good_matches;

    for(int i = 0; i<descriptors_object.rows; i++) {
	if( matches[i].distance < 3*min_dist)
	    good_matches.push_back( matches[i] );
    }

    Mat img_matches;
    drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
		 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    vector<Point2f> obj;
    vector<Point2f> scene;

    for (int i = 0; i< good_matches.size(); i++) {
	// -- Get the keypoints from the good matches
	obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
	scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }

    Mat H = findHomography( obj, scene, CV_RANSAC );

    // -- Get the corners from the iamge-1 ( the object to be "detected" )
    vector<Point2f> obj_corners(4);
    obj_corners[0] = Point(0,0); obj_corners[1] = Point(img_object.cols, 0);
    obj_corners[2] = Point(img_object.cols, img_object.rows);
    obj_corners[3] = Point(0, img_object.rows);
    vector<Point2f> scene_corners(4);

    perspectiveTransform(obj_corners, scene_corners, H);

    // -- Draw lines between the corners (the mapped object in the scene)
    line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0,255,0), 4);
    line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar(0,255,0), 4);
    line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar(0,255,0), 4);
    line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar(0,255,0), 4);

    imshow("Matches", img_matches);
}

#include "detect_markers.h"
#include <iostream>
#include <opencv2/opencv.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <string>
using namespace std;
using namespace cv;


Mat img_dispenser = imread("sift/auto_dispenser.png", IMREAD_GRAYSCALE);
Mat img_fiducial = imread("sift/id7.png", IMREAD_GRAYSCALE);
Mat img_scene;

void cameraPoseFromHomography(const Mat& H, Mat& pose)
{
    pose = Mat::eye(3, 4, CV_32FC1);      // 3x4 matrix, the camera pose
    float norm1 = (float)norm(H.col(0));  
    float norm2 = (float)norm(H.col(1));  
    float tnorm = (norm1 + norm2) / 2.0f; // Normalization value

    Mat p1 = H.col(0);       // Pointer to first column of H
    Mat p2 = pose.col(0);    // Pointer to first column of pose (empty)

    normalize(p1, p2);   // Normalize the rotation, and copies the column to pose

    p1 = H.col(1);           // Pointer to second column of H
    p2 = pose.col(1);        // Pointer to second column of pose (empty)

    normalize(p1, p2);   // Normalize the rotation and copies the column to pose

    p1 = pose.col(0);
    p2 = pose.col(1);

    Mat p3 = p1.cross(p2);   // Computes the cross-product of p1 and p2
    cout << "p3: " << p3 << endl;
    Mat c2 = pose.col(2);    // Pointer to third column of pose
    p3.copyTo(c2);       // Third column is the crossproduct of columns one and two

    pose.col(3) = H.col(2) / tnorm;  //vector t [R|t] is the last column of pose
}


// input the video and the string location of the image trying to complete with
void checkSIFT(Mat src, string objectString, Mat intrinsics, Mat distortion)
{
    Mat img_object = imread(objectString, 0);
    Mat img_scene;
    cvtColor(src, img_scene, CV_BGR2GRAY);


    // -- Step 1: Detect the keypoints using SURF Detector
    int minFeatures = 500; //500;
    
    SiftFeatureDetector detector( minFeatures );
    //OrbFeatureDetector detector( minFeatures );
    //SurfFeatureDetector detector( minFeatures);

    vector<KeyPoint> keypoints_object, keypoints_scene;

    detector.detect(img_object, keypoints_object);
    detector.detect(img_scene, keypoints_scene);

    // -- Step 2: Calculate descriptors (feature vectors)
    //SurfDescriptorExtractor extractor;
    //OrbDescriptorExtractor extractor;
    SiftDescriptorExtractor extractor;

    Mat descriptors_object, descriptors_scene;

    extractor.compute( img_object, keypoints_object, descriptors_object );
    extractor.compute( img_scene, keypoints_scene, descriptors_scene);

    // -- Step 3: Matching descriptor vector using FLANN matcher
    FlannBasedMatcher matcher;
    //BFMatcher matcher;
    vector<DMatch> matches;
    matcher.match( descriptors_object, descriptors_scene, matches);

    double max_dist = 0; double min_dist = 750;// 750;

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
	if( matches[i].distance < 10*min_dist)
	    good_matches.push_back( matches[i] );
    }

    Mat img_matches;
    /*
    drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
		 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    */
    vector<Point2f> obj;
    vector<Point2f> scene;

    for (int i = 0; i< good_matches.size(); i++) {
	// -- Get the keypoints from the good matches
	obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
	scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }

    /*
    cout << "obj: " << obj.size() << endl;
    cout << "scene: " << scene.size() << endl << endl;
    */
    if (obj.size() <= 3 || scene.size() <= 3) {
	//cout << "This when it is suppose to crash" << endl;
	return;
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


    Mat rvec = Mat(Size(3,1), CV_64F);
    Mat tvec = Mat(Size(3,1), CV_64F);
    vector<Point3f> markerPoints;

    markerPoints.push_back( Point3f( 0.0, 0.0, 0.0 ) );
    markerPoints.push_back( Point3f( 1.0, 0.0, 0.0 ) );
    markerPoints.push_back( Point3f( 1.0, 1.0, 0.0 ) );
    markerPoints.push_back( Point3f( 0.0, 1.0, 0.0 ) );
 
    Mat pose;   

    solvePnP( Mat(markerPoints), Mat(scene_corners), intrinsics, distortion,
	      rvec, tvec, false);
    //    cameraPoseFromHomography(H, pose);
    //cout << "Pose: " << endl << pose << endl;
    cout << "rvec: " << rvec << endl;
    cout << "tvec: " << tvec << endl << endl;
}

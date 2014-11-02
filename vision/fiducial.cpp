
#include "fiducial.h"
#include <algorithm>


bool check_sift(Mat src, string objectString, Mat intrinsics, Mat distortion,
    int minFeat, int minDist, int multi,   Mat &HT ) {
    Mat img_object = imread(objectString, 0);
    Mat img_scene = src;

    // Step 1: Detect the keypoints using SURF Detector
    int minFeatures = minFeat; //500; //500;
    
    SiftFeatureDetector detector( minFeatures );

    vector<KeyPoint> keypoints_object, keypoints_scene;

    detector.detect(img_object, keypoints_object);
    detector.detect(img_scene, keypoints_scene);

    // Step 2: Calculate descriptors (feature vectors)

    SiftDescriptorExtractor extractor;

    Mat descriptors_object, descriptors_scene;

    extractor.compute( img_object, keypoints_object, descriptors_object );
    extractor.compute( img_scene, keypoints_scene, descriptors_scene);

    // Step 3: Matching descriptor vector using FLANN matcher
    FlannBasedMatcher matcher;
    vector<DMatch> matches;
    matcher.match( descriptors_object, descriptors_scene, matches);

    double max_dist = 0; double min_dist = minDist;

    // Quick calculation of max and min distance between keypoints
    for(int i=0; i< descriptors_object.rows; i++) {
        double dist = matches[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    // Draw only "good" matches (i.e. whose distance is less than 3*min_dist
    vector<DMatch> good_matches;

    for(int i = 0; i<descriptors_object.rows; i++) {
        if( matches[i].distance < multi*min_dist) {
            good_matches.push_back( matches[i] );
        }
    }

    Mat img_matches;
    drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
         good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
         vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    vector<Point2f> obj;
    vector<Point2f> scene;

    for (size_t i = 0; i< good_matches.size(); i++) {
    // Get the keypoints from the good matches
    obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
    scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }


    if (obj.size() <= 3 || scene.size() <= 3) {
        return false;
    }

    Mat H = findHomography( obj, scene, CV_RANSAC );

    // Get the corners from the iamge-1 ( the object to be "detected" )
    vector<Point2f> obj_corners(4);
    obj_corners[0] = Point(0,0); obj_corners[1] = Point(img_object.cols, 0);
    obj_corners[2] = Point(img_object.cols, img_object.rows);
    obj_corners[3] = Point(0, img_object.rows);
    vector<Point2f> scene_corners(4);

    perspectiveTransform(obj_corners, scene_corners, H);
    
    // Draw lines between the corners (the mapped object in the scene)
    line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0,255,0), 4);
    line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar(0,255,0), 4);
    line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar(0,255,0), 4);
    line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar(0,255,0), 4);

    imshow("matches", img_matches);
    waitKey(0);
    destroyWindow("matches");

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

    cout << "rvec: " << rvec << endl;
    cout << rvec.at<double>(0)*(180/(22/7)) << ":" << rvec.at<double>(1)*(180/(22/7)) <<":" << rvec.at<double>(2)*(180/(22/7)) << endl;
    cout << "tvec: " << tvec << endl;

    // check if the solve PnP is valid
    double checkDepth = tvec.at<double>(2)*-SCALE;
    if  (checkDepth > 100 || checkDepth <= 50 || isnan(checkDepth)) {
        return false;
    }

    HT = reconfigure_reference(rvec,tvec);
    return true;
}


Mat reconfigure_reference(Mat rvec, Mat tvec) {
    //NOTE: x,y & z are in units of cm  
    float x = tvec.at<double>(0), y = tvec.at<double>(1), z = tvec.at<double>(2)*SCALE;
    float rotx = rvec.at<double>(0), roty = rvec.at<double>(1), rotz = rvec.at<double>(2);

    Mat HT;
    //  x = (x - FID_WIDTH);
   // y = (y-FID_WIDTH);
    //offset rotation on x copied from fiducial.py -> validity of this may 
    //need checking
    //rotx = (rotx - 0.27);

    HT = (Mat_<float>(4, 4) << 
	    cos(rotz) * cos(roty), 
	    cos(rotz) * sin(roty) * sin(rotx) - sin(rotz) * cos(rotx), 
	    cos(rotz) * sin(roty) * cos(rotx) + sin(rotz) * sin(rotx),
	    x,

	    sin(rotz) * cos(roty),
	    sin(rotz) * sin(roty) * sin(rotx) + cos(rotz) * cos(rotx),
	    sin(rotz) * sin(roty) * cos(rotx) - cos(rotz) * sin(rotx),
	    y,

	    sin(roty),
	    cos(roty) * sin(rotx),
	    cos(roty) * cos(rotx),
	    z,
	    
	    0, 0, 0, 1);

    return HT;
}

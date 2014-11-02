
#include "fiducial.h"

#define SCALE 4.8623

bool find_fid(Mat *rgbMat, Mat *cameraMtx, Mat *distCoeffs, Mat *HT) {
    try {
	//if (sift_feature(rgbMat, cameraMtx, distCoeffs, HT)) {
	if (sift_feature(rgbMat, cameraMtx, distCoeffs, HT)) {
	    return true;
	} else {
	    return false;
	}
    }
    catch (int e) {
	   cout << "cv error in fiducial.find" << endl;
	   return false;
    }
}


bool sift_feature(Mat *rgbMat, Mat *cameraMat, Mat *distCoeffs, Mat *HT) {
    
    //id7 is fiducial marker
    Mat id7 = imread("id7.png", CV_LOAD_IMAGE_GRAYSCALE); 
    
    //convert rgbMat to grayscale, as grayIm
    Mat grayIm;
    cvtColor(*rgbMat, grayIm, CV_BGR2GRAY);

    SiftFeatureDetector detector; //use default constructor for this class

    std::vector<KeyPoint> keypoints_id7, keypoints_grayIm;
    detector.detect(id7, keypoints_id7);
    detector.detect(grayIm, keypoints_grayIm);

    if ( keypoints_id7.empty() || keypoints_grayIm.empty() ) {
        return false;
    }

    // Calculate descriptors for key points
    SiftDescriptorExtractor extractor;

    Mat descriptors_id7, descriptors_grayIm;

    extractor.compute(id7, keypoints_id7, descriptors_id7);
    extractor.compute(grayIm, keypoints_grayIm, descriptors_grayIm);
    
    // Descriptor Matching using flann matcher
  
    FlannBasedMatcher matcher; 
    std::vector<DMatch> matches;

    matcher.match(descriptors_id7, descriptors_grayIm, matches);

    /* Get the min distance of keypoints -> NOTE distance here is a
      measure of "goodness" of matches.... Apparently */

    double min_dist = 1000, max_dist = 0;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_id7.rows; i++ ) { 
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    // Save good matches and corresponding keypoints
    std::vector<DMatch> good_matches;
    std::vector<Point2f> goodKp_id7;
    std::vector<Point2f> goodKp_grayIm;

    
    int goodMatchCount = 0;
    std::vector<char> mask;

    Mat outIm;
    drawMatches(id7, keypoints_id7, grayIm, keypoints_grayIm, matches, outIm, CV_RGB(0, 255, 0), CV_RGB(0, 0, 255), mask, 0);
    imshow("matches", outIm);
    waitKey(0);
    
    for (int i = 0; i < descriptors_id7.rows; i++) {
	if (matches[i].distance < min_dist * 3 /*&& matches[i].distance > max_dist*0.35 */ ){
	    good_matches.push_back(matches[i]);
	    goodKp_id7.push_back(keypoints_id7[matches[i].trainIdx].pt);
	    goodKp_grayIm.push_back(keypoints_grayIm[matches[i].queryIdx].pt);
	    //mask.push_back(1);
	    goodMatchCount++;
	} else {
	   // mask.push_back(0);
	}
    }
    

    //exit if there isn't a minimum number of good matches
    if (goodMatchCount < MIN_MATCH_COUNT) {
	return false;
    }

    cout << goodKp_id7 << endl;
    Mat homoMat = findHomography(goodKp_id7, goodKp_grayIm, CV_RANSAC, 5);

    //Get the corners from the train image id7, and grayIm
    std::vector<Point2f> id7_corners, grayIm_corners;
    id7_corners.push_back(Point(0, 0));
    id7_corners.push_back(Point(id7.cols, 0));
    id7_corners.push_back(Point(id7.cols, id7.rows));
    id7_corners.push_back(Point(0, id7.rows));

    cout << "homoMat: " << homoMat << endl;

    perspectiveTransform(id7_corners, grayIm_corners, homoMat);

    cout << grayIm_corners << endl;
    // Draw the lines around object in grayIm 
    line(grayIm, grayIm_corners[0], grayIm_corners[1], Scalar(255, 255,255), 4);
    line(grayIm, grayIm_corners[1], grayIm_corners[2], Scalar(255, 255, 255), 4);
    line(grayIm, grayIm_corners[2], grayIm_corners[3], Scalar(255, 255, 255), 4);
    line(grayIm, grayIm_corners[3], grayIm_corners[0], Scalar(255, 255, 255), 4); 
    
    imshow("fid", grayIm);
    waitKey(0);
    cout << grayIm_corners << endl;
    // Show fiducial was found    
    Mat rvec, tvec;
    //cout <<goodKp_id7 << endl << goodKp_grayIm << endl;
    solvePnP(goodKp_id7, goodKp_grayIm, *cameraMat, *distCoeffs, rvec, tvec); 
    
    reconfigure_reference(&tvec, &rvec, HT); 

    return true;
} 

bool check_sift(Mat src, string objectString, Mat intrinsics, Mat distortion,
    int minFeat, int minDist, int multi  ) {
    Mat img_object = imread(objectString, 0);
    Mat img_scene = src;

    // -- Step 1: Detect the keypoints using SURF Detector
    int minFeatures = minFeat; //500; //500;
    
    SiftFeatureDetector detector( minFeatures );

    vector<KeyPoint> keypoints_object, keypoints_scene;

    detector.detect(img_object, keypoints_object);
    detector.detect(img_scene, keypoints_scene);

    // -- Step 2: Calculate descriptors (feature vectors)

    SiftDescriptorExtractor extractor;

    Mat descriptors_object, descriptors_scene;

    extractor.compute( img_object, keypoints_object, descriptors_object );
    extractor.compute( img_scene, keypoints_scene, descriptors_scene);

    // -- Step 3: Matching descriptor vector using FLANN matcher
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

    // -- Draw only "good" matches (i.e. whose distance is less than 3*min_dist
    vector<DMatch> good_matches;

    for(int i = 0; i<descriptors_object.rows; i++) {
    if( matches[i].distance < multi*min_dist)
        good_matches.push_back( matches[i] );
    }

    Mat img_matches;
    drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
         good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
         vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    vector<Point2f> obj;
    vector<Point2f> scene;

    for (size_t i = 0; i< good_matches.size(); i++) {
    // -- Get the keypoints from the good matches
    obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
    scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
    }


    if (obj.size() <= 3 || scene.size() <= 3) {
        return false;
    }

    
    Mat H = findHomography( obj, scene, CV_RANSAC );

    // -- Get the corners from the iamge-1 ( the object to be "detected" )
    vector<Point2f> obj_corners(4);
    obj_corners[0] = Point(0,0); obj_corners[1] = Point(img_object.cols, 0);
    obj_corners[2] = Point(img_object.cols, img_object.rows);
    obj_corners[3] = Point(0, img_object.rows);
    vector<Point2f> scene_corners(4);

    perspectiveTransform(obj_corners, scene_corners, H);

    cout << "perspectiveTransform: " << scene_corners << endl;
    
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

    cout << "rvec: " << rvec << endl;
    cout << "tvec: " << tvec << endl << endl;

    // check if the solve PnP is valid
    double checkDepth = tvec.at<double>(2)*SCALE;
    if  (checkDepth > 100 || checkDepth <= 0) {
        cout << "Invalid depth, retrying find fiducial" << endl;
        return false;
    }

    return true;
}


void reconfigure_reference(Mat *rvec, Mat *tvec, Mat *HT) {
    //NOTE: x,y & z are in units of cm  
    float x = tvec->at<double>(0), y = tvec->at<double>(1), z = tvec->at<double>(2);
    float rotx = rvec->at<double>(0), roty = rvec->at<double>(1), rotz = rvec->at<double>(2);


    x = (x - FID_WIDTH/2);
    
    //offset rotation on x copied from fiducial.py -> validity of this may 
    //need checking
    rotx = (rotx - 0.27);

    //maths, nothing custom
    *HT = (Mat_<float>(4, 4) << 
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

    cout <<"FID at: " << endl << HT << endl;
}

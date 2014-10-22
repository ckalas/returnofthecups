//please include id7.png in the directory the program is executed

#include "fiducial.h"


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
    //std::vector<char> mask;

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
    cout << mask << endl;

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

//please include id7.png in the directory the program is executed

#include "fiducial.h"

bool find_fid(Mat *rgbMat, Mat *depthMat, Mat *inverseCamera) {
    try {
	if (siftFeature(img, mtx, dist)) {
	    return true;
	} else {
	    return false;
	}
    }
    catch {
	cout << "cv error in fiducial.find";
	return false;
    }
}

bool siftFeature(Mat *rgbMat, Mat *cameraMat, Mat *distCoeffs) {
    
    //id7 is fiducial marker
    Mat id7 = imread("./id7.png", CV_LOAD_IMAGE_GRAYSCALE); 
    
    //convert rgbMat to grayscale, as grayIm
    Mat grayIm;
    cvtcolor(*rgbMat, grayIm, CV_BGR2GRAY);

    // ----- Detect the keypoints using Sift Detector
    SiftFeatureDetector detector; //use default constructor for this class

    std::vector<KeyPoint> keypoints_id7, keypoints_grayIm;
    detector.detect(id7, keypoints_id7);
    detector.detect(grayIm, keypoints_grayIm);

    //neccessary??? just copied from Andy's fiducial.py code, I would expect
    //and exception if there are no keypoint later on though
    if ( (keypoints_id7 == NULL) || (keypoints_grayIm == NULL) ) {
	return false;
    }

    // ----- Calculate descriptors for key points
    SiftDescriptorExtractor extractor;

    Mat descriptors_id7, descriptors_grayIm;

    extractor.compute(id7, keypoints_id7, descriptors_id7);
    extractor.compute(grayIm, keypoints_grayIm, descriptors_graIm);
    
    // ----- Descriptor Matching using flann matcher
    FlannBasedMatcher matcher; 
    std::vector<DMatch> matches;

    matcher.match(descriptors_id7, descriptors_grayIm, matches);

    // ----- get the min distance of keypoints -> NOTE distance here is a
    // measure of "goodness" of matches.... Apparently
    double dist_min = 100000; //set dist_min to a large value initially

    for (int i = 0; i < descriptors_id7.rows; i++) {
	double dist = mathes[i].distance;
	if (dist < min_dist) min_dist = dist;
    }

    // ----- save good matches and corresponding keypoints
    std::vector<Dmatch> good_matches;
    std::vector<Point2f> goodKp_id7;
    std::vector<Point2f> goodKp_grayIm;
    int goodMatchCount = 0;
    
    for (int i = 0; i < descriptors_id7.rows; i++) {
	if (matches[i].distance < 3*min_dist){
	    good_matches.push_back(matches[i]); //push_back appends
	    goodKp_id7.push_back(keypoints_id7[matches[i].trainIdx].pt);
	    goodKp_grayIm.push_back(keypoints_grayIm[matches[i].queryIdx].pt);
	    goodMatchCount++;
	}
    }
    
    //exit if there isn't a minimum number of good matches
    if (goodMatchCount < MIN_MATCH_COUNT) {
	return false;
    }
    Mat homoMat = findHomography(goodKp_id7, goodKp_grayIm, CV_RANSAC); 

    // ----- get the corners from the train image id7, and grayIm
    std::vector<Point2f> id7_corners, grayIm_corners;

    id7_corners[0] = cvPoint(0, 0);
    id7_corners[1] = cvPoint(id7.cols, 0);
    id7_corners[2] = cvPoint(id7.cols, id7.rows);
    id7_corners[3] = cvPoint(0, id7.rows);

    perspectiveTransform(id7_corners, grayIm_corners, homoMat);

    //Draw the lines around object in grayIm 
    line(rgbMat, grayIm_corners[0], grayIm_corners[1], Scalar(0, 255, 0), 4);
    line(rgbMat, grayIm_corners[1], grayIm_corners[2], Scalar(0, 255, 0), 4);
    line(rgbMat, grayIm_corners[2], grayIm_corners[3], Scalar(0, 255, 0), 4);
    line(rgbMat, grayIm_corners[3], grayIm_corners[0], Scalar(0, 255, 0), 4);
    
    //show fiducial was found
    imshow("Fiducial found", rgbMat);

    waitKey(0);
    
    Mat rvec;
    Mat tvec;

    solvePnP(goodKp_id7, goodKp_grayIm, *cameraMat, *distCoeffs, rvec, tvec); 
    
    //reconfigure_reference(tvec, rvec); 

    return true;
} 

void reconfigure_reference(Mat rvec, Mat tvec, Mat *Homogeneous_Transform) {
    //NOTE: x,y & z are in units of cm  
    float x = tvec[0], y = tvec[1], z = tvec[2];
    float rotx = rvec[0], roty = rvec[1], rotz = rvec[2];

    x = (x - FID_WIDTH/2);
    
    //offset rotation on x copied from fiducial.py -> validity of this may 
    //need checking
    rotx = (rotx - 0.27);

    //maths, nothing custom
    *Homogeneous_Transform = (Mat_<float>(4, 4) << 
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

    cout << Homogeneous_Transform;
}

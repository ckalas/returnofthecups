#include "fiducial.h"

#define DEBUG 0

/**
 * Returns true when a valid fiducial marker has been found.
 * Note that valid here means 50 < depth < 100 and 0 < rot(x,y,z) < 60.
 * It appears to be taking the centre of the marker  as the location of it.
 *
 * @param   src         the current rgb frame 
 * @param   depthMat    the current depth frame 
 * @param   filename    the relative filename of the fiducial marker
 * @param   intrinsics  the matrix representing camera intrinsic params
 * @param   distortion  the matrix representing camera distortion params
 * @param   minFeat     threshold value for sift detector
 * @param   minDist     threshold value for sift detector
 * @param   &HT         a matrix reference to store the homogeneous transform
 * @returns             true if a valid marker has been found
 */

bool check_sift(Mat src, Mat depthMat, string filename, Mat intrinsics, Mat distortion,
    int minFeat, int minDist, int multi, Mat &HT, vector<double> *tvec_r1 ) {
    Mat img_object = imread(filename, 0);
    Mat img_scene = src;

    // id7.png is always perpendicular to ground so expect low rotation
    int max_angle = filename == "id7.png" ? 50 : 180;

    // Step 1: Detect the keypoints using SIFT Detector
    int minFeatures = minFeat;
    
    SiftFeatureDetector detector(minFeatures);

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

    // Get the corners from the image ( the object to be "detected" )
    vector<Point2f> obj_corners(4);
    obj_corners[0] = Point(0,0); obj_corners[1] = Point(img_object.cols, 0);
    obj_corners[2] = Point(img_object.cols, img_object.rows);
    obj_corners[3] = Point(0, img_object.rows);
    vector<Point2f> scene_corners(4);

    perspectiveTransform(obj_corners, scene_corners, H);
    
    #if DEBUG
    // Draw lines between the corners (the mapped object in the scene)
    line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0), scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0,255,0), 4);
    line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0), scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0,255,0), 4);
    line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0), scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0,255,0), 4);
    line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0), scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0,255,0), 4);

    imshow("matches", img_matches);
    waitKey(0);
    destroyWindow("matches");
    #endif

    Mat rvec = Mat(Size(3,1), CV_64F);
    Mat tvec = Mat(Size(3,1), CV_64F);
    vector<Point3f> markerPoints;

    /*
     *   Corner locations
     *
     *   (0)   (1)
     *   (3)   (2)
     *
     *   Note that here the real life size of the marker is specified as the 3D point.
     */

    markerPoints.push_back(Point3f(0.0, 0.0, 0.0));
    markerPoints.push_back(Point3f(FID_SIZE, 0.0, 0.0));
    markerPoints.push_back(Point3f(FID_SIZE, FID_SIZE, 0.0));
    markerPoints.push_back(Point3f(0.0, FID_SIZE, 0.0));
 
    solvePnP(Mat(markerPoints), Mat(scene_corners), intrinsics, distortion,rvec, tvec, false, CV_P3P);
    // Use depth map to get accurate depth depth(y,x)
    double depth = depthMat.at<unsigned short>(scene_corners[0].y+FID_PIX, 
                   scene_corners[0].x+FID_PIX)/10.0;
    // check if the solve PnP is valid
    double rotx= abs(rvec.at<double>(0)*(180/M_PI));
    double rotz = abs(rvec.at<double>(2)*(180/M_PI));

    #if DEBUG
    destroyWindow("depth fid");
    cout << "Fiducial Depth: " << depth << endl;
    cout << "Fiducial Rotation: " << rvec.at<double>(0)*(180/M_PI) << ":" 
         << rvec.at<double>(1)*(180/M_PI) <<":" << rvec.at<double>(2)*(180/M_PI)<< endl;
    cout << "tvec: " << tvec << endl;
    cout << "rvec: " << rvec << endl;
    #endif

    if  (std::isnan(depth) || depth > 120 || depth <= 50 || rotx > max_angle || rotx < 1 
         || rotz > max_angle) {
        return false;
    }
    // Compute the homogeneous transform
    tvec.at<double>(2) = -depth;
    if (filename == "id7.png") {
        HT = reconfigure_reference(rvec,tvec);
    }
    if( filename == "id11.png" ) {
        tvec.at<double>(2)= -tvec.at<double>(2); 
        Mat temp = Mat(tvec);
        cout << "tvec: " << tvec << endl;
        double xt, yt, zt;
        Mat add = Mat::ones(1,1, CV_64F);
        Mat temp2;
        (temp).push_back(add);
        temp2 = HT*temp;
        xt = -(temp2.at<double>(0)-18);
        yt = (temp2.at<double>(2)+20);
        zt = temp2.at<double>(1);
        (*tvec_r1)[0] = xt;
        (*tvec_r1)[1] = yt;
        (*tvec_r1)[2] = zt;

    }
    return true;
}

/**
 * Returns the homogeneous transform given rotation and translation vectors
 * Note: This keeps the same frame as the camera as follows:
 *          +x = right of camera (camera perspective)
 *          +y = up
 *          +z = towards object
 *
 * IMPORTANT: The translation and rotation is from the camera perspective: as such
 *            all the values have been negated.
 *
 * @param   rvec    the rotation vector (radians) (rotx,roty,rotz)
 * @param   tvec    the translation vector (pixels) (x,y,z) where z is read from depthMat 
 * @returns the 4x4 homogeneous transform
 */

Mat reconfigure_reference(Mat rvec, Mat tvec) {

    float x = -tvec.at<double>(0), y = -tvec.at<double>(1), z = tvec.at<double>(2);
    float rotx = -rvec.at<double>(0), roty = -rvec.at<double>(1), rotz = -rvec.at<double>(2);
    // This seems to make it work
    std::swap(roty,rotx);
    Mat HT;

    HT = (Mat_<float>(4,4) <<
        cos(roty) * cos(rotx),
        cos(roty) * sin(rotx) * sin(rotz) - sin(roty) * cos(rotz),
        cos(roty) * sin(rotx) * cos(rotz) + sin(roty) * sin(rotz),
        x,

        sin(roty) * cos(rotx),
        sin(roty) * sin(rotx) * sin(rotz) + cos(roty) * cos(rotz),
        sin(roty) * sin(rotx) * cos(rotz) - cos(roty) * sin(rotz),
        y,

        -sin(rotx),
        cos(rotx) * sin(rotz),
        cos(rotx) * cos(rotz),
        z,
        
        0, 0, 0, 1);

    return HT;
}

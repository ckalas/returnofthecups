#include <iostream>
#include <stdio.h>
#include "cups.h"

using namespace cv;
using namespace std;

void detect_cups(Mat *frame, CascadeClassifier cascade) {

	std::vector<cv::Rect> matches;
	Mat gray;

	cvtColor(*frame, gray, CV_BGR2GRAY);
	equalizeHist(gray,gray);

	cascade.detectMultiScale(gray, matches, 1.3, 3,0|CV_HAAR_SCALE_IMAGE, Size(20, 30));
	for (size_t i = 0; i < matches.size(); i++) {
		Point tl (matches[i].x, matches[i].y);
		Point br (matches[i].x+matches[i].width, matches[i].y+matches[i].height);
		rectangle(*frame, tl ,br, Scalar( 0, 255, 255 ), +2, 4);
	}

}
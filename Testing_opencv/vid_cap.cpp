#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
  VideoCapture cap(0);
  cap.set(CV_CAP_PROP_GAIN, 0.0);
  //  cap.set(CV_CAP_PROP_BRIGHTNESS, 0.0);

  cout << "Check VideoCapture param: " << cap.get(CV_CAP_PROP_GAIN) << endl;
  
  if (!cap.isOpened()) {
    cout << "cannot open camera";
  }

  long int e1, e2;
  char k;
  double t;
  int fps;



  while (true) {
    e1 = getTickCount();

    Mat cameraFrame;
    cap.read(cameraFrame);
    imshow("cam", cameraFrame);
    moveWindow("cam", 0, 0);

    k = waitKey(1);

    if (k == 'q' || k == 'Q')
      break;

    e2 = getTickCount();
    t = (e2 - e1) / getTickFrequency();
    fps = int( 1 / t );
    cout << "FPS: " << fps << endl;
		
  }

  return 0;
}

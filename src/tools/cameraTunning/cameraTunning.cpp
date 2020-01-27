#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {
  VideoCapture capture(0);

  if (!capture.isOpened()) {
    cerr << "Could not access or detect camera" << endl;
    cin.get();
  }

  system("../startCam.sh");

  String rawWindow = "Raw";
  String threshWindow = "HSV Threshold";
  namedWindow(rawWindow, WINDOW_NORMAL);
  namedWindow(rawWindow, WINDOW_NORMAL);

  int hLow = 60;
  int hHigh = 70;
  int sLow = 220; 
  int sHigh = 255;
  int lLow = 100;
  int lHigh = 140;

  createTrackbar( "Hue Low", threshWindow, &hLow, 180, on_trackbar );
  createTrackbar( "Hue High", threshWindow, &hHigh, 180, on_trackbar );
  createTrackbar( "Saturation Low", threshWindow, &sLow, 255, on_trackbar );
  createTrackbar( "Saturation High", threshWindow, &sHigh, 255, on_trackbar );
  createTrackbar( "Lightness Low", threshWindow, &lLow, 255, on_trackbar );
  createTrackbar( "Lightness High", threshWindow, &lHigh, 255, on_trackbar );

  Mat rawFrame, threshFrame;
  while(true) {
    capture >> rawFrame;

    if (rawFrame.empty()) {
      cerr << "Lost Camera Connection" << endl;
      cin.get();
      break;
    }

    cvtColor(rawFrame, threshFrame, CV_BGR2HLS);
    inRange(hsv, Scalar(hLow, lLow, sLow), Scalar(hHigh, lHigh, sHigh), threshFrame);

    imshow(rawWindow, rawFrame);
    imshow(threshWindow, threshFrame);

    if (waitKey(10) == 27) {
      cout << "Esc pressed. Capture ended" << endl;
      break;  
    }
  }

  destroyAllWindows();

  return 0;
}

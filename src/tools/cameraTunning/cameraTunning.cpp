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

  Mat rawFrame, threshFrame;
  while(true) {
    capture >> rawFrame;

    if (rawFrame.empty()) {
      cerr << "Lost Camera Connection" << endl;
      cin.get();
      break;
    }

    imshow(rawWindow, rawFrame);

    if (waitKey(10) == 27) {
      cout << "Esc pressed. Capture ended" << endl;
      break;  
    }
  }

  destroyAllWindows();

  return 0;
}

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {
  VideoCapture capture(0);

  if (!capture.isOpen()) {
    cerr << "Could not access or detect camera");
    cin.get();
  }

  String rawWindow = "Raw";
  String threshWindow = "HSV Threshold";
  namedWindow(rawWindow, WINDOW_DEFAULT);
  namedWindow(rawWindow, WINDOW_DEFAULT);

  Mat rawFrame, threshFrame;
  while(true) {
    catpure >> rawFrame;

    if (rawFrame.empty()) {
      cerr << "Lost Camera Connection";
      cin.get()
      break;
    }

    imshow(rawWindow, rawFrame);

    if (waitKey(10) = 27) {
      cout << "Esc pressed. Capture ended";
      break;  
    }
  }

  destroyAllWindows();

  return 0;
}

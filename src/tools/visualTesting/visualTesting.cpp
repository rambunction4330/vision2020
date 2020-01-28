#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define DONOTKNOW 10000000

const int iLowH = 59;
const int iHighH = 61;
const int iLowS = 220; 
const int iHighS = 255;
const int iLowV = 115;
const int iHighV = 135;
const double horizontalFOV = 70.42;
const double verticalFOV = 43.3;
const double diagonalFOV = 78;
double powerPortRelativeBearing = DONOTKNOW;
double powerPortGlobalYAngle = DONOTKNOW;
double loadingBayGlobalYAngle = DONOTKNOW;
double loadingBayRelativeBearing = DONOTKNOW;

int main() {

  VideoCapture capture(0);
  system("../startCam.sh");

  if(!capture.isOpened()) {
    cout << "Failed to connect to the camera." << endl;
  }
  double width = capture.get(CAP_PROP_FRAME_WIDTH);
  double height = capture.get(CAP_PROP_FRAME_HEIGHT);
  double focalLength = (width/2)/(tan(((horizontalFOV * (M_PI/180))/2)));

  //Ideal shape of high goal reflective tape.
  std::vector<Point> powerPortTarget;
  powerPortTarget.push_back(Point2d(0,17));
  powerPortTarget.push_back(Point2d(9.805,0));
  powerPortTarget.push_back(Point2d(29.445,0));
  powerPortTarget.push_back(Point2d(39.25,17));
  powerPortTarget.push_back(Point2d(36.941,17));
  powerPortTarget.push_back(Point2d(28.290,2));
  powerPortTarget.push_back(Point2d(10.960,2));
  powerPortTarget.push_back(Point2d(2.309,17));
  powerPortTarget.push_back(Point2d(0,17));
  
  std::vector<Point> loadingBayTarget;
  loadingBayTarget.push_back(Point2d(0,0));
  loadingBayTarget.push_back(Point2d(0,11));
  loadingBayTarget.push_back(Point2d(7,11));
  loadingBayTarget.push_back(Point2d(7,0));
  loadingBayTarget.push_back(Point2d(0,0));

  String rawWindow = "Raw";
  String threshWindow = "Thresh";
  namedWindow(rawWindow, WINDOW_NORMAL);
  namedWindow(threshWindow, WINDOW_NORMAL);

  while(true) {
    Mat frame, hsv, thresh;
	
    capture >> frame;

    if(frame.empty()) {
      cout << "failed to capture an image" << endl;
    }
     
    cvtColor(frame, hsv, CV_BGR2HSV);
    inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresh);

    std::vector < std::vector<Point> > contours;
    findContours(thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    
    double bestPowerPortMatch = DONOTKNOW;
    double bestLoadingBayMatch = DONOTKNOW;
    std::vector<Point> powerPortContour;
    std::vector<Point> loadingBayContour;

    for (vector< vector<Point> >::iterator it = contours.begin(); it != contours.end(); ++it) {
      double matchPowerPort = matchShapes(powerPortTarget, *it, CV_CONTOURS_MATCH_I2, 0);
      double matchLoadingBay = matchShapes(loadingBayTarget, *it, CV_CONTOURS_MATCH_I2, 0);

      if ( matchPowerPort < bestPowerPortMatch && matchPowerPort != 0 && matchPowerPort < matchLoadingBay) {
        bestPowerPortMatch = matchPowerPort;
        powerPortContour = *it;
      }
      if ( matchLoadingBay < bestLoadingBayMatch && matchLoadingBay != 0 && matchLoadingBay < matchPowerPort) {
        bestLoadingBayMatch = matchLoadingBay;
        loadingBayContour = *it;
        
      }
      else {
        cout << "whoops!" << endl;
      }
    }

    Moments powerPortMoms = moments(Mat(powerPortContour));
    Moments loadingBayMoms = moments(Mat(loadingBayContour));
    Point2d powerPortCenter(powerPortMoms.m10/powerPortMoms.m00, powerPortMoms.m01/powerPortMoms.m00);
    Point2d loadingBayCenter(loadingBayMoms.m10/loadingBayMoms.m00, loadingBayMoms.m01/loadingBayMoms.m00);
    double powerPortXAngle = atan((powerPortCenter.x - (width/2)) / focalLength) * (180/M_PI);
    double powerPortYAngle = atan((powerPortCenter.y - (height/2)) / focalLength) * (180/M_PI);
    double loadingBayXAngle = atan((loadingBayCenter.x - (width/2)) / focalLength) * (180/M_PI);
    double loadingBayYAngle = atan((loadingBayCenter.y - (height/2)) / focalLength) * (180/M_PI);

    cout << powerPortXAngle << ", " << loadingBayXAngle << endl;

    imshow(rawWindow, frame);
    imshow(threshWindow, thresh);

    if (waitKey(10) == 27) {
      destroyAllWindows();
      break;
    }
  }

  destroyAllWindows();

  return 0;
}

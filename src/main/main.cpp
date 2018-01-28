#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>

#define PORTNUMBER  9001 
#define DONOTKNOW 10000000

using namespace std;
using namespace cv;

// HSV (Hue, Saturation, Value) thresholds
const int iLowH = 59;
const int iHighH = 61;
const int iLowS = 220; 
const int iHighS = 255;
const int iLowV = 115;
const int iHighV = 135;

// Camera data units in degrees and meters
const double horizontalFOV = 51.43;
const double verticalFOV = 20;
const double mountYAngle = 9.10;
const double mountXAngle = 0.0;
const double mountHeight = 0.672;

//Target data units in meters
const double powerPortHeight = 0.0;
const double loadingBayHeight = 0.564;

// Variables to hold data collected from vision
double powerPortRelativeBearing = DONOTKNOW;
double powerPortGlobalYAngle = DONOTKNOW;
double powerPortGlobalDistance = DONOTKNOW;
double loadingBayGlobalYAngle = DONOTKNOW;
double loadingBayRelativeBearing = DONOTKNOW;
double loadingBayGlobalDistance = DONOTKNOW;
//pthread_mutex_t dataLock;
std::mutex dataMutex;

// forward declaration of functions
void handleClient(int&);
void receiveNextCommand(char*, int);
void capture(int& arg);

int main(void)
{
  int n, s;
  socklen_t len;
  int max;
  int number;
  struct sockaddr_in name;
  //pthread_mutex_init(&dataLock, NULL);
  //mutex = std::mutex();

  // create the socket
  if ( (s = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    perror("socket");
    exit(1);
  }

  memset(&name, 0, sizeof(struct sockaddr_in));
  name.sin_family = AF_INET;
  name.sin_port = htons(PORTNUMBER);
  len = sizeof(struct sockaddr_in);

  // listen on all network interfaces
  n = INADDR_ANY;
  memcpy(&name.sin_addr, &n, sizeof(long));

  int reuse = 1;
  if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0) {
    perror("setsockopt(SO_REUSEADDR)");
    exit(1);
  }

  if (setsockopt(s, SOL_SOCKET, SO_KEEPALIVE, (const char*)&reuse, sizeof(reuse)) < 0) {
    perror("setsockopt(SO_KEEPALIVE)");
    exit(1);
  }

  // bind socket the network interface
  if (bind(s, (struct sockaddr *) &name, len) < 0) {
    perror("bind");
    exit(1);
  }

  // listen for connections
  if (listen(s, 5) < 0) {
    perror("listen");
    exit(1);
  }
 
  //pthread_t captureThreadId;
  int i = 3;
  //int captureThread = pthread_create(&captureThreadId, NULL, capture, (void*)&i);

  std::thread captureThread(capture, std::ref(i));

  // it is important to detach the thread to avoid memory leak
  //pthread_detach(captureThreadId);
  captureThread.detach();

  while(true) {

    // block until get a request
    int ns = accept(s, (struct sockaddr *) &name, &len);

    if ( ns < 0 ) {
      perror("accept");
      exit(1);
    }

    // each client connection is handled by a seperate thread since
    // a single client can hold a connection open indefinitely making multiple
    // data requests prior to closing the connection
    //pthread_t threadId;
    //int thread = pthread_create(&threadId, NULL, handleClient, (void*) &ns);
    std::thread childThread(handleClient, std::ref(ns));

    // it is important to detach the thread to avoid a memory leak
    //pthread_detach(threadId);
    childThread.detach();
  } 
  
  close(s);
  exit(0);
}

void capture(int& i) {  

  // Define capture device (camera)
  VideoCapture capture(0);

  // Make sure camera is connected
  if(!capture.isOpened()) {
    cout << "Failed to connect to the camera." << endl;
  }

  // Get the camera frams height and width
  double width = capture.get(CAP_PROP_FRAME_WIDTH);
  double height = capture.get(CAP_PROP_FRAME_HEIGHT);

  // Calculate the camera focal length in pixles
  // Uses pinhole camera modle
  // 
  // Cpp standard library uses radients not degrees
  //   degrees --> radients: * (pi/180)
  //   radients --> degrees: * (180/pi)
  //
  // w = camera frame width (in pixles)
  // θ = Feild of view (in degrees)
  // f = focal length (in pixles)
  //
  // tan(θ°/2) = (w/2)/f
  // 
  // f = (w/2)/tan(θ°/2)
  //
  //  pinhole       / 
  //        |      / 
  // sensor |     /  
  //  |     |    /   
  //  |\    |   /    
  //  | \   |  /     
  //  |  \  | /      
  //  |   \ |/       
  // w|____\/_________
  //  | f  /\ θ°
  //  |   /  \
  //  |  /    \
  //  | /      \
  //  |/        \
  //             \
  //              \
  //               \
  //                \
  //
  double focalLength = (width/2)/(tan(((horizontalFOV * (M_PI/180))/2)));

  // Comparison shape for power port vision target
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
  
  // Comparison shape for loading bay vision target
  std::vector<Point> loadingBayTarget;
  loadingBayTarget.push_back(Point2d(0,0));
  loadingBayTarget.push_back(Point2d(0,11));
  loadingBayTarget.push_back(Point2d(7,11));
  loadingBayTarget.push_back(Point2d(7,0));
  loadingBayTarget.push_back(Point2d(0,0));

  // Uncomment for debugging
  // Defines windows for debugging
  //  String rawWindow = "Raw";
  //  String threshWindow = "Thresh";
  //  namedWindow(rawWindow, WINDOW_NORMAL);
  //  namedWindow(threshWindow, WINDOW_NORMAL);

  while(true) {
    // Declare Mats
    Mat frame, hsv, thresh;

    // Get the current frame from the camer	
    capture >> frame;

    // Cheak to be sure it is still getting camer data
    if(frame.empty()) {
      cout << "failed to capture an image" << endl;
    }
     
    // Convert into HSV color space for better thresholding
    cvtColor(frame, hsv, CV_BGR2HSV);
    // Threshold the frame into black and white image
    // All pixles within range are pure white
    // All pixles out of the threshold are pure black
    inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), thresh);

    // Find all the contours in the thresholded image and hold them in vector
    // Only finds external contours
    std::vector < std::vector<Point> > contours;
    findContours(thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    
    double bestPowerPortMatch = DONOTKNOW;
    double bestLoadingBayMatch = DONOTKNOW;
    std::vector<Point> powerPortContour;
    std::vector<Point> loadingBayContour;

    // Iterate over all he vectors to find which ones are which targets
    for (vector< vector<Point> >::iterator it = contours.begin(); it != contours.end(); ++it) {
      // Cheak how well they match each shape. Lower numbers are better. 0 is best.
      double matchPowerPort = matchShapes(powerPortTarget, *it, CV_CONTOURS_MATCH_I2, 0);
      double matchLoadingBay = matchShapes(loadingBayTarget, *it, CV_CONTOURS_MATCH_I2, 0);

      // Assign each contour to whichver shape in matches best filtering out any single points due to noise
      if ( matchPowerPort < bestPowerPortMatch && matchPowerPort < matchLoadingBay& & matchPowerPort != 0) {
        bestPowerPortMatch = matchPowerPort;
        powerPortContour = *it;
      }
      if ( matchLoadingBay < bestLoadingBayMatch && matchLoadingBay < matchPowerPort && matchLoadingBay != 0) {
        bestLoadingBayMatch = matchLoadingBay;
        loadingBayContour = *it;
        
      }
    }

    // Calculate contour moments, and the center for each contour
    Moments powerPortMoms = moments(Mat(powerPortContour));
    Moments loadingBayMoms = moments(Mat(loadingBayContour));
    Point2d powerPortCenter(powerPortMoms.m10/powerPortMoms.m00, powerPortMoms.m01/powerPortMoms.m00);
    Point2d loadingBayCenter(loadingBayMoms.m10/loadingBayMoms.m00, loadingBayMoms.m01/loadingBayMoms.m00);

    // Find the angle from the camera to the center of the image
    // Uses pinhole camera modle
    // 
    // Cpp standard library uses radients not degrees
    //   degrees --> radients: * (pi/180)
    //   radients --> degrees: * (180/pi)
    //
    // w = camera frame width (in pixles)
    // c = center of the target
    // f = focal length (in pixles) 
    // θ = angle of target from camera center
    //
    // tan(θ°) = (c-(w/2))/f
    // 
    // θ° = atan((c-(w/2))/f)
    //
    //  pinhole       / 
    //        |      / 
    // sensor |     /  
    //  |     |    /   
    //  |\    |   /    
    //  | \   |  /     
    // c*  \  | /      
    //  |\_ \ |/       
    // w|__\_\/_________
    //  | f  /\\_ θ°
    //  |   /  \ \_
    //  |  /    \  \_
    //  | /      \   \_
    //  |/        \    \_
    //             \     \
    //              \     *
    //               \
    //                \
    //
    double powerPortXAngle = -atan((powerPortCenter.x - (width/2)) / focalLength) * (180/M_PI) + mountXAngle;
    double powerPortYAngle = -atan((powerPortCenter.y - (height/2)) / focalLength) * (180/M_PI) + mountYAngle;
    double loadingBayXAngle = -atan((loadingBayCenter.x - (width/2)) / focalLength) * (180/M_PI) + mountXAngle;
    double loadingBayYAngle = -atan((loadingBayCenter.y - (height/2)) / focalLength) * (180/M_PI) + mountYAngle;

    //Using Y angle find the distance from the target
    //
    // Cpp standard library uses radients not degrees
    //   degrees --> radients: * (pi/180)
    //   radients --> degrees: * (180/pi)
    //
    // h = height of the camera
    // t = target height
    // θ = angle to target
    // d = distance to target
    //
    // tan(θ°) = (t-h)/d
    //
    // d = (t-h)/tan(θ°)
    //
    //                *target
    //               /|
    //              / |
    //             /  |
    //            /   |
    //           /    |
    //          /     |t
    //         / θ°   |
    // Camera /_ _ _ _|
    //        |   d   |
    //      h |       |
    //        |_______| Ground
    //
    double powerPortDistance = (powerPortHeight - mountHeight) / (tan(powerPortYAngle * (M_PI/180)));
    double loadingBayDistance = (loadingBayHeight - mountHeight) / (tan(loadingBayYAngle * (M_PI/180)));

    // obtain the lock and copy the data
    //pthread_mutex_lock(&dataLock);
    dataMutex.lock();

    powerPortRelativeBearing = powerPortXAngle;
    powerPortGlobalYAngle = powerPortYAngle;
    powerPortGlobalDistance = powerPortDistance;
    loadingBayRelativeBearing = loadingBayXAngle;
    loadingBayGlobalYAngle = loadingBayYAngle;
    loadingBayGlobalDistance = loadingBayDistance;

    //pthread_mutex_unlock(&dataLock);
    dataMutex.unlock();

    cout  << loadingBayDistance << endl;

  }
}



void handleClient(int& ns) {
  // printf("Thread starting\n");
  //int ns = *((int*) arg);
  char sendbuffer[1024];
  char command[128];

  // start conversation with client
  while(true) {

    receiveNextCommand(command, ns);

    if ( strcmp(command, "STOP") == 0 ) {
      //printf("Received STOP command\n");
      break;
    } else if ( strcmp(command, "DATA") == 0 ) {
      //printf("Received DATA command\n");

      // obtain the lock and copy the data
      //pthread_mutex_lock(&dataLock);
      dataMutex.lock();

      double copyPowerPortRelativeBearing = powerPortRelativeBearing;
      double copyPowerPortGlobalYAngle = powerPortGlobalYAngle;
      double copyPowerPortGlobalDistance = powerPortGlobalDistance;
      double copyLoadingBayRelativeBearing = loadingBayRelativeBearing;
      double copyLoadingBayGlobalYAngle = loadingBayGlobalYAngle;
      double copyLoadingBayGlobalDistance = loadingBayGlobalDistance;

      //pthread_mutex_unlock(&dataLock);
      dataMutex.unlock();

      // the protocol will send an empty line when the data transfer is complete
      int sendbufferLen = -1;
      if (isnan(copyPowerPortRelativeBearing) && isnan(copyLoadingBayRelativeBearing)) {
        sendbufferLen = sprintf(sendbuffer, "\n");
      } else if (isnan(copyPowerPortRelativeBearing)){
        sendbufferLen = sprintf(sendbuffer, "lbrb=%.1f\nlbya=%.1f\nlbd=%.1f\n\n", copyLoadingBayRelativeBearing, copyLoadingBayGlobalYAngle, copyLoadingBayGlobalDistance);
      } else if (isnan(copyLoadingBayRelativeBearing)){
        sendbufferLen = sprintf(sendbuffer, "pprb=%.1f\nppya=%.1f\nppd=%.1f\n\n", copyPowerPortRelativeBearing, copyPowerPortGlobalYAngle, copyPowerPortGlobalDistance);
      } else{
        sendbufferLen = sprintf(sendbuffer, "pprb=%.1f\nppya=%.1f\nppd=%.1f\nlbrb=%.1f\nlbya=%.1f\nlbd=%.1f\n\n", copyPowerPortRelativeBearing, copyPowerPortGlobalYAngle, copyPowerPortGlobalDistance, copyLoadingBayRelativeBearing, copyLoadingBayGlobalYAngle, copyLoadingBayGlobalDistance);
      }
      // write response to client
      write(ns, sendbuffer, sendbufferLen);
    } else {
      //printf("Received unknown command '%s'\n", command);
      break;
    }

  }
  close(ns);
  //printf("Thread ending\n");
}

void receiveNextCommand(char *command, int ns) {
  int receiveLength = read(ns, command, 1024);
  int commandLength = 0;
  while(commandLength < receiveLength) {
    char value = command[commandLength];
    if ( value == 0x0d || value == 0x0a ) {
      break;
    } 
    commandLength++;
  }

  // add the terminating 0 to mark the end of the string value in the char *
  command[commandLength] = 0;
}

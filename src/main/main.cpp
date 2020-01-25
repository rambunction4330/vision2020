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

#define PORTNUMBER  9001 
#define DONOTKNOW 10000000

using namespace std;
using namespace cv;

//added for further changes
const int iLowH = 60;
const int iHighH = 70;
const int iLowS = 220; 
const int iHighS = 255;
const int iLowL = 100;
const int iHighL = 140;
const double horizontalFOV = 70.42;
const double verticalFOV = 43.3;
const double diagonalFOV = 78;
double powerPortRelativeBearing = DONOTKNOW;
double powerPortGlobalYAngle = DONOTKNOW;
double loadingBayGlobalYAngle = DONOTKNOW;
double loadingBayRelativeBearing = DONOTKNOW;
pthread_mutex_t dataLock;

// forward declaration of functions
void *handleClient(void *arg);
void receiveNextCommand(char*, int);
void *capture(void *arg);

int main(void)
{
  int n, s;
  socklen_t len;
  int max;
  int number;
  struct sockaddr_in name;
  pthread_mutex_init(&dataLock, NULL);

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
 
  pthread_t captureThreadId;
  int i = 3;
  int captureThread = pthread_create(&captureThreadId, NULL, capture, (void*)&i);

  // it is important to detach the thread to avoid memory leak
  pthread_detach(captureThreadId);

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
    pthread_t threadId;
    int thread = pthread_create(&threadId, NULL, handleClient, (void*) &ns);
    // it is important to detach the thread to avoid a memory leak
    pthread_detach(threadId);
  } 
  
  close(s);
  exit(0);
}

void *capture(void *arg) {  

  VideoCapture capture(0);
  system("../startCam.sh");

  if(!capture.isOpened()) {
    cout << "Failed to connect to the camera." << endl;
  }
  double width = capture.get(CAP_PROP_FRAME_WIDTH);
  double height = capture.get(CAP_PROP_FRAME_HEIGHT);
  double focalLength = (width/2)/(tan((horizontalFOV/2) * (M_PI/180)));

  Mat frame, hsv, thresh;
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

  while(true) {
		
    capture >> frame;
    if(frame.empty()) {
      cout << "failed to capture an image" << endl;
    }
     
    cvtColor(frame, hsv, CV_BGR2HLS);
    inRange(hsv, Scalar(iLowH, iLowS, iLowL), Scalar(iHighH, iHighS, iHighL), thresh);

    std::vector < std::vector<Point> > contours;
    findContours(thresh, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    
    double bestPowerPortMatch = DONOTKNOW;
    double bestLoadingBayMatch = DONOTKNOW;
    std::vector<Point> powerPortContour;
    std::vector<Point> loadingBayContour;

    for (vector< vector<Point> >::iterator it = contours.begin(); it != contours.end(); ++it) {
      double matchPowerPort = matchShapes(powerPortTarget, *it, CV_CONTOURS_MATCH_I2, 0);
      double matchLoadingBay = matchShapes(powerPortTarget, *it, CV_CONTOURS_MATCH_I2, 0);

      if ( matchPowerPort < bestPowerPortMatch && matchPowerPort != 0) {
        bestPowerPortMatch = matchPowerPort;
        powerPortContour = *it;
      }

      if ( matchLoadingBay < bestLoadingBayMatch && matchLoadingBay != 0 && matchLoadingBay > matchPowerPort) {
        bestLoadingBayMatch = matchLoadingBay;
        loadingBayContour = *it;
      }
    }

    Moments powerPortMoms = moments(Mat(powerPortContour));
    Moments loadingBayMoms = moments(Mat(loadingBayContour));
    Point2d powerPortCenter(powerPortMoms.m10/powerPortMoms.m00, powerPortMoms.m01/powerPortMoms.m00);
    Point2d loadingBayCenter(loadingBayMoms.m10/loadingBayMoms.m00, loadingBayMoms.m01/loadingBayMoms.m00);
    double powerPortXAngle = atan(powerPortCenter.x - (width/2)) / focalLength;
    double powerPortYAngle = atan(powerPortCenter.y - (height/2)) / focalLength;
    double loadingBayXAngle = atan(loadingBayCenter.x - (width/2)) / focalLength;
    double loadingBayYAngle = atan(loadingBayCenter.y - (height/2)) / focalLength;

    // obtain the lock and copy the data
    pthread_mutex_lock(&dataLock);
    powerPortRelativeBearing = powerPortXAngle;
    powerPortGlobalYAngle = powerPortYAngle;
    loadingBayRelativeBearing = loadingBayXAngle;
    loadingBayGlobalYAngle = loadingBayYAngle;
    pthread_mutex_unlock(&dataLock);
  }
}



void *handleClient(void *arg) {
  // printf("Thread starting\n");
  int ns = *((int*) arg);
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
      pthread_mutex_lock(&dataLock);
      double copyPowerPortRelativeBearing = powerPortRelativeBearing;
      double copyPowerPortGlobalYAngle = powerPortGlobalYAngle;
      double copyLoadingBayRelativeBearing = loadingBayRelativeBearing;
      double copyLoadingBayGlobalYAngle = loadingBayGlobalYAngle;
      pthread_mutex_unlock(&dataLock);
      
      // the protocol will send an empty line when the data transfer is complete
      int sendbufferLen = -1;
      if ( copyPowerPortRelativeBearing == DONOTKNOW && copyLoadingBayRelativeBearing == DONOTKNOW) {
        sendbufferLen = sprintf(sendbuffer, "\n");
      } else if ( copyPowerPortRelativeBearing == DONOTKNOW ){
        sendbufferLen = sprintf(sendbuffer, "lbrb=%.1f\nlbya=%.1f\n", copyLoadingBayRelativeBearing, copyLoadingBayGlobalYAngle);
      } else if ( copyLoadingBayRelativeBearing == DONOTKNOW ){
        sendbufferLen = sprintf(sendbuffer, "pprb=%.1f\nppya=%.1f\n", copyPowerPortRelativeBearing, copyPowerPortGlobalYAngle);
      } else{
        sendbufferLen = sprintf(sendbuffer, "pprb=%.1f\nppya=%.1f\nlbrb=%.1f\nlbya=%.1f\n", copyPowerPortRelativeBearing, copyPowerPortGlobalYAngle, copyLoadingBayRelativeBearing, copyLoadingBayGlobalYAngle);
      }

        sendbufferLen = sprintf(sendbuffer, "rb=%.1f\nya=%.1f\n\n", copyRelativeBearing, copyGlobalYAngle);
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
  return 0;
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

// Code modified from https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main( int argc, char** argv) {
  VideoCapture cap(0); //captures the video from USB Camera (Need to change line to access camera)
  if (!cap.isOpened()) //if not sucess, exit program
    cout << "Cannot open this webcam" << endl //once the USB Camera is accessed, change this to "Cannot access Camera"
      return -1;
}

namedWindow("Control", CV_WINDOW_AUTOSIZE); //Creates a window called Control

int iLowH = 170;
int iHighH = 180;

int iLowS = 150;
int IHighS = 250;

int iLOwV = 60;
int iHighV = 250;

//Creating trackbars in "Controls" Window
 createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 createTrackbar("HighH", "Control", &iHighH, 179);

 createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 createTrackbar("HighS", "Control", &iHighS, 255);

 createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
 createTrackbar("HighV", "Control", &iHighV, 255);

 int iLastX = -1; 
 int iLastY = -1;

// Capturing a temporary image from Camera
Mat imgTmp;
cap.read(imgTmp);

// Creating a black image with the size of the camera output
Mat imgLines = Mat::zerps( imgTmp.size(), CV_8UC3);;
 while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video



         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }
    }
Mat imgHSV;

cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
Mat imgThresholded;

//morphological opening (removes small objects from the foreground)
erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  //morphological closing (removes small holes from the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  //Calculate the moments of the thresholded image
Moments oMoments = moments(imgThresholded);

double dM01 = oMoments.m01;
double dM10 = oMoments.m10;
double dArea = oMomens.m00;

if (dArea > 10000) {
  int posX = dM10/dArea; // X coordinate of the position of the center of the object = 1st order spacial moment around x axis
  int posY = dM01/dArea; // Y coordinate of the position of the center of the object = 2nd order spacial moment around y axis

if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
 {
  //Draw a red line from the previous point to the current point
  line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
 }

  iLastX = posX; //The Last recorded X coordinate
  iLastY = posY; //The Last recorded Y
}

imshow("Thresholded Image", imgThresholded); //show the thresholded image
imgOriginal = imgOriginal + imgLines;
imshow("Original", imgOriginal); //show the original image

if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            break; 
       }
    }

   return 0;
}

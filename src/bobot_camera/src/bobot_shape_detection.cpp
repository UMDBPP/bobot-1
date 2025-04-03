//https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html
//Reminder convert to ROS 2 

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    VideoCapture cap(0); // capture video from webcam

    if (!cap.isOpened()) {
        cout << "Cannot open the web cam" << endl;
        return -1;
    }

    namedWindow("Control", WINDOW_AUTOSIZE);

    int iLowH = 0, iHighH = 179;
    int iLowS = 0, iHighS = 255;
    int iLowV = 0, iHighV = 255;

    createTrackbar("LowH", "Control", &iLowH, 179);
    createTrackbar("HighH", "Control", &iHighH, 179);
    createTrackbar("LowS", "Control", &iLowS, 255);
    createTrackbar("HighS", "Control", &iHighS, 255);
    createTrackbar("LowV", "Control", &iLowV, 255);
    createTrackbar("HighV", "Control", &iHighV, 255);

    while (true) {
        Mat frame;
        bool success = cap.read(frame);
        if (!success) break;

        Mat imgHSV, imgThresholded;
        cvtColor(frame, imgHSV, COLOR_BGR2HSV);
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV),
                         Scalar(iHighH, iHighS, iHighV), imgThresholded);

        // Clean up the image
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_RECT, Size(3, 3)));
        dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_RECT, Size(3, 3)));

        dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_RECT, Size(3, 3)));
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_RECT, Size(3, 3)));

        // Contour detection
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(imgThresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours.size(); ++i) {
            double area = contourArea(contours[i]);
            if (area < 1000) continue; // Filter small contours

            vector<Point> approx;
            approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.02, true);

            if (approx.size() == 4 && isContourConvex(approx)) {
                // Check aspect ratio to differentiate between rectangle and square
                Rect boundingBox = boundingRect(approx); 
                float aspectRatio = (float)boundingBox.width / (float)boundingBox.height;

                // If aspect ratio is close to 1, it's a square
                if (abs(aspectRatio - 1) < 0.2) {
                    drawContours(frame, vector<vector<Point>>{approx}, -1, Scalar(0, 255, 0), 3);
                    putText(frame, "Square", approx[0], FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 0), 2);
                } else {
                    drawContours(frame, vector<vector<Point>>{approx}, -1, Scalar(0, 0, 255), 3);
                    putText(frame, "Rectangle", approx[0], FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 0), 2);
                }
            }
        }

        imshow("Original", frame);
        imshow("Thresholded", imgThresholded);

        if (waitKey(30) == 27) break; // ESC to quit
    }

    return 0;
}

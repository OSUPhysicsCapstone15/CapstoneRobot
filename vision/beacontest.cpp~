#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <string>
#include <unistd.h>
#include <ctime>

using namespace cv;
using namespace std;

struct hsvParams
  {
  int hL, sL, vL, hH, sH, vH;
};

Point computeCentroid(const cv::Mat &mask) {
    Moments m = moments(mask, true);
    Point center(m.m10/m.m00, m.m01/m.m00);
    return center;
}


//get robot and camera angles
//img -> the current camera image
//objectRows -> rows from the left that the object is in
//objectCols -> columns from the top that the object is
void tilt_turn_degrees(Mat img, int object_rows, int object_cols)
{
    double camera_height = 1.2;     // height of camera from ground in meters
    int camera_diagonal_angle = 69; // diagonal angle of view for camera in degrees
                                    // logitech c525 fov is 69 degrees, Samsung Galaxy S5 is 90 degrees
    
    int rows = img.rows; // height of camera image in pixels
    int cols = img.cols; // width of camera image in pixels
    //cout << "Rows: " << rows << "\n" << "Cols: " << cols << endl;
    
    //logitech c525 fov is 69 degrees, Samsung Galaxy S5 is 90 degrees
    double camera_diagonal = 69; // the angle of the cameras diagonal in degrees
    double pixel_diagonal = sqrt(rows * rows + cols * cols); // (pythagorean) diagonal length of image in pixels
    double degrees_per_pixel = camera_diagonal / pixel_diagonal; // ratio of real world degrees to pixels in the image
    
    int center_rows = rows / 2; // the center height is half of the total height
    int center_cols = cols / 2; // the center width is half of the total width
    //cout << "Center Rows: " << center_rows << "\n" << "Center Cols: " << center_cols << endl;
    
    int diff_rows = center_rows - object_rows; // difference between center and object rows
    int diff_cols = center_cols - object_cols; // difference between center and object cols
    //cout << "Diff Rows: " << diff_rows << "\n" << "Diff Cols: " << diff_cols << endl;
    
    double turn_robot_x_degrees = diff_cols * degrees_per_pixel; // positive -> turn left, negative -> turn right
    double tilt_camera_x_degrees = diff_rows * degrees_per_pixel; // positive -> tilt up, negative -> tilt down
    cout << "Turn robot " << turn_robot_x_degrees << " degrees.\n" << "Tilt camera " << tilt_camera_x_degrees << " degrees." << endl;
    
    double tilted_degrees = 90 - tilt_camera_x_degrees; // assuming camera is parallel to ground (90 degrees)

    double tilted_radians = tilted_degrees * 3.1415962 / 180.0; // c++ tan() function uses radians

    double height = camera_height; // height of camera from the ground in meters

    double distance = height * tan(tilted_radians); // triangle formula for finding distance

    cout << "Distance is " << distance << " meters" << endl;
}

<<<<<<< HEAD
=======

>>>>>>> 307c2810f4c3dd409af2eb1552ee8f18a002d689


 int main( int argc, char** argv )
 {
  namedWindow("Thresholded Image ON", WINDOW_AUTOSIZE);
  namedWindow("Thresholded Image OFF", WINDOW_AUTOSIZE);
  namedWindow("Original ON", WINDOW_AUTOSIZE);
  namedWindow("Original OFF", WINDOW_AUTOSIZE);
  namedWindow("Diff", WINDOW_AUTOSIZE);

  hsvParams hsv = {76,0,224,97,37,255};

  //Set up blob detection parameters
  SimpleBlobDetector::Params params;
  params.minDistBetweenBlobs = 0.0f;
  params.filterByInertia = true;
  params.filterByConvexity = false;
  params.filterByColor = false;
  params.filterByCircularity = false;
  params.filterByArea = true;

        params.minThreshold = 200;
        params.maxThreshold = 250;
        params.thresholdStep = 1;

        params.minArea = 50;
        params.minConvexity = 0.3;
        params.minInertiaRatio = 0.15;

        params.maxArea = 8000;
        params.maxConvexity = 10;


  vector<KeyPoint> keypoints;

  string text("Object not found");

    VideoCapture cap(0); //capture the video from web cam
    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

Mat imgOriginalON, imgOriginalOFF;
clock_t start;
double duration=0;
double timer=1;

while(true){

cout<<"Taking On in "<<timer<<" s"<<endl;
start = std::clock();
duration=0;
while(duration<timer){
    cap>>imgOriginalON;
    duration = (clock() - start ) / (double) CLOCKS_PER_SEC;
}

cout<<"Taking On"<<endl;
cap>>imgOriginalON;
cout<<"Taking OFF in "<<timer<<" s"<<endl;

start = std::clock();
duration=0;
while(duration<timer){
    cap>>imgOriginalOFF;
    duration = (clock() - start ) / (double) CLOCKS_PER_SEC;
}

cout<<"Taking OFF"<<endl;
cap>>imgOriginalOFF;

    Mat imgHSVON,imgHSVOFF;
 //imgOriginalON = imread("./beaconON.jpg");
 //imgOriginalOFF = imread("./beaconOFF.jpg");
 if(imgOriginalON.empty() || imgOriginalOFF.empty())
 {
     cout << "can not open " << endl;
     return -1;
 }


   cvtColor(imgOriginalON, imgHSVON, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
   cvtColor(imgOriginalOFF, imgHSVOFF, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

  Mat imgThresholdedON, imgThresholdedOFF;

  inRange(imgHSVON, Scalar(hsv.hL, hsv.sL, hsv.vL), Scalar(hsv.hH, hsv.sH, hsv.vH), imgThresholdedON); //Threshold the image
  inRange(imgHSVOFF, Scalar(hsv.hL, hsv.sL, hsv.vL), Scalar(hsv.hH, hsv.sH, hsv.vH), imgThresholdedOFF); //Threshold the image

  //morphological opening (remove small objects from the foreground)
  erode(imgThresholdedON, imgThresholdedON, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholdedON, imgThresholdedON, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  erode(imgThresholdedON, imgThresholdedON, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  erode(imgThresholdedOFF, imgThresholdedOFF, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholdedOFF, imgThresholdedOFF, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  erode(imgThresholdedOFF, imgThresholdedOFF, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  Mat imgTHRESH=imgThresholdedON-imgThresholdedOFF, out;
  //SimpleBlobDetector blobDetect = SimpleBlobDetector(params);
  Ptr<SimpleBlobDetector> blobDetect = SimpleBlobDetector::create(params);
  blobDetect->detect( imgTHRESH, keypoints );

  drawKeypoints(imgTHRESH, keypoints, out, CV_RGB(0,0,0), DrawMatchesFlags::DEFAULT);
  //Circle blobs
  for(int i = 0; i < keypoints.size(); i++)
    {
    circle(out, keypoints[i].pt, 1.5*keypoints[i].size, CV_RGB(0,255,0), 20, 8);
    }

   if(keypoints.size() == 2){
    text = "Object Found";
    cout<<endl<<endl<<"Object Found"<<endl;
    Mat diff=imgThresholdedON-imgThresholdedOFF;
    int xCord=((keypoints[0].pt.x)+(keypoints[1].pt.y))/2;
    int yCord=abs((keypoints[1].pt.y)-(keypoints[0].pt.y))/2;
    tilt_turn_degrees(diff, yCord, xCord);
  }
  else{
    text = "Error";
    cout<<endl<<endl<<"No Object Found"<<endl;
  }

  putText(out, text, Point(100,200), FONT_HERSHEY_PLAIN, 20, Scalar(0, 0, 255), 20);

  imshow("Thresholded Image ON", imgThresholdedON); //show the thresholded image
  imshow("Thresholded Image OFF", imgThresholdedOFF); //show the thresholded image

  //imshow("blob Image", blobtest);
  imshow("Original ON", imgOriginalON); //show the original image
  imshow("Original OFF", imgOriginalOFF); //show the original image

  imshow("Diff", out);
 //       if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
 //      {
 //           cout << "esc key is pressed by user" << endl;
            //break;
 //      }
 //   }
	//waitKey(-1);
}
   return 0;

}

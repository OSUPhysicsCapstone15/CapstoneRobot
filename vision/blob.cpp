// blobdetect.cpp
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <unistd.h>
#include <string>
using namespace std;
using namespace cv;

////////////////////////////////////////////
struct hsvParams
  {
  int hL, sL, vL, hH, sH, vH;
  };
////////////////////////////////////////////
void removenoise(Mat&);
/////////////////////////////////////////////

Mat findBiggestBlob(Mat &src){
int largest_area=0;
int largest_contour_index=0;
Mat temp(src.rows,src.cols,CV_8UC1);
Mat dst(src.rows,src.cols,CV_8UC1,Scalar::all(0));
src.copyTo(temp);

vector<vector<Point> > contours; // storing contour
vector<Vec4i> hierarchy;

findContours( temp, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

for( int i = 0; i< contours.size(); i++ ) // iterate
{
    double a=contourArea( contours[i],false);  //Find the largest area of contour
    if(a>largest_area)
    {
        largest_area=a;
        largest_contour_index=i;
    }
}

drawContours( dst, contours,largest_contour_index, Scalar(255,0,0), CV_FILLED, 8, hierarchy );
// Draw the largest contour
return dst;
}

void findGrass(Mat &src, Mat &HSV){
  int iLowH = 20;
  int iHighH = 50;

  int iLowS = 60;
  int iHighS = 255;

  int iLowV = 0;
  int iHighV = 255;

  Mat imgThresholded;
  Mat temp;
  src.copyTo(temp);

  inRange(HSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    //dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    //erode( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(50, 50)) );

   imgThresholded=findBiggestBlob(imgThresholded);

    for(int i=0;i<imgThresholded.rows-1;i++){
        for(int j=0;j<imgThresholded.cols-1;j++){
            if(imgThresholded.at<uchar>(i,j)==0){
                temp.at<Vec3b>(i,j)=(0,0,0);
            }
        }
    }

    for(int i=0;i<temp.cols-1;i++){
        int minRow=0;
        while(minRow<temp.rows-1 && temp.at<Vec3b>(minRow,i)[0]==0 && temp.at<Vec3b>(minRow,i)[1]==0 && temp.at<Vec3b>(minRow,i)[2]==0){
            src.at<Vec3b>(minRow,i)=(0,0,0);
            minRow++;
        }
        if(minRow!=temp.rows-1){
            int maxRow=temp.rows-1;
            while(maxRow>maxRow && temp.at<Vec3b>(maxRow,i)[0]==0 && temp.at<Vec3b>(maxRow,i)[1]==0 && temp.at<Vec3b>(maxRow,i)[2]==0){
                src.at<Vec3b>(maxRow,i)=(0,0,0);
                maxRow--;
            }
        }
    }
}


int main()
  {
  hsvParams hsv = {20,0,50,180,70,200};

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

        params.minArea = 100;
        params.minConvexity = 0.3;
        params.minInertiaRatio = 0.15;

        params.maxArea = 8000;
        params.maxConvexity = 10;


  SimpleBlobDetector blob_detector;
  blob_detector.create(params);
  vector<KeyPoint> keypoints;

  const string filename("./cut_long/50 ft.jpg");
  string text("Object not found");

  Mat img, imgHSV, imgTHRESH, out;
  img = imread(filename, CV_LOAD_IMAGE_COLOR);
  if(img.empty()){
     cout << "can not open " << endl;
     return -1;
  }

  //convert color to HSV, threshold and remove noise
  cvtColor(img, imgHSV, COLOR_BGR2HSV);
  findGrass(img,imgHSV);
  cvtColor(img, imgHSV, COLOR_BGR2HSV);

  inRange(imgHSV, Scalar(hsv.hL, hsv.sL, hsv.vL),
         Scalar(hsv.hH, hsv.sH, hsv.vH), imgTHRESH);
  removenoise(imgTHRESH);

  namedWindow("Input", WINDOW_AUTOSIZE);
  namedWindow("Detection", WINDOW_AUTOSIZE);

  //Initialize blobdetector with predefine parameters
  Ptr<SimpleBlobDetector> blobDetect = SimpleBlobDetector::create(params);
  blobDetect->detect( imgTHRESH, keypoints );

  //SimpleBlobDetector blobDetect;
  //blobDetect.create(params);
  //Detect blobs
  //blobDetect.detect(imgTHRESH, keypoints);
  drawKeypoints(imgTHRESH, keypoints, out, CV_RGB(0,0,0), DrawMatchesFlags::DEFAULT);
  //Circle blobs
  for(int i = 0; i < keypoints.size(); i++)
    {
    circle(out, keypoints[i].pt, 1.5*keypoints[i].size, CV_RGB(0,255,0), 20, 8);
    }

  if(keypoints.size() == 1)
    text = "Object Found";
  if(keypoints.size() > 1)
    text = "Error";

  putText(out, text, Point(100,200), FONT_HERSHEY_PLAIN, 20, Scalar(0, 0, 255), 20);

  //equalizeHist(img, out);


  for(;;)
    {
    imshow("Input", img);
    imshow("Detection", out);
    if(waitKey(30) >= 0 ) break;
    }

  return 0;
  }
//.....................................................
//removenoise()
void removenoise(Mat& image)
  {
  //Morphologial opening
  erode(image,image,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
  dilate(image,image,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
  //Morphological closing
  dilate(image,image,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
  erode(image,image,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
  }
//.................................................................................

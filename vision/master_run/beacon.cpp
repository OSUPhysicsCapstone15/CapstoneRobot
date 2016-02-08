#include "beacon.h"
#include "functions.h"


int beacon_main(int cam)
 {
int thresh=220;
  namedWindow("Original ON", WINDOW_AUTOSIZE);
  namedWindow("Original OFF", WINDOW_AUTOSIZE);
  namedWindow("Diff", WINDOW_AUTOSIZE);

  //hsvParams hsv = {76,0,224,97,37,255};
  hsvParams hsv = {20,0,0,97,37,255};

  //Set up blob detection parameters
  SimpleBlobDetector::Params params;
  params.minDistBetweenBlobs = 50.0f;
  params.filterByInertia = true;
  params.filterByConvexity = false;
  params.filterByColor = false;
  params.filterByCircularity = false;
  params.filterByArea = true;

        params.minThreshold = 200;
        params.maxThreshold = 250;
        params.thresholdStep = 1;

        params.minArea = 0;
        params.minConvexity = 0.3;
        params.minInertiaRatio = 0.15;

        params.maxArea = 2000;
        params.maxConvexity = 10;


  vector<KeyPoint> keypoints;

  const string filenameON("./beaconON.jpg");
  const string filenameOFF("./beaconOFF.jpg");
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

imgOriginalON = imread(filenameON, CV_LOAD_IMAGE_COLOR);
imgOriginalOFF = imread(filenameOFF, CV_LOAD_IMAGE_COLOR);

cout<<"Taking On in "<<timer<<" s"<<endl;
start = std::clock();
duration=0;
while(cam && duration<timer){
    cap>>imgOriginalON;
    duration = (clock() - start ) / (double) CLOCKS_PER_SEC;
}

cout<<"Taking On"<<endl;
if(cam)
	cap>>imgOriginalON;
cout<<"Taking OFF in "<<timer<<" s"<<endl;

start = std::clock();
duration=0;
while(cam && duration<timer){
    cap>>imgOriginalOFF;
    duration = (clock() - start ) / (double) CLOCKS_PER_SEC;
}

cout<<"Taking OFF"<<endl;
if(cam)
	cap>>imgOriginalOFF;

Mat imgHSVON,imgHSVOFF;

 if(imgOriginalON.empty() || imgOriginalOFF.empty())
 {
     cout << "can not open " << endl;
     return -1;
 }

   Mat diff;
   absdiff(imgOriginalON,imgOriginalOFF,diff);
   cvtColor(diff, diff, COLOR_BGR2GRAY); //Convert the captured 
  
  threshold(diff, diff, thresh, 255, cv::THRESH_BINARY);
  dilate(diff, diff, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  //Initialize blobdetector with predefine parameters
  //lab computer version
  //SimpleBlobDetector blobDetect = SimpleBlobDetector(params);
  //blobDetect.detect( diff, keypoints );
  //opencv 3.0 version
  Ptr<SimpleBlobDetector> blobDetect = SimpleBlobDetector::create(params);
  blobDetect->detect( diff, keypoints );
Mat out;
  drawKeypoints(diff, keypoints, out, CV_RGB(0,0,0), DrawMatchesFlags::DEFAULT);
/*
  cvtColor(out, out, COLOR_BGR2HSV);
  inRange(out, Scalar(hsv.hL, hsv.sL, hsv.vL),
         Scalar(hsv.hH, hsv.sH, hsv.vH), out);
  blobDetect.detect( out, keypoints );
  drawKeypoints(out, keypoints, out, CV_RGB(0,0,0), DrawMatchesFlags::DEFAULT);

  for(int i=0;i<diff.rows;i++){
     for(int j=0;j<diff.cols;j++){
            if(out.at<Vec3b>(i,j)[0]==0 && out.at<Vec3b>(i,j)[1]==0 && out.at<Vec3b>(i,j)[2]==0){
                imgOriginalON.at<Vec3b>(i,j)=(0,0,0);
            }
     }
  }
  inRange(imgOriginalON, Scalar(hsv.hL, hsv.sL, hsv.vL),
         Scalar(hsv.hH, hsv.sH, hsv.vH), out);
  blobDetect.detect( out, keypoints );
  drawKeypoints(out, keypoints, out, CV_RGB(0,0,0), DrawMatchesFlags::DEFAULT);
*/
  //Circle blobs
  for(int i = 0; i < keypoints.size(); i++)
    {
	if(keypoints[i].size>0)
    	   circle(out, keypoints[i].pt, 1.5*keypoints[i].size, CV_RGB(0,255,0), 1, 8);
    }

   if(keypoints.size() == 4){
    text = "Object Found";
    cout<<endl<<endl<<"Object Found"<<endl;
    int xCord=((keypoints[0].pt.x)+(keypoints[1].pt.x))/2;
    int yCord=abs((keypoints[1].pt.y)+(keypoints[0].pt.y))/2;
    tilt_turn_degrees(diff, yCord, xCord, 1);
  }
  else{
    text = "Error";
    cout<<endl<<endl<<"No Object Found"<<endl;
	while(keypoints.size() > 2)
	   thresh+=5;
  }
  imshow("Original ON", imgOriginalON); //show the original image
  imshow("Original OFF", imgOriginalOFF); //show the original image
  imshow("Diff", out);
  waitKey(-1);
    
}
return 0;
}

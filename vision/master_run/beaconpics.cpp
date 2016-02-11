#include "beaconpics.h"
#include "functions.h"
#include "sys/time.h"

int beaconpics_main()
{
  int thresh=190;
  namedWindow("Original 1", WINDOW_NORMAL);
  namedWindow("Original 2", WINDOW_NORMAL);
  namedWindow("Original 3", WINDOW_NORMAL);
  namedWindow("Diff", WINDOW_NORMAL);

  //hsvParams hsv = {76,0,224,97,37,255};
  hsvParams hsv = {20,0,0,97,37,255};

  //Set up blob detection parameters
  SimpleBlobDetector::Params params;
 // params.blobColor //can we use this???
  params.minDistBetweenBlobs = 50.0f;
  params.filterByInertia = true;
  params.filterByConvexity = false;
  params.filterByColor = false;
  params.filterByCircularity = false;
  params.filterByArea = true;

        params.minThreshold = 190;
        params.maxThreshold = 255;
        params.thresholdStep = 1;

        params.minArea = 0;
        params.minConvexity = 0.3;
        params.minInertiaRatio = 0.10;

        params.maxArea = 2000;
        params.maxConvexity = 10;


  vector<KeyPoint> keypoints;

  VideoCapture cap(0); //capture the video from web cam

  if ( !cap.isOpened() )  // if not success, exit program
  {
       cout << "Cannot open the web cam" << endl;
       return -1;
  }

  while(true){

    Mat imgOriginal1 = getPic(cap);
    Mat imgOriginal2 = getPic(cap);
    Mat imgOriginal3 = getPic(cap);

    Mat imgHSV1,imgHSV2, imgHSV3;

    if(imgOriginal1.empty() || imgOriginal2.empty() || imgOriginal3.empty())
    {
       cout << "can not open " << endl;
       return -1;
    }

    Mat diff;
    absdiff(imgOriginal1,imgOriginal2,diff);
    cvtColor(diff, diff, COLOR_BGR2GRAY); //Convert the captured

    threshold(diff, diff, thresh, 255, cv::THRESH_BINARY);
    dilate(diff, diff, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //opencv 3.0 version
    //detect beacon blobs between pictures 1&2
    Ptr<SimpleBlobDetector> blobDetect = SimpleBlobDetector::create(params);
    blobDetect->detect( diff, keypoints );
    cout<<keypoints.size()<<endl;
    //detect blobs between images 2&3
    if(keypoints.size() ==0){
	absdiff(imgOriginal2,imgOriginal3,diff);
   	cvtColor(diff, diff, COLOR_BGR2GRAY); //Convert the captured

  	threshold(diff, diff, thresh, 255, cv::THRESH_BINARY);
  	dilate(diff, diff, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	blobDetect = SimpleBlobDetector::create(params);
   	blobDetect->detect( diff, keypoints );
    }
    cout<<keypoints.size()<<endl;

    Mat out;
    drawKeypoints(diff, keypoints, out, CV_RGB(0,0,0), DrawMatchesFlags::DEFAULT);
  /*//finding if things are green or red
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
    string text;
    if(keypoints.size() == 4)
    {
      text = "Object Found";
      cout<<endl<<endl<<"Object Found"<<endl;
      Point cent;
      cent=findkeyPoint(keypoints);
      cout<<"dist: "<<printDistanceFromLights(keypoints)<<endl;; 
      circle(out, cent, 5, CV_RGB(0,100,0), -1, 8);
      tilt_turn_degrees(diff, cent.y, cent.x, 1);
    }
    else
    {
      text = "Error";
      cout<<endl<<endl<<"No Object Found"<<endl;
  //	while(keypoints.size() > 2)
  //	   thresh+=5;
    }
    imshow("Original 1", imgOriginal1); //show the original image
    imshow("Original 2", imgOriginal2); //show the original image
    imshow("Original 3", imgOriginal3); //show the original image
    imshow("Diff", out);
    waitKey(-1);
  }
  return 0;
}

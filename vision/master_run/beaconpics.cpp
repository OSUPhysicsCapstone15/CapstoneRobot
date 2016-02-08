#include "beacon.h"
#include "functions.h"

Point findkeyPoint(vector<KeyPoint> keypoints){
  int left=keypoints[0].pt.x,right=keypoints[0].pt.x,top=keypoints[0].pt.y,bot=keypoints[0].pt.y;
  for(int i=1;i<keypoints.size();i++){
	if(keypoints[i].pt.x<left){
		left=keypoints[i].pt.x;
		top=keypoints[i].pt.y;
	}
	if(keypoints[i].pt.x>right){
		right=keypoints[i].pt.x;
		bot=keypoints[i].pt.y;
	}
  }

int xcent=(right+left)/2;
int ycent=(top+bot)/2;
return Point(xcent,ycent);

}

int beaconpics_main()
 {
int thresh=150;
  namedWindow("Original 1", WINDOW_AUTOSIZE);
  namedWindow("Original 2", WINDOW_AUTOSIZE);
  namedWindow("Original 3", WINDOW_AUTOSIZE);
  namedWindow("Diff", WINDOW_NORMAL);

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

    VideoCapture cap(0); //capture the video from web cam
    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }
  
Mat imgOriginal1, imgOriginal2, imgOriginal3;
clock_t start;
double duration=0;
double timer=.75;

while(true){

cout<<"Taking 1 in "<<timer<<" s"<<endl;
start = std::clock();
duration=0;
while(duration<timer){
    cap>>imgOriginal1;
    duration = (clock() - start ) / (double) CLOCKS_PER_SEC;
}
 cvtColor( imgOriginal1, imgOriginal1, COLOR_BGR2GRAY );
 imwrite( "../angles.jpg", imgOriginal1 );
imshow("Diff",imgOriginal1);
waitKey(-1);
cout<<"Taking 1"<<endl;
	cap>>imgOriginal1;

cout<<"Taking 2 in "<<timer<<" s"<<endl;

start = std::clock();
duration=0;
while(duration<timer){
    cap>>imgOriginal2;
    duration = (clock() - start ) / (double) CLOCKS_PER_SEC;
}

cout<<"Taking 2"<<endl;
	cap>>imgOriginal2;

cout<<"Taking 3 in "<<timer<<" s"<<endl;

start = std::clock();
duration=0;
while(duration<timer){
    cap>>imgOriginal3;
    duration = (clock() - start ) / (double) CLOCKS_PER_SEC;
}

cout<<"Taking 3"<<endl;
	cap>>imgOriginal3;

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
   if(keypoints.size() == 4){
    text = "Object Found";
    cout<<endl<<endl<<"Object Found"<<endl;
    Point cent;
    cent=findkeyPoint(keypoints);
    circle(out, cent, 5, CV_RGB(0,100,0), -1, 8);
    tilt_turn_degrees(diff, cent.y, cent.x);
  }
  else{
    text = "Error";
    cout<<endl<<endl<<"No Object Found"<<endl;
	while(keypoints.size() > 2)
	   thresh+=5;
  }
  imshow("Original 1", imgOriginal1); //show the original image
  imshow("Original 2", imgOriginal2); //show the original image
  imshow("Original 3", imgOriginal3); //show the original image
  imshow("Diff", out);
  waitKey(-1);
    
}
return 0;
}

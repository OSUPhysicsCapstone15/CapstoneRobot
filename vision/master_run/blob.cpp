#include "blob.h"
#include "functions.h"


int blob_main(int cam)
{
  hsvParams hsv = {0,0,200,180,90,255};

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

clock_t start;
double duration=0;

  const string filename("./webcam1.jpg");
  string text("Object not found");
  //Initialize camera
  VideoCapture cap;
  if(cam){
	  VideoCapture cap(1);
	  if ( !cap.isOpened() )  // if not success, exit program
	   {
	    cout << "Cannot open the web cam" << endl;
	    return -1;
	   }
   }

while(true){
  Mat img, imgHSV, imgTHRESH, out;
  img = imread(filename, CV_LOAD_IMAGE_COLOR);
  
  if(cam){
	cap>>img;
  }

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

  //Initialize blobdetector with predefine parameters
  //lab computer version
  SimpleBlobDetector blobDetect = SimpleBlobDetector(params);
  blobDetect.detect( imgTHRESH, keypoints );
  //opencv 3.0 version
//  Ptr<SimpleBlobDetector> blobDetect = SimpleBlobDetector::create(params);
//  blobDetect->detect( imgTHRESH, keypoints );

  drawKeypoints(imgTHRESH, keypoints, out, CV_RGB(0,0,0), DrawMatchesFlags::DEFAULT);
  //Circle blobs
  for(int i = 0; i < keypoints.size(); i++)
    circle(out, keypoints[i].pt, 1.5*keypoints[i].size, CV_RGB(0,255,0), 20, 8);

  if(keypoints.size() == 1){
    text = "Object Found";
    cout<<endl<<endl<<"Object Found"<<endl;
    tilt_turn_degrees(img, keypoints[0].pt.y, keypoints[0].pt.x);
  }
  if(keypoints.size() > 1){
    text = "Error";
    cout<<"No Object Found"<<endl;
  }

  putText(out, text, Point(100,200), FONT_HERSHEY_PLAIN, 20, Scalar(0, 0, 255), 20);

   duration=0;
   start = std::clock();
   while(duration<4){
       cap>>img;
       duration = (clock() - start ) / (double) CLOCKS_PER_SEC;
   }
}
return 0;
}



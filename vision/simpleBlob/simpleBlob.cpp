//#include <opencv2/core/core.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
	//For images
	/*
	// Read image
	string filename("/Users/rvansoelen/Documents/robot/res/samples/12025472_886928838064220_1656118452_n.jpg"); // by default
	if( argc > 1) {
		filename = argv[1];
    }
    
    ifstream ifile(filename);
	if (ifile) {
	  // The file exists, and is open for input
	  cout<<"The file exists"<<endl;
	}

    Mat image = imread(filename.c_str(), IMREAD_GRAYSCALE); // Read the file
    
    if( image.empty() )                      // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
	*/
	
	//For webcam
	
	VideoCapture cap(0); //Default camera
	if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the video file" << endl;
         return -1;
    }
   
    namedWindow("Window",CV_WINDOW_AUTOSIZE);
    Mat frame, grayFrame;
    
    // Set up the detector with some parameters.
	SimpleBlobDetector::Params params;
	//params.filterByColor = true;
	//params.blobColor = 225;
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	
	std::vector<KeyPoint> keypoints;
    
	while(1)
        {

	        if (!cap.read(frame)) //if not success, break loop
	        {
                        cout << "Frame dropped" << endl;	
                        continue;
	        }
		//Convert to grayscale
		//frame.convertTo(grayFrame, CV_BGR2GRAY);	
	 
		// Detect blobs.
		detector->detect( frame, keypoints);
 
		// Draw detected blobs as red circles.
		// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
		Mat im_with_keypoints;
		drawKeypoints(frame, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
 
		// Show blobs
		imshow("Window", frame);
		imshow("keypoints", im_with_keypoints );
		waitKey(30);
	}

}

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;

const float WIDTH = 1; //meters
const float HEIGHT = 1; //meters


beaconLocation(vector<KeyPoint> keyPoints, beacon_loc *b_loc) {
	//convert keypoints to point2f points
	vector<Point2f> imgPoints;
	KeyPoint.convert(keyPoints, imgPoints);

	//fill known points with beacon dimensions
	vector<Point2f> kwnPoints = {Point2f(0, HEIGHT/2), //Top
								Point2f(-WIDTH/2, 0),  //Left
								Point2f(WIDTH/2, 0), //Right
								Point2f(0, -HEIGHT/2) //Bottom
	}; 

	//get saved calibration matrix 
	//TODO

	//get roation-translation matrix
	Mat rvec, tvec;
	solvePnP(Mat(kwnPoints), Mat(imgPoints), cameraMatrix, distCoeffs, rvec, tvec, false);

	//print out stuff for sanity check
	cout<<"Rotation vector"<<endl;
	for(MatConstIterator_< _Tp > it = rvec.begin(); it != rvec.end(); it++) {
		cout<<*it<<endl;
	}
	cout<<"Translation vector"<<endl;
	for(MatConstIterator_< _Tp > it = tvec.begin(); it != tvec.end(); it++) {
		cout<<*it<<endl;
	}

	//fill beacon struct with appropriate values
	//TODO

}




int main(int argc, char* argv[]) {
	beacon_loc b_loc;
	//do stuff
	beaconLocation(points, &b_loc)
}
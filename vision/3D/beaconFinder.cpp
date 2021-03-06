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

const float WIDTH = 61; //inches
const float HEIGHT = 55; //inches

struct beacon_loc {
	float angle_from_robot=0; //degrees for all angles
	float distance=0; //meters
	float angle_from_beacon=0;
	bool only_bottom=0;
	bool beacon_not_found=0;
	bool beacon_angle_conf=0;
};

//returns true if success, false otherwise
bool beaconLocation(vector<KeyPoint> keyPoints, beacon_loc *b_loc) {
	//convert keypoints to point2f points
	vector<Point2f> imgPoints;
	KeyPoint::convert(keyPoints, imgPoints);

	//fill known points with beacon dimensions
	vector<Point3f> kwnPoints = {Point3f(0, HEIGHT/2, 0), //Top
								Point3f(-WIDTH/2, 0, 0),  //Left
								Point3f(WIDTH/2, 0, 0), //Right
								Point3f(0, -HEIGHT/2, 0) //Bottom
	}; 

	//get saved calibration matrix 
	string filename = "out_camera_data.xml";
	FileStorage fs(filename, FileStorage::READ);
	if(!fs.isOpened()) {
		cout<<"Calibration file could not be opened"<<endl;
		return false;
	}
	Mat cameraMatrix, distCoeffs;
	fs["Camera_Matrix"]>>cameraMatrix;
	fs["Distortion_Coefficients"]>>distCoeffs;
	if(cameraMatrix.empty() || distCoeffs.empty()) {
		cout<<"Calibration file not formatted correctly"<<endl;
		return false;
	} else {
		cout<<"Camera Matrix"<<endl;
		cout<<cameraMatrix<<endl;
		cout<<"Distortion Coefficients"<<endl;
		cout<<distCoeffs<<endl;
	}

	cout<<"Known Points"<<endl;
	cout<<kwnPoints<<endl;
	cout<<"Image Points"<<endl;
	cout<<imgPoints<<endl;

	//get rotation-translation matrix
	Mat rvec, tvec;
	solvePnP(Mat(kwnPoints), Mat(imgPoints), cameraMatrix, distCoeffs, rvec, tvec, false);

	//print out stuff for sanity check
	cout<<"Rotation vector"<<endl;
	cout<<rvec<<endl;
	cout<<"Translation vector"<<endl;
	cout<<tvec<<endl;

	//fill beacon struct with appropriate values
	//TODO

	return true;

}




int main(int argc, char* argv[]) {
	beacon_loc b_loc;
	vector<KeyPoint> points = {KeyPoint(460.97, 330.58, 1),
								KeyPoint(351.536, 208.6, 1),
								KeyPoint(350.863, 409.35, 1),
								KeyPoint(241.621, 325.604, 1)
	};
	//do stuff
	beaconLocation(points, &b_loc);

	return 0;
}

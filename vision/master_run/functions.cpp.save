#include "functions.h"


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
  int iLowH = 30;
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

void removenoise(Mat& image){
  //Morphologial opening
  erode(image,image,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
  dilate(image,image,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
  //Morphological closing
  dilate(image,image,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
  erode(image,image,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
  }

//get robot and camera angles
//img -> the current camera image
//objectRows -> rows from the left that the object is in
//objectCols -> columns from the top that the object is
//targetIsBeacon -> 1 if target is becon, 0 otherwise
void tilt_turn_degrees(Mat img, int object_rows, int object_cols, bool targetIsBeacon){

    double camera_height = 23.25;     						// height of camera from ground in inches
    double beacon_height = 30.5;
    double object_height = 1.0;
    double target_height;

    int rows = img.rows; 							// height of camera image in pixels
    int cols = img.cols; 							// width of camera image in pixels

    double aspectRatio = cols/rows;

    double horizontalFOV = 55.85; 						//horizontalFOV is a property of a specific camera
    double verticalFOV = 2 * atan(tan(horizontalFOV / 2) * aspectRatio); 	//conversion form horizontal to vertical FOV

    (targetIsBeacon) ? target_height = beacon_height : target_height = object_height;

    double verticalRatio = verticalFOV / rows;  				//ratio of real world degrees to pixels in the image
    double horizontalRatio = horizontalFOV / cols;

    int center_rows = rows / 2; 						// the center height is half of the total height
    int center_cols = cols / 2; 						// the center width is half of the total width

    int diff_rows = center_rows - object_rows; 					// difference between center and object rows
    int diff_cols = center_cols - object_cols; 					// difference between center and object cols

    double turn_robot_x_degrees = diff_cols * horizontalRatio; 			// positive -> turn left, negative -> turn right

    double tilt_camera_x_degrees = diff_rows * verticalRatio; 			// positive -> tilt up, negative -> tilt down

    double tilted_radians = tilt_camera_x_degrees * 3.1415962 / 180.0; 		// c++ tan() function uses radians

    double height_diff = target_height - camera_height; 			// height of camera from the beacon

    double distance = height_diff / tan(tilted_radians); 			// triangle formula for finding distance

    cout << "Turn robot " << turn_robot_x_degrees << " degrees." << endl;
    //cout << "Tilt camera " << tilt_camera_x_degrees << " degrees." << endl;
    cout << "Distance is " << distance << " inches" << endl;
}

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


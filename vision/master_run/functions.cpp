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
void tilt_turn_degrees(Mat img, int object_rows, int object_cols){

    double camera_height = 23.25;     // height of camera from ground in inches
    double beacon_height = 30.5;
    double camera_diagonal_angle = 75; // diagonal angle of view for camera in degrees
                                    // logitech c525 fov is 69 degrees, Samsung Galaxy S5 is 90 degrees

    int rows = img.rows; // height of camera image in pixels
    int cols = img.cols; // width of camera image in pixels

    double pixel_diagonal = sqrt(rows * rows + cols * cols); // (pythagorean) diagonal length of image in pixels
    double degrees_per_pixel = camera_diagonal_angle / pixel_diagonal; // ratio of real world degrees to pixels in the image

    int center_rows = rows / 2; // the center height is half of the total height
    int center_cols = cols / 2; // the center width is half of the total width

    int diff_rows = center_rows - object_rows; // difference between center and object rows
    int diff_cols = center_cols - object_cols; // difference between center and object cols

    double turn_robot_x_degrees = diff_cols * degrees_per_pixel; // positive -> turn left, negative -> turn right
    double tilt_camera_x_degrees = abs(diff_rows) * degrees_per_pixel; // positive -> tilt up, negative -> tilt down
    cout << "Turn robot " << turn_robot_x_degrees << " degrees.\n" << "Tilt camera " << tilt_camera_x_degrees << " degrees." << endl;

    double tilted_degrees = 90 - tilt_camera_x_degrees; // assuming camera is parallel to ground (90 degrees)

    double tilted_radians = tilted_degrees * 3.1415962 / 180.0; // c++ tan() function uses radians

    double height = beacon_height - camera_height; // height of camera from the beacon

    double distance = height * tan(tilted_radians); // triangle formula for finding distance

    cout << "Distance is " << distance << " inches" << endl;
}


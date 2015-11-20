

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <iostream>

using namespace cv;
using namespace std;

#define R_CAM 0
#define L_CAM 1

int main(int argc, char** argv)
{
    
    VideoCapture camr(R_CAM); // open the default camera
    VideoCapture caml(L_CAM);
    if(!camr.isOpened())  // check if we succeeded
        return -1;
    if(!caml.isOpened())  // check if we succeeded
        return -1;
    int count = 1;  //number of next pair of pictures
    while(1){
		Mat framer, framel, framel_g, framer_g;

        camr >> framer;
        caml >> framel;
		if (framel.empty() || framer.empty())
		{
		    printf("Frame Dropped");
		    continue;
		}
		cvtColor( framel, framel_g, CV_BGR2GRAY ); 
		cvtColor( framer, framer_g, CV_BGR2GRAY );

		namedWindow("left", WINDOW_NORMAL); //WINDOW_AUTOSIZE);
		imshow("left", framel_g);
		namedWindow("right", WINDOW_NORMAL); //WINDOW_AUTOSIZE);
		imshow("right", framer_g);
		fflush(stdout);  //needed?
		char k = waitKey(30);
		//cout<<"\""<<k<<"\""<<endl;
		if(k == ' '){
			//save images
			cout<<"Saving image pair "<<count<<endl;
			string n = "";
			if(count < 10)
				n = "0" + to_string(count);
			else if(count < 100)
				n = to_string(count);
			else {
				cout<<"Too many images."<<endl;
				return 0;	
			}
			imwrite("../res/stereo/left" + n + ".jpg", framel_g);
			imwrite("../res/stereo/right"+ n + ".jpg", framer_g);
			count++;
		} else if(k >= 0){
			cout<<"Exiting..."<<endl;
			break;
		}

    }
    return 0;
}

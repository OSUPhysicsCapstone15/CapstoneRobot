

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>

using namespace cv;

int main(int argc, char** argv)
{
    //const char* imgl_filename = "../res/stereo/left.png"; //01.jpg";
    //const char* imgr_filename = "../res/stereo/right.png"; //01.jpg";
    Rect roi1, roi2;
    Mat Q;
    Mat imglGray;
    Mat imgrGray;
    Mat disp, disp8;
    
    int SADWindowSize = 0, numberOfDisparities = 0;
    
    Ptr<StereoBM> bm = StereoBM::create(16,9);
    
    VideoCapture camr(0); // open the default camera
    VideoCapture caml(1);
    if(!camr.isOpened())  // check if we succeeded
        return -1;
    if(!caml.isOpened())  // check if we succeeded
        return -1;
    while(1){
    		Mat framer;
    		Mat framel;
        camr >> framer;
        caml >> framel;
		if (framel.empty())
		{
		    printf("could not load the left image");
		    continue;
		}
		
		if (framer.empty())
		{
		    printf("could not load the right image");
		    continue;
		}
		
		//Size img_size = img1.size();
		
		//numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
		
		//bm->setROI1(roi1);
		//bm->setROI2(roi2);
		//bm->setPreFilterCap(31);
		//bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
		//bm->setMinDisparity(0);
		//bm->setNumDisparities(numberOfDisparities);
		//bm->setTextureThreshold(10);
		//bm->setUniquenessRatio(15);
		//bm->setSpeckleWindowSize(100);
		//bm->setSpeckleRange(32);
		//bm->setDisp12MaxDiff(1);
	  
		cvtColor(framel, imglGray, CV_BGR2GRAY);
		cvtColor(framer, imgrGray, CV_BGR2GRAY);
		
		bm->compute(imglGray, imgrGray, disp);
		
		disp.convertTo(disp8, CV_8U); //, 255/(numberOfDisparities*16.));
		
		namedWindow("left", WINDOW_AUTOSIZE);
		imshow("left", framel);
		namedWindow("right", WINDOW_AUTOSIZE);
		imshow("right", framer);
		//namedWindow("disparity", WINDOW_AUTOSIZE);
		//imshow("disparity", disp);
		namedWindow("disparity 8", WINDOW_AUTOSIZE);
		imshow("disparity 8", disp8);
		fflush(stdout);  //needed?
		if(waitKey(30) >=0 ) break;
    }
    return 0;
}

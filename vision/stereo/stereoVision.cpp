
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>

using namespace cv;

int main(int argc, char** argv)
{
    const char* imgl_filename = "../res/stereo/left.png"; //01.jpg";
    const char* imgr_filename = "../res/stereo/right.png"; //01.jpg";
    Rect roi1, roi2;
    Mat Q;
    Mat imgl = imread(imgl_filename), imglGray;
    Mat imgr = imread(imgr_filename), imgrGray;
    Mat disp, disp8;
    
    int SADWindowSize = 0, numberOfDisparities = 0;
    
    Ptr<StereoBM> bm = StereoBM::create(64, 11);//16,9);
    
    if (imgl.empty())
    {
        printf("could not load the first image");
        return -1;
    }
    
    if (imgr.empty())
    {
        printf("could not load the second image");
        return -1;
    }
    
    //Size img_size = img1.size();
    
    //numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
    
    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setPreFilterCap(31);
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setMinDisparity(0);
    //bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(1);
  
    cvtColor(imgl, imglGray, CV_BGR2GRAY);
    cvtColor(imgr, imgrGray, CV_BGR2GRAY);
    
    bm->compute(imglGray, imgrGray, disp);
    
    disp.convertTo(disp8, CV_8U); //, 255/(numberOfDisparities*16.));
    
    namedWindow("left", WINDOW_AUTOSIZE);
    imshow("left", imgl);
    namedWindow("right", WINDOW_AUTOSIZE);
    imshow("right", imgr);
    namedWindow("disparity", WINDOW_AUTOSIZE);
    imshow("disparity", disp);
    namedWindow("disparity 8", WINDOW_AUTOSIZE);
    imshow("disparity 8", disp8);
    fflush(stdout);  //needed?
    waitKey();
    
    return 0;
}

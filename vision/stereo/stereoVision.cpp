
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>

using namespace cv;

int main(int argc, char** argv)
{
    const char* img1_filename = "/Users/rvansoelen/Documents/robot/res/depth/left.png"; //01.jpg";
    const char* img2_filename = "/Users/rvansoelen/Documents/robot/res/depth/right.png"; //01.jpg";
    Rect roi1, roi2;
    Mat Q;
    Mat img1 = imread(img1_filename), img1Gray;
    Mat img2 = imread(img2_filename), img2Gray;
    Mat disp, disp8;
    
    int SADWindowSize = 0, numberOfDisparities = 0;
    
    Ptr<StereoBM> bm = StereoBM::create(64, 11);//16,9);
    
    if (img1.empty())
    {
        printf("could not load the first image");
        return -1;
    }
    
    if (img2.empty())
    {
        printf("could not load the second image");
        return -1;
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
  
    cvtColor(img1, img1Gray, CV_BGR2GRAY);
    cvtColor(img2, img2Gray, CV_BGR2GRAY);
    
    bm->compute(img1Gray, img2Gray, disp);
    
    disp.convertTo(disp8, CV_8U); //, 255/(numberOfDisparities*16.));
    
    namedWindow("left", 1);
    imshow("left", img1);
    namedWindow("right", 1);
    imshow("right", img2);
    namedWindow("disparity", 0);
    imshow("disparity", disp);
    namedWindow("disparity 8", 0);
    imshow("disparity 8", disp8);
    fflush(stdout);
    waitKey();
    printf("\n");
    
    return 0;
}

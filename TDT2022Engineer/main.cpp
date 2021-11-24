#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "ksconnect.h"
using namespace std;
using namespace cv;
int main()
{
    ksconnect::ksDetect ksDetect_;
    VideoCapture Capture("/home/shelly/gitee/look/TDT2022Engineer/video/test.mp4");
    cv::Mat image;
    while (true)
    {
        double time = (double)getTickCount();
        Capture >> image;
        ksDetect_.Get(image);
        time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
        cout << "fps:" << 1 / time << "fps" << endl;
        waitKey(30);
    }
}
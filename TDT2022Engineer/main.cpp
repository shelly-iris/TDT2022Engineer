#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "ksconnect.h"
#include "usart.h"
using namespace std;
using namespace cv;
int main()
{
    ksconnect::ksDetect ksDetect_; //"/home/shelly/gitee/look/TDT2022Engineer/video/test.mp4"
    VideoCapture Capture(0);
    cv::Mat image;
    tdtusart::Send_Struct_t sendStruct;
    while (true)
    {
        double time = (double)getTickCount();
        Capture >> image;
        ksDetect_.Get(image, sendStruct);
        time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
        cout << "fps:" << 1 / time << "fps" << endl;
        waitKey(30);
    }
}
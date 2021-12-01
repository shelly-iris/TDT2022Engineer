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
    VideoCapture Capture(2);
    cv::Mat image;
    tdtusart::Send_Struct_t sendStruct;
    engineer_tool::ReceiveMessage receiveMessage;
    tdtusart::Recv_Struct_t recvStruct;

    tdtusart::RealUsart realUsart;
    tdtusart::Usart &usart = realUsart;
    while (true)
    {
        double time = (double)getTickCount();
        engineer_tool::TransformRecv(recvStruct, receiveMessage);
        int usart_ret = usart.UsartRecv(recvStruct);
        cout << "usart_ret " << usart_ret << endl;
     //   sendStruct.sendAngle1 = 250;
   // sendStruct.judgeFlag = 2;
        int usart_send = usart.UsartSend(sendStruct);
       // cout << "* " << usart_send << endl;

        // cout << "usart" << sendStruct.sendAngle1 << endl;
        Capture >> image;
        ksDetect_.Get(image, sendStruct);
        time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
        // cout << "fps:" << 1 / time << "fps" << endl;
        waitKey(1);
    }

    return 0;
}
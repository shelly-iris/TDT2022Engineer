#ifndef KSCONECT_H
#define KSCONECT_H

#include <algorithm>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "tool.h"
namespace ksconnect
{

    class ksDetect
    {
    public:
        ksDetect()
        {
        }
        ~ksDetect() {}
        void getksBar(cv::Mat &src);
        void Get(cv::Mat &img);
        void aroundJudge(cv::Mat &img,Point &center);
        void modelJudge(cv::Mat &img, std::vector<engineer_tool::modelL> &L_LU,
                        std::vector<engineer_tool::modelL> &L_LD, std::vector<engineer_tool::modelL> &L_RD,
                        std::vector<engineer_tool::modelL> &L_RU, std::vector<engineer_tool::modelL> &square);
        // void modelJudge(std::vector<engineer_tool::CustomRect> &modelks);
        cv::Mat ksoperate;
        int judgeFlag = 0;
        float sendAngle1 = 0;
        float sengAngle2 = 0;
        std::vector<engineer_tool::modelL> allKsbars;
        //粗筛选轮廓
        std::vector<engineer_tool::modelL> L_LU;
        //L型左上
        std::vector<engineer_tool::modelL> L_LD;
        //L型左下
        std::vector<engineer_tool::modelL> L_RU;
        //L型右上
        std::vector<engineer_tool::modelL> L_RD;
        //L型右下
        std::vector<engineer_tool::modelL> square;
        //正方形
        std::vector<engineer_tool::modelL> resultks;
        //筛选出来的结果
    };

}

#endif
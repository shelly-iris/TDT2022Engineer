#include "ksconnect.h"
#include "tool.h"
#include <cmath>
using namespace std;
using namespace cv;
#define minarea 50
#define maxarea 20000

namespace ksconnect
{
    void ksDetect::getksBar(cv::Mat &src)
    {
        Mat pyr, img, img_thresholded;
        pyrDown(src, pyr, Size(src.cols / 2, src.rows / 2)); //高斯降噪，并只取奇数行列缩小图片    // 缩小和放大图像以滤除噪音
        pyrUp(pyr, src, src.size());                         //插入偶数行列，再次高斯降噪
        cvtColor(src, img, COLOR_BGR2GRAY);
        threshold(img, img_thresholded, 120, 255, THRESH_BINARY); //190
        Canny(img_thresholded, img_thresholded, 100, 200, 3);
        this->ksoperate = img_thresholded.clone();
        imshow("operate", ksoperate);
    }
    static double angle(Point pt1, Point pt2, Point pt0) //三点形成的角度
    {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt0.x - pt2.x;
        double dy2 = pt0.y - pt2.y;
        return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
        //cos<a,b>=(ab的内积)/(|a||b|)
    }
    static double angle22(Point pt1, Point pt2, Point pt0) //三点形成的角度
    {
        double theta = atan2(pt1.x - pt0.x, pt1.y - pt0.y) - atan2(pt2.x - pt0.x, pt2.y - pt0.y);
        if (theta > M_PI)
            theta -= 2 * M_PI;
        if (theta < -M_PI)
            theta += 2 * M_PI;

        return theta;
        //cos<a,b>=(ab的内积)/(|a||b|)
    }
    double distance(cv::Point a, cv::Point b)
    {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }
    static double getSendangle(Point centerPoint, Point squarePoint)
    {
        double sendAngle22 = 0;
        cv::Point2f hypothetical = cv::Point2f(0, 0); //假设点
        hypothetical.x = centerPoint.x + 30;
        hypothetical.y = centerPoint.y;

        sendAngle22 = angle22(hypothetical, squarePoint, centerPoint) * 57.3f;

        return sendAngle22; //+45?
    }
    void ksDetect::Get(cv::Mat &img)
    {

        vector<vector<Point>> all_contours;
        getksBar(img);
        findContours(this->ksoperate, all_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        vector<vector<Point>> contours_poly(all_contours.size()); //用于存放折线点集
        //   vector<engineer_tool::modelL> contours_poly(all_contours.size()); //用于存放折线点集
        for (int i = 0; i < (int)all_contours.size(); i++)
        {
            approxPolyDP(Mat(all_contours[i]), contours_poly[i], arcLength(all_contours[i], true) * 0.04, true);
        }
        for (const auto &contours : contours_poly)
        {
            engineer_tool::modelL contour(contours);
            std::vector<cv::Point2f> Points_;
            Points_ = contour.GetVertices2f();
            const int Area_difference = 7000; //350
            bool rec_condition1 = false;
            bool l_condition1 = false;
            bool l_condition2 = false;
            float area = contour.GetArea();
            if (area < minarea || area > maxarea)
                continue;
            if (contour.GetNum() == 4 && isContourConvex(Mat(contours)))
            {
                double maxCosine22 = 0;
                for (int j = 2; j < 5; j++) //012,123,230
                {
                    double cosine = fabs(angle(Points_[j % 4], Points_[j - 2], Points_[j - 1]));
                    maxCosine22 = MAX(maxCosine22, cosine);
                    //  cout << "beta" << maxCosine22 << endl;
                }
                if (maxCosine22 < 0.3)
                {
                    square.emplace_back(contour);
                }
                // double beta = 0, alfa = 0;
                // if ((contour[0].y - contour[2].y) == 0)
                //     continue;
                // if ((contour[1].y - contour[3].y) == 0)
                //     continue;
                // beta = abs(contour[0].x - contour[2].x) / abs(contour[0].y - contour[2].y);
                // alfa = abs(contour[1].x - contour[3].x) / abs(contour[1].y - contour[3].y);
                // cout << "beta" << beta << endl;
                // if (abs(beta) < 3 && abs(alfa) < 3)
                // {
                //     square.emplace_back(contour);
                // }
            }
            if ((contour.GetNum() == 6 && !isContourConvex(Mat(contours))))
            {
                int a = pow((Points_[0].x - Points_[4].x), 2) + pow((Points_[0].y - Points_[4].y), 2);
                int b = pow((Points_[0].x - Points_[2].x), 2) + pow((Points_[0].y - Points_[2].y), 2);
                double length1 = sqrt(pow(Points_[0].x - Points_[1].x, 2) + pow(Points_[0].y - Points_[1].y, 2));
                double length2 = sqrt(pow(Points_[0].x - Points_[5].x, 2) + pow(Points_[0].y - Points_[5].y, 2));
                double area = pow(std::max(length1, length2), 2);
                cv::Point l_center;
                l_center.x = (Points_[0].x + Points_[4].x + Points_[2].x) / 3.0;
                l_center.y = (Points_[0].y + Points_[4].y + Points_[2].y) / 3.0;

                // if (abs(contour.GetArea() - area * 0.56) < Area_difference)
                if (abs(1 - contour.GetArea() / area * 0.56) < 0.46)
                {
                    l_condition2 = true;
                }

                if (a >= b) //0与4为对角点
                {

                    if (l_center.x < Points_[2].x && l_center.y < Points_[2].y)
                    {
                        if ((Points_[4].y + Points_[5].y > Points_[3].y + Points_[2].y) && (Points_[3].y + Points_[2].y > Points_[1].y + Points_[0].y))
                            if (abs((Points_[5].x - Points_[4].x) - (Points_[5].y - Points_[0].y)) < 15)
                            {
                                l_condition1 = true;
                                if (l_condition1 && l_condition2)
                                    L_RD.emplace_back(contour);
                            }
                    }

                    else if (l_center.x < Points_[2].x && l_center.y > Points_[2].y)
                    {
                        if (Points_[3].y + Points_[4].y > Points_[1].y + Points_[2].y && (Points_[1].y + Points_[2].y > Points_[0].y + Points_[5].y))
                            if (abs((Points_[5].x - Points_[0].x) - (Points_[4].y - Points_[5].y)) < 15)
                            {
                                l_condition1 = true;
                                if (l_condition1 && l_condition2)
                                    L_RU.emplace_back(contour);
                            }
                    }
                    else if (l_center.x > Points_[2].x && l_center.y > Points_[2].y)
                    {
                        if (Points_[0].y + Points_[1].y > Points_[2].y + Points_[3].y && (Points_[2].y + Points_[3].y > Points_[5].y + Points_[4].y))
                            if (abs((Points_[0].y - Points_[5].y) - (Points_[4].x - Points_[5].x)) < 15)
                            {
                                l_condition1 = true;
                                if (l_condition1 && l_condition2)
                                    L_LU.emplace_back(contour);
                            }
                    }
                }
                else
                {

                    if (l_center.x > Points_[4].x && l_center.y < Points_[4].y)
                    {
                        if (Points_[1].y + Points_[2].y > Points_[4].y + Points_[3].y && Points_[4].y + Points_[3].y > Points_[0].y + Points_[5].y)
                            if (abs((Points_[2].x - Points_[1].x) - (Points_[1].y - Points_[0].y)) < 15)
                            {
                                l_condition1 = true;
                                if (l_condition1 && l_condition2)
                                    L_LD.emplace_back(contour);
                            }
                    }
                    else if (l_center.x > Points_[4].x && l_center.y > Points_[4].y)
                    {
                        if (Points_[2].y + Points_[3].y > Points_[4].y + Points_[5].y && Points_[4].y + Points_[5].y > Points_[0].y + Points_[1].y)
                            if (abs((Points_[2].y - Points_[1].y) - (Points_[0].x - Points_[1].x)) < 15)
                            {
                                l_condition1 = true;
                                if (l_condition1 && l_condition2)
                                    L_LU.emplace_back(contour);
                            }
                    }

                } //0，2位角点

            } //六边形判断
            // allKsbars.emplace_back(contour);

            //   if(l_condition1 && l_condition2)
            //   {
            //        drawContours(img, contours, -1, cv::Scalar(0, 0, 255), 2, 8);
            //   }
            // else continue;
        }
        //   assert(L_RU.size());
        // if (L_RU.size() == 0)
        // {
        //     return;
        // }
        modelJudge(img, L_LU, L_LD, L_RD, L_RU, square);
        /**
         * 用做调试时绘制轮廓，正式版可删除
         */
        std::vector<std::vector<Point>> L1;
        std::vector<std::vector<Point>> L2;
        std::vector<std::vector<Point>> L3;
        std::vector<std::vector<Point>> L4;
        std::vector<std::vector<Point>> square0;
        for (auto &it : L_RU)
        {
            L1.push_back(L_RU.back().getCount());
        }
        for (auto &it : L_LD)
        {
            L2.push_back(L_LD.back().getCount());
        }
        for (auto &it : L_LU)
        {
            L3.push_back(L_LU.back().getCount());
        }
        for (auto &it : L_RD)
        {
            L4.push_back(L_RD.back().getCount());
        }
        for (auto &it : square)
        {
            square0.push_back(square.back().getCount());
        }
        //  cout << L1.size() << endl;
        drawContours(img, square0, -1, cv::Scalar(0, 0, 255), 2, 8);
        drawContours(img, L1, -1, cv::Scalar(0, 0, 255), 2, 8);
        drawContours(img, L2, -1, cv::Scalar(0, 0, 255), 2, 8);
        drawContours(img, L3, -1, cv::Scalar(0, 0, 255), 2, 8);
        drawContours(img, L4, -1, cv::Scalar(0, 0, 255), 2, 8);

        imshow("origin", img);
        L_RU.clear();
        L_LD.clear();
        L_LU.clear();
        L_RD.clear();
        square.clear();
    }
    void ksDetect::aroundJudge(Mat &img,Point &center)
    {
        
    }
    void ksDetect::modelJudge(Mat &img, std::vector<engineer_tool::modelL> &L_LU,
                              std::vector<engineer_tool::modelL> &L_LD, std::vector<engineer_tool::modelL> &L_RD,
                              std::vector<engineer_tool::modelL> &L_RU, std::vector<engineer_tool::modelL> &square)
    {
        //diagonal=对角线，standardLen=标志块基础长度（rec边长，L型最长的边）

        const int Range = 30; //30
        double diagonal,
            standardLen,
            value;
        for (const auto &ldSingle : L_LD) //左下右上匹配
        {
            for (const auto &ruSingle : L_RU)
            {
                diagonal = distance(ldSingle.center_, ruSingle.center_);
                standardLen = ldSingle.cross_;
                value = diagonal / standardLen;
                // cout << "standardLen" << standardLen << endl;
                // cout << "diagonal" << diagonal << endl;
                cout << "value" << value << endl;
                if (value < 7 && value > 5) //75
                    if (ldSingle.center_.y > ruSingle.center_.y)
                        if (ruSingle.center_.x > ldSingle.center_.x)
                        {
                            cv::line(img, ldSingle.center_, ruSingle.center_, cv::Scalar(0, 0, 255), 2);
                            cv::Point center; //当前 匹配的中心点
                            center.x = (ruSingle.center_.x + ldSingle.center_.x) * 0.5;
                            center.y = (ldSingle.center_.y + ruSingle.center_.y) * 0.5;

                            if (L_LU.size() == 0 && L_RD.size() == 0)
                            {
                                if (square.size() > 1)
                                {
                                    for (const auto &squareSingle : square)
                                    {
                                        if (distance(squareSingle.center_, center) > diagonal * 0.5 - Range && distance(squareSingle.center_, center) < diagonal * 0.5 + Range)
                                        {
                                            //to do哪一个面
                                            aroundJudge(img,center);
                                            cv::putText(img, "opposite", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
                                        }
                                    }
                                }
                            }
                            if (L_RD.size()) //左下，右下，右上匹配
                            {
                                for (const auto &rdSingle : L_RD)
                                {
                                    if (distance(center, rdSingle.center_) < diagonal * 0.5 + Range && distance(center, rdSingle.center_) > diagonal * 0.5 - Range)
                                        if (rdSingle.center_.x > center.x && rdSingle.center_.y < center.y)
                                        {
                                            cv::line(img, center, rdSingle.center_, cv::Scalar(0, 0, 255), 2);
                                            for (const auto &squareSingle : square)
                                            { //匹配正方形
                                                //
                                                if (distance(squareSingle.center_, center) > diagonal * 0.5 - Range && distance(squareSingle.center_, center) < diagonal * 0.5 + Range)
                                                {
                                                    if (squareSingle.center_.x < center.x && squareSingle.center_.y < center.y)
                                                    { //rec在左上

                                                        this->sendAngle1 = getSendangle(center, squareSingle.center_)+45;

                                                        this->judgeFlag = 3;
                                                        cv::line(img, center, squareSingle.center_, cv::Scalar(0, 0, 255), 2);
                                                        cv::putText(img, "UP", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
                                                    }
                                                }
                                            }
                                        }
                                }
                            }
                            if (L_LU.size()) //左上，右下，右上匹配
                            {
                                for (const auto &luSingle : L_LU)
                                {
                                    if (distance(center, luSingle.center_) < diagonal * 0.5 + Range && distance(center, luSingle.center_) > diagonal * 0.5 - Range)
                                        if (luSingle.center_.x < center.x && luSingle.center_.y < center.y)
                                        {
                                            cv::line(img, center, luSingle.center_, cv::Scalar(0, 0, 255), 2);
                                            for (const auto &squareSingle : square)
                                            { //匹配正方形

                                                if (distance(squareSingle.center_, center) > diagonal * 0.5 - Range && distance(squareSingle.center_, center) < diagonal * 0.5 + Range)
                                                {
                                                    if (squareSingle.center_.x > center.x && squareSingle.center_.y > center.y)
                                                    { //rec在右下
                                                        this->sendAngle1 = 45-getSendangle(center, squareSingle.center_);

                                                        this->judgeFlag = 3;
                                                        cv::line(img, center, squareSingle.center_, cv::Scalar(0, 0, 255), 2);
                                                        cv::putText(img, "DOWN", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
                                                    }
                                                }
                                            }
                                        }
                                }
                            }
                        }
            }
        }
        for (const auto &luSingle : L_LU) //左上右下匹配
        {
            for (const auto &rdSingle : L_RD)
            {
                diagonal = distance(luSingle.center_, rdSingle.center_);
                standardLen = luSingle.cross_;
                value = diagonal / standardLen;
                // cout << "standardLen" << standardLen << endl;
                // cout << "diagonal" << diagonal << endl;
                cout << "value" << value << endl;
                if (value < 7 && value > 5) //75
                    if (luSingle.center_.y < rdSingle.center_.y)
                        if (rdSingle.center_.x > luSingle.center_.x)
                        {
                            cv::line(img, luSingle.center_, rdSingle.center_, cv::Scalar(0, 0, 255), 2);
                            cv::Point center; //当前 匹配的中心点
                            center.x = (rdSingle.center_.x + luSingle.center_.x) * 0.5;
                            center.y = (luSingle.center_.y + rdSingle.center_.y) * 0.5;

                            if (L_RU.size() == 0 && L_LD.size() == 0)
                            {
                                if (square.size() > 1)
                                {
                                    for (const auto &squareSingle : square)
                                    {
                                        if (distance(squareSingle.center_, center) > diagonal * 0.5 - Range && distance(squareSingle.center_, center) < diagonal * 0.5 + Range)
                                        {
                                            //to do哪一个面
                                            cv::putText(img, "opposite", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
                                        }
                                    }
                                }
                            }
                            if (L_RU.size()) //左上，右下，右上匹配
                            {
                                for (const auto &ruSingle : L_RU)
                                {
                                    if (distance(center, ruSingle.center_) < diagonal * 0.5 + Range && distance(center, ruSingle.center_) > diagonal * 0.5 - Range)
                                        if (ruSingle.center_.x > center.x && ruSingle.center_.y < center.y)
                                        {
                                            cv::line(img, center, ruSingle.center_, cv::Scalar(0, 0, 255), 2);
                                            for (const auto &squareSingle : square)
                                            { //匹配正方形
                                                //
                                                if (distance(squareSingle.center_, center) > diagonal * 0.5 - Range && distance(squareSingle.center_, center) < diagonal * 0.5 + Range)
                                                {
                                                    if (squareSingle.center_.x < center.x && squareSingle.center_.y > center.y)
                                                    { //rec在左下
                                                        this->sendAngle1 = getSendangle(center, squareSingle.center_)+45;

                                                        this->judgeFlag = 3;
                                                        cv::line(img, center, squareSingle.center_, cv::Scalar(0, 0, 255), 2);
                                                        cv::putText(img, "LEFT", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
                                                    }
                                                }
                                            }
                                        }
                                }
                            }
                            if (L_LD.size()) //左上，左下，右下匹配
                            {
                                for (const auto &ldSingle : L_LD)
                                {
                                    if (distance(center, ldSingle.center_) < diagonal * 0.5 + Range && distance(center, ldSingle.center_) > diagonal * 0.5 - Range)
                                        if (ldSingle.center_.x < center.x && ldSingle.center_.y > center.y)
                                        {
                                            cv::line(img, center, ldSingle.center_, cv::Scalar(0, 0, 255), 2);
                                            for (const auto &squareSingle : square)
                                            { //匹配正方形

                                                if (distance(squareSingle.center_, center) > diagonal * 0.5 - Range && distance(squareSingle.center_, center) < diagonal * 0.5 + Range)
                                                {
                                                    if (squareSingle.center_.x > center.x && squareSingle.center_.y < center.y)
                                                    { //rec在右上
                                                        this->sendAngle1 = getSendangle(center, squareSingle.center_)+45;

                                                        this->judgeFlag = 3;
                                                        cv::line(img, center, squareSingle.center_, cv::Scalar(0, 0, 255), 2);
                                                        cv::putText(img, "RIGHT", center, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 5, 8, 0);
                                                    }
                                                }
                                            }
                                        }
                                }
                            }
                        }
            }
        }
        cout << "angle" << this->sendAngle1 << endl;
    }
}

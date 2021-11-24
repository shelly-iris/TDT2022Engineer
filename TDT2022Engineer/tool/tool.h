#ifndef TDTENGINEER_TOOL_H
#define TDTENGINEER_TOOL_H

#include "usart.h"
#include <algorithm>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace engineer_tool
{
    class CustomRect
    {
    public:
        /**
         * 用来构造一个空的类
         */
        CustomRect() = default; //编译器能够为 =default;的函数自动生成函数体
        /**
         * 从RotatedRect来的构造函数
         * @param rRect 用来构造的RotatedRect
         */
        explicit CustomRect(const cv::RotatedRect &r_rect);
        /**
         * 从点集构造,最小外接矩形
         * @param inputContour 用来构造的点集
         */
        explicit CustomRect(const std::vector<cv::Point> &input_contour);

        CustomRect(const cv::Point2f &center, const cv::Size2f &size, float angle);
        CustomRect(const cv::Point &center, const cv::Size &size, float angle);

        CustomRect(const CustomRect &c_rect) = default;

        inline cv::Size2f GetSize() const { return this->size_; };
        inline void SetSize(cv::Size const &size)
        {
            this->size_ = size;
            FixWidthAndHeight(this->size_.width, this->size_.height, this->angle_);
            UpdateVertices();
        };
        inline float GetAngle() const { return this->angle_; };
        inline void SetAngle(float angle)
        {
            this->angle_ = angle;
            UpdateVertices();
        };
        ;
        inline void setFindStatus(bool flag) { this->find_status = flag; }
        inline void setLinkStatus(int num) { this->link_status = num; }
        inline void setTag(int num) { this->tag = num; }
        inline void setCoordinition_number(int num) { this->coordinition_number = num; }
        inline void SetCenter(cv::Point2d const &center)
        {
            this->center_ = center;
            UpdateVertices();
        };

        inline float Vertical() const { return fmax(this->bl_.y, this->br_.y) - fmin(this->tl_.y, this->tr_.y); }; //
        inline float Cross() const { return fmax(this->tr_.x, this->br_.x) - fmin(this->tl_.x, this->bl_.x); };
        inline bool getStatus() const { return this->find_status; }
        inline int getLinkStatus() const { return this->link_status; }
        inline float GetWidth() const { return this->size_.width; };
        inline float GetHeight() const { return this->size_.height; };
        inline float GetArea() const { return this->size_.area(); }; //计算灯条的面积，用于大小排序，仅用于增强开火决策跟踪效果
        inline std::vector<cv::Point> GetVertices() const { return {this->bl_, this->tl_, this->tr_, this->br_}; };
        inline std::vector<cv::Point2f> GetVertices2f() const { return {this->bl_, this->tl_, this->tr_, this->br_}; };
        inline cv::Rect GetRect() const { return cv::boundingRect(std::vector<cv::Point2i>({this->bl_, this->tl_, this->tr_, this->br_})); };
        inline cv::RotatedRect GetRotRect() const { return cv::minAreaRect(std::vector<cv::Point2i>({this->bl_, this->tl_, this->tr_, this->br_})); };
        inline cv::Point GetCenter() const { return this->center_; };
        inline cv::Point2f GetCenter2f() const { return this->center_; };
        inline cv::Point2f GetTl() const { return this->tl_; };
        inline cv::Point2f GetBl() const { return this->bl_; };
        inline cv::Point2f GetTr() const { return this->tr_; };
        inline cv::Point2f GetBr() const { return this->br_; };
        inline int getTag() const { return this->tag; };
        inline int getCoordinition_number() const { return this->coordinition_number; }
        inline void showMarks(cv::Point2f p, int num, cv::Mat &image) { cv::putText(image, std::to_string(num), p, cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 0, 255), 2, 8); }
        inline int get_pointnum() const { return this->pointnum_; }

        std::vector<cv::Point> getCOunt() const
        {

            return m_points;
        }

    protected:
        /**
         * @brief 通过size, center, angle 来计算出 四个顶点
         */
        void UpdateVertices();

        inline void FixWidthAndHeight(float &width, float &height, float &angle)
        {
            if (width < height)
            {
                angle = angle - 90;
                //交换两个数
                float tmp;
                tmp = height;
                height = width;
                width = tmp;
            }
            angle = fabs(angle_); //取绝对值
        }
        static bool compare(cv::Point x, cv::Point y)
        {
            return x.x < y.x;
        }

        void get_center(int num, std::vector<std::vector<cv::Point>> &model_points, std::vector<cv::Point> &center)
        {
            for (int i = 0; i < num; i++)
            {
                cv::Point x;
                x.x = (model_points[i][0].x + model_points[i][2].x + model_points[i][4].x) / 3.0;
                x.y = (model_points[i][0].y + model_points[i][2].y + model_points[i][4].y) / 3.0;
                center.push_back(x);
            }
            std::sort(center.begin(), center.end(), compare);
        }

        //外部可读可写变量
        cv::Size2f size_ = cv::Size2f(0, 0);
        float angle_ = 0;
        cv::Point2f center_ = cv::Point2f(0, 0);
        int pointnum_ = 0;
        bool find_status = 0;
        int coordinition_number = 0; //每个灯条的配位数
        int tag;
        std::vector<cv::Point> m_points;
        //相邻灯条的连接状态，0代表全连接，中间没有灯条，1代表半连接，中间一个灭的灯条
        // 2代表虚连接，中间两个灭的灯条，3代表拉跨连接，中间三个灭的灯条
        int link_status = -1;

        //外部只读变量
        /**
         * 四个顶点
         */
        cv::Point2f tl_ = cv::Point2f(0, 0);
        cv::Point2f bl_ = cv::Point2f(0, 0);
        cv::Point2f tr_ = cv::Point2f(0, 0);
        cv::Point2f br_ = cv::Point2f(0, 0);

    public:
        /**
         * @brief 找出点集在对应angle角度的最小外接矩形
         * @param _points 点集
         * @param angle 角度
         * @return 矩形
         */
        static CustomRect minCustomRect(cv::InputArray _points, float angle);
    };
    class modelL
    {
    public:
        modelL() = default; //编译器能够为 =default;的函数自动生成函数体

        /**
         * 从点集构造,最小外接矩形
         * @param inputContour 用来构造的点集
         */
        explicit modelL(const std::vector<cv::Point> &input_contour);
        inline float GetArea() const { return this->size_.area(); };
        inline int GetNum() const { return this->numb_; };
        inline std::vector<cv::Point2f> GetVertices2f() const { return {this->p0, this->p1, this->p2, this->p3, this->p4, this->p5}; };
        std::vector<cv::Point> getCount() const { return m_points; }
        cv::Point getcenter() const { return this->center_; }
        // inline float Cross() const { return fmax(this->tr_.x, this->br_.x) - fmin(this->tl_.x, this->bl_.x); };
        cv::Point2f p0 = cv::Point2f(0, 0);
        cv::Point2f p1 = cv::Point2f(0, 0);
        cv::Point2f p2 = cv::Point2f(0, 0);
        cv::Point2f p3 = cv::Point2f(0, 0);
        cv::Point2f p4 = cv::Point2f(0, 0);
        cv::Point2f p5 = cv::Point2f(0, 0);
        cv::Point2f center_ = cv::Point2f(0, 0);
        cv::Size2f size_ = cv::Size2f(0, 0);
        int numb_ = 0;
        std::vector<cv::Point> m_points;
        int maxpoint = 0;
        int minpoint =100000000;
        int cross_;
    };
    class modelKs
    {
    public:
        std::vector<cv::Point> center;
        std::vector<std::vector<cv::Point>> model_points;
        int num;
        static bool compare(cv::Point x, cv::Point y)
        {
            return x.x < y.x;
        }

        int get_num()
        {
            num = model_points.size();
            return num;
        }

        void get_center(int num, std::vector<std::vector<cv::Point>> &model_points, std::vector<cv::Point> &center)
        {
            for (int i = 0; i < num; i++)
            {
                cv::Point x;
                x.x = (model_points[i][0].x + model_points[i][2].x + model_points[i][4].x) / 3.0;
                x.y = (model_points[i][0].y + model_points[i][2].y + model_points[i][4].y) / 3.0;
                center.push_back(x);
            }
            std::sort(center.begin(), center.end(), compare);
        }

        void clear()
        {
            model_points.clear();
            center.clear();
        }
    };
}

#endif
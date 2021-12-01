#include "tool.h"
#include <cmath>
using namespace std;
using namespace cv;

namespace engineer_tool
{

    /**
     * @class   CustomRect
     */

    CustomRect::CustomRect(const cv::RotatedRect &r_rect)
    {
        this->size_ = r_rect.size;
        this->angle_ = r_rect.angle;
        this->center_ = r_rect.center;
        FixWidthAndHeight(this->size_.width, this->size_.height, this->angle_);
        this->UpdateVertices();
    }

    CustomRect::CustomRect(const std::vector<cv::Point> &input_contour)
    {
        m_points.clear();
        m_points.insert(m_points.begin(), input_contour.begin(), input_contour.end());
        // std::copy();
        cv::RotatedRect r_rect = cv::minAreaRect(input_contour);
        this->size_ = r_rect.size;
        this->angle_ = r_rect.angle;
        this->center_ = r_rect.center;
        this->pointnum_ = input_contour.size();
        FixWidthAndHeight(this->size_.width, this->size_.height, this->angle_);
        this->UpdateVertices();
    }

    CustomRect::CustomRect(const cv::Point2f &center, const cv::Size2f &size, float angle)
    {
        this->center_ = center;
        this->size_ = size;
        this->angle_ = angle;
        FixWidthAndHeight(this->size_.width, this->size_.height, this->angle_);
        this->UpdateVertices();
    }

    CustomRect::CustomRect(const cv::Point &center, const cv::Size &size, float angle)
    {
        this->center_ = center;
        this->size_ = size;
        this->angle_ = angle;
        FixWidthAndHeight(this->size_.width, this->size_.height, this->angle_);
        this->UpdateVertices();
    }

    void CustomRect::UpdateVertices()
    {
        float r = pow((float(this->size_.width * this->size_.width) / 4.f) + (float(this->size_.height * this->size_.height) / 4.f), 0.5);
        double a = double(this->angle_) * CV_PI / 180.f;
        double b = atan2(this->size_.height, this->size_.width);
        if (this->angle_ < 45)
        {
            this->bl_ = cv::Point2f(float(this->center_.x) - r * cos(a + b), float(this->center_.y) + r * sin(a + b));
            this->tl_ = cv::Point2f(float(this->center_.x) - r * cos(a - b), float(this->center_.y) + r * sin(a - b));
            this->tr_ = cv::Point2f(float(this->center_.x) + r * cos(a + b), float(this->center_.y) - r * sin(a + b));
            this->br_ = cv::Point2f(float(this->center_.x) + r * cos(a - b), float(this->center_.y) - r * sin(a - b));
            return;
        }
        if (this->angle_ < 90)
        {
            this->br_ = cv::Point2f(float(this->center_.x) - r * cos(a + b), float(this->center_.y) + r * sin(a + b));
            this->bl_ = cv::Point2f(float(this->center_.x) - r * cos(a - b), float(this->center_.y) + r * sin(a - b));
            this->tl_ = cv::Point2f(float(this->center_.x) + r * cos(a + b), float(this->center_.y) - r * sin(a + b));
            this->tr_ = cv::Point2f(float(this->center_.x) + r * cos(a - b), float(this->center_.y) - r * sin(a - b));
            return;
        }
        if (this->angle_ < 135)
        {
            this->br_ = cv::Point2f(float(this->center_.x) - r * cos(a + b), float(this->center_.y) + r * sin(a + b));
            this->bl_ = cv::Point2f(float(this->center_.x) - r * cos(a - b), float(this->center_.y) + r * sin(a - b));
            this->tl_ = cv::Point2f(float(this->center_.x) + r * cos(a + b), float(this->center_.y) - r * sin(a + b));
            this->tr_ = cv::Point2f(float(this->center_.x) + r * cos(a - b), float(this->center_.y) - r * sin(a - b));
            return;
        }
        this->tr_ = cv::Point2f(float(this->center_.x) - r * cos(a + b), float(this->center_.y) + r * sin(a + b));
        this->br_ = cv::Point2f(float(this->center_.x) - r * cos(a - b), float(this->center_.y) + r * sin(a - b));
        this->bl_ = cv::Point2f(float(this->center_.x) + r * cos(a + b), float(this->center_.y) - r * sin(a + b));
        this->tl_ = cv::Point2f(float(this->center_.x) + r * cos(a - b), float(this->center_.y) - r * sin(a - b));
    }

    CustomRect CustomRect::minCustomRect(cv::InputArray _points, float angle)
    {
        cv::Mat hull;

        angle = -angle;

        while (angle > 0 || angle <= -90)
        {
            if (angle > 0)
            {
                angle -= 90;
            }
            else
            {
                angle += 90;
            }
        }

        cv::convexHull(_points, hull, true, true);

        if (hull.depth() != CV_32F)
        {
            cv::Mat temp;
            hull.convertTo(temp, CV_32F);
            hull = temp;
        }

        const cv::Point2f *h_points = hull.ptr<cv::Point2f>();

        angle = -angle;

        double maxx = 0;
        double maxy = 0;
        double minx = 0;
        double miny = 0;
        if (hull.size)
        {
            double x = cos(angle * CV_PI / 180.f) * h_points[0].x - sin(angle * CV_PI / 180.f) * h_points[0].y;
            double y = sin(angle * CV_PI / 180.f) * h_points[0].x + cos(angle * CV_PI / 180.f) * h_points[0].y;
            maxx = x;
            maxy = y;
            minx = maxx;
            miny = maxy;
        }

        for (int i = 0; i < hull.rows; i++)
        {
            double x = cos(angle * CV_PI / 180.f) * h_points[i].x - sin(angle * CV_PI / 180.f) * h_points[i].y;
            double y = sin(angle * CV_PI / 180.f) * h_points[i].x + cos(angle * CV_PI / 180.f) * h_points[i].y;
            maxx = maxx < x ? x : maxx;
            maxy = maxy < y ? y : maxy;
            minx = minx > x ? x : minx;
            miny = miny > y ? y : miny;
        }

        auto crect = CustomRect(cv::Point2d(cos(-angle * CV_PI / 180) * (maxx + minx) / 2 - sin(-angle * CV_PI / 180) * (maxy + miny) / 2, sin(-angle * CV_PI / 180) * (maxx + minx) / 2 + cos(-angle * CV_PI / 180) * (maxy + miny) / 2), cv::Size(maxx - minx, maxy - miny), -angle);
        return crect;
    }
    modelL::modelL(const std::vector<cv::Point> &input_contour)
    {
        m_points.clear();
        m_points.insert(m_points.begin(), input_contour.begin(), input_contour.end());
        cv::RotatedRect r_rect = cv::minAreaRect(input_contour);
        this->size_ = r_rect.size;
        this->numb_ = input_contour.size();

        if (input_contour.size() == 4)
        {

            this->p0 = input_contour[0];
            this->p1 = input_contour[1];
            this->p2 = input_contour[2];
            this->p3 = input_contour[3];
            for (int i = 0; i < 4; i++)
            {
                this->maxpoint = std::max(m_points[i].x, int(maxpoint));
                this->minpoint = std::min(m_points[i].x, int(minpoint));
            }
            this->cross_ = maxpoint - minpoint;
            this->center_.x = (p0.x + p1.x + p2.x + p3.x) / 4;
            this->center_.y = (p0.y + p1.y + p2.y + p3.y) / 4;
        }
        else
        {

            this->p0 = input_contour[0];
            this->p1 = input_contour[1];
            this->p2 = input_contour[2];
            this->p3 = input_contour[3];
            this->p4 = input_contour[4];
            this->p5 = input_contour[5];
            for (int i = 0; i < 6; i++)
            {
                this->maxpoint = std::max(m_points[i].x, int(maxpoint));
                this->minpoint = std::min(m_points[i].x, int(minpoint));
            }
            this->cross_ = maxpoint - minpoint;
            this->center_.x = (p0.x + p1.x + p2.x + p3.x + p4.x + p5.x) / 6;
            this->center_.y = (p0.y + p1.y + p2.y + p3.y + p4.y + p5.y) / 6;
        }
    }

    void TransformRecv(const tdtusart::Recv_Struct_t &recv, engineer_tool::ReceiveMessage &receiveMessage)
    {
        receiveMessage.vision = recv.vision;
        // receiveMessage.use_vision = recv.vision;
        // receiveMessage.PositionJudge = recv.positionJudge;
        // receiveMessage.chassis_lock = recv.chassisLock;
    }

    void TransformSend(const engineer_tool::SendMessage &sendMessage, tdtusart::Send_Struct_t &send)
    {
        send.judgeFlag = sendMessage.judgeFlag;
        send.sendAngle1 = sendMessage.sendAngle1;
    }

} // namespace engineer_tool
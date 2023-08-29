//
// Created by Wang on 23-6-14.
//

#include "Base/armor.hpp"


namespace base
{
    LightBlob::LightBlob()
    {
        this->angle = 0;
    }


    LightBlob::LightBlob(cv::RotatedRect box)
    {
        rrect = box;
        if (rrect.size.width > rrect.size.height) {
            std::swap<float>(rrect.size.width, rrect.size.height);
        }
        cv::Point2f p[4];
        box.points(p);
        std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
        up = (p[0] + p[1]) / 2;
        down = (p[2] + p[3]) / 2;
        length = cv::norm(up -  down);
        width = cv::norm(p[0] - p[1]);
        angle = std::atan2(up.x - down.x, up.y - down.y);

        angle = angle / CV_PI * 180;
        if(angle<0)
        {
            angle = -(angle + 180);
        }
        else
        {
            angle = 180-angle;
        }

    }

    Armor::Armor()
    {
        this->num_id = 10;
        this->type = ArmorType::WRONG;
    }

    Armor::Armor(const LightBlob & l1, const LightBlob & l2)
    {
        if (l1.rrect.center.x < l2.rrect.center.x) {
            left = l1, right = l2;
        } else {
            left = l2, right = l1;
        }
        center_point = (left.rrect.center + right.rrect.center) / 2;

        points.push_back(left.down);
        points.push_back(left.up);
        points.push_back(right.up);
        points.push_back(right.down);


        //get rect
        auto width = 15+sqrt(pow(right.rrect.center.x - left.rrect.center.x, 2) + pow(right.rrect.center.y - left.rrect.center.y, 2));
        auto height = 15+MAX(left.rrect.size.height, right.rrect.size.height);
        auto angle = atan2(right.rrect.center.y - left.rrect.center.y, right.rrect.center.x - left.rrect.center.x) * 180 / CV_PI;
        this-> rrect = cv::RotatedRect(center_point, cv::Size(width, height), angle);
        this->rect = cv::Rect2d(center_point - cv::Point2d(width / 2.0, height / 2.0), cv::Size(width, height));
    }

}

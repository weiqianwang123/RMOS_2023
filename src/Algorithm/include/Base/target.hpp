//
// Created by Wang on 23-7-8.
//

#ifndef RMOS_TARGET_HPP
#define RMOS_TARGET_HPP

#include <opencv2/core.hpp>
#include "const.hpp"
#include "armor.hpp"


namespace base
{

    struct RoboInfo
    {
        int enemy_color;
        int bullet_speed;
    };
    struct Command
    {
        int mode;
        int aim_state;
    };
    class Target
    {
    public:
        Target()
        {
            num_id_ = 0;
            gun_pitch_ = 0;
            gun_yaw_ = 0;
            track_state_  = LOST;
            timestamp_recv_ = 0;
            position_ = cv::Point3f (0,0,0);
            velocity_ = cv::Point3f (0,0,0);
            yaw_ = 0;
            radius_1_ = 0;
            radius_2_ = 0;
            dz_ = 0;

            aiming_point_ = cv::Point3f (0,0,0);

        };

        int num_id_;

        float gun_pitch_;
        float gun_yaw_;
        TrackState track_state_;
        uint64 timestamp_recv_;

        cv::Point3f position_;
        cv::Point3f velocity_;
        float yaw_;
        float v_yaw_;
        float radius_1_;
        float radius_2_;
        float dz_;

        cv::Point3f aiming_point_;
    };


}







#endif //RMOS_TARGET_HPP

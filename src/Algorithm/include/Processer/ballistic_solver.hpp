//
// Created by Wang on 23-7-18.
//

#ifndef RMOS_BALLISTIC_SOLVER_HPP
#define RMOS_BALLISTIC_SOLVER_HPP

#include "processer_interfaces/ballistic_solver_interface.hpp"
#include <iostream>

namespace processer
{
    struct BallisticParam
    {

        // 补偿系数
        float level_one_first;
        float level_two_first;
        float level_two_second;
        float level_three_first;
        float level_three_second;
        float level_three_third;


        double k;   // 空气阻力系数/质量
        double g;   // 重力加速度


    };
    class BallisticSolver :  BallisticSolverInterface
    {
    public:
        BallisticSolver();
        ~BallisticSolver();
        cv::Point3f getAngleTime(cv::Point3f position)override;
        bool setBulletSpeed(int bullet_speed)override;

        int bullet_speed_;
    private:
        void setBS_coeff(cv::Point3f position);

        BallisticParam ballistic_param_;
        double bs_coeff;        // 弹速系数



    };
}













#endif //RMOS_BALLISTIC_SOLVER_HPP

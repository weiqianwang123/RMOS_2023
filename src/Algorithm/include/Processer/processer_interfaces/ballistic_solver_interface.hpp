//
// Created by Wang on 23-7-10.
//

#ifndef RMOS_BALLISTIC_SOLVER_INTERFACE_HPP
#define RMOS_BALLISTIC_SOLVER_INTERFACE_HPP

#include "../../Base/armor.hpp"


namespace processer
{
    class BallisticSolverInterface
    {
    public:
        virtual cv::Point3f getAngleTime(cv::Point3f position)=0;
        virtual bool setBulletSpeed(int bullet_speed)=0;
    };

}





#endif //RMOS_BALLISTIC_SOLVER_INTERFACE_HPP

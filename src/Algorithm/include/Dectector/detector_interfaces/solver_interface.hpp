//
// Created by Wang on 23-6-18.
//

#ifndef RMOS_SOLVER_INTERFACE_HPP
#define RMOS_SOLVER_INTERFACE_HPP

#include "../../Base/armor.hpp"

namespace detector
{
    class SolverInterface{

    public:
        virtual bool solveArmorPose(const base::Armor& armor,const cv::Mat& camera_matrix,const cv::Mat& dist_coeffs,cv::Mat &tVec, cv::Mat &rVec) = 0;

    };

} // namespace detector




#endif //RMOS_SOLVER_INTERFACE_HPP

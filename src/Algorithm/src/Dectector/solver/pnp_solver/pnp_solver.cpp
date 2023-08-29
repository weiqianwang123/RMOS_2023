//
// Created by Wang on 23-6-19.
//

#include "Dectector/solver/pnp_solver/pnp_solver.hpp"
namespace detector
{
    bool PnpSolver::solveArmorPose(const base::Armor& armor,const cv::Mat& camera_matrix,const cv::Mat& dist_coeffs,cv::Mat &tVec, cv::Mat &rVec)
    {
        if (camera_matrix.empty())
            return false;

        std::vector<cv::Point2d> point2D;
        point2D = armor.points;
        switch (armor.type)
        {
            case base::ArmorType::SMALL:
                cv::solvePnP(small_armor, point2D, camera_matrix, dist_coeffs, rVec, tVec, false, cv::SOLVEPNP_ITERATIVE);
                break;
            case base::ArmorType::BIG:
                cv::solvePnP(big_armor, point2D, camera_matrix, dist_coeffs, rVec, tVec, false, cv::SOLVEPNP_ITERATIVE);
                break;
            default:
                cv::solvePnP(big_armor, point2D, camera_matrix, dist_coeffs, rVec, tVec, false, cv::SOLVEPNP_ITERATIVE);
                break;
        }

        return true;



    }

}

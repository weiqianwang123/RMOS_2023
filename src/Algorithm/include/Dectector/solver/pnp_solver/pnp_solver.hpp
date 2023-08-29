//
// Created by Wang on 23-6-18.
//

#ifndef RMOS_PNP_SOLVER_HPP
#define RMOS_PNP_SOLVER_HPP

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>
#include <iostream>

#include "../../../Base/armor.hpp"
#include "../../detector_interfaces/solver_interface.hpp"


namespace detector
{
    class PnpSolver : public SolverInterface
    {
    public:
        PnpSolver()
        {
            small_armor.push_back(cv::Point3f(0.0,small_width / 2.0, -small_height / 2.0));
            small_armor.push_back(cv::Point3f(0.0,small_width / 2.0, small_height / 2.0));
            small_armor.push_back(cv::Point3f(0.0,-small_width / 2.0, small_height / 2.0));
            small_armor.push_back(cv::Point3f(0.0,-small_width / 2.0, -small_height / 2.0));

            big_armor.push_back(cv::Point3d(0.0,big_width / 2.0, -big_height / 2.0));
            big_armor.push_back(cv::Point3d(0.0,big_width / 2.0,  big_height / 2.0));
            big_armor.push_back(cv::Point3d(0.0,-big_width / 2.0, big_height / 2.0));
            big_armor.push_back(cv::Point3d(0.0,-big_width / 2.0, -big_height / 2.0));
        };
        ~PnpSolver(){};

        bool solveArmorPose(const base::Armor& armor,const cv::Mat& camera_matrix,const cv::Mat& dist_coeffs,cv::Mat &tVec, cv::Mat &rVec) override;

    private:
        float small_width = 125;
        float small_height = 55;
        float big_width = 225;
        float big_height = 55;
        std::vector<cv::Point3f> small_armor;
        std::vector<cv::Point3f> big_armor;
        std::vector<cv::Point3f> rune_armor; //TO DO

    };


}
#endif //RMOS_PNP_SOLVER_HPP

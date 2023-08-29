//
// Created by nuc12 on 23-7-15.
//

#ifndef RMOS_CONTROLER_HPP
#define RMOS_CONTROLER_HPP

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Eigen
#include <Eigen/Eigen>

#include "Base/armor.hpp"
#include "Tool/tracker/tracker.hpp"
#include "Processer/ballistic_solver.hpp"

namespace processer
{
    class Controler {
    public:
        Controler();
        ~Controler();

        int getAimingPoint(std::vector<base::Armor> armors,cv::Point3f& aimming_point,double timestamp);// 1:move 2:slow_move 3:stop

        bool judgeFire(cv::Point3f aiming_point_camera , double v_yaw, double roll,cv::Point2f& rect_points_out_1,cv::Point2f& rect_points_out_2,cv::Point2f& rect_points_out_3,cv::Point2f& aim_point_2d_out);
        bool getParam(cv::Mat camera_matrix);

        BallisticSolver ballistic_solver_;
        tool::Tracker tracker_;

    private:
        double dt_{0};
        double lost_time_thres_;
        cv::Mat  camera_matrix_;



        double s2qxyz_{0};
        double s2qyaw_{0};
        double s2qr_{0};
        double r_xyz_factor{0};
        double r_yaw{0};

        int true_x_;
        double delay_{0};
        double last_time_ {0};


    };

}




#endif //RMOS_CONTROLER_HPP

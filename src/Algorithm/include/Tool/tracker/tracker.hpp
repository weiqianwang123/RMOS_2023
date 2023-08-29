//
// Created by nuc12 on 23-7-12.
//

#ifndef RMOS_TRACKER_HPP
#define RMOS_TRACKER_HPP

// Eigen
#include <Eigen/Eigen>

//Opencv
#include <opencv2/core.hpp>


// STD
#include <memory>
#include <iostream>


#include "Base/armor.hpp"
#include "Tool/filter/extend_kalman_filter.hpp"


namespace tool
{
    class Tracker
    {
    public:
        Tracker();
        ~Tracker();
        void init(std::vector<base::Armor> armors);
        void update(std::vector<base::Armor> armors);
        void reset();

        base::TrackState tracker_state;
        ExtendedKalmanFilter ekf;
        Eigen::VectorXd measurement;
        Eigen::VectorXd target_state;
        int tracked_id;
        int tracking_thres;
        double lost_thres;

        // To store another pair of armors message
        double dz, another_r;
        double last_yaw_{0};


    private:

        base::Armor tracked_armor;



        double max_match_distance_;
        double max_match_yaw_diff_;
        int detect_count_;
        int lost_count_;
        int outpost_direction_{0};

    private:
        void initEKF(const base::Armor & a);
        void handleArmorJump(const base::Armor & a);
        Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);



    };
}

#endif //RMOS_TRACKER_HPP

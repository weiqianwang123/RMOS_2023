//
// Created by Wang on 23-7-15.
//


#include "Tool/tracker/tracker.hpp"



namespace tool
{
    Tracker::Tracker()
            : tracker_state(base::LOST),
              tracked_id(-1),
              measurement(Eigen::VectorXd::Zero(4)),
              target_state(Eigen::VectorXd::Zero(9))

    {
        cv::FileStorage fs("./src/Algorithm/configure/Tool/tracker/param.xml", cv::FileStorage::READ);

        if(!fs.isOpened())
        {
            std::cout<<"open tool tracker param fail"<<std::endl;
            exit(0);
        }

        fs["max_match_distance"] >> max_match_distance_;
        fs["max_match_yaw_diff"] >> max_match_yaw_diff_;
        fs["tracking_thres"] >> tracking_thres;
        fs["lost_thres"] >> lost_thres;

        fs.release();


    }


    void Tracker::init(std::vector<base::Armor> armors)
    {
        if (armors.empty()) {
            return;
        }

        double min_distance = DBL_MAX;
        tracked_armor = armors[0];
        for (const auto &armor: armors) {
            if (armor.distance_to_image_center < min_distance) {
                min_distance = armor.distance_to_image_center;
                tracked_armor = armor;
            }
        }

        initEKF(tracked_armor);

        tracked_id = tracked_armor.num_id;
        tracker_state = base::DETECTING;
        outpost_direction_ = 0;

    }


    void Tracker::update(std::vector<base::Armor> armors)
    {
        // KF predict
        Eigen::VectorXd ekf_prediction = ekf.predict();

        bool matched = false;
        // Use KF prediction as default target state if no matched armor is found
        target_state = ekf_prediction;
        this->last_yaw_ = target_state(6);                // ??

        if (!armors.empty()) {
            // Find the closest armor with the same id
            base::Armor same_id_armor;
            int same_id_armors_count = 0;
            auto predicted_position = getArmorPositionFromState(ekf_prediction);
            double min_position_diff = DBL_MAX;
            double yaw_diff = DBL_MAX;
            for (const auto & armor : armors) {
                // Only consider armors with the same id
                if (armor.num_id == tracked_id) {
                    same_id_armor = armor;
                    same_id_armors_count++;
                    // Calculate the difference between the predicted position and the current armor position
                    auto p = armor.position;
                    Eigen::Vector3d position_vec(p.x, p.y, p.z);
                    double position_diff = (predicted_position - position_vec).norm();
                    if (position_diff < min_position_diff) {
                        // Find the closest armor
                        min_position_diff = position_diff;
                        yaw_diff = abs(armor.yaw - ekf_prediction(6));
                        tracked_armor = armor;
                    }
                }
            }



            // Check if the distance and yaw difference of closest armor are within the threshold
            if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
                // Matched armor found
                matched = true;
                auto p = tracked_armor.position;
                // Update EKF
                double measured_yaw = tracked_armor.yaw;
                measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
                target_state = ekf.update(measurement);
                this->last_yaw_ =  tracked_armor.yaw;
            } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
                // Matched armor not found, but there is only one armor with the same id
                // and yaw has jumped, take this case as the target is spinning and armor jumped
                matched = true;
                handleArmorJump(same_id_armor);
            } else {
                matched = false;
                // No matched armor found

            }
        }

        // Suppress R from spreading
        if( tracked_id == 1)
        {
            if (target_state(8) < 0.3) {
                target_state(8) = 0.3;
                ekf.setState(target_state);
            }else if(target_state(8) > 0.4){
                target_state(8) = 0.4;
                ekf.setState(target_state);
            }

        }
        else
        {
            if (target_state(8) < 0.2) {
                target_state(8) = 0.2;
                ekf.setState(target_state);
            }else if(target_state(8) > 0.3){
                target_state(8) = 0.3;
                ekf.setState(target_state);
            }
        }

        if(tracked_id == 7 && outpost_direction_ == 0) 
        {
            if(target_state(7)>0.5)
            {
               target_state(7) = 2.512;
               outpost_direction_ = 1;
            }
            else if(target_state(7)<-0.5)
            {
              target_state(7) = -2.512;
              outpost_direction_ = -1;
            }
            target_state(8) = 0.2712;
            target_state(1) = 0;
            target_state(3) = 0;
            target_state(5) = 0;
            ekf.setState(target_state);
            
        }

        if(tracked_id == 7 && outpost_direction_ !=0)
        {
            if(outpost_direction_ == 1)
            {
               target_state(7) = 2.512;
               outpost_direction_ = 1;
            }
            else if(outpost_direction_ == -1)
            {
              target_state(7) = -2.512;
              outpost_direction_ = -1;
            }
            target_state(8) = 0.2712;
            target_state(1) = 0;
            target_state(3) = 0;
            target_state(5) = 0;
            ekf.setState(target_state);
            
        }

        if(abs(dz)>0.055)
        {
            if(dz>0)
            {
                dz = 0.055;
            }
            else
            {
                dz = -0.055;
            }
        }

        if(abs(another_r- target_state(8))>0.15)
        {
            double average_r = (another_r + target_state(8))/2;
            target_state(8) = average_r;
            another_r = average_r;
        }

        if(abs(target_state(5))>0)
        {
            target_state(5) = 0;
        }



        // Tracking state machine
        if (tracker_state == base::DETECTING) {
            if (matched) {
                detect_count_++;
                if (detect_count_ > tracking_thres) {
                    detect_count_ = 0;
                    tracker_state = base::TRACKING;
                }
            } else {
                tracker_state = base::LOST;
            }
        } else if (tracker_state == base::TRACKING) {
            if (!matched) {
                tracker_state = base::TEMP_LOST;
                lost_count_++;
            }
        } else if (tracker_state == base::TEMP_LOST) {
            if (!matched) {
                lost_count_++;
                if (lost_count_ > lost_thres) {
                    lost_count_ = 0;
                    tracker_state = base::LOST;
                }
            } else {
                tracker_state = base::TRACKING;
                lost_count_ = 0;
            }
        }
    }

    void Tracker::reset()
    {
        this->tracker_state = base::LOST;
        this->lost_count_ = 0;
        this->detect_count_ = 0;
        this->tracked_id = -1;
    }












    void Tracker::initEKF(const base::Armor & a)
    {
        double xa = a.position.x;
        double ya = a.position.y;
        double za = a.position.z;
        last_yaw_ = 0;
        double yaw = a.yaw;
        last_yaw_ = a.yaw;

        // Set initial position at 0.26m behind the target
        target_state = Eigen::VectorXd::Zero(9);
        double r = 0.26;
        double xc = xa + r * cos(yaw);
        double yc = ya + r * sin(yaw);
        double zc = za;
        dz = 0, another_r = r;
        target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

        ekf.setState(target_state);
    }

    void Tracker::handleArmorJump(const base::Armor & a)
    {
        double yaw = a.yaw;
        target_state(6) = yaw;

        // Only 4 armors has 2 radius and height
        if (a.num_id != 11 && a.num_id != 12 && a.num_id != 13 && a.num_id != 7) {
            dz = target_state(4) - a.position.z;
            target_state(4) = a.position.z;
            std::swap(target_state(8), another_r);
        }

        // If position difference is larger than max_match_distance_,
        // take this case as the ekf diverged, reset the state
        auto p = a.position;
        Eigen::Vector3d current_p(p.x, p.y, p.z);
        Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);
        if ((current_p - infer_p).norm() > max_match_distance_) {
            double r = target_state(8);
            target_state(0) = p.x + r * cos(yaw);  // xc
            target_state(1) = 0;                   // vxc
            target_state(2) = p.y + r * sin(yaw);  // yc
            target_state(3) = 0;                   // vyc
            target_state(4) = p.z;                 // za
            target_state(5) = 0;                   // vza
            this->last_yaw_ = yaw;                 // ??
        }
        ekf.setState(target_state);
    }

    Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & x)
    {
        // Calculate predicted position of the current armor
        double xc = x(0), yc = x(2), za = x(4);
        double yaw = x(6), r = x(8);
        double xa = xc - r * cos(yaw);
        double ya = yc - r * sin(yaw);
        return Eigen::Vector3d(xa, ya, za);
    }






}
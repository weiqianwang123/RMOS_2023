//
// Created by nuc12 on 23-6-16.
//

#ifndef RMOS_CJ_DETECTOR_HPP
#define RMOS_CJ_DETECTOR_HPP

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
#include "../../detector_interfaces/detector_interface.hpp"


namespace detector
{
    struct ProcessParamsCj
    {
        int binary_thres;
        int enemy_color;
    };

    struct LightParamsCj
    {
        // width / height
        double min_ratio;
        double max_ratio;
        // vertical angle
        double max_angle;
    };

    struct ArmorParamsCj
    {
        double min_light_ratio;
        // light pairs distance
        double min_small_center_distance;
        double max_small_center_distance;
        double min_large_center_distance;
        double max_large_center_distance;
        // horizontal angle
        double armor_max_angle;
    };





    class CjDetector : public DetectorInterface{




    public:

        explicit CjDetector();
        ~CjDetector();
        bool detectArmors(const cv::Mat & image, std::vector<base::Armor>& armors) override;
        bool setEnemyColor(int enemy_color) override;
    private:
        bool findLights(const cv::Mat & image, std::vector<base::LightBlob>& lights) override;
        bool isLight(base::LightBlob light) override;
        bool matchLights(std::vector<base::LightBlob>& lights,std::vector<base::Armor>& armors) override;
        bool isArmor(base::LightBlob light_1,base::LightBlob light_2) override;


        ProcessParamsCj process_params_;
        LightParamsCj light_params_;
        ArmorParamsCj armor_params_;
        base::Color enemy_color_;
    public:
        cv::Mat src_;

    };
}
#endif //RMOS_DETECTOR_HPP

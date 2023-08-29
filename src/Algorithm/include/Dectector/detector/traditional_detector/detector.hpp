//
// Created by Wang on 23-6-15.
//

#ifndef RMOS_DETECTOR_HPP
#define RMOS_DETECTOR_HPP

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

    struct ProcessParams
    {
        double blue_threshold;
        double red_threshold;
        double blue_red_diff;
        double red_blue_diff;
        int enemy_color;

    };

    struct LightParams
    {
        double angle_to_vertigal_max;
        double height_width_min_ratio;
        double size_area_min_ratio;

    };

    struct ArmorParams
    {
        double lights_angle_max_diff;
        double lights_length_max_ratio;
        double lights_Y_max_ratio;
        double width_height_min_ratio;
        double width_height_max_ratio;
        double max_angle;
        double inside_thresh;
    };


    // 输入图像为BGR
    class Detector : public DetectorInterface
    {
    public:

        explicit Detector();
        ~Detector();
        bool detectArmors(const cv::Mat & image, std::vector<base::Armor>& armors) override;
        bool setEnemyColor(int enemy_color) override;
    private:
        bool findLights(const cv::Mat & image, std::vector<base::LightBlob>& lights) override;
        bool isLight(base::LightBlob light) override;
        bool matchLights(std::vector<base::LightBlob>& lights,std::vector<base::Armor>& armors) override;
        bool isArmor(base::LightBlob light_1,base::LightBlob light_2) override;




        ProcessParams process_params_;
        LightParams light_params_;
        ArmorParams armor_params_;
        base::Color enemy_color_;
    public:
        cv::Mat src_;


    };

}

#endif //RMOS_DETECTOR_HPP

//
// Created by Wang on 23-6-16.
//
#include "Dectector/detector/cj_detector/cj_detector.hpp"


namespace detector
{
    CjDetector::CjDetector()
    {
        cv::FileStorage fs("./src/Algorithm/configure/Detector/detector/cj_detector/param.xml", cv::FileStorage::READ);

        if(!fs.isOpened())
        {
            std::cout<<"open cj detect param fail"<<std::endl;
            exit(0);
        }
        fs["binary_thres"] >> process_params_.binary_thres;
        fs["enemy_color"] >> process_params_.enemy_color;
        fs["min_ratio"] >> light_params_.min_ratio;
        fs["max_ratio"] >> light_params_.max_angle;
        fs["max_angle"] >> light_params_.max_angle;
        fs["min_light_ratio"] >> armor_params_.min_light_ratio;
        fs["min_small_center_distance"] >> armor_params_.min_small_center_distance;
        fs["max_small_center_distance"] >> armor_params_.max_small_center_distance;
        fs["min_large_center_distance"] >> armor_params_.min_large_center_distance;
        fs["max_large_center_distance"] >> armor_params_.max_large_center_distance;
        fs["armor_max_angle"] >> armor_params_.armor_max_angle;

        fs.release();
    }


    bool CjDetector::detectArmors(const cv::Mat & image, std::vector<base::Armor>& armors)
    {
        src_ = image;
        std::vector<base::LightBlob> lights;
        if(!findLights(image,lights))
        {
            return false;
        }
        if(!matchLights(lights,armors))
        {
            return false;
        }
//        for(auto &light : lights)
//        {
//            cv::line(src_,light.up,light.down ,cv::Scalar(0, 0, 0),5);
//        }
//        for(auto &armor : armors)
//        {
//            cv::line(src_,armor.left.up,armor.right.down ,cv::Scalar(255, 0, 255),5);
//            cv::line(src_,armor.left.down,armor.right.up ,cv::Scalar(255, 0, 255),5);
//        }
//
//        cv::imshow("HelloWorld",src_);
//        cv::waitKey(1);
        return true;

    }

    bool CjDetector::findLights(const cv::Mat & image, std::vector<base::LightBlob>& lights)
    {

        cv::Mat gray_img;
        cv::cvtColor(image, gray_img, cv::COLOR_RGB2GRAY);

        cv::Mat binary_img;
        cv::threshold(gray_img, binary_img, process_params_.binary_thres, 255, cv::THRESH_BINARY);

        //cv::imshow("2",binary_img);
        using std::vector;
        vector <vector<cv::Point>> contours;
        vector <cv::Vec4i> hierarchy;
        cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &contour: contours)
        {
            if (contour.size() < 5) continue;

            auto r_rect = cv::minAreaRect(contour);
            auto light = base::LightBlob(r_rect);
            if (isLight(light))
            {
                auto rect = r_rect.boundingRect();
                if (  // Avoid assertion failed
                        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= image.cols && 0 <= rect.y &&
                        0 <= rect.height && rect.y + rect.height <= image.rows)
                {
                    int sum_r = 0, sum_b = 0;
                    auto roi = image(rect);
                    // Iterate through the ROI
                    for (int i = 0; i < roi.rows; i++)
                    {
                        for (int j = 0; j < roi.cols; j++)
                        {
                            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0)
                            {
                                // if point is inside contour
                                sum_r += roi.at<cv::Vec3b>(i, j)[2];
                                sum_b += roi.at<cv::Vec3b>(i, j)[0];
                            }
                        }
                    }
                    // Sum of red pixels > sum of blue pixels ?
                    light.color = sum_r > sum_b ? base::RED : base::BLUE;
                    lights.emplace_back(light);
                }

            }


        }
        return true;
    }

    bool CjDetector::isLight(base::LightBlob light)
    {
        float ratio = light.width / light.length;
        bool ratio_ok = light_params_.min_ratio < ratio && ratio < light_params_.max_ratio;
        bool angle_ok = light.angle < light_params_.max_angle;
        bool is_light = ratio_ok && angle_ok;

        return is_light;

    }


    bool CjDetector::matchLights(std::vector<base::LightBlob>& lights,std::vector<base::Armor>& armors) {
        for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
            for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
                if (light_1->color != process_params_.enemy_color || light_2->color != process_params_.enemy_color) continue;

                auto points = std::vector < cv::Point2f > {light_1->up, light_1->down, light_2->up, light_2->down};
                auto bounding_rect = cv::boundingRect(points);
                bool is_contain_light = true;
                for (const auto &test_light: lights) {
                    if (test_light.rrect.center == light_1->rrect.center ||
                        test_light.rrect.center == light_2->rrect.center)
                        continue;

                    if (
                            bounding_rect.contains(test_light.up) || bounding_rect.contains(test_light.down) ||
                            bounding_rect.contains(test_light.rrect.center)) {
                        is_contain_light = true;
                    }
                }

                is_contain_light = false;

                if (is_contain_light)continue;

                if (isArmor(*light_1, *light_2)) {
                    base::Armor armor(*light_1,*light_2);
                    armors.emplace_back(armor);
                    float avg_light_length = ((*light_1).length +(*light_2).length) / 2;
                    float center_distance = cv::norm((*light_1).rrect.center - (*light_2).rrect.center) / avg_light_length;
                    armor.type = center_distance > armor_params_.min_large_center_distance ? base::BIG : base::SMALL;


                }
            }

        }
        return true;
    }

    bool  CjDetector::isArmor(base::LightBlob light_1,base::LightBlob light_2)
    {
        float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                                   : light_2.length / light_1.length;
        bool light_ratio_ok = light_length_ratio > armor_params_.min_light_ratio;

        // Distance between the center of 2 lights (unit : light length)
        float avg_light_length = (light_1.length + light_2.length) / 2;
        float center_distance = cv::norm(light_1.rrect.center - light_2.rrect.center) / avg_light_length;
        bool center_distance_ok = (armor_params_.min_small_center_distance <= center_distance &&
                                   center_distance < armor_params_.max_small_center_distance) ||
                                  (armor_params_.min_large_center_distance <= center_distance &&
                                   center_distance < armor_params_.max_large_center_distance);

        // Angle of light center connection
        cv::Point2f diff = light_1.rrect.center - light_2.rrect.center;
        float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
        bool angle_ok = angle < armor_params_.armor_max_angle;

        bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;



        return is_armor;

    }


    bool CjDetector::setEnemyColor(int enemy_color)
    {
        this->process_params_.enemy_color = enemy_color;
        if (enemy_color == 0)
        {
            this->enemy_color_ = base::Color::RED;
            return true;
        }
        else if (enemy_color == 1)
        {
            this->enemy_color_ = base::Color::BLUE;
            return true;
        }
        else
        {
            return false;
        }
    }



}
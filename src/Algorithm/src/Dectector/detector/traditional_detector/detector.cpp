//
// Created by nuc12 on 23-6-23.
//



#include "Dectector/detector/traditional_detector/detector.hpp"

namespace detector
{

    Detector::Detector()
    {
        cv::FileStorage fs("./src/Algorithm/configure/Detector/detector/traditional_detector/param.xml", cv::FileStorage::READ);

        if(!fs.isOpened())
        {
            std::cout<<"open traditional detect param fail"<<std::endl;
            exit(0);
        }
        fs["blue_threshold"] >> process_params_.blue_threshold;
        fs["red_threshold"] >> process_params_.red_threshold;
        fs["blue_red_diff"] >> process_params_.blue_red_diff;
        fs["red_blue_diff"] >> process_params_.red_blue_diff;
        fs["enemy_color"] >> process_params_.enemy_color;
        fs["angle_to_vertigal_max"] >> light_params_.angle_to_vertigal_max;
        fs["height_width_min_ratio"] >> light_params_.height_width_min_ratio;
        fs["size_area_min_ratio"] >> light_params_.size_area_min_ratio;
        fs["lights_angle_max_diff"] >> armor_params_.lights_angle_max_diff;
        fs["lights_length_max_ratio"] >> armor_params_.lights_length_max_ratio;
        fs["lights_Y_max_ratio"] >> armor_params_.lights_Y_max_ratio;
        fs["width_height_min_ratio"] >> armor_params_.width_height_min_ratio;
        fs["width_height_max_ratio"] >> armor_params_.width_height_max_ratio;
        fs["max_angle"] >> armor_params_.max_angle;
        fs["inside_thresh"] >> armor_params_.inside_thresh;

        if (process_params_.enemy_color == 0)
        {
            this->enemy_color_ = base::Color::RED;
        }
        else if (process_params_.enemy_color== 1)
        {
            this->enemy_color_ = base::Color::BLUE;
        }


        fs.release();
    }


    bool Detector::detectArmors(const cv::Mat & image, std::vector<base::Armor>& armors)
    {

        if(image.empty())return false;


        src_ = image;
        std::vector<base::LightBlob> lights;
        bool is_find_lights = this->findLights(image,lights);
        bool is_find_armors = false;
        if(is_find_lights)
        {
            is_find_armors = this->matchLights(lights,armors);
        }
        else
        {
            is_find_armors = false;
        }

//        if(is_find_lights)
//        {
//            for(auto &light : lights)
//            {
//                cv::line(src_,light.up,light.down ,cv::Scalar(0, 100, 0),5);
//            }
//
//        }
//        if(is_find_armors)
//        {
//            for(auto &armor : armors)
//            {
//                cv::line(src_,armor.left.up,armor.right.down ,cv::Scalar(255, 0, 255),2);
//                cv::line(src_,armor.left.down,armor.right.up ,cv::Scalar(255, 0, 255),2);
//            }
//        }
//
//        cv::imshow("src",src_);
//        cv::waitKey(1);
        return true;


    }

    bool Detector::findLights(const cv::Mat & image, std::vector<base::LightBlob>& lights)
    {
        lights.clear();
        cv::Mat gaussian;
        cv::Mat gray;

        cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Mat> bgr;
        split(image, bgr);

        /*-----------预处理得二值图-----------*/
        cv::Mat gray_binary, color_binary, binary;
        if (enemy_color_ == base::RED)
        {
            threshold(gray, gray_binary, process_params_.red_threshold, 255, cv::THRESH_BINARY);

            subtract(bgr[2], bgr[0], color_binary);
            threshold(color_binary, color_binary, process_params_.red_blue_diff, 255, cv::THRESH_BINARY);

            binary = gray_binary & color_binary;
        }
        else
        {
            threshold(gray, gray_binary, process_params_.blue_threshold, 255, cv::THRESH_BINARY);

            subtract(bgr[0], bgr[2], color_binary);
            threshold(color_binary, color_binary, process_params_.blue_red_diff, 255, cv::THRESH_BINARY);

            binary = gray_binary & color_binary;
        }
        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, element);
        /*-----------寻找并筛选灯条轮廓-----------*/
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        debug_binary_ = binary;
        for (const auto &contour: contours)
        {
            if (cv::contourArea(contour) < 4)continue;

            cv::RotatedRect light_rrect = cv::minAreaRect(contour);
            if (cv::contourArea(contour) / light_rrect.size.area() < light_params_.size_area_min_ratio)
            {
                continue;
            }
            base::LightBlob  light(light_rrect);
            if(isLight(light))
            {
                lights.push_back(light);
            }

        }
        if (lights.size() < 2)
        {
            return false;
        }

        return true;
    }

    bool Detector::isLight(base::LightBlob light)
    {
        cv::RotatedRect light_rrect = light.rrect;
        if (abs(light.angle) > light_params_.angle_to_vertigal_max)
        {
            return false;
        }
        if (light.length / light.width < light_params_.height_width_min_ratio)
        {
            return false;
        }

        return true;

    }

    bool Detector::matchLights(std::vector<base::LightBlob>& lights,std::vector<base::Armor>& armors)
    {


        armors.clear();
        std::vector<base::Armor> temp_armors;
        auto cmp = [](base::LightBlob a, base::LightBlob b) -> bool
        {
            return a.rrect.center.x < b.rrect.center.x;
        };

        sort(lights.begin(), lights.end(), cmp);
        for (int i = 0; i < lights.size() - 1; i++)
        {
            for (int j = i + 1; j < lights.size(); j++)
            {
                int match_flag = 1;
                if (!isArmor(lights[i], lights[j]))
                {
                    continue;
                }
                float x_min = lights[i].rrect.center.x < lights[j].rrect.center.x? lights[i].rrect.center.x :lights[j].rrect.center.x;
                float x_max = lights[i].rrect.center.x > lights[j].rrect.center.x? lights[i].rrect.center.x :lights[j].rrect.center.x;
                float y_min = lights[i].up.y < lights[j].up.y? lights[i].up.y :lights[j].up.y;
                float y_max = lights[i].down.y > lights[j].down.y? lights[i].down.y :lights[j].down.y;
                for(int m=i+1;m<j;m++)
                {
                    float light_center_x = lights[m].rrect.center.x;
                    float light_center_y = lights[m].rrect.center.y;
                    if((light_center_x>x_min&&light_center_x<x_max)&&(light_center_y>y_min&&light_center_y<y_max))
                    {
                        match_flag = 0;
                        break;
                    }
                }
                if(match_flag == 1)
                {
                    lights[i].matched_count ++;
                    lights[j].matched_count ++;
                    temp_armors.push_back(base::Armor(lights[i], lights[j]));
                }
                else
                {
                    continue;
                }
            }

        }

        armors = temp_armors;

//        auto cmp_2 = [](base::Armor a, base::Armor b) -> bool
//        {
//            return a.rrect.center.x < b.rrect.center.x;
//        };
//
//        sort(temp_armors.begin(), temp_armors.end(), cmp_2);
//
//        if(temp_armors.size()>1)
//        {
//            for(int i=0;i<temp_armors.size();i++)
//            {
//                std::cout<<"left"<<temp_armors[i].left.matched_count<<std::endl;
//                std::cout<<"right"<<temp_armors[i].right.matched_count<<std::endl;
//                if(temp_armors[i].left.matched_count==2)
//                {
//
//                    float light_angle_1 = abs(temp_armors[i].left.angle-temp_armors[i].right.angle);
//                    float light_angle_2 = abs(temp_armors[i-1].left.angle-temp_armors[i-1].right.angle);
//                    if(light_angle_1<light_angle_2)
//                    {
//                        std::cout<<"777"<<std::endl;
//                        armors.pop_back();
//                        armors.push_back(temp_armors[i]);
//                    }
//                }
//                else
//                {
//                    armors.push_back(temp_armors[i]);
//                }
//
//            }
//
//        }
//        else
//        {
//            armors = temp_armors;
//        }


        return true;

    }

    bool  Detector::isArmor(base::LightBlob light_1,base::LightBlob light_2)
    {

        float width = light_2.rrect.center.x - light_1.rrect.center.x;
        float height = (light_2.length + light_1.length) / 2.0;
        float angle_i_org = light_1.angle;
        float angle_j_org = light_2.angle;

        // 匹配灯条：角度差、长度比、高度差、组成的宽高比，滤除/\ \/
        if (abs(angle_i_org - angle_j_org) > armor_params_.lights_angle_max_diff)
        {
            return false;
        }

        if(std::max(light_1.length, light_2.length) / std::min(light_1.length, light_2.length) > armor_params_.lights_length_max_ratio)
        {
            return false;
        }


        if(width / height > armor_params_.width_height_max_ratio ||
           width / height < armor_params_.width_height_min_ratio)
        {
            return false;
        }

        base::Armor temp(light_1, light_2);
        if (fabs(temp.rrect.angle) > armor_params_.max_angle)
        {
            return false;
        }




        //  滤除类如窗户形成的伪装甲板，即装甲板区域过亮
        cv::Mat temp_inarmor = src_(temp.rect & cv::Rect2d(cv::Point(0, 0), src_.size()));
        cv::cvtColor(temp_inarmor, temp_inarmor, cv::COLOR_BGR2GRAY);
        if (cv::mean(temp_inarmor)[0] > armor_params_.inside_thresh)
        {
            return false;
        }

        return true;

    }

    bool Detector::setEnemyColor(int enemy_color)
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
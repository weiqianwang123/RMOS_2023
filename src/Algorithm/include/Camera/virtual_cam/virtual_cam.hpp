//
// Created by nuc12 on 23-6-26.
//

#ifndef RMOS_VIRTUAL_CAM_HPP
#define RMOS_VIRTUAL_CAM_HPP

#endif //RMOS_VIRTUAL_CAM_HPP

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


#include "../camera_interfaces/camera_interface.hpp"


namespace camera
{
    class VirtualCam : public CamInterface
    {
    public:

        // 打开设备
        bool open() override;

        // 关闭设备
        bool close() override{};


        // 设置参数
        bool set_parameter(CamParamType type, int value) override{};

        // 得到参数
        bool get_parameter(CamParamType type, int &value) override{};

        // 返回错误信息
        std::string error_message() override
        {
            return ("Error: " + error_message_);
        }



        // 返回是否打开
        bool is_open() override;

        // 获取Mat图像
        bool grab_image(cv::Mat &image) override;

        bool set_path(std::string path)
        {
            this->video_path = path;
            return true;
        }

        VirtualCam(){};
        ~VirtualCam(){};

    private:
        std::string video_path;
        cv::VideoCapture capture;
        bool is_open_;
        std::string error_message_; // 错误消息，对外传输


    };

    }




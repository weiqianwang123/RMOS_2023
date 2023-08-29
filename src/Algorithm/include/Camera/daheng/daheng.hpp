//
// Created by nuc12 on 23-6-14.
//

#ifndef RMOS_DAHENG_HPP
#define RMOS_DAHENG_HPP

#include <string>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "driver/DxImageProc.h"
#include "driver/GxIAPI.h"

#include "../camera_interfaces/camera_interface.hpp"


#define ACQ_TRANSFER_SIZE (64 * 1024)
#define ACQ_TRANSFER_NUMBER_URB 64
#define ACQ_BUFFER_NUM 3

namespace camera
{
    class DahengCam : public CamInterface
    {

    public:
        // 打开设备
        bool open() override;

        // 关闭设备
        bool close() override;

        // 返回是否打开
        bool is_open() override;

        // 获取Mat图像
        bool grab_image(cv::Mat &image) override;

        // 设置参数
        bool set_parameter(CamParamType type, int value) override;

        // 得到参数
        bool get_parameter(CamParamType type, int &value) override;

        // 返回错误信息
        std::string error_message() override
        {
            return ("Error: " + error_message_);
        }

        // 重设参数
        bool reset_parameters(const std::map<CamParamType, int> & param_map);

        // 构造
        explicit DahengCam(const std::string camera_sn = "");

        ~DahengCam();

    private:
        bool is_open_;
        GX_DEV_HANDLE device_; // 设备权柄
        PGX_FRAME_BUFFER pFrameBuffer_; // raw 图像的buffer
        uint8_t *rgbImagebuf_; // rgb 图像的buffer
        std::string error_message_; // 错误消息，对外传输
        std::string camera_sn_; // TODO ： 相机sn号

    public:
        std::unordered_map<CamParamType, int> params_;

    private:
        // 设置相机初始化参数
        bool setInit();

        //  初始化相机sdk
        bool init_sdk();
    };

}


#endif //RMOS_DAHENG_HPP

//
// Created by Wang on 23-6-14.
//

#ifndef RMOS_CAM_NODE_HPP
#define RMOS_CAM_NODE_HPP


//std
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>

//other
#include <opencv2/core.hpp>

//ros
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>

#include "../../Algorithm/include/Camera/camera_interfaces/camera_interface.hpp"
#include "../../Algorithm/include/Camera/daheng/daheng.hpp"
#include "../../Algorithm/include/Camera/virtual_cam/virtual_cam.hpp"


namespace rmos_cam
{
    class CamNode : public rclcpp::Node
    {
        public:
            CamNode(const std::string & node_name, const rclcpp::NodeOptions & options) : Node(node_name, options)
            {
                RCLCPP_INFO(this->get_logger(), "Starting node [%s]", node_name.c_str());
            };

        protected:
            image_transport::CameraPublisher img_pub_;                // 信息发布
            sensor_msgs::msg::CameraInfo camera_info_msg_;            // 相机消息
            sensor_msgs::msg::Image::SharedPtr image_msg_;
            cv::Mat image_;
            std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;

            uint32_t frame_id_ = 0;                                   // 帧计数器
    };

    class DahengCamNode : public virtual CamNode
    {
    public:
        DahengCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        ~DahengCamNode();
    protected:
        std::shared_ptr<camera::DahengCam> cam_dev_;
        std::thread capture_thread_;                    // 采图线程

    };
    class VirtualCamNode : public virtual CamNode
    {
    public:
        VirtualCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        ~VirtualCamNode();
    protected:
        std::shared_ptr<camera::VirtualCam> virtual_dev_;
        std::thread capture_thread_;                    // 采图线程

    };



} // namespace rmos_cam



#endif //RMOS_CAM_NODE_HPP

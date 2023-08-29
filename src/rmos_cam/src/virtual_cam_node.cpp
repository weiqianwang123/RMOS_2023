//
// Created by Wang on 23-6-26.
//


//std
#include <chrono>
#include <sstream>

//ros
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>

#include "../include/cam_node.hpp"


namespace rmos_cam {
    VirtualCamNode::VirtualCamNode(const rclcpp::NodeOptions &options) : CamNode("virtual_camera", options) {
        //video path
        std::string path = "/home/nuc12/Desktop/Vision_code/vision_test_data/video/red/1.avi";
        // cam dev
        virtual_dev_ = std::make_shared<camera::VirtualCam>();
        virtual_dev_->set_path(path);

        virtual_dev_->open();

        img_pub_ = image_transport::create_camera_publisher(this, "/image_raw", rmw_qos_profile_sensor_data);

        // load camera_info
        cam_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "VirtualCam");


        auto pkg_path = ament_index_cpp::get_package_share_directory("rmos_bringup");
        auto yaml_path = "file://" + pkg_path + "/configure/virtual_cam_info.yaml";
        if (!cam_info_manager_->loadCameraInfo(yaml_path)) {
            RCLCPP_WARN(this->get_logger(), "Load Camera Info Fail!");
        } else {
            camera_info_msg_ = cam_info_manager_->getCameraInfo();
        }

        capture_thread_ = std::thread{[this]() -> void {
            int count_ = 120;
            int frame_count_ = 0;
            while (rclcpp::ok()) {
//                std::string pic_path ="/home/nuc12/Desktop/Vision_code/vision_test_data/picture/blue/" + std::to_string(count_)+".png";
//                    image_ = cv::imread(pic_path);
//                    image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
//                    (*image_msg_).header.stamp = camera_info_msg_.header.stamp = this->now();
//                    (*image_msg_).header.frame_id = "camera";
//                    camera_info_msg_.header.frame_id = (*image_msg_).header.frame_id;
//                    RCLCPP_WARN(this->get_logger(), std::to_string(count_));
//                    img_pub_.publish(*image_msg_, camera_info_msg_);
//                    frame_count_++;
//                    if(frame_count_%200==0)
//                    {
//                        count_++;
//                    }
//                    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//
//                if (!virtual_dev_->is_open()) {
//                    exit(0);
//                }
                    if (virtual_dev_->grab_image(image_)) {
                    image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
                    (*image_msg_).header.stamp = camera_info_msg_.header.stamp = this->now();
                    (*image_msg_).header.frame_id = "camera";
                    camera_info_msg_.header.frame_id = (*image_msg_).header.frame_id;
                    RCLCPP_WARN(this->get_logger(), " Image from video");
                    img_pub_.publish(*image_msg_, camera_info_msg_);
                    //std::this_thread::sleep_for(std::chrono::milliseconds(10));
                } else {
                    std::cout << virtual_dev_->error_message() << std::endl;
                    exit(0);
                }
            }
        }};
    }

    VirtualCamNode::~VirtualCamNode() {
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
    }

}






#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_cam::VirtualCamNode)


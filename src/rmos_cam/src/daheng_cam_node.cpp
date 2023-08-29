//
// Created by Wang on 23-6-14.
//


//std
#include <chrono>
#include <sstream>

//ros
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>

#include "../include/cam_node.hpp"

namespace rmos_cam
{
    DahengCamNode::DahengCamNode(const rclcpp::NodeOptions &options) : CamNode("daheng_camera", options)
    {
        // cam dev
        cam_dev_ = std::make_shared<camera::DahengCam>();

        // parameter
        int Width, Height, Exposure, RGain, GGain, BGain, Gamma, Fps;
        int AutoExposure, AutoWhiteBalance;

        cv::FileStorage fs("./rmos_bringup/configure/daheng_camera.xml", cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Open daheng_camera.xml fail!");
            exit(0);
        }
        fs["width"] >> Width;
        fs["height"] >> Height;
        fs["exposure"] >> Exposure;
        fs["rgain"] >> RGain;
        fs["ggain"] >> GGain;
        fs["bgain"] >> BGain;
        fs["fps"] >> Fps;
        fs["gain"] >> Gamma;


        // set paramter
        cam_dev_->set_parameter(camera::CamParamType::Height, Height);
        cam_dev_->set_parameter(camera::CamParamType::Width, Width);
        cam_dev_->set_parameter(camera::CamParamType::AutoExposure, AutoExposure);
        cam_dev_->set_parameter(camera::CamParamType::Exposure, Exposure);
        cam_dev_->set_parameter(camera::CamParamType::AutoWhiteBalance, AutoWhiteBalance);
        cam_dev_->set_parameter(camera::CamParamType::RGain, RGain);
        cam_dev_->set_parameter(camera::CamParamType::GGain, GGain);
        cam_dev_->set_parameter(camera::CamParamType::BGain, BGain);
        cam_dev_->set_parameter(camera::CamParamType::Gamma, Gamma);
        cam_dev_->set_parameter(camera::CamParamType::Fps, Fps);

        cam_dev_->open();

        img_pub_ = image_transport::create_camera_publisher(this, "/image_raw", rmw_qos_profile_default);


        // load camera_info
        cam_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "DahengCam");
        auto pkg_path = ament_index_cpp::get_package_share_directory("rmos_bringup");
        auto yaml_path = "file://" + pkg_path + "/configure/daheng_cam_info.yaml";
        if (!cam_info_manager_->loadCameraInfo(yaml_path))
        {
            RCLCPP_WARN(this->get_logger(), "Load Camera Info Fail!");
        }
        else
        {
            camera_info_msg_ = cam_info_manager_->getCameraInfo();
        }

        capture_thread_ = std::thread{[this]() -> void
                                      {
                                          while (rclcpp::ok())
                                          {
                                              if (!cam_dev_->is_open())
                                              {
                                                  exit(0);
                                              }
                                              //sensor_msgs::msg::Image image_msg_;

                                              if (cam_dev_->grab_image(image_))
                                              {
                                                  image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",image_).toImageMsg();
                                                  (*image_msg_).header.stamp = camera_info_msg_.header.stamp = this->now();
                                                  (*image_msg_).header.frame_id = "camera";
                                                  camera_info_msg_.header.frame_id = (*image_msg_).header.frame_id;

                                                  img_pub_.publish(*image_msg_, camera_info_msg_);

                                              }
                                              else
                                              {
                                                  std::cout << cam_dev_->error_message() << std::endl;
                                                  exit(0);
                                              }
                                          }
                                      }};
    }

    DahengCamNode::~DahengCamNode()
    {
        if (capture_thread_.joinable())
        {
            capture_thread_.join();
        }
        cam_dev_->close();
        RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
    }
} // namespace rmos_cam


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_cam::DahengCamNode)







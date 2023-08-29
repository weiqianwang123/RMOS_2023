//
// Created by Wang on 23-6-16.
//

#ifndef RMOS_DETECTOR_NODE_HPP
#define RMOS_DETECTOR_NODE_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "std_msgs/msg/bool.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Eigen>
#include <tf2_ros/static_transform_broadcaster.h>

// STD
#include <memory>
#include <string>
#include <vector>

//interfaces
#include "rmos_interfaces/msg/armors.hpp"
#include "rmos_interfaces/msg/armor.hpp"
#include "rmos_interfaces/msg/color.hpp"
#include "rmos_interfaces/msg/aimpoint.hpp"

#include "../../Algorithm/include/Dectector/detector_interfaces/detector_interface.hpp"
#include "../../Algorithm/include/Dectector/detector/cj_detector/cj_detector.hpp"
#include "../../Algorithm/include/Dectector/detector/traditional_detector/detector.hpp"
#include "../../Algorithm/include/Dectector/classifier/cj_classifier/cj_classifier.hpp"
#include "../../Algorithm/include/Dectector/classifier/onnx_classifier/onnx_classifier.hpp"
#include "../../Algorithm/include/Dectector/solver/pnp_solver/pnp_solver.hpp"
#include "../../Algorithm/include/Debug/debug.hpp"

namespace rmos_detector
{
    class BaseDetectorNode : public rclcpp::Node
    {
    public:
        BaseDetectorNode(const std::string &node_name,
                         const rclcpp::NodeOptions &options) : Node(node_name, options)
        {
            RCLCPP_INFO(this->get_logger(), "Starting node [%s]", node_name.c_str());
        }

        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;

        std::shared_ptr<image_transport::Subscriber> image_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Subscription<rmos_interfaces::msg::Color>::SharedPtr color_sub_;
        rclcpp::Subscription<rmos_interfaces::msg::Aimpoint>::SharedPtr aim_sub_;






    };

    class BasicDetectorNode : public BaseDetectorNode
    {
    public:
        BasicDetectorNode(const rclcpp::NodeOptions &options) : BaseDetectorNode("basic_detector", options)
        {
            RCLCPP_INFO(this->get_logger(), "Starthing basic_detector node");

            //subscriber
            this->image_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
                    this, "/image_raw", std::bind(&BasicDetectorNode::imageCallBack, this, std::placeholders::_1),
                    "raw",
                    rmw_qos_profile_default));
            this->camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", rclcpp::SensorDataQoS(),
                                                                                             [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg)
                                                                                             {
                                                                                                 RCLCPP_INFO(this->get_logger(), "Receive camera infomation");

                                                                                                 this->camera_info_msg_ = *camera_info_msg;

                                                                                                 this->camera_matrix_.create(3, 3, CV_64FC1);
                                                                                                 this->dist_coeffs_.create(1, 5, CV_64FC1);

                                                                                                 for (int i = 0; i < 9; i++)
                                                                                                 {
                                                                                                     this->camera_matrix_.at<double>(i / 3, i % 3) = camera_info_msg->k[i];
                                                                                                 }
                                                                                                 for (int i = 0; i < camera_info_msg->d.size(); i++)
                                                                                                 {
                                                                                                     this->dist_coeffs_.at<double>(0, i) = camera_info_msg->d[i];
                                                                                                 }

                                                                                                 this->camera_info_sub_.reset();

                                                                                             });

            this->color_sub_ = this->create_subscription<rmos_interfaces::msg::Color>
                    ("/color_info", rclcpp::SensorDataQoS(), [this](rmos_interfaces::msg::Color::ConstSharedPtr color_msg)
                    {
                        int enemy_color = (*color_msg).color;
                        this->detector_->setEnemyColor(enemy_color);
                    });
            this->aim_sub_ = this->create_subscription<rmos_interfaces::msg::Aimpoint>
                    ("/aim", rclcpp::SensorDataQoS(), [this](rmos_interfaces::msg::Aimpoint::ConstSharedPtr aim_msg)
                    {
                       this->aim_point_.x = (*aim_msg).aim_point.x;
                       this->aim_point_.y = (*aim_msg).aim_point.y;

                       cv::RotatedRect rect(cv::Point2f((*aim_msg).point_1.x, (*aim_msg).point_1.y),
                                            cv::Point2f((*aim_msg).point_2.x, (*aim_msg).point_2.y),
                                            cv::Point2f((*aim_msg).point_3.x, (*aim_msg).point_3.y));
                       this->debug_fire_rect_ = rect;
                    });
            // publisher
            this->armors_pub_ = this->create_publisher<rmos_interfaces::msg::Armors>("/rmos_detector/armors", rclcpp::SensorDataQoS());

            //debug info publisher
            debug_img_pub_ = image_transport::create_camera_publisher(this, "/debug_image", rmw_qos_profile_default);
            debug_bin_img_pub_ = image_transport::create_camera_publisher(this, "/debug_bin_image", rmw_qos_profile_default);


            //cj_detector_ = std::make_shared<detector::CjDetector>();
            detector_ = std::make_shared<detector::Detector>();
            //cj_classifier_ = std::make_shared<detector::CjClassifier>();
            pnp_solver_ = std::make_shared<detector::PnpSolver>();
            onnx_classifier_ =  std::make_shared<detector::OnnxClassifier>();


            /*publish static TF*/
            this->tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);



        }

    protected:
        void imageCallBack(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg);

        //std::shared_ptr<detector::CjDetector> cj_detector_;
        std::shared_ptr<detector::Detector> detector_;

       // std::shared_ptr<detector::CjClassifier> cj_classifier_;
        std::shared_ptr<detector::OnnxClassifier> onnx_classifier_;

        std::shared_ptr<detector::PnpSolver> pnp_solver_;

        /* Publisher */
        rclcpp::Publisher<rmos_interfaces::msg::Armors>::SharedPtr armors_pub_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;

        /*debug*/
        image_transport::CameraPublisher debug_img_pub_;
        image_transport::CameraPublisher debug_bin_img_pub_;
        sensor_msgs::msg::Image::SharedPtr debug_image_msg_;
        sensor_msgs::msg::Image::SharedPtr debug_bin_image_msg_;

        cv::RotatedRect debug_fire_rect_{ cv::RotatedRect(cv::Point2f(0, 0), cv::Point2f(0, 0), cv::Point2f(0, 0))};
        cv::Point2f aim_point_{cv::Point2f(0, 0)};      
          

        //camera param
        sensor_msgs::msg::CameraInfo camera_info_msg_;


    };


}


#endif //RMOS_DETECTOR_NODE_HPP

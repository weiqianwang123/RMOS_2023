//
// Created by Wang on 23-7-10.
//

#ifndef RMOS_PROCESSER_NODE_HPP
#define RMOS_PROCESSER_NODE_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "message_filters/subscriber.h"

//interfaces
#include "rmos_interfaces/msg/armors.hpp"
#include "rmos_interfaces/msg/armor.hpp"
#include "rmos_interfaces/msg/aimpoint.hpp"
#include "rmos_interfaces/msg/target.hpp"
#include "rmos_interfaces/msg/quaternion_time.hpp"
#include "rmos_interfaces/msg/bullet_speed.hpp"
#include "rmos_interfaces/msg/autoaim_state.hpp"


#include "../../Algorithm/include/Processer/controler.hpp"
#include "../../Algorithm/include/Debug/debug.hpp"


namespace rmos_processer
{
    using tf2_filter = tf2_ros::MessageFilter<rmos_interfaces::msg::Armors>;
    using namespace message_filters;
    class ProcesserNode : public rclcpp::Node {
    public:
        ProcesserNode(const rclcpp::NodeOptions &options);

        ~ProcesserNode() = default;

    protected:

        /**
         *  @brief  amrors_sub_的回调函数
         */
        void armorsCallBack(const rmos_interfaces::msg::Armors::SharedPtr armors_msg);
        /**
         *  @brief  quaternion_sub_的回调函数
         */
        void quaternionCallBack(const rmos_interfaces::msg::QuaternionTime::SharedPtr quaternion_msg);

        /**
          *  @brief  bs_sub_的回调函数
          */
        void bsCallBack(const rmos_interfaces::msg::BulletSpeed::SharedPtr bs_msg);

        /**
          *  @brief  autoaim_state_sub_的回调函数
          */
        void autoaimStateCallBack(const rmos_interfaces::msg::AutoaimState::SharedPtr autoaim_state_msg);

        /**
         * @brief  RVIZ可视化
         */
        void publishMarkers(const rmos_interfaces::msg::Target &target_msg);


        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

        message_filters::Subscriber<rmos_interfaces::msg::Armors> armors_sub_;
        rclcpp::CallbackGroup::SharedPtr armors_sub_callback_group_;
        rclcpp::Subscription<rmos_interfaces::msg::QuaternionTime>::SharedPtr quaternion_sub_;
        rclcpp::CallbackGroup::SharedPtr quaternion_sub_callback_group_;
        rclcpp::Subscription<rmos_interfaces::msg::BulletSpeed>::SharedPtr bs_sub_;
        rclcpp::CallbackGroup::SharedPtr bs_sub_callback_group_;
        rclcpp::Subscription<rmos_interfaces::msg::AutoaimState>::SharedPtr autoaim_state_sub_;
        rclcpp::CallbackGroup::SharedPtr aimstate_sub_callback_group_;

        /*Subscriber with tf2 message_filter*/
        std::string target_frame_;
        std::shared_ptr <tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr <tf2_ros::TransformListener> tf2_listener_;
        std::shared_ptr <tf2_filter> tf2_filter_;

        /* Publisher */
        rclcpp::Publisher<rmos_interfaces::msg::Target>::SharedPtr target_pub_;
        rclcpp::Publisher<rmos_interfaces::msg::Aimpoint>::SharedPtr aim_pub_;

        /* Buffer */
        std::queue<rmos_interfaces::msg::QuaternionTime> quaternion_buf_;
        std::queue<rmos_interfaces::msg::AutoaimState> autoaim_state_buf_;


        /* Visualization marker publisher  */
        visualization_msgs::msg::MarkerArray detect_marker_array_;
        visualization_msgs::msg::Marker pose_marker_;
        visualization_msgs::msg::Marker text_marker_;
        visualization_msgs::msg::Marker armor_marker_;


        visualization_msgs::msg::MarkerArray process_marker_array_;
        visualization_msgs::msg::Marker position_marker_;
        visualization_msgs::msg::Marker linear_v_marker_;
        visualization_msgs::msg::Marker angular_v_marker_;
        visualization_msgs::msg::Marker armors_marker_;
        visualization_msgs::msg::Marker aiming_marker_;


        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detect_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr process_marker_pub_;

        /*Controler*/
        std::shared_ptr<processer::Controler> controler_;

        /*params*/
        int imu_data_count_{0};
        //camera param
        sensor_msgs::msg::CameraInfo camera_info_msg_;
        cv::Mat camera_matrix_;
    };




}





#endif //RMOS_PROCESSER_NODE_HPP

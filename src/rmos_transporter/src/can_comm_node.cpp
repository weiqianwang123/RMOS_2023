//
// Created by Wang on 23-7-9.
//

#include <vector>
#include <Eigen/Dense>
#include "../include/comm_node.hpp"

using namespace std::chrono;

namespace rmos_transporter
{
    CanCommNode::CanCommNode(const rclcpp::NodeOptions &options) : CommNode("can_comm", options)
    {
        memset(data_buf_, 0, sizeof(data_buf_));
        memset(state_buf_, 0, sizeof(state_buf_));

        // create callback group
        this->send_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        this->target_sub_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // create publisher and subscriber
        this->tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        auto target_sub_options = rclcpp::SubscriptionOptions();
        target_sub_options.callback_group = this->target_sub_callback_group_;
        this->target_sub_ = this->create_subscription<rmos_interfaces::msg::Target>
                ("/target", rclcpp::SensorDataQoS(),
                 std::bind(&CanCommNode::targetCallBack, this, std::placeholders::_1),
                 target_sub_options);


        // create send timer
        this->send_data_timer_ = this->create_wall_timer
                (2ms, std::bind(&CanCommNode::sendDataCallBack, this), send_callback_group_);
        this->send_state_timer_ = this->create_wall_timer
                (5ms, std::bind(&CanCommNode::sendStateCallBack, this), send_callback_group_);

        this->recevie_thread_ = std::thread(&CanCommNode::recevieCallBack, this);
    }

    CanCommNode::~CanCommNode()
    {
        if (recevie_thread_.joinable())
            recevie_thread_.join();
    }

    void CanCommNode::targetCallBack(const rmos_interfaces::msg::Target::SharedPtr target)
    {
        if(debug::get_debug_option(base::SHOW_TOTAL_COST))
        {
            if(target->gun_pitch!=0)
            {
                rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
                auto time = steady_clock_.now();
                RCLCPP_INFO(this->get_logger(), "Cost %.4f ms",(time.seconds()-last_time_) * 1000);
                last_time_ = time.seconds();
            }
        }
        this->target2data(target, data_buf_);
        this->target2state(target, state_buf_);
    }

    void CanCommNode::target2data(const rmos_interfaces::msg::Target::SharedPtr target, u_char *buf)
    {
        memset(buf, 0, 8);
        int16_t pitch_yaw[2] = {0};
        pitch_yaw[0] = (int16_t)(target->gun_pitch * 32768.0 / 180.0);
        pitch_yaw[1] = (int16_t)(target->gun_yaw * 32768.0 / 180.0);
        memcpy(buf, pitch_yaw, 4);
        memcpy(&buf[4], &target->timestamp_recv, 4);
    }

    void CanCommNode::target2state(const rmos_interfaces::msg::Target::SharedPtr target, u_char *buf)
    {
        memset(buf, 0, 3);
        switch ((base::TrackState)target->track_state)
        {
            case base::TrackState::TRACKING:
            case base::TrackState::TEMP_LOST:
                buf[0] |= 0x01;
                break;
            default:
                break;
        }
        switch(target->outpost_direction)
        {
            case 1:
                buf[0] |= 0x40;
            case -1:
                buf[0] |= 0;

        }
        if (target->suggest_fire)
            buf[0] |= 0x02;
        switch (target->id)
        {
            case 0:
                buf[1] |= 0x80;
                break;
            case 1:
                buf[1] |= 0x01;
                break;
            case 2:
                buf[1] |= 0x02;
                break;
            case 3:
            case 11:
                buf[1] |= 0x04;
                break;
            case 4:
            case 12:
                buf[1] |= 0x08;
                break;
            case 5:
                buf[1] |= 0x10;
                break;
            case 6:
                buf[1] |= 0x40;
                break;
            case 7:
                buf[1] |= 0x20;
                break;
            default:
                break;
        }

        double distance = sqrt(target->position.x *  target->position.x +
                               target->position.y *  target->position.y +
                               target->position.z *  target->position.z);



        if(distance<3){

            buf[2] |= 0x01;

        }
        else if(3< distance&&distance<5){

            buf[2] |= 0x02;

        }
        else{

            buf[2] |= 0x03;

        }
    }

    void CanCommNode::sendDataCallBack()
    {
        this->can_.send(DATA_SEND_ID, data_buf_, 8);
    }

    void CanCommNode::sendStateCallBack()
    {
        this->can_.send(STATE_SEND_ID, state_buf_, 3);
    }

    void CanCommNode::recevieCallBack()
    {
        rmos_interfaces::msg::QuaternionTime quaternion_time_msg;
        quaternion_time_msg.quaternion_stamped.header.frame_id = std::string("Imu");

        rmos_interfaces::msg::Color color_msg;
        rmos_interfaces::msg::Mode mode_msg;
        rmos_interfaces::msg::BulletSpeed bs_msg;
        rmos_interfaces::msg::AutoaimState autoaim_state_msg;

        while (rclcpp::ok())
        {


            uint id = 0;
            u_char buf[8] = {0};
            u_char dlc = 0;

            double quaternion[4] = {0};
            Eigen::Quaterniond q_temp;

            this->can_.receive(id, buf, dlc);

            switch (id)
            {

                case IMU_RECEIVE_ID:
                {
                    int time_offset = 0;
                    quaternion_time_msg.quaternion_stamped.header.stamp = this->now() + rclcpp::Duration(0,time_offset);
                    this->transfer2Quaternion(buf, quaternion);
                    q_temp = Eigen::Quaterniond(quaternion);
                    q_temp.normalize();
                    quaternion_time_msg.quaternion_stamped.quaternion.w = q_temp.x();
                    quaternion_time_msg.quaternion_stamped.quaternion.x = q_temp.y();
                    quaternion_time_msg.quaternion_stamped.quaternion.y = q_temp.z();
                    quaternion_time_msg.quaternion_stamped.quaternion.z = q_temp.w();

                    geometry_msgs::msg::TransformStamped t;
                    t.header.stamp =  quaternion_time_msg.quaternion_stamped.header.stamp;
                    t.header.frame_id = "world";
                    t.child_frame_id = "IMU";
                    t.transform.rotation.x = quaternion_time_msg.quaternion_stamped.quaternion.x;
                    t.transform.rotation.y = quaternion_time_msg.quaternion_stamped.quaternion.y;
                    t.transform.rotation.z = quaternion_time_msg.quaternion_stamped.quaternion.z;
                    t.transform.rotation.w = quaternion_time_msg.quaternion_stamped.quaternion.w;
                    tf_publisher_->sendTransform(t);
                    break;
                }
                case IMU_TIME_RECEIVE_ID:
                {
                    if (dlc == 4)
                    {
                        memcpy(&quaternion_time_msg.timestamp_recv, buf, 4);
                        this->quaternion_pub_->publish(quaternion_time_msg);

                    }
                    break;
                }
                case MODE_RECEIVE_ID:
                {
                    if ((buf[0] & 0x10) == 0x10)
                    {
                        mode_msg.mode = (int) base::Mode::NORMAL_RUNE;
                    }
                    else if ((buf[0] & 0x20) == 0x20)
                    {
                        mode_msg.mode = (int) base::Mode::RUNE;
                    }
                    else
                    {
                        mode_msg.mode = (int)base::Mode::NORMAL;
                    }

                    this->mode_pub_->publish(mode_msg);

                    if((buf[6] & 0x02) == 0x02)
                    {
                        autoaim_state_msg.autoaim_state = 1;
                    }
                    else
                    {
                        autoaim_state_msg.autoaim_state = 0;
                    }
                    this->autoaim_state_pub_->publish(autoaim_state_msg);

                    break;
                }

                case  HERO_BS_RECEIVE_ID:
                {
                    auto bullet_speed = (uint16_t) buf[4] | ((uint16_t) buf[5] << 8);
                    bs_msg.speed = (int) bullet_speed;
                    this->bs_pub_->publish(bs_msg);
                    break;
                }

                case ROBOT_RECEIVE_ID:
                {

                    if (dlc == 6) {
                        if (buf[0] < 10) {
                            color_msg.color = (int) (base::Color::BLUE);
                        } else if (buf[0] > 100) {
                            color_msg.color = (int) base::Color::RED;
                        } else {
                            color_msg.color = (int) (base::Color::BLUE);
                        }
                        this->color_pub_->publish(color_msg);
                    }
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    void CanCommNode::transfer2Quaternion(u_char *buf, double *quaternion)
    {
        int16_t q[4] = {0};
        memcpy(q, buf, 8);

        for (int i = 0; i < 4; i++)
        {
            quaternion[i] = (double)q[i] / 32768.0;
        }
        // double *q_;
        // q_ = quaternion;
        // std::cout<<"yaw"<<atan2f(2.0f*(q_[0]*q_[3]+q_[1]*q_[2]),2.0f*(q_[0]*q_[0]+q_[1]*q_[1]-1.0f))<<
        //             "pitch"<<asinf(-2.0f*(q_[1]*q_[3]-q_[0]*q_[2]))<<
        //             "roll"<<atan2f(2.0f*(q_[0]*q_[1]+q_[2]*q_[3]),2.0f*(q_[0]*q_[0]+q_[3]*q_[3])-1.0f)<<std::endl;
        // std::cout<<"q"<<q_[0]<<" "<<q_[1]<<" "<<q_[2]<<" "<<q_[3]<<std::endl;

    }

} // namespace rmos_comm
#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_transporter::CanCommNode)

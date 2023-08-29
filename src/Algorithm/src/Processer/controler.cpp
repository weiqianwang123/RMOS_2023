//
// Created by Wang on 23-7-16.
//
#include "Processer/controler.hpp"

namespace processer
{

    Controler::Controler()
    {
        cv::FileStorage fs("./src/Algorithm/configure/Processer/controler/param.xml", cv::FileStorage::READ);

        if(!fs.isOpened())
        {
            std::cout<<"open processer controler param fail"<<std::endl;
            exit(0);
        }
        fs["lost_time_thres"] >> lost_time_thres_;
        fs["s2qxyz"] >> s2qxyz_;
        fs["s2qyaw"] >> s2qyaw_;
        fs["s2qr"] >> s2qr_;
        fs["r_xyz_factor"] >> r_xyz_factor;
        fs["r_yaw"] >> r_yaw;
        fs["delay"] >> delay_;
        fs["true_x"] >> true_x_;
        fs.release();

        // EKF
        // xa = x_armor, xc = x_robot_center
        // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
        // measurement: xa, ya, za, yaw
        // f - Process function
        auto f = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_;
            x_new(2) += x(3) * dt_;
            x_new(4) += x(5) * dt_;
            x_new(6) += x(7) * dt_;
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_f = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(9, 9);
            // clang-format off
            f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,
                    0,   1,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   1,   dt_, 0,   0,   0,   0,   0,
                    0,   0,   0,   1,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   1,   dt_, 0,   0,   0,
                    0,   0,   0,   0,   0,   1,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   1,   dt_, 0,
                    0,   0,   0,   0,   0,   0,   0,   1,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto h = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_h = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 9);
            double yaw = x(6), r = x(8);
            // clang-format off
            //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
            h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),
                    0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),
                    0,   0,   0,   0,   1,   0,   0,          0,   0,
                    0,   0,   0,   0,   0,   0,   1,          0,   0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_q = [this]() {
            Eigen::MatrixXd q(9, 9);
            double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
            q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
                    q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
                    0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
                    0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
                    0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
                    0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
                    0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
                    0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
                    0,      0,      0,      0,      0,      0,      0,      0,      q_r;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_r = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 9> p0;
        p0.setIdentity();
        tracker_.ekf = tool::ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};
    }

    int Controler::getAimingPoint(std::vector<base::Armor> armors,cv::Point3f& aiming_point,double timestamp)
    {
        double time = timestamp;
        bool is_tracking = false;
        if (tracker_.tracker_state == base::LOST)
        {
            is_tracking = false;
            tracker_.init(armors);
        }
        else
        {

            dt_ = time - last_time_;
            if(tracker_.tracked_id == 7 )
            {
                tracker_.lost_thres = static_cast<int>(lost_time_thres_ / dt_)*20;
            }
            else if (tracker_.tracked_id==11||tracker_.tracked_id==12||tracker_.tracked_id==13)
            {
                tracker_.lost_thres = static_cast<int>(lost_time_thres_ / dt_)*2;
            }
            tracker_.update(armors);

            if (tracker_.tracker_state == base::DETECTING) {
                is_tracking = false;
            } else if (
                    tracker_.tracker_state == base::TRACKING ||
                    tracker_.tracker_state == base::TEMP_LOST) {
                is_tracking = true;
            }
        }
        last_time_ = time;

        if(is_tracking)
        {

            //get car state
            const auto & state = tracker_.target_state;
            double yaw = state[6], r1 = state[8], r2 =  tracker_.another_r;
            double xc =  state[0], yc = state[2], za = state[4];
            double vx = state[1], vy = state[3], vz = state[5];
            double dz =   tracker_.dz;double v_yaw =  state[7];


            //predict
            cv::Point3d p_center = cv::Point3d(xc, yc, za + dz/ 2);
            cv::Point3d velocity_linear = cv::Point3d(vx, vy, vz);
            double all_time = ballistic_solver_.getAngleTime(p_center * 1000).z + delay_;

            cv::Point3d linear_change = cv::Point3d(velocity_linear.x * (all_time+0.05),
                                                    velocity_linear.y * (all_time+0.05),
                                                    velocity_linear.z * (all_time+0.05));
            cv::Point3d p_predict_center = p_center + linear_change; //预测中心点
            double yaw_predict = yaw + v_yaw * all_time;  //预测yaw


            // get armors num
            int armors_num = 4;
            if(this->tracker_.tracked_id == 11 || this->tracker_.tracked_id == 12 || this->tracker_.tracked_id == 13 )
            {
                armors_num = 2;
            }
            else if(this->tracker_.tracked_id == 7)
            {
                armors_num = 3;
            }


            cv::Point3d p[armors_num];
            cv::Point2d p_predict_center_2d(p_predict_center.x, p_predict_center.y);
            double angle_to_center[armors_num];

            bool is_current_pair = true;
            double r = 0;
            int min_dis_point_index = 0;
            double min_dis = DBL_MAX;
            //整车模型建立
            for (int i = 0; i < armors_num; i++)
            {
                double tmp_yaw = yaw_predict +i * (2.0 * M_PI / armors_num);
                if(armors_num == 4)
                {
                    r = is_current_pair ? r1 : r2;
                    p[i].z = za + (is_current_pair ? 0 : dz);
                    is_current_pair = !is_current_pair;
                }
                else
                {
                    r = r1;
                    p[i].z = za;
                }
                p[i].x = (p_predict_center.x - r * cos(tmp_yaw));
                p[i].y = (p_predict_center.y - r * sin(tmp_yaw));
                double dis = sqrt(pow(p[i].x, 2) + pow(p[i].y, 2) + pow(p[i].z, 2));
                if(dis < min_dis)
                {
                    if(abs(dis-min_dis)>0.2)
                    {
                        min_dis = dis;
                        min_dis_point_index = i;        
                    }

                    
                }



                cv::Point2d p_2d(p[i].x - p_predict_center_2d.x, p[i].y - p_predict_center_2d.y);
                angle_to_center[i] = (p_2d.x * p_predict_center_2d.x +
                                      p_2d.y * p_predict_center_2d.y) /
                                     (sqrt(p_2d.x * p_2d.x + p_2d.y * p_2d.y) *
                                      sqrt(p_predict_center_2d.x * p_predict_center_2d.x +
                                           p_predict_center_2d.y * p_predict_center_2d.y));  //圆心和原点连线，圆心与装甲板连线，两者夹角的余弦值

            }

            //求解最优点
            double limit_area = 0.6 + abs(v_yaw)*0.08;
            if(limit_area>0.99)
            {
                limit_area = 0.99;
            }
            if(this->tracker_.tracked_id == 7)
            {
                limit_area = 0.993;
            }

            if(angle_to_center[min_dis_point_index] < -limit_area)
            {
                aiming_point = p[min_dis_point_index];
                return 1;
            }
            else
            {
                int next_index_1 = (min_dis_point_index+1)%armors_num;
                int next_index_2 = min_dis_point_index - 1;
                if(min_dis_point_index == 0)
                {
                    next_index_2 = armors_num - 1;
                }
                cv::Point3f next_point_1 = p[next_index_1];
                cv::Point3f next_point_2 = p[next_index_2];
                cv::Point3f next_point = next_point_1;

                if(v_yaw < 0)
                {
                    next_point = next_point_1;
                }
                else
                {
                    next_point = next_point_2;
                }
                aiming_point = next_point;
                return 2;
            }
        }
        else
        {
            aiming_point = cv::Point3f(0,0,0);
            return 3;
        }

    }

    bool Controler::judgeFire(cv::Point3f aiming_point_camera , double v_yaw, double roll,cv::Point2f& rect_points_out_1,cv::Point2f& rect_points_out_2,cv::Point2f& rect_points_out_3,cv::Point2f& aim_point_2d_out)
    {
        cv::Point2f aim_point_2d;
        cv::Mat a;
        a = (cv::Mat_<double>(3, 1) << aiming_point_camera.x,aiming_point_camera.y,aiming_point_camera.z);
        cv::Mat b = camera_matrix_*a;
        cv::Point3d b_new(b);
        b_new = b_new/b_new.z;
        aim_point_2d.x = std::ceil(b_new.x);
        aim_point_2d.y = std::ceil(b_new.y);

 
        
        aim_point_2d_out = aim_point_2d;
    
        double fire_area = abs(10/v_yaw);
        if(fire_area>10)
        {
            fire_area = 40;
        }
        

        cv::Point2f center(this->true_x_,360);
        cv::Size size_of_rect(fire_area*2,710);
        cv::RotatedRect fire_rect(center,size_of_rect,-(float)roll);
        cv::Point2f rect_points[4];
        fire_rect.points(rect_points);
        rect_points_out_1 = rect_points[0];
        rect_points_out_2 = rect_points[1];
        rect_points_out_3 = rect_points[2];
      

        std::vector<cv::Point2f> contour;
        for(int i=0;i<4;i++)
        {
            contour.push_back(rect_points[i]);
        }

        double indicator = cv::pointPolygonTest(contour,aim_point_2d,true);


        if(indicator>0)
        {
            return true;
        }
        else
        {
            return false;
        
        }


    }

    bool Controler::getParam(cv::Mat camera_matrix)
    {
        this->camera_matrix_ = camera_matrix;
        if(camera_matrix.empty())
        {
            std::cout<<"camera matrix while get param empty"<<std::endl;
            return false;
        }
        else
        {
            return true;

        }

    }


}